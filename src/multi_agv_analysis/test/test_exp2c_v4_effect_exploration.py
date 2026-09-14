#!/usr/bin/env python3
import copy
import importlib.util
from pathlib import Path
import tempfile
import subprocess
import unittest
import yaml

SPEC=importlib.util.spec_from_file_location('exploration',Path(__file__).resolve().parents[1]/'scripts/analyze_exp2c_v4_effect_exploration.py')
E=importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(E)


class EffectExplorationTest(unittest.TestCase):
    def test_direct_entry_is_independent_and_stops_an_unusable_run(self):
        runner=Path(__file__).resolve().parents[2]/'multi_agv_bringup/scripts/run_exp2c_v4_effect_exploration_fake.sh'
        subprocess.run(['bash','-n',str(runner)],check=True)
        text=runner.read_text()
        self.assertIn('--check-run "$run_dir"',text)
        self.assertIn('run_one "$candidate" M1b "$candidate"',text)
        self.assertIn('run_one "$candidate" M1 "$candidate"',text)
        self.assertNotIn('for candidate in A B',text)
        self.assertEqual(E.V4.CANDIDATES,{'A':(.024,1.5),'B':(.022,2.)})

    def test_exploration_configuration_does_not_change_control(self):
        config=Path(__file__).resolve().parents[2]/'multi_agv_bringup/config'
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary); E.prepare(root)
            for candidate,pulse in E.CANDIDATES.items():
                for method in ('M1','M1b'):
                    old=yaml.safe_load((config/f'exp2c_v4_{method}_candidate_B.yaml').read_text())
                    new=yaml.safe_load((root/f'configs/{candidate}_{method}.yaml').read_text())
                    old['formal_upper'].pop('configuration_status');new['formal_upper'].pop('configuration_status')
                    self.assertEqual(old['formal_upper'],new['formal_upper'])
                    transient=new['formal_fake_runtime']['transient_yaw_v4']
                    self.assertEqual((transient['amplitude'],transient['duration']),pulse)
                    self.assertTrue(transient['effect_exploration'])
                    self.assertEqual(new['formal_fake_runtime']['evidence_scope'],E.MARKERS)
                    self.assertFalse(new['formal_upper']['hardware_execution_authorized'])

    def test_integrated_risk_and_threshold_duration(self):
        rows=[]
        for i,value in enumerate((.3,.4,.8)):
            row={'relative_wall_time':i*.01,'transient_yaw_causal_wheel_margin':value,
                 'transient_yaw_left_raw':.156,'transient_yaw_right_raw':.12,
                 'agv2_wheel_left_applied':.15,'agv2_wheel_right_applied':.12,
                 'candidate_common_velocity':.1,'common_velocity_reference':.097}
            rows.append(row)
        result=E.extras(rows)
        self.assertAlmostEqual(result['J_risk_0p5'],.003)
        self.assertAlmostEqual(result['J_risk_0p7'],.007)
        self.assertAlmostEqual(result['controller_raw_above_0p155_duration_seconds'],.03)
        self.assertAlmostEqual(result['actual_reference_reduction_mean_mps'],.003)
        self.assertEqual(result['physical_speed_limiter_max_continuous_seconds'],0.)

    def test_success_requires_mechanism_and_resource_benefit_not_formal_pass(self):
        phase={'risk_contraction_peak':.008,'risk_boundary_active_duration_seconds':.3,
               'actual_reference_reduction_peak_mps':.002,'common_velocity_reference_minimum':.098,
               'wheel_margin_below_0p5_duration_seconds':.2,'J_risk_0p5':.01,'J_risk_0p7':.02,
               'controller_raw_above_0p150_duration_seconds':.3,'controller_raw_wheel_peak_mps':.158}
        m1={'candidate':'E0','phases':{'post_trigger':phase,'baseline':{'actual_reference_reduction_peak_mps':0.,'common_velocity_reference_minimum':.1}},
            'exploration_run_usable':True,'task_completion_time_seconds':52.,'historical_baseline_reused':False}
        baseline=copy.deepcopy(m1);baseline['historical_baseline_reused']=True
        baseline['phases']['post_trigger']['controller_raw_wheel_peak_mps']=.161
        self.assertEqual(E.comparison(m1,baseline)['status'],'EFFECT_WORKING_POINT_FOUND')
        m1['exploration_run_usable']=False
        self.assertEqual(E.comparison(m1,baseline)['status'],'CANDIDATE_HARD_STOP')
        m1['exploration_run_usable']=True
        m1['phases']['post_trigger']['risk_boundary_active_duration_seconds']=0.
        self.assertEqual(E.comparison(m1,baseline)['status'],'EFFECT_INSUFFICIENT')


if __name__=='__main__': unittest.main()

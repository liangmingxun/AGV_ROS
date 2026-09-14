#!/usr/bin/env python3
import copy
import json
import importlib.util
from pathlib import Path
import subprocess
import tempfile
import unittest
import yaml

SPEC=importlib.util.spec_from_file_location('v5',Path(__file__).resolve().parents[1]/'scripts/analyze_exp2c_v5_yaw_effectiveness.py')
V5=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(V5)

class YawEffectivenessTest(unittest.TestCase):
    def test_whole_recording_does_not_hide_startup_limiter(self):
        rows=[]
        for t in (1.,1.01,1.02):
            r={'stamp':t,'algorithm_valid':True}
            for i in (1,2,3):
                for s in ('left','right'):
                    r[f'agv{i}_wheel_{s}_pre_limit']=.161 if t==1. else .1
                    r[f'agv{i}_wheel_{s}_speed_limit_active']=t==1.
            rows.append(r)
        diagnostic=[{'yaw_effectiveness_source_stamp':1.02,'yaw_effectiveness_triggered':True,
                     'yaw_effectiveness_trigger_wall_time':2.}]
        result=V5.whole_recorded_execution(rows,diagnostic)
        self.assertEqual(result['limiter_samples'],1)
        self.assertAlmostEqual(result['limiter_duration_seconds'],.01)
        self.assertTrue(result['limiter_all_before_disturbance_trigger'])
        self.assertEqual(result['controller_all_car_raw_peak_mps'],.161)
        self.assertAlmostEqual(result['v5_native_diagnostic_start_delay_from_recording_seconds'],.02)

    def test_missing_core_chain_still_yields_failed_report(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary)
            result=V5.analyze_or_failure(root,next(iter(V5.CANDIDATES)),root/'missing.log')
            self.assertFalse(result['exploration_run_usable'])
            self.assertFalse(result['task_complete'])
            self.assertIn('STATE_CHAIN_INVALID',result['exploration_hard_stop_reasons'][0])

    def test_quality_exposure_records_without_promoting_to_fatal(self):
        rows=[{'relative_wall_time':i*.01} for i in range(4)]
        result=V5.exposure(rows,[.009,.0105,.012,.008],.010)
        self.assertTrue(result['quality_exceeded'])
        self.assertAlmostEqual(result['duration_above_threshold_seconds'],.02)
        self.assertEqual(result['first_exceedance_value'],.0105)
        self.assertAlmostEqual(result['first_exceedance_relative_wall_seconds'],.01)
        with self.assertRaises(ValueError): V5.exposure(rows,[.01,float('nan'),.01,.01],.01)

    def test_scoped_localization_keeps_quality_definition(self):
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);V5.prepare(root)
            config=yaml.safe_load((root/'configs/localization_v5_quality_observation.yaml').read_text())
            self.assertEqual(config['maximum_rigid_fit_residual'],.010)
            self.assertTrue(config[V5.QUALITY_POLICY])
            self.assertEqual(config['platform_transport_type'],'fake')
            self.assertFalse(config['hardware_execution_authorized'])
            self.assertEqual(config['projector']['maximum_projection_distance'],.08)

    def test_hard_stop_report_does_not_invent_a_comparison(self):
        with tempfile.TemporaryDirectory() as temporary:
            run=Path(temporary)/'Y0/M1b_R1';run.mkdir(parents=True)
            V5.write_hard_stopped_candidate(run,{'method_id':'M1b_R1','candidate':next(iter(V5.CANDIDATES)),
                'exploration_hard_stop_reasons':['held fail-zero:'],'phases':{}})
            report=json.loads((run.parent/'effect_comparison.json').read_text())
            self.assertEqual(report['status'],'CANDIDATE_HARD_STOP')
            self.assertIn('M1',report['not_run'])
            self.assertIsNone(report['paired_geometry_or_risk_advantage'])

    def test_configs_keep_all_control_parameters_and_disable_legacy(self):
        config=Path(__file__).resolve().parents[2]/'multi_agv_bringup/config'
        with tempfile.TemporaryDirectory() as temporary:
            root=Path(temporary);V5.prepare(root)
            for c,(gamma,duration) in V5.CANDIDATES.items():
                for method in ('M1','M1b'):
                    old=yaml.safe_load((config/f'exp2c_v4_{method}_candidate_B.yaml').read_text())
                    new=yaml.safe_load((root/f'configs/{c}_{method}.yaml').read_text())
                    old['formal_upper'].pop('configuration_status');new['formal_upper'].pop('configuration_status')
                    self.assertEqual(old['formal_upper'],new['formal_upper'])
                    runtime=new['formal_fake_runtime']
                    self.assertFalse(runtime['transient_yaw_v4']['enabled'])
                    self.assertFalse(runtime['yaw_drive_disturbance_v2']['enabled'])
                    self.assertFalse(runtime['risk_disturbance_v1']['enabled'])
                    self.assertEqual(runtime['yaw_effectiveness_v5']['gamma_min'],gamma)
                    self.assertEqual(runtime['yaw_effectiveness_v5']['duration'],duration)
                    self.assertFalse(new['formal_upper']['hardware_execution_authorized'])

    def test_resource_only_benefit_is_not_a_geometry_working_point(self):
        phase={'risk_contraction_peak':.008,'risk_boundary_active_duration_seconds':.3,
               'actual_reference_reduction_peak_mps':.002,'common_velocity_reference_minimum':.098,
               'wheel_margin_below_0p5_duration_seconds':.2,'J_risk_0p5':.01,'J_risk_0p7':.02,
               'controller_raw_above_0p150_duration_seconds':.3,'controller_raw_wheel_peak_mps':.158,
               'lateral_error_IAE':.03,'heading_error_IAE':.3,'support_error_IAE':.04,'rigid_fit_error_IAE':.02}
        first={'candidate':next(iter(V5.CANDIDATES)),'phases':{'post_trigger':phase,
               'baseline':{'actual_reference_reduction_peak_mps':0.,'common_velocity_reference_minimum':.1}},
               'exploration_run_usable':True,'task_completion_time_seconds':52.,'historical_baseline_reused':False}
        second=copy.deepcopy(first);second['phases']['post_trigger']['controller_raw_wheel_peak_mps']=.161
        self.assertEqual(V5.comparison(first,second)['status'],'EFFECT_INSUFFICIENT')
        second['phases']['post_trigger']['lateral_error_IAE']=.032
        self.assertEqual(V5.comparison(first,second)['status'],'YAW_EFFECTIVENESS_WORKING_POINT_FOUND')

    def test_direct_entry_syntax_and_no_cal_or_risk_injection(self):
        repo=Path(__file__).resolve().parents[2]
        subprocess.run(['bash','-n',str(repo/'multi_agv_bringup/scripts/run_exp2c_v5_yaw_effectiveness_fake.sh')],check=True)
        model=(repo/'multi_agv_control/include/multi_agv_control/yaw_effectiveness_degradation.hpp').read_text()
        self.assertNotIn('1.46',model)
        self.assertNotIn('CapabilityReport',model)
        self.assertNotIn('risk_margin',model)
        self.assertEqual(list(V5.CANDIDATES.values()),[(.12,3.5),(.10,3.5),(.08,3.5)])

if __name__=='__main__':unittest.main()

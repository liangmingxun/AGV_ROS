#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import tempfile
import unittest
import yaml
import subprocess
import copy
S=importlib.util.spec_from_file_location('hold',Path(__file__).resolve().parents[1]/'scripts/analyze_exp2c_v5b_yaw_hold.py')
H=importlib.util.module_from_spec(S);S.loader.exec_module(H)
class HoldAnalysisTest(unittest.TestCase):
    def test_profile(self):
        for t,g in ((-1,1),(0,1),(.3,.6),(.6,.2),(2,.2),(3.1,.2),(3.4,.6),(3.7,1),(10,1)):
            self.assertAlmostEqual(H.hold_gamma(t),g)
    def test_isolated_configs_and_single_gain_change(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);H.prepare(root)
            candidates=list(H.CANDIDATES)
            for method in ('M1','M1b'):
                a=yaml.safe_load((root/f'configs/{candidates[0]}_{method}.yaml').read_text())
                b=yaml.safe_load((root/f'configs/{candidates[1]}_{method}.yaml').read_text())
                self.assertFalse(a['formal_fake_runtime']['yaw_effectiveness_v5']['enabled'])
                self.assertEqual(a['formal_fake_runtime']['yaw_effectiveness_hold_v5b']['hold'],2.5)
                self.assertFalse(a['formal_upper']['hardware_execution_authorized'])
                if method=='M1':
                    self.assertEqual(b['formal_upper']['agents']['risk_gain_upper'],[.12]*3)
                    b['formal_upper']['agents']['risk_gain_upper']=[.1]*3
                self.assertEqual(a,b)
    def test_runner_syntax(self):
        path=Path(__file__).resolve().parents[2]/'multi_agv_bringup/scripts/run_exp2c_v5b_yaw_hold_fake.sh'
        subprocess.run(['bash','-n',str(path)],check=True)
    def test_signed_geometry_is_not_hidden(self):
        rows=[{'relative_wall_time':0.},{'relative_wall_time':.01}]
        metric=H.exposure(rows,[-.02,.01])
        self.assertEqual(metric['signed_minimum'],-.02)
        self.assertEqual(metric['signed_maximum'],.01)
        self.assertEqual(metric['peak'],.02)
    def test_selection_requires_geometry_and_true_mechanism(self):
        phase={'risk_contraction_peak':.02,'risk_boundary_active_duration_seconds':1.,
               'actual_reference_reduction_peak_mps':.01,'common_velocity_reference_minimum':.08,
               'wheel_margin_below_0p5_duration_seconds':.1,'J_risk_0p5':.01,'J_risk_0p7':.01,
               'controller_raw_above_0p150_duration_seconds':.1,'controller_raw_wheel_peak_mps':.15,
               'physical_speed_limiter_max_continuous_seconds':0.}
        for key in ('lateral_error_IAE','heading_error_IAE','support_error_IAE','rigid_fit_error_IAE'):
            phase[key]=.89
        a={'candidate':next(iter(H.CANDIDATES)),'exploration_run_usable':True,
           'historical_baseline_reused':False,'task_completion_time_seconds':52.,
           'phases':{'post_trigger':phase,'pulse':phase,
                     'baseline':{'actual_reference_reduction_peak_mps':0.,'common_velocity_reference_minimum':.1}}}
        b=copy.deepcopy(a)
        for key in ('lateral_error_IAE','heading_error_IAE','support_error_IAE','rigid_fit_error_IAE'):
            b['phases']['post_trigger'][key]=1.
        b['phases']['post_trigger']['J_risk_0p5']=.1
        self.assertEqual(H.comparison(a,b)['status'],'S1_GEOMETRY_WORKING_POINT_FOUND')
        a['phases']['post_trigger']['heading_error_IAE']=1.01
        self.assertEqual(H.comparison(a,b)['status'],'EFFECT_INSUFFICIENT')
if __name__=='__main__':unittest.main()

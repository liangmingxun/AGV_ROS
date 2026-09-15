#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import tempfile
import unittest
import subprocess
import yaml

SPEC=importlib.util.spec_from_file_location('repeat',Path(__file__).resolve().parents[1]/'scripts/analyze_exp2c_v5b_three_method_repeat.py')
R=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(R)

class RepeatTests(unittest.TestCase):
    def test_frozen_m2b_control_and_s1_config(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);R.prepare(root,'M2b')
            frozen=yaml.safe_load((R.CONFIG/'exp2b_M2b.yaml').read_text())
            integrated=yaml.safe_load((root/'configs'/f'{R.S1}_M2b.yaml').read_text())
            self.assertEqual(frozen,integrated)
            runtime=yaml.safe_load((root/'configs/M2b_runtime.yaml').read_text())['formal_fake_runtime']
            original=yaml.safe_load((R.CONFIG/'formal_serial_m2b_circle_0p10_observation_runtime.yaml').read_text())['formal_fake_runtime']
            for key in ('tracker','execution','distributed_initial','risk','capability_reserve','leader','lower'):
                self.assertEqual(runtime[key],original[key],key)
            self.assertEqual(runtime['yaw_effectiveness_hold_v5b']['gamma_hold'],.2)
            self.assertFalse(runtime['serial_execution_authorized'])
            self.assertFalse(list((root/'configs').glob('S1R12*')))
    def test_m1_m1b_parameter_parity_except_mode(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);R.prepare(root,'M1')
            a=yaml.safe_load((root/'configs'/f'{R.S1}_M1.yaml').read_text())
            b=yaml.safe_load((root/'configs'/f'{R.S1}_M1b.yaml').read_text())
            self.assertEqual(a['formal_upper']['agents'],b['formal_upper']['agents'])
            self.assertEqual(a['formal_upper']['agents']['risk_gain_upper'],[.1]*3)
    def test_m2b_branch_is_independent_and_returns_before_upper_step(self):
        path=R.CONFIG.parents[1]/'multi_agv_control/src/formal_fake_algorithm_node.cpp'
        source=path.read_text();start=source.index('    if (m2b_selected_) {\n      M2bInput input;')
        end=source.index('    std::array<UpperAgentInput, 3> upper_input;',start)
        branch=source[start:end]
        self.assertIn('m2b_controller_->step(input)',branch)
        self.assertNotIn('upper_generator_->step',branch)
        self.assertNotIn('robust_margin',branch)
        self.assertIn('applyYawHold(now, tracking, &execution_tracking)',branch)
        self.assertLess(branch.index('wheel_demand_before_limit'),branch.index('applyYawHold'))
        self.assertLess(branch.index('applyYawHold'),branch.index('applySerialExecutionLimitPolicy'))
        self.assertTrue(branch.rstrip().endswith('}'))
        self.assertIn('return;',branch)
        self.assertEqual(source.count('applyYawHold(now, tracking, &execution_tracking)'),2)
    def test_order_rotation_and_complete_method_naming(self):
        self.assertEqual(len(R.ORDERS),5)
        for order in R.ORDERS:self.assertEqual(set(order),set(R.METHODS))
        self.assertEqual(R.METHODS['M2b'],'M2b_COMPLETE')
    def test_ci_and_no_fake_zero_for_missing(self):
        result=R.descriptive([1,2,3,4,5])
        self.assertEqual(result['mean'],3)
        self.assertLess(result['ci95_low'],3);self.assertGreater(result['ci95_high'],3)
        self.assertIsNone(R.descriptive([None])['mean'])
    def test_shell_syntax(self):
        for name in ('run_exp2c_v5b_repeat_one_fake.sh','run_exp2c_v5b_three_method_repeat_fake.sh'):
            subprocess.run(['bash','-n',str(R.CONFIG.parent/'scripts'/name)],check=True)
    def test_complete_five_triad_aggregation_and_na_baseline(self):
        with tempfile.TemporaryDirectory() as d:
            root=Path(d)
            for i,order in enumerate(R.ORDERS,1):
                triad=root/f'Triad{i:02d}';R.save(triad/'order.json',order)
                for method,folder in R.METHODS.items():
                    values={key:1. if method!='M1' else .93 for key in R.CORE if key!='task_time_seconds'}
                    values['physical_speed_limiter_duration_seconds']=0.
                    data={'category':'VALID_COMPLETED','complete':True,'task_time_seconds':52. if method=='M1' else 51.7,
                      'phases':{'primary':values},'M1_method_specific_mechanism':{
                        'risk_contraction_peak':.02,'risk_boundary_active_duration':2.,'actual_reference_reduction_peak':.007}}
                    values['J_risk_0p5']=.1 if method=='M1' else 1.
                    R.save(triad/folder/'run/repeat_metrics.json',data)
            result=R.aggregate(root)
            self.assertEqual(result['triads_completed'],5)
            self.assertEqual(result['ablation_status'],'M1_ABLATION_REPEATABILITY_CONFIRMED')
            self.assertEqual(result['literature_comparison_status'],'M1_VS_M2B_MIXED_RESULT')
            self.assertTrue((root/'analysis_plots/lateral_error_IAE.png').exists())
            zero=next(s for s in result['paired_summaries']['M1b'] if s['metric']=='physical_speed_limiter_duration_seconds' and s['statistic']=='relative_improvement_percent')
            self.assertEqual(zero['n'],0);self.assertIsNone(zero['mean'])

if __name__=='__main__':unittest.main()

# Exp2c-v5b measured effect results

EFFECT_EXPLORATION_ONLY / NOT_FORMAL_PAPER_EVIDENCE

Starting and ending Git HEAD: `7a4788258f1e6fdcee3bda3ccbfe84e76c91889c` (no commit/push).

## Outcome

Both S1 and S1R12 completed fresh M1b+R1 and M1+R1 tasks, without motion fail-zero, emergency, nonfinite demand or structural failure. Both stages remain **EFFECT_INSUFFICIENT** under the requested approximately 10% geometry target. No stronger pulse or gain was run.

M1 risk contraction → active boundary → actual published reference reduction → lower wheel burden / better wheel margin is observed. All four primary geometry IAEs improve, but only 6.3–7.7% in S1 and 7.0–8.6% in S1R12. This does not establish the requested 10% working point, nor formal paper evidence.

## Mathematical and configuration scope

See [independent protocol](exp2c-v5b-hold-effect-exploration.md) for the exact quintic ramp-hold-ramp definition. γhold=0.20; down/hold/up=0.6/2.5/0.6 s; one-shot trigger at Robot2 actual progress 2.0 m. Mean wheel component is preserved; only Robot2 differential execution is attenuated.

S1 uses upper risk gain 0.10. S1R12 changes only M1's three upper risk gains to 0.12. Generated config comparison and unit test verify that all other entries are identical. M1b is freshly rerun for S1R12, not reused. Physical upper, hard envelope, wheel limits/safe margin, nominal upper/inner margin, R1/tracker, calibration, path and geometry remain unchanged. Metadata records 0.10 m/s; every hardware grant remains false and every chassis transport is fake.

Old bell header, old v5 analyzer, archived Y0 report, upper generator implementation and tracker implementation have no diff. The old v5 path is still enabled only by its old block; the new model uses a separate block, diagnostic topic and converter prefix.

## Primary fixed-window geometry comparisons

The primary window is pulse plus 15 s recovery: [0,18.7) s relative to each run's own wall-clock trigger. Improvement=(M1b−M1)/M1b. IAE units are m·s except heading rad·s.

| Metric | S1 M1b | S1 M1 | S1 improvement | S1R12 M1b | S1R12 M1 | S1R12 improvement |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Robot2 lateral IAE | 0.2914484 | 0.2689521 | 7.719% | 0.2914531 | 0.2674246 | 8.244% |
| Robot2 heading IAE | 0.5367137 | 0.5028779 | 6.304% | 0.5380453 | 0.5003976 | 6.997% |
| support IAE | 0.2976164 | 0.275344 | 7.484% | 0.2976076 | 0.2738739 | 7.975% |
| rigid-fit IAE | 0.120865 | 0.1116757 | 7.603% | 0.1210958 | 0.110666 | 8.613% |

## S1: complete measured metrics

Status: EFFECT_INSUFFICIENT. Task complete M1b/M1: true/true; generic validation valid: true/true; hardware grants false, reported wheel limits unchanged: true/true.

Full phase metrics and source context: [JSON](../../experiment_data//exp2c_v5b_yaw_effectiveness_hold_exploration/EFFECT_EXPLORATION_ONLY_NOT_FORMAL_PAPER_EVIDENCE_20260914_S1_complete/S1_gamma0p20_down0p6_hold2p5_up0p6_R10/effect_comparison.json), [metrics CSV](../../experiment_data//exp2c_v5b_yaw_effectiveness_hold_exploration/EFFECT_EXPLORATION_ONLY_NOT_FORMAL_PAPER_EVIDENCE_20260914_S1_complete/S1_gamma0p20_down0p6_hold2p5_up0p6_R10/effect_metrics.csv), [all-robot signed geometry CSV](../../experiment_data//exp2c_v5b_yaw_effectiveness_hold_exploration/EFFECT_EXPLORATION_ONLY_NOT_FORMAL_PAPER_EVIDENCE_20260914_S1_complete/S1_gamma0p20_down0p6_hold2p5_up0p6_R10/quality_exposure.csv).

### Pulse + recovery, all metrics

Wheel/controller burden below refers to Robot2 pre-degradation demand; applied/degraded values are separate. Robust and wheel margins are dimensionless. Contraction/reduction/reference are m/s; contraction integral is m. Raw duration and limiter duration are s. Native error peaks/RMS are m or rad; IAE is m·s or rad·s.

| Metric | M1b | M1 |
| --- | ---: | ---: |
| J_risk_0p5 | 1.138753 | 0.1480033 |
| J_risk_0p7 | 1.672603 | 0.3616681 |
| actual_reference_reduction_mean_mps | -3.435982e-8 | 0.0004362859 |
| actual_reference_reduction_peak_mps | 0.000003378846 | 0.007017797 |
| actual_reference_reduction_positive_fraction | 0.05828877 | 0.2780749 |
| applied_wheel_peak_mps | 0.16 | 0.1474584 |
| common_velocity_reference_mean | 0.09990457 | 0.09852028 |
| common_velocity_reference_minimum | 0.09986355 | 0.08130986 |
| common_velocity_reference_peak | 0.09996682 | 0.1008652 |
| controller_raw_above_0p150_duration_seconds | 2.940362 | 1.94949 |
| controller_raw_above_0p155_duration_seconds | 2.580416 | 0.8901582 |
| controller_raw_above_0p160_duration_seconds | 1.93062 | 0 |
| controller_raw_wheel_peak_mps | 0.1774251 | 0.1588404 |
| controller_raw_wheel_rms_mps | 0.1249507 | 0.122857 |
| disturbed_wheel_peak_mps | 0.1710531 | 0.1474584 |
| heading_error_IAE | 0.5367137 | 0.5028779 |
| heading_error_peak | 0.12861 | 0.1210398 |
| heading_error_rms | 0.04438224 | 0.04228234 |
| lateral_error_IAE | 0.2914484 | 0.2689521 |
| lateral_error_peak | 0.0364002 | 0.0332115 |
| lateral_error_rms | 0.01837758 | 0.01690282 |
| longitudinal_error_IAE | 0.03905811 | 0.03273 |
| longitudinal_error_peak | 0.007293075 | 0.005409093 |
| longitudinal_error_rms | 0.002601919 | 0.002059925 |
| observation_duration_seconds | 18.70113 | 18.70023 |
| pairwise_side_error_IAE | 0.2397099 | 0.2218759 |
| pairwise_side_error_peak | 0.03836287 | 0.03572776 |
| pairwise_side_error_rms | 0.01595386 | 0.01477426 |
| physical_speed_limiter_duration_seconds | 0.5601771 | 0 |
| physical_speed_limiter_max_continuous_seconds | 0.5601771 | 0 |
| physical_speed_limiter_samples | 56 | 0 |
| progress_error_IAE | 0.06092892 | 0.05397515 |
| progress_error_peak | 0.01258966 | 0.01049448 |
| progress_error_rms | 0.004265556 | 0.003618609 |
| rigid_fit_error_IAE | 0.120865 | 0.1116757 |
| rigid_fit_error_peak | 0.01996278 | 0.01852085 |
| rigid_fit_error_rms | 0.008045752 | 0.007427132 |
| risk_boundary_active_duration_seconds | 0 | 3.489434 |
| risk_boundary_active_fraction | 0 | 0.1865985 |
| risk_contraction_integral | 0 | 0.06359067 |
| risk_contraction_mean | 0 | 0.003400529 |
| risk_contraction_minimum | 0 | 2.317706e-8 |
| risk_contraction_peak | 0 | 0.02569014 |
| robust_margin_mean | 0.8641766 | 0.9565752 |
| robust_margin_minimum | 0 | 0.1476098 |
| robust_margin_p05 | 0 | 0.5180831 |
| samples | 1870 | 1870 |
| support_error_IAE | 0.2976164 | 0.275344 |
| support_error_peak | 0.04567211 | 0.04168862 |
| support_error_rms | 0.01919956 | 0.01772776 |
| upper_clipping_reduction_mean_mps | 0 | 0.0004360805 |
| upper_clipping_reduction_peak_mps | 0 | 0.007017797 |
| wheel_demand_capability_ratio_peak | 1.108907 | 0.9927523 |
| wheel_margin_below_0p5_duration_seconds | 2.580786 | 0.8898804 |
| wheel_margin_below_0p5_fraction | 0.1380016 | 0.04758661 |
| wheel_margin_below_0p7_duration_seconds | 2.750117 | 1.260232 |
| wheel_margin_below_0p7_fraction | 0.1470562 | 0.06739125 |
| wheel_margin_below_1p0_duration_seconds | 2.940156 | 1.949746 |
| wheel_margin_below_1p0_fraction | 0.157218 | 0.1042632 |
| wheel_margin_mean | 0.8648548 | 0.9553155 |
| wheel_margin_minimum | 0 | 0.1159639 |
| wheel_margin_p05 | 0 | 0.5293744 |

### Fixed 3.7 s pulse (startup excluded)

| Metric | M1b | M1 |
| --- | ---: | ---: |
| common_velocity_reference_mean | 0.09989192 | 0.09445451 |
| common_velocity_reference_minimum | 0.09987795 | 0.08130986 |
| actual_reference_reduction_peak_mps | 0.000002939403 | 0.007017797 |
| actual_reference_reduction_mean_mps | -2.699438e-8 | 0.001639115 |
| risk_contraction_peak | 0 | 0.02569014 |
| risk_contraction_integral | 0 | 0.03519755 |
| risk_boundary_active_duration_seconds | 0 | 1.959419 |
| risk_boundary_active_fraction | 0 | 0.5295683 |
| controller_raw_wheel_peak_mps | 0.1774251 | 0.1588404 |
| disturbed_wheel_peak_mps | 0.1710531 | 0.1474584 |
| applied_wheel_peak_mps | 0.16 | 0.1474584 |
| physical_speed_limiter_duration_seconds | 0.2699816 | 0 |
| physical_speed_limiter_samples | 27 | 0 |
| physical_speed_limiter_max_continuous_seconds | 0.2699816 | 0 |

M1 pulse-mean reference 0.09445451 m/s is below historical Y0's approximate 0.0958 m/s. Y0 is a different disturbance profile and is context only, not a paired causal comparison.

Actual reference reduction is **own candidate minus own published common reference**. Tiny M1b reductions of a few μm/s are asynchronous alignment noise, not risk action. Minimum reference minus 0.10 m/s is not substituted for this metric.

### Estimator's actual unchanged 10 mm rigid-fit quality threshold

| Metric | M1b | M1 |
| --- | ---: | ---: |
| peak | 0.01996278 | 0.01852085 |
| RMS | 0.008045854 | 0.007427918 |
| IAE | 0.1208641 | 0.1116965 |
| duration_above_threshold_seconds | 4.139873 | 3.500901 |
| first_exceedance_relative_wall_seconds | 1.838495 | 1.844462 |
| first_exceedance_ros_stamp | 1789392000 | 1789392000 |
| first_exceedance_value | 0.01002764 | 0.01010326 |

Both methods exceed the 10 mm quality threshold; this fact is retained, not turned into formal pass. The JSON includes first source timestamp and simultaneous γ, raw wheels, reference, native heading/lateral errors and margin.

### All three robots and all three pairwise sides

Values are absolute peak/RMS/IAE; signed minima/maxima/means are retained in JSON and quality CSV. No unfavorable robot or side is removed.

| Signal | M1b peak / RMS / IAE | M1 peak / RMS / IAE |
| --- | ---: | ---: |
| agv1_heading | 0.009641831 / 0.003065088 / 0.03490564 | 0.009821434 / 0.002739254 / 0.03194719 |
| agv1_lateral | 0.002858194 / 0.00170686 / 0.0303324 | 0.002789081 / 0.001651769 / 0.02943294 |
| agv1_longitudinal | 0.002972727 / 0.001335563 / 0.02116515 | 0.003975324 / 0.001185719 / 0.01699876 |
| agv1_progress | 0.002827629 / 0.001259848 / 0.01984564 | 0.003787078 / 0.001119237 / 0.01595376 |
| agv2_heading | 0.12861 / 0.04438224 / 0.5367137 | 0.1210398 / 0.04228234 / 0.5028779 |
| agv2_lateral | 0.0364002 / 0.01837758 / 0.2914484 | 0.0332115 / 0.01690282 / 0.2689521 |
| agv2_longitudinal | 0.007293075 / 0.002601919 / 0.03905811 | 0.005409093 / 0.002059925 / 0.03273 |
| agv2_progress | 0.01258966 / 0.004265556 / 0.06092892 | 0.01049448 / 0.003618609 / 0.05397515 |
| agv3_heading | 0.01093 / 0.003517613 / 0.0361152 | 0.01106632 / 0.003187334 / 0.03214571 |
| agv3_lateral | 0.003307104 / 0.002566023 / 0.04720904 | 0.003146622 / 0.002461461 / 0.04537851 |
| agv3_longitudinal | 0.001022712 / 0.0003558865 / 0.005585998 | 0.00132951 / 0.0004106093 / 0.006213915 |
| agv3_progress | 0.002111445 / 0.0009980956 / 0.01756557 | 0.002475548 / 0.0009660412 / 0.0167145 |
| d12 | 0.03657974 / 0.01389378 / 0.2077981 | 0.03377102 / 0.01280869 / 0.193105 |
| d13 | 0.000958528 / 0.000480979 / 0.00829434 | 0.001052939 / 0.0005342208 / 0.008480497 |
| d23 | 0.03836287 / 0.01593724 / 0.2385454 | 0.03572776 / 0.01474638 / 0.2200858 |
| pairwise_side | 0.03836287 / 0.01595386 / 0.2397099 | 0.03572776 / 0.01477426 / 0.2218759 |
| progress | 0.01258966 / 0.004265556 / 0.06092892 | 0.01049448 / 0.003618609 / 0.05397515 |
| rigid_fit | 0.01996278 / 0.008045752 / 0.120865 | 0.01852085 / 0.007427132 / 0.1116757 |
| support | 0.04567211 / 0.01919956 / 0.2976164 | 0.04168862 / 0.01772776 / 0.275344 |

### Recovery and task cost

Recovery means entering the baseline-derived envelope for the existing confirmation interval, measured from pulse end. Censored means not recovered within the fixed 15 s observation; not zero and not task failure.

| Signal | M1b recovery / s | M1 recovery / s |
| --- | ---: | ---: |
| heading | 9.53008 | 8.900047 |
| lateral | censored / unavailable | censored / unavailable |
| longitudinal | 4.7803 | 4.110366 |
| rigid_fit | censored / unavailable | censored / unavailable |
| support | censored / unavailable | censored / unavailable |

Task time M1b 51.77999 s; M1 52.13006 s. Increase 0.3500667 s (0.676%).

### Full-recording startup and raw-demand coverage

Unlike the filtered native-diagnostic full-task window, the whole-recording audit includes startup before the first native diagnostic message. Missing fields during invalid arming/terminal transitions are exposed; missing raw while algorithm valid must not be hidden.

| Whole-recording metric | M1b | M1 |
| --- | ---: | ---: |
| agv1_raw_above_0p150_duration_seconds | 0.3200116 | 0.3103023 |
| agv1_raw_above_0p155_duration_seconds | 0.2501364 | 0.2501514 |
| agv1_raw_above_0p160_duration_seconds | 0.1500711 | 0.149941 |
| agv1_raw_peak_mps | 0.1642271 | 0.1650543 |
| agv2_raw_above_0p150_duration_seconds | 3.269995 | 2.280652 |
| agv2_raw_above_0p155_duration_seconds | 2.850229 | 1.149998 |
| agv2_raw_above_0p160_duration_seconds | 2.090086 | 0.1599967 |
| agv2_raw_peak_mps | 0.1774251 | 0.1651137 |
| agv3_raw_above_0p150_duration_seconds | 0.3099792 | 0.3001366 |
| agv3_raw_above_0p155_duration_seconds | 0.2400575 | 0.2300684 |
| agv3_raw_above_0p160_duration_seconds | 0.1299627 | 0.149941 |
| agv3_raw_peak_mps | 0.162959 | 0.1637861 |
| controller_all_car_raw_peak_mps | 0.1774251 | 0.1651137 |
| limiter_all_before_disturbance_trigger | false | true |
| limiter_duration_seconds | 0.7400353 | 0.1698909 |
| limiter_first_ros_stamp | 1789392000 | 1789392000 |
| limiter_samples | 74 | 17 |
| missing_raw_samples | 238 | 183 |
| missing_raw_while_algorithm_valid_samples | 0 | 0 |
| raw_samples | 5194 | 5229 |
| v5_native_diagnostic_start_delay_from_recording_seconds | 0.03197932 | 0.03563285 |

Jrisk0.5 decreases 87.00%. All post-trigger physical limiter exposure and first raw/degraded/applied input-output values are in the JSON. M1 post-trigger limiter is zero, but startup limiter events remain recorded. M1b's short pulse-end limiter extends into early recovery; the combined continuous exposure is 0.5601771 s.

Selection limiter convention: disturbance-window maximum continuous limiter <0.5 s; observational selection only, not a safety gate. It is a transparent exploratory convention, not a user-specified numeric safety threshold and not a control gate.

## S1R12: complete measured metrics

Status: EFFECT_INSUFFICIENT. Task complete M1b/M1: true/true; generic validation valid: true/true; hardware grants false, reported wheel limits unchanged: true/true.

Full phase metrics and source context: [JSON](../../experiment_data//exp2c_v5b_yaw_effectiveness_hold_exploration/EFFECT_EXPLORATION_ONLY_NOT_FORMAL_PAPER_EVIDENCE_20260914_S1R12_complete/S1R12_gamma0p20_down0p6_hold2p5_up0p6_R12/effect_comparison.json), [metrics CSV](../../experiment_data//exp2c_v5b_yaw_effectiveness_hold_exploration/EFFECT_EXPLORATION_ONLY_NOT_FORMAL_PAPER_EVIDENCE_20260914_S1R12_complete/S1R12_gamma0p20_down0p6_hold2p5_up0p6_R12/effect_metrics.csv), [all-robot signed geometry CSV](../../experiment_data//exp2c_v5b_yaw_effectiveness_hold_exploration/EFFECT_EXPLORATION_ONLY_NOT_FORMAL_PAPER_EVIDENCE_20260914_S1R12_complete/S1R12_gamma0p20_down0p6_hold2p5_up0p6_R12/quality_exposure.csv).

### Pulse + recovery, all metrics

Wheel/controller burden below refers to Robot2 pre-degradation demand; applied/degraded values are separate. Robust and wheel margins are dimensionless. Contraction/reduction/reference are m/s; contraction integral is m. Raw duration and limiter duration are s. Native error peaks/RMS are m or rad; IAE is m·s or rad·s.

| Metric | M1b | M1 |
| --- | ---: | ---: |
| J_risk_0p5 | 1.165307 | 0.1237609 |
| J_risk_0p7 | 1.704845 | 0.3096928 |
| actual_reference_reduction_mean_mps | 1.153084e-8 | 0.0004627399 |
| actual_reference_reduction_peak_mps | 0.000003895552 | 0.007772017 |
| actual_reference_reduction_positive_fraction | 0.06866197 | 0.2890495 |
| applied_wheel_peak_mps | 0.16 | 0.1462884 |
| common_velocity_reference_mean | 0.09996897 | 0.09862852 |
| common_velocity_reference_minimum | 0.0999187 | 0.08016573 |
| common_velocity_reference_peak | 0.1000331 | 0.1010328 |
| controller_raw_above_0p150_duration_seconds | 2.970174 | 1.639696 |
| controller_raw_above_0p155_duration_seconds | 2.620384 | 0.7800684 |
| controller_raw_above_0p160_duration_seconds | 1.989952 | 0 |
| controller_raw_wheel_peak_mps | 0.1774135 | 0.1587368 |
| controller_raw_wheel_rms_mps | 0.1250033 | 0.1229739 |
| disturbed_wheel_peak_mps | 0.1714088 | 0.1462884 |
| heading_error_IAE | 0.5380453 | 0.5003976 |
| heading_error_peak | 0.12893 | 0.1205694 |
| heading_error_rms | 0.04448941 | 0.04211244 |
| lateral_error_IAE | 0.2914531 | 0.2674246 |
| lateral_error_peak | 0.03647617 | 0.03301149 |
| lateral_error_rms | 0.01838954 | 0.01680199 |
| longitudinal_error_IAE | 0.03877289 | 0.03215142 |
| longitudinal_error_peak | 0.007512197 | 0.005433406 |
| longitudinal_error_rms | 0.002616268 | 0.00202326 |
| observation_duration_seconds | 18.69995 | 18.69979 |
| pairwise_side_error_IAE | 0.239927 | 0.2199996 |
| pairwise_side_error_peak | 0.0384721 | 0.03550967 |
| pairwise_side_error_rms | 0.01597772 | 0.01468345 |
| physical_speed_limiter_duration_seconds | 0.6005199 | 0 |
| physical_speed_limiter_max_continuous_seconds | 0.6005199 | 0 |
| physical_speed_limiter_samples | 52 | 0 |
| progress_error_IAE | 0.06070281 | 0.05338676 |
| progress_error_peak | 0.01264433 | 0.01036278 |
| progress_error_rms | 0.00427859 | 0.003571714 |
| rigid_fit_error_IAE | 0.1210958 | 0.110666 |
| rigid_fit_error_peak | 0.01999157 | 0.01835679 |
| rigid_fit_error_rms | 0.008063943 | 0.007372372 |
| risk_boundary_active_duration_seconds | 0 | 3.530013 |
| risk_boundary_active_fraction | 0 | 0.1887729 |
| risk_contraction_integral | 0 | 0.06485163 |
| risk_contraction_mean | 0 | 0.003468041 |
| risk_contraction_minimum | 0 | 2.97163e-8 |
| risk_contraction_peak | 0 | 0.02683427 |
| robust_margin_mean | 0.8635475 | 0.9625763 |
| robust_margin_minimum | 0 | 0.137783 |
| robust_margin_p05 | 0 | 0.6404281 |
| samples | 1704 | 1799 |
| support_error_IAE | 0.2976076 | 0.2738739 |
| support_error_peak | 0.045812 | 0.04139506 |
| support_error_rms | 0.01921496 | 0.01763238 |
| upper_clipping_reduction_mean_mps | 0 | 0.0004580671 |
| upper_clipping_reduction_peak_mps | 0 | 0.007772017 |
| wheel_demand_capability_ratio_peak | 1.108834 | 0.9921049 |
| wheel_margin_below_0p5_duration_seconds | 2.610299 | 0.7898941 |
| wheel_margin_below_0p5_fraction | 0.1395886 | 0.0422408 |
| wheel_margin_below_0p7_duration_seconds | 2.769924 | 1.109704 |
| wheel_margin_below_0p7_fraction | 0.1481247 | 0.05934313 |
| wheel_margin_below_1p0_duration_seconds | 2.969946 | 1.639595 |
| wheel_margin_below_1p0_fraction | 0.1588211 | 0.08767984 |
| wheel_margin_mean | 0.862822 | 0.961514 |
| wheel_margin_minimum | 0 | 0.1263211 |
| wheel_margin_p05 | 0 | 0.6236419 |

### Fixed 3.7 s pulse (startup excluded)

| Metric | M1b | M1 |
| --- | ---: | ---: |
| common_velocity_reference_mean | 0.09997541 | 0.0941475 |
| common_velocity_reference_minimum | 0.09994859 | 0.08016573 |
| actual_reference_reduction_peak_mps | 0.000002363944 | 0.007772017 |
| actual_reference_reduction_mean_mps | -1.190282e-7 | 0.001775617 |
| risk_contraction_peak | 0 | 0.02683427 |
| risk_contraction_integral | 0 | 0.03686584 |
| risk_boundary_active_duration_seconds | 0 | 2.000043 |
| risk_boundary_active_fraction | 0 | 0.540595 |
| controller_raw_wheel_peak_mps | 0.1774135 | 0.1587368 |
| disturbed_wheel_peak_mps | 0.1714088 | 0.1462884 |
| applied_wheel_peak_mps | 0.16 | 0.1462884 |
| physical_speed_limiter_duration_seconds | 0.2701083 | 0 |
| physical_speed_limiter_samples | 27 | 0 |
| physical_speed_limiter_max_continuous_seconds | 0.2701083 | 0 |

M1 pulse-mean reference 0.0941475 m/s is below historical Y0's approximate 0.0958 m/s. Y0 is a different disturbance profile and is context only, not a paired causal comparison.

Actual reference reduction is **own candidate minus own published common reference**. Tiny M1b reductions of a few μm/s are asynchronous alignment noise, not risk action. Minimum reference minus 0.10 m/s is not substituted for this metric.

### Estimator's actual unchanged 10 mm rigid-fit quality threshold

| Metric | M1b | M1 |
| --- | ---: | ---: |
| peak | 0.01999157 | 0.01835679 |
| RMS | 0.008064343 | 0.007372419 |
| IAE | 0.1210997 | 0.1106608 |
| duration_above_threshold_seconds | 4.129717 | 3.469527 |
| first_exceedance_relative_wall_seconds | 1.841477 | 1.841029 |
| first_exceedance_ros_stamp | 1789392000 | 1789392000 |
| first_exceedance_value | 0.0100341 | 0.010055 |

Both methods exceed the 10 mm quality threshold; this fact is retained, not turned into formal pass. The JSON includes first source timestamp and simultaneous γ, raw wheels, reference, native heading/lateral errors and margin.

### All three robots and all three pairwise sides

Values are absolute peak/RMS/IAE; signed minima/maxima/means are retained in JSON and quality CSV. No unfavorable robot or side is removed.

| Signal | M1b peak / RMS / IAE | M1 peak / RMS / IAE |
| --- | ---: | ---: |
| agv1_heading | 0.008865898 / 0.00287944 / 0.03064311 | 0.009384515 / 0.002635394 / 0.02888176 |
| agv1_lateral | 0.00291813 / 0.001762946 / 0.03143725 | 0.002886 / 0.00174357 / 0.03124979 |
| agv1_longitudinal | 0.002427414 / 0.001083838 / 0.01783321 | 0.003867338 / 0.001093061 / 0.01650799 |
| agv1_progress | 0.002297079 / 0.001012001 / 0.01656985 | 0.003679352 / 0.001024287 / 0.01532351 |
| agv2_heading | 0.12893 / 0.04448941 / 0.5380453 | 0.1205694 / 0.04211244 / 0.5003976 |
| agv2_lateral | 0.03647617 / 0.01838954 / 0.2914531 | 0.03301149 / 0.01680199 / 0.2674246 |
| agv2_longitudinal | 0.007512197 / 0.002616268 / 0.03877289 | 0.005433406 / 0.00202326 / 0.03215142 |
| agv2_progress | 0.01264433 / 0.00427859 / 0.06070281 | 0.01036278 / 0.003571714 / 0.05338676 |
| agv3_heading | 0.01099321 / 0.003535658 / 0.03607384 | 0.01120042 / 0.003182718 / 0.03217606 |
| agv3_lateral | 0.003296427 / 0.002550466 / 0.04690611 | 0.003144881 / 0.002460483 / 0.04536887 |
| agv3_longitudinal | 0.0009682837 / 0.0003709799 / 0.005864923 | 0.001541575 / 0.0004587252 / 0.006634938 |
| agv3_progress | 0.002122662 / 0.001014574 / 0.01783001 | 0.002713135 / 0.0009927952 / 0.01685073 |
| d12 | 0.03645693 / 0.0140193 / 0.2104006 | 0.03317976 / 0.01271664 / 0.1916962 |
| d13 | 0.0006851602 / 0.0003443412 / 0.005919599 | 0.0006983509 / 0.0003749586 / 0.006575554 |
| d23 | 0.0384721 / 0.01596313 / 0.2389035 | 0.03550967 / 0.01466595 / 0.2188051 |
| pairwise_side | 0.0384721 / 0.01597772 / 0.239927 | 0.03550967 / 0.01468345 / 0.2199996 |
| progress | 0.01264433 / 0.00427859 / 0.06070281 | 0.01036278 / 0.003571714 / 0.05338676 |
| rigid_fit | 0.01999157 / 0.008063943 / 0.1210958 | 0.01835679 / 0.007372372 / 0.110666 |
| support | 0.045812 / 0.01921496 / 0.2976076 | 0.04139506 / 0.01763238 / 0.2738739 |

### Recovery and task cost

Recovery means entering the baseline-derived envelope for the existing confirmation interval, measured from pulse end. Censored means not recovered within the fixed 15 s observation; not zero and not task failure.

| Signal | M1b recovery / s | M1 recovery / s |
| --- | ---: | ---: |
| heading | 9.56026 | 8.899331 |
| lateral | censored / unavailable | censored / unavailable |
| longitudinal | 4.420311 | 3.869705 |
| rigid_fit | censored / unavailable | censored / unavailable |
| support | censored / unavailable | censored / unavailable |

Task time M1b 51.64999 s; M1 52.04007 s. Increase 0.3900778 s (0.755%).

### Full-recording startup and raw-demand coverage

Unlike the filtered native-diagnostic full-task window, the whole-recording audit includes startup before the first native diagnostic message. Missing fields during invalid arming/terminal transitions are exposed; missing raw while algorithm valid must not be hidden.

| Whole-recording metric | M1b | M1 |
| --- | ---: | ---: |
| agv1_raw_above_0p150_duration_seconds | 0.3701472 | 0.3099997 |
| agv1_raw_above_0p155_duration_seconds | 0.2999411 | 0.2600064 |
| agv1_raw_above_0p160_duration_seconds | 0.2000055 | 0.1700041 |
| agv1_raw_peak_mps | 0.1658314 | 0.1653058 |
| agv2_raw_above_0p150_duration_seconds | 3.33005 | 1.940049 |
| agv2_raw_above_0p155_duration_seconds | 2.919971 | 1.0299 |
| agv2_raw_above_0p160_duration_seconds | 2.180015 | 0.1500442 |
| agv2_raw_peak_mps | 0.1774135 | 0.164648 |
| agv3_raw_above_0p150_duration_seconds | 0.3701472 | 0.2999997 |
| agv3_raw_above_0p155_duration_seconds | 0.2999411 | 0.249969 |
| agv3_raw_above_0p160_duration_seconds | 0.2000055 | 0.1500442 |
| agv3_raw_peak_mps | 0.1657861 | 0.1647975 |
| controller_all_car_raw_peak_mps | 0.1774135 | 0.1653058 |
| limiter_all_before_disturbance_trigger | false | true |
| limiter_duration_seconds | 0.8100798 | 0.169975 |
| limiter_first_ros_stamp | 1789392000 | 1789392000 |
| limiter_samples | 81 | 17 |
| missing_raw_samples | 261 | 222 |
| missing_raw_while_algorithm_valid_samples | 0 | 0 |
| raw_samples | 5185 | 5221 |
| v5_native_diagnostic_start_delay_from_recording_seconds | 0.008856058 | 0.1287792 |

Jrisk0.5 decreases 89.38%. All post-trigger physical limiter exposure and first raw/degraded/applied input-output values are in the JSON. M1 post-trigger limiter is zero, but startup limiter events remain recorded. M1b's short pulse-end limiter extends into early recovery; the combined continuous exposure is 0.6005199 s.

Selection limiter convention: disturbance-window maximum continuous limiter <0.5 s; observational selection only, not a safety gate. It is a transparent exploratory convention, not a user-specified numeric safety threshold and not a control gate.

## S1 → S1R12 additional benefit and recommendation

Each gain stage has its own fresh M1b baseline; a single pair cannot establish significance of small differences.

| M1-only metric | Additional S1R12 change vs S1 |
| --- | ---: |
| lateral_error_IAE | 0.568% reduction |
| heading_error_IAE | 0.493% reduction |
| support_error_IAE | 0.534% reduction |
| rigid_fit_error_IAE | 0.904% reduction |
| J_risk_0p5 | 16.380% reduction |
| J_risk_0p7 | 14.371% reduction |
| controller_raw_wheel_peak_mps | 0.065% reduction |

S1R12's M1-only geometry gains over S1 are approximately 0.5–0.9%, much smaller than the upper-gain increase. The result supports resource protection with modest geometry benefit, not the requested ≥10% enhancement.

Recommendation: retain both as exploratory evidence; do not freeze S1R12 as a formal working point from one pair. Prefer S1 / gain 0.10 as the conservative repeat-validation candidate because 0.12 has not shown a substantial extra geometry effect. If further validation is desired, run fresh repeated S1 pairs with this frozen profile before choosing a formal protocol. No additional parameter search is performed here.

## Changed files

New:

- `docs/test-protocols/exp2c-v5b-hold-effect-exploration.md`
- `docs/test-protocols/exp2c-v5b-hold-effect-results.md`
- `src/multi_agv_control/include/multi_agv_control/yaw_effectiveness_hold_degradation.hpp`
- `src/multi_agv_control/test/test_yaw_effectiveness_hold.cpp`
- `src/multi_agv_analysis/scripts/analyze_exp2c_v5b_yaw_hold.py`
- `src/multi_agv_analysis/test/test_exp2c_v5b_yaw_hold.py`
- `src/multi_agv_bringup/scripts/run_exp2c_v5b_yaw_hold_fake.sh`

Modified:

- Three affected package CMakeLists: register model test, analyzer/analysis test and fake runner.
- `v5_exploration_policy.hpp`: whitelist the independent v5b fake identity; old identity and hardware/serial rejection unchanged.
- `formal_fake_algorithm_node.cpp`: independent configuration/scope, execution-copy injection and native diagnostic publication.
- `path_state_estimator_node.cpp`: error-message wording only, also forcing the missed header-dependent binary rebuild.
- `conversion.py`, `test_metrics.py`: independent v5b topic/raw CSV/aligned columns; old bell fields not replaced.
- `test_rigid_fit_runtime_gate.cpp`: v5b scope admission and serial/hardware rejection assertions.

## Verification and historical failure

- Affected C++ nodes (formal fake, formal algorithm, path estimator) built successfully; analysis/bringup CMake registration configured successfully.
- New hold model: **15/15 groups passed**, covering trigger, ramp/hold/end, continuity, mean/differential, unaffected robots, wall-clock one-shot, nonfinite input and old bell independence. Runtime report verifies unchanged capability/wheel limits throughout all four fresh runs.
- Upper reference generator: **14/14 passed**.
- Quality-policy/runtime-gate tests: **4/4 passed**, including added exact v5b fake admission and serial/hardware rejection.
- Old bell model: **5/5 passed**; old v5 analysis: **8/8 passed**.
- New v5b analysis: **5/5 passed**, including isolated config parity, signed errors, selection semantics and shell syntax.
- Conversion/metrics: **62/62 passed**.
- Bringup static tests: **49/49 passed**.
- Python compile and shell syntax: passed.
- Both fresh stages: generic postprocessing passed for both methods; exploration effect target remains insufficient.
- Known existing `PlanarSupportTracker.FrozenEquilateralFixtureMatchesStartTransformsAndWheelBound` failure is not fixed or masked. The historical tracker suite result was 10/11; it was not rerun as a new failing check in this turn because tracker/geometry are unchanged.
- `git diff --check`: clean. `git status --short`: shows only the listed source/docs modifications and new files; experimental data roots are ignored, retained locally, and not staged.

Initial integration-startup failures remain separate roots and are excluded for lack of a usable pair, not for poor geometry. The selected S1/R12 results are fresh and independent of them. During S1R12's aligned native diagnostics, distinct sample counts are 1704/1799 and maximum spacing is 20.51/20.42 ms (M1b/M1); weighted integration retains the fixed horizon. No gap exceeded 30 ms. Small gain-stage differences require repeated validation.

All 34 requested report items are covered by the configuration/scope, two complete-stage tables, additional-benefit recommendation, tests and worktree sections. No serial execution, commit or push occurred.


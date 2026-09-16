# Candidate B v2 spatial composite fake results

EXPLORATORY_FAKE_VALIDATION
NOT_FORMAL_PAPER_EVIDENCE

Commit: `4f2ee3d0b96c966b0d166a3c3eee6d4fda024c80`

Start dirty state:

```text
M src/multi_agv_analysis/CMakeLists.txt
 M src/multi_agv_analysis/src/multi_agv_analysis/conversion.py
 M src/multi_agv_analysis/test/test_classic_additive_candidate_A.py
 M src/multi_agv_bringup/CMakeLists.txt
 M src/multi_agv_control/CMakeLists.txt
 M src/multi_agv_control/include/multi_agv_control/v5_exploration_policy.hpp
 M src/multi_agv_control/src/formal_fake_algorithm_node.cpp
?? docs/test-protocols/candidate-B-spatial-composite-rho0p85-av0p020-fake-results.md
?? src/multi_agv_analysis/scripts/analyze_candidate_B_spatial_composite_fake.py
?? src/multi_agv_analysis/test/test_candidate_B_spatial_composite_fake.py
?? src/multi_agv_bringup/config/formal_fake_candidate_B_spatial_composite.yaml
?? src/multi_agv_bringup/scripts/run_candidate_B_spatial_composite_fake_comparison.sh
?? src/multi_agv_bringup/scripts/run_candidate_B_spatial_composite_one_fake.sh
?? src/multi_agv_control/include/multi_agv_control/candidate_b_spatial_composite_disturbance.hpp
?? src/multi_agv_control/test/test_candidate_b_spatial_composite_disturbance.cpp
```

Experiment identity: `candidate_B_spatial_composite_fake_validation`

Frozen profile:

```json
{
  "enabled": true,
  "minimum_effectiveness": 0.85,
  "longitudinal_amplitude": 0.02,
  "longitudinal_frequency": 1.0,
  "longitudinal_phase": 0.0,
  "yaw_amplitude": 0.2625,
  "yaw_frequency": 1.0,
  "yaw_phase": 1.5707963267948966,
  "zone_start": 2.0,
  "zone_end": 2.8,
  "ramp_in_distance": 0.05,
  "ramp_out_distance": 0.05,
  "disturbance_model_version": "candidate_B_v2_spatial_composite",
  "freeze_id": "candidate_B_v2_spatial_rho0p85_av0p020_aw0p2625"
}
```

Spatial coordinate: each measured support centre is projected onto the existing load centerline PathProjector. The original s_actual is a support-specific common load parameter and is not used as the world-zone coordinate.

The coordinate therefore represents the measured support centre crossing the same world-space centerline section. No 2.598 s delay and no fixed 8 s termination are used; each robot owns an independent entry wall clock and exits only after its projected progress reaches 2.8 m.

Fresh run paths:

- `/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/experiment_data/candidate_B_spatial_composite_fake/CANDIDATE_B_SPATIAL_COMPOSITE_RHO0P85_AV0P020_20260916_115313/M1b_R1/run`
- `/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/experiment_data/candidate_B_spatial_composite_fake/CANDIDATE_B_SPATIAL_COMPOSITE_RHO0P85_AV0P020_20260916_115313/M1_R1/run`
- `/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/experiment_data/candidate_B_spatial_composite_fake/CANDIDATE_B_SPATIAL_COMPOSITE_RHO0P85_AV0P020_20260916_115313/M2b_M2b/run`

Fake comparison: **VALID**; physical readiness: **NOT_AUTHORIZED**.

| Method | task/s | support RMS/max | rigid RMS/max | side RMS/max | load pos/yaw RMS | native peak | native >.160/s | native >.180/s | post-B peak | post-B >.160/.180 s | limiter/s | post-limit/actual peak | margin min/mean/p05 | margin <.5/<.7 s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| M1_R1 | 52.82 | 0.0156028/0.0385646 | 0.008525/0.0174322 | 0.0179086/0.0369936 | 0.011286/0.0164905 | 0.165926 | 0.789267 | 0 | 0.151561 | 0/0 | 0 | 0.151561/0.151081 | 0/0.951008/0.520168 | 1.7797/2.12974 |
| M1b_R1 | 51.6 | 0.0186971/0.039912 | 0.00881382/0.0173849 | 0.0187816/0.0373824 | 0.0138189/0.0196089 | 0.192023 | 4.57812 | 1.71957 | 0.154446 | 0/0 | 0 | 0.154446/0.154446 | 0/0.832171/0 | 5.38009/5.66061 |
| M2b_M2b | 51.6201 | 0.0190274/0.0402208 | 0.00901825/0.0179185 | 0.0191675/0.0384291 | 0.0141642/0.0196237 | 0.192082 | 4.71035 | 1.76924 | 0.155444 | 0/0 | 0 | 0.155444/0.155444 | 0/0.8297/0 | 5.47008/5.68043 |

## Per-robot RMSE

| Method | Robot | progress / m | lateral / m | heading / rad | velocity / (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| M1_R1 | 1 | 0.013584818 | 0.0046353823 | 0.037488718 | 0.011769987 |
| M1_R1 | 2 | 0.011998025 | 0.0070932865 | 0.1268991 | 0.011458699 |
| M1_R1 | 3 | 0.015750247 | 0.0057018163 | 0.18598503 | 0.015245163 |
| M1b_R1 | 1 | 0.015370871 | 0.0055081615 | 0.037887358 | 0.011175117 |
| M1b_R1 | 2 | 0.014399854 | 0.0087415653 | 0.13226935 | 0.0096336069 |
| M1b_R1 | 3 | 0.018114704 | 0.0066685175 | 0.19067607 | 0.014742363 |
| M2b_M2b | 1 | 0.015824257 | 0.0055924775 | 0.037765667 | 0.011678542 |
| M2b_M2b | 2 | 0.014687518 | 0.0088077161 | 0.13237333 | 0.0099759332 |
| M2b_M2b | 3 | 0.018681097 | 0.0067428627 | 0.19105807 | 0.015129432 |

## M1 vs M1b

| Metric | M1 | Comparator | Improvement |
| --- | ---: | ---: | ---: |
| support_error_rmse | 0.015602835 | 0.018697073 | 16.549% |
| support_error_max | 0.038564605 | 0.039912011 | 3.376% |
| rigid_fit_error_rmse | 0.008524996 | 0.0088138197 | 3.277% |
| rigid_fit_error_max | 0.017432152 | 0.017384924 | -0.272% |
| pairwise_side_error_rmse | 0.017908572 | 0.018781593 | 4.648% |
| pairwise_side_error_max | 0.036993609 | 0.037382406 | 1.040% |
| equivalent_load_position_error_rmse | 0.011286002 | 0.013818908 | 18.329% |
| equivalent_load_position_error_max | 0.025005719 | 0.026195044 | 4.540% |
| equivalent_load_yaw_error_rmse | 0.016490535 | 0.019608862 | 15.903% |
| equivalent_load_yaw_error_max | 0.038852265 | 0.042319857 | 8.194% |
| native_controller_raw_peak_mps | 0.16592564 | 0.19202314 | 13.591% |
| native_controller_raw_rms_mps | 0.10271127 | 0.11539516 | 10.992% |
| native_over_0p160_duration_seconds | 0.78926682 | 4.5781157 | 82.760% |
| native_over_0p180_duration_seconds | 0 | 1.7195728 | 100.000% |
| post_candidate_b_peak_mps | 0.15156073 | 0.15444632 | 1.868% |
| post_candidate_b_over_0p160_duration_seconds | 0 | 0 | N/A |
| post_candidate_b_over_0p180_duration_seconds | 0 | 0 | N/A |
| physical_limiter_duration_seconds | 0 | 0 | N/A |
| post_limit_peak_mps | 0.15156073 | 0.15444632 | 1.868% |
| actual_peak_mps | 0.15108128 | 0.15444632 | 2.179% |
| wheel_margin_below_0p5_duration_seconds | 1.7796988 | 5.3800921 | 66.921% |
| wheel_margin_below_0p7_duration_seconds | 2.1297386 | 5.660615 | 62.376% |
| robot1_progress_error_rmse | 0.013584818 | 0.015370871 | 11.620% |
| robot1_lateral_error_rmse | 0.0046353823 | 0.0055081615 | 15.845% |
| robot1_heading_error_rmse | 0.037488718 | 0.037887358 | 1.052% |
| robot1_velocity_error_rmse | 0.011769987 | 0.011175117 | -5.323% |
| robot2_progress_error_rmse | 0.011998025 | 0.014399854 | 16.680% |
| robot2_lateral_error_rmse | 0.0070932865 | 0.0087415653 | 18.856% |
| robot2_heading_error_rmse | 0.1268991 | 0.13226935 | 4.060% |
| robot2_velocity_error_rmse | 0.011458699 | 0.0096336069 | -18.945% |
| robot3_progress_error_rmse | 0.015750247 | 0.018114704 | 13.053% |
| robot3_lateral_error_rmse | 0.0057018163 | 0.0066685175 | 14.496% |
| robot3_heading_error_rmse | 0.18598503 | 0.19067607 | 2.460% |
| robot3_velocity_error_rmse | 0.015245163 | 0.014742363 | -3.411% |
| wheel_margin_minimum | 0 | 0 | N/A |
| wheel_margin_mean | 0.95100786 | 0.83217102 | 14.280% |
| wheel_margin_p05 | 0.52016789 | 0 | N/A |

## M1 vs M2b

| Metric | M1 | Comparator | Improvement |
| --- | ---: | ---: | ---: |
| support_error_rmse | 0.015602835 | 0.01902745 | 17.998% |
| support_error_max | 0.038564605 | 0.040220798 | 4.118% |
| rigid_fit_error_rmse | 0.008524996 | 0.0090182505 | 5.470% |
| rigid_fit_error_max | 0.017432152 | 0.017918483 | 2.714% |
| pairwise_side_error_rmse | 0.017908572 | 0.019167486 | 6.568% |
| pairwise_side_error_max | 0.036993609 | 0.03842911 | 3.735% |
| equivalent_load_position_error_rmse | 0.011286002 | 0.014164203 | 20.320% |
| equivalent_load_position_error_max | 0.025005719 | 0.026451879 | 5.467% |
| equivalent_load_yaw_error_rmse | 0.016490535 | 0.019623652 | 15.966% |
| equivalent_load_yaw_error_max | 0.038852265 | 0.042278589 | 8.104% |
| native_controller_raw_peak_mps | 0.16592564 | 0.19208178 | 13.617% |
| native_controller_raw_rms_mps | 0.10271127 | 0.11569145 | 11.220% |
| native_over_0p160_duration_seconds | 0.78926682 | 4.7103488 | 83.244% |
| native_over_0p180_duration_seconds | 0 | 1.7692368 | 100.000% |
| post_candidate_b_peak_mps | 0.15156073 | 0.1554443 | 2.498% |
| post_candidate_b_over_0p160_duration_seconds | 0 | 0 | N/A |
| post_candidate_b_over_0p180_duration_seconds | 0 | 0 | N/A |
| physical_limiter_duration_seconds | 0 | 0 | N/A |
| post_limit_peak_mps | 0.15156073 | 0.1554443 | 2.498% |
| actual_peak_mps | 0.15108128 | 0.1554443 | 2.807% |
| wheel_margin_below_0p5_duration_seconds | 1.7796988 | 5.4700811 | 67.465% |
| wheel_margin_below_0p7_duration_seconds | 2.1297386 | 5.6804268 | 62.507% |
| robot1_progress_error_rmse | 0.013584818 | 0.015824257 | 14.152% |
| robot1_lateral_error_rmse | 0.0046353823 | 0.0055924775 | 17.114% |
| robot1_heading_error_rmse | 0.037488718 | 0.037765667 | 0.733% |
| robot1_velocity_error_rmse | 0.011769987 | 0.011678542 | -0.783% |
| robot2_progress_error_rmse | 0.011998025 | 0.014687518 | 18.311% |
| robot2_lateral_error_rmse | 0.0070932865 | 0.0088077161 | 19.465% |
| robot2_heading_error_rmse | 0.1268991 | 0.13237333 | 4.135% |
| robot2_velocity_error_rmse | 0.011458699 | 0.0099759332 | -14.863% |
| robot3_progress_error_rmse | 0.015750247 | 0.018681097 | 15.689% |
| robot3_lateral_error_rmse | 0.0057018163 | 0.0067428627 | 15.439% |
| robot3_heading_error_rmse | 0.18598503 | 0.19105807 | 2.655% |
| robot3_velocity_error_rmse | 0.015245163 | 0.015129432 | -0.765% |
| wheel_margin_minimum | 0 | 0 | N/A |
| wheel_margin_mean | 0.95100786 | 0.82969989 | 14.621% |
| wheel_margin_p05 | 0.52016789 | 0 | N/A |

## Spatial verification

```json
{
  "M1": {
    "Robot1": {
      "d_omega_peak": 0.2624995147689083,
      "d_omega_rms": 0.16895655785770616,
      "d_v_peak": 0.01999998930150595,
      "d_v_rms": 0.014053317403058941,
      "entry_common_progress": 1.8309003682132916,
      "entry_wall_time": 1789530956.2913673,
      "exit_common_progress": 2.632255530940638,
      "exit_wall_time": 1789530965.1613107,
      "exposure_duration_seconds": 8.869943380355835,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9242476059985183,
      "zone_entry_progress": 2.0005806275398084,
      "zone_exit_progress": 2.8004931177892187
    },
    "Robot2": {
      "d_omega_peak": 0.26249960420022594,
      "d_omega_rms": 0.17099006762261204,
      "d_v_peak": 0.019999988461993176,
      "d_v_rms": 0.014219606758954678,
      "entry_common_progress": 2.071828455622685,
      "entry_common_progress_minus_robot1_m": 0.2409280874093933,
      "entry_time_minus_robot1_seconds": 2.410080909729004,
      "entry_wall_time": 1789530958.7014482,
      "exit_common_progress": 2.8744142791307166,
      "exit_wall_time": 1789530967.871354,
      "exposure_duration_seconds": 9.1699059009552,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9429854885535575,
      "zone_entry_progress": 2.0007080503631522,
      "zone_exit_progress": 2.800087756525675
    },
    "Robot3": {
      "d_omega_peak": 0.2624996471979172,
      "d_omega_rms": 0.16983506247344923,
      "d_v_peak": 0.01999999525462588,
      "d_v_rms": 0.01418066620924075,
      "entry_common_progress": 2.1108223413377063,
      "entry_common_progress_minus_robot1_m": 0.2799219731244147,
      "entry_time_minus_robot1_seconds": 2.799913167953491,
      "entry_wall_time": 1789530959.0912805,
      "exit_common_progress": 2.913904279114508,
      "exit_wall_time": 1789530968.2612863,
      "exposure_duration_seconds": 9.170005798339844,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9356443498940361,
      "zone_entry_progress": 2.0005789901588376,
      "zone_exit_progress": 2.80098086223935
    },
    "entry_order": [
      [
        1789530956.2913673,
        "Robot1"
      ],
      [
        1789530958.7014482,
        "Robot2"
      ],
      [
        1789530959.0912805,
        "Robot3"
      ]
    ]
  },
  "M1b": {
    "Robot1": {
      "d_omega_peak": 0.26249972532566446,
      "d_omega_rms": 0.17676482065565208,
      "d_v_peak": 0.019999998665319752,
      "d_v_rms": 0.013702191821145742,
      "entry_common_progress": 1.8313519079787453,
      "entry_wall_time": 1789530869.0853763,
      "exit_common_progress": 2.6299827658626294,
      "exit_wall_time": 1789530877.0858178,
      "exposure_duration_seconds": 8.000441551208496,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9368272479407519,
      "zone_entry_progress": 2.0015153198925604,
      "zone_exit_progress": 2.8002031336204505
    },
    "Robot2": {
      "d_omega_peak": 0.2624997949411991,
      "d_omega_rms": 0.1762924067636439,
      "d_v_peak": 0.019999996809003,
      "d_v_rms": 0.013726420225083571,
      "entry_common_progress": 2.0709312644457305,
      "entry_common_progress_minus_robot1_m": 0.23957935646698525,
      "entry_time_minus_robot1_seconds": 2.4000022411346436,
      "entry_wall_time": 1789530871.4853785,
      "exit_common_progress": 2.86958995110219,
      "exit_wall_time": 1789530879.4856377,
      "exposure_duration_seconds": 8.000259160995483,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.936184868541773,
      "zone_entry_progress": 2.00013415952413,
      "zone_exit_progress": 2.8005419303088286
    },
    "Robot3": {
      "d_omega_peak": 0.2624995900561033,
      "d_omega_rms": 0.1767961840557403,
      "d_v_peak": 0.01999999220838858,
      "d_v_rms": 0.01361461598845868,
      "entry_common_progress": 2.1108577335370766,
      "entry_common_progress_minus_robot1_m": 0.27950582555833137,
      "entry_time_minus_robot1_seconds": 2.8003907203674316,
      "entry_wall_time": 1789530871.885767,
      "exit_common_progress": 2.9055398876687817,
      "exit_wall_time": 1789530879.845748,
      "exposure_duration_seconds": 7.9599809646606445,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9313970119654483,
      "zone_entry_progress": 2.001250109524545,
      "zone_exit_progress": 2.801539569147655
    },
    "entry_order": [
      [
        1789530869.0853763,
        "Robot1"
      ],
      [
        1789530871.4853785,
        "Robot2"
      ],
      [
        1789530871.885767,
        "Robot3"
      ]
    ]
  },
  "M2b": {
    "Robot1": {
      "d_omega_peak": 0.2624998036856292,
      "d_omega_rms": 0.17659491699552565,
      "d_v_peak": 0.019999997290347248,
      "d_v_rms": 0.013693615632328144,
      "entry_common_progress": 1.8305048212294543,
      "entry_wall_time": 1789531045.73261,
      "exit_common_progress": 2.6305123682186418,
      "exit_wall_time": 1789531053.7327034,
      "exposure_duration_seconds": 8.000093460083008,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9352580295622837,
      "zone_entry_progress": 2.0001287725439765,
      "zone_exit_progress": 2.8003844403585396
    },
    "Robot2": {
      "d_omega_peak": 0.26249968965163156,
      "d_omega_rms": 0.1766720998949411,
      "d_v_peak": 0.0199999923841658,
      "d_v_rms": 0.01370888693428414,
      "entry_common_progress": 2.071502816277165,
      "entry_common_progress_minus_robot1_m": 0.2409979950477108,
      "entry_time_minus_robot1_seconds": 2.410029411315918,
      "entry_wall_time": 1789531048.1426394,
      "exit_common_progress": 2.8695060865212887,
      "exit_wall_time": 1789531056.122675,
      "exposure_duration_seconds": 7.9800355434417725,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9368682235040166,
      "zone_entry_progress": 2.000458138872714,
      "zone_exit_progress": 2.800258197460349
    },
    "Robot3": {
      "d_omega_peak": 0.26249962054750803,
      "d_omega_rms": 0.17690328282261097,
      "d_v_peak": 0.01999999217467983,
      "d_v_rms": 0.013613318767883,
      "entry_common_progress": 2.110499653964027,
      "entry_common_progress_minus_robot1_m": 0.2799948327345725,
      "entry_time_minus_robot1_seconds": 2.80010986328125,
      "entry_wall_time": 1789531048.5327199,
      "exit_common_progress": 2.9045102453550222,
      "exit_wall_time": 1789531056.4730294,
      "exposure_duration_seconds": 7.940309524536133,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9317076131472773,
      "zone_entry_progress": 2.0004452418505805,
      "zone_exit_progress": 2.801182450644057
    },
    "entry_order": [
      [
        1789531045.73261,
        "Robot1"
      ],
      [
        1789531048.1426394,
        "Robot2"
      ],
      [
        1789531048.5327199,
        "Robot3"
      ]
    ]
  }
}
```

## M1 mechanism

```json
{
  "actual_reference_reduction_mean": 0.002741739477951218,
  "actual_reference_reduction_peak": 0.012878541861923581,
  "common_reference_mean": 0.09047517927800393,
  "common_reference_minimum": 0.06722898791336546,
  "risk_boundary_active_duration": 8.179887056350708,
  "risk_contraction_integral": 0.17414808731952053,
  "risk_contraction_peak": 0.03977101208663454,
  "risk_factor_mean": 0.12016276106309084,
  "risk_factor_peak": 1.0,
  "risk_signal_mean": 0.11415462300993631,
  "risk_signal_peak": 0.95,
  "robust_margin_mean": 0.852573806316246,
  "robust_margin_minimum": 0.0
}
```

The frozen disturbance drove robust margin to zero at its worst sample, risk factor to 1.0, and risk contraction to its recorded peak. The risk boundary was active for the recorded duration and produced the measured candidate-minus-effective reference reduction. This timing-consistent chain supports, but does not by itself prove, strict causality.

M1 task-time cost was 2.364% versus M1b and 2.325% versus M2b.

## Validity and physical-readiness notes

- All three runs completed with zero algorithm-invalid and localization-invalid fractions and identical fake-only disturbance profiles.
- M1b and M2b native demands exceeded 0.180 m/s briefly; this does not invalidate the fake scientific comparison, but requires a separate physical-safe qualification.
- All post-Candidate-B demands remained below 0.160 m/s in these fake runs and no fake physical limiter duration was recorded.
- Minimum normalized wheel margin reached zero for all methods, so that minimum alone cannot express improvement; mean/p05 and threshold durations are retained.
- M1 rigid-fit maximum was slightly worse than M1b in this run even though rigid-fit RMSE improved; this unfavorable point is retained.
- CapabilityReport was not modified. Candidate A and Candidate B v1 remained disabled.
- No serial or hardware execution was started. Physical readiness remains NOT_AUTHORIZED; no transition to physical qualification is recommended without a separate safety review.

## Regression verification for this implementation revision

- Candidate B v2 C++: PASS, 5 tests; Python: PASS, 3 tests.
- Candidate B v1 C++: PASS, 4 tests; Python: PASS, 3 tests.
- Candidate A C++: PASS, 9 tests; Python: PASS, 4 tests.
- Analysis metrics: PASS, 64 tests; bringup static: PASS, 49 tests.
- Involved C++ build, shell syntax, Python compile, and `git diff --check`: PASS.

Python regression commands used `PYTHONNOUSERSITE=1` so ROS Noetic loaded the compatible system NumPy 1.17.4 and SciPy 1.3.3. The host user-site NumPy is incompatible with system SciPy; this is an analysis-environment issue, not a Candidate B regression.

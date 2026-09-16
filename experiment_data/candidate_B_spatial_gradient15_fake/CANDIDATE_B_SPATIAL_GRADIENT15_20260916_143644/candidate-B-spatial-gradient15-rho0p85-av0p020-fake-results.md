# Candidate B v2 spatial composite fake results

EXPLORATORY_FAKE_VALIDATION
NOT_FORMAL_PAPER_EVIDENCE

Commit: `1c4661d3752f0582a7096f59892bf87183911db9`

Start dirty state:

```text
M src/multi_agv_analysis/CMakeLists.txt
 M src/multi_agv_analysis/scripts/analyze_candidate_B_spatial_composite_fake.py
 M src/multi_agv_analysis/test/test_candidate_B_spatial_composite_fake.py
 M src/multi_agv_bringup/CMakeLists.txt
 M src/multi_agv_bringup/scripts/run_candidate_B_spatial_composite_one_fake.sh
 M src/multi_agv_control/include/multi_agv_control/candidate_b_spatial_composite_disturbance.hpp
 M src/multi_agv_control/include/multi_agv_control/v5_exploration_policy.hpp
 M src/multi_agv_control/src/formal_fake_algorithm_node.cpp
 M src/multi_agv_control/src/path_state_estimator_node.cpp
 M src/multi_agv_control/test/test_candidate_b_spatial_composite_disturbance.cpp
?? src/multi_agv_analysis/scripts/analyze_candidate_B_spatial_gradient15_fake.py
?? src/multi_agv_analysis/test/test_candidate_B_spatial_gradient15_fake.py
?? src/multi_agv_bringup/config/formal_fake_candidate_B_spatial_gradient15.yaml
?? src/multi_agv_bringup/scripts/run_candidate_B_spatial_gradient15_fake_comparison.sh
```

Experiment identity: `candidate_B_spatial_gradient15_fake_validation`

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
  "severity_scale": [
    1.0,
    1.15,
    0.85
  ],
  "disturbance_model_version": "candidate_B_v2_spatial_composite",
  "freeze_id": "candidate_B_v2_spatial_gradient15_rho0p85_av0p020_aw0p2625"
}
```

Spatial coordinate: each measured support centre is projected onto the existing load centerline PathProjector. The original s_actual is a support-specific common load parameter and is not used as the world-zone coordinate.

Candidate B v2 rejects a spatial projection unless it is valid, has finite progress and distance, and remains within 0.30 m of the load centerline. The limit is deliberately wider than the nominal 0.15 m support-centre lateral offset of the 30 cm formation, while still rejecting grossly inconsistent poses.

The coordinate therefore represents the measured support centre crossing the same world-space centerline section. No 2.598 s delay and no fixed 8 s termination are used; each robot owns an independent entry wall clock and exits only after its projected progress reaches 2.8 m.

Fresh run paths:

- `/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/experiment_data/candidate_B_spatial_gradient15_fake/CANDIDATE_B_SPATIAL_GRADIENT15_20260916_143644/M1b_R1/run`
- `/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/experiment_data/candidate_B_spatial_gradient15_fake/CANDIDATE_B_SPATIAL_GRADIENT15_20260916_143644/M1_R1/run`
- `/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/experiment_data/candidate_B_spatial_gradient15_fake/CANDIDATE_B_SPATIAL_GRADIENT15_20260916_143644/M2b_M2b/run`

Fake comparison: **VALID**; physical readiness: **NOT_AUTHORIZED**.

| Method | task/s | support RMS/max | rigid RMS/max | side RMS/max | load pos/yaw RMS | native peak | native >.160/s | native >.180/s | post-B peak | post-B >.160/.180 s | limiter/s | post-limit/actual peak | margin min/mean/p05 | margin <.5/<.7 s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| M1_R1 | 53.12 | 0.0176668/0.0443255 | 0.00849724/0.0171715 | 0.0166231/0.0352776 | 0.0111981/0.0215713 | 0.170241 | 1.03035 | 0 | 0.15847 | 0/0 | 0 | 0.15847/0.15847 | 0/0.943123/0.359203 | 2.13032/2.47045 |
| M1b_R1 | 51.74 | 0.0216862/0.0458141 | 0.00883846/0.0172067 | 0.0174984/0.0361371 | 0.0142456/0.0262182 | 0.202173 | 5.07034 | 2.17002 | 0.15694 | 0/0 | 0 | 0.15694/0.15694 | 0/0.816252/0 | 5.69003/5.82985 |
| M2b_M2b | 51.62 | 0.0221117/0.0467867 | 0.00902651/0.0177439 | 0.0178218/0.0371081 | 0.0145276/0.0265765 | 0.20295 | 5.13905 | 2.19959 | 0.158157 | 0/0 | 0 | 0.158157/0.158157 | 0/0.815746/0 | 5.69012/5.85042 |

## Per-robot RMSE

| Method | Robot | progress / m | lateral / m | heading / rad | velocity / (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| M1_R1 | 1 | 0.013486567 | 0.0044692716 | 0.037902335 | 0.012215451 |
| M1_R1 | 2 | 0.013524144 | 0.0080204808 | 0.12768807 | 0.013639687 |
| M1_R1 | 3 | 0.013007244 | 0.0049450761 | 0.1834996 | 0.013366565 |
| M1b_R1 | 1 | 0.01550217 | 0.0055390843 | 0.037818207 | 0.011227761 |
| M1b_R1 | 2 | 0.016730864 | 0.0099308218 | 0.13531276 | 0.01122035 |
| M1b_R1 | 3 | 0.015622551 | 0.0058559526 | 0.18900643 | 0.012767922 |
| M2b_M2b | 1 | 0.015843942 | 0.0056133276 | 0.037804482 | 0.011690145 |
| M2b_M2b | 2 | 0.017104325 | 0.010012485 | 0.13542594 | 0.011532808 |
| M2b_M2b | 3 | 0.015959677 | 0.0059074871 | 0.18910912 | 0.013033454 |

## M1 vs M1b

| Metric | M1 | Comparator | Improvement |
| --- | ---: | ---: | ---: |
| support_error_rmse | 0.017666754 | 0.02168616 | 18.534% |
| support_error_max | 0.044325463 | 0.045814133 | 3.249% |
| rigid_fit_error_rmse | 0.0084972413 | 0.008838459 | 3.861% |
| rigid_fit_error_max | 0.017171533 | 0.017206655 | 0.204% |
| pairwise_side_error_rmse | 0.016623055 | 0.01749842 | 5.003% |
| pairwise_side_error_max | 0.035277596 | 0.036137138 | 2.379% |
| equivalent_load_position_error_rmse | 0.011198106 | 0.014245625 | 21.393% |
| equivalent_load_position_error_max | 0.025595341 | 0.026634628 | 3.902% |
| equivalent_load_yaw_error_rmse | 0.021571325 | 0.026218171 | 17.724% |
| equivalent_load_yaw_error_max | 0.052941616 | 0.056465677 | 6.241% |
| native_controller_raw_peak_mps | 0.17024075 | 0.20217348 | 15.795% |
| native_controller_raw_rms_mps | 0.10113929 | 0.11611015 | 12.894% |
| native_over_0p160_duration_seconds | 1.0303512 | 5.0703373 | 79.679% |
| native_over_0p180_duration_seconds | 0 | 2.170017 | 100.000% |
| post_candidate_b_peak_mps | 0.15847027 | 0.15693999 | -0.975% |
| post_candidate_b_over_0p160_duration_seconds | 0 | 0 | N/A |
| post_candidate_b_over_0p180_duration_seconds | 0 | 0 | N/A |
| physical_limiter_duration_seconds | 0 | 0 | N/A |
| post_limit_peak_mps | 0.15847027 | 0.15693999 | -0.975% |
| actual_peak_mps | 0.15847027 | 0.15693999 | -0.975% |
| wheel_margin_below_0p5_duration_seconds | 2.1303184 | 5.6900294 | 62.561% |
| wheel_margin_below_0p7_duration_seconds | 2.4704509 | 5.8298502 | 57.624% |
| robot1_progress_error_rmse | 0.013486567 | 0.01550217 | 13.002% |
| robot1_lateral_error_rmse | 0.0044692716 | 0.0055390843 | 19.314% |
| robot1_heading_error_rmse | 0.037902335 | 0.037818207 | -0.222% |
| robot1_velocity_error_rmse | 0.012215451 | 0.011227761 | -8.797% |
| robot2_progress_error_rmse | 0.013524144 | 0.016730864 | 19.166% |
| robot2_lateral_error_rmse | 0.0080204808 | 0.0099308218 | 19.236% |
| robot2_heading_error_rmse | 0.12768807 | 0.13531276 | 5.635% |
| robot2_velocity_error_rmse | 0.013639687 | 0.01122035 | -21.562% |
| robot3_progress_error_rmse | 0.013007244 | 0.015622551 | 16.741% |
| robot3_lateral_error_rmse | 0.0049450761 | 0.0058559526 | 15.555% |
| robot3_heading_error_rmse | 0.1834996 | 0.18900643 | 2.914% |
| robot3_velocity_error_rmse | 0.013366565 | 0.012767922 | -4.689% |
| wheel_margin_minimum | 0 | 0 | N/A |
| wheel_margin_mean | 0.94312348 | 0.81625246 | 15.543% |
| wheel_margin_p05 | 0.35920348 | 0 | N/A |

## M1 vs M2b

| Metric | M1 | Comparator | Improvement |
| --- | ---: | ---: | ---: |
| support_error_rmse | 0.017666754 | 0.022111718 | 20.102% |
| support_error_max | 0.044325463 | 0.046786716 | 5.261% |
| rigid_fit_error_rmse | 0.0084972413 | 0.0090265102 | 5.863% |
| rigid_fit_error_max | 0.017171533 | 0.01774394 | 3.226% |
| pairwise_side_error_rmse | 0.016623055 | 0.017821849 | 6.727% |
| pairwise_side_error_max | 0.035277596 | 0.037108058 | 4.933% |
| equivalent_load_position_error_rmse | 0.011198106 | 0.014527611 | 22.918% |
| equivalent_load_position_error_max | 0.025595341 | 0.027307898 | 6.271% |
| equivalent_load_yaw_error_rmse | 0.021571325 | 0.026576544 | 18.833% |
| equivalent_load_yaw_error_max | 0.052941616 | 0.05752226 | 7.963% |
| native_controller_raw_peak_mps | 0.17024075 | 0.20295008 | 16.117% |
| native_controller_raw_rms_mps | 0.10113929 | 0.11627819 | 13.020% |
| native_over_0p160_duration_seconds | 1.0303512 | 5.1390495 | 79.951% |
| native_over_0p180_duration_seconds | 0 | 2.1995885 | 100.000% |
| post_candidate_b_peak_mps | 0.15847027 | 0.15815737 | -0.198% |
| post_candidate_b_over_0p160_duration_seconds | 0 | 0 | N/A |
| post_candidate_b_over_0p180_duration_seconds | 0 | 0 | N/A |
| physical_limiter_duration_seconds | 0 | 0 | N/A |
| post_limit_peak_mps | 0.15847027 | 0.15815737 | -0.198% |
| actual_peak_mps | 0.15847027 | 0.15815737 | -0.198% |
| wheel_margin_below_0p5_duration_seconds | 2.1303184 | 5.6901178 | 62.561% |
| wheel_margin_below_0p7_duration_seconds | 2.4704509 | 5.8504162 | 57.773% |
| robot1_progress_error_rmse | 0.013486567 | 0.015843942 | 14.879% |
| robot1_lateral_error_rmse | 0.0044692716 | 0.0056133276 | 20.381% |
| robot1_heading_error_rmse | 0.037902335 | 0.037804482 | -0.259% |
| robot1_velocity_error_rmse | 0.012215451 | 0.011690145 | -4.494% |
| robot2_progress_error_rmse | 0.013524144 | 0.017104325 | 20.931% |
| robot2_lateral_error_rmse | 0.0080204808 | 0.010012485 | 19.895% |
| robot2_heading_error_rmse | 0.12768807 | 0.13542594 | 5.714% |
| robot2_velocity_error_rmse | 0.013639687 | 0.011532808 | -18.269% |
| robot3_progress_error_rmse | 0.013007244 | 0.015959677 | 18.499% |
| robot3_lateral_error_rmse | 0.0049450761 | 0.0059074871 | 16.291% |
| robot3_heading_error_rmse | 0.1834996 | 0.18910912 | 2.966% |
| robot3_velocity_error_rmse | 0.013366565 | 0.013033454 | -2.556% |
| wheel_margin_minimum | 0 | 0 | N/A |
| wheel_margin_mean | 0.94312348 | 0.81574618 | 15.615% |
| wheel_margin_p05 | 0.35920348 | 0 | N/A |

## Spatial verification

```json
{
  "M1": {
    "Robot1": {
      "d_omega_peak": 0.26249971995797916,
      "d_omega_rms": 0.16844893787004822,
      "d_v_peak": 0.0199999895174176,
      "d_v_rms": 0.014077036236633911,
      "entry_common_progress": 1.8313100437579002,
      "entry_wall_time": 1789540711.8975792,
      "exit_common_progress": 2.6316748724698376,
      "exit_wall_time": 1789540720.977845,
      "exposure_duration_seconds": 9.080265760421753,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.1722341057501601,
      "projection_distance_mean": 0.021327467399286423,
      "q_max": 1.0,
      "q_mean": 0.9240396246839319,
      "zone_entry_progress": 2.0009619464063517,
      "zone_exit_progress": 2.8003505669263733
    },
    "Robot2": {
      "d_omega_peak": 0.3018746788971563,
      "d_omega_rms": 0.19915754806929792,
      "d_v_peak": 0.02299999482613311,
      "d_v_rms": 0.016214692627485754,
      "entry_common_progress": 2.0720715139019052,
      "entry_common_progress_minus_robot1_m": 0.24076147014400506,
      "entry_time_minus_robot1_seconds": 2.4096500873565674,
      "entry_wall_time": 1789540714.3072293,
      "exit_common_progress": 2.8776473143013295,
      "exit_wall_time": 1789540723.7371156,
      "exposure_duration_seconds": 9.42988634109497,
      "minimum_rho": 0.8275,
      "projection_distance_available": true,
      "projection_distance_max": 0.1714406294679022,
      "projection_distance_mean": 0.15388212216834143,
      "q_max": 1.0,
      "q_mean": 0.9452642536662983,
      "zone_entry_progress": 2.0009579632876857,
      "zone_exit_progress": 2.8007064758948346
    },
    "Robot3": {
      "d_omega_peak": 0.22312476683454094,
      "d_omega_rms": 0.14645412060731758,
      "d_v_peak": 0.016999995566073962,
      "d_v_rms": 0.0119886613439461,
      "entry_common_progress": 2.1110334668459956,
      "entry_common_progress_minus_robot1_m": 0.2797234230880954,
      "entry_time_minus_robot1_seconds": 2.7996439933776855,
      "entry_wall_time": 1789540714.6972232,
      "exit_common_progress": 2.913215943037934,
      "exit_wall_time": 1789540724.1071491,
      "exposure_duration_seconds": 9.409925937652588,
      "minimum_rho": 0.8725,
      "projection_distance_available": true,
      "projection_distance_max": 0.17136118274477893,
      "projection_distance_mean": 0.14350605528613247,
      "q_max": 1.0,
      "q_mean": 0.9416406651111472,
      "zone_entry_progress": 2.000725979691551,
      "zone_exit_progress": 2.8004932259950923
    },
    "entry_order": [
      [
        1789540711.8975792,
        "Robot1"
      ],
      [
        1789540714.3072293,
        "Robot2"
      ],
      [
        1789540714.6972232,
        "Robot3"
      ]
    ]
  },
  "M1b": {
    "Robot1": {
      "d_omega_peak": 0.2624997729467739,
      "d_omega_rms": 0.17667339754503572,
      "d_v_peak": 0.019999992413267033,
      "d_v_rms": 0.013692476185902084,
      "entry_common_progress": 1.8314155182475622,
      "entry_wall_time": 1789540625.641933,
      "exit_common_progress": 2.631235369982974,
      "exit_wall_time": 1789540633.6517172,
      "exposure_duration_seconds": 8.00978422164917,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.1730249781288269,
      "projection_distance_mean": 0.021255303371886143,
      "q_max": 1.0,
      "q_mean": 0.9356452280123043,
      "zone_entry_progress": 2.0010702826170585,
      "zone_exit_progress": 2.800650448267799
    },
    "Robot2": {
      "d_omega_peak": 0.3018748362548324,
      "d_omega_rms": 0.20311654646230765,
      "d_v_peak": 0.022999992027303393,
      "d_v_rms": 0.01577400089271609,
      "entry_common_progress": 2.072078527495502,
      "entry_common_progress_minus_robot1_m": 0.2406630092479396,
      "entry_time_minus_robot1_seconds": 2.4099698066711426,
      "entry_wall_time": 1789540628.0519028,
      "exit_common_progress": 2.869852997502483,
      "exit_wall_time": 1789540636.041586,
      "exposure_duration_seconds": 7.989683151245117,
      "minimum_rho": 0.8275,
      "projection_distance_available": true,
      "projection_distance_max": 0.1722508020089283,
      "projection_distance_mean": 0.15443835309104173,
      "q_max": 1.0,
      "q_mean": 0.9371812353836165,
      "zone_entry_progress": 2.0009514413986946,
      "zone_exit_progress": 2.800465535922951
    },
    "Robot3": {
      "d_omega_peak": 0.22312484726358514,
      "d_omega_rms": 0.15042863090756425,
      "d_v_peak": 0.016999999622154087,
      "d_v_rms": 0.011588055632618235,
      "entry_common_progress": 2.111017275917844,
      "entry_common_progress_minus_robot1_m": 0.27960175767028184,
      "entry_time_minus_robot1_seconds": 2.799684524536133,
      "entry_wall_time": 1789540628.4416175,
      "exit_common_progress": 2.9048056365996926,
      "exit_wall_time": 1789540636.3917816,
      "exposure_duration_seconds": 7.950164079666138,
      "minimum_rho": 0.8725,
      "projection_distance_available": true,
      "projection_distance_max": 0.17232963609022056,
      "projection_distance_mean": 0.14353714812696455,
      "q_max": 1.0,
      "q_mean": 0.933533864786334,
      "zone_entry_progress": 2.0009059172386436,
      "zone_exit_progress": 2.800018093264802
    },
    "entry_order": [
      [
        1789540625.641933,
        "Robot1"
      ],
      [
        1789540628.0519028,
        "Robot2"
      ],
      [
        1789540628.4416175,
        "Robot3"
      ]
    ]
  },
  "M2b": {
    "Robot1": {
      "d_omega_peak": 0.26249965332759473,
      "d_omega_rms": 0.17662287395930779,
      "d_v_peak": 0.01999999626213609,
      "d_v_rms": 0.013693769255103861,
      "entry_common_progress": 1.8304950384024379,
      "entry_wall_time": 1789540800.9929135,
      "exit_common_progress": 2.630499841551943,
      "exit_wall_time": 1789540808.9934008,
      "exposure_duration_seconds": 8.000487327575684,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.17200763593469026,
      "projection_distance_mean": 0.02117251638484085,
      "q_max": 1.0,
      "q_mean": 0.9354053384879866,
      "zone_entry_progress": 2.0005957414301796,
      "zone_exit_progress": 2.8007847687023344
    },
    "Robot2": {
      "d_omega_peak": 0.30187444970694616,
      "d_omega_rms": 0.20309353739015104,
      "d_v_peak": 0.022999992552559297,
      "d_v_rms": 0.015767188770250278,
      "entry_common_progress": 2.071503187831756,
      "entry_common_progress_minus_robot1_m": 0.24100814942931792,
      "entry_time_minus_robot1_seconds": 2.4103634357452393,
      "entry_wall_time": 1789540803.403277,
      "exit_common_progress": 2.86950929050372,
      "exit_wall_time": 1789540811.3832352,
      "exposure_duration_seconds": 7.9799582958221436,
      "minimum_rho": 0.8275,
      "projection_distance_available": true,
      "projection_distance_max": 0.17184223209032995,
      "projection_distance_mean": 0.1543765530808311,
      "q_max": 1.0,
      "q_mean": 0.9366351151332166,
      "zone_entry_progress": 2.000246791734546,
      "zone_exit_progress": 2.8003176505637346
    },
    "Robot3": {
      "d_omega_peak": 0.22312472687929885,
      "d_omega_rms": 0.15032190419233946,
      "d_v_peak": 0.01699999246777929,
      "d_v_rms": 0.011574635726064961,
      "entry_common_progress": 2.110499532847376,
      "entry_common_progress_minus_robot1_m": 0.280004494444938,
      "entry_time_minus_robot1_seconds": 2.800116777420044,
      "entry_wall_time": 1789540803.7930303,
      "exit_common_progress": 2.9055001112868575,
      "exit_wall_time": 1789540811.7428412,
      "exposure_duration_seconds": 7.949810981750488,
      "minimum_rho": 0.8725,
      "projection_distance_available": true,
      "projection_distance_max": 0.17174239812639905,
      "projection_distance_mean": 0.14357074399726696,
      "q_max": 1.0,
      "q_mean": 0.9317763438706665,
      "zone_entry_progress": 2.000089865124613,
      "zone_exit_progress": 2.800898241052886
    },
    "entry_order": [
      [
        1789540800.9929135,
        "Robot1"
      ],
      [
        1789540803.403277,
        "Robot2"
      ],
      [
        1789540803.7930303,
        "Robot3"
      ]
    ]
  }
}
```

Projection-distance mean/max above establish Candidate B v2 spatial-coordinate validity independently for every robot. Disturbance-profile validity, fake-comparison validity, and physical readiness are separate decisions.

## M1 mechanism

```json
{
  "actual_reference_reduction_mean": 0.003352589969903304,
  "actual_reference_reduction_peak": 0.016211560429065984,
  "common_reference_mean": 0.08861330073140176,
  "common_reference_minimum": 0.0590678658012219,
  "risk_boundary_active_duration": 8.759583353996277,
  "risk_contraction_integral": 0.20103702786971614,
  "risk_contraction_peak": 0.0479321341987781,
  "risk_factor_mean": 0.14150456824645968,
  "risk_factor_peak": 1.0,
  "risk_signal_mean": 0.1344293398341368,
  "risk_signal_peak": 0.95,
  "robust_margin_mean": 0.8279641472598701,
  "robust_margin_minimum": 0.0
}
```

The frozen disturbance drove robust margin to zero at its worst sample, risk factor to 1.0, and risk contraction to its recorded peak. The risk boundary was active for the recorded duration and produced the measured candidate-minus-effective reference reduction. This timing-consistent chain supports, but does not by itself prove, strict causality.

M1 task-time cost was 2.667% versus M1b and 2.906% versus M2b.

## Validity and physical-readiness notes

- All three runs completed with zero algorithm-invalid and localization-invalid fractions and identical fake-only disturbance profiles.
- M1b/M2b pre-disturbance native controller demand exceeded 0.180 m/s for 2.170 s and 2.200 s, respectively, during the analysis window. This does not invalidate the fake scientific comparison, but requires a separate physical-safe qualification.
- The four stages are reported separately: native controller demand, post-Candidate-B execution demand, post-limit command, and fake-plant actual feedback. Native demand above 0.180 m/s is not described as actual wheel command above 0.180 m/s.
- All post-Candidate-B execution demands remained below 0.160 m/s in these fake runs and no fake physical limiter duration was recorded; post-limit and actual peaks remain separately tabulated.
- Minimum normalized wheel margin reached zero for all methods, so that minimum alone cannot express improvement; mean/p05 and threshold durations are retained.
- M1 rigid-fit maximum was slightly worse than M1b in this run even though rigid-fit RMSE improved; this unfavorable point is retained.
- CapabilityReport was not modified. Candidate A and Candidate B v1 remained disabled.
- No serial or hardware execution was started. Physical readiness remains NOT_AUTHORIZED; no transition to physical qualification is recommended without a separate safety review.

## Regression verification for this implementation revision

- Candidate B v2 C++ and Python tests: PASS.
- Candidate B v1 C++: PASS, 4 tests; Python: PASS, 3 tests.
- Candidate A C++: PASS, 9 tests; Python: PASS, 4 tests.
- Analysis metrics: PASS, 64 tests; bringup static: PASS, 49 tests.
- Involved C++ build, shell syntax, Python compile, and `git diff --check`: PASS.

Python regression commands used `PYTHONNOUSERSITE=1` so ROS Noetic loaded the compatible system NumPy 1.17.4 and SciPy 1.3.3. The host user-site NumPy is incompatible with system SciPy; this is an analysis-environment issue, not a Candidate B regression.

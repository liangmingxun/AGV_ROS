# Candidate B v2 spatial composite fake results

EXPLORATORY_FAKE_VALIDATION
NOT_FORMAL_PAPER_EVIDENCE

Commit: `47857252aadab2ac6a13aa9942d6f1c7abc81971`

Start dirty state:

```text
M src/multi_agv_analysis/scripts/analyze_candidate_B_spatial_composite_fake.py
 M src/multi_agv_analysis/test/test_candidate_B_spatial_gradient15_fake.py
 M src/multi_agv_bringup/scripts/run_candidate_B_spatial_composite_one_fake.sh
 M src/multi_agv_bringup/scripts/run_candidate_B_spatial_gradient15_fake_comparison.sh
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

- `/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/experiment_data/candidate_B_spatial_gradient15_fake/CANDIDATE_B_SPATIAL_GRADIENT15_M1UPPER100HZ_20260916_202414/M1b_R1/run`
- `/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/experiment_data/candidate_B_spatial_gradient15_fake/CANDIDATE_B_SPATIAL_GRADIENT15_M1UPPER100HZ_20260916_202414/M1_R1/run`
- `/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/experiment_data/candidate_B_spatial_gradient15_fake/CANDIDATE_B_SPATIAL_GRADIENT15_M1UPPER100HZ_20260916_202414/M2b_M2b/run`

Fake comparison: **VALID**; physical readiness: **NOT_AUTHORIZED**.

| Method | task/s | support RMS/max | rigid RMS/max | side RMS/max | load pos/yaw RMS | native peak | native >.160/s | native >.180/s | post-B peak | post-B >.160/.180 s | limiter/s | post-limit/actual peak | margin min/mean/p05 | margin <.5/<.7 s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| M1_R1 | 53.1499 | 0.0177559/0.0446975 | 0.00848764/0.0169793 | 0.0166118/0.0349534 | 0.0112651/0.0217374 | 0.170786 | 1.02062 | 0 | 0.15832 | 0/0 | 0 | 0.15832/0.156205 | 0/0.943573/0.401094 | 2.06009/2.49042 |
| M1b_R1 | 51.62 | 0.021844/0.0464546 | 0.00882364/0.0170853 | 0.0174674/0.0359982 | 0.014374/0.0264088 | 0.203238 | 5.16009 | 2.20037 | 0.157715 | 0/0 | 0 | 0.157715/0.156031 | 0/0.813905/0 | 5.70013/5.85033 |
| M2b_M2b | 51.6699 | 0.0221247/0.0471681 | 0.00896977/0.0173387 | 0.0177332/0.0364066 | 0.0145432/0.0266946 | 0.203621 | 5.12878 | 2.19989 | 0.158101 | 0/0 | 0 | 0.158101/0.158101 | 0/0.815451/0 | 5.68899/5.82903 |

## Spatial-domain RMSE (primary formation metric)

Common `load_s_reference` interval: 1.83165079–2.90379923 m; 5001 equal-progress samples. Linear interpolation and trapezoidal spatial integration remove repeated time weighting caused by method-dependent zone residence time.

| Method | Support spatial RMSE / m | Rigid-fit spatial RMSE / m | Pairwise spatial RMSE / m |
| --- | ---: | ---: | ---: |
| M1_R1 | 0.0170795514 | 0.00820439019 | 0.016148452 |
| M1b_R1 | 0.021854357 | 0.0088271434 | 0.0174741992 |
| M2b_M2b | 0.022137786 | 0.00897722339 | 0.017747888 |

| Comparison | Support improvement | Rigid-fit improvement | Pairwise improvement |
| --- | ---: | ---: | ---: |
| M1 vs M1b | 21.848% | 7.055% | 7.587% |
| M1 vs M2b | 22.849% | 8.609% | 9.012% |

## Per-robot RMSE

| Method | Robot | progress / m | lateral / m | heading / rad | velocity / (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| M1_R1 | 1 | 0.013514898 | 0.0044761713 | 0.037835471 | 0.012109078 |
| M1_R1 | 2 | 0.013618517 | 0.0080214117 | 0.1280246 | 0.013110761 |
| M1_R1 | 3 | 0.013068353 | 0.004948307 | 0.18390675 | 0.013068111 |
| M1b_R1 | 1 | 0.015582672 | 0.0056032333 | 0.037830848 | 0.011210583 |
| M1b_R1 | 2 | 0.016862561 | 0.010003799 | 0.13556911 | 0.011189453 |
| M1b_R1 | 3 | 0.015683342 | 0.00589301 | 0.18909528 | 0.012713838 |
| M2b_M2b | 1 | 0.015781006 | 0.0056173013 | 0.037759483 | 0.011513031 |
| M2b_M2b | 2 | 0.017108301 | 0.010007231 | 0.13549383 | 0.011429842 |
| M2b_M2b | 3 | 0.015916569 | 0.0059051569 | 0.18908702 | 0.012990031 |

## M1 vs M1b

| Metric | M1 | Comparator | Improvement |
| --- | ---: | ---: | ---: |
| support_error_rmse | 0.017755911 | 0.021843991 | 18.715% |
| support_error_max | 0.044697466 | 0.046454647 | 3.783% |
| rigid_fit_error_rmse | 0.0084876376 | 0.0088236448 | 3.808% |
| rigid_fit_error_max | 0.016979269 | 0.017085301 | 0.621% |
| pairwise_side_error_rmse | 0.016611779 | 0.017467383 | 4.898% |
| pairwise_side_error_max | 0.034953443 | 0.035998177 | 2.902% |
| equivalent_load_position_error_rmse | 0.011265133 | 0.014374045 | 21.629% |
| equivalent_load_position_error_max | 0.025927101 | 0.02726138 | 4.894% |
| equivalent_load_yaw_error_rmse | 0.021737362 | 0.026408832 | 17.689% |
| equivalent_load_yaw_error_max | 0.053515695 | 0.057065162 | 6.220% |
| native_controller_raw_peak_mps | 0.17078615 | 0.2032383 | 15.968% |
| native_controller_raw_rms_mps | 0.10162032 | 0.11643461 | 12.723% |
| native_over_0p160_duration_seconds | 1.020623 | 5.160089 | 80.221% |
| native_over_0p180_duration_seconds | 0 | 2.2003701 | 100.000% |
| post_candidate_b_peak_mps | 0.15832019 | 0.15771488 | -0.384% |
| post_candidate_b_over_0p160_duration_seconds | 0 | 0 | N/A |
| post_candidate_b_over_0p180_duration_seconds | 0 | 0 | N/A |
| physical_limiter_duration_seconds | 0 | 0 | N/A |
| post_limit_peak_mps | 0.15832019 | 0.15771488 | -0.384% |
| actual_peak_mps | 0.15620467 | 0.1560307 | -0.111% |
| wheel_margin_below_0p5_duration_seconds | 2.0600905 | 5.7001312 | 63.859% |
| wheel_margin_below_0p7_duration_seconds | 2.4904227 | 5.8503258 | 57.431% |
| robot1_progress_error_rmse | 0.013514898 | 0.015582672 | 13.270% |
| robot1_lateral_error_rmse | 0.0044761713 | 0.0056032333 | 20.114% |
| robot1_heading_error_rmse | 0.037835471 | 0.037830848 | -0.012% |
| robot1_velocity_error_rmse | 0.012109078 | 0.011210583 | -8.015% |
| robot2_progress_error_rmse | 0.013618517 | 0.016862561 | 19.238% |
| robot2_lateral_error_rmse | 0.0080214117 | 0.010003799 | 19.816% |
| robot2_heading_error_rmse | 0.1280246 | 0.13556911 | 5.565% |
| robot2_velocity_error_rmse | 0.013110761 | 0.011189453 | -17.171% |
| robot3_progress_error_rmse | 0.013068353 | 0.015683342 | 16.674% |
| robot3_lateral_error_rmse | 0.004948307 | 0.00589301 | 16.031% |
| robot3_heading_error_rmse | 0.18390675 | 0.18909528 | 2.744% |
| robot3_velocity_error_rmse | 0.013068111 | 0.012713838 | -2.787% |
| wheel_margin_minimum | 0 | 0 | N/A |
| wheel_margin_mean | 0.94357325 | 0.81390533 | 15.932% |
| wheel_margin_p05 | 0.40109396 | 0 | N/A |

## M1 vs M2b

| Metric | M1 | Comparator | Improvement |
| --- | ---: | ---: | ---: |
| support_error_rmse | 0.017755911 | 0.022124693 | 19.746% |
| support_error_max | 0.044697466 | 0.047168125 | 5.238% |
| rigid_fit_error_rmse | 0.0084876376 | 0.008969765 | 5.375% |
| rigid_fit_error_max | 0.016979269 | 0.017338736 | 2.073% |
| pairwise_side_error_rmse | 0.016611779 | 0.017733157 | 6.324% |
| pairwise_side_error_max | 0.034953443 | 0.036406582 | 3.991% |
| equivalent_load_position_error_rmse | 0.011265133 | 0.014543177 | 22.540% |
| equivalent_load_position_error_max | 0.025927101 | 0.027548469 | 5.886% |
| equivalent_load_yaw_error_rmse | 0.021737362 | 0.026694645 | 18.570% |
| equivalent_load_yaw_error_max | 0.053515695 | 0.05779366 | 7.402% |
| native_controller_raw_peak_mps | 0.17078615 | 0.20362121 | 16.126% |
| native_controller_raw_rms_mps | 0.10162032 | 0.11629558 | 12.619% |
| native_over_0p160_duration_seconds | 1.020623 | 5.1287777 | 80.100% |
| native_over_0p180_duration_seconds | 0 | 2.1998861 | 100.000% |
| post_candidate_b_peak_mps | 0.15832019 | 0.15810115 | -0.139% |
| post_candidate_b_over_0p160_duration_seconds | 0 | 0 | N/A |
| post_candidate_b_over_0p180_duration_seconds | 0 | 0 | N/A |
| physical_limiter_duration_seconds | 0 | 0 | N/A |
| post_limit_peak_mps | 0.15832019 | 0.15810115 | -0.139% |
| actual_peak_mps | 0.15620467 | 0.15810115 | 1.200% |
| wheel_margin_below_0p5_duration_seconds | 2.0600905 | 5.6889937 | 63.788% |
| wheel_margin_below_0p7_duration_seconds | 2.4904227 | 5.8290277 | 57.276% |
| robot1_progress_error_rmse | 0.013514898 | 0.015781006 | 14.360% |
| robot1_lateral_error_rmse | 0.0044761713 | 0.0056173013 | 20.315% |
| robot1_heading_error_rmse | 0.037835471 | 0.037759483 | -0.201% |
| robot1_velocity_error_rmse | 0.012109078 | 0.011513031 | -5.177% |
| robot2_progress_error_rmse | 0.013618517 | 0.017108301 | 20.398% |
| robot2_lateral_error_rmse | 0.0080214117 | 0.010007231 | 19.844% |
| robot2_heading_error_rmse | 0.1280246 | 0.13549383 | 5.513% |
| robot2_velocity_error_rmse | 0.013110761 | 0.011429842 | -14.706% |
| robot3_progress_error_rmse | 0.013068353 | 0.015916569 | 17.895% |
| robot3_lateral_error_rmse | 0.004948307 | 0.0059051569 | 16.204% |
| robot3_heading_error_rmse | 0.18390675 | 0.18908702 | 2.740% |
| robot3_velocity_error_rmse | 0.013068111 | 0.012990031 | -0.601% |
| wheel_margin_minimum | 0 | 0 | N/A |
| wheel_margin_mean | 0.94357325 | 0.81545067 | 15.712% |
| wheel_margin_p05 | 0.40109396 | 0 | N/A |

## Spatial verification

```json
{
  "M1": {
    "Robot1": {
      "d_omega_peak": 0.2624995889492893,
      "d_omega_rms": 0.16826268495158936,
      "d_v_peak": 0.019999995468944765,
      "d_v_rms": 0.01406390981089315,
      "entry_common_progress": 1.8311884277516541,
      "entry_wall_time": 1789561564.6788151,
      "exit_common_progress": 2.6325860761949063,
      "exit_wall_time": 1789561573.7588885,
      "exposure_duration_seconds": 9.080073356628418,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.17226993146255962,
      "projection_distance_mean": 0.021293479600183875,
      "q_max": 1.0,
      "q_mean": 0.9222421438044215,
      "zone_entry_progress": 2.001165356337687,
      "zone_exit_progress": 2.8011543465525306
    },
    "Robot2": {
      "d_omega_peak": 0.3018747969619119,
      "d_omega_rms": 0.19848748624193402,
      "d_v_peak": 0.022999989881488644,
      "d_v_rms": 0.01623153568826505,
      "entry_common_progress": 2.0711778352839247,
      "entry_common_progress_minus_robot1_m": 0.2399894075322706,
      "entry_time_minus_robot1_seconds": 2.400036334991455,
      "entry_wall_time": 1789561567.0788515,
      "exit_common_progress": 2.8778319172111813,
      "exit_wall_time": 1789561576.4786978,
      "exposure_duration_seconds": 9.399846315383911,
      "minimum_rho": 0.8275,
      "projection_distance_available": true,
      "projection_distance_max": 0.17315879170727308,
      "projection_distance_mean": 0.15392696621484078,
      "q_max": 1.0,
      "q_mean": 0.9433893769850671,
      "zone_entry_progress": 2.000278037240747,
      "zone_exit_progress": 2.802080613729847
    },
    "Robot3": {
      "d_omega_peak": 0.2231246604219533,
      "d_omega_rms": 0.14610386607989034,
      "d_v_peak": 0.016999995708353723,
      "d_v_rms": 0.012011190534359024,
      "entry_common_progress": 2.1111761768081543,
      "entry_common_progress_minus_robot1_m": 0.27998774905650015,
      "entry_time_minus_robot1_seconds": 2.800063371658325,
      "entry_wall_time": 1789561567.4788785,
      "exit_common_progress": 2.9130645377910414,
      "exit_wall_time": 1789561576.8387523,
      "exposure_duration_seconds": 9.35987377166748,
      "minimum_rho": 0.8725,
      "projection_distance_available": true,
      "projection_distance_max": 0.17315872931806983,
      "projection_distance_mean": 0.14359264358987636,
      "q_max": 1.0,
      "q_mean": 0.9415333045157661,
      "zone_entry_progress": 2.0013241903058576,
      "zone_exit_progress": 2.8005725099865737
    },
    "entry_order": [
      [
        1789561564.6788151,
        "Robot1"
      ],
      [
        1789561567.0788515,
        "Robot2"
      ],
      [
        1789561567.4788785,
        "Robot3"
      ]
    ]
  },
  "M1b": {
    "Robot1": {
      "d_omega_peak": 0.26249958828448466,
      "d_omega_rms": 0.17701318507678593,
      "d_v_peak": 0.019999992537355633,
      "d_v_rms": 0.013684330081807845,
      "entry_common_progress": 1.8316507938229376,
      "entry_wall_time": 1789561476.6010108,
      "exit_common_progress": 2.6304870394381252,
      "exit_wall_time": 1789561484.5808651,
      "exposure_duration_seconds": 7.979854345321655,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.1729049155355703,
      "projection_distance_mean": 0.021153624637440065,
      "q_max": 1.0,
      "q_mean": 0.9368763298452919,
      "zone_entry_progress": 2.001673016717877,
      "zone_exit_progress": 2.800410342870671
    },
    "Robot2": {
      "d_omega_peak": 0.3018747910766085,
      "d_omega_rms": 0.20316001713751294,
      "d_v_peak": 0.02299998411630944,
      "d_v_rms": 0.01575271191803212,
      "entry_common_progress": 2.0719216311948565,
      "entry_common_progress_minus_robot1_m": 0.2402708373719189,
      "entry_time_minus_robot1_seconds": 2.4002301692962646,
      "entry_wall_time": 1789561479.001241,
      "exit_common_progress": 2.8707600702863085,
      "exit_wall_time": 1789561486.9810185,
      "exposure_duration_seconds": 7.979777574539185,
      "minimum_rho": 0.8275,
      "projection_distance_available": true,
      "projection_distance_max": 0.1723340251439452,
      "projection_distance_mean": 0.15435430361090988,
      "q_max": 1.0,
      "q_mean": 0.9360847099890723,
      "zone_entry_progress": 2.0010133490316804,
      "zone_exit_progress": 2.8017636356233115
    },
    "Robot3": {
      "d_omega_peak": 0.22312475080227018,
      "d_omega_rms": 0.15033465709422075,
      "d_v_peak": 0.016999995373755218,
      "d_v_rms": 0.011582783594951851,
      "entry_common_progress": 2.1099583196352456,
      "entry_common_progress_minus_robot1_m": 0.278307525812308,
      "entry_time_minus_robot1_seconds": 2.779855489730835,
      "entry_wall_time": 1789561479.3808663,
      "exit_common_progress": 2.904798628811095,
      "exit_wall_time": 1789561487.3208146,
      "exposure_duration_seconds": 7.939948320388794,
      "minimum_rho": 0.8725,
      "projection_distance_available": true,
      "projection_distance_max": 0.17239964566189692,
      "projection_distance_mean": 0.1436229084940349,
      "q_max": 1.0,
      "q_mean": 0.932485325809324,
      "zone_entry_progress": 2.000198538185031,
      "zone_exit_progress": 2.8004404152390117
    },
    "entry_order": [
      [
        1789561476.6010108,
        "Robot1"
      ],
      [
        1789561479.001241,
        "Robot2"
      ],
      [
        1789561479.3808663,
        "Robot3"
      ]
    ]
  },
  "M2b": {
    "Robot1": {
      "d_omega_peak": 0.26249971453835247,
      "d_omega_rms": 0.17677962018234825,
      "d_v_peak": 0.019999996819768308,
      "d_v_rms": 0.013679840315366464,
      "entry_common_progress": 1.8314995400601228,
      "entry_wall_time": 1789561654.42406,
      "exit_common_progress": 2.631501164431113,
      "exit_wall_time": 1789561662.424272,
      "exposure_duration_seconds": 8.000211954116821,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.17301242335169567,
      "projection_distance_mean": 0.021185852012247507,
      "q_max": 1.0,
      "q_mean": 0.9353507598017058,
      "zone_entry_progress": 2.001594088162037,
      "zone_exit_progress": 2.8013175758294357
    },
    "Robot2": {
      "d_omega_peak": 0.30187452141453525,
      "d_omega_rms": 0.20308679392114945,
      "d_v_peak": 0.0229999869366499,
      "d_v_rms": 0.015768987749436834,
      "entry_common_progress": 2.0715010199715196,
      "entry_common_progress_minus_robot1_m": 0.24000147991139675,
      "entry_time_minus_robot1_seconds": 2.400217056274414,
      "entry_wall_time": 1789561656.8242772,
      "exit_common_progress": 2.8694973193014306,
      "exit_wall_time": 1789561664.8038225,
      "exposure_duration_seconds": 7.97954535484314,
      "minimum_rho": 0.8275,
      "projection_distance_available": true,
      "projection_distance_max": 0.17211273189405374,
      "projection_distance_mean": 0.15440124909379005,
      "q_max": 1.0,
      "q_mean": 0.9367347033986234,
      "zone_entry_progress": 2.0006960264357865,
      "zone_exit_progress": 2.8008236769712096
    },
    "Robot3": {
      "d_omega_peak": 0.22312465267496384,
      "d_omega_rms": 0.15058280846812103,
      "d_v_peak": 0.016999996562456068,
      "d_v_rms": 0.011569640361576419,
      "entry_common_progress": 2.111501074134631,
      "entry_common_progress_minus_robot1_m": 0.2800015340745081,
      "entry_time_minus_robot1_seconds": 2.7999579906463623,
      "entry_wall_time": 1789561657.224018,
      "exit_common_progress": 2.9054982653617816,
      "exit_wall_time": 1789561665.1638575,
      "exposure_duration_seconds": 7.9398393630981445,
      "minimum_rho": 0.8725,
      "projection_distance_available": true,
      "projection_distance_max": 0.17203924586305383,
      "projection_distance_mean": 0.14358164292695058,
      "q_max": 1.0,
      "q_mean": 0.9330588209844317,
      "zone_entry_progress": 2.0018056044488803,
      "zone_exit_progress": 2.8010192400461573
    },
    "entry_order": [
      [
        1789561654.42406,
        "Robot1"
      ],
      [
        1789561656.8242772,
        "Robot2"
      ],
      [
        1789561657.224018,
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
  "actual_reference_reduction_mean": 0.0034056626786572427,
  "actual_reference_reduction_peak": 0.014347967499237106,
  "common_reference_mean": 0.08897400645806515,
  "common_reference_minimum": 0.06138521927156993,
  "risk_boundary_active_duration": 8.829983234405518,
  "risk_boundary_active_fraction": 0.7261530356961389,
  "risk_contraction_integral": 0.1961941911221367,
  "risk_contraction_peak": 0.04561478072843007,
  "risk_factor_mean": 0.13766402022426305,
  "risk_factor_peak": 1.0,
  "risk_signal_mean": 0.13078081921304993,
  "risk_signal_peak": 0.95,
  "robust_margin_mean": 0.8303134243424685,
  "robust_margin_minimum": 0.0
}
```

The frozen disturbance drove robust margin to zero at its worst sample, risk factor to 1.0, and risk contraction to its recorded peak. The risk boundary was active for the recorded duration and produced the measured candidate-minus-effective reference reduction. This timing-consistent chain supports, but does not by itself prove, strict causality.

M1 task-time cost was 2.964% versus M1b and 2.864% versus M2b.

## Validity and physical-readiness notes

- All three runs completed with zero algorithm-invalid and localization-invalid fractions and identical fake-only disturbance profiles.
- M1b/M2b pre-disturbance native controller demand exceeded 0.180 m/s for 2.200 s and 2.200 s, respectively, during the analysis window. This does not invalidate the fake scientific comparison, but requires a separate physical-safe qualification.
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

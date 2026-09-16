# Candidate B v2 spatial composite fake results

EXPLORATORY_FAKE_VALIDATION
NOT_FORMAL_PAPER_EVIDENCE

Commit: `0532fcd77591d39cd82c5503a6d56523574aad2b`

Start dirty state:

```text
clean
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

Candidate B v2 rejects a spatial projection unless it is valid, has finite progress and distance, and remains within 0.30 m of the load centerline. The limit is deliberately wider than the nominal 0.15 m support-centre lateral offset of the 30 cm formation, while still rejecting grossly inconsistent poses.

The coordinate therefore represents the measured support centre crossing the same world-space centerline section. No 2.598 s delay and no fixed 8 s termination are used; each robot owns an independent entry wall clock and exits only after its projected progress reaches 2.8 m.

Fresh run paths:

- `/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/experiment_data/candidate_B_spatial_composite_fake/CANDIDATE_B_SPATIAL_COMPOSITE_RHO0P85_AV0P020_20260916_134850/M1b_R1/run`
- `/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/experiment_data/candidate_B_spatial_composite_fake/CANDIDATE_B_SPATIAL_COMPOSITE_RHO0P85_AV0P020_20260916_134850/M1_R1/run`
- `/home/etlab/AGV_ROS/.worktrees/platform-foundation-linux/experiment_data/candidate_B_spatial_composite_fake/CANDIDATE_B_SPATIAL_COMPOSITE_RHO0P85_AV0P020_20260916_134850/M2b_M2b/run`

Fake comparison: **VALID**; physical readiness: **NOT_AUTHORIZED**.

| Method | task/s | support RMS/max | rigid RMS/max | side RMS/max | load pos/yaw RMS | native peak | native >.160/s | native >.180/s | post-B peak | post-B >.160/.180 s | limiter/s | post-limit/actual peak | margin min/mean/p05 | margin <.5/<.7 s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| M1_R1 | 52.82 | 0.0156297/0.0382639 | 0.00852273/0.0174494 | 0.0179251/0.0371727 | 0.011353/0.0163958 | 0.165387 | 0.740078 | 0 | 0.150897 | 0/0 | 0 | 0.150897/0.150897 | 0/0.951776/0.502293 | 1.7904/2.06013 |
| M1b_R1 | 51.77 | 0.0186377/0.0394113 | 0.00881852/0.0174757 | 0.0187719/0.0373351 | 0.0138225/0.0194211 | 0.191275 | 4.60974 | 1.70999 | 0.15422 | 0/0 | 0 | 0.15422/0.15422 | 0/0.831933/0 | 5.40096/5.64918 |
| M2b_M2b | 51.68 | 0.0190816/0.0403199 | 0.00901969/0.017951 | 0.0191716/0.038436 | 0.0141773/0.0197053 | 0.19211 | 4.71977 | 1.77976 | 0.155768 | 0/0 | 0 | 0.155768/0.155389 | 0/0.829288/0 | 5.46995/5.68085 |

## Per-robot RMSE

| Method | Robot | progress / m | lateral / m | heading / rad | velocity / (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: |
| M1_R1 | 1 | 0.013645655 | 0.0046675078 | 0.03742618 | 0.011699111 |
| M1_R1 | 2 | 0.012015727 | 0.0071201986 | 0.12712299 | 0.011060471 |
| M1_R1 | 3 | 0.01581849 | 0.0057256478 | 0.18612079 | 0.015126439 |
| M1b_R1 | 1 | 0.015441235 | 0.0055135912 | 0.0377949 | 0.011182271 |
| M1b_R1 | 2 | 0.014347447 | 0.0087339803 | 0.13217179 | 0.0096961189 |
| M1b_R1 | 3 | 0.018134669 | 0.0066717393 | 0.19075325 | 0.014709775 |
| M2b_M2b | 1 | 0.015807108 | 0.0055939185 | 0.037743968 | 0.011587753 |
| M2b_M2b | 2 | 0.014737976 | 0.0088133636 | 0.13244902 | 0.010010559 |
| M2b_M2b | 3 | 0.018678232 | 0.0067411952 | 0.19095482 | 0.015198657 |

## M1 vs M1b

| Metric | M1 | Comparator | Improvement |
| --- | ---: | ---: | ---: |
| support_error_rmse | 0.015629674 | 0.018637694 | 16.139% |
| support_error_max | 0.038263899 | 0.039411252 | 2.911% |
| rigid_fit_error_rmse | 0.0085227336 | 0.0088185223 | 3.354% |
| rigid_fit_error_max | 0.01744941 | 0.017475685 | 0.150% |
| pairwise_side_error_rmse | 0.017925125 | 0.018771922 | 4.511% |
| pairwise_side_error_max | 0.037172707 | 0.037335094 | 0.435% |
| equivalent_load_position_error_rmse | 0.011353044 | 0.013822531 | 17.866% |
| equivalent_load_position_error_max | 0.025090386 | 0.025818634 | 2.821% |
| equivalent_load_yaw_error_rmse | 0.01639578 | 0.01942106 | 15.577% |
| equivalent_load_yaw_error_max | 0.038389844 | 0.041560847 | 7.630% |
| native_controller_raw_peak_mps | 0.16538734 | 0.1912754 | 13.534% |
| native_controller_raw_rms_mps | 0.10308542 | 0.11538662 | 10.661% |
| native_over_0p160_duration_seconds | 0.74007797 | 4.6097364 | 83.945% |
| native_over_0p180_duration_seconds | 0 | 1.7099874 | 100.000% |
| post_candidate_b_peak_mps | 0.15089666 | 0.15421988 | 2.155% |
| post_candidate_b_over_0p160_duration_seconds | 0 | 0 | N/A |
| post_candidate_b_over_0p180_duration_seconds | 0 | 0 | N/A |
| physical_limiter_duration_seconds | 0 | 0 | N/A |
| post_limit_peak_mps | 0.15089666 | 0.15421988 | 2.155% |
| actual_peak_mps | 0.15089666 | 0.15421988 | 2.155% |
| wheel_margin_below_0p5_duration_seconds | 1.7904048 | 5.400965 | 66.850% |
| wheel_margin_below_0p7_duration_seconds | 2.0601304 | 5.6491756 | 63.532% |
| robot1_progress_error_rmse | 0.013645655 | 0.015441235 | 11.628% |
| robot1_lateral_error_rmse | 0.0046675078 | 0.0055135912 | 15.345% |
| robot1_heading_error_rmse | 0.03742618 | 0.0377949 | 0.976% |
| robot1_velocity_error_rmse | 0.011699111 | 0.011182271 | -4.622% |
| robot2_progress_error_rmse | 0.012015727 | 0.014347447 | 16.252% |
| robot2_lateral_error_rmse | 0.0071201986 | 0.0087339803 | 18.477% |
| robot2_heading_error_rmse | 0.12712299 | 0.13217179 | 3.820% |
| robot2_velocity_error_rmse | 0.011060471 | 0.0096961189 | -14.071% |
| robot3_progress_error_rmse | 0.01581849 | 0.018134669 | 12.772% |
| robot3_lateral_error_rmse | 0.0057256478 | 0.0066717393 | 14.181% |
| robot3_heading_error_rmse | 0.18612079 | 0.19075325 | 2.429% |
| robot3_velocity_error_rmse | 0.015126439 | 0.014709775 | -2.833% |
| wheel_margin_minimum | 0 | 0 | N/A |
| wheel_margin_mean | 0.95177627 | 0.83193348 | 14.405% |
| wheel_margin_p05 | 0.50229315 | 0 | N/A |

## M1 vs M2b

| Metric | M1 | Comparator | Improvement |
| --- | ---: | ---: | ---: |
| support_error_rmse | 0.015629674 | 0.019081638 | 18.091% |
| support_error_max | 0.038263899 | 0.040319876 | 5.099% |
| rigid_fit_error_rmse | 0.0085227336 | 0.0090196886 | 5.510% |
| rigid_fit_error_max | 0.01744941 | 0.01795103 | 2.794% |
| pairwise_side_error_rmse | 0.017925125 | 0.019171589 | 6.502% |
| pairwise_side_error_max | 0.037172707 | 0.038435956 | 3.287% |
| equivalent_load_position_error_rmse | 0.011353044 | 0.01417732 | 19.921% |
| equivalent_load_position_error_max | 0.025090386 | 0.026441092 | 5.108% |
| equivalent_load_yaw_error_rmse | 0.01639578 | 0.019705305 | 16.795% |
| equivalent_load_yaw_error_max | 0.038389844 | 0.042072988 | 8.754% |
| native_controller_raw_peak_mps | 0.16538734 | 0.19210994 | 13.910% |
| native_controller_raw_rms_mps | 0.10308542 | 0.11573064 | 10.926% |
| native_over_0p160_duration_seconds | 0.74007797 | 4.719768 | 84.320% |
| native_over_0p180_duration_seconds | 0 | 1.7797575 | 100.000% |
| post_candidate_b_peak_mps | 0.15089666 | 0.1557679 | 3.127% |
| post_candidate_b_over_0p160_duration_seconds | 0 | 0 | N/A |
| post_candidate_b_over_0p180_duration_seconds | 0 | 0 | N/A |
| physical_limiter_duration_seconds | 0 | 0 | N/A |
| post_limit_peak_mps | 0.15089666 | 0.1557679 | 3.127% |
| actual_peak_mps | 0.15089666 | 0.15538923 | 2.891% |
| wheel_margin_below_0p5_duration_seconds | 1.7904048 | 5.4699531 | 67.268% |
| wheel_margin_below_0p7_duration_seconds | 2.0601304 | 5.6808541 | 63.736% |
| robot1_progress_error_rmse | 0.013645655 | 0.015807108 | 13.674% |
| robot1_lateral_error_rmse | 0.0046675078 | 0.0055939185 | 16.561% |
| robot1_heading_error_rmse | 0.03742618 | 0.037743968 | 0.842% |
| robot1_velocity_error_rmse | 0.011699111 | 0.011587753 | -0.961% |
| robot2_progress_error_rmse | 0.012015727 | 0.014737976 | 18.471% |
| robot2_lateral_error_rmse | 0.0071201986 | 0.0088133636 | 19.211% |
| robot2_heading_error_rmse | 0.12712299 | 0.13244902 | 4.021% |
| robot2_velocity_error_rmse | 0.011060471 | 0.010010559 | -10.488% |
| robot3_progress_error_rmse | 0.01581849 | 0.018678232 | 15.311% |
| robot3_lateral_error_rmse | 0.0057256478 | 0.0067411952 | 15.065% |
| robot3_heading_error_rmse | 0.18612079 | 0.19095482 | 2.532% |
| robot3_velocity_error_rmse | 0.015126439 | 0.015198657 | 0.475% |
| wheel_margin_minimum | 0 | 0 | N/A |
| wheel_margin_mean | 0.95177627 | 0.82928837 | 14.770% |
| wheel_margin_p05 | 0.50229315 | 0 | N/A |

## Spatial verification

```json
{
  "M1": {
    "Robot1": {
      "d_omega_peak": 0.26249989649852884,
      "d_omega_rms": 0.16910014775584062,
      "d_v_peak": 0.019999996156448947,
      "d_v_rms": 0.014039535171149848,
      "entry_common_progress": 1.8311420037896575,
      "entry_wall_time": 1789537893.6410487,
      "exit_common_progress": 2.632172983688352,
      "exit_wall_time": 1789537902.4813333,
      "exposure_duration_seconds": 8.840284585952759,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.17247280958812997,
      "projection_distance_mean": 0.02132693238594019,
      "q_max": 1.0,
      "q_mean": 0.9240512821230117,
      "zone_entry_progress": 2.0006023428445667,
      "zone_exit_progress": 2.800358109979339
    },
    "Robot2": {
      "d_omega_peak": 0.2624996342552273,
      "d_omega_rms": 0.17076905410867493,
      "d_v_peak": 0.019999993638571727,
      "d_v_rms": 0.014233866767964919,
      "entry_common_progress": 2.072169138200667,
      "entry_common_progress_minus_robot1_m": 0.24102713441100954,
      "entry_time_minus_robot1_seconds": 2.4102656841278076,
      "entry_wall_time": 1789537896.0513144,
      "exit_common_progress": 2.873918656494788,
      "exit_wall_time": 1789537905.1713705,
      "exposure_duration_seconds": 9.12005615234375,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.17263582897535446,
      "projection_distance_mean": 0.15436982549048436,
      "q_max": 1.0,
      "q_mean": 0.9429803264281401,
      "zone_entry_progress": 2.0010281335480697,
      "zone_exit_progress": 2.800342144844352
    },
    "Robot3": {
      "d_omega_peak": 0.2624998113401918,
      "d_omega_rms": 0.16963388578355865,
      "d_v_peak": 0.019999994886422855,
      "d_v_rms": 0.014193643871087895,
      "entry_common_progress": 2.1101813734186656,
      "entry_common_progress_minus_robot1_m": 0.2790393696290081,
      "entry_time_minus_robot1_seconds": 2.790152072906494,
      "entry_wall_time": 1789537896.4312007,
      "exit_common_progress": 2.912416519722168,
      "exit_wall_time": 1789537905.5510008,
      "exposure_duration_seconds": 9.119800090789795,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.17268964143651938,
      "projection_distance_mean": 0.14330544226363492,
      "q_max": 1.0,
      "q_mean": 0.9355482043052149,
      "zone_entry_progress": 2.000572321395228,
      "zone_exit_progress": 2.800436825055049
    },
    "entry_order": [
      [
        1789537893.6410487,
        "Robot1"
      ],
      [
        1789537896.0513144,
        "Robot2"
      ],
      [
        1789537896.4312007,
        "Robot3"
      ]
    ]
  },
  "M1b": {
    "Robot1": {
      "d_omega_peak": 0.26249975035026807,
      "d_omega_rms": 0.17652878070183195,
      "d_v_peak": 0.019999999927018265,
      "d_v_rms": 0.013703959068251639,
      "entry_common_progress": 1.8308557327479345,
      "entry_wall_time": 1789537806.2959406,
      "exit_common_progress": 2.630426227551611,
      "exit_wall_time": 1789537814.3063326,
      "exposure_duration_seconds": 8.0103919506073,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.17259373758447186,
      "projection_distance_mean": 0.02124522169276716,
      "q_max": 1.0,
      "q_mean": 0.9357115597840557,
      "zone_entry_progress": 2.000302609306905,
      "zone_exit_progress": 2.800130225347929
    },
    "Robot2": {
      "d_omega_peak": 0.2624995340674651,
      "d_omega_rms": 0.17644242308234032,
      "d_v_peak": 0.019999977458065997,
      "d_v_rms": 0.013717595592121355,
      "entry_common_progress": 2.0714053645018327,
      "entry_common_progress_minus_robot1_m": 0.2405496317538982,
      "entry_time_minus_robot1_seconds": 2.410813570022583,
      "entry_wall_time": 1789537808.7067542,
      "exit_common_progress": 2.8699865947792302,
      "exit_wall_time": 1789537816.7064703,
      "exposure_duration_seconds": 7.99971604347229,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.1725046639401374,
      "projection_distance_mean": 0.15453058961454771,
      "q_max": 1.0,
      "q_mean": 0.936292424363572,
      "zone_entry_progress": 2.0001061997895118,
      "zone_exit_progress": 2.8004248059993366
    },
    "Robot3": {
      "d_omega_peak": 0.26249956536151214,
      "d_omega_rms": 0.17665751769457932,
      "d_v_peak": 0.01999999271343164,
      "d_v_rms": 0.01362533139432121,
      "entry_common_progress": 2.110308288819341,
      "entry_common_progress_minus_robot1_m": 0.27945255607140673,
      "entry_time_minus_robot1_seconds": 2.8003149032592773,
      "entry_wall_time": 1789537809.0962555,
      "exit_common_progress": 2.9049022228172894,
      "exit_wall_time": 1789537817.055961,
      "exposure_duration_seconds": 7.959705352783203,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.17249471309931128,
      "projection_distance_mean": 0.143529170149596,
      "q_max": 1.0,
      "q_mean": 0.9314784170344211,
      "zone_entry_progress": 2.0000817715771833,
      "zone_exit_progress": 2.800361916366756
    },
    "entry_order": [
      [
        1789537806.2959406,
        "Robot1"
      ],
      [
        1789537808.7067542,
        "Robot2"
      ],
      [
        1789537809.0962555,
        "Robot3"
      ]
    ]
  },
  "M2b": {
    "Robot1": {
      "d_omega_peak": 0.2624993884596227,
      "d_omega_rms": 0.1765946220998524,
      "d_v_peak": 0.01999998689609176,
      "d_v_rms": 0.013694140629220758,
      "entry_common_progress": 1.8305060487146936,
      "entry_wall_time": 1789537982.3215308,
      "exit_common_progress": 2.6305139803254374,
      "exit_wall_time": 1789537990.32124,
      "exposure_duration_seconds": 7.999709129333496,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.1723350612338977,
      "projection_distance_mean": 0.02118744315369716,
      "q_max": 1.0,
      "q_mean": 0.9353232036030127,
      "zone_entry_progress": 2.0001249250309385,
      "zone_exit_progress": 2.800240057868624
    },
    "Robot2": {
      "d_omega_peak": 0.26249963195318604,
      "d_omega_rms": 0.17668027786052284,
      "d_v_peak": 0.019999998064371582,
      "d_v_rms": 0.013709472811078113,
      "entry_common_progress": 2.071500770655611,
      "entry_common_progress_minus_robot1_m": 0.2409947219409172,
      "entry_time_minus_robot1_seconds": 2.40978741645813,
      "entry_wall_time": 1789537984.7313182,
      "exit_common_progress": 2.8694996024106083,
      "exit_wall_time": 1789537992.7109392,
      "exposure_duration_seconds": 7.979620933532715,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.1726422954055246,
      "projection_distance_mean": 0.15449359879537286,
      "q_max": 1.0,
      "q_mean": 0.9369160745520708,
      "zone_entry_progress": 2.0002093028724217,
      "zone_exit_progress": 2.800084932375139
    },
    "Robot3": {
      "d_omega_peak": 0.2624998271994968,
      "d_omega_rms": 0.1770301657218902,
      "d_v_peak": 0.019999995201924,
      "d_v_rms": 0.013620074231511623,
      "entry_common_progress": 2.1105043581554455,
      "entry_common_progress_minus_robot1_m": 0.2799983094407519,
      "entry_time_minus_robot1_seconds": 2.799478054046631,
      "entry_wall_time": 1789537985.1210089,
      "exit_common_progress": 2.9035101323353167,
      "exit_wall_time": 1789537993.0511572,
      "exposure_duration_seconds": 7.930148363113403,
      "minimum_rho": 0.85,
      "projection_distance_available": true,
      "projection_distance_max": 0.17269605283777714,
      "projection_distance_mean": 0.14359640836094398,
      "q_max": 1.0,
      "q_mean": 0.9328214770506178,
      "zone_entry_progress": 2.000052660100994,
      "zone_exit_progress": 2.8002797298287194
    },
    "entry_order": [
      [
        1789537982.3215308,
        "Robot1"
      ],
      [
        1789537984.7313182,
        "Robot2"
      ],
      [
        1789537985.1210089,
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
  "actual_reference_reduction_mean": 0.002655248062571989,
  "actual_reference_reduction_peak": 0.01323760414025446,
  "common_reference_mean": 0.09078595169681963,
  "common_reference_minimum": 0.06836541365862092,
  "risk_boundary_active_duration": 8.168609142303467,
  "risk_contraction_integral": 0.1698230498222055,
  "risk_contraction_peak": 0.038634586341379076,
  "risk_factor_mean": 0.11787271416027933,
  "risk_factor_peak": 1.0,
  "risk_signal_mean": 0.11197907845226536,
  "risk_signal_peak": 0.95,
  "robust_margin_mean": 0.8538862299652663,
  "robust_margin_minimum": 0.0
}
```

The frozen disturbance drove robust margin to zero at its worst sample, risk factor to 1.0, and risk contraction to its recorded peak. The risk boundary was active for the recorded duration and produced the measured candidate-minus-effective reference reduction. This timing-consistent chain supports, but does not by itself prove, strict causality.

M1 task-time cost was 2.028% versus M1b and 2.206% versus M2b.

## Validity and physical-readiness notes

- All three runs completed with zero algorithm-invalid and localization-invalid fractions and identical fake-only disturbance profiles.
- M1b/M2b pre-disturbance native controller demand exceeded 0.180 m/s for 1.710 s and 1.780 s, respectively, during the analysis window. This does not invalidate the fake scientific comparison, but requires a separate physical-safe qualification.
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

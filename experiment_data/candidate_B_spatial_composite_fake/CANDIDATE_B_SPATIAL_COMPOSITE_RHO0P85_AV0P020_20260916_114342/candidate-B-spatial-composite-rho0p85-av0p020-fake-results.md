# Candidate B v2 spatial composite fake results

EXPLORATORY_FAKE_VALIDATION
NOT_FORMAL_PAPER_EVIDENCE

Commit: `4f2ee3d0b96c966b0d166a3c3eee6d4fda024c80`

Spatial coordinate: each measured support centre is projected onto the existing load centerline PathProjector. The original s_actual is a support-specific common load parameter and is not used as the world-zone coordinate.

Fake comparison: **VALID**; physical readiness: **NOT_AUTHORIZED**.

| Method | task/s | support RMS/max | rigid RMS/max | side RMS/max | native peak | native >.160/s | native >.180/s | post-B peak | post-B >.180/s | limiter/s | margin min/mean/p05 |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| M1_R1 | 52.72 | 0.0156016/0.0385803 | 0.00854475/0.0176283 | 0.0179558/0.0375384 | 0.16609 | 0.850562 | 0 | 0.151703 | 0 | 0 | 0/0.950817/0.493511 |
| M1b_R1 | 51.62 | 0.0186939/0.0395091 | 0.00883298/0.0176889 | 0.0188066/0.0377244 | 0.191562 | 4.68005 | 1.73074 | 0.154783 | 0 | 0 | 0/0.829913/0 |
| M2b_M2b | 51.6299 | 0.0190781/0.0403284 | 0.00899832/0.0180416 | 0.0191201/0.0386037 | 0.192126 | 4.69989 | 1.77028 | 0.155714 | 0 | 0 | 0/0.829464/0 |

## M1 vs M1b

| Metric | M1 | Comparator | Improvement |
| --- | ---: | ---: | ---: |
| support_error_rmse | 0.015601592 | 0.018693928 | 16.542% |
| support_error_max | 0.0385803 | 0.039509078 | 2.351% |
| rigid_fit_error_rmse | 0.0085447478 | 0.0088329848 | 3.263% |
| rigid_fit_error_max | 0.017628252 | 0.017688882 | 0.343% |
| pairwise_side_error_rmse | 0.017955806 | 0.018806597 | 4.524% |
| pairwise_side_error_max | 0.037538406 | 0.037724367 | 0.493% |
| equivalent_load_position_error_rmse | 0.011303534 | 0.013879675 | 18.561% |
| equivalent_load_yaw_error_rmse | 0.016477532 | 0.01947794 | 15.404% |
| native_controller_raw_peak_mps | 0.1660901 | 0.19156153 | 13.297% |
| native_over_0p160_duration_seconds | 0.85056233 | 4.6800539 | 81.826% |
| wheel_margin_minimum | 0 | 0 | N/A |

## M1 vs M2b

| Metric | M1 | Comparator | Improvement |
| --- | ---: | ---: | ---: |
| support_error_rmse | 0.015601592 | 0.019078143 | 18.223% |
| support_error_max | 0.0385803 | 0.040328363 | 4.335% |
| rigid_fit_error_rmse | 0.0085447478 | 0.0089983213 | 5.041% |
| rigid_fit_error_max | 0.017628252 | 0.018041577 | 2.291% |
| pairwise_side_error_rmse | 0.017955806 | 0.019120134 | 6.090% |
| pairwise_side_error_max | 0.037538406 | 0.038603744 | 2.760% |
| equivalent_load_position_error_rmse | 0.011303534 | 0.014165085 | 20.201% |
| equivalent_load_yaw_error_rmse | 0.016477532 | 0.019761377 | 16.617% |
| native_controller_raw_peak_mps | 0.1660901 | 0.19212639 | 13.552% |
| native_over_0p160_duration_seconds | 0.85056233 | 4.6998911 | 81.903% |
| wheel_margin_minimum | 0 | 0 | N/A |

## Spatial verification

```json
{
  "M1": {
    "Robot1": {
      "d_omega_peak": 0.26249961144842443,
      "d_omega_rms": 0.1690824389121022,
      "d_v_peak": 0.019999990042809883,
      "d_v_rms": 0.014049690902794804,
      "entry_common_progress": 1.8312928956526402,
      "entry_wall_time": 1789530386.6836126,
      "exit_common_progress": 2.632243097376933,
      "exit_wall_time": 1789530395.5433326,
      "exposure_duration_seconds": 8.85971999168396,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9246161142722953,
      "zone_entry_progress": 2.0009020345156063,
      "zone_exit_progress": 2.8005058990772453
    },
    "Robot2": {
      "d_omega_peak": 0.26249965149433174,
      "d_omega_rms": 0.17087143156609852,
      "d_v_peak": 0.019999990218083373,
      "d_v_rms": 0.014217016269060573,
      "entry_common_progress": 2.0715840341526044,
      "entry_common_progress_minus_robot1_m": 0.2402911384999642,
      "entry_time_minus_robot1_seconds": 2.399768114089966,
      "entry_wall_time": 1789530389.0833807,
      "exit_common_progress": 2.8749115491148842,
      "exit_wall_time": 1789530398.2535453,
      "exposure_duration_seconds": 9.170164585113525,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9421895831756096,
      "zone_entry_progress": 2.000313886991851,
      "zone_exit_progress": 2.8008567768198462
    },
    "Robot3": {
      "d_omega_peak": 0.26249973448287217,
      "d_omega_rms": 0.16983592020952273,
      "d_v_peak": 0.01999999628832766,
      "d_v_rms": 0.014184206387753834,
      "entry_common_progress": 2.110633513362529,
      "entry_common_progress_minus_robot1_m": 0.2793406177098887,
      "entry_time_minus_robot1_seconds": 2.789686441421509,
      "entry_wall_time": 1789530389.473299,
      "exit_common_progress": 2.9134012252852974,
      "exit_wall_time": 1789530398.6333613,
      "exposure_duration_seconds": 9.160062313079834,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9357928825910405,
      "zone_entry_progress": 2.0010064624392627,
      "zone_exit_progress": 2.8010266430217583
    },
    "entry_order": [
      [
        1789530386.6836126,
        "Robot1"
      ],
      [
        1789530389.0833807,
        "Robot2"
      ],
      [
        1789530389.473299,
        "Robot3"
      ]
    ]
  },
  "M1b": {
    "Robot1": {
      "d_omega_peak": 0.26249973750077654,
      "d_omega_rms": 0.17654029621843317,
      "d_v_peak": 0.019999992802704182,
      "d_v_rms": 0.013694725304034828,
      "entry_common_progress": 1.8303915285659058,
      "entry_wall_time": 1789530298.6088853,
      "exit_common_progress": 2.6305498445679127,
      "exit_wall_time": 1789530306.6090004,
      "exposure_duration_seconds": 8.000115156173706,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9351090758129635,
      "zone_entry_progress": 2.000378189295888,
      "zone_exit_progress": 2.8005414480576785
    },
    "Robot2": {
      "d_omega_peak": 0.26249979782539906,
      "d_omega_rms": 0.17667271884320915,
      "d_v_peak": 0.019999995005070503,
      "d_v_rms": 0.013711255062048138,
      "entry_common_progress": 2.0714962908511185,
      "entry_common_progress_minus_robot1_m": 0.24110476228521271,
      "entry_time_minus_robot1_seconds": 2.409960985183716,
      "entry_wall_time": 1789530301.0188463,
      "exit_common_progress": 2.869536931320354,
      "exit_wall_time": 1789530308.9992275,
      "exposure_duration_seconds": 7.98038125038147,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9370203074470599,
      "zone_entry_progress": 2.0003748363362552,
      "zone_exit_progress": 2.800163390856968
    },
    "Robot3": {
      "d_omega_peak": 0.2624997377662341,
      "d_omega_rms": 0.17691350677620157,
      "d_v_peak": 0.019999997354498717,
      "d_v_rms": 0.013619342263667858,
      "entry_common_progress": 2.110501343234742,
      "entry_common_progress_minus_robot1_m": 0.28010981466883633,
      "entry_time_minus_robot1_seconds": 2.7999343872070312,
      "entry_wall_time": 1789530301.4088197,
      "exit_common_progress": 2.904540785963282,
      "exit_wall_time": 1789530309.3489518,
      "exposure_duration_seconds": 7.940132141113281,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9323016647686909,
      "zone_entry_progress": 2.00036192436039,
      "zone_exit_progress": 2.80043403908791
    },
    "entry_order": [
      [
        1789530298.6088853,
        "Robot1"
      ],
      [
        1789530301.0188463,
        "Robot2"
      ],
      [
        1789530301.4088197,
        "Robot3"
      ]
    ]
  },
  "M2b": {
    "Robot1": {
      "d_omega_peak": 0.26249973670359816,
      "d_omega_rms": 0.17688429669817504,
      "d_v_peak": 0.019999995659482753,
      "d_v_rms": 0.013688191396388107,
      "entry_common_progress": 1.8315048480623428,
      "entry_wall_time": 1789530475.715817,
      "exit_common_progress": 2.630511410782333,
      "exit_wall_time": 1789530483.706145,
      "exposure_duration_seconds": 7.990328073501587,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9364522813711169,
      "zone_entry_progress": 2.0011433761749133,
      "zone_exit_progress": 2.800040203584532
    },
    "Robot2": {
      "d_omega_peak": 0.2624997021307118,
      "d_omega_rms": 0.17667607659890094,
      "d_v_peak": 0.019999990821568444,
      "d_v_rms": 0.013708933958539133,
      "entry_common_progress": 2.0715037582363944,
      "entry_common_progress_minus_robot1_m": 0.23999891017405162,
      "entry_time_minus_robot1_seconds": 2.4003958702087402,
      "entry_wall_time": 1789530478.1162128,
      "exit_common_progress": 2.86950002031535,
      "exit_wall_time": 1789530486.0961776,
      "exposure_duration_seconds": 7.979964733123779,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.9368464231777355,
      "zone_entry_progress": 2.0002082272932022,
      "zone_exit_progress": 2.8000915898575225
    },
    "Robot3": {
      "d_omega_peak": 0.2624996344641457,
      "d_omega_rms": 0.17694040777688552,
      "d_v_peak": 0.01999999256205057,
      "d_v_rms": 0.01361195584253711,
      "entry_common_progress": 2.110503897451751,
      "entry_common_progress_minus_robot1_m": 0.27899904938940834,
      "entry_time_minus_robot1_seconds": 2.7902462482452393,
      "entry_wall_time": 1789530478.5060632,
      "exit_common_progress": 2.904505801441932,
      "exit_wall_time": 1789530486.4460583,
      "exposure_duration_seconds": 7.939995050430298,
      "minimum_rho": 0.85,
      "q_max": 1.0,
      "q_mean": 0.931789294476438,
      "zone_entry_progress": 2.0007619609088847,
      "zone_exit_progress": 2.8013620116703564
    },
    "entry_order": [
      [
        1789530475.715817,
        "Robot1"
      ],
      [
        1789530478.1162128,
        "Robot2"
      ],
      [
        1789530478.5060632,
        "Robot3"
      ]
    ]
  }
}
```

## M1 mechanism

```json
{
  "actual_reference_reduction_mean": 0.0,
  "actual_reference_reduction_peak": 0.0,
  "common_reference_mean": 0.09055131664768999,
  "common_reference_minimum": 0.06727738023864678,
  "risk_boundary_active_duration": 0,
  "risk_contraction_integral": 0.17360914662338986,
  "risk_contraction_peak": 0.03972261976135322,
  "risk_factor_mean": 0.12032243365511765,
  "risk_factor_peak": 1.0,
  "risk_signal_mean": 0.11430631197236178,
  "risk_signal_peak": 0.95,
  "robust_margin_mean": 0.8524789317429374,
  "robust_margin_minimum": 0.0
}
```

CapabilityReport was not modified. Candidate A and Candidate B v1 remained disabled. No serial or hardware execution was started.

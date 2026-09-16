# Candidate B rho=0.85 fake results

EXPLORATORY_FAKE_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE

Commit: `6914e2716e9f57fa2655e7a7913f4ca75641c199`

Formula: `v_c_exe=rho*v_c_native`, `omega_exe=omega_native+d_omega`; native omega is not scaled by rho.
Frozen profile: Robot2 only, `rho_min=0.85`, `d_omega_peak=0.2625 rad/s`, progress trigger `2.0 m`, duration `8.0 s`, q5 ramp-in/out `0.5 s`.
M1/M1b use frozen R1; M2b uses its complete frozen upper/reference generator and nonlinear lower controller. All runs use fake transport.

Final decision: **RHO_0P85_TOO_STRONG**

| Method | Valid | task/s | native peak | post-B peak | post-limit peak | actual peak | post-B >0.160/s | limiter/s | post-B >0.18/s | support RMS/max | rigid RMS/max | side RMS/max | R2 progress/lateral/heading RMS |
| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| M1_R1 | VALID_COMPLETED | 52.46 | 0.171798 | 0.170359 | 0.16 | 0.16 | 0.140092 | 0.140092 | 0 | 0.0167938/0.0237028 | 0.00630647/0.00892917 | 0.0138508/0.021299 | 0.0120586/0.008649/0.0522345 |
| M1b_R1 | VALID_COMPLETED | 51.59 | 0.17854 | 0.17844 | 0.16 | 0.16 | 0.219923 | 0.219923 | 0 | 0.018372/0.0246346 | 0.00682481/0.00936464 | 0.0150759/0.0217126 | 0.013139/0.00967867/0.0528476 |
| M2b_M2b | VALID_COMPLETED | 51.56 | 0.180539 | 0.180489 | 0.16 | 0.16 | 0.220012 | 0.210011 | 0.0302355 | 0.0187557/0.0247325 | 0.00690884/0.00944044 | 0.0152238/0.0218325 | 0.0134781/0.00974134/0.0530428 |

The five execution layers are reported independently as native controller demand, post-effectiveness longitudinal component, post-yaw Candidate-B demand, post-0.160 limiter feedback, and actual fake-plant feedback.

## Absolute and percentage comparisons

M1 versus M1b (positive percentage means lower M1 error/burden):

| Metric | M1 | M1b | improvement |
| --- | ---: | ---: | ---: |
| support_error_rmse | 0.016793753 | 0.018371973 | 8.590% |
| support_error_max | 0.023702775 | 0.024634569 | 3.782% |
| rigid_fit_error_rmse | 0.0063064685 | 0.0068248069 | 7.595% |
| rigid_fit_error_max | 0.0089291654 | 0.0093646439 | 4.650% |
| pairwise_side_error_rmse | 0.013850804 | 0.015075881 | 8.126% |
| pairwise_side_error_max | 0.021299026 | 0.021712593 | 1.905% |
| progress_error_rmse | 0.012058623 | 0.013139019 | 8.223% |
| lateral_error_rmse | 0.0086489979 | 0.0096786707 | 10.639% |
| heading_error_rmse | 0.052234518 | 0.052847576 | 1.160% |
| native_controller_raw_peak_mps | 0.17179834 | 0.17853956 | 3.776% |
| wheel_margin_minimum | 0 | 0 | n/a |
| limiter_duration_seconds | 0.1400919 | 0.21992338 | 36.300% |

M1 versus M2b (positive percentage means lower M1 error/burden):

| Metric | M1 | M2b | improvement |
| --- | ---: | ---: | ---: |
| support_error_rmse | 0.016793753 | 0.018755746 | 10.461% |
| support_error_max | 0.023702775 | 0.024732464 | 4.163% |
| rigid_fit_error_rmse | 0.0063064685 | 0.0069088397 | 8.719% |
| rigid_fit_error_max | 0.0089291654 | 0.0094404408 | 5.416% |
| pairwise_side_error_rmse | 0.013850804 | 0.015223848 | 9.019% |
| pairwise_side_error_max | 0.021299026 | 0.021832507 | 2.444% |
| progress_error_rmse | 0.012058623 | 0.01347805 | 10.531% |
| lateral_error_rmse | 0.0086489979 | 0.0097413417 | 11.213% |
| heading_error_rmse | 0.052234518 | 0.053042802 | 1.524% |
| native_controller_raw_peak_mps | 0.17179834 | 0.18053852 | 4.841% |
| wheel_margin_minimum | 0 | 0 | n/a |
| limiter_duration_seconds | 0.1400919 | 0.21001101 | 33.293% |

## M1 causal mechanism

```json
{
  "actual_reference_reduction_mean": 0.001244850981945073,
  "actual_reference_reduction_peak": 0.013229209761396521,
  "common_reference_mean": 0.09637991575677149,
  "common_reference_minimum": 0.0698545608425474,
  "risk_boundary_active_duration": 8.709796905517578,
  "risk_contraction_integral": 0.1668261915584657,
  "risk_contraction_peak": 0.037145439157452595,
  "risk_factor_mean": 0.04970810785770096,
  "risk_factor_peak": 1.0,
  "risk_signal_mean": 0.04722270246481589,
  "risk_signal_peak": 0.95,
  "robust_margin_mean": 0.9260768141515527,
  "robust_margin_minimum": 0.0
}
```

All three CapabilityReport wheel limits remained exactly 0.160 m/s; Candidate B does not mutate capability.
Recommend fallback to rho=0.90: `True`.
No physical/serial execution was authorized or started.

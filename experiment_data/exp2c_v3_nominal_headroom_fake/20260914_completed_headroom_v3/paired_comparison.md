# Exp2c-v3 paired fake comparison

A = 0.020 m/s; nominal inner upper = 0.107 m/s

| metric | M1+R1 | M1b+R1 |
|---|---:|---:|
| active_samples | 797 | 800 |
| robust_margin_minimum | 0.5721032745435728 | 0.6093667116630813 |
| robust_margin_mean | 0.9590272525314307 | 0.9573842096626944 |
| wheel_margin_minimum | 0.5721032745435728 | 0.6093667116630813 |
| wheel_margin_mean | 0.9614436152149953 | 0.9609964977100072 |
| wheel_margin_p05 | 0.6618458525975635 | 0.6939113414071904 |
| wheel_margin_dominant_fraction | 0.0451693851944793 | 0.05375 |
| risk_factor_peak | 0.1803197127102478 | 0.15636057899345612 |
| risk_factor_mean | 0.015157052433436949 | 0.014846530151977882 |
| risk_signal_peak | 0.1713037270747354 | 0.1485425500437833 |
| risk_signal_mean | 0.014399199811765118 | 0.014104203644379 |
| risk_contraction_peak | 0.008002152754006828 | 0.0 |
| risk_contraction_integral | 0.017133930043861687 | 0.0 |
| common_reference_velocity_minimum_mps | 0.09899784724599317 | 0.0997869491535643 |
| common_reference_velocity_mean_mps | 0.10024063964168674 | 0.09985341371809907 |
| robot2_actual_velocity_mean_mps | 0.10036803709882824 | 0.0999948421098671 |
| controller_wheel_raw_peak_mps | 0.15427896725456428 | 0.1539063328833692 |
| controller_wheel_raw_rms_mps | 0.12408910345976343 | 0.12364965995048925 |
| controller_differential_wheel_peak_mps | 0.060560040769227955 | 0.06052041305827452 |
| controller_differential_wheel_rms_mps | 0.034145722960697386 | 0.03412154680838947 |
| wheel_demand_capability_ratio_peak | 0.9642435453410267 | 0.9619145805210574 |
| speed_limiter_duration_seconds | 0.0 | 0.0 |
| hard_capability_margin_minimum_mps | 0.012086277497978193 | 0.013128862971715838 |
| mapped_capability_minimum_mps | 0.12042911075700145 | 0.12042875838825415 |
| mapped_capability_maximum_mps | 0.12046881846126042 | 0.12046881843919899 |
| capability_report_constancy_range_mps | 0.0 | 0.0 |
| derating_active_samples | 0 | 0 |
| disturbance_amplitude_minimum_mps | 0.02 | 0.02 |
| disturbance_amplitude_maximum_mps | 0.02 | 0.02 |
| robot2_heading_error_rmse_rad | 0.12143850509217094 | 0.12112724897534055 |
| robot2_heading_error_max_rad | 0.18953697863749183 | 0.18962178952761155 |
| robot2_support_position_rmse_m | 0.006100916068915033 | 0.006099635773232211 |
| robot2_support_position_max_m | 0.02270829117597019 | 0.02271916691753312 |
| rigid_fit_residual_rmse_m | 0.0023411626892130316 | 0.002345259339494021 |
| rigid_fit_residual_max_m | 0.009552192761402241 | 0.009555006334648468 |
| pairwise_side_error_rmse_m | 0.008475623758040425 | 0.008477753586872146 |
| pairwise_side_error_max_m | 0.01896104448514363 | 0.01901090944742173 |
| equivalent_load_position_rmse_m | 0.003542745013736853 | 0.0034600391393761463 |
| equivalent_load_yaw_rmse_rad | 0.0025682010035491887 | 0.0028555948651154354 |
| robot2_progress_rmse_m | 0.002858238987401653 | 0.0027355593838672802 |
| robot2_velocity_rmse_mps | 0.009429105999241813 | 0.008368057291767199 |
| task_completion_time_seconds | 51.62989330291748 | 51.7000949382782 |
| risk_boundary_active_fraction | 0.06148055207026349 | 0.0 |
| risk_boundary_active_duration_seconds | 0.49 | 0.0 |
| manifest_nominal_common_velocity_mps | 0.1 | 0.1 |
| actual_reference_reduction_peak_mps | 0.0005147806607340016 | 3.6156668693498872e-06 |
| actual_reference_reduction_mean_mps | 1.9180231298044258e-05 | -1.8892647552287353e-07 |
| actual_reference_reduction_positive_fraction | 0.27352572145545795 | 0.07 |
| upper_clipping_reduction_peak_mps | 0.0005147806607340016 | 0.0 |
| upper_clipping_reduction_mean_mps | 1.979658081201286e-05 | 0.0 |
| task_velocity_shortfall_peak_mps | 0.0010021527540068353 | 0.0002130508464357056 |
| task_velocity_shortfall_mean_mps | 4.9690365344869206e-05 | 0.00014658628190084006 |
| perturbed_wheel_physical_margin_minimum_mps | 0.017553439133736914 | 0.019017214630484558 |
| perturbed_wheel_physical_margin_mean_mps | 0.027184997643398212 | 0.027659239575625216 |
| perturbed_differential_wheel_rms_mps | 0.020972167725292264 | 0.020910938012470074 |
| risk_boundary_analysis_samples | 797 | 800 |

## Actual reference reduction

Same-run candidate minus published reference peak/mean: 0.0005147806607340016 / 1.9180231298044258e-05 m/s

## Causal questions

- A_no_disturbance_m1_maintained_task_velocity: True
- B_risk_boundary_became_active: True
- C_m1_produced_actual_reference_reduction: True
- D_controller_raw_peak_reduced: False
- D_controller_differential_rms_reduced: False
- E_minimum_wheel_margin_improved: False
- F_heading_rmse_improved: False
- F_support_rmse_improved: False
- F_rigid_fit_rmse_improved: True

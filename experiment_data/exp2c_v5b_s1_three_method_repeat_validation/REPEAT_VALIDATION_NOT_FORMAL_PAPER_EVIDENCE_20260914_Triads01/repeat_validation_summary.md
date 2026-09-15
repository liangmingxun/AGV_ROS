# REPEAT_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE

M1_ABLATION_REPEATABILITY_CONFIRMED

M1_VS_M2B_MIXED_RESULT

Frozen S1, M1 upper risk gain 0.10. No parameter search. Five triads only; all unfavorable runs retained.

| Comparison | Metric | Mean improvement % | SD | 95% CI |
| --- | --- | ---: | ---: | --- |
| M1 vs M1b | lateral_error_IAE | 7.807162824406539 | 0.9166254378676328 | [6.669022018368536, 8.945303630444542] |
| M1 vs M1b | heading_error_IAE | 6.434273980315786 | 0.7323159127712523 | [5.524983676115992, 7.34356428451558] |
| M1 vs M1b | support_error_IAE | 7.518978329253924 | 0.8223260758786153 | [6.497925646051269, 8.54003101245658] |
| M1 vs M1b | rigid_fit_error_IAE | 8.289214827590856 | 0.7330476693891814 | [7.379015927543132, 9.199413727638579] |
| M1 vs M1b | pairwise_side_error_IAE | 7.891959437763689 | 0.9779009511388529 | [6.677735032925355, 9.106183842602023] |
| M1 vs M1b | progress_error_IAE | 12.700838175819365 | 2.041945125092619 | [10.165428427684173, 15.236247923954558] |
| M1 vs M1b | J_risk_0p5 | 86.1001118475034 | 3.305618639738003 | [81.99564419074942, 90.20457950425738] |
| M1 vs M1b | J_risk_0p7 | 77.26202598111902 | 4.630204708803948 | [71.5128674898836, 83.01118447235444] |
| M1 vs M1b | wheel_margin_below_0p5_duration_seconds | 65.4404728887022 | 7.123358611054098 | [56.59565495506154, 74.28529082234286] |
| M1 vs M1b | controller_raw_above_0p160_duration_seconds | 99.89045760218518 | 0.24494424793227565 | [99.58631914796058, 100.19459605640978] |
| M1 vs M1b | physical_speed_limiter_duration_seconds | 100.0 | 0.0 | [100.0, 100.0] |
| M1 vs M1b | task_time_seconds | -0.8968031350530357 | 0.20529725768897938 | [-1.1517133488554028, -0.6418929212506685] |
| M1 vs M2b | lateral_error_IAE | 7.339004805780207 | 0.929221487511235 | [6.1852239383801555, 8.492785673180258] |
| M1 vs M2b | heading_error_IAE | 6.108773187510548 | 0.7749392176237029 | [5.146559060191002, 7.070987314830093] |
| M1 vs M2b | support_error_IAE | 7.117462447524811 | 0.8067939684637053 | [6.115695422915483, 8.119229472134139] |
| M1 vs M2b | rigid_fit_error_IAE | 8.233669099023324 | 0.5931951334793347 | [7.497120057872356, 8.97021814017429] |
| M1 vs M2b | pairwise_side_error_IAE | 7.786056313873 | 0.7855392472635608 | [6.81068051136975, 8.76143211637625] |
| M1 vs M2b | progress_error_IAE | 14.819455546379542 | 1.4044876073404773 | [13.075553848421524, 16.56335724433756] |
| M1 vs M2b | J_risk_0p5 | 86.18468131705825 | 3.3270256669647207 | [82.05363332528827, 90.31572930882822] |
| M1 vs M2b | J_risk_0p7 | 77.3739915314636 | 4.515479071432798 | [71.76728373382292, 82.98069932910428] |
| M1 vs M2b | wheel_margin_below_0p5_duration_seconds | 65.49700644107219 | 6.756857679779315 | [57.10725951910358, 73.8867533630408] |
| M1 vs M2b | controller_raw_above_0p160_duration_seconds | 99.89859919919333 | 0.2267390835766415 | [99.6170654421305, 100.18013295625616] |
| M1 vs M2b | physical_speed_limiter_duration_seconds | 100.0 | 0.0 | [100.0, 100.0] |
| M1 vs M2b | task_time_seconds | -0.7208403667209058 | 0.16629373041245182 | [-0.92732130490105, -0.5143594285407616] |

Zero-baseline exposure has N/A percentage, not zero; absolute paired differences remain available.
Student-t CIs describe mean paired effects across fresh fake runs; scheduler variation is not independent physical-world evidence.
M2b internal risk boundary/robust margin/reference reduction are N/A, not fabricated zeros.
All source/native diagnostics, signed geometry, measured rigid fit, censoring and startup limiter exposure are retained per run.

# Exp2c-v5b S1 three-method repeat-validation results

Evidence scope: `REPEAT_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE`  
Experiment identity: `exp2c_v5b_s1_three_method_repeat_validation`  
Acquisition HEAD: `f418fa7264b1f351a2dacf76c3d872e6825ff781`

## Result

Five independent fresh triads (15 valid runs) completed. The frozen S1 result is:

- `M1_ABLATION_REPEATABILITY_CONFIRMED` for M1+R1 versus strict M1b+R1.
- `M1_VS_M2B_MIXED_RESULT` for M1+R1 versus the complete M2b literature method. M1 improved every selected geometry and execution-resource pair, while adding about 0.37 s (0.72%) mean task time versus M2b.
- This is repeat-validation evidence only. It is not formal paper evidence.

All primary-window quantities below use the preregistered `[0,18.7)` s window unless explicitly marked as full-recording or full-task. IAE values are integrals in the metric's native units over that window.

## Frozen identity and implementation audit (items 1--9)

1. The current/acquisition HEAD is `f418fa7264b1f351a2dacf76c3d872e6825ff781`.
2. The worktree started clean. It now contains only the declared repeat-validation implementation, analysis, runner, test, and documentation changes; the result tree is ignored experiment data and retained locally.
3. S1 was unchanged in every run: `gamma_hold=0.20`, `T_down=0.6 s`, `T_hold=2.5 s`, `T_up=0.6 s`, duration `3.7 s`, trigger `2.0 m`, task speed `0.100 m/s`, clockwise smooth-entry/exit `R=0.7 m`, and the frozen 30 cm equilateral support geometry.
4. M1 `risk_gain_upper` remained `[0.10, 0.10, 0.10]` in every generated run configuration.
5. The complete literature implementation remains in `m2b_controller.cpp`/`m2b_controller.hpp`, with frozen parameters copied from `exp2b_M2b.yaml` and the existing M2b runtime.
6. It is reported as `M2b_COMPLETE`, with wire method ID `M2b_M2b`; it is not called M2b+R1.
7. Static/runtime inspection confirms the M2b branch calls its independent `M2bController::step()` and returns before the M1/M1b/M2a upper-reference path.
8. No M2b controller law or frozen M2b parameter was modified. A zero diff was verified for the controller sources and frozen `exp2b_M2b.yaml`.
9. The shared S1 transform is applied after each method's native controller raw wheel command and before the common limiter/fake plant. M2b native raw demand remains separately recorded. The same external capability and execution semantics are used for all methods.

## Acquisition and completion (items 10--12)

| Triad | Recorded order | M1 | M1b | M2b complete |
|---|---|---:|---:|---:|
| 01 | M1b, M1, M2b | valid/completed | valid/completed | valid/completed |
| 02 | M2b, M1b, M1 | valid/completed | valid/completed | valid/completed |
| 03 | M1, M2b, M1b | valid/completed | valid/completed | valid/completed |
| 04 | M1b, M2b, M1 | valid/completed | valid/completed | valid/completed |
| 05 | M2b, M1, M1b | valid/completed | valid/completed | valid/completed |

Each method invocation created a fresh private ROS master, algorithm/controller, fake plant, estimator, tracker, and disturbance state. A separate M2b smoke run passed before the triads and is excluded from the statistics. A pre-motion recorder path-resolution failure during a resumed Triad02 attempt is retained under `M2b_COMPLETE_preflight_failed_relative_path`; it never moved and is excluded.

## M1 versus M1b ablation (items 13--20)

Positive percentage means the lower-is-better metric improved with M1. The five per-triad percentages are shown in acquisition order.

| Metric | Triad01..05 improvement (%) | Mean ± SD (%) | 95% t CI (%) |
|---|---|---:|---:|
| Robot2 lateral IAE | 8.36, 6.58, 8.84, 8.08, 7.17 | 7.81 ± 0.92 | [6.67, 8.95] |
| Robot2 heading IAE | 7.03, 5.37, 7.07, 6.69, 6.01 | 6.43 ± 0.73 | [5.52, 7.34] |
| support IAE | 7.87, 6.38, 8.51, 7.79, 7.05 | 7.52 ± 0.82 | [6.50, 8.54] |
| rigid-fit IAE | 8.36, 7.65, 9.43, 8.38, 7.63 | 8.29 ± 0.73 | [7.38, 9.20] |
| pairwise-side IAE | 8.33, 7.01, 9.18, 8.12, 6.82 | 7.89 ± 0.98 | [6.68, 9.11] |
| progress IAE | 10.57, 13.02, 15.08, 10.65, 14.19 | 12.70 ± 2.04 | [10.17, 15.24] |
| `J_risk_0p5` | 83.54, 84.54, 91.06, 87.89, 83.48 | 86.10 ± 3.31 | [82.00, 90.20] |
| margin `<0.5` duration | 67.52, 56.10, 74.53, 68.23, 60.83 | 65.44 ± 7.12 | [56.60, 74.29] |
| raw `>0.160` duration | 99.45, 100, 100, 100, 100 | 99.89 ± 0.24 | [99.59, 100.19] |
| primary limiter duration | 100, 100, 100, 100, 100 | 100 ± 0 | [100, 100] |
| task time | -1.01, -0.66, -0.95, -0.72, -1.15 | -0.90 ± 0.21 | [-1.15, -0.64] |

Absolute primary means were: lateral IAE `0.26955` versus `0.29238`, heading `0.50431` versus `0.53899`, support `0.27599` versus `0.29843`, rigid-fit `0.11129` versus `0.12135`, `J_risk_0p5` `0.16241` versus `1.16809`, margin `<0.5` duration `0.909 s` versus `2.630 s`, raw `>0.160` duration `0.002 s` versus `1.987 s`, limiter duration `0` versus `0.598 s`, and task time `52.069 s` versus `51.606 s` (M1 versus M1b).

## M1 versus complete M2b (items 21--28)

| Metric | Triad01..05 M1 comparative improvement (%) | Mean ± SD (%) | 95% t CI (%) |
|---|---|---:|---:|
| Robot2 lateral IAE | 7.90, 6.37, 8.30, 7.81, 6.31 | 7.34 ± 0.93 | [6.19, 8.49] |
| Robot2 heading IAE | 6.50, 5.32, 6.82, 6.68, 5.22 | 6.11 ± 0.77 | [5.15, 7.07] |
| support IAE | 7.40, 6.33, 8.03, 7.61, 6.21 | 7.12 ± 0.81 | [6.12, 8.12] |
| rigid-fit IAE | 8.33, 7.77, 9.16, 8.24, 7.66 | 8.23 ± 0.59 | [7.50, 8.97] |
| pairwise-side IAE | 7.81, 7.26, 8.91, 8.09, 6.87 | 7.79 ± 0.79 | [6.81, 8.76] |
| progress IAE | 14.64, 15.39, 15.42, 12.50, 16.15 | 14.82 ± 1.40 | [13.08, 16.56] |
| `J_risk_0p5` | 83.68, 85.22, 90.95, 88.15, 82.94 | 86.18 ± 3.33 | [82.05, 90.32] |
| margin `<0.5` duration | 67.16, 57.41, 74.13, 68.71, 60.06 | 65.50 ± 6.76 | [57.11, 73.89] |
| raw `>0.160` duration | 99.49, 100, 100, 100, 100 | 99.90 ± 0.23 | [99.62, 100.18] |
| primary limiter duration | 100, 100, 100, 100, 100 | 100 ± 0 | [100, 100] |
| task time | -1.01, -0.62, -0.66, -0.62, -0.70 | -0.72 ± 0.17 | [-0.93, -0.51] |

Absolute M2b means were: lateral IAE `0.29090`, heading `0.53712`, support `0.29714`, rigid-fit `0.12127`, pairwise-side `0.24047`, progress `0.06197`, `J_risk_0p5=1.17485`, margin `<0.5` duration `2.630 s`, raw `>0.160` duration `1.996 s`, primary limiter `0.612 s`, and task time `51.696 s`. M1 means are listed above. The paired mean task-time cost was `+0.373 ± 0.086 s`, 95% CI `[+0.266,+0.479] s`.

## Run-level interpretation (items 29--35)

29. No negative or direction-reversing pair occurred for the selected geometry/resource metrics. Triad02 and Triad05 had the smallest geometry effects; they remain valid and were not removed. Task time was consistently worse for M1, which is why the literature comparison is classified as mixed rather than silently promoted to an unqualified advantage.
30. In Triad01--05 order, primary-window M1 raw `>0.160` was `0.0106, 0, 0, 0, 0 s`, with mean `0.0021 s`; M1b mean was `1.987 s`; M2b mean was `1.996 s`.
31. Primary-window limiter duration was `0 s` in all five M1 runs, `0.598 ± 0.018 s` for M1b, and `0.612 ± 0.018 s` for M2b. Full-recording M1 still has a mean `0.167 s` limiter event, and all five were explicitly identified as occurring before S1 trigger (startup); M1b/M2b full-recording means were about `0.804/0.808 s`, including both startup and S1/recovery exposure. Therefore the defensible result is elimination of disturbance/recovery limiter exposure, not elimination of every startup limiter sample.
32. The M1 causal mechanism activated in all five runs: mean risk-contraction peak `0.02586 m/s`, integral `0.06592 m`, active duration `3.522 s`, active fraction `0.1883`, actual-reference-reduction peak `0.00725 m/s`, mean `0.000472 m/s`, and positive fraction `0.2268`. Published common-reference minimum averaged `0.08114 m/s`. These are M1-only mechanism metrics.
33. Every M2b run contained non-empty native M2b state diagnostics (62 named diagnostic channels in the smoke audit), remained finite/valid, and completed. M1 robust-margin/risk fields are `N/A` for M2b, not zero. M2b-specific fields are not mapped onto M1 semantics.
34. The ablation decision is `M1_ABLATION_REPEATABILITY_CONFIRMED`: all five runs completed, the mechanism activated five out of five, every selected geometry pair improved by more than 5%, and resource stress fell sharply without a new instability.
35. The literature decision is `M1_VS_M2B_MIXED_RESULT`: M1 consistently improved tracking/formation quality, risk exposure, raw-wheel burden, and S1 limiter exposure, but took about 0.72% longer. This preserves the explicitly required multi-objective interpretation.

## Recommendation (items 36--37)

36. Keep frozen S1 as a candidate for a later, separately authorized fresh formal acquisition. Do not relabel this dataset as formal evidence and do not tune S1 or M1 from these repeats.
37. Keep the complete M2b implementation as the formal literature comparator: its integration is stable, repeat variability is low, method-specific diagnostics are complete, and its task-time advantage is a meaningful tradeoff to report.

## Verification and repository state (items 38--41)

38. Affected package build passed, including the formal fake/algorithm nodes, estimator, M2b controller target, upper-reference target, and runtime-gate targets.
39. Passed: M2b controller `5/5`; yaw hold degradation `15/15`; upper reference `14/14`; quality/runtime gate `4/4`; conversion/metrics `62/62`; bringup static `49/49`; three-method analyzer `7/7`; Python compilation and shell syntax. The unchanged historical tracker fixture remains documented as the known `10/11` result and was not modified or hidden.
40. `git diff --check` passes.
41. `git status --short` lists the declared modified/new implementation files and this documentation; no automatic commit or push was performed. Historical v4/v5/v5b data and formal/serial authorization semantics were not changed.

## Evidence files

The complete machine-readable results are under:

`experiment_data/exp2c_v5b_s1_three_method_repeat_validation/REPEAT_VALIDATION_NOT_FORMAL_PAPER_EVIDENCE_20260914_Triads01`

Use `repeat_validation_summary.json` for the full nested statistics; `method_absolute_statistics.csv`, `M1_vs_M1b_pairwise_improvements.csv`, `M1_vs_M2b_pairwise_comparison.csv`, `summary_M1_vs_M1b.csv`, and `summary_M1_vs_M2b.csv` for tabular review. Raw, unsmoothed analysis plots are in `analysis_plots/`.

# Exp2c-v5 Y0 complete quality-observation pair

EFFECT_EXPLORATION_ONLY / NOT_FORMAL_PAPER_EVIDENCE

HEAD: `06badcf2bbc758007d2089ae6c4fe651052c51d2`.
Branch: `fix/platform-foundation-closeout`. No commit/push/serial.

## Scope and stop policy

See [full stop-path audit](exp2c-v5-quality-policy-audit.md). Only original Y0
gamma_min=0.12, T=3.5 s, trigger=2.0 m was run: fresh M1b first, fresh M1 second.
No Y1/Y2, no duration/amplitude search. Hardware authorizations remain false.

The unchanged 10 mm threshold is a measured quality flag in explicit v5 fake
observation, not a stand-alone fatal failure. Finite tracking/formation errors,
raw envelope exceedance, wheel/robust margin zero, limiter exposure and censored
recovery are observations. Numerical invalidity, unavailable core states,
non-computable geometry and fixed 95 s task timeout remain fatal. Old v4,
formal/serial/Stage-D safety and historical Y0 results are retained.

## Evidence

New root:
`experiment_data/exp2c_v5_yaw_effectiveness_exploration/EFFECT_EXPLORATION_ONLY_NOT_FORMAL_PAPER_EVIDENCE_20260914_Y0_quality_observation_v2`.

Under `Y0_gamma0p12_T3p5`:

- `M1b_R1/` and `M1_R1/`: bag, original configuration/parameter/manifest snapshots,
  converted CSV, standard wheel-chain CSV, validation and metrics.
- `effect_comparison.json`: complete phase/full-task statistics, all three native
  tracking errors, each pairwise side, true estimator quality flags and first
  exceedance context, coverage and causal checks.
- `effect_comparison.md`, `effect_metrics.csv`, `quality_exposure.csv`.

The original hard-stopped Y0 root and all v4 screening/exploration evidence are
unchanged. New configs are generated under this new root only; the frozen
`localization_odom_exp2c_fake.yaml` still contains 0.010 m.

## Task and validity

| Metric | M1b+R1 | M1+R1 |
| --- | ---: | ---: |
| Complete 5.178229715 m reference task | true | true |
| Generic postprocessing validation | passed | passed |
| Motion fail-zero / emergency / structural failure | none | none |
| Complete 3.5 s pulse +15 s recovery | yes | yes |
| Recorded-start to reference completion / s | 51.7501 | 51.9901 |

Time increases 0.2400 s, approximately 0.464%. Task time uses existing
recorded-start semantics; valid-control diagnostic duration is a different
quantity and must not be substituted for task completion time.

## Geometry and native tracking

Unless stated otherwise, statistics below use the entire fixed 18.5 s
pulse+recovery window. Position errors use mm, RMS mm, IAE mm*s. Native heading
uses rad, RMS rad, IAE rad*s. Support/progress/native rows refer to Robot2;
pairwise side is the maximum absolute deviation of all three sides each tick.

| Error | M1b peak / RMS / IAE | M1 peak / RMS / IAE |
| --- | ---: | ---: |
| True measured rigid-fit | 10.9028 / 4.0961 / 59.6699 | 10.7247 / 3.9650 / 57.5913 |
| Support | 25.3459 / 10.2961 / 159.9423 | 24.3726 / 9.9891 / 154.8049 |
| Native lateral | 19.2998 / 9.8774 / 157.5363 | 18.5498 / 9.5320 / 152.4173 |
| Native longitudinal | 4.7519 / 1.4009 / 19.4101 | 4.4460 / 1.2692 / 18.1120 |
| Native heading / rad | 0.113179 / 0.027255 / 0.269099 | 0.112246 / 0.026636 / 0.260205 |
| Pairwise maximum side | 20.6476 / 8.1414 / 118.8928 | 20.3370 / 7.8882 / 114.9057 |
| Progress | 7.6714 / 2.2002 / 30.9332 | 7.1548 / 1.9979 / 29.0993 |

| True estimator 10 mm exposure | M1b | M1 |
| --- | ---: | ---: |
| Exceeded flag | true | true |
| First exceedance after pulse trigger / s | 2.25002 | 2.25349 |
| First measured value / mm | 10.02557 | 10.01737 |
| Duration >10 mm / s | 0.560169 | 0.479998 |
| Full valid-control rigid-fit RMS / mm | 2.49264 | 2.38340 |
| Full valid-control rigid-fit IAE / mm*s | 71.1201 | 68.3674 |

Duration >10 mm decreases about 14.3%; peak decreases only about 1.6%.
Fixed-window geometry IAE improves 3.21% support, 3.25% lateral, 3.31% heading,
3.49% rigid-fit. These are modest single-pair trends, not statistical significance.
Not every metric improves: Robot1 lateral IAE and d13 IAE slightly increase.
All three robots and all three side statistics are retained in JSON/CSV.

No standalone existing hard quality thresholds were found for native heading,
lateral, longitudinal, support norm, side deviation or progress error. Their
exposure thresholds/durations are null, not invented. Recovery thresholds are
separate pre-trigger median/MAD analysis envelopes, not quality/fatal gates.

## Wheel risk and execution burden

Pulse+15 s recovery, Robot2 controller pre-degradation raw unless indicated.

| Metric | M1b | M1 |
| --- | ---: | ---: |
| Wheel margin min / p05 / mean | 0 / 0.44655 / 0.94838 | 0 / 1 / 0.97674 |
| Robust margin min / p05 / mean | 0 / 0.51636 / 0.94895 | 0.00272 / 1 / 0.97743 |
| Pulse-only wheel min / p05 / mean | 0 / 0 / 0.72695 | 0 / 0.09973 / 0.87739 |
| Pulse-only robust min / p05 / mean | 0 / 0 / 0.72997 | 0.00272 / 0.09803 / 0.88103 |
| Wheel margin <0.5 duration / s | 0.95004 | 0.45073 |
| Wheel margin <0.7 duration / s | 1.07013 | 0.55033 |
| J_risk_0p5 | 0.410617 | 0.136622 |
| J_risk_0p7 | 0.612616 | 0.238123 |
| Controller raw peak / m/s | 0.167409 | 0.160421 |
| Controller raw RMS / m/s | 0.123100 | 0.121999 |
| Raw >0.150 duration / s | 1.22017 | 0.73012 |
| Raw >0.155 duration / s | 0.94999 | 0.44969 |
| Raw >0.160 duration / s | 0.66982 | 0.01919 |
| First raw >0.160 after trigger / s | 2.27003 | 2.35040 |
| Degraded / applied peak / m/s | 0.157385 | 0.136139 |
| Pulse+recovery actual speed limiter duration / s | 0 | 0 |

J_risk_0p5 decreases 66.7%, J_risk_0p7 61.1%; raw>0.160 exposure decreases
97.1%. Wheel margin still briefly reaches zero under M1; it is not misreported
as fully recovered at every instant. Whole-window p05=1 does not imply the
pulse was risk-free; pulse-only quantiles are shown separately.

## Mechanism and recovery

| Metric | M1b | M1 |
| --- | ---: | ---: |
| Risk contraction peak / m/s | 0 | 0.0242061 |
| Risk contraction integral / m | 0 | 0.0435977 |
| Risk boundary active duration / s | 0 | 2.17996 |
| Risk boundary active fraction | 0 | 0.117835 |
| Actual reference reduction peak / m/s | 0.000004975 numerical residual | 0.00755611 |
| Actual reference reduction mean / m/s | approximately 0 | 0.000254856 |
| Minimum published common reference / m/s | 0.0998079 | 0.0827939 |
| Native heading recovery after pulse end / s | 5.03016 | 4.69011 |
| Native lateral recovery after pulse end / s | 14.12945 | 13.85978 |
| Native longitudinal recovery / s | 0.98023 | 0.55986 |
| Support recovery / s | 13.68975 | 12.19989 |
| Rigid-fit recovery / s | 11.11982 | 10.67003 |

Actual reference reduction means same-run candidate common velocity minus
published effective common velocity. Do not confuse 0.100-minimum reference
with this quantity, or call nominal inner upper minus reference a reduction.
All recovery metrics recover within the fixed 15 s horizon in this pair.

## Startup / recording coverage limitations

Standard whole-recording wheel-chain data, including startup, must not be
replaced by the filtered v5 diagnostic subset:

| Whole recorded execution | M1b | M1 |
| --- | ---: | ---: |
| All-car raw peak / m/s | 0.167409 | 0.164389 |
| Robot2 raw peak / m/s | 0.167409 | 0.164097 |
| Robot2 total raw >0.160 / s | 0.90005 | 0.16023 |
| Actual limiter samples / duration s | 23 / 0.230165 | 16 / 0.159874 |
| All limiter events before yaw trigger | true | true |
| Missing standard raw while algorithm valid | 0 | 0 |
| First native v5 diagnostic after recording start / s | 0.99002 | 0.02649 |

M1b native startup telemetry is delayed; native full-task summaries therefore
cover available valid native diagnostics, not every startup instant. Standard
raw/limiter coverage is complete during valid control and separately exposes
the startup events. The fixed pre-trigger 3 s, full pulse and recovery windows
are complete. This limitation is explicitly retained, not converted into a
claim of zero full-run limiting.

## Conclusion

`YAW_EFFECTIVENESS_WORKING_POINT_FOUND` under the pre-existing exploratory
trend criterion. Active M1 risk feedback produces actual reference reduction,
large wheel-risk exposure reduction and modest tracking/formation IAE
improvement without relying on pulse/recovery physical limiting. Both tasks
complete and recover. Y0 is not shown to be overstrong or persistently
divergent; shortening to T=3.0 s is not needed from this evidence. No further
search was run. Repeatability and formal/hardware conclusions remain untested.

## Changed files and verification

This continuation edits/adds:

- `src/multi_agv_control/include/multi_agv_control/v5_exploration_policy.hpp`
- `src/multi_agv_control/src/path_state_estimator_node.cpp`
- `src/multi_agv_control/src/formal_fake_algorithm_node.cpp`
- `src/multi_agv_control/test/test_rigid_fit_runtime_gate.cpp`
- `src/multi_agv_control/CMakeLists.txt`
- `src/multi_agv_analysis/src/multi_agv_analysis/conversion.py`
- `src/multi_agv_analysis/scripts/analyze_exp2c_v5_yaw_effectiveness.py`
- `src/multi_agv_analysis/test/test_exp2c_v5_yaw_effectiveness.py`
- `src/multi_agv_bringup/scripts/run_exp2c_v5_yaw_effectiveness_fake.sh`
- This report, quality audit, and current-policy notice in the v5 protocol.

Existing v4/v5 uncommitted files remain present; they were not discarded or
committed. Generated results remain under ignored experiment_data, so their
absence from git status does not mean they were deleted or uploaded.

Verification: affected-package build passed after fixing a constructor field
lookup; runtime-gate/policy 4 tests, upper-reference 14 tests (including zero
causal margins), hardware execution gates 25 tests, yaw-effectiveness model
5 tests, conversion/metrics 61 tests, bringup static 49 tests, updated v5
analysis 8 tests passed. Python compilation and shell syntax passed. Actual
fresh fake M1b/M1 pair passed. `git diff --check` clean. Private owned ROS
processes shut down normally.

Known pre-existing unrelated failure from the earlier verification remains:
`PlanarSupportTracker.FrozenEquilateralFixtureMatchesStartTransformsAndWheelBound`.
Tracker implementation/test and geometry were not changed to conceal it.
No full suite or physical test is claimed.

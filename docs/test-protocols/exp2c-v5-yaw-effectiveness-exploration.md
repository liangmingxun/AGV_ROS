# EFFECT_EXPLORATION_ONLY / NOT_FORMAL_PAPER_EVIDENCE

Current continuation supersedes the original exploration stop policy below:
[v5 quality-policy audit](exp2c-v5-quality-policy-audit.md). Only fresh original
Y0 is run; finite quality exceedance and limiter exposure are observations.
The original Y0 hard-stop results below remain historical evidence, unchanged.

Independent Exp2c-v5 yaw effectiveness exploration; preserve all earlier v1–v4
code semantics/configurations/evidence. Starting HEAD:
`06badcf2bbc758007d2089ae6c4fe651052c51d2`; previous v4 exploration changes
remain uncommitted. No hardware grant, serial, commit or push.

## Model and causality

Robot2 actual progress >=2 m latches one wall-clock trigger. With elapsed t,
duration T, u=clamp(2 min(t,T-t)/T,0,1), Q=10u^3-15u^4+6u^5:

    gamma = 1-(1-gamma_min)Q, 0<t<T; otherwise gamma=1
    vc = (left_raw+right_raw)/2
    delta = (right_raw-left_raw)/2
    left_degraded = vc-gamma*delta
    right_degraded = vc+gamma*delta

Implementation uses algebraically equivalent symmetric shifts for numerical
identity outside the window. No empirical CAL factor; gamma_min is the real
minimum yaw effectiveness. This is an abstract fake execution model, not a
diagnosis of one particular mechanical failure. Mean longitudinal command
is unchanged by injection; physical chassis limitations are separately logged.

Save controller-generated raw after tracker/execution adapter/startup blend;
modify only Robot2 execution copy before existing safety/physical limits.
Keep previous raw and CapabilityReport untouched. Upper M1 receives no gamma
preview/feedforward: only resulting actual-motion errors and causal raw margin.
The new function is disabled by default and structurally restricted to fake
M1/M1b+R1 and the exact v5 experiment identity. Legacy pulse combination is
rejected. No changes to gains, bounds, path, geometry, calibration or gates.

## Native tracking diagnostics

Topic `/multi_agv/yaw_effectiveness_state` with schema
`yaw_effectiveness_v5:header19+3x3`: ROS source stamp, wall time, trigger wall,
triggered/active/finished, elapsed, actual progress, gamma_min, T, Q, gamma,
raw L/R, degraded L/R, mean injection, differential injection, previous-raw
wheel margin, then native longitudinal/lateral/heading errors for each car.
The errors are copied directly from `PlanarTrackingResult`, never recomputed
from support tangent yaw. CSV has separate v5 fields and agvN_tracker errors;
public command/feedback exports supply applied/actual wheel speeds.

## Ordered exploration and reporting

Y0=.12/3.5 s, Y1=.10/3.5 s, Y2=.08/3.5 s. Each runs fresh M1b then M1,
without formal screening. No gamma below .08, no duration search beyond 3.5.
Success requires real contraction/active boundary/reference reduction, clear
wheel resource benefit, and >=3% improvement in at least one geometry IAE
(lateral/heading preferred, support/rigid secondary). This is an exploratory
trend criterion, not formal significance or evidence of repeatability.
Stop at first working point, even if other geometric metrics regress.

Baseline is trigger-preceding 3 s; pulse and 15 s post-end recovery are reported
separately and combined. Native heading/lateral/longitudinal and support/rigid
recovery: baseline median +/- max(3 MAD,floor), continuous hold .5 s, fixed
floors heading .010 rad, lateral/longitudinal/support .002 m, rigid .001 m.
Unrecovered values are null. J_risk_.5/.7 integrate threshold deficits using
wall-clock weights. Actual reference reduction is own candidate minus own
published common reference, NOT nominal upper minus reference.

Hard stop: emergency, motion-time fail-zero/nonfinite/fake failure, incomplete
task, unchanged rigid-fit rejection, or continuous pulse/recovery physical
speed limiting >=.5 s. Early startup limiting/brief raw overshoot are not
disqualifying; record them separately. Hard-stopped runs are not valid benefits.

## Entry

```bash
bash src/multi_agv_bringup/scripts/run_exp2c_v5_yaw_effectiveness_fake.sh \
  --candidate Y0_gamma0p12_T3p5 --method pair --ros-port 11646 \
  --output-root /absolute/fresh/exp2c_v5/exploration
```

Use Y1/Y2 names only after insufficient Y0/Y1 effect. Never overwrite roots.
Each candidate produces `M1b_R1`, `M1_R1`, `effect_comparison.json`,
`effect_comparison.md`, `effect_metrics.csv`, plus raw bags/config snapshots,
validation and logs. Reports carry both evidence-scope markers; no formal
screening or selected-candidate file is modified.

## Actual Y0: hard-stopped M1b, no stronger search

Y0 recorded at private localhost master 11646. Existing
`localization_odom_exp2c_fake.yaml` has `maximum_rigid_fit_residual: 0.01`.
It was not changed. First runtime rejection was 0.010108 m against the
0.010000 m gate; motion subsequently held fail-zero and task was incomplete.
Under the user hard-stop rule, M1, Y1 and Y2 were not run. No gamma/T is
recommended for formal freezing; no M1 geometry advantage can be concluded.

Only 2.330 s of post-trigger valid diagnostic horizon is available, not a
completed 3.5 s degradation/recovery window. All table values below are
partial, cannot be compared to completed runs, and recovery times are null.

| Partial M1b Y0 metric | Value |
| --- | ---: |
| Robust margin min / p05 / mean | 0 / 0.164991 / 0.897143 |
| Wheel margin min / p05 / mean | 0 / 0.147811 / 0.892995 |
| Wheel margin below .5 / .7 duration, s | 0.239619 / 0.319864 |
| J_risk_.5 / J_risk_.7, s | 0.079679 / 0.136886 |
| Controller raw peak / RMS, m/s | 0.171369 / 0.121642 |
| Raw >.150 / .155 / .160 duration, s | 0.449894 / 0.249523 / 0.080106 |
| Degraded / applied peak, m/s | 0.142410 / 0.140797 |
| Pulse physical speed-limiter duration, s | 0 |
| Native lateral peak / IAE, m / m*s | 0.013133 / 0.010072 |
| Native heading peak / IAE, rad / rad*s | 0.122282 / 0.097626 |
| Support peak / IAE, m / m*s | 0.024036 / 0.019799 |
| Rigid-fit peak / IAE, m / m*s | 0.010441 / 0.007821 |

M1b risk contraction/active duration are zero by design, not a conclusion
about M1. Actual reference reduction peak is only 0.000004002 m/s.
Capability reports stay at .160 m/s. Observed gamma minimum
0.120000000000743 verifies no empirical multiplier; mean injection maximum
6.94e-18 m/s and differential identity error 1.74e-17 m/s verify the model.
Startup limiting totals .180243 s and is not the hard-stop cause.

Relevant compilation and new/model/converter/gate tests pass. One existing
`PlanarSupportTracker.FrozenEquilateralFixtureMatchesStartTransformsAndWheelBound`
test fails at frozen chassis start-position/yaw assertions for robots 2/3.
Its source and tracker implementation are identical to HEAD, and are outside
this task's no-tracker-change scope. Report this failure rather than silently
changing gains/geometry or claiming a fully green regression.

## This task's file inventory

New: `yaw_effectiveness_degradation.hpp`, `test_yaw_effectiveness_degradation.cpp`,
`analyze_exp2c_v5_yaw_effectiveness.py`, `test_exp2c_v5_yaw_effectiveness.py`,
`run_exp2c_v5_yaw_effectiveness_fake.sh`, and this protocol.
Updated: analysis/bringup/control CMakeLists, analysis conversion and metric
tests, and shared `formal_fake_algorithm_node.cpp` with disabled-by-default
v5 hooks only. Previous uncommitted v4 exploration files/changes are preserved.
Final git status: 8 modified tracked files and 10 new untracked files, including
that prior v4 work. No versioned old configuration or experiment was rewritten.
`git diff --check` passes; HEAD is unchanged; no commit/push/serial occurred.

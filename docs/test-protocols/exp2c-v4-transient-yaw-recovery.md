# Exp2c-v4 transient yaw recovery (fake only)

Independent experiment on branch `fix/platform-foundation-closeout`. Preserve
all v1/v2/v3 versioned configurations, evidence and reports. All hardware
authorizations are false; do not run serial, commit/push or tune parameters.

## Frozen pulse and control

Robot2's measured `CooperativeState.s_actual[1] >= 2.0 m` latches a single
`ros::WallTime` trigger. With elapsed wall time t and Q(u)=10u^3-15u^4+6u^5:

    d(t) = A Q(2 min(t,T-t)/T),  0 < t < T
    d(t) = 0,                  otherwise
    vL_disturbed = vL_controller_raw - d(t)
    vR_disturbed = vR_controller_raw + d(t)

The first half smoothly rises, second half smoothly falls, without a plateau,
opposite-sign pulse or retrigger. Value and first two derivatives are
continuous at entry, midpoint and exit. Completion depends only on elapsed
wall time, never remaining path distance, so M1 slowing cannot prolong it.

Only A=(0.024 m/s,1.5 s) and B=(0.022 m/s,2.0 s) are permitted by the C++
validator. Both use this exact envelope. Test A first in M1b screening; B only
after A safety/rigid-fit/speed-limiter invalidity. Infrastructure, weak
mechanism or unsuccessful recovery alone does not authorize automatic B or
parameter tuning. Freeze an admissible candidate, then record fresh M1b and
M1 runs without reusing screening evidence.

Task velocity 0.100; nominal/initial upper 0.112; inner margin 0.005; physical
wheel limit 0.160 m/s. M1 risk gains/decay, wheel-safe margin, hard capability
envelope, R1, tracker, calibration, geometry, path and all gates remain intact.

## Injection and margin provenance

In `formal_fake_algorithm_node.cpp`, after tracker/execution adapter and
startup blend, copy all three controller raw wheel pairs to
`wheel_demand_before_limit`. Modify only `execution_tracking[1]` with the
transient pulse, before the unchanged emergency/publication/chassis limiter
chain. Never overwrite controller-generated raw with injected raw.

The next-tick M1 wheel-risk input remains:

    clip(min(limitL-|previous_controller_rawL|,
             limitR-|previous_controller_rawR|)/wheel_safe_margin,0,1)

The new `transient_yaw_v4:header19` diagnostic separately records wall trigger,
elapsed, fixed A/T, envelope, nonnegative d, controller raw, disturbed raw,
zero longitudinal injection, differential injection and this exact causal
wheel margin. CapabilityReport is not modified. The recorder accepts an
optional `record_topics_config`; its legacy default is unchanged. v4 uses a
separate topic registry requiring the new diagnostic stream.

## Frozen analysis before any screening

Use the 3 seconds immediately preceding the trigger as baseline. For heading,
Robot2 support-position norm and rigid-fit residual, freeze per-run bounds
`median +/- max(3*MAD,floor)` using only this pre-trigger baseline. Floors:
heading=0.010 rad, support=0.002 m, rigid-fit=0.001 m. No result-dependent
threshold selection. After the pulse ends, observe a fixed 15 seconds.
Recovery time is first re-entry into the baseline bounds maintained at least
0.5 seconds; missing sample gaps above 0.03 seconds break the hold. Unrecovered
values are null/censored, never fabricated as zero. A complete observation
horizon and recovery of all three signals are required for screening.

Tables separately report baseline, pulse, recovery and pulse+recovery. Metrics
include exact wheel/robust min/p05/mean, durations/fractions below 1/.7/.5,
risk contraction, actual same-run candidate-minus-published-reference
reduction, active constraint, controller raw peak/RMS/>0.150 duration,
capability ratio, disturbed peak, physical speed-limiter time, and geometry
peak/RMS/IAE. Heading uses measured robot yaw minus the recorded support
tangent yaw, as in prior Exp2c metrics. Side error is the maximum absolute
three-pair length deviation per sample. Controller-aligned progress error
retains its separately recorded origin convention. Never require smaller
differential-wheel RMS as a success criterion.

Screening requires valid/complete task, unchanged .160 reports/no derating,
all-car controller raw <.160, no physical speed limiter, no emergency/rigid
gate rejection, proper one-shot waveform, causal wheel margin entering
the <1 risk region, and measured recovery. New runtime and manifest nominal
velocity must both be .100; all recorded chassis transports must be fake and
hardware flags false. A failed/inadmissible paired condition is reported, not
silently treated as effective or replaced with a different candidate.

## Run

    bash src/multi_agv_bringup/scripts/run_exp2c_v4_transient_yaw_recovery_fake.sh \
      --ros-port 11644 --output-root /absolute/nonexistent/v4-output

The script refuses an existing output root or an occupied localhost master.
It records original bags/config snapshots and CSVs under an independent v4
directory. Generic validation settings and historical evidence are untouched.
No new plots are produced (`--skip-plots`). Reports are `screen_A.json`
(and B only if authorized by A failure), `selected_candidate.json`, and
`paired_comparison.{json,md}` / `paired_metrics.csv`.

If contraction/reference reduction happens but execution risk and recovery
do not improve, stop this route and report it. No third candidate or further
nominal-upper/risk/disturbance adjustment is allowed.

## Executed screening, 2026-09-14

Baseline HEAD: `3b59c3fef02ab7b2c8e0b47f891becc5da519896`.
Evidence root: `experiment_data/exp2c_v4_transient_yaw_recovery/20260914_frozen_transient_v4`.
Both fresh M1b tasks completed and passed generic validation, but both failed
the stricter, predeclared v4 raw-wheel/physical-limiter admissibility rule.
No candidate was selected; no official paired run was authorized or executed.

| Metric | Screen A | Screen B |
| --- | ---: | ---: |
| Full-run all-car controller raw peak, m/s | 0.165594 | 0.165758 |
| Full-run physical speed-limiter samples | 21 | 20 |
| Robot2 pulse+15 s recovery raw peak, m/s | 0.161153 | 0.160091 |
| Disturbed wheel peak in that window, m/s | 0.159851 | 0.154123 |
| Raw above 0.150 duration, s | 0.599962 | 0.690409 |
| Wheel margin below 0.5 duration, s | 0.399710 | 0.469874 |
| Support-error peak, m | 0.015241 | 0.016034 |
| Rigid-fit peak, m | 0.006444 | 0.006712 |
| Pairwise side-error peak, m | 0.012783 | 0.012919 |
| Robot2 progress-error peak, m | 0.005969 | 0.005785 |
| Support recovery after pulse end, s | 8.159842 | 9.100195 |
| Rigid-fit recovery after pulse end, s | 6.510398 | 7.270002 |
| Heading-proxy recovery after pulse end, s | 0.109893 | 0.000080 |
| Task completion time, s | 51.629910 | 51.569951 |

The full-run raw peaks and physical speed limiting occur at startup,
Robot2 progress approximately 0.043–0.078 m, before any yaw injection.
There is no physical speed limiting in either pulse+recovery window, but
controller raw still exceeds the separately required strict 0.160 m/s bound.
Neither issue is hidden by removing startup samples or relaxing tolerances.
CapabilityReport stays at 0.160, derating count is zero, and neither run
shows emergency or rigid-fit gate rejection. M1b risk contraction/active
fraction are zero by design; its tiny candidate-minus-reference differences
(under 0.000005 m/s) are not evidence of M1 risk-triggered slowing.

Heading here is explicitly the robot-versus-support-tangent proxy, whose
nominal baseline is approximately 0.115 rad, not a reconstructed chassis
tracker heading error. Its recovery is baseline-relative; near-zero B
re-entry time does not imply the whole formation has recovered. Support and
rigid-fit times remain necessary. No M1 resource-release or recovery benefit,
and no paired task-time cost, can be concluded from these two M1b screens.
Further control/startup changes or candidate tuning require a new user task.

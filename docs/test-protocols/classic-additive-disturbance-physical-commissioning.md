# Classic additive disturbance Candidate A physical commissioning

Scope: `PHYSICAL_CLASSIC_ADDITIVE_DISTURBANCE_COMMISSIONING_PREPARATION`.
This protocol does not authorize Codex or an unattended process to move a
vehicle. Every physical run is initiated separately by the operator.

## Frozen profile and injection chain

Candidate A is applied only to Robot2 after the native planar tracker demand
has been copied for M1's one-step-delayed wheel-margin calculation:

`native/controller raw -> Candidate A -> 0.18 m/s emergency assessment ->`
`0.16 m/s algorithm publication limit -> chassis limiter -> Stage-D physical-`
`to-firmware command scale -> serial/STM32`.

The base profile is fixed at `Av=0.030 m/s`, `Aomega=0.350 rad/s`, both angular
frequencies `1.0 rad/s`, phase `pi/2`, trigger progress `2.0 m`, duration
`8.0 s`, and `0.5 s` quintic ramps. Qualification scales are exactly 0.25,
0.50, 0.75 and 1.00. Scale zero is reserved for baseline/dry-run equivalence.
Neither `CapabilityReport` nor the M1 risk input sees the disturbed demand.

## Authorization and progression

The physical profile is fail-closed until both
`classic_additive_physical.enabled` and
`classic_additive_physical.hardware_execution_authorized` are true in
`formal_serial_classic_additive_candidate_A_commissioning.yaml`. Method gates
are separate. Enabling one Candidate A boolean is insufficient. The shared
entry accepts M1+R1, M1b+R1 and the complete M2b+M2b method; M2b additionally
requires `m2b_physical_authorized` and its exact-scope method overlay. It is
never substituted into the shared R1 lower layer.

The initial `maximum_qualified_scale` is 0.25. It may be raised to 0.50 only
after both M1 and M1b pass 25%; likewise for 75% and 100%. Never skip a level,
never chain levels, and never exceed 100%. At each level run M1 first, review
it, then run M1b. A failure produces
`PHYSICAL_COMMISSIONING_LEVEL_FAILED` and ends that invocation.

## Run acceptance and stop classification

Immediately stop for the existing emergency abort, watchdog failure, stale
state/feedback/capability, invalid load state, vision loss at the existing
abort rule, sustained physical saturation at the existing abort rule, command
gate rejection, uncontrolled yaw, obvious formation divergence, load slip or
collision risk, or an operator emergency stop. Do not loosen a limit to retry.

Keep runtime safety failures separate from data-quality exceedances and
performance degradation. A paper-quality threshold such as rigid-fit above
0.01 m is not by itself a hardware abort. A completed capture remains
`REVIEW_REQUIRED` until the operator reviews the bag, validation, disturbance
profile, Robot2-only action, wheel chain, geometry, voltage and event log.

The bag records native wheel demand, disturbed demand, post-publication-limit
demand and actual feedback as separate fields. It also records lambda, actual
Av/Aomega, envelope, disturbances, trigger state/time and elapsed time.

## Qualification order

1. P0: build, tests, scale-zero dry run and fake regression; no serial command.
2. P1: disturbance-disabled M1 baseline, then disturbance-disabled M1b.
3. P2: 25% M1; after review, 25% M1b.
4. P3/P4/P5: repeat at 50/75/100 only after both methods pass the prior level.

The largest level passed by both methods becomes `lambda_safe_common`. If one
method fails 100%, use 75%; never increase beyond Candidate A.

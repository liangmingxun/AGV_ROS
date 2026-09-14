# Exp2c-v3: independent nominal risk headroom pilot

This is a fake-only experiment on top of Exp2c-v2. Never run serial with these
authorizations or overwrite v1/v2 evidence. No hardware grant is introduced.

Frozen control change: M1/M1b initial and nominal upper are 0.112 m/s instead
of 0.115 m/s. Inner margin remains 0.005 m/s, yielding 0.107 m/s. Task speed
remains 0.100 m/s. Robot2 yaw disturbance stays A=0.020 m/s on its measured
2.0–2.8 m progress window, with the existing 0.15 m transition and execution
chain. All physical envelopes, wheel limits, risk gains, R1, tracking,
calibration, path and safety gates remain unchanged.

Run `bash src/multi_agv_bringup/scripts/run_exp2c_v3_nominal_headroom_fake.sh`
with a free private `--ros-port` and a nonexistent `--output-root`. The script
starts its own localhost master. It runs fresh no-disturbance M1 then M1b,
checks valid moving samples at public progress 0.4–5.17 m, and stops if M1
mean reference is below 0.0995 m/s, reference below 0.099 exceeds 5%, or
active risk boundary exceeds 5%. Both recordings must be valid and their
manifest nominal velocity must equal 0.100 m/s. Only after passing does it
run fresh disturbed M1b then M1. Never automatically tune parameters.

## Diagnostics and definitions

Only experiment ID `exp2c_v3_nominal_headroom_fake` emits the extended
`formal_algorithm_state_v3:header10+3x29+reference4` diagnostic. The existing
97 values stay in their original positions; the tail contains the three
actual `UpperAgentInput.candidate_velocity` values and the upper generator's
post-intersection common velocity. Older diagnostic formats remain supported.

- `candidate_common_velocity`: mean of those three same-run pre-intersection
  inputs. These are the distributed state velocities supplied to the upper
  generator, not task velocity or another run's reference.
- `actual_reference_reduction`: candidate minus the recorded published common
  reference, preserving its sign. Peak/mean use valid disturbance-window
  samples. A negative value is not a reduction. Downstream distributed-reference
  evolution can differ from the upper generator's own clipped output.
- `upper_clipping_reduction`: positive part of candidate minus the upper
  generator's post-intersection output; distinguishes direct clipping from
  downstream reference evolution.
- `risk_boundary_active`: positive risk contraction, candidate above common
  upper, and both upper-generator and published effective output contacting that upper within
  0.0001 m/s. Fraction uses valid disturbance-window samples; duration is
  sample count times the recorded analysis sample period.
- `risk_contraction`: baseline common upper minus dynamic common upper,
  separate from reference reduction.
- `paired_reference_difference`: fresh M1b reference minus M1 reference,
  clipped at zero, on a shared time/progress grid. This counterfactual comparison
  is not the same-run actual reference reduction.

Wheel burden follows v2's **pre-disturbance controller raw** nominal wheel
commands. Perturbed demand and remaining physical margin are reported
separately. Heading/side metrics cover the disturbance window; summary
support/rigid-fit metrics cover the full valid run. Evidence is fake-only,
one run per condition, not proof of hardware effectiveness or repeatability.

The recorder launch's legacy nominal velocity default is 0.08 m/s. This
runner explicitly supplies `nominal_common_velocity:=0.10`, and the analyzer
checks the resulting new manifests. Historical manifests are not rewritten.
It also checks the recorded runtime leader velocity/experiment identity, all
three actual fake transport bindings, and recursively rejects any non-false
hardware execution authorization in the new parameter snapshots.

## Completed fresh pilot

Evidence root: `experiment_data/exp2c_v3_nominal_headroom_fake/20260914_completed_headroom_v3`.
All four runs completed and passed validation. Stage 1 passed: M1/M1b steady
reference means were 0.100032930 / 0.099978236 m/s; neither had active upper
clipping or samples below 0.099 m/s.

At A=0.020, M1 active-boundary fraction was 0.06148055 (0.49 s). Risk contraction
peaked at 0.008002153 m/s. Same-run reference reduction peaked at 0.000514781
m/s and averaged 0.0000191802 m/s over the disturbance window. However, M1's
window mean reference was 0.100240640 m/s versus M1b's 0.099853414 m/s: this
was only a short limiting episode, not sustained reference slowing.

The complete beneficial causal chain is **not established** in this pair.
M1 controller wheel peak was 0.154278967 versus 0.153906333 m/s; differential
RMS was 0.034145723 versus 0.034121547 m/s. Minimum normalized wheel margin
was 0.572103 versus 0.609367. Heading/support did not improve; rigid-fit RMS
changed only from 2.345259 to 2.341163 mm. Completion time changed from
51.700095 to 51.629893 s (-0.070202 s), not an established time advantage.
No further upper/disturbance tuning was performed.

Verification: affected packages/dependencies built; 8 v3, 59 conversion/metrics,
49 bringup static and 7 v2 screening Python tests passed; 5 yaw disturbance,
14 upper-reference and 25 execution-gate C++ tests passed. Shell syntax,
Python compilation and `git diff --check` passed. No commit/push or serial
execution occurred; v1/v2 configs/evidence were not rewritten.

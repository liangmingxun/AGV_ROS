# Exp2c-v5b ramp-hold-ramp yaw-effectiveness exploration

EFFECT_EXPLORATION_ONLY / NOT_FORMAL_PAPER_EVIDENCE

Starting HEAD: `7a4788258f1e6fdcee3bda3ccbfe84e76c91889c`.
Fake only; all hardware grants remain false. No serial execution, automatic
commit/push, or changes to archived v4/v5 screening/results are authorized.

## Independent model and causal chain

Robot2's measured progress first reaching 2.0 m latches a wall-clock trigger.
Let elapsed time be t, Q(u)=10u³−15u⁴+6u⁵, and h=0.20:

- Before trigger and t≥3.7 s: γ=1.
- 0≤t<0.6 s: γ=1−(1−h)Q(t/0.6).
- 0.6≤t<3.1 s: γ=h.
- 3.1≤t<3.7 s: γ=h+(1−h)Q((t−3.1)/0.6).

The quintic endpoints have zero first and second derivatives. Numerical envelope
clamping only prevents floating-point overshoot of [0,1]. The trigger is one-shot;
progress regression cannot pause or extend the pulse.

For the original controller demand vL,vR, vc=(vL+vR)/2 and d=(vR−vL)/2:
degraded vL=vc−γd; degraded vR=vc+γd. The mean is preserved to floating-point
precision; differential execution is attenuated. Only Robot2's execution copy is
modified, after saving the controller raw demand. The existing execution limiter
and fake chassis remain downstream. Capability reports stay unchanged. Risk uses
the previous pre-degradation raw demand, not γ or a disturbance preview.

The old bell model and diagnostic topic are untouched. v5b has its own runtime
block `yaw_effectiveness_hold_v5b`, topic
`/multi_agv/yaw_effectiveness_hold_state`, raw CSV, aligned-field prefix, and native
tracker diagnostic fields. Existing v5 fields are not replaced.

## Authorized stages

S1: `S1_gamma0p20_down0p6_hold2p5_up0p6_R10`, fresh M1b+R1 then M1+R1.
Upper risk gain remains 0.10. If the mechanism is effective but geometry remains
below the requested approximately 10% target, S1R12 permits only M1's three upper
risk gains to become 0.12. All other configuration entries are identical.
This implementation runs a fresh pair for each stage; it does not reuse Y0 data.
No further amplitude, duration, trigger, margin, controller or geometry search.

Fixed task velocity is 0.100 m/s. R1, tracker, physical upper, nominal upper,
inner margin, wheel margins/limits, hard envelope, Stage-D calibration, fixture,
and the clockwise R=0.7 m smooth-exit path remain unchanged. Metadata records
nominal common velocity as 0.10 m/s.

Quality policy is authorized only for exact v5/v5b fake experiment identities and
hardware=false. Finite geometry errors, including the unchanged 10 mm rigid-fit
threshold, are observations, not task-abort gates. True nonfinite values,
invalid state chains, structural non-computability and the fixed 95 s timeout
remain fatal. Physical limiters remain applied and fully reported.

## Fixed analysis

Baseline: [−3,0) s; pulse: [0,3.7) s; recovery: [3.7,18.7) s;
primary combined window: [0,18.7) s. No outcome-dependent window selection.
Full-task native diagnostic coverage and whole-recorded control coverage are
reported separately. Whole-recorded control data retain early startup limiter
events even before the first native diagnostic message.

Report native longitudinal/lateral/heading errors for all three robots, all
pairwise sides, support/progress errors, estimator rigid-fit values and source
timestamps. Signed minima/maxima/means are retained alongside absolute
peak/RMS/IAE. Recovery censoring is reported, not silently removed.

Actual reference reduction means the same run's candidate common velocity minus
its published effective common reference. Risk contraction is separately the
dynamic boundary's contraction; neither nominal-inner-upper minus reference nor
a cross-run velocity difference is called actual reference reduction.

Exploratory selection requires a true risk contraction, active boundary, actual
reference reduction, clear resource benefit, at least one main geometry IAE
improving ≥10%, and all four main geometry IAEs improving in the same direction.
For the qualitative "not long-term limiter dependent" condition, the report uses
an explicit operational convention: maximum continuous limiter during the fixed
3.7 s pulse <0.5 s for both methods. This is observational selection, not a new
control safety gate; all pulse/recovery/startup events remain exposed.

## Invocation

From the platform-foundation-linux worktree, after building the affected nodes:

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
bash src/multi_agv_bringup/scripts/run_exp2c_v5b_yaw_hold_fake.sh \
  --candidate S1_gamma0p20_down0p6_hold2p5_up0p6_R10 --method pair \
  --ros-port 11647
```

Use a fresh output root and unoccupied private localhost ROS port. The runner
never checks or changes old `screen_passed`, `selected_candidate`, or formal-pair
semantics. Analysis exit 10 means effect insufficient, not physical failure.

Initial `..._20260914_S1` and `..._20260914_S1_verified` roots are retained startup
integration failures: the estimator binary had not rebuilt after the policy
header dependency changed. Neither produced a usable pair. The selected analysis
roots end in `S1_complete` and, if run, `S1R12_complete`.

Detailed measured results and verification are in
`exp2c-v5b-hold-effect-results.md`.

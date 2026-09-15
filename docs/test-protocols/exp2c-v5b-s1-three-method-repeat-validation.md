# Exp2c-v5b frozen S1 three-method repeated validation

REPEAT_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE

Starting HEAD: `f418fa7264b1f351a2dacf76c3d872e6825ff781`.
Starting `git status --short` was empty. v5 quality observation/Y0, v5b hold,
S1/S1R12 code, protocols and results, and the existing M2b/Exp2b implementation
were present. No previous interrupted turn had started a run or changed files.
No reset, historical-evidence overwrite, serial execution, commit or push.

## Frozen experiment

Only S1 is run: Robot2 actual progress reaches 2.0 m once, then wall-clock
quintic ramp-down/hold/ramp-up = 0.6/2.5/0.6 s; gamma_hold=0.20. Task velocity
0.100 m/s, clockwise R=0.7 m smooth-exit path, 30 cm equilateral support geometry.
Wheel mean is preserved; differential execution is attenuated. M1 upper risk
gain is frozen at 0.10. No gain, pulse, duration, tracker, R1, path, calibration,
wheel-limit or geometry search is authorized.

All methods use the same fake chassis, calibration, initial geometry, path,
physical 0.16 m/s wheel limit, execution limiter, trigger/profile and fixed 95 s
timeout. Shared tracker parameters, leader state and distributed initialization
match between the already-frozen M1 and M2b runtimes. Existing execution options
differ where the two methods have different lower-layer semantics; they are
preserved, not normalized by changing the method.

## M2b audit and minimal integration

`formal_fake_algorithm_node.cpp` selects both upper/lower M2b and executes
`m2b_controller_->step(input)`, then returns before the `UpperAgentInput` /
upper-generator step. `M2bController` implementation and
`config/exp2b_M2b.yaml` are unchanged. M2b is not M2a and not M2b+R1.
Results use the label M2b_COMPLETE; existing wire method_id remains M2b_M2b
for compatibility with recorder/command-authority checks.

The common `applyYawHold` helper operates on the saved native tracking raw
wheel demand and a separate execution copy. Both the original M1/M1b branch
and the M2b branch call it after saving pre-degradation demand and before the
identical limiter/publication/fake plant. M2b capability remains log-only in its
equations; no robust-margin feedback is passed to M2b.

The exact independent repeat-validation identity is admitted to the existing
quality policy only for fake transport and hardware=false. Finite quality
threshold exceedances are observations. Nonfinite values, invalid state chains,
non-computable/structurally invalid fits and fixed task timeout remain fatal.
Old formal/serial identities and safety semantics are unchanged.

An independent M2b smoke test verifies startup, trigger, gamma, wheel-mean
preservation, complete raw/applied wheel and all-robot native geometry data,
task completion and unified analysis. `M2B_INTEGRATION_READY` is required before
the repeated runner can start. Smoke is never included in the repeated sample.

## Freshness and order

Five triads, 15 fresh runs. Every method invocation starts and owns a fresh
localhost ROS master and fresh plant/controller/estimator/disturbance state.
No retained dynamic boundaries, previous wheel-demand history or trigger state.

| Triad | Order |
| --- | --- |
| 01 | M1b → M1 → M2b_COMPLETE |
| 02 | M2b_COMPLETE → M1b → M1 |
| 03 | M1 → M2b_COMPLETE → M1b |
| 04 | M1b → M2b_COMPLETE → M1 |
| 05 | M2b_COMPLETE → M1 → M1b |

Each triad has `order.json`; each run's runtime snapshot includes triad, method
and order. Config SHA256 and starting Git state are retained. Each method folder
contains a `run/` evidence subdirectory and independent config/log directories.

## Metrics and statistics

All use baseline [−3,0), disturbance [0,3.7), recovery [3.7,18.7), primary
[0,18.7) s, plus full-task and pre-trigger statistics. Time-weighted integration
uses observed timestamps, not outcome-selected windows or smoothed signals.
Whole-recorded raw/limiter coverage separately retains early startup before
the first available native diagnostic message.

Common native geometry: all three robots' longitudinal/lateral/heading and
progress errors, Robot2 support error, true estimator rigid-fit and duration
above unchanged 10 mm threshold, all d12/d13/d23 side deviations and fleet max.
Signed min/max/mean are retained with absolute peak/RMS/IAE. Censored recovery
is N/A, not zero. Full source diagnostic CSVs remain unchanged.

Common execution-resource margin is
`clip(min(physical reported wheel limit − abs(current pre-disturbance raw))/0.010,0,1)`.
It is an observational current-demand metric for all methods, distinct from
M1's causal previous-demand margin used internally. Threshold exposure and
J_risk integrals therefore share exactly the same definition across methods.
Report controller raw peak/RMS and durations above 0.150/0.155/0.160, degraded
and applied peak, and all-fleet limiter samples/duration/max-continuous by phase.

M1/M1b mechanism diagnostics are separate: risk contraction, active boundary,
own candidate minus own published common reference, published reference and
robust margin. M2b equivalents are N/A, not fabricated zero. M2b's actual
transformed-state, consensus, mapping, controller and constraint diagnostics
are retained separately.

For every triad, comparison A is strict M1-vs-M1b ablation; comparison B is
M1-vs-complete-literature-M2b, not ablation. Percentage=(baseline−M1)/baseline.
Zero baseline gives N/A percentage while preserving absolute paired difference.
All runs/outliers and negative differences are retained; no good-run selection.

Outputs include individual metrics, paired CSVs, absolute method statistics,
mean/sample-SD/median/min/max and Student-t 95% CIs for paired absolute
differences and per-triad percentages. No historical S1/Y0 pair is included.
Five deterministic fake runs mostly reflect scheduling variation, not five
independent physical experiments; CIs are not formal paper evidence.

The ablation confirmation rule requires five valid M1/M1b pairs, active M1
mechanism each time, all four primary mean geometry improvements ≥5%, majority
direction consistency, and material mean J_risk0.5 improvement. M2b completion
does not independently determine ablation validity. Literature comparison
reports tradeoffs; the automated conservative label is mixed whenever geometry,
resources and task cost do not all favor one method.

## Commands

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
bash src/multi_agv_bringup/scripts/run_exp2c_v5b_repeat_one_fake.sh \
  --method M2b --ros-port 11649 --output-root /absolute/fresh/smoke-root

bash src/multi_agv_bringup/scripts/run_exp2c_v5b_three_method_repeat_fake.sh \
  --smoke-run /absolute/fresh/smoke-root/run \
  --output-root /absolute/fresh/repeat-root --ros-port 11649
```

Both roots must be fresh and the port unoccupied. Only fake runs are allowed.
Do not reuse this entry for hardware. Detailed measured results are retained in
the independent data root's `repeat_validation_summary.json/.md`; this protocol
does not alter v4/v5/v5b historical evidence.

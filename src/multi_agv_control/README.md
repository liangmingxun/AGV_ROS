# multi_agv_control geometry and odometry-state foundation

Task 8 provides deterministic, side-effect-free geometry primitives. Task 9
adds local-window projection and a read-only odometry pretest estimator.
Command publication exists only in explicitly launched controller/pretest
executables; the geometry and estimator components remain side-effect free.

## `SCurvePath`

The raw path is

```text
x = xi
y = A sin(2 pi xi / L_xi)
```

Each table interval is integrated with Simpson's rule. The monotonic table is
used for `s <-> xi` lookup, while position and derivatives are evaluated from
the analytic raw curve. `PathSample` contains `p(s)`, `p'(s)`, `p''(s)`, unit
tangent/normal, continuous S-curve heading, curvature and `d curvature / ds`.
Finite queries outside the path are clamped to the endpoints; non-finite
queries and invalid configurations are rejected.

## `SupportGeometry`

Each physical support offset is represented in the load-path Frenet frame as
`q=[q_tangent, q_normal]`. A single common load arc length produces all three
support references. Samples include

```text
p_i(s)
p_i'(s) = (1-kappa*q_normal)*t + kappa*q_tangent*n
g_i(s)  = norm(p_i'(s))
d theta_i / ds
```

Construction densely checks `1-kappa*q_normal >= chi_min` and
`g_i >= g_min` for all three supports. Invalid or non-finite geometry is
rejected before it can be consumed by later controller tasks.

`base_link` is the midpoint of the left/right drive-wheel axle, not the
chassis-outline centre. The measured turntable centre is at
`base_to_support=[-0.01783, 0] m`. The planar tracker constructs an
offset-compensated, nonholonomic chassis reference before converting chassis
speed/yaw rate to wheel-edge velocities. Capability mapping must use those
chassis-reference derivatives rather than uncorrected support-path
derivatives.

The YAML files now freeze the unloaded `0.40 m` equilateral pretest fixture:
Robot1 front, Robot2 left-rear and Robot3 right-rear, on the Robot1-validated
`A=0.05 m`, longitudinal `1.0 m` short S. This is not the final loaded-object
geometry. The generic path, support and central-controller
`hardware_execution_authorized` fields remain false. Only the separate
`three_car_unloaded_bounded_pretest` fixture authorizes the frozen
`0.05 m/s` by `1.00 m` continuous S-path gate, and its launch still defaults to
commands disabled with four explicit serial confirmations required.

## Task 9 projection and state estimation

`PathProjector` searches only around the previous progress, performs a coarse
scan and bounded refinement, and rejects non-converged or excessive-residual
solutions. `StateEstimator` uses only increasing measurement timestamps,
causal finite differences and the configured one-pole filter. Invalid samples
do not silently become reference values.

`path_state_estimator_node` consumes `/agv1/odom`, `/agv2/odom` and
`/agv3/odom`, transforms them into `world`, projects each measured support
point, and publishes `/multi_agv/cooperative_state` at 100 Hz. Robot validity
is independent. The load pose in odometry mode is an explicitly pretest-only
three-support rigid fit; a stale or inconsistent robot invalidates the load
fit without invalidating the other robots.

For software or odometry pretests:

```bash
roslaunch multi_agv_bringup odom_state_estimator.launch
```

Before formal physical experiments, replace and freeze every provisional path,
support offset and `world_to_odom` transform in the bringup YAML files.

## Task 10 capability mapping

`CapabilityMapper` converts the wheel-linear speed and asymmetric
acceleration/deceleration limits reported by each chassis into path-channel
limits using the local support-path speed scale, heading rate and wheel
separation. Left and right wheel constraints are evaluated independently, the
public bound is the minimum across three robots, and engineering reserves are
applied exactly once. The checked-in reserves are zero-valued software
fixtures and remain hardware-gated until the measured delay/model budget is
available.

## Task 11 constant-reference pretest

`PlanarSupportTracker` evaluates all three support references at one common
load progress, adds body-frame position/heading feedback to the analytic
feedforward, and emits raw left/right wheel-linear demands. The 100 Hz
`multi_agv_controller_node` publishes the frozen command/reference/controller
messages and sends explicit zero commands when cooperative state is stale.

The bringup entry defaults to command publication disabled:

```bash
roslaunch multi_agv_bringup odom_pretest.launch
```

Command publication may be enabled automatically only for `fake` transport.
For non-fake transport the path, geometry and pretest YAML authorization gates
must all be explicitly replaced and approved first.

## Task 12 causal Robot2 derating pretest

`ExperimentSupervisor` implements the frozen phase machine and changes the
Robot2 derating sequence only when the target changes. Activation and recovery
consume only measured Robot2 or load progress from `CooperativeState`; no
reference topic enters the trigger. The node republishes the current target at
10 Hz with an unchanged sequence, while the chassis continues to own the
smooth 250 Hz capability transition and physical limiter.

The complete fake odometry pretest can be launched explicitly with:

```bash
roslaunch multi_agv_bringup odom_pretest.launch \
  enable_commands:=true enable_derating:=true
```

Both command and derating publication default to disabled. Non-fake derating
is refused while the pretest configuration remains hardware-gated.

When command publication is disabled, `multi_agv_controller_node` does not
register `/agvX/chassis_command` publishers at all. This makes
`central_odom_pretest.launch` a graph-level read-only entry instead of a
publisher that merely remains silent. The distributed three-car topology is
checked with:

```bash
rosrun multi_agv_bringup check_three_car_readonly_gate.py \
  --observe-seconds 5
```

After the measured `world_to_odom`, unloaded support layout and path dimensions
have been frozen, repeat it with `--require-valid-state`. That second gate
requires all three robot/support/path states and the fitted virtual-load pose
and path state to be valid. Neither invocation authorizes motion.

The unloaded single-car floor S gate is intentionally separate from the fleet
controller. `single_car_s_pretest.launch` accepts `robot_index:=1|2|3`, runs
one bounded sine period at `0.05 m/s`, and publishes only the selected
`/agvN/chassis_command`. The odometry-only entry anchors to the selected
starting odometry pose. The camera entry consumes the selected
`/pose_provider/agvN/base_pose_fused`:
camera measurements correct absolute SE(2), while actual-wheel translation
and corrected-IMU yaw from `/agvN/odom` propagate the pose at chassis rate.
The bounded curvature-feedforward scale/preview interface defaults to the
original uncompensated tracker after the first physical A/B trial rejected
the candidate compensation. At the prescribed endpoint it immediately
commands zero, confirms stopped wheel feedback, and records a passive endpoint
window; endpoint pose remains an offline metric and never causes extra motion.
The latched `/agvN/s_pretest/result` is the run authority, and an independent
actual-wheel-speed watchdog remains active. Serial execution still requires
explicit command, clear-area and wheels-on-floor launch acknowledgements; the
launch defaults remain non-moving.

After the distributed three-car read-only gate has passed repeatedly, the
first fleet motion uses `three_car_unloaded_bounded_pretest.launch`, not the
generic central controller. It refuses competing command/reference
publishers, requires one cooperative-state source and two command subscribers
per car (chassis plus recorder), checks synchronized valid state, voltage,
stationary wheel feedback, progress consistency and bounded tracking errors,
then advances `1.00 m` continuously at `0.05 m/s`. Any invalid state or excessive
command causes repeated zero commands to all three cars and all-wheel stop
confirmation.

## Task 13/14 formal algorithm and fake-only ROS integration

The library contains the formal algorithm structure without authorising a
physical run:

- `DynamicBoundaryProjector` performs a true two-dimensional Euclidean
  projection onto the five-constraint admissible set and reports its active
  constraints.
- `UpperReferenceGenerator` provides isolated M1, M2a and M4 modes. M1 updates
  at 25 Hz and holds between updates; M2a logs capability but cannot use it in
  control; M4 uses a pre-registered fixed speed and rejects changes while a
  run is active.
- `LowerChannelController` provides the shared constrained-error transform,
  asymmetric acceleration limiting, parameter/disturbance estimate bounds and
  isolated R1--R4 adaptation/robust switches at a caller-supplied 100 Hz `dt`.
  The theoretical acceleration limit is applied exactly once, before the
  existing chassis wheel limiter.

The approved Python references have now been used to generate deterministic
100-sample fixtures. The M1 boundary and distributed `z/upsilon/phi` update,
and the complete R1 lower equations for all three agents, match those fixtures
to at most `1e-8`. R2--R4 remain the registered two-switch ablations of that
shared lower core.

The source identities are frozen in
`test/data/reference_hashes.txt`; the generated fixtures are
`test/data/m1_golden.csv` and `test/data/lower_m1_golden.csv`. They originate
from:

```text
experiment2_v81_paper_faithful_comparison.py
experiment1_v37_polished_sim.py
```

The reference programs themselves remain external inputs and are not copied
into the package. `test/reference/export_golden.py` fails when an approved
source path is absent and redirects only the reference output directory; it
does not alter the equations.

`formal_fake_algorithm_node` now loads one `exp2a_M*.yaml`, one
`exp3_R*.yaml`, the frozen path/support files and
`formal_fake_runtime.yaml`. It connects capability mapping, the selected
upper mode, the selected lower mode and the planar tracker to all three fake
chassis command topics. The formal launch is:

```bash
roslaunch multi_agv_bringup formal_fake_algorithm.launch
```

That launch always starts `three_fake_chassis.launch`; no transport argument is
exposed. The node independently requires `platform_transport_type=fake`,
algorithm authorization, golden verification and
`hardware_execution_authorized=false` on both layers. A serial value, missing
golden gate, hardware authorization or unregistered M4 speed causes startup to
fail before any command publisher becomes active. In addition, the node does
not register any `/agvX/chassis_command` publisher until all three actual
`/agvX/chassis_controller/transport_type` parameters exist and equal `fake`;
an actual serial chassis binding is fatal, even if the algorithm node's own
parameter was forged as fake. The experiment YAML files therefore set
`algorithm_execution_authorized: true` but deliberately retain
`hardware_execution_authorized: false`.

At 100 Hz the node publishes `/multi_agv/path_reference`,
`/multi_agv/controller_state` and
`/multi_agv/formal_algorithm_state`. The last topic has fixed schema label
`formal_algorithm_state_v1:header9+3x27` and records generation time, selected
modes, common boundary/reference values, per-robot dynamic-boundary/risk
state, distributed `z/upsilon/phi` state, constrained-error terms, raw/limited
input, parameter/disturbance estimates and limit flags. If cooperative state
or any capability report is missing, stale or invalid—or any algorithm stage
rejects its input—the node continuously sends explicit zero commands to all
three fake chassis and marks the public/debug state invalid.

The M1/R1 and M2a/R4 rostests cover non-default configuration loading, the
complete valid command chain, actual fake-chassis binding, fixed
internal-state record shape, unique command authority, absence of `/cmd_vel`,
and cooperative-state loss followed by three-car fail-zero. This is software
authorization only. There is intentionally no serial launch path for the
formal controller, and none of these changes authorise a physical experiment.

## Task 15 camera localization software boundary

`camera_pose_adapter_node` accepts four calibrated world-frame
`geometry_msgs/PoseStamped` streams for agv1/agv2/agv3/load plus independent
confidence streams. It applies configured planar tag-to-target rigid
transforms, preserves transformed raw poses, emits only causal filtered poses
when confidence/time/frame gates pass, and never publishes `/tf` or chassis
commands. `path_state_estimator_node` selects `localization_mode: camera`,
retains each original measurement stamp and publishes the unchanged
`CooperativeState` schema with `SOURCE_CAMERA`.

Camera dropout is independent: losing one robot tag invalidates only that
robot's pose/support/path flags; the other robots and directly observed load
remain available. Consumers still subscribe only to
`/multi_agv/cooperative_state`, so switching odometry/camera does not change
the controller interface or command authority.

Camera `PoseStamped.frame_id` uses `world@xxxxxxxx`, where the hexadecimal
suffix is a nonzero ground-calibration generation token. ROS1 owns and
rewrites `header.seq`, so it has no calibration meaning. A generation change clears all four adapter filters and all
camera state-estimator histories; late samples from a retired generation are
rejected. A stream that returns after a stale gap also restarts its own pose
filter from the first accepted measurement.

The checked-in `localization_camera.yaml` is deliberately blocked by both
`calibration_authorized: false` and `extrinsics_frozen: false`. Its zero
tag-to-target transforms are placeholders, not measurements. The software
test uses a separate synthetic, authorized calibration. Do not pass
`calibration_authorized:=true` on `camera_formal.launch` until camera intrinsics,
world extrinsics, all four tag transforms, confidence semantics, latency and
occlusion thresholds have been measured and archived.

## Task 16–18 software completion boundary

`M2bController` implements the complete Experiment2 literature comparison:
fixed velocity bounds, constrained distributed generator, RK4 substeps,
nonlinear error mapping and literature lower law. The checked-in
`m2b_golden.csv` matches 100 authoritative samples, capability feedback is
log-only, and `/multi_agv/m2b_algorithm_state` preserves all paper mapping
terms. `formal_fake_m2b.launch` remains structurally fake-only.

M2b owns frozen literature-channel acceleration/deceleration limits. Runtime
`CapabilityReport` values are log-only inside `M2bController`; physical
capability is enforced later by the identical chassis-side wheel limiter used
by the other methods. The exact-sign zero-boundary-layer setting is retained
only for authoritative golden reproduction. Positive boundary layers are
supported and continuity-tested for separately preregistered physical
configurations.

The supervisor now opens and closes one contiguous evaluation interval from
actual progress and can continue through post-restoration until its registered
end. Task 17 recording can require an exact SHA approval; the checked-in
registry authorizes only software rehearsals and refuses formal statistics.
Stage A–D and formal-camera protocols live under `docs/test-protocols/`.
They are unsigned templates, not evidence that physical gates passed.

The recorder publishes `/experiment_recorder/armed` at 5 Hz only after
rosbag has subscribed to every required stream, alongside the latched
`/experiment_recorder/method_id`. The formal algorithm can require a fresh,
method-matching heartbeat and will retain command authority while repeatedly
publishing zero without advancing its reference. The optional software
watchdog Boolean can feed the fifth causal risk margin; stale primary state
still causes immediate fail-zero.

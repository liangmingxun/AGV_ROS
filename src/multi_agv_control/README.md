# multi_agv_control geometry and odometry-state foundation

Task 8 provides deterministic, side-effect-free geometry primitives. Task 9
adds local-window projection and a read-only odometry pretest estimator. No
component in this package publishes a chassis command.

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

The YAML files currently contain software-validation fixtures only. Their
`hardware_execution_authorized` fields remain false until the laboratory path
and physical tray offsets have been measured and frozen.

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

The unloaded Robot1 floor S gate is intentionally separate from the fleet
controller. `robot1_single_s_pretest.launch` runs one bounded sine period at
`0.05 m/s`, consumes only `/agv1/odom` and `/agv1/chassis_feedback`, and is the
only pretest node allowed to publish `/agv1/chassis_command`. It anchors the
path to the starting odometry pose and uses the measured rear support offset
when constructing the nonholonomic chassis reference. Serial execution still
requires explicit command, clear-area and wheels-on-floor launch
acknowledgements; the launch defaults remain non-moving.

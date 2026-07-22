# multi_agv_control geometry foundation

Task 8 provides deterministic, side-effect-free geometry primitives. It does
not publish ROS topics and cannot command a chassis.

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

The YAML files currently contain software-validation fixtures only. Their
`hardware_execution_authorized` fields remain false until the laboratory path
and physical tray offsets have been measured and frozen.

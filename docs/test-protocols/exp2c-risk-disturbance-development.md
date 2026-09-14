# Exp2c risk-disturbance development status

Status: software-integrated and fail-closed; NOT physically authorized. No
physical authorization is granted by these changes.

Implemented:

- `UpperMode::kM1b`: nominal boundary intersected with mapped capability minus
  reserve. Risk diagnostics do not affect this baseline.
- Opt-in shared hard envelope for M1/M1b, disabled for existing configurations.
  Capability is checked between risk updates as well. An infeasible intersection
  is invalid, never projected upwards to satisfy minimum width.
- Independent M1 risk state, with contraction measured from the nominal/capability
  baseline rather than from unused physical headroom.
- Pure Robot2-only acceleration disturbance function: quintic spatial window
  2.0–2.8 m with 0.15 m transitions, 0.30 Hz, bounded peak fraction <=0.70.
  Relative terms are 0.60 constant, 0.25 saturated speed ratio and 0.15 sinusoid.
  These are candidate test settings, not an experimentally selected level.
- Independent `run_circle_0p10_risk_comparison.sh` parser and M1/M1b configs,
  using the shared R0.7 smooth-exit path, R1, reference-v1 runtime, Stage-D
  platform settings, recorder and safety gates.
- Versioned disturbance/envelope diagnostic stream, bag conversion and causal
  metrics. Existing streams retain their schemas.

Verified: formal and fake algorithm binaries build; focused controller, gate,
analysis and bringup-static tests pass. Eight local fake ROS integrations were
run below. No physical experiment was run.

Still required before any physical use:

1. Obtain a defensible paired selection; the current four levels did not pass.
2. Add comparison figures after a valid paired dataset exists.
3. Complete independent manual physical authorization before any serial run.

Exploratory selection may rank identical-condition candidates by M1 improvement
over M1b, but must retain all candidate outcomes and safety/control costs. It
does not establish general superiority or predict physical performance.
Freeze a selected candidate before independent physical repetition. Do not
change the baseline's calibration or gates to obtain a favourable comparison.

Before the paired screen, the current undisturbed M1 run covered the entire injection window, but had no
M1b or injected-disturbance response. Therefore the screening tool reports
`INSUFFICIENT_PAIRED_RESPONSE_EVIDENCE` and selects no level. Existing
production YAML, R1, planar tracker, calibration, path, wheel limits,
camera/fusion and experiment data are unchanged.

Screening source:
`m1_r1_circle_r0p7_cw_smooth_exit_0p10_normal_pilot_20260913_181024`.
All four candidates cover 800 samples (approximately 8.00 s). At unit
available acceleration, the computed peak magnitudes are exactly 0.30, 0.45,
0.60 and 0.70, with absolute acceleration impulses 1.6632, 2.4949, 3.3265 and
3.8809 respectively. These quantify excitation only and are not response or
method-comparison results.

## 2026-09-14 fake paired screen

`run_risk_disturbance_fake_screen.sh` uses a private localhost ROS master and
fake chassis only. M1 and M1b share the exact 30 cm initial fixture, circle
path, R1 controller and spatial disturbance. All eight bags passed conversion,
validation and metric generation under:

`experiment_data/risk_disturbance_fake_screen/20260914_143545/`

The rule was fixed before running: M1 Robot2 progress RMSE <= 90% of M1b,
progress maximum <= 95%, velocity RMSE <= 105%, and nonnegative hard-capability
margin for both. Disturbance-region Robot2 progress RMSE (M1 / M1b) was:

- 0p30: 1.729 / 1.646 mm;
- 0p45: 2.317 / 2.481 mm;
- 0p60: 2.996 / 2.975 mm;
- 0p70: 3.452 / 3.406 mm.

No level passed. The favourable direction at 0p45 is only about 6.6%, below
the required 10%; `selected_level` therefore remains null. Stop selection at
this point: do not authorize Exp2c hardware and do not tune the controller or
disturbance after viewing these outcomes.

# Unified geometric progress / planar reference pilot

The current normal and reference-v1 0.10 m/s runtimes set
`execution/align_progress_to_initial_pose: false`. R1 (and M2b if selected)
uses signed native geometric progress, matching the ideal path used by the
planar tracker and formation feedback. Initial position mismatch is an
initial tracking error, not a permanent agent-specific target offset.
Stationary initialization still records offsets as evidence.

Legacy configurations default this switch to true and are unchanged. The
four frozen startup parameters, measured-velocity feedback, signed 25 mm
start extension, 120 ms projection, physical limiter and safety gates are
unchanged. The existing ramp/envelope and planar linear feedback fade-in
remain active; no new claim is made that R1 initial position error itself
is faded or that perfect formation recovery is guaranteed.

Analysis uses recorded ControllerState.path_progress_actual for the main
tracking curve. If that evidence is absent, the legacy recorded-origin
alignment remains the fallback. The origin-aligned displacement is separately
exported as agvN_s_origin_aligned_actual and summarized under
path.origin_aligned_diagnostic. Source-time diagnostics retain the separately
labeled origin-aligned displacement coordinate. Original geometric poses/progress, initial
offsets and formation errors are retained. Fixed plot axes and raw/trend
folders are unchanged; historical experiments are not relabeled as new
controller runs or rewritten.

This is a new controller-coordinate candidate, not part of the startup-only
freeze qualification. Robot1 compiles the central algorithm. No chassis
code changed; Robot2/3 do not require synchronization or recompilation.
First repeat no-derating M1 and compare actual geometric convergence,
startup speed peak, steady error and validity. Only then use the same runtime
and safety gates for paired 0.80 M1/M2a derating trials.

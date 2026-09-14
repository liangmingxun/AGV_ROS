# M2a unloaded deformation observation

Dedicated condition: 0.10 m/s, Robot2 capability ratio 0.80, reference-v1.
Use `--formation-observation --confirm-support-range-0p80-clear` with the
existing circle comparison interface. This does not replace the original
M2a comparison configuration. No physical payload or mechanical connection.

Initial formation anchoring still requires the original 25 mm residual gate.
In this observation only, off-path distance and scalar progress-correction
thresholds are diagnostic rather than validity gates. The bounded projection
search and finite/converged projection are retained. Motion remains measured,
not command-derived. Original configurations retain all their error gates.
Only the runtime finite rigid-fit residual becomes warning-only. Any pair of
measured support centres exceeding 0.80 m immediately latches the independent
range gate until estimator restart. All localization, timing, finite-command,
physical wheel limits, watchdogs and original task distance/time protections
remain in place. The operator must clear the full expanded footprint including
vehicle bodies and keep emergency stop available. This is not a collision
avoidance system; stop manually if vehicles approach each other or obstacles.

At large deformation the virtual-load fit is only a geometric diagnostic, not
a true rigid payload measurement. Report support side-length errors, progress
errors, residuals and completion/range-abort status without claiming successful
formation tracking solely from reaching the endpoint. Keep aborted bags.

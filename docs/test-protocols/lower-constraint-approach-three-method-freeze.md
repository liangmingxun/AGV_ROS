# Lower-constraint three-method physical comparison freeze

Status: `FROZEN_ACCEPTED_PHYSICAL_COMPARISON`

Date: 2026-09-17

Code baseline: `46d1993d8e078a62c38b019bbd2b1e2b6911731d`

Experiment: `lower_layer_constraint_approach_physical_comparison`

Condition: unloaded three-robot physical circle experiment at 0.10 m/s with
the common lower-controller velocity interval fixed to `[-0.05, 0.14] m/s`.

## Frozen run set

R1:

- `lower_constraint_approach_R1_20260917_184920`
- `lower_constraint_approach_R1_20260917_185150`
- `lower_constraint_approach_R1_20260917_195046`

PaperNM:

- `lower_constraint_approach_PaperNM_20260917_183819`
- `lower_constraint_approach_PaperNM_20260917_190102`
- `lower_constraint_approach_PaperNM_20260917_194359`

PDPenalty:

- `lower_constraint_approach_PDPenalty_20260917_182205`
- `lower_constraint_approach_PDPenalty_20260917_185415`
- `lower_constraint_approach_PDPenalty_20260917_195702`

The R1 run `lower_constraint_approach_R1_20260917_194626` is not part of the
frozen set. It ranked last in the joint assessment of lower-layer position and
velocity tracking, controller burden, support error, rigid-fit error, and
pairwise error. This exclusion was not selected solely to optimize formation
geometry.

Incomplete runs, including PDPenalty `183616` and invalid PDPenalty `195440`,
are not part of this comparison.

## Frozen aggregate results

Values are the mean of three fresh physical runs. Parenthesized values are the
sample standard deviation where retained in the review.

| Metric | R1 | PaperNM | PDPenalty |
|---|---:|---:|---:|
| Lower-layer position RMSE (mm) | 1.651 (0.185) | 1.726 (0.558) | 1.879 (0.583) |
| Position maximum (mm) | 7.12 (1.95) | 11.80 (6.71) | 11.72 (9.30) |
| Velocity-error RMSE (mm/s) | 2.594 (0.181) | 3.061 (1.184) | 2.749 (0.546) |
| Controller-input RMS (m/s^2) | 0.01266 | 0.01469 | 0.01302 |
| Controller-input P99 (m/s^2) | 0.04490 | 0.04977 | 0.04835 |
| Support RMSE (mm) | 14.30 | 14.83 | 15.40 |
| Rigid-fit RMSE (mm) | 7.82 | 7.82 | 8.62 |
| Pairwise RMSE (mm) | 10.32 | 10.33 | 11.48 |
| Equivalent-load position RMSE (mm) | 9.92 | 10.27 | 10.58 |
| Equivalent-load yaw RMSE (rad) | 0.03827 | 0.04172 | 0.04080 |
| Wheel-demand peak (m/s) | 0.1426 | 0.1534 | 0.1473 |
| Task time (s) | 54.039 | 54.023 | 54.067 |

No selected run crossed the lower-controller velocity interval. The frozen
interpretation is therefore limited to constraint-adjacent tracking,
formation, repeatability, and controller-burden behavior; it is not evidence
that a comparison controller violated the prescribed lower-layer constraint.

R1 has the best aggregate lower-layer tracking, peak suppression, support
error, controller burden, and wheel-demand peak. R1 and PaperNM are effectively
tied in rigid-fit and pairwise RMSE. PDPenalty has the lowest public/common path
progress RMSE, so the frozen conclusion does not claim that R1 dominates every
reported scalar metric.

Raw rosbag and packet-capture files are intentionally excluded from Git. The
retained per-run CSV, JSON, YAML, logs, and generated figures preserve the
reviewed evidence and processing provenance.

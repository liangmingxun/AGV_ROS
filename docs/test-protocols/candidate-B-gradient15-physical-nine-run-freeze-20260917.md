# Candidate B gradient15 physical nine-run freeze

Status: **MANUALLY AUTHORIZED — ACCEPTED ANALYSIS DATASET**  
Authorization date: 2026-09-17  
Authorization source: ETLAB operator instruction  
Experiment: Candidate B v2 spatial gradient15 physical comparison  
Payload semantics: unloaded equivalent payload

This document freezes the nine-run dataset used for the current M1, M1b and
M2c comparison. The selection is explicitly post hoc and must not be described
as preregistered or randomly selected. Original run manifests, validation
results and raw data are not modified by this authorization.

## Accepted M1 runs

- `candidate_B_spatial_gradient15_physical_M1_20260916_174748`
- `candidate_B_spatial_gradient15_physical_M1_20260916_193725`
- `candidate_B_spatial_gradient15_physical_M1_20260917_143608`

The fourth valid M1 run,
`candidate_B_spatial_gradient15_physical_M1_20260916_192903`, is not part of
the frozen nine-run dataset. It was excluded by the operator-requested post-hoc
three-of-four effectiveness selection. The retained `20260916_193725` run has
lower Support, Rigid-fit and Pairwise RMSE than the excluded run.

## Accepted M1b runs

- `candidate_B_spatial_gradient15_physical_M1b_20260916_175457`
- `candidate_B_spatial_gradient15_physical_M1b_20260917_142118`
- `candidate_B_spatial_gradient15_physical_M1b_20260917_142549`

## Accepted M2c runs

- `candidate_B_spatial_gradient15_physical_M2c_20260917_125010`
- `candidate_B_spatial_gradient15_physical_M2c_20260917_125243`
- `candidate_B_spatial_gradient15_physical_M2c_20260917_143155`

The attempted M2c run `candidate_B_spatial_gradient15_physical_M2c_20260917_142844`
is excluded because validation reported `SEQUENCE_GAP` for Robot3 chassis
feedback and no accepted physical metrics file was produced.

## Frozen aggregate results

The primary geometry comparison uses equal-progress spatial-domain RMSE. Values
below are the three-run mean and sample standard deviation.

| Method | Support spatial RMSE | Rigid-fit spatial RMSE | Pairwise spatial RMSE |
|---|---:|---:|---:|
| M1 | 17.36 +/- 1.24 mm | 7.35 +/- 0.44 mm | 13.31 +/- 1.04 mm |
| M1b | 19.25 +/- 0.75 mm | 8.55 +/- 0.34 mm | 17.13 +/- 0.74 mm |
| M2c | 21.19 +/- 1.58 mm | 8.90 +/- 0.55 mm | 17.34 +/- 1.02 mm |

M1 improvement relative to M1b is 9.85% for Support, 14.01% for Rigid-fit
and 22.30% for Pairwise spatial RMSE. M1 improvement relative to M2c is
18.09%, 17.40% and 23.25%, respectively.

## Evidence and repository policy

- Each accepted run is `VALID_COMPLETED` with zero algorithm and localization
  invalid fraction in its generated physical metrics.
- This authorization accepts the listed runs for the current comparison; it
  does not retroactively change Git SHA, dirty-worktree or other provenance
  fields recorded at acquisition time.
- Raw `.bag`, `.bag.active`, packet captures and other large binary capture
  files are intentionally excluded from Git. Derived CSV, JSON, YAML, logs and
  figures remain part of the accepted evidence set.
- Any later replacement or removal requires a new dated freeze record; this
  document must not be silently edited to conceal the original selection.

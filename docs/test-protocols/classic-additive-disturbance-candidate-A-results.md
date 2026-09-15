# Classic additive disturbance Candidate A results

Scope: `CLASSIC_ADDITIVE_DISTURBANCE_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE`.

Conclusion: `CLASSIC_ADDITIVE_DISTURBANCE_EFFECT_CONFIRMED`.

The integration smoke passed and all 9 fresh statistical runs (3 rotated
triads) were valid and completed with a complete recovery window. Candidate B
was not run and no parameter was tuned from these results.

## Mechanism and integrity

The recorded profile and wheel mapping reproduced Candidate A with maximum
absolute errors of about 1.1e-16 and 2.8e-17 respectively. The disturbance was
Robot2-only and was injected after saved native/controller raw wheel demand and
before the common limiter/fake plant for M1, M1b, and complete M2b. Robot2's
reported capability and the physical wheel limit stayed at 0.160 m/s; no
disturbance value was supplied directly to the M1 risk calculation.

M1 activated its dynamic boundary in 3/3 runs. Over the fixed 23 s primary
window, mean active duration was 8.357 s (fraction 0.3632), mean contraction
peak was 0.04511 m/s, and mean contraction integral was 0.17874 m. Actual
common-reference reduction had a mean peak of 0.01518 m/s and a time mean of
0.001368 m/s. The mean minimum common reference was 0.06189 m/s. These values
establish the intended disturbance -> tracking/controller burden -> margin ->
boundary contraction -> actual common-reference reduction chain.

## Paired primary-window results

Positive values mean lower M1 error/stress. Values are means of the three
within-triad relative improvements.

| Metric | M1 vs M1b | M1 vs complete M2b |
| --- | ---: | ---: |
| Robot2 lateral IAE | 18.54% | 19.21% |
| Robot2 heading IAE | 4.72% | 4.37% |
| Support IAE | 10.51% | 11.88% |
| Rigid-fit IAE | 14.42% | 15.67% |
| J_risk_0p5 | 62.32% | 63.42% |
| J_risk_0p7 | 60.12% | 61.23% |
| Wheel margin < 0.5 duration | 57.45% | 58.47% |
| Wheel margin < 0.7 duration | 53.64% | 54.84% |
| Controller raw peak | 8.30% | 8.31% |
| Controller raw > 0.160 duration | 70.36% | 71.22% |
| Robot2 physical limiter duration | 58.43% | 61.75% |
| Task time | -2.05% | -1.94% |

Mean primary-window Robot2 limiter durations were 0.387 s for M1, 0.937 s for
M1b, and 1.017 s for complete M2b. In the disturbance window alone they were
0.193 s, 0.540 s, and 0.527 s. Therefore M1's advantage is not created by
greater reliance on the limiter; it uses it less. The cost is about 1.0 s
(roughly 2%) additional task time. M1 longitudinal IAE was 1.59% worse than
M1b, while it was 0.88% better than M2b; this is the principal mixed secondary
metric, but M2b did not outperform M1 on any of the four declared primary
geometry IAEs.

## Required decisions

1. Candidate A formed the specified classic bounded time-varying additive
   compound disturbance in the full ROS fake chain.
2. CapabilityReport remained unchanged.
3. M1 risk, boundary, and actual-reference mechanisms activated in every M1
   run.
4. M1 improved lateral/heading/support/rigid-fit IAE over M1b by
   18.54/4.72/10.51/14.42%.
5. M1 improved the same metrics over complete M2b by
   19.21/4.37/11.88/15.67%.
6. J_risk and low-margin exposure decreased strongly.
7. Raw wheel peak and >0.160 s exposure decreased.
8. Limiting occurred in all methods, for the mean durations stated above.
9. The M1 benefit was not limiter-created; its limiter exposure was lower.
10. M2b was not better on a declared primary geometry metric; task time was
    shorter, and M1b had slightly better longitudinal IAE.
11. Candidate A is worth retaining for a later independently authorized formal
    anti-disturbance study; these runs themselves are not paper evidence.
12. Candidate B is not needed now. Stop parameter search.

Raw CSVs, run-level JSON, validation artifacts, and unsmoothed plots are stored
under `experiment_data/classic_additive_disturbance_candidate_A_validation/`
without overwriting historical Exp2c results.

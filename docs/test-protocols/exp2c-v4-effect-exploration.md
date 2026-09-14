# EFFECT_EXPLORATION_ONLY / NOT_FORMAL_PAPER_EVIDENCE

New user-authorized task, separate from Exp2c-v4 formal screening. Original
A/B reports, `screen_passed`, selection and formal paired logic are unchanged.
An exploratory working point does not invalidate the historical failed screens.

FAKE ONLY, all hardware grants false; no serial, commit or push. No changes
to M1 risk gains, R1, tracker, nominal bounds, inner margin, wheel-safe margin,
calibration, geometry, path, physical limits or safety gates. Only pulse A/T
are explored. Pulse shape/actual-progress trigger/wall-clock end and causal
previous controller raw margin remain unchanged.

Order: E0=(.022,2.0), E1=(.022,2.5), E2=(.022,3.0), E3=(.024,2.5),
E4=(.026,2.5). E0 runs only a new M1, referencing existing M1b-B; all later
conditions run fresh M1b then fresh M1. Stop immediately at first demonstrated
working point. No additional candidates. Default formal validator still
rejects all exploration-only A/T combinations; explicit fake experiment
identity and runtime opt-in are both required for exploration.

Operational hard stop: emergency, motion-time fail-zero, nonfinite command,
incomplete task/fake chain, unchanged rigid-fit hard-gate rejection, or
continuous physical speed limiting >=0.5 s in pulse+15 s recovery. Startup
speed limiting and brief raw overshoot alone are recorded, not disqualifying.
The 0.5 s long-limiter definition is fixed before E0, independent of results.
Recovery rules remain pre-trigger 3 s median +/- max(3 MAD,floor), return for
0.5 s, fixed 15 s recovery. Unrecovered signals remain null, not false zero.

J_risk_0p5 = integral max(0,0.5-wheel_margin) dt; same for 0.7. Report
separate baseline/pulse/recovery/combined metrics, raw and disturbed and
applied peaks, raw above .150/.155/.160 duration, causal margin statistics,
geometry and recovery, and startup limiting separately. Actual reference
reduction is this run's candidate common velocity minus published common
velocity (time-weighted mean), not nominal-bound-minus-reference.

Success: positive risk contraction, real active boundary and real common
reference reduction, plus any clear resource benefit versus M1b. Resolution
floors (not percentage efficacy targets): duration .02 s, integrated risk
.001 s, raw peak .0001 m/s, reference effect .00001 m/s. Record all signed
deltas, including regressions; faster heading/support/rigid recovery is
auxiliary, not required. Heading proxy limitation remains explicit.

Direct entry:

```bash
bash src/multi_agv_bringup/scripts/run_exp2c_v4_effect_exploration_fake.sh \
  --candidate E0 --method M1 --ros-port 11645 --output-root /new/exploration/root \
  --baseline /existing/screen_B/M1b_R1 \
  --baseline-log /existing/logs/screen_B/M1b_R1/algorithm.log
```

For E1..E4, use `--candidate E1 --method pair` and a fresh output root;
no baseline argument and no screening pass is needed. These are explicitly
exploratory comparisons, never formal pairs. Reports and CSVs carry both
evidence-scope markers; generated upper/auth configurations carry false
hardware grants and exploration-only provenance. E0 baseline is read-only
and its aligned CSV hash is included in the new comparison report.

## E0 result: stop search

On baseline HEAD `06badcf2bbc758007d2089ae6c4fe651052c51d2`, one fresh M1
E0 run completed at independent localhost master 11645. Source M1b-B was
read-only; its strict-screen failure remains unchanged. Evidence root:
`experiment_data/exp2c_v4_effect_exploration/EFFECT_EXPLORATION_ONLY_NOT_FORMAL_PAPER_EVIDENCE_20260914_E0`.

Status: **EFFECT_WORKING_POINT_FOUND**. No E1/E2/E3/E4 run was executed.
M1 contraction peak 0.014269 m/s, active duration 1.249953 s, actual reference
reduction peak 0.003428 m/s. Common reference minimum 0.092731 m/s includes
candidate dynamics as well as upper clipping; it must not be mislabeled as
the 0.003428 m/s candidate-minus-reference reduction.

In pulse+15 s recovery, M1 vs M1b: wheel margin below .5 duration
0.319957 vs 0.469874 s; J_risk_0p5 0.0720635 vs 0.1232645 s;
J_risk_0p7 0.1468036 vs 0.2276014 s; raw above .150 duration
0.569555 vs 0.690409 s; raw peak 0.158685 vs 0.160091 m/s.
Both have zero physical speed limiting in that window. M1 has no emergency,
motion-time fail-zero, rigid-fit gate rejection or incomplete task.

Geometry/recovery are mixed: support recovery 9.020199 vs 9.100195 s;
rigid-fit recovery 7.789857 vs 7.270002 s (M1 slower); heading proxy recovery
0.009693 vs 0.000080 s, neither a useful whole-formation recovery claim.
Peak progress error 0.006033 vs 0.005785 m (M1 slightly worse).
Thus E0 supports an exploratory wheel-resource relief effect, NOT a claim
of uniformly faster geometric recovery, repeatability or formal paper proof.

The first runner invocation completed and postprocessed the recorded motion,
then hit a shell tail error because the script was edited while Bash was
still reading it. The entry is now syntax-checked; comparison was generated
separately from the completed bag/CSV without repeating or replacing E0.

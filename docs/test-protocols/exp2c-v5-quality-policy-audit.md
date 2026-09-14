# Exp2c-v5 quality observation audit

EFFECT_EXPLORATION_ONLY / NOT_FORMAL_PAPER_EVIDENCE

Baseline HEAD: 06badcf2bbc758007d2089ae6c4fe651052c51d2.
Existing v4/v5 dirty work and original Y0 evidence are retained. No commit/push.

## Stop-path audit

| Source / condition | Classification / v5 policy |
| --- | --- |
| Estimator runtime rigid-fit residual >0.010 m | QUALITY_THRESHOLD_EXCEEDED; flag, measured residual CSV, continue |
| Initial camera anchoring residual | Not used by v5 odometry fake; unchanged |
| Projector finite distance >0.080 m; finite scalar progress correction | QUALITY_THRESHOLD_EXCEEDED; diagnostic only in explicit v5 policy; convergence and measurement timing remain required |
| Support, lateral, heading, longitudinal, side, progress finite errors | Performance observations; tracker has no standalone hard error thresholds; no new thresholds invented |
| Controller raw >0.150/.155/.160; physical speed limiter | EXECUTION_LIMIT_ACTIVE; count/integrate; actual execution limiter unchanged |
| Causal wheel margin or robust margin =0 | Valid finite risk inputs, not stop conditions; unchanged risk equations |
| Analyzer continuous physical limiter >=0.5 s | Old exploration stop removed for new runs; retain exposure statistic |
| Baseline median+3MAD recovery not reached in 15 s | Censored/null analysis, not runner stop |
| Missing/stale/invalid CooperativeState pose/support/path/load or CapabilityReport; bad freshness timing | STATE_CHAIN_INVALID; existing fail-zero retained |
| Missing/wrong/stale recorder heartbeat during motion; binding/identity/authorization failure | STATE_CHAIN_INVALID; zero/deny execution retained; pre-arm zeros are not performance failures |
| Projector not converged/nonfinite; estimator nonfinite input, bad/backward time, speed/jump impossible for measurement interval | NUMERICAL_INVALID or STATE_CHAIN_INVALID; retained |
| Invalid capability mapping, upper/distributed reference, final reference derivative, lower channel or planar tracker | NUMERICAL_INVALID / structural mapping failure; retained |
| Nonfinite command publication or degraded fake pulse arithmetic | NUMERICAL_INVALID; retained |
| Nonfinite or mathematically degenerate measured support triangle/rigid fit | STRUCTURAL_GEOMETRY_INVALID; retained; relative area tolerance 1e-12 is numerical degeneracy, not mm-quality guard |
| Chassis watchdog/command freshness, emergency, controller invalid, fake plant failure | STATE_CHAIN_INVALID / NUMERICAL_INVALID; actual zero/emergency remains |
| Serial command sequence handoff, battery, feedback, .18 raw demand policy, publication/reconciliation safety gates | Serial-only, never entered in v5 fake; unchanged |
| Existing disconnected-camera observation 0.80 m range latch | Not enabled/reused by v5; unchanged |
| Required pipeline process exit/setup deadline/occupied private master/provenance violation | STATE_CHAIN_INVALID; deny or retain failed run |
| Fixed 95 s motion wall-clock deadline | TASK_TIMEOUT; task_complete=false, recorder closed and failure report retained |
| Reference reaches fixed 5.178229715 m terminal target | Normal latched terminal zero, not failure |
| Generic postprocessing quality validation failure | Retained verbatim, not changed into formal pass; independent exploration report may be usable |

No existing gross numerical-divergence distance guard is used in v5 odometry fake.
No new 10/15/30 mm fatal guard is introduced. Finite performance deterioration
is observed until completion or the fixed task deadline.

## Isolation

`effect_exploration_continue_on_quality_exceedance` defaults false. Enabling it
requires exact v5 experiment ID, fake transport, hardware=false; estimator also
verifies three actual fake chassis transport parameters. New generated run
localization config is separate from the old frozen YAML. Path, geometry,
calibration, risk gains, R1, tracker, wheel envelope and actual plant limiter are
unchanged. Original Y0/v4 outputs are never reused or overwritten.

For quantities without an existing quality threshold, exposure threshold/time
are null; peak/RMS/IAE remain reported. The 10 mm rigid-fit threshold is unchanged.

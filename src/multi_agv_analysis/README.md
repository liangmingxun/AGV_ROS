# multi_agv_analysis

Task 17 data pipeline. It intentionally separates four stages:

1. `record_experiment.py` performs command-authority and `move_base`
   preflight checks, snapshots parameters/configurations and records the bag.
2. `bag_to_csv.py` preserves raw source and bag timestamps in per-message CSV
   files and creates a latest-at-or-before causal alignment table.
3. `validate_run.py` rejects incomplete or inconsistent runs.
4. `compute_metrics.py` refuses an invalid report and computes the frozen
   wheel, path, recovery and task metrics without zero-phase filtering.

Typical offline use:

```bash
rosrun multi_agv_analysis bag_to_csv.py RUN.bag RUN_converted
rosrun multi_agv_analysis validate_run.py \
  RUN_converted RUN/manifest.yaml \
  $(rospack find multi_agv_analysis)/config/validation_defaults.yaml \
  RUN/validation.json
rosrun multi_agv_analysis compute_metrics.py \
  RUN_converted/aligned_samples.csv RUN/manifest.yaml \
  RUN/validation.json RUN/metrics.json
```

The paper-logical one-command wrapper is:

```bash
rosrun multi_agv_analysis process_experiment_run.py RUN_DIR \
  $(rospack find multi_agv_analysis)/config/validation_defaults.yaml
```

It keeps the raw bag and creates/updates `run_meta.json`, the six logical CSV
views, `summary_metrics.json`, validation evidence, and draft figures 2--7 in
PNG and PDF. Invalid runs remain archived and are marked invalid rather than
silently excluded.

Parquet export is optional (`bag_to_csv.py --parquet`) and requires a pandas
Parquet engine such as `pyarrow`. CSV is always the canonical portable output.
Communication-age summaries are diagnostics only; they are not an independent
watchdog-safety claim.

`experiment.launch` deliberately defaults
`nominal_agv2_mapped_capability` to `0.0`, which leaves recovery metrics
undefined. A formal run must pass the frozen, mapped path-channel nominal
capability explicitly; the wheel-speed limit is not a valid substitute.

The recorder does not announce that a run is armed until its dedicated
`rosbag` node is subscribed to every required topic. It then publishes a
5 Hz armed heartbeat together with the exact method ID, allowing a gated
algorithm to fail zero after abrupt recorder loss or a method mismatch. Any
mid-run change in the three `/agvX/chassis_command` publishers terminates
recording and leaves an invalid manifest for audit.

For formal camera runs, pass `camera_mode:=true` and a Windows sender JSON
manifest to `experiment.launch`. The M1+R1 engineering serial entry explicitly
sets `require_windows_sender_manifest:=false`; this does not affect recorded
poses, metrics or plots, but its manifest records the missing Windows-side
provenance. Camera evidence still requires all configured raw/confidence/
filtered streams plus `/vision/aruco/calibration_epoch`.
Validation rejects missing/zero calibration identity and any world-frame
generation change within one run.

`/multi_agv/formal_algorithm_state` is required because Task 13/14 internal
states are part of the reproducibility evidence. `/multi_agv/experiment_state`
is currently recommended rather than mandatory so the fake integration
rehearsal remains usable. If it is absent, metrics explicitly report
`evaluation_window.source=all_samples_fallback` and
`formal_statistics_ready=false`; such output must not enter paper statistics.

Validation also enforces configured minimum topic rates. Streams frozen at
100 Hz use a 90 Hz acceptance floor, which catches materially under-rate logs
while retaining margin for normal scheduling and rosbag jitter.

## Parameter-independent support tools

The package also provides fail-closed tools that do not require checked-in
physical calibration values:

- `calibration_workspace.py`: current dynamic-ground tag-to-target candidates
  plus optional later alignment to a surveyed laboratory frame;
- `experiment_batch.py`: deterministic paired-block run planning;
- `aggregate_experiment.py`: paired summaries, bootstrap intervals, sign-flip
  tests, Markdown tables and optional SVG figures;
- `manage_stage_evidence.py`: ordered Stage A-D evidence and non-authorizing
  approval candidates;
- `fleet_health_check.py`: read-only inventory and ROS graph checks;
- `software_watchdog.py`: observer-only receipt-age/authority diagnostics.

The calibration and approval outputs are candidates only. They cannot set a
camera, hardware or formal-statistics authorization flag.

`camera_quality_report.py CONVERTED_DIR REPORT.json` produces descriptive
per-topic rate, arrival-delay, timestamp-order, long-gap, confidence and
raw/filtered acceptance summaries. It always emits
`quality_authorized=false`; physical thresholds remain a separate reviewed
policy.

The aligned metric pipeline also exports per-robot path errors, planar
support/load errors, rigid-fit residuals, capability/demand margins,
speed/acceleration/deceleration limiter durations, R1 internal-state extrema
and M2b `delta_z/delta_w`. M2b runs conditionally require the dedicated
`/multi_agv/m2b_algorithm_state` stream.

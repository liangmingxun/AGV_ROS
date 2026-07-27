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

Parquet export is optional (`bag_to_csv.py --parquet`) and requires a pandas
Parquet engine such as `pyarrow`. CSV is always the canonical portable output.
Communication-age summaries are diagnostics only; they are not an independent
watchdog-safety claim.

`experiment.launch` deliberately defaults
`nominal_agv2_mapped_capability` to `0.0`, which leaves recovery metrics
undefined. A formal run must pass the frozen, mapped path-channel nominal
capability explicitly; the wheel-speed limit is not a valid substitute.

The recorder does not announce that a run is armed until its dedicated
`rosbag` node is subscribed to every required topic. Any mid-run change in
the three `/agvX/chassis_command` publishers terminates recording and leaves
an invalid manifest for audit.

`/multi_agv/formal_algorithm_state` is required because Task 13/14 internal
states are part of the reproducibility evidence. `/multi_agv/experiment_state`
is currently recommended rather than mandatory so the fake integration
rehearsal remains usable. If it is absent, metrics explicitly report
`evaluation_window.source=all_samples_fallback` and
`formal_statistics_ready=false`; such output must not enter paper statistics.

Validation also enforces configured minimum topic rates. Streams frozen at
100 Hz use a 90 Hz acceptance floor, which catches materially under-rate logs
while retaining margin for normal scheduling and rosbag jitter.

#!/usr/bin/env python3

import argparse
import sys

from multi_agv_analysis.io_utils import (
    atomic_dump_json,
    load_yaml,
    read_csv,
)
from multi_agv_analysis.metrics import compute_metrics
from multi_agv_analysis.payload import resolve_payload_context


def main():
    parser = argparse.ArgumentParser(
        description="Compute frozen causal metrics after run validation.")
    parser.add_argument("aligned_csv")
    parser.add_argument("manifest")
    parser.add_argument("validation_report")
    parser.add_argument("output")
    parser.add_argument("--sample-period", type=float, default=0.01)
    parser.add_argument("--command-epsilon", type=float, default=1e-6)
    parser.add_argument("--nominal-agv2-capability", type=float)
    parser.add_argument("--nominal-common-velocity", type=float)
    parser.add_argument("--evaluation-target", type=float)
    parser.add_argument("--publication-mode", action="store_true")
    arguments = parser.parse_args()

    validation = load_yaml(arguments.validation_report)
    if not validation.get("valid", False):
        print("ERROR: validation report is not valid; metrics refused",
              file=sys.stderr)
        return 3
    manifest = load_yaml(arguments.manifest)
    metrics_config = manifest.get("metrics", {})
    rows = read_csv(arguments.aligned_csv)
    payload_context = resolve_payload_context(
        manifest, rows, publication_mode=arguments.publication_mode)
    result = compute_metrics(
        rows,
        sample_period=arguments.sample_period,
        command_epsilon=arguments.command_epsilon,
        nominal_agv2_capability=(
            arguments.nominal_agv2_capability
            if arguments.nominal_agv2_capability is not None
            else metrics_config.get("nominal_agv2_capability")),
        nominal_common_velocity=(
            arguments.nominal_common_velocity
            if arguments.nominal_common_velocity is not None
            else metrics_config.get("nominal_common_velocity")),
        evaluation_target=(
            arguments.evaluation_target
            if arguments.evaluation_target is not None
            else metrics_config.get("evaluation_target")),
        payload_context=payload_context)
    result["run_id"] = manifest.get("run_id", "")
    result["experiment_id"] = manifest.get("experiment_id", "")
    result["method_id"] = manifest.get("method_id", "")
    result["payload_context"] = payload_context
    atomic_dump_json(arguments.output, result)
    print("METRICS WRITTEN: {}".format(arguments.output))
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3

"""One-command causal conversion, validation, metrics, CSV views and plots."""

import argparse
import sys
from pathlib import Path

from multi_agv_analysis.conversion import convert_bag
from multi_agv_analysis.io_utils import (
    atomic_dump_json, load_yaml, read_csv)
from multi_agv_analysis.metrics import compute_metrics
from multi_agv_analysis.paper_pipeline import export_views, plot_run
from multi_agv_analysis.payload import resolve_payload_context
from multi_agv_analysis.validation import validate_converted_run


PROCESSING_VERSION = "paper_run_pipeline_v11_stage_d_payload_context"

RAW_DISPLAY_PROFILE = {
    "display": {
        "smoothing_window_seconds": 0.0,
        "wheel_feedback_smoothing_window_seconds": 0.0,
        "show_raw_samples": False,
        "maximum_plot_rate_hz": 1000.0,
    },
}

SMOOTHED_DISPLAY_PROFILE = {
    "display": {
        "smoothing_window_seconds": 0.80,
        "wheel_feedback_smoothing_window_seconds": 0.80,
        "show_raw_samples": False,
        "maximum_plot_rate_hz": 25.0,
    },
}


def battery_edges(converted):
    rows = read_csv(converted / "chassis_feedback.csv")
    grouped = {index: [] for index in range(1, 4)}
    for row in rows:
        try:
            grouped[int(row["robot_id"])].append(float(row["battery_voltage"]))
        except (KeyError, TypeError, ValueError):
            pass
    start = {"agv{}".format(index): values[0] if values else None
             for index, values in grouped.items()}
    end = {"agv{}".format(index): values[-1] if values else None
           for index, values in grouped.items()}
    return start, end


def update_meta(run_dir, valid, abort_reason=""):
    path = run_dir / "run_meta.json"
    meta = load_yaml(path) if path.exists() else {}
    start, end = battery_edges(run_dir / "converted")
    meta.update({
        "processing_version": PROCESSING_VERSION,
        "battery_voltage_start": start,
        "battery_voltage_end": end,
        "valid_run": bool(valid),
        "abort_reason": abort_reason,
    })
    atomic_dump_json(path, meta)


def publication_provenance(run_meta, manifest, aligned, payload_context):
    ratios = []
    for row in aligned:
        try:
            value = float(row.get("agv2_derating_speed_ratio"))
            if value > 0.0:
                ratios.append(value)
        except (TypeError, ValueError):
            pass
    metrics = manifest.get("metrics", {})
    return {
        "git_commit": run_meta.get("git_sha", manifest.get("git_sha", "")),
        "config_hashes": run_meta.get(
            "config_hashes", manifest.get("config_hashes", [])),
        "wheel_speed_scale_freeze_id": payload_context.get(
            "wheel_speed_scale_freeze_id", ""),
        "wheel_speed_scale_freeze_snapshot_present": payload_context.get(
            "wheel_speed_scale_freeze_snapshot_present", False),
        "method_id": run_meta.get(
            "method_id", manifest.get("method_id", "")),
        "experiment_id": run_meta.get(
            "experiment_id", manifest.get("experiment_id", "")),
        "agv2_derating_ratio_observed": min(ratios) if ratios else 1.0,
        "nominal_reference_speed_mps": metrics.get(
            "nominal_common_velocity"),
        "run_timestamp": run_meta.get(
            "start_timestamp", manifest.get("started_at")),
        "payload_mode": payload_context.get("payload_mode"),
        "load_metric_source": payload_context.get("load_metric_source"),
    }


def main():
    parser = argparse.ArgumentParser(
        description="Rebuild all paper data products from one raw bag.")
    parser.add_argument("run_dir")
    parser.add_argument("validation_rules")
    parser.add_argument("--skip-plots", action="store_true")
    parser.add_argument(
        "--publication-mode", action="store_true",
        help="Require explicit Stage-D and payload source provenance.")
    args = parser.parse_args()
    run_dir = Path(args.run_dir).resolve()
    status_path = run_dir / "postprocess_status.json"
    status = {
        "schema_version": 1,
        "conversion": "pending",
        "validation": "pending",
        "metrics": "pending",
        "plots": "skipped" if args.skip_plots else "pending",
        "plots_raw": "skipped" if args.skip_plots else "pending",
        "plots_smoothed_0p8s": "skipped" if args.skip_plots else "pending",
    }
    try:
        manifest_path = run_dir / "manifest.yaml"
        manifest = load_yaml(manifest_path)
        bag_name = manifest.get("bag", "")
        bag_path = run_dir / bag_name
        if not bag_name or not bag_path.is_file():
            bags = list(run_dir.glob("*.bag"))
            if len(bags) != 1:
                raise RuntimeError("run must contain exactly one raw bag")
            bag_path = bags[0]
        converted = run_dir / "converted"
        payload_metadata = manifest.get("payload", {})
        measured_payload_pose_topic = (
            payload_metadata.get("measured_pose_topic")
            if isinstance(payload_metadata, dict) else None)
        convert_bag(
            bag_path, converted,
            measured_payload_pose_topic=(
                measured_payload_pose_topic or
                "/pose_provider/load/pose_filtered"))
        status["conversion"] = "passed"
        rules = load_yaml(args.validation_rules).get(
            "experiment_recording", {})
        validation = validate_converted_run(
            converted, manifest_path, rules)
        atomic_dump_json(run_dir / "validation.json", validation)
        if not validation.get("valid", False):
            status["validation"] = "failed"
            atomic_dump_json(status_path, status)
            reason = "; ".join(
                item.get("code", "INVALID")
                for item in validation.get("issues", []))
            update_meta(run_dir, False, reason)
            print("RUN PIPELINE REFUSED INVALID RUN: {}".format(reason),
                  file=sys.stderr)
            return 4
        status["validation"] = "passed"
        aligned = read_csv(converted / "aligned_samples.csv")
        run_meta_path = run_dir / "run_meta.json"
        run_meta = load_yaml(run_meta_path) if run_meta_path.exists() else {}
        payload_context = resolve_payload_context(
            run_meta, aligned, publication_mode=args.publication_mode)
        metrics_cfg = manifest.get("metrics", {})
        metrics = compute_metrics(
            aligned,
            sample_period=float(rules.get("sample_period", 0.01)),
            nominal_agv2_capability=metrics_cfg.get(
                "nominal_agv2_capability"),
            nominal_common_velocity=metrics_cfg.get(
                "nominal_common_velocity"),
            evaluation_target=metrics_cfg.get("evaluation_target"),
            payload_context=payload_context)
        metrics.update({
            "run_id": manifest.get("run_id", ""),
            "experiment_id": manifest.get("experiment_id", ""),
            "method_id": manifest.get("method_id", ""),
            "processing_version": PROCESSING_VERSION,
            "payload_context": payload_context,
            "publication_provenance": publication_provenance(
                run_meta, manifest, aligned, payload_context),
        })
        atomic_dump_json(run_dir / "summary_metrics.json", metrics)
        export_views(converted, run_dir)
        status["metrics"] = "passed"
        update_meta(run_dir, True)
        atomic_dump_json(status_path, status)
        if not args.skip_plots:
            try:
                plot_run(
                    converted / "aligned_samples.csv", run_dir / "plots_raw",
                    RAW_DISPLAY_PROFILE,
                    publication_mode=args.publication_mode)
                status["plots_raw"] = "passed"
                atomic_dump_json(status_path, status)
                plot_run(
                    converted / "aligned_samples.csv",
                    run_dir / "plots_smoothed_0p8s",
                    SMOOTHED_DISPLAY_PROFILE,
                    publication_mode=args.publication_mode)
                status["plots_smoothed_0p8s"] = "passed"
                status["plots"] = "passed"
            except Exception as error:
                if status["plots_raw"] == "pending":
                    status["plots_raw"] = "failed"
                elif status["plots_smoothed_0p8s"] == "pending":
                    status["plots_smoothed_0p8s"] = "failed"
                status["plots"] = "failed"
                status["plot_error"] = str(error)
                atomic_dump_json(status_path, status)
                print(
                    "PAPER RUN DATA PASSED; PLOT GENERATION FAILED: {}".format(
                        error), file=sys.stderr)
                return 5
        atomic_dump_json(status_path, status)
        print("PAPER RUN PIPELINE: PASSED")
        print("run_dir={}".format(run_dir))
        return 0
    except Exception as error:
        status["pipeline_error"] = str(error)
        if run_dir.exists():
            try:
                update_meta(run_dir, False, str(error))
                atomic_dump_json(status_path, status)
            except Exception:
                pass
        print("PAPER RUN PIPELINE FAILED: {}".format(error), file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())

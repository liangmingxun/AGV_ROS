"""Descriptive, non-authorizing quality summaries for Task-15 camera CSVs."""

import math
from collections import defaultdict
from pathlib import Path

from .io_utils import finite_float, read_csv


def _quantile(values, probability):
    values = sorted(value for value in values if math.isfinite(value))
    if not values:
        return None
    position = probability * (len(values) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    weight = position - lower
    return values[lower] * (1.0 - weight) + values[upper] * weight


def _stream(rows):
    source_order = [finite_float(row.get("header_stamp")) for row in rows]
    finite_stamps = [value for value in source_order if math.isfinite(value)]
    ordered = sorted(finite_stamps)
    gaps = [right - left for left, right in zip(ordered, ordered[1:])
            if right >= left]
    delays = [
        finite_float(row.get("bag_stamp")) -
        finite_float(row.get("header_stamp"))
        for row in rows]
    delays = [value for value in delays if math.isfinite(value)]
    duration = ordered[-1] - ordered[0] if len(ordered) > 1 else 0.0
    median_gap = _quantile(gaps, 0.5)
    return {
        "samples": len(rows),
        "duration": duration,
        "observed_rate": (
            (len(ordered) - 1) / duration if duration > 0.0 else None),
        "backward_or_duplicate_stamps": sum(
            right <= left for left, right in
            zip(source_order, source_order[1:])
            if math.isfinite(left) and math.isfinite(right)),
        "gap_median": median_gap,
        "gap_p95": _quantile(gaps, 0.95),
        "gap_max": max(gaps, default=None),
        "large_gap_count": (
            sum(gap > 1.5 * median_gap for gap in gaps)
            if median_gap is not None and median_gap > 0.0 else 0),
        "arrival_delay_mean": (
            sum(delays) / len(delays) if delays else None),
        "arrival_delay_p95": _quantile(delays, 0.95),
        "arrival_delay_max": max(delays, default=None),
    }


def summarize_camera_quality(converted_dir):
    root = Path(converted_dir)
    pose_path = root / "camera_pose.csv"
    confidence_path = root / "camera_confidence.csv"
    epoch_path = root / "camera_calibration_epoch.csv"
    pose_rows = read_csv(pose_path) if pose_path.exists() else []
    confidence_rows = (
        read_csv(confidence_path) if confidence_path.exists() else [])
    epoch_rows = read_csv(epoch_path) if epoch_path.exists() else []
    poses = defaultdict(list)
    confidence = defaultdict(list)
    for row in pose_rows:
        poses[row.get("topic", "")].append(row)
    for row in confidence_rows:
        confidence[row.get("topic", "")].append(
            finite_float(row.get("confidence")))
    streams = {topic: _stream(rows) for topic, rows in sorted(poses.items())}
    confidence_summary = {}
    for topic, values in sorted(confidence.items()):
        values = [value for value in values if math.isfinite(value)]
        confidence_summary[topic] = {
            "samples": len(values),
            "minimum": min(values, default=None),
            "mean": sum(values) / len(values) if values else None,
            "p05": _quantile(values, 0.05),
            "p95": _quantile(values, 0.95),
            "maximum": max(values, default=None),
        }
    acceptance = {}
    for entity, target in (
            ("agv1", "base_pose"), ("agv2", "base_pose"),
            ("agv3", "base_pose"), ("load", "pose")):
        raw = len(poses.get(
            "/pose_provider/{}/{}_raw".format(entity, target), []))
        filtered = len(poses.get(
            "/pose_provider/{}/{}_filtered".format(entity, target), []))
        acceptance[entity] = {
            "raw_samples": raw,
            "filtered_samples": filtered,
            "filtered_to_raw_ratio": (
                filtered / raw if raw else None),
        }
    epoch_values = []
    for row in epoch_rows:
        value = str(row.get("epoch_unix_ns", "")).strip()
        if value and value not in epoch_values:
            epoch_values.append(value)
    epoch_tokens = []
    for row in pose_rows:
        topic = str(row.get("topic", ""))
        if (topic.startswith("/camera/world/") and
                topic.endswith("_tag_pose")):
            token = str(
                row.get("calibration_epoch_token", "")).strip()
            if token not in ("", "0") and token not in epoch_tokens:
                epoch_tokens.append(token)
    return {
        "schema_version": 2,
        "quality_authorized": False,
        "threshold_policy_applied": False,
        "streams": streams,
        "confidence": confidence_summary,
        "filter_acceptance": acceptance,
        "calibration_epoch": {
            "full_epoch_samples": len(epoch_rows),
            "unique_full_epochs": epoch_values,
            "unique_in_band_tokens": epoch_tokens,
            "continuous": (
                len(epoch_values) == 1 and len(epoch_tokens) == 1),
        },
        "warning": (
            "Descriptive evidence only. Freeze physical thresholds in a "
            "separately reviewed policy before calibration authorization."),
    }

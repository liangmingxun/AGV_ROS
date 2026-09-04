"""Pre-metric validity audit for converted experiment runs."""

import math
from collections import defaultdict
from pathlib import Path

from .io_utils import (
    bool_value,
    finite_float,
    load_yaml,
    read_csv,
    sha256_file,
)


UINT32_MODULUS = 1 << 32


def _issue(code, detail):
    return {"code": code, "detail": detail}


def _load_rows(converted_dir, filename):
    path = Path(converted_dir) / filename
    return read_csv(path) if path.exists() else []


def _check_monotonic(rows, filename, issues):
    previous = {}
    for row_index, row in enumerate(rows, start=2):
        topic = row.get("topic", filename)
        stamp = finite_float(row.get("header_stamp"))
        if not math.isfinite(stamp):
            stamp = finite_float(row.get("stamp"))
        if not math.isfinite(stamp):
            issues.append(_issue(
                "INVALID_STAMP",
                "{} row {} has no finite source stamp".format(
                    filename, row_index)))
            continue
        if topic in previous and stamp < previous[topic] - 1e-12:
            issues.append(_issue(
                "BACKWARD_STAMP",
                "{} topic {} moved backward: {:.9f} -> {:.9f}".format(
                    filename, topic, previous[topic], stamp)))
        previous[topic] = stamp


def _sequence_gap(current, previous, allow_wrap):
    if allow_wrap:
        return (current - previous) % UINT32_MODULUS
    return current - previous


def _check_sequence_rule(converted_dir, rule, issues):
    filename = rule["file"]
    field = rule["field"]
    group_fields = rule.get("group_by", [])
    if isinstance(group_fields, str):
        group_fields = [group_fields]
    max_gap = int(rule["max_gap"])
    allow_wrap = bool(rule.get("allow_wrap", True))
    rows = _load_rows(converted_dir, filename)
    previous = {}
    for row_index, row in enumerate(rows, start=2):
        try:
            value = int(row[field])
        except (KeyError, TypeError, ValueError):
            issues.append(_issue(
                "INVALID_SEQUENCE",
                "{} row {} has invalid {}".format(
                    filename, row_index, field)))
            continue
        key = tuple(row.get(name, "") for name in group_fields)
        if key in previous:
            gap = _sequence_gap(value, previous[key], allow_wrap)
            if gap == 0 and rule.get("reject_duplicates", False):
                issues.append(_issue(
                    "DUPLICATE_SEQUENCE",
                    "{} {} duplicated for group {}".format(
                        filename, field, key)))
            elif gap > max_gap:
                issues.append(_issue(
                    "SEQUENCE_GAP",
                    "{} {} gap {} exceeds {} for group {}".format(
                        filename, field, gap, max_gap, key)))
        previous[key] = value


def _invalid_localization_statistics(rows, sample_period):
    invalid = []
    for row in rows:
        if "localization_valid" in row:
            valid = bool_value(row.get("localization_valid"))
        else:
            valid = bool_value(row.get("load_pose_valid"))
            valid = valid and bool_value(row.get("load_path_state_valid"))
            for index in range(1, 4):
                valid = valid and bool_value(
                    row.get("robot_pose_valid_{}".format(index)))
                valid = valid and bool_value(
                    row.get("support_pose_valid_{}".format(index)))
                valid = valid and bool_value(
                    row.get("path_state_valid_{}".format(index)))
        invalid.append(not valid)

    longest = current = 0
    for value in invalid:
        current = current + 1 if value else 0
        longest = max(longest, current)
    return {
        "samples": len(rows),
        "invalid_samples": sum(invalid),
        "invalid_fraction": (
            float(sum(invalid)) / len(rows) if rows else 1.0),
        "maximum_consecutive_invalid_seconds": longest * sample_period,
    }


def _invalid_algorithm_statistics(rows, sample_period):
    invalid = [
        not bool_value(row.get("algorithm_state_available", False)) or
        not bool_value(row.get("algorithm_valid", False))
        for row in rows]
    longest = current = 0
    for value in invalid:
        current = current + 1 if value else 0
        longest = max(longest, current)
    return {
        "samples": len(rows),
        "invalid_samples": sum(invalid),
        "invalid_fraction": (
            float(sum(invalid)) / len(rows) if rows else 1.0),
        "maximum_consecutive_invalid_seconds": longest * sample_period,
    }


def _check_task_completion(aligned_rows, manifest, required, issues):
    target = finite_float(manifest.get("metrics", {}).get(
        "evaluation_target"))
    completion_stamp = None
    if math.isfinite(target):
        for row in aligned_rows:
            if finite_float(row.get("load_s_actual")) >= target:
                completion_stamp = finite_float(row.get("stamp"))
                break
    if required and not math.isfinite(target):
        issues.append(_issue(
            "TASK_TARGET_MISSING",
            "task completion is required but evaluation_target is absent"))
    elif required and completion_stamp is None:
        issues.append(_issue(
            "TASK_INCOMPLETE",
            "load progress never reached evaluation target {:.6f}".format(
                target)))
    return {
        "required": bool(required),
        "target": target if math.isfinite(target) else None,
        "completion_stamp": completion_stamp,
        "complete": completion_stamp is not None,
    }


def _communication_age_report(converted_dir):
    report = {}
    for filename in ("chassis_feedback.csv", "capability_report.csv",
                     "chassis_command.csv", "cooperative_state.csv"):
        values = []
        for row in _load_rows(converted_dir, filename):
            receipt = finite_float(row.get("bag_stamp"))
            source = finite_float(row.get("header_stamp"))
            if math.isfinite(receipt) and math.isfinite(source):
                values.append(receipt - source)
        if values:
            report[filename] = {
                "count": len(values),
                "minimum": min(values),
                "mean": sum(values) / len(values),
                "maximum": max(values),
                "note": (
                    "Observed bag-receipt minus source-stamp age; this is "
                    "not evidence of an independent communication watchdog."),
            }
    return report


def _check_methods(converted_dir, manifest, issues):
    expected_experiment = str(manifest.get("experiment_id", ""))
    expected_method = str(manifest.get("method_id", ""))
    for filename in ("chassis_command.csv", "controller_state.csv",
                     "experiment_state.csv"):
        for row in _load_rows(converted_dir, filename):
            method = str(row.get("method_id", ""))
            experiment = str(row.get("experiment_id", ""))
            if expected_method and method != expected_method:
                issues.append(_issue(
                    "METHOD_MISMATCH",
                    "{} contains method {} instead of {}".format(
                        filename, method or "<empty>", expected_method)))
                break
            if (expected_experiment and
                    experiment != expected_experiment):
                issues.append(_issue(
                    "EXPERIMENT_MISMATCH",
                    "{} contains experiment {} instead of {}".format(
                        filename, experiment or "<empty>",
                        expected_experiment)))
                break


def _check_manual_abort(converted_dir, issues):
    for row in _load_rows(converted_dir, "experiment_state.csv"):
        if bool_value(row.get("manual_abort", False)):
            issues.append(_issue(
                "MANUAL_ABORT",
                "run contains a manual abort: {}".format(
                    row.get("abort_reason", ""))))
            return


def _check_hashes(manifest_path, manifest, issues):
    base = Path(manifest_path).resolve().parent
    for entry in manifest.get("config_hashes", []):
        relative = entry.get("archived_path") or entry.get("path")
        expected = str(entry.get("sha256", ""))
        path = Path(relative)
        if not path.is_absolute():
            path = base / path
        if not path.is_file():
            issues.append(_issue(
                "CONFIG_MISSING", "configuration snapshot missing: {}".format(
                    path)))
        elif not expected or sha256_file(path) != expected:
            issues.append(_issue(
                "CONFIG_HASH_MISMATCH",
                "configuration hash mismatch: {}".format(path)))


def _check_authority(manifest, issues):
    authority = manifest.get("command_authority", {})
    for index in range(1, 4):
        topic = "/agv{}/chassis_command".format(index)
        publishers = authority.get(topic)
        if not isinstance(publishers, list) or len(publishers) != 1:
            issues.append(_issue(
                "COMMAND_AUTHORITY",
                "{} must have exactly one recorded publisher, got {}".format(
                    topic, publishers)))
            continue
        if any("move_base" in publisher for publisher in publishers):
            issues.append(_issue(
                "MOVE_BASE_AUTHORITY",
                "move_base is forbidden on {}".format(topic)))
    violations = manifest.get("command_authority_violations", [])
    if violations:
        issues.append(_issue(
            "COMMAND_AUTHORITY_CHANGED",
            "command authority changed {} time(s) during the run".format(
                len(violations))))


def _check_formal_statistics_gate(
        converted_dir, manifest, issues, warnings):
    requested = bool(manifest.get("formal_statistics_requested", False))
    approval = manifest.get("configuration_approval")
    rows = _load_rows(converted_dir, "experiment_state.csv")
    active = [
        index for index, row in enumerate(rows)
        if bool_value(row.get("evaluation_active", False))]
    bounded = bool(active)
    if bounded:
        bounded = (
            active[0] > 0 and active[-1] < len(rows) - 1 and
            active == list(range(active[0], active[-1] + 1)))
    if requested:
        if not isinstance(approval, dict):
            issues.append(_issue(
                "CONFIG_APPROVAL_MISSING",
                "formal statistics were requested without an approval"))
        elif not approval.get("formal_statistics_authorized", False):
            issues.append(_issue(
                "CONFIG_APPROVAL_NOT_FORMAL",
                "selected approval does not authorize formal statistics"))
        if not rows:
            issues.append(_issue(
                "EVALUATION_WINDOW_MISSING",
                "formal statistics require ExperimentState"))
        elif not bounded:
            issues.append(_issue(
                "EVALUATION_WINDOW_UNBOUNDED",
                "formal evaluation window must have inactive samples before "
                "and after one contiguous active interval"))
    elif not bounded:
        warnings.append(_issue(
            "DRESS_REHEARSAL_ONLY",
            "no bounded formal evaluation window; outputs are not "
            "formal-statistics ready"))
    return {
        "formal_statistics_requested": requested,
        "configuration_approval_present": isinstance(approval, dict),
        "samples": len(rows),
        "active_samples": len(active),
        "bounded_and_contiguous": bounded,
    }


def _check_topic_rates(inventory, rules, issues):
    report = {}
    topics = inventory.get("topics", {})
    for topic, minimum in rules.get("minimum_topic_rates", {}).items():
        minimum = finite_float(minimum)
        entry = topics.get(topic)
        if entry is None:
            continue
        messages = finite_float(entry.get("messages"))
        first = finite_float(entry.get("first_bag_stamp"))
        last = finite_float(entry.get("last_bag_stamp"))
        if (not math.isfinite(minimum) or minimum <= 0.0 or
                not math.isfinite(messages) or messages < 2.0 or
                not math.isfinite(first) or not math.isfinite(last) or
                last <= first):
            issues.append(_issue(
                "TOPIC_RATE_UNAVAILABLE",
                "{} has insufficient inventory data for its rate gate".format(
                    topic)))
            continue
        observed = (messages - 1.0) / (last - first)
        report[topic] = {
            "messages": int(messages),
            "duration": last - first,
            "observed": observed,
            "minimum": minimum,
        }
        if observed < minimum:
            issues.append(_issue(
                "TOPIC_RATE_LOW",
                "{} rate {:.3f} Hz is below {:.3f} Hz".format(
                    topic, observed, minimum)))
    return report


def _check_camera_evidence(
        converted_dir, inventory, rules, issues, virtual_load_from_robots=False):
    camera_used = False
    fused_used = False
    for row in _load_rows(converted_dir, "cooperative_state.csv"):
        sources = [finite_float(row.get("load_localization_source"))]
        sources.extend(
            finite_float(row.get(
                "robot_localization_source_{}".format(index)))
            for index in range(1, 4))
        if any(source in (2.0, 3.0) for source in sources):
            camera_used = True
        if any(source == 3.0 for source in sources):
            fused_used = True
    missing = []
    epoch_values = []
    epoch_tokens = []
    camera_topic_key = (
        "camera_virtual_load_required_topics"
        if virtual_load_from_robots else "camera_required_topics")
    if camera_used:
        observed = set(inventory.get("topics", {}))
        missing = [
            topic for topic in rules.get(camera_topic_key, [])
            if topic not in observed]
        if fused_used:
            missing.extend(
                topic for topic in rules.get("fused_required_topics", [])
                if topic not in observed)
        missing = list(dict.fromkeys(missing))
        for topic in missing:
            issues.append(_issue(
                "CAMERA_EVIDENCE_MISSING",
                "camera/fused localization was used but raw evidence is "
                "absent: {}".format(topic)))
        if rules.get("camera_calibration_epoch_required", False):
            for row in _load_rows(
                    converted_dir, "camera_calibration_epoch.csv"):
                value = str(row.get("epoch_unix_ns", "")).strip()
                if value and value not in epoch_values:
                    epoch_values.append(value)
            for row in _load_rows(converted_dir, "camera_pose.csv"):
                topic = str(row.get("topic", ""))
                if (not topic.startswith("/camera/world/") or
                        not topic.endswith("_tag_pose")):
                    continue
                token = str(
                    row.get("calibration_epoch_token", "")).strip()
                if token not in ("", "0") and token not in epoch_tokens:
                    epoch_tokens.append(token)
            if not epoch_values:
                issues.append(_issue(
                    "CAMERA_CALIBRATION_EPOCH_MISSING",
                    "camera localization has no recorded full calibration "
                    "epoch"))
            if not epoch_tokens:
                issues.append(_issue(
                    "CAMERA_CALIBRATION_TOKEN_MISSING",
                    "camera raw poses have no nonzero in-band calibration "
                    "epoch token"))
            maximum_changes = int(rules.get(
                "maximum_camera_calibration_epoch_changes", 0))
            if len(epoch_values) - 1 > maximum_changes:
                issues.append(_issue(
                    "CAMERA_CALIBRATION_EPOCH_CHANGED",
                    "recorded {} calibration epochs; at most {} change(s) "
                    "are allowed".format(
                        len(epoch_values), maximum_changes)))
            if len(epoch_tokens) - 1 > maximum_changes:
                issues.append(_issue(
                    "CAMERA_CALIBRATION_TOKEN_CHANGED",
                    "raw poses contain {} world-frame generation tokens; "
                    "at most {} change(s) are allowed".format(
                        len(epoch_tokens), maximum_changes)))
    return {
        "camera_or_fused_used": camera_used,
        "fused_used": fused_used,
        "required_topics": (
            list(dict.fromkeys(
                list(rules.get(camera_topic_key, [])) +
                (list(rules.get("fused_required_topics", []))
                 if fused_used else [])))
            if camera_used else []),
        "missing_topics": missing,
        "calibration_epochs": epoch_values,
        "calibration_epoch_tokens": epoch_tokens,
    }


def validate_converted_run(converted_dir, manifest_path, rules):
    """Return a structured audit; callers must reject ``valid == False``."""
    converted_dir = Path(converted_dir)
    manifest = load_yaml(manifest_path)
    issues = []
    warnings = []

    inventory_path = converted_dir / "topic_inventory.yaml"
    inventory = load_yaml(inventory_path) if inventory_path.exists() else {}
    observed_topics = set(inventory.get("topics", {}))
    required_topics = list(rules.get("required_topics", []))
    required_topics.extend(
        rules.get("method_required_topics", {}).get(
            manifest.get("method_id", ""), []))
    required_topics = list(dict.fromkeys(required_topics))
    for topic in required_topics:
        if topic not in observed_topics:
            issues.append(_issue(
                "MISSING_TOPIC", "required topic absent: {}".format(topic)))
    for topic in rules.get("recommended_topics", []):
        if topic not in observed_topics:
            warnings.append(_issue(
                "MISSING_RECOMMENDED_TOPIC",
                "recommended topic absent: {}; metrics using this stream "
                "are not formal-statistics ready".format(topic)))
    topic_rates = _check_topic_rates(inventory, rules, issues)
    camera_evidence = _check_camera_evidence(
        converted_dir, inventory, rules, issues,
        bool(manifest.get("virtual_load_from_robots", False)))

    raw_files = rules.get("stamp_files", [
        "chassis_command.csv",
        "chassis_feedback.csv",
        "capability_report.csv",
        "cooperative_state.csv",
        "path_reference.csv",
        "controller_state.csv",
    ])
    for filename in raw_files:
        rows = _load_rows(converted_dir, filename)
        if rows:
            _check_monotonic(rows, filename, issues)

    for rule in rules.get("sequence_rules", []):
        _check_sequence_rule(converted_dir, rule, issues)

    sample_period = float(rules.get("sample_period", 0.01))
    cooperative_rows = _load_rows(converted_dir, "cooperative_state.csv")
    aligned_rows = _load_rows(converted_dir, "aligned_samples.csv")
    experiment_state_rows = _load_rows(
        converted_dir, "experiment_state.csv")
    if experiment_state_rows:
        localization_rows = [
            row for row in aligned_rows
            if bool_value(row.get("experiment_state_available", False)) and
            bool_value(row.get("evaluation_active", False))]
        localization_scope = "evaluation_active"
        excluded_localization_samples = (
            len(aligned_rows) - len(localization_rows))
    else:
        # Legacy or engineering bags without ExperimentState retain the old
        # conservative whole-recording audit.  Missing window metadata must
        # never silently hide an invalid interval.
        localization_rows = cooperative_rows
        localization_scope = "full_recording_fallback"
        excluded_localization_samples = 0
    localization = _invalid_localization_statistics(
        localization_rows, sample_period)
    localization["scope"] = localization_scope
    localization["excluded_samples"] = excluded_localization_samples
    exclusion = rules.get("localization_exclusion", {})
    if (localization["invalid_fraction"] >
            float(exclusion.get("maximum_invalid_fraction", 0.0))):
        issues.append(_issue(
            "LOCALIZATION_INVALID_FRACTION",
            "invalid localization fraction {:.6f} exceeds {:.6f}".format(
                localization["invalid_fraction"],
                float(exclusion.get("maximum_invalid_fraction", 0.0)))))
    if (localization["maximum_consecutive_invalid_seconds"] >
            float(exclusion.get(
                "maximum_consecutive_invalid_seconds", 0.0))):
        issues.append(_issue(
            "LOCALIZATION_INVALID_DURATION",
            "consecutive invalid localization {:.6f}s exceeds {:.6f}s".format(
                localization["maximum_consecutive_invalid_seconds"],
                float(exclusion.get(
                    "maximum_consecutive_invalid_seconds", 0.0)))))

    algorithm = None
    algorithm_exclusion = rules.get("algorithm_exclusion")
    if algorithm_exclusion is not None:
        algorithm_rows = (
            localization_rows if experiment_state_rows else aligned_rows)
        algorithm = _invalid_algorithm_statistics(
            algorithm_rows, sample_period)
        maximum_fraction = float(algorithm_exclusion.get(
            "maximum_invalid_fraction", 0.0))
        maximum_duration = float(algorithm_exclusion.get(
            "maximum_consecutive_invalid_seconds", 0.0))
        if algorithm["invalid_fraction"] > maximum_fraction:
            issues.append(_issue(
                "ALGORITHM_INVALID_FRACTION",
                "invalid algorithm fraction {:.6f} exceeds {:.6f}".format(
                    algorithm["invalid_fraction"], maximum_fraction)))
        if (algorithm["maximum_consecutive_invalid_seconds"] >
                maximum_duration):
            issues.append(_issue(
                "ALGORITHM_INVALID_DURATION",
                "consecutive invalid algorithm state {:.6f}s exceeds "
                "{:.6f}s".format(
                    algorithm["maximum_consecutive_invalid_seconds"],
                    maximum_duration)))

    task_completion = _check_task_completion(
        aligned_rows, manifest,
        bool(rules.get("require_task_completion", False)), issues)

    _check_methods(converted_dir, manifest, issues)
    _check_manual_abort(converted_dir, issues)
    _check_hashes(manifest_path, manifest, issues)
    _check_authority(manifest, issues)
    formal_gate = _check_formal_statistics_gate(
        converted_dir, manifest, issues, warnings)

    return {
        "schema_version": 1,
        "valid": not issues,
        "issues": issues,
        "warnings": warnings,
        "topic_rates": topic_rates,
        "camera_evidence": camera_evidence,
        "localization": localization,
        "algorithm": algorithm,
        "task_completion": task_completion,
        "communication_ages": _communication_age_report(converted_dir),
        "formal_statistics_gate": formal_gate,
        "communication_age_claim": (
            "Diagnostic observation only; no independent communication "
            "watchdog safety claim is made."),
    }

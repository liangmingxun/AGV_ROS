#!/usr/bin/env python3
"""Analyze independent Exp2c-v3 no-disturbance and paired fake runs."""

import argparse
import csv
import importlib.util
import json
import math
import sys
from pathlib import Path

import yaml


TASK_VELOCITY = 0.100
WINDOW = (2.0, 2.8)
BOUNDARY_TOLERANCE = 1.0e-4
MINIMUM_STAGE1_MEAN_REFERENCE = 0.0995
MAXIMUM_STAGE1_BELOW_FRACTION = 0.05
MAXIMUM_STAGE1_ACTIVE_FRACTION = 0.05

_V2_PATH = Path(__file__).resolve().with_name(
    "analyze_exp2c_v2_yaw_paired.py")
_V2_SPEC = importlib.util.spec_from_file_location("exp2c_v2_paired", str(_V2_PATH))
_V2 = importlib.util.module_from_spec(_V2_SPEC)
_V2_SPEC.loader.exec_module(_V2)


def active_window_rows(rows, disturbance):
    selected = []
    for row in rows:
        progress = _V2.num(row, "yaw_drive_disturbance_path_progress")
        valid = _V2.flag(row, "algorithm_valid")
        if not valid or not math.isfinite(progress):
            continue
        if disturbance:
            if _V2.flag(row, "yaw_drive_disturbance_active"):
                selected.append(row)
        elif WINDOW[0] <= progress <= WINDOW[1]:
            selected.append(row)
    return selected


def boundary_active(row):
    contraction = _V2.num(row, "risk_contraction")
    reference = _V2.num(row, "upper_effective_common_velocity")
    published = _V2.num(row, "common_velocity_reference")
    candidate = _V2.num(row, "candidate_common_velocity")
    upper = _V2.num(row, "common_boundary_upper")
    return (math.isfinite(contraction + reference + upper + candidate + published) and
            contraction > 1.0e-9 and
            candidate > upper + 1.0e-9 and
            abs(reference - upper) <= BOUNDARY_TOLERANCE and
            abs(published - upper) <= BOUNDARY_TOLERANCE)


def reference_reduction_metrics(rows):
    """Same-run upper candidate minus published effective reference.

    Keep the signed difference: negative differences are not reductions.
    Also expose the direct upper clipping separately from downstream dynamics.
    """
    differences = []
    clipping = []
    for row in rows:
        candidate = _V2.num(row, "candidate_common_velocity")
        effective = _V2.num(row, "common_velocity_reference")
        upper_effective = _V2.num(row, "upper_effective_common_velocity")
        if not math.isfinite(candidate + effective + upper_effective):
            raise ValueError("missing/nonfinite same-run v3 reference diagnostics")
        differences.append(candidate - effective)
        clipping.append(max(0.0, candidate - upper_effective))
    if not differences:
        raise ValueError("empty reference analysis window")
    return {
        "actual_reference_reduction_peak_mps": max(differences),
        "actual_reference_reduction_mean_mps": _V2.mean(differences),
        "actual_reference_reduction_positive_fraction":
            sum(value > 1e-6 for value in differences) / len(differences),
        "upper_clipping_reduction_peak_mps": max(clipping),
        "upper_clipping_reduction_mean_mps": _V2.mean(clipping),
        "actual_reference_reduction_definition":
            "same_run_candidate_common_velocity_minus_published_common_velocity",
    }


def manifest_nominal_velocity(run_dir):
    manifest = yaml.safe_load((run_dir / "manifest.yaml").read_text(
        encoding="utf-8"))
    params = yaml.safe_load((run_dir / "rosparams.yaml").read_text(encoding="utf-8"))
    runtime = params.get("formal_fake_algorithm", {}).get("formal_fake_runtime", {})
    leader_velocity = float(runtime.get("leader", {}).get("velocity", math.nan))
    if (runtime.get("experiment_id") != "exp2c_v3_nominal_headroom_fake" or
            not math.isfinite(leader_velocity) or
            abs(leader_velocity - TASK_VELOCITY) > 1e-12):
        raise ValueError("incorrect v3 runtime experiment identity/task velocity")
    for robot in ("agv1", "agv2", "agv3"):
        if params.get(robot, {}).get("chassis_controller", {}).get("transport_type") != "fake":
            raise ValueError("non-fake chassis snapshot")
    def inspect_authorizations(value):
        if isinstance(value, dict):
            for key, child in value.items():
                if key.endswith("hardware_execution_authorized") and child is not False:
                    raise ValueError("hardware execution authorization is not false")
                inspect_authorizations(child)
        elif isinstance(value, list):
            for child in value:
                inspect_authorizations(child)
    inspect_authorizations(params)
    return float(manifest.get("metrics", {}).get("nominal_common_velocity", math.nan))


def stage1_metrics(run_dir):
    validation, summary, rows = _V2.load_run(run_dir)
    # Test all post-startup moving samples, not just the future disturbance
    # window. Terminal invalid/zero samples cannot dilute the active fraction.
    selected = [row for row in rows if _V2.flag(row, "algorithm_valid") and
                0.4 <= _V2.num(row, "load_s_reference") < 5.17]
    if not selected:
        raise ValueError("empty stage1 steady-motion window")
    reductions = reference_reduction_metrics(selected)
    references = [_V2.num(row, "common_velocity_reference") for row in selected]
    references = _V2.finite(references)
    active = sum(boundary_active(row) for row in selected)
    sample_period = float(summary.get("sample_period", 0.01))
    return dict(reductions, **{
        "validation_valid": bool(validation.get("valid")),
        "validation_issue_codes": [issue.get("code") for issue in
                                   validation.get("issues", [])],
        "analysis_samples": len(selected),
        "common_reference_velocity_minimum_mps": min(references, default=None),
        "common_reference_velocity_mean_mps": _V2.mean(references),
        "reference_below_0p099_fraction": (
            sum(value < 0.099 for value in references) / len(references)
            if references else None),
        "risk_boundary_active_fraction": (
            float(active) / len(selected) if selected else None),
        "risk_boundary_active_duration_seconds": active * sample_period,
        "task_completion_time_seconds": summary.get("task", {}).get(
            "completion_time"),
        "manifest_nominal_common_velocity_mps": manifest_nominal_velocity(
            run_dir),
    })


def stage1_passes(metrics):
    m1 = metrics["M1_R1"]
    return all(value is not None for value in (
        m1["common_reference_velocity_mean_mps"],
        m1["reference_below_0p099_fraction"],
        m1["risk_boundary_active_fraction"])) and all(
        method["validation_valid"] and
        abs(method["manifest_nominal_common_velocity_mps"] - TASK_VELOCITY) <= 1e-12
        for method in metrics.values()) and (
        m1["common_reference_velocity_mean_mps"] >=
        MINIMUM_STAGE1_MEAN_REFERENCE) and (
        m1["reference_below_0p099_fraction"] <=
        MAXIMUM_STAGE1_BELOW_FRACTION) and (
        m1["risk_boundary_active_fraction"] <=
        MAXIMUM_STAGE1_ACTIVE_FRACTION)


def write_stage1(root, methods):
    report = {
        "schema_version": 1,
        "experiment": "Exp2c-v3 nominal headroom no-disturbance precheck",
        "task_velocity_mps": TASK_VELOCITY,
        "nominal_inner_upper_mps": 0.107,
        "analysis_window_m": [0.4, 5.17],
        "methods": methods,
        "acceptance": {
            "minimum_m1_mean_reference_mps": MINIMUM_STAGE1_MEAN_REFERENCE,
            "maximum_m1_reference_below_0p099_fraction":
                MAXIMUM_STAGE1_BELOW_FRACTION,
            "maximum_m1_risk_boundary_active_fraction":
                MAXIMUM_STAGE1_ACTIVE_FRACTION,
        },
    }
    report["stage1_passed"] = stage1_passes(methods)
    path = root / "stage1_precheck.json"
    path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")
    lines = [
        "# Exp2c-v3 stage-1 no-disturbance precheck", "",
        "| metric | M1+R1 | M1b+R1 |", "|---|---:|---:|",
    ]
    for key in methods["M1_R1"]:
        if isinstance(methods["M1_R1"][key], (int, float)) and not isinstance(
                methods["M1_R1"][key], bool):
            lines.append("| {} | {} | {} |".format(
                key, methods["M1_R1"][key], methods["M1b_R1"].get(key)))
    lines.extend(["", "stage1_passed: **{}**".format(
        report["stage1_passed"])])
    (root / "stage1_precheck.md").write_text("\n".join(lines) + "\n",
                                               encoding="utf-8")
    return report


def paired_run_metrics(run_dir):
    metrics, rows = _V2.run_metrics(run_dir)
    # Exp2c-v2's baseline-upper gap is deliberately not carried into v3.
    metrics.pop("reference_drop_peak_mps", None)
    metrics.pop("reference_drop_mean_mps", None)
    metrics.pop("reference_drop_definition", None)
    selected = active_window_rows(rows, disturbance=True)
    active = sum(boundary_active(row) for row in selected)
    sample_period = 0.01
    try:
        summary = json.loads((run_dir / "summary_metrics.json").read_text())
        sample_period = float(summary.get("sample_period", sample_period))
    except (OSError, ValueError, TypeError):
        pass
    metrics.update({
        "risk_boundary_active_fraction": (
            float(active) / len(selected) if selected else None),
        "risk_boundary_active_duration_seconds": active * sample_period,
        "manifest_nominal_common_velocity_mps": manifest_nominal_velocity(
            run_dir),
    })
    if (not metrics["validation_valid"] or not selected or
            not math.isfinite(metrics["manifest_nominal_common_velocity_mps"]) or
            abs(metrics["manifest_nominal_common_velocity_mps"] - TASK_VELOCITY) > 1e-12):
        raise ValueError("invalid/empty paired run or incorrect nominal velocity metadata")
    metrics.update(reference_reduction_metrics(selected))
    task_gap = [max(0.0, TASK_VELOCITY - _V2.num(row, "common_velocity_reference"))
                for row in selected]
    metrics["task_velocity_shortfall_peak_mps"] = max(task_gap)
    metrics["task_velocity_shortfall_mean_mps"] = _V2.mean(task_gap)
    physical_margins, disturbed_differentials = [], []
    for row in selected:
        left = _V2.num(row, "yaw_drive_disturbance_left_disturbed")
        right = _V2.num(row, "yaw_drive_disturbance_right_disturbed")
        limit = min(_V2.num(row, "agv2_wheel_left_reported_limit"),
                    _V2.num(row, "agv2_wheel_right_reported_limit"))
        if math.isfinite(left + right + limit):
            physical_margins.append(limit - max(abs(left), abs(right)))
            disturbed_differentials.append(right - left)
    metrics["perturbed_wheel_physical_margin_minimum_mps"] = min(physical_margins, default=None)
    metrics["perturbed_wheel_physical_margin_mean_mps"] = _V2.mean(physical_margins)
    metrics["perturbed_differential_wheel_rms_mps"] = _V2.rms(disturbed_differentials)
    metrics["risk_boundary_analysis_samples"] = len(selected)
    metrics["wheel_burden_definition"] = "controller raw before antisymmetric yaw injection"
    metrics["geometry_scope"] = "heading/side errors: yaw window; support/rigid-fit: full valid run"
    return metrics, selected


def interpolate(samples, coordinate, fields, grid):
    points = [(_V2.num(row, coordinate), row) for row in samples
              if math.isfinite(_V2.num(row, coordinate))]
    points.sort(key=lambda item: item[0])
    output, cursor = [], 0
    for value in grid:
        while cursor + 1 < len(points) and points[cursor + 1][0] < value:
            cursor += 1
        if (not points or value < points[0][0] or value > points[-1][0] or
                cursor + 1 >= len(points)):
            continue
        x0, row0 = points[cursor]
        x1, row1 = points[cursor + 1]
        weight = 0.0 if x1 == x0 else (value - x0) / (x1 - x0)
        result = {coordinate: value}
        for field in fields:
            y0, y1 = _V2.num(row0, field), _V2.num(row1, field)
            if not math.isfinite(y0 + y1):
                break
            result[field] = y0 + weight * (y1 - y0)
        else:
            output.append(result)
    return output


def paired_alignment(m1_rows, m1b_rows, domain, output_path):
    if not m1_rows or not m1b_rows:
        raise ValueError("empty paired alignment input")
    fields = (
        "common_velocity_reference", "common_boundary_upper",
        "risk_contraction", "agv2_robust_margin",
        "yaw_drive_disturbance_left_nominal",
        "yaw_drive_disturbance_right_nominal", "agv2_s_dot_actual")
    if domain == "time":
        for rows in (m1_rows, m1b_rows):
            origin = _V2.num(rows[0], "stamp")
            for row in rows:
                row["v3_relative_time"] = _V2.num(row, "stamp") - origin
        coordinate = "v3_relative_time"
    else:
        coordinate = "yaw_drive_disturbance_path_progress"
    low = max(min(_V2.num(row, coordinate) for row in m1_rows),
              min(_V2.num(row, coordinate) for row in m1b_rows))
    high = min(max(_V2.num(row, coordinate) for row in m1_rows),
               max(_V2.num(row, coordinate) for row in m1b_rows))
    if not math.isfinite(low + high) or high <= low:
        raise ValueError("no paired coordinate overlap")
    grid = [low + (high - low) * index / 400.0 for index in range(401)]
    m1 = interpolate(m1_rows, coordinate, fields, grid)
    m1b = interpolate(m1b_rows, coordinate, fields, grid)
    rows, reductions = [], []
    counterfactual = {row[coordinate]: row for row in m1b}
    for effective in m1:
        candidate = counterfactual.get(effective[coordinate])
        if candidate is None:
            continue
        reduction = max(0.0, candidate["common_velocity_reference"] -
                        effective["common_velocity_reference"])
        reductions.append(reduction)
        row = {
            coordinate: effective[coordinate],
            "m1b_counterfactual_common_velocity_mps":
                candidate["common_velocity_reference"],
            "effective_common_velocity_mps":
                effective["common_velocity_reference"],
            "paired_reference_difference_mps": reduction,
        }
        for field in fields[1:]:
            row["m1_" + field] = effective[field]
            row["m1b_" + field] = candidate[field]
        rows.append(row)
    if not rows:
        raise ValueError("no finite aligned paired samples")
    with output_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    return {
        "samples": len(rows),
        "domain_minimum": low,
        "domain_maximum": high,
        "paired_reference_difference_peak_mps": max(reductions, default=None),
        "paired_reference_difference_mean_mps": _V2.mean(reductions),
        "paired_reference_difference_positive_fraction": (
            sum(value > 1.0e-6 for value in reductions) / len(reductions)
            if reductions else None),
        "definition": (
            "fresh_paired_M1b_counterfactual_common_velocity_minus_"
            "M1_effective_common_velocity_clipped_at_zero"),
    }


def write_final(root, stage1, methods, time_alignment, progress_alignment):
    m1, m1b = methods["M1_R1"], methods["M1b_R1"]
    report = {
        "schema_version": 1,
        "experiment": "Exp2c-v3 paired fake nominal risk headroom",
        "task_velocity_mps": TASK_VELOCITY,
        "nominal_inner_upper_mps": 0.107,
        "yaw_disturbance_amplitude_mps": 0.020,
        "yaw_disturbance_window_m": list(WINDOW),
        "stage1": stage1,
        "methods": methods,
        "time_domain_alignment": time_alignment,
        "robot2_actual_progress_domain_alignment": progress_alignment,
        "causal_questions": {
            "A_no_disturbance_m1_maintained_task_velocity":
                stage1["stage1_passed"],
            "B_risk_boundary_became_active":
                (m1["risk_boundary_active_fraction"] or 0.0) > 0.0,
            "C_m1_produced_actual_reference_reduction":
                (m1[
                    "actual_reference_reduction_peak_mps"] or 0.0) > 1.0e-5,
            "D_controller_raw_peak_reduced":
                m1["controller_wheel_raw_peak_mps"] <
                m1b["controller_wheel_raw_peak_mps"],
            "D_controller_differential_rms_reduced":
                m1["controller_differential_wheel_rms_mps"] <
                m1b["controller_differential_wheel_rms_mps"],
            "E_minimum_wheel_margin_improved":
                m1["wheel_margin_minimum"] > m1b["wheel_margin_minimum"],
            "F_heading_rmse_improved":
                m1["robot2_heading_error_rmse_rad"] <
                m1b["robot2_heading_error_rmse_rad"],
            "F_support_rmse_improved":
                m1["robot2_support_position_rmse_m"] <
                m1b["robot2_support_position_rmse_m"],
            "F_rigid_fit_rmse_improved":
                m1["rigid_fit_residual_rmse_m"] <
                m1b["rigid_fit_residual_rmse_m"],
        },
        "task_time_cost_m1_minus_m1b_seconds": (
            m1["task_completion_time_seconds"] -
            m1b["task_completion_time_seconds"]),
        "historical_exp2c_v1_v2_modified": False,
        "hardware_authorization_changed": False,
    }
    report["full_causal_chain_supported_in_this_fake_pair"] = all(
        report["causal_questions"].values())
    report["evidence_scope"] = "one fresh fake run per condition; not hardware authorization or repeatability proof"
    (root / "paired_comparison.json").write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lines = ["# Exp2c-v3 paired fake comparison", "",
             "A = 0.020 m/s; nominal inner upper = 0.107 m/s", "",
             "| metric | M1+R1 | M1b+R1 |", "|---|---:|---:|"]
    for key in m1:
        if isinstance(m1[key], (int, float)) and not isinstance(m1[key], bool):
            lines.append("| {} | {} | {} |".format(
                key, m1[key], m1b.get(key)))
    lines.extend([
        "", "## Actual reference reduction", "",
        "Same-run candidate minus published reference peak/mean: {} / {} m/s".format(
            m1["actual_reference_reduction_peak_mps"],
            m1["actual_reference_reduction_mean_mps"]),
        "", "## Causal questions", "",
    ])
    lines.extend("- {}: {}".format(key, value) for key, value in
                 report["causal_questions"].items())
    (root / "paired_comparison.md").write_text("\n".join(lines) + "\n",
                                                encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("run_root", type=Path)
    parser.add_argument("--stage1-only", action="store_true")
    args = parser.parse_args()
    root = args.run_root.resolve()
    stage1_root = root / "stage1_no_disturbance"
    try:
        stage1_methods = {
            method: stage1_metrics(stage1_root / method)
            for method in ("M1_R1", "M1b_R1")}
        stage1 = write_stage1(root, stage1_methods)
        if not stage1["stage1_passed"]:
            print(json.dumps(stage1, indent=2, sort_keys=True))
            print("EXP2C_V3_STAGE1_STATUS=FAILED_STOP_BEFORE_DISTURBANCE",
                  file=sys.stderr)
            return 10
        if args.stage1_only:
            print(json.dumps(stage1, indent=2, sort_keys=True))
            print("EXP2C_V3_STAGE1_STATUS=PASSED")
            return 0

        paired_root = root / "stage2_yaw_0p020"
        m1b, m1b_rows = paired_run_metrics(paired_root / "M1b_R1")
        m1, m1_rows = paired_run_metrics(paired_root / "M1_R1")
        time_alignment = paired_alignment(
            m1_rows, m1b_rows, "time", root / "paired_time_domain.csv")
        progress_alignment = paired_alignment(
            m1_rows, m1b_rows, "progress",
            root / "paired_robot2_progress_domain.csv")
        report = write_final(
            root, stage1, {"M1_R1": m1, "M1b_R1": m1b},
            time_alignment, progress_alignment)
    except Exception as error:
        print("ERROR: {}".format(error), file=sys.stderr)
        return 2
    print(json.dumps(report, indent=2, sort_keys=True))
    print("EXP2C_V3_PAIRED_FAKE_STATUS=COMPLETE")
    return 0


if __name__ == "__main__":
    sys.exit(main())

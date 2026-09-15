#!/usr/bin/env python3
"""Analyze the frozen Candidate-A classic additive disturbance fake validation."""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import statistics

import yaml

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    "repeat", HERE / "analyze_exp2c_v5b_three_method_repeat.py")
R = importlib.util.module_from_spec(spec)
spec.loader.exec_module(R)
H, V4, V2 = R.H, R.V4, R.V2

IDENTITY = "classic_additive_disturbance_candidate_A_validation"
MARKERS = ["CLASSIC_ADDITIVE_DISTURBANCE_VALIDATION",
           "NOT_FORMAL_PAPER_EVIDENCE"]
METHODS = {"M1": "M1_R1", "M1b": "M1b_R1", "M2b": "M2b_COMPLETE"}
ORDERS = [["M1b", "M1", "M2b"], ["M2b", "M1b", "M1"],
          ["M1", "M2b", "M1b"]]
GEOMETRY = ["lateral_error_IAE", "heading_error_IAE", "support_error_IAE",
            "rigid_fit_error_IAE"]
CORE = GEOMETRY + ["longitudinal_error_IAE", "pairwise_side_error_IAE",
                   "progress_error_IAE", "J_risk_0p5", "J_risk_0p7",
                   "wheel_margin_below_0p5_duration_seconds",
                   "wheel_margin_below_0p7_duration_seconds",
                   "controller_raw_wheel_peak_mps",
                   "controller_raw_above_0p160_duration_seconds",
                   "physical_speed_limiter_duration_seconds",
                   "task_time_seconds"]
CONFIG = HERE.parents[1] / "multi_agv_bringup" / "config"
PROFILE = {"enabled": True, "trigger_progress": 2.0,
           "linear_amplitude": 0.030, "angular_amplitude": 0.350,
           "linear_frequency": 1.0, "angular_frequency": 1.0,
           "angular_phase": math.pi / 2.0, "duration": 8.0,
           "ramp_in": 0.5, "ramp_out": 0.5}


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True,
                               allow_nan=False) + "\n")


def prepare(root, method):
    """Create isolated configs without changing archived experiment configs."""
    root = root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    old = H.CANDIDATES
    candidate = next(iter(old))
    try:
        H.CANDIDATES = {candidate: old[candidate]}
        H.prepare(root)
    finally:
        H.CANDIDATES = old
    metadata = {"triad": root.parent.name, "method": METHODS[method]}
    for name in ("M1", "M1b"):
        path = root / "configs" / f"{candidate}_{name}.yaml"
        data = yaml.safe_load(path.read_text())
        runtime = data["formal_fake_runtime"]
        runtime.update({"experiment_id": IDENTITY, "evidence_scope": MARKERS,
                        "classic_additive_candidate_A": dict(PROFILE),
                        "classic_validation_metadata": metadata})
        runtime["yaw_effectiveness_hold_v5b"] = {"enabled": False}
        data["formal_upper"]["configuration_status"] = "_".join(MARKERS)
        path.write_text(yaml.safe_dump(data, sort_keys=False))
        auth_path = root / "configs" / f"{candidate}_{name}_authorization.yaml"
        auth = yaml.safe_load(auth_path.read_text())
        auth["evidence_scope"] = MARKERS
        auth["classic_additive_candidate_A"] = {
            "hardware_execution_authorized": False}
        auth_path.write_text(yaml.safe_dump(auth, sort_keys=False))

    m2_path = root / "configs" / f"{candidate}_M2b.yaml"
    m2_path.write_text(yaml.safe_dump(
        yaml.safe_load((CONFIG / "exp2b_M2b.yaml").read_text()), sort_keys=False))
    runtime = yaml.safe_load((CONFIG /
        "formal_serial_m2b_circle_0p10_observation_runtime.yaml").read_text())
    fr = runtime["formal_fake_runtime"]
    fr.update({"experiment_id": IDENTITY,
               "command_publication_authorized": True,
               "serial_execution_authorized": False,
               H.QUALITY_POLICY: True, "evidence_scope": MARKERS,
               "classic_additive_candidate_A": dict(PROFILE),
               "classic_validation_metadata": metadata,
               "yaw_effectiveness_hold_v5b": {"enabled": False}})
    (root / "configs" / "M2b_runtime.yaml").write_text(
        yaml.safe_dump(runtime, sort_keys=False))
    source_auth = root / "configs" / f"{candidate}_M1_authorization.yaml"
    auth = yaml.safe_load(source_auth.read_text())
    auth["authorization_scope"]["upper_config"] = str(m2_path)
    auth["authorization_scope"]["runtime_config"] = str(
        root / "configs" / "M2b_runtime.yaml")
    auth["evidence_scope"] = MARKERS
    (root / "configs" / f"{candidate}_M2b_authorization.yaml").write_text(
        yaml.safe_dump(auth, sort_keys=False))

    loc_path = root / "configs" / "localization_v5_quality_observation.yaml"
    loc = yaml.safe_load(loc_path.read_text())
    loc.update({"experiment_id": IDENTITY, "platform_transport_type": "fake",
                "hardware_execution_authorized": False})
    loc_path.write_text(yaml.safe_dump(loc, sort_keys=False))
    topics_path = root / "configs" / "record_topics_v5.yaml"
    topics = yaml.safe_load(topics_path.read_text())
    for key in ("topics", "required_topics"):
        values = topics["experiment_recording"][key]
        for legacy in ("/multi_agv/yaw_effectiveness_hold_state",):
            while legacy in values:
                values.remove(legacy)
        for topic in ("/multi_agv/classic_additive_disturbance_state",
                      "/multi_agv/v5_exploration_quality_state"):
            if topic not in values:
                values.append(topic)
        if method == "M2b" and "/multi_agv/m2b_algorithm_state" not in values:
            values.append("/multi_agv/m2b_algorithm_state")
    topics_path.write_text(yaml.safe_dump(topics, sort_keys=False))
    validation = yaml.safe_load((HERE.parent / "config" /
                                 "validation_defaults.yaml").read_text())
    registry = validation["experiment_recording"]
    for registered in ("M1_R1", "M1b_R1"):
        required = registry["method_required_topics"][registered]
        if "/multi_agv/classic_additive_disturbance_state" not in required:
            required.append("/multi_agv/classic_additive_disturbance_state")
    registry["minimum_topic_rates"]["/multi_agv/classic_additive_disturbance_state"] = 90.0
    (root / "configs" / "validation_classic.yaml").write_text(
        yaml.safe_dump(validation, sort_keys=False))
    save(root / "VALIDATION_SCOPE.json", {
        "experiment_id": IDENTITY, "markers": MARKERS,
        "method": METHODS[method], "hardware_authorization": False,
        "serial_execution": False, "candidate": "A", "profile": PROFILE,
        "windows_seconds": {"baseline": [-3, 0], "disturbance": [0, 8],
                            "recovery": [8, 23], "primary": [0, 23]},
        "parameter_search": False, "task_timeout_seconds": 100,
        "m2b_config_sha256": hashlib.sha256(
            (CONFIG / "exp2b_M2b.yaml").read_bytes()).hexdigest()})


def durations(rows):
    return V4.durations(rows)


def exposure(rows, values):
    dt = durations(rows)
    av = [abs(v) for v in values]
    return {"peak": max(av), "RMS": math.sqrt(V4.weighted_mean(
                [v*v for v in av], dt)), "IAE": sum(v*t for v, t in zip(av, dt))}


def phase_metrics(rows):
    dt = durations(rows)
    out = {"samples": len(rows), "duration_seconds": sum(dt),
           "maximum_spacing_seconds": max(dt)}
    for key in ("lateral", "heading", "longitudinal", "support",
                "rigid_fit", "pairwise_side", "progress"):
        for tag, value in exposure(rows, [r["errors"][key] for r in rows]).items():
            out[f"{key}_error_{tag}"] = value
    raw = [(V2.num(r, "classic_additive_left_raw"),
            V2.num(r, "classic_additive_right_raw")) for r in rows]
    disturbed = [(V2.num(r, "classic_additive_left_disturbed"),
                  V2.num(r, "classic_additive_right_disturbed")) for r in rows]
    raw_peak = [max(abs(a), abs(b)) for a, b in raw]
    limits = [(V2.num(r, "agv2_wheel_left_reported_limit"),
               V2.num(r, "agv2_wheel_right_reported_limit")) for r in rows]
    margin = [min(1.0, max(0.0, min(l-abs(a), rr-abs(b)) / .010))
              for (a, b), (l, rr) in zip(raw, limits)]
    out.update({
        "controller_raw_wheel_peak_mps": max(raw_peak),
        "controller_raw_wheel_RMS_mps": math.sqrt(V4.weighted_mean(
            [(a*a+b*b)/2 for a, b in raw], dt)),
        "disturbed_wheel_peak_mps": max(abs(v) for pair in disturbed for v in pair),
        "applied_wheel_peak_mps": max(abs(V2.num(r, f"agv2_wheel_{side}_applied"))
                                      for r in rows for side in ("left", "right")),
        "wheel_margin_minimum": min(margin), "wheel_margin_p05": V2.percentile(margin, .05),
        "wheel_margin_mean": V4.weighted_mean(margin, dt),
        "J_risk_0p5": sum(max(0., .5-v)*t for v, t in zip(margin, dt)),
        "J_risk_0p7": sum(max(0., .7-v)*t for v, t in zip(margin, dt))})
    for tag, threshold in (("150", .150), ("155", .155), ("160", .160)):
        out[f"controller_raw_above_0p{tag}_duration_seconds"] = sum(
            t for v, t in zip(raw_peak, dt) if v > threshold)
    for tag, threshold in (("0p5", .5), ("0p7", .7)):
        out[f"wheel_margin_below_{tag}_duration_seconds"] = sum(
            t for v, t in zip(margin, dt) if v < threshold)
    active = [any(V2.flag(r, f"agv2_wheel_{side}_speed_limit_active")
                  for side in ("left", "right")) for r in rows]
    run = maximum = 0.
    for flag, step in zip(active, dt):
        run = run + step if flag else 0.
        maximum = max(maximum, run)
    out["physical_speed_limiter_duration_seconds"] = sum(
        t for flag, t in zip(active, dt) if flag)
    out["physical_speed_limiter_max_continuous_seconds"] = maximum
    return out


def classic_expected(t):
    if t < 0 or t >= 8:
        return 0., 0., 0.
    if t < .5:
        u = t/.5
        q = u**3*(10-15*u+6*u*u)
    elif t >= 7.5:
        u = (8-t)/.5
        q = u**3*(10-15*u+6*u*u)
    else:
        q = 1.
    return q, q*.030*math.sin(t), q*.350*math.sin(t+math.pi/2)


def analyze(run, method, log):
    params = yaml.safe_load((run / "rosparams.yaml").read_text())
    manifest = yaml.safe_load((run / "manifest.yaml").read_text())
    p = params["formal_fake_algorithm"]
    runtime = p["formal_fake_runtime"]
    assert manifest["experiment_id"] == runtime["experiment_id"] == IDENTITY
    assert runtime["leader"]["velocity"] == manifest["metrics"]["nominal_common_velocity"] == .10
    for key, expected in PROFILE.items():
        assert abs(float(runtime["classic_additive_candidate_A"][key])-float(expected)) < 1e-12
    for i in (1, 2, 3):
        assert params[f"agv{i}"]["chassis_controller"]["transport_type"] == "fake"
    if method == "M2b":
        frozen = yaml.safe_load((CONFIG / "exp2b_M2b.yaml").read_text())
        for key in ("formal_m2b", "formal_lower", "formal_upper"):
            assert p[key] == frozen[key]
    else:
        assert p["formal_upper"]["agents"]["risk_gain_upper"] == [.1]*3
    rows_all = list(csv.DictReader((run / "converted/aligned_samples.csv").open()))
    rows = []
    seen = set()
    for source in rows_all:
        if not V2.flag(source, "algorithm_valid") or not V2.flag(
                source, "classic_additive_state_available"):
            continue
        wall = V2.num(source, "classic_additive_wall_time")
        if wall in seen:
            continue
        seen.add(wall)
        rows.append(dict(source))
    triggered = next(r for r in rows if V2.flag(r, "classic_additive_triggered"))
    trigger = V2.num(triggered, "classic_additive_trigger_wall_time")
    profile_error = 0.
    mapping_error = 0.
    capability_values = []
    for row in rows:
        t = V2.num(row, "classic_additive_wall_time") - trigger
        row["relative_wall_time"] = t
        row["errors"] = V4.error_signals(row)
        for robot in (1, 2, 3):
            for key in ("longitudinal", "lateral", "heading"):
                row[f"agv{robot}_hold_tracker_{key}_error"] = row[
                    f"agv{robot}_classic_tracker_{key}_error"]
        for key in ("lateral", "heading", "longitudinal"):
            row["errors"][key] = V2.num(row, f"agv2_classic_tracker_{key}_error")
        q, dv, dw = classic_expected(t)
        profile_error = max(profile_error,
            abs(V2.num(row, "classic_additive_envelope")-q),
            abs(V2.num(row, "classic_additive_d_v")-dv),
            abs(V2.num(row, "classic_additive_d_omega")-dw))
        left = V2.num(row, "classic_additive_left_raw") + dv - .139284482/2*dw
        right = V2.num(row, "classic_additive_right_raw") + dv + .139284482/2*dw
        mapping_error = max(mapping_error,
            abs(left-V2.num(row, "classic_additive_left_disturbed")),
            abs(right-V2.num(row, "classic_additive_right_disturbed")))
        capability_values += [V2.num(row, "agv2_wheel_left_reported_limit"),
                              V2.num(row, "agv2_wheel_right_reported_limit")]
    rows.sort(key=lambda r: r["relative_wall_time"])
    assert profile_error < 1e-9 and mapping_error < 1e-9
    assert all(abs(v-.16) < 1e-12 for v in capability_values)
    selected = {"baseline": [r for r in rows if -3 <= r["relative_wall_time"] < 0],
                "disturbance": [r for r in rows if 0 <= r["relative_wall_time"] < 8],
                "recovery": [r for r in rows if 8 <= r["relative_wall_time"] < 23],
                "primary": [r for r in rows if 0 <= r["relative_wall_time"] < 23]}
    phases = {name: phase_metrics(value) for name, value in selected.items() if value}
    validation = json.loads((run / "validation.json").read_text())
    summary = json.loads((run / "summary_metrics.json").read_text())
    complete = bool(validation.get("task_completion", {}).get("complete"))
    full_recovery = bool(selected["recovery"] and
        selected["recovery"][-1]["relative_wall_time"] >= 22.97)
    motion_log = log.read_text(errors="replace").lower()
    invalid = any(word in motion_log for word in (
        "numerical_invalid", "state_chain_invalid", "held fail-zero:"))
    category = "INVALID_RUN" if invalid else (
        "VALID_COMPLETED" if complete and full_recovery else
        "METHOD_PERFORMANCE_RESULT_RECOVERY_CENSORED")
    geometry_detail = H.quality_metrics(selected["primary"])
    mechanism = None
    if method == "M1":
        rs = selected["primary"]
        dt = durations(rs)
        contraction = [V2.num(r, "risk_contraction") for r in rs]
        reduction = [V2.num(r, "candidate_common_velocity")-
                     V2.num(r, "common_velocity_reference") for r in rs]
        active = [V4.V3.boundary_active(r) for r in rs]
        margin = [V2.num(r, "agv2_robust_margin") for r in rs]
        risk_signal = [V2.num(r, "agv2_risk_signal") for r in rs]
        dynamic_upper = [V2.num(r, "agv2_boundary_upper") for r in rs]
        mechanism = {"robust_margin_minimum": min(margin),
            "robust_margin_p05": V2.percentile(margin, .05),
            "robust_margin_mean": V4.weighted_mean(margin, dt),
            "risk_contraction_peak": max(contraction),
            "risk_contraction_integral": sum(v*t for v, t in zip(contraction, dt)),
            "risk_signal_peak": max(risk_signal),
            "risk_signal_mean": V4.weighted_mean(risk_signal, dt),
            "dynamic_upper_boundary_minimum": min(dynamic_upper),
            "dynamic_upper_boundary_mean": V4.weighted_mean(dynamic_upper, dt),
            "risk_boundary_active_duration": sum(t for a, t in zip(active, dt) if a),
            "risk_boundary_active_fraction": sum(t for a, t in zip(active, dt) if a)/sum(dt),
            "actual_reference_reduction_peak": max(reduction),
            "actual_reference_reduction_mean": V4.weighted_mean(reduction, dt),
            "common_reference_minimum": min(V2.num(r, "common_velocity_reference") for r in rs),
            "common_reference_mean": V4.weighted_mean(
                [V2.num(r, "common_velocity_reference") for r in rs], dt)}
    m2b = None
    if method == "M2b":
        fields = [field for field in rows[0] if field.startswith("m2b_") and
                  all(math.isfinite(V2.num(row, field)) for row in rows)]
        m2b = {field: {"minimum": min(V2.num(row, field) for row in rows),
                       "maximum": max(V2.num(row, field) for row in rows)}
               for field in fields}
        assert m2b, "missing complete M2b diagnostics"
    result = {"experiment_id": IDENTITY, "markers": MARKERS,
        "formal_evidence": False, "method": METHODS[method], "category": category,
        "complete": complete, "recovery_censored": not full_recovery,
        "task_time_seconds": summary.get("task", {}).get("completion_time"),
        "phases": phases, "M1_method_specific_mechanism": mechanism,
        "M2b_method_specific_diagnostics": m2b,
        "geometry_all_robots_and_pairwise_sides": geometry_detail,
        "capability_report_unchanged": True,
        "capability_limit_minimum": min(capability_values),
        "capability_limit_maximum": max(capability_values),
        "profile_maximum_absolute_error": profile_error,
        "mapping_maximum_absolute_error": mapping_error,
        "run_path": str(run)}
    plot_run(run, rows, result)
    return result


def plot_run(run, rows, result):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    out = run / "classic_plots_raw"
    out.mkdir(exist_ok=True)
    t = [r["relative_wall_time"] for r in rows]
    specifications = [
        ("01_disturbance", [("d_v", "classic_additive_d_v"),
                             ("d_omega", "classic_additive_d_omega")]),
        ("02_robot2_lateral_error", [("lateral", "agv2_classic_tracker_lateral_error")]),
        ("03_robot2_heading_error", [("heading", "agv2_classic_tracker_heading_error")]),
        ("04_support_error", [("support", "errors:support")]),
        ("05_rigid_fit_error", [("rigid_fit", "errors:rigid_fit")]),
        ("06_controller_raw", [("left", "classic_additive_left_raw"),
                                ("right", "classic_additive_right_raw")]),
        ("07_disturbed_applied", [("disturbed_left", "classic_additive_left_disturbed"),
                                   ("disturbed_right", "classic_additive_right_disturbed"),
                                   ("applied_left", "agv2_wheel_left_applied"),
                                   ("applied_right", "agv2_wheel_right_applied")]),
        ("08_wheel_margin", []),
        ("09_robust_margin", [("robust_margin", "agv2_robust_margin")]),
        ("10_dynamic_upper", [("dynamic_upper", "agv2_boundary_upper")]),
        ("11_common_reference", [("common_reference", "common_velocity_reference")]),
        ("12_actual_reference_reduction", [])]
    for name, series in specifications:
        if name.startswith(("09", "10", "12")) and result["method"] != "M1_R1":
            continue
        fig, ax = plt.subplots(figsize=(8, 4.5))
        if name.startswith("08"):
            values = [min(1., max(0., min(
                V2.num(r, "agv2_wheel_left_reported_limit")-abs(V2.num(r, "classic_additive_left_raw")),
                V2.num(r, "agv2_wheel_right_reported_limit")-abs(V2.num(r, "classic_additive_right_raw")))/.010)) for r in rows]
            ax.plot(t, values, label="wheel margin")
        elif name.startswith("12"):
            ax.plot(t, [V2.num(r, "candidate_common_velocity")-
                        V2.num(r, "common_velocity_reference") for r in rows],
                    label="actual reference reduction")
        else:
            for label, field in series:
                ax.plot(t, [r["errors"][field.split(":", 1)[1]] if field.startswith("errors:") else V2.num(r, field)
                            for r in rows], label=label)
        ax.axvspan(0, 8, color="gold", alpha=.12)
        ax.set(xlabel="Time from trigger (s)", title=f"{result['method']} / raw unsmoothed")
        ax.legend(); fig.tight_layout(); fig.savefig(out / f"{name}.png"); plt.close(fig)


def metric(data, key):
    return data["task_time_seconds"] if key == "task_time_seconds" else data["phases"]["primary"].get(key)


def describe(values):
    values = [v for v in values if v is not None and math.isfinite(v)]
    return {"n": len(values), "mean": statistics.mean(values) if values else None,
            "std": statistics.stdev(values) if len(values) > 1 else None,
            "min": min(values) if values else None, "max": max(values) if values else None}


def write_csv(path, rows):
    if not rows:
        return
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)


def aggregate(root):
    triads = []
    comparisons = {"M1b": [], "M2b": []}
    absolute = []
    for triad_path in sorted(root.glob("Triad[0-9][0-9]")):
        data = {m: json.loads((triad_path / folder / "run" /
                "classic_metrics.json").read_text()) for m, folder in METHODS.items()}
        triads.append({"triad": triad_path.name,
                       "order": json.loads((triad_path / "order.json").read_text()),
                       "methods": data})
        for key in CORE:
            for method, result in data.items():
                absolute.append({"triad": triad_path.name, "method": METHODS[method],
                                 "metric": key, "value": metric(result, key)})
            for comparator in comparisons:
                a, b = metric(data["M1"], key), metric(data[comparator], key)
                comparisons[comparator].append({"triad": triad_path.name,
                    "metric": key, "baseline_value": b, "M1_value": a,
                    "baseline_minus_M1": b-a,
                    "relative_improvement_percent": 100*(b-a)/b if abs(b) > 1e-12 else None})
    write_csv(root / "three_method_comparison.csv", absolute)
    for comparator, rows in comparisons.items():
        write_csv(root / f"M1_vs_{comparator}.csv", rows)
    mechanism = [t["methods"]["M1"]["M1_method_specific_mechanism"] for t in triads]
    active = len(mechanism) == 3 and all(m and m["risk_contraction_peak"] > 0 and
        m["risk_boundary_active_duration"] > 0 and
        m["actual_reference_reduction_peak"] > 0 for m in mechanism)
    improvements = {name: {key: describe([r["relative_improvement_percent"]
        for r in rows if r["metric"] == key]) for key in GEOMETRY}
        for name, rows in comparisons.items()}
    same_direction = sum(1 for c in improvements.values() for s in c.values()
                         if s["mean"] is not None and s["mean"] > 0)
    resource = all(describe([r["baseline_minus_M1"] for r in comparisons[c]
        if r["metric"] in ("J_risk_0p5", "controller_raw_wheel_peak_mps")])["mean"] >= 0
        for c in comparisons)
    all_valid = len(triads) == 3 and all(d["category"] == "VALID_COMPLETED"
        for t in triads for d in t["methods"].values())
    if active and all_valid and same_direction >= 6 and resource:
        conclusion = "CLASSIC_ADDITIVE_DISTURBANCE_EFFECT_CONFIRMED"
    elif triads:
        conclusion = "CLASSIC_ADDITIVE_DISTURBANCE_MIXED_OR_WEAK_EFFECT"
    else:
        conclusion = "CLASSIC_ADDITIVE_DISTURBANCE_NOT_USEFUL"
    result = {"experiment_id": IDENTITY, "markers": MARKERS,
        "triads_completed": len(triads), "valid_runs": sum(d["category"] == "VALID_COMPLETED"
            for t in triads for d in t["methods"].values()),
        "invalid_runs": sum(d["category"] == "INVALID_RUN"
            for t in triads for d in t["methods"].values()),
        "all_runs_valid_completed": all_valid, "mechanism_active_3_of_3": active,
        "geometry_improvements_percent": improvements, "conclusion": conclusion,
        "candidate_B_tested": False, "triads": triads}
    save(root / "summary.json", result)
    lines = ["# Classic additive disturbance Candidate A results", "",
        "CLASSIC_ADDITIVE_DISTURBANCE_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE", "",
        conclusion, "", f"Fresh triads: {len(triads)}; valid runs: {result['valid_runs']}; invalid runs: {result['invalid_runs']}.", "",
        "CapabilityReport and physical limits remained unchanged; the disturbance is injected after native raw demand and before the common limiter.", "",
        "| Comparison | Metric | Mean improvement (%) |", "| --- | --- | ---: |"]
    for comparator, values in improvements.items():
        for key, stat in values.items():
            lines.append(f"| M1 vs {comparator} | {key} | {stat['mean']} |")
    lines += ["", "M1-only mechanism values are N/A for M1b and complete M2b, never zero-filled.",
              "Candidate B was not run and no parameter search was performed."]
    (root / "summary.md").write_text("\n".join(lines) + "\n")
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--prepare", type=Path)
    parser.add_argument("--method", choices=METHODS)
    parser.add_argument("--check-run", type=Path)
    parser.add_argument("--check-log", type=Path)
    parser.add_argument("--aggregate", type=Path)
    args = parser.parse_args()
    if args.prepare:
        prepare(args.prepare, args.method); return 0
    if args.aggregate:
        aggregate(args.aggregate.resolve()); return 0
    try:
        result = analyze(args.check_run.resolve(), args.method,
                         args.check_log.resolve())
    except Exception as exc:
        result = {"experiment_id": IDENTITY, "markers": MARKERS,
                  "category": "INVALID_RUN", "reason": str(exc)}
    save(args.check_run / "classic_metrics.json", result)
    print(result["category"])
    return 20 if result["category"] == "INVALID_RUN" else 0


if __name__ == "__main__":
    raise SystemExit(main())

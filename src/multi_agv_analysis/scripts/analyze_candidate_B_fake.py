#!/usr/bin/env python3
"""Analyze the fixed rho=0.85 Candidate B exploratory fake triad."""
import argparse
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    "candidate_b_scaffold", HERE / "analyze_exp2c_v5b_yaw_hold.py")
H = importlib.util.module_from_spec(spec)
spec.loader.exec_module(H)
V4, V2 = H.V4, H.V2

IDENTITY = "candidate_B_effectiveness_fake_validation"
MARKERS = ["EXPLORATORY_FAKE_VALIDATION", "NOT_FORMAL_PAPER_EVIDENCE"]
METHODS = {"M1": "M1_R1", "M1b": "M1b_R1", "M2b": "M2b_M2b"}
CONFIG = HERE.parents[1] / "multi_agv_bringup/config"
PROFILE = yaml.safe_load(
    (CONFIG / "formal_fake_candidate_B_effectiveness.yaml").read_text()
)["formal_fake_runtime"]["candidate_b"]


def save(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, sort_keys=True,
                               allow_nan=False) + "\n")


def prepare(root, method):
    """Create isolated configs without importing the legacy SciPy analysis."""
    root = root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    old = H.CANDIDATES
    candidate = next(iter(H.CANDIDATES))
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
        runtime.update({"experiment_id": IDENTITY,
                        "evidence_scope": MARKERS,
                        "candidate_b": dict(PROFILE),
                        "candidate_b_metadata": metadata,
                        "classic_additive_candidate_A": {"enabled": False},
                        "classic_additive_physical": {
                            "enabled": False,
                            "hardware_execution_authorized": False},
                        "yaw_effectiveness_hold_v5b": {"enabled": False}})
        data["formal_upper"]["configuration_status"] = "_".join(MARKERS)
        path.write_text(yaml.safe_dump(data, sort_keys=False))
        auth_path = root / "configs" / f"{candidate}_{name}_authorization.yaml"
        auth = yaml.safe_load(auth_path.read_text())
        auth["evidence_scope"] = MARKERS
        auth["candidate_b"] = {"hardware_execution_authorized": False}
        auth_path.write_text(yaml.safe_dump(auth, sort_keys=False))

    m2_path = root / "configs" / f"{candidate}_M2b.yaml"
    m2_path.write_text(yaml.safe_dump(
        yaml.safe_load((CONFIG / "exp2b_M2b.yaml").read_text()),
        sort_keys=False))
    runtime_data = yaml.safe_load((
        CONFIG / "formal_serial_m2b_circle_0p10_observation_runtime.yaml"
    ).read_text())
    runtime = runtime_data["formal_fake_runtime"]
    runtime.update({"experiment_id": IDENTITY,
                    "command_publication_authorized": True,
                    "serial_execution_authorized": False,
                    H.QUALITY_POLICY: True,
                    "evidence_scope": MARKERS,
                    "candidate_b": dict(PROFILE),
                    "candidate_b_metadata": metadata,
                    "classic_additive_candidate_A": {"enabled": False},
                    "classic_additive_physical": {
                        "enabled": False,
                        "hardware_execution_authorized": False},
                    "yaw_effectiveness_hold_v5b": {"enabled": False}})
    m2_runtime_path = root / "configs/M2b_runtime.yaml"
    m2_runtime_path.write_text(yaml.safe_dump(runtime_data, sort_keys=False))
    source_auth = root / "configs" / f"{candidate}_M1_authorization.yaml"
    auth = yaml.safe_load(source_auth.read_text())
    auth["authorization_scope"]["upper_config"] = str(m2_path)
    auth["authorization_scope"]["runtime_config"] = str(m2_runtime_path)
    auth["evidence_scope"] = MARKERS
    auth["candidate_b"] = {"hardware_execution_authorized": False}
    (root / "configs" / f"{candidate}_M2b_authorization.yaml").write_text(
        yaml.safe_dump(auth, sort_keys=False))

    runtime_paths = [root / "configs" / f"{candidate}_{name}.yaml"
                     for name in ("M1", "M1b")]
    runtime_paths.append(root / "configs/M2b_runtime.yaml")
    for path in runtime_paths:
        data = yaml.safe_load(path.read_text())
        runtime = data["formal_fake_runtime"]
        runtime["experiment_id"] = IDENTITY
        runtime["evidence_scope"] = MARKERS
        runtime["classic_additive_candidate_A"] = {"enabled": False}
        runtime["classic_additive_physical"] = {"enabled": False,
                                                   "hardware_execution_authorized": False}
        runtime["candidate_b"] = dict(PROFILE)
        runtime["candidate_b_metadata"] = metadata
        path.write_text(yaml.safe_dump(data, sort_keys=False))
    for path in (root / "configs").glob("*_authorization.yaml"):
        data = yaml.safe_load(path.read_text())
        data["evidence_scope"] = MARKERS
        data["candidate_b"] = {"hardware_execution_authorized": False}
        path.write_text(yaml.safe_dump(data, sort_keys=False))
    loc = root / "configs/localization_v5_quality_observation.yaml"
    data = yaml.safe_load(loc.read_text())
    data.update({"experiment_id": IDENTITY, "platform_transport_type": "fake",
                 "hardware_execution_authorized": False})
    loc.write_text(yaml.safe_dump(data, sort_keys=False))
    topics = root / "configs/record_topics_v5.yaml"
    data = yaml.safe_load(topics.read_text())
    registry = data["experiment_recording"]
    for key in ("topics", "required_topics"):
        values = registry[key]
        unused = {"/multi_agv/classic_additive_disturbance_state",
                  "/multi_agv/yaw_effectiveness_hold_state",
                  "/multi_agv/yaw_effectiveness_state"}
        values[:] = [v for v in values if v not in unused]
        if "/multi_agv/candidate_B_disturbance_state" not in values:
            values.append("/multi_agv/candidate_B_disturbance_state")
        if method == "M2b" and "/multi_agv/m2b_algorithm_state" not in values:
            values.append("/multi_agv/m2b_algorithm_state")
    for required in registry.get("method_required_topics", {}).values():
        required[:] = [v for v in required if v not in unused]
        if "/multi_agv/candidate_B_disturbance_state" not in required:
            required.append("/multi_agv/candidate_B_disturbance_state")
    topics.write_text(yaml.safe_dump(data, sort_keys=False))
    validation = root / "configs/validation_candidate_b.yaml"
    data = yaml.safe_load((HERE.parent / "config" /
                           "validation_defaults.yaml").read_text())
    registry = data["experiment_recording"]
    for registered in METHODS.values():
        required = registry["method_required_topics"].setdefault(registered, [])
        required[:] = [v for v in required if v not in unused]
        if "/multi_agv/candidate_B_disturbance_state" not in required:
            required.append("/multi_agv/candidate_B_disturbance_state")
    registry["minimum_topic_rates"][
        "/multi_agv/candidate_B_disturbance_state"] = 90.
    validation.write_text(yaml.safe_dump(data, sort_keys=False))
    save(root / "CANDIDATE_B_SCOPE.json", {
        "experiment_id": IDENTITY, "markers": MARKERS,
        "formal_evidence": False, "parameter_search": False,
        "hardware_authorization": False, "serial_execution": False,
        "method": METHODS[method], "profile": PROFILE,
        "m2b_config_sha256": hashlib.sha256(
            (CONFIG / "exp2b_M2b.yaml").read_bytes()).hexdigest()})


def expected(t):
    if t < 0. or t >= 8.:
        return 0., 1., 0.
    if t < .5:
        u = t / .5
        q = u*u*u*(10. - 15.*u + 6.*u*u)
    elif t >= 7.5:
        u = (8. - t) / .5
        q = u*u*u*(10. - 15.*u + 6.*u*u)
    else:
        q = 1.
    return q, 1. - .15*q, q*.2625*math.sin(t + math.pi/2.)


def durations(rows):
    return V4.durations(rows)


def exposure(rows, values):
    dt = durations(rows)
    absolute = [abs(v) for v in values]
    return {"rmse": math.sqrt(V4.weighted_mean([v*v for v in values], dt)),
            "max": max(absolute),
            "iae": sum(v*t for v, t in zip(absolute, dt))}


def phase_metrics(rows):
    dt = durations(rows)
    out = {"samples": len(rows), "duration_seconds": sum(dt),
           "maximum_spacing_seconds": max(dt)}
    for key in ("longitudinal", "lateral", "heading", "support",
                "rigid_fit", "pairwise_side", "progress"):
        values = [r["errors"][key] for r in rows]
        for tag, value in exposure(rows, values).items():
            out[f"{key}_error_{tag}"] = value
    derived = {
        "equivalent_load_position": [math.hypot(
            V2.num(r, "equivalent_load_pose_x") - V2.num(r, "load_x_reference"),
            V2.num(r, "equivalent_load_pose_y") - V2.num(r, "load_y_reference"))
            for r in rows],
        "equivalent_load_yaw": [V2.wrap(
            V2.num(r, "equivalent_load_pose_yaw") -
            V2.num(r, "load_yaw_reference")) for r in rows],
        "robot2_velocity": [V2.num(r, "agv2_s_dot_actual") -
                            V2.num(r, "common_velocity_reference")
                            for r in rows]}
    for key, values in derived.items():
        for tag, value in exposure(rows, values).items():
            out[f"{key}_error_{tag}"] = value
    native = [(V2.num(r, "candidate_b_left_native"),
               V2.num(r, "candidate_b_right_native")) for r in rows]
    disturbed = [(V2.num(r, "candidate_b_left_post_disturbance"),
                  V2.num(r, "candidate_b_right_post_disturbance")) for r in rows]
    post_limit = [(V2.num(r, "candidate_b_left_post_limit"),
                   V2.num(r, "candidate_b_right_post_limit")) for r in rows]
    actual = [(V2.num(r, "candidate_b_actual_left"),
               V2.num(r, "candidate_b_actual_right")) for r in rows]
    peak = lambda values: max(abs(v) for pair in values for v in pair)
    native_differential = [right-left for left, right in native]
    limits = [(V2.num(r, "agv2_wheel_left_reported_limit"),
               V2.num(r, "agv2_wheel_right_reported_limit")) for r in rows]
    margins = [min(1., max(0., min(l-abs(a), rr-abs(b))/.010))
               for (a, b), (l, rr) in zip(native, limits)]
    rho = [V2.num(r, "candidate_b_rho") for r in rows]
    envelope = [V2.num(r, "candidate_b_envelope") for r in rows]
    dv = [V2.num(r, "candidate_b_v_c_after_effectiveness") -
          V2.num(r, "candidate_b_v_c_native") for r in rows]
    dw = [V2.num(r, "candidate_b_d_omega") for r in rows]
    native_v = [V2.num(r, "candidate_b_v_c_native") for r in rows]
    native_omega = [V2.num(r, "candidate_b_omega_native") for r in rows]
    post_v = [V2.num(r, "candidate_b_v_c_after_effectiveness") for r in rows]
    post_omega = [V2.num(r, "candidate_b_omega_after_disturbance") for r in rows]
    contraction = [V2.num(r, "risk_contraction") for r in rows]
    contraction = [value if math.isfinite(value) else 0. for value in contraction]
    rms = lambda values: math.sqrt(V4.weighted_mean([v*v for v in values], dt))
    out.update({
        "effectiveness_rho_minimum": min(rho),
        "effectiveness_rho_mean": V4.weighted_mean(rho, dt),
        "disturbance_envelope_mean": V4.weighted_mean(envelope, dt),
        "longitudinal_effectiveness_delta_peak_mps": max(abs(v) for v in dv),
        "longitudinal_effectiveness_delta_rms_mps": rms(dv),
        "yaw_disturbance_peak_radps": max(abs(v) for v in dw),
        "yaw_disturbance_rms_radps": rms(dw),
        "native_longitudinal_peak_mps": max(abs(v) for v in native_v),
        "native_longitudinal_rms_mps": rms(native_v),
        "native_yaw_peak_radps": max(abs(v) for v in native_omega),
        "native_yaw_rms_radps": rms(native_omega),
        "post_effectiveness_longitudinal_peak_mps": max(abs(v) for v in post_v),
        "post_disturbance_yaw_peak_radps": max(abs(v) for v in post_omega),
        "native_controller_raw_peak_mps": peak(native),
        "native_controller_raw_rms_mps": math.sqrt(V4.weighted_mean(
            [(left*left+right*right)/2. for left, right in native], dt)),
        "native_differential_wheel_peak_mps": max(
            abs(v) for v in native_differential),
        "native_differential_wheel_rms_mps": rms(native_differential),
        "native_wheel_demand_capability_ratio_peak": max(
            max(abs(left)/limit_left, abs(right)/limit_right)
            for (left, right), (limit_left, limit_right)
            in zip(native, limits)),
        "post_candidate_b_wheel_peak_mps": peak(disturbed),
        "post_limit_wheel_peak_mps": peak(post_limit),
        "actual_wheel_peak_mps": peak(actual),
        "wheel_margin_minimum": min(margins),
        "wheel_margin_mean": V4.weighted_mean(margins, dt),
        "wheel_margin_p05": V2.percentile(margins, .05),
        "wheel_margin_below_0p5_duration_seconds": sum(
            t for value, t in zip(margins, dt) if value < .5),
        "wheel_margin_below_0p7_duration_seconds": sum(
            t for value, t in zip(margins, dt) if value < .7),
        "native_over_0p160_duration_seconds": sum(
            t for pair, t in zip(native, dt) if peak([pair]) > .160),
        "post_candidate_b_over_0p160_duration_seconds": sum(
            t for pair, t in zip(disturbed, dt) if peak([pair]) > .160),
        "native_over_0p18_duration_seconds": sum(
            t for r, t in zip(rows, dt) if V2.flag(r, "candidate_b_native_over_0p18")),
        "post_disturbance_over_0p18_duration_seconds": sum(
            t for r, t in zip(rows, dt) if V2.flag(r, "candidate_b_post_disturbance_over_0p18")),
        "limiter_duration_seconds": sum(
            t for r, t in zip(rows, dt) if
            V2.flag(r, "agv2_wheel_left_speed_limit_active") or
            V2.flag(r, "agv2_wheel_right_speed_limit_active")),
        "capability_report_unchanged": all(
            abs(l - .160) <= 1e-12 and abs(rr - .160) <= 1e-12
            for l, rr in limits),
        "risk_contraction_peak": max(contraction),
        "risk_contraction_integral": sum(
            value*t for value, t in zip(contraction, dt)),
        "common_reference_minimum": min(
            V2.num(r, "common_velocity_reference") for r in rows),
        "common_reference_mean": V4.weighted_mean(
            [V2.num(r, "common_velocity_reference") for r in rows], dt)})
    return out


def analyze(run, method, log):
    params = yaml.safe_load((run / "rosparams.yaml").read_text())
    manifest = yaml.safe_load((run / "manifest.yaml").read_text())
    p = params["formal_fake_algorithm"]
    runtime = p["formal_fake_runtime"]
    assert manifest["experiment_id"] == runtime["experiment_id"] == IDENTITY
    assert runtime["leader"]["velocity"] == .10
    assert runtime["candidate_b"] == PROFILE
    assert not runtime["classic_additive_candidate_A"]["enabled"]
    for i in (1, 2, 3):
        assert params[f"agv{i}"]["chassis_controller"]["transport_type"] == "fake"
    if method == "M2b":
        frozen = yaml.safe_load((CONFIG / "exp2b_M2b.yaml").read_text())
        for key in ("formal_m2b", "formal_lower", "formal_upper"):
            assert p[key] == frozen[key]
    rows_all = list(csv.DictReader((run / "converted/aligned_samples.csv").open()))
    rows, seen = [], set()
    for source in rows_all:
        if not V2.flag(source, "algorithm_valid") or not V2.flag(
                source, "candidate_b_state_available"):
            continue
        wall = V2.num(source, "candidate_b_wall_time")
        if wall in seen:
            continue
        seen.add(wall)
        row = dict(source)
        row["errors"] = V4.error_signals(row)
        for key in ("longitudinal", "lateral", "heading"):
            row["errors"][key] = V2.num(
                row, f"agv2_candidate_b_tracker_{key}_error")
        rows.append(row)
    triggered = next(r for r in rows if V2.flag(r, "candidate_b_triggered"))
    trigger = V2.num(triggered, "candidate_b_trigger_wall_time")
    profile_error = mapping_error = 0.
    for row in rows:
        t = V2.num(row, "candidate_b_wall_time") - trigger
        row["relative_wall_time"] = t
        q, rho, dw = expected(t)
        profile_error = max(profile_error,
            abs(V2.num(row, "candidate_b_envelope")-q),
            abs(V2.num(row, "candidate_b_rho")-rho),
            abs(V2.num(row, "candidate_b_d_omega")-dw))
        b = V2.num(row, "candidate_b_wheel_separation")
        vc = V2.num(row, "candidate_b_v_c_native")
        omega = V2.num(row, "candidate_b_omega_native")
        left = rho*vc - b/2.*(omega+dw)
        right = rho*vc + b/2.*(omega+dw)
        mapping_error = max(mapping_error,
            abs(left-V2.num(row, "candidate_b_left_post_disturbance")),
            abs(right-V2.num(row, "candidate_b_right_post_disturbance")),
            abs(omega-V2.num(row, "candidate_b_omega_before_added_disturbance")))
    rows.sort(key=lambda r: r["relative_wall_time"])
    if profile_error >= 1e-9 or mapping_error >= 1e-9:
        raise ValueError("Candidate B waveform or component mapping mismatch")
    selected = {
        "baseline": [r for r in rows if -3 <= r["relative_wall_time"] < 0],
        "disturbance": [r for r in rows if 0 <= r["relative_wall_time"] < 8],
        "recovery": [r for r in rows if 8 <= r["relative_wall_time"] < 23],
        "primary": [r for r in rows if 0 <= r["relative_wall_time"] < 23]}
    phases = {k: phase_metrics(v) for k, v in selected.items() if v}
    validation = json.loads((run / "validation.json").read_text())
    summary = json.loads((run / "summary_metrics.json").read_text())
    complete = bool(validation.get("task_completion", {}).get("complete"))
    invalid_log = any(token in log.read_text(errors="replace").lower()
                      for token in ("numerical_invalid", "state_chain_invalid",
                                    "held fail-zero:", "safety abort latched"))
    category = "VALID_COMPLETED" if complete and not invalid_log else "INVALID_RUN"
    mechanism = None
    if method == "M1":
        rs = selected["primary"]
        dt = durations(rs)
        contraction = [V2.num(r, "risk_contraction") for r in rs]
        reduction = [V2.num(r, "candidate_common_velocity")-
                     V2.num(r, "common_velocity_reference") for r in rs]
        mechanism = {
            "robust_margin_minimum": min(V2.num(r, "agv2_robust_margin") for r in rs),
            "robust_margin_mean": V4.weighted_mean(
                [V2.num(r, "agv2_robust_margin") for r in rs], dt),
            "risk_contraction_peak": max(contraction),
            "risk_contraction_integral": sum(v*t for v, t in zip(contraction, dt)),
            "risk_factor_peak": max(V2.num(r, "agv2_risk_factor") for r in rs),
            "risk_factor_mean": V4.weighted_mean(
                [V2.num(r, "agv2_risk_factor") for r in rs], dt),
            "risk_signal_peak": max(V2.num(r, "agv2_risk_signal") for r in rs),
            "risk_signal_mean": V4.weighted_mean(
                [V2.num(r, "agv2_risk_signal") for r in rs], dt),
            "risk_boundary_active_duration": sum(
                t for r, t in zip(rs, dt) if V4.V3.boundary_active(r)),
            "actual_reference_reduction_peak": max(reduction),
            "actual_reference_reduction_mean": V4.weighted_mean(reduction, dt),
            "common_reference_minimum": min(
                V2.num(r, "common_velocity_reference") for r in rs),
            "common_reference_mean": V4.weighted_mean(
                [V2.num(r, "common_velocity_reference") for r in rs], dt)}
    result = {"experiment_id": IDENTITY, "markers": MARKERS,
        "formal_evidence": False, "method": METHODS[method],
        "category": category, "complete": complete,
        "task_time_seconds": summary.get("task", {}).get("completion_time"),
        "phases": phases, "M1_mechanism": mechanism,
        "profile_maximum_absolute_error": profile_error,
        "mapping_maximum_absolute_error": mapping_error,
        "capability_report_unchanged": True, "run_path": str(run)}
    return result


def percent(m1, baseline):
    return 100.*(baseline-m1)/baseline if abs(baseline) > 1e-12 else None


def aggregate(root):
    data = {m: json.loads((root / folder / "run/candidate_b_metrics.json").read_text())
            for m, folder in {"M1": "M1_R1", "M1b": "M1b_R1",
                              "M2b": "M2b_M2b"}.items()}
    valid = all(v["category"] == "VALID_COMPLETED" for v in data.values())
    disturbance = {m: d["phases"]["disturbance"] for m, d in data.items()}
    mechanism = data["M1"]["M1_mechanism"] or {}
    too_strong = (not valid or any(
        d["post_disturbance_over_0p18_duration_seconds"] > 0 or
        d["limiter_duration_seconds"] > .5 for d in disturbance.values()))
    mechanism_active = (mechanism.get("risk_contraction_peak", 0) > 0 and
                        mechanism.get("actual_reference_reduction_peak", 0) > 0)
    geometry_better = any(disturbance["M1"][key] < disturbance[c][key]
        for c in ("M1b", "M2b") for key in
        ("support_error_rmse", "rigid_fit_error_rmse",
         "pairwise_side_error_rmse"))
    conclusion = ("RHO_0P85_TOO_STRONG" if too_strong else
                  "RHO_0P85_ACCEPT" if mechanism_active and geometry_better else
                  "RHO_0P85_MECHANISM_TOO_WEAK")
    comparisons = {}
    keys = ["support_error_rmse", "support_error_max",
            "rigid_fit_error_rmse", "rigid_fit_error_max",
            "pairwise_side_error_rmse", "pairwise_side_error_max",
            "progress_error_rmse", "lateral_error_rmse", "heading_error_rmse",
            "native_controller_raw_peak_mps", "wheel_margin_minimum",
            "limiter_duration_seconds"]
    for comparator in ("M1b", "M2b"):
        comparisons[f"M1_vs_{comparator}"] = {key: {
            "M1": disturbance["M1"][key], comparator: disturbance[comparator][key],
            "improvement_percent": percent(disturbance["M1"][key],
                                           disturbance[comparator][key])}
            for key in keys}
    result = {"experiment_id": IDENTITY, "markers": MARKERS,
              "profile": PROFILE, "methods": data,
              "comparisons": comparisons, "all_runs_valid": valid,
              "conclusion": conclusion,
              "recommend_rho_0p90": conclusion == "RHO_0P85_TOO_STRONG"}
    save(root / "candidate_b_comparison.json", result)
    rows = []
    for method, values in disturbance.items():
        row = {"method": METHODS[method], "task_time_seconds": data[method]["task_time_seconds"]}
        row.update(values); rows.append(row)
    with (root / "candidate_b_metrics.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader(); writer.writerows(rows)
    lines = ["# Candidate B rho=0.85 fake results", "",
        "EXPLORATORY_FAKE_VALIDATION / NOT_FORMAL_PAPER_EVIDENCE", "",
        f"Commit: `{Path(root / 'start_head.txt').read_text().strip()}`", "",
        "Formula: `v_c_exe=rho*v_c_native`, `omega_exe=omega_native+d_omega`; native omega is not scaled by rho.",
        "Frozen profile: Robot2 only, `rho_min=0.85`, `d_omega_peak=0.2625 rad/s`, progress trigger `2.0 m`, duration `8.0 s`, q5 ramp-in/out `0.5 s`.",
        "M1/M1b use frozen R1; M2b uses its complete frozen upper/reference generator and nonlinear lower controller. All runs use fake transport.", "",
        f"Final decision: **{conclusion}**", "",
        "| Method | Valid | task/s | native peak | post-B peak | post-limit peak | actual peak | post-B >0.160/s | limiter/s | post-B >0.18/s | support RMS/max | rigid RMS/max | side RMS/max | R2 progress/lateral/heading RMS |", 
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |"]
    for method in ("M1", "M1b", "M2b"):
        d, p = data[method], disturbance[method]
        lines.append("| {} | {} | {:.6g} | {:.6g} | {:.6g} | {:.6g} | {:.6g} | {:.6g} | {:.6g} | {:.6g} | {:.6g}/{:.6g} | {:.6g}/{:.6g} | {:.6g}/{:.6g} | {:.6g}/{:.6g}/{:.6g} |".format(
            METHODS[method], d["category"], d["task_time_seconds"],
            p["native_controller_raw_peak_mps"], p["post_candidate_b_wheel_peak_mps"],
            p["post_limit_wheel_peak_mps"], p["actual_wheel_peak_mps"],
            p["post_candidate_b_over_0p160_duration_seconds"], p["limiter_duration_seconds"],
            p["post_disturbance_over_0p18_duration_seconds"],
            p["support_error_rmse"], p["support_error_max"],
            p["rigid_fit_error_rmse"], p["rigid_fit_error_max"],
            p["pairwise_side_error_rmse"], p["pairwise_side_error_max"],
            p["progress_error_rmse"], p["lateral_error_rmse"], p["heading_error_rmse"]))
    lines += ["", "The five execution layers are reported independently as native controller demand, post-effectiveness longitudinal component, post-yaw Candidate-B demand, post-0.160 limiter feedback, and actual fake-plant feedback.", "",
              "## Absolute and percentage comparisons", ""]
    for comparator in ("M1b", "M2b"):
        lines += [f"M1 versus {comparator} (positive percentage means lower M1 error/burden):", "",
                  "| Metric | M1 | {} | improvement |".format(comparator),
                  "| --- | ---: | ---: | ---: |"]
        for key in keys:
            value = comparisons[f"M1_vs_{comparator}"][key]
            improvement = ("n/a" if value["improvement_percent"] is None else
                           "{:.3f}%".format(value["improvement_percent"]))
            lines.append("| {} | {:.8g} | {:.8g} | {} |".format(
                key, value["M1"], value[comparator], improvement))
        lines.append("")
    lines += ["## M1 causal mechanism", "", "```json",
              json.dumps(mechanism, indent=2, sort_keys=True), "```", "",
              "All three CapabilityReport wheel limits remained exactly 0.160 m/s; Candidate B does not mutate capability.",
              f"Recommend fallback to rho=0.90: `{result['recommend_rho_0p90']}`.",
              "No physical/serial execution was authorized or started."]
    (root / "candidate-B-rho0p85-fake-results.md").write_text("\n".join(lines)+"\n")
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
        prepare(args.prepare.resolve(), args.method); return 0
    if args.aggregate:
        aggregate(args.aggregate.resolve()); return 0
    try:
        result = analyze(args.check_run.resolve(), args.method,
                         args.check_log.resolve())
    except Exception as exc:
        result = {"experiment_id": IDENTITY, "markers": MARKERS,
                  "category": "INVALID_RUN", "reason": str(exc)}
    save(args.check_run / "candidate_b_metrics.json", result)
    print(result["category"])
    return 20 if result["category"] == "INVALID_RUN" else 0


if __name__ == "__main__":
    raise SystemExit(main())

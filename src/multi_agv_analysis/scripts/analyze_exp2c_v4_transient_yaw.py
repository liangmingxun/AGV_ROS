#!/usr/bin/env python3
"""Frozen, fake-only transient-yaw screening and recovery analysis."""
import argparse
import csv
import importlib.util
import json
import math
from pathlib import Path
import statistics
import sys
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from multi_agv_analysis.metrics import _rigid_fit_residual

SPEC = importlib.util.spec_from_file_location("v3_reference_diagnostics",
    str(Path(__file__).with_name("analyze_exp2c_v3_nominal_headroom.py")))
V3 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(V3)
V2 = V3._V2
IDENTITY = "exp2c_v4_transient_yaw_recovery"
CANDIDATES = {"A": (.024, 1.5), "B": (.022, 2.0)}
BASELINE_SECONDS = 3.0
RECOVERY_SECONDS = 15.0
HOLD_SECONDS = .5
FLOORS = {"heading": .010, "support": .002, "rigid_fit": .001}


def audit_metadata(run):
    manifest = yaml.safe_load((run / "manifest.yaml").read_text())
    params = yaml.safe_load((run / "rosparams.yaml").read_text())
    runtime = params.get("formal_fake_algorithm", {}).get("formal_fake_runtime", {})
    if manifest.get("experiment_id") != IDENTITY or runtime.get("experiment_id") != IDENTITY:
        raise ValueError("not fresh Exp2c-v4 evidence")
    for value in (manifest.get("metrics", {}).get("nominal_common_velocity"),
                  runtime.get("leader", {}).get("velocity")):
        if value is None or not math.isfinite(float(value)) or abs(float(value)-.100) > 1e-12:
            raise ValueError("nominal/runtime velocity must be .100 m/s")
    for robot in ("agv1", "agv2", "agv3"):
        if params.get(robot, {}).get("chassis_controller", {}).get("transport_type") != "fake":
            raise ValueError("serial/non-fake transport is prohibited")
    def check(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key.endswith("hardware_execution_authorized") and item is not False:
                    raise ValueError("hardware authorization is not false")
                check(item)
        elif isinstance(value, list):
            for item in value: check(item)
    check(params)
    pulse = runtime.get("transient_yaw_v4", {})
    if pulse.get("enabled") is not True or pulse.get("trigger_progress") != 2.0:
        raise ValueError("incorrect transient trigger configuration")
    if runtime.get("yaw_drive_disturbance_v2", {}).get("enabled", False) or runtime.get("risk_disturbance_v1", {}).get("enabled", False):
        raise ValueError("legacy disturbance is also enabled")
    return manifest, pulse


def error_signals(row):
    actual = [(V2.num(row, "agv{}_support_pose_x".format(i)),
               V2.num(row, "agv{}_support_pose_y".format(i))) for i in (1, 2, 3)]
    reference = [(V2.num(row, "agv{}_support_reference_x".format(i)),
                  V2.num(row, "agv{}_support_reference_y".format(i))) for i in (1, 2, 3)]
    side = [abs(math.dist(actual[i], actual[j])-math.dist(reference[i], reference[j]))
            for i, j in ((0, 1), (0, 2), (1, 2))]
    return {
        "heading": V2.wrap(V2.num(row, "agv2_robot_pose_yaw")-V2.num(row, "agv2_support_reference_yaw")),
        "support": math.dist(actual[1], reference[1]),
        "rigid_fit": _rigid_fit_residual(actual, reference),
        "pairwise_side": max(side),
        "progress": V2.num(row, "agv2_s_tracking_actual")-V2.num(row, "load_s_reference"),
    }


def durations(rows):
    if not rows: raise ValueError("empty analysis phase")
    delta = [rows[i+1]["relative_wall_time"]-rows[i]["relative_wall_time"]
             for i in range(len(rows)-1)]
    if any(value <= 0 or value > .10 for value in delta):
        raise ValueError("non-contiguous wall-clock observation")
    return delta + [statistics.median(delta) if delta else .01]


def weighted_mean(values, dt):
    if not values or not all(math.isfinite(value) for value in values):
        raise ValueError("missing/nonfinite metrics")
    return sum(value*weight for value, weight in zip(values, dt))/sum(dt)


def limiter_locations(rows):
    """Locate speed limiting without conflating startup with the yaw pulse."""
    limited = [row for row in rows if any(V2.flag(row,
        "agv{}_wheel_{}_speed_limit_active".format(i, side))
        for i in (1, 2, 3) for side in ("left", "right"))]
    return {
        "samples": len(limited),
        "duration_seconds": sum(dt for row, dt in zip(rows, durations(rows))
            if any(V2.flag(row, "agv{}_wheel_{}_speed_limit_active".format(i, side))
                for i in (1, 2, 3) for side in ("left", "right"))),
        "all_before_transient_trigger": bool(limited) and all(
            not V2.flag(row, "transient_yaw_triggered") for row in limited),
        "robot2_progress_range_m": [min(V2.num(row, "transient_yaw_progress") for row in limited),
            max(V2.num(row, "transient_yaw_progress") for row in limited)] if limited else None,
    }


def envelope(values, floor):
    if not values or not all(math.isfinite(value) for value in values):
        raise ValueError("missing baseline error")
    median = statistics.median(values)
    mad = statistics.median(abs(value-median) for value in values)
    radius = max(3.0*mad, floor)
    return {"median": median, "MAD": mad, "floor": floor,
            "lower": median-radius, "upper": median+radius}


def recovery_time(rows, key, bounds, end):
    """First re-entry sustained for 0.5 s; missing horizon is censored, not zero."""
    start = previous = None
    for row in rows:
        time = row["relative_wall_time"]
        if time < end: continue
        if previous is not None and time-previous > .03:
            start = None
        value = row["errors"][key]
        if math.isfinite(value) and bounds["lower"] <= value <= bounds["upper"]:
            if start is None: start = time
            if time-start >= HOLD_SECONDS:
                return max(0.0, start-end)
        else:
            start = None
        previous = time
    return None


def phase_metrics(rows):
    dt = durations(rows)
    result = {"samples": len(rows), "observation_duration_seconds": sum(dt)}
    for name, field in (("robust_margin", "agv2_robust_margin"),
                        ("wheel_margin", "transient_yaw_causal_wheel_margin")):
        values = [V2.num(row, field) for row in rows]
        result[name+"_minimum"] = min(values)
        result[name+"_p05"] = V2.percentile(values, .05)
        result[name+"_mean"] = weighted_mean(values, dt)
        if name == "wheel_margin":
            for threshold in (1.0, .7, .5):
                seconds = sum(weight for value, weight in zip(values, dt) if value < threshold)
                tag = str(threshold).replace(".", "p")
                result[name+"_below_"+tag+"_duration_seconds"] = seconds
                result[name+"_below_"+tag+"_fraction"] = seconds/sum(dt)
    for key in ("risk_contraction", "common_velocity_reference"):
        values = [V2.num(row, key) for row in rows]
        result[key+"_minimum"] = min(values)
        result[key+"_peak"] = max(values)
        result[key+"_mean"] = weighted_mean(values, dt)
        if key == "risk_contraction": result[key+"_integral"] = sum(v*t for v,t in zip(values,dt))
    active = sum(weight for row, weight in zip(rows, dt) if V3.boundary_active(row))
    result["risk_boundary_active_fraction"] = active/sum(dt)
    result["risk_boundary_active_duration_seconds"] = active
    result.update(V3.reference_reduction_metrics(rows))
    raw = [(V2.num(row, "transient_yaw_left_raw"), V2.num(row, "transient_yaw_right_raw")) for row in rows]
    disturbed = [(V2.num(row, "transient_yaw_left_disturbed"), V2.num(row, "transient_yaw_right_disturbed")) for row in rows]
    raw_peaks = [max(abs(l), abs(r)) for l,r in raw]
    result["controller_raw_wheel_peak_mps"] = max(raw_peaks)
    result["controller_raw_wheel_rms_mps"] = math.sqrt(weighted_mean([(l*l+r*r)/2 for l,r in raw], dt))
    result["controller_raw_above_0p150_duration_seconds"] = sum(t for v,t in zip(raw_peaks,dt) if v > .150)
    result["wheel_demand_capability_ratio_peak"] = max(
        max(abs(l)/V2.num(row,"agv2_wheel_left_reported_limit"),
            abs(r)/V2.num(row,"agv2_wheel_right_reported_limit")) for row,(l,r) in zip(rows,raw))
    result["disturbed_wheel_peak_mps"] = max(abs(v) for pair in disturbed for v in pair)
    result["physical_speed_limiter_duration_seconds"] = sum(t for row,t in zip(rows,dt) if any(
        V2.flag(row,"agv{}_wheel_{}_speed_limit_active".format(i,side)) for i in (1,2,3) for side in ("left","right")))
    for key in ("heading", "support", "rigid_fit", "pairwise_side", "progress"):
        values = [abs(row["errors"][key]) for row in rows]
        result[key+"_error_peak"] = max(values)
        result[key+"_error_IAE"] = sum(v*t for v,t in zip(values,dt))
        result[key+"_error_rms"] = math.sqrt(weighted_mean([v*v for v in values],dt))
    return result


def analyze_run(run, candidate, log_path=None):
    manifest, config = audit_metadata(run)
    amp, total = CANDIDATES[candidate]
    observed_amp, observed_duration = float(config.get("amplitude",math.nan)), float(config.get("duration",math.nan))
    if not math.isfinite(observed_amp+observed_duration) or abs(observed_amp-amp) > 1e-12 or abs(observed_duration-total) > 1e-12:
        raise ValueError("candidate differs from frozen configuration")
    validation = json.loads((run / "validation.json").read_text())
    summary_path = run / "summary_metrics.json"
    summary = json.loads(summary_path.read_text()) if summary_path.exists() else {}
    with (run / "converted/aligned_samples.csv").open(newline="") as stream:
        all_rows = list(csv.DictReader(stream))
    log = log_path.read_text(errors="replace") if log_path and log_path.exists() else ""
    emergency = "safety abort latched" in log.lower() or "raw wheel demand threshold event" in log.lower()
    rigid_rejection = "hard gate rejected" in log.lower()
    raw_evidence = V2.finite([V2.num(row,"agv{}_wheel_{}_pre_limit".format(i,side))
        for row in all_rows for i in (1,2,3) for side in ("left","right")])
    speed_limiter_evidence = any(V2.flag(row,"agv{}_wheel_{}_speed_limit_active".format(i,side))
        for row in all_rows for i in (1,2,3) for side in ("left","right"))
    safety_evidence = bool(emergency or rigid_rejection or speed_limiter_evidence or
        raw_evidence and max(abs(v) for v in raw_evidence) >= .160)
    rows = []
    seen = set()
    for row in all_rows:
        if not V2.flag(row,"algorithm_valid") or not V2.flag(row,"transient_yaw_state_available"): continue
        time = V2.num(row,"transient_yaw_wall_time")
        if not math.isfinite(time) or time in seen: continue
        seen.add(time); rows.append(row)
    triggers = {V2.num(row,"transient_yaw_trigger_wall_time") for row in rows if V2.flag(row,"transient_yaw_triggered")}
    if len(triggers) != 1 or not all(math.isfinite(v) for v in triggers):
        raise ValueError("missing/multiple transient wall-clock triggers")
    trigger = next(iter(triggers))
    for row in rows:
        row["relative_wall_time"] = V2.num(row,"transient_yaw_wall_time")-trigger
        row["errors"] = error_signals(row)
    rows.sort(key=lambda row: row["relative_wall_time"])
    baseline = [row for row in rows if -BASELINE_SECONDS <= row["relative_wall_time"] < 0]
    pulse = [row for row in rows if 0 <= row["relative_wall_time"] < total]
    recovery = [row for row in rows if total <= row["relative_wall_time"] < total+RECOVERY_SECONDS]
    if len(baseline) < 200 or not pulse or not recovery:
        if safety_evidence:
            return {"run_dir":str(run),"candidate":candidate,"screen_passed":False,
                "fallback_B_allowed":True,"validation_valid":bool(validation.get("valid")),
                "emergency_observed":emergency,"rigid_gate_rejection_observed":rigid_rejection,
                "reason":"safety/rigidity/limiter invalid with truncated recovery; no gate relaxed"}
        raise ValueError("insufficient baseline/pulse/recovery evidence")
    bounds = {key: envelope([row["errors"][key] for row in baseline], floor) for key,floor in FLOORS.items()}
    rec_times = {key: recovery_time(recovery,key,bounds[key],total) for key in FLOORS}
    phases = {name: phase_metrics(selected) for name,selected in
              (("baseline",baseline),("pulse",pulse),("recovery",recovery),("post_trigger",pulse+recovery))}
    full_raw_peak = max(abs(V2.num(row,"agv{}_wheel_{}_pre_limit".format(i,side)))
                        for row in rows for i in (1,2,3) for side in ("left","right"))
    full_limiter_samples = sum(any(V2.flag(row,"agv{}_wheel_{}_speed_limit_active".format(i,side))
        for i in (1,2,3) for side in ("left","right")) for row in rows)
    derating_samples = sum(V2.flag(row,"agv{}_capability_derating_active".format(i))
                           for row in rows for i in (1,2,3))
    reported_limits = [V2.num(row,"agv{}_wheel_{}_reported_limit".format(i,side)) for row in rows for i in (1,2,3) for side in ("left","right")]
    normal_capability = all(math.isfinite(v) and abs(v-.160)<1e-12 for v in reported_limits)
    finished = [row for row in rows if row["relative_wall_time"] >= total]
    exact_end = finished and all(V2.num(row,"transient_yaw_disturbance") == 0 for row in finished)
    waveform_valid = exact_end and all(
        V2.num(row,"transient_yaw_disturbance") >= 0 and
        abs(V2.num(row,"transient_yaw_mean_longitudinal_delta")) < 1e-12 and
        abs(V2.num(row,"transient_yaw_amplitude")-amp) < 1e-12 and
        abs(V2.num(row,"transient_yaw_duration")-total) < 1e-12 for row in rows)
    task_complete = validation.get("task_completion",{}).get("complete",False)
    safety_invalid = safety_evidence or full_raw_peak >= .160 or full_limiter_samples > 0
    mechanism = phases["post_trigger"]["wheel_margin_below_1p0_duration_seconds"] > 0
    horizon_complete = recovery[-1]["relative_wall_time"] >= total+RECOVERY_SECONDS-.03
    passed = bool(validation.get("valid") and task_complete and normal_capability and
        derating_samples == 0 and not safety_invalid and waveform_valid and
        mechanism and horizon_complete and all(v is not None for v in rec_times.values()))
    return {
        "run_dir": str(run), "method_id": manifest["method_id"], "candidate":candidate,
        "amplitude_mps":amp, "duration_seconds":total,
        "validation_valid":bool(validation.get("valid")),
        "validation_issue_codes":[issue.get("code") for issue in validation.get("issues",[])],
        "task_complete":task_complete, "task_completion_time_seconds":summary.get("task",{}).get("completion_time"),
        "normal_capability_reports":normal_capability, "derating_samples":derating_samples,
        "mapped_capability_minimum_mps":min(V2.num(row,"agv2_mapped_path_velocity_upper") for row in rows),
        "mapped_capability_maximum_mps":max(V2.num(row,"agv2_mapped_path_velocity_upper") for row in rows),
        "all_car_controller_raw_peak_mps":full_raw_peak,
        "full_run_physical_speed_limiter_samples":full_limiter_samples,
        "physical_speed_limiter_locations":limiter_locations(rows),
        "emergency_observed":emergency, "rigid_gate_rejection_observed":rigid_rejection,
        "waveform_valid":bool(waveform_valid), "wheel_margin_entered_risk_region":mechanism,
        "recovery_horizon_complete":horizon_complete,
        "recovery_times_seconds":rec_times, "recovery_envelopes":bounds,
        "phases":phases, "screen_passed":passed,
        "fallback_B_allowed":bool(safety_invalid),
        "nominal_common_velocity_mps":manifest["metrics"]["nominal_common_velocity"],
    }


def write_report(path, value):
    path.write_text(json.dumps(value,indent=2,sort_keys=True,allow_nan=False)+"\n")


def paired_report(root, candidate):
    screen = json.loads((root/"selected_candidate.json").read_text())
    if not screen.get("screen_passed") or screen.get("candidate") != candidate:
        raise ValueError("no frozen passing screening candidate")
    methods = {}
    for method in ("M1b", "M1"):
        methods[method] = analyze_run(root/"paired"/(method+"_R1"),candidate,
            root/"logs/paired"/(method+"_R1")/"algorithm.log")
        if "phases" not in methods[method]:
            write_report(root/"paired_failed_admissibility.json",methods)
            raise ValueError("paired {} has truncated unsafe evidence; stop without tuning".format(method))
    m1,m1b = methods["M1"],methods["M1b"]
    a,b = m1["phases"]["post_trigger"],m1b["phases"]["post_trigger"]
    ra,rb = m1["phases"]["recovery"],m1b["phases"]["recovery"]
    questions = {
        "A_disturbance_created_high_controller_burden": b["controller_raw_wheel_peak_mps"] > m1b["phases"]["baseline"]["controller_raw_wheel_peak_mps"] and m1b["wheel_margin_entered_risk_region"],
        "B_m1_wheel_and_robust_margins_declined":a["wheel_margin_minimum"] < m1["phases"]["baseline"]["wheel_margin_minimum"] and a["robust_margin_minimum"] < m1["phases"]["baseline"]["robust_margin_minimum"],
        "C_m1_boundary_active":a["risk_boundary_active_duration_seconds"] > 0,
        "D_m1_actual_reference_reduction":a["actual_reference_reduction_peak_mps"] > m1["phases"]["baseline"]["actual_reference_reduction_peak_mps"],
        "E_controller_raw_peak_reduced":a["controller_raw_wheel_peak_mps"] < b["controller_raw_wheel_peak_mps"],
        "E_recovery_raw_above_0p150_duration_reduced":ra["controller_raw_above_0p150_duration_seconds"] < rb["controller_raw_above_0p150_duration_seconds"],
        "E_recovery_wheel_margin_below_0p5_duration_reduced":ra["wheel_margin_below_0p5_duration_seconds"] < rb["wheel_margin_below_0p5_duration_seconds"],
        "G_no_physical_speed_limiter":not (m1["full_run_physical_speed_limiter_samples"] or m1b["full_run_physical_speed_limiter_samples"]),
    }
    for key in FLOORS:
        first, second = m1["recovery_times_seconds"][key], m1b["recovery_times_seconds"][key]
        questions["F_"+key+"_recovery_faster"] = first is not None and second is not None and first < second
    benefit = any(questions[key] for key in questions if key.startswith(("E_","F_")))
    report = {"experiment":IDENTITY,"selected_candidate":candidate,"screening_run_reused":False,
        "methods":methods,"causal_questions":questions,
        "paired_admissible":all(value["screen_passed"] for value in methods.values()),
        "full_causal_chain_supported_in_this_fake_pair":all(questions.values()) and all(value["screen_passed"] for value in methods.values()),
        "any_execution_or_recovery_benefit_observed":benefit,
        "stop_without_further_tuning":True,
        "hardware_authorization":False,"evidence_scope":"one fresh ROS fake pair, not hardware conclusion"}
    write_report(root/"paired_comparison.json",report)
    lines = ["# Exp2c-v4 transient yaw recovery", "", "Frozen candidate: "+candidate,
             "", "Statistics use a fixed 15 s recovery horizon; no retrospective criterion selection.",
             "", "| phase / metric | M1b+R1 | M1+R1 |", "|---|---:|---:|"]
    for phase in ("baseline","pulse","recovery","post_trigger"):
        for key in m1["phases"][phase]:
            lines.append("| {}/{} | {} | {} |".format(phase,key,m1b["phases"][phase][key],m1["phases"][phase][key]))
    for key in FLOORS:
        lines.append("| {} recovery time / s | {} | {} |".format(key,m1b["recovery_times_seconds"][key],m1["recovery_times_seconds"][key]))
    lines += ["", "## Causal checks", ""]+["- {}: {}".format(key,value) for key,value in questions.items()]
    lines += ["", "Full beneficial chain supported: "+str(report["full_causal_chain_supported_in_this_fake_pair"]),
              "No further tuning; no serial, commit or push."]
    (root/"paired_comparison.md").write_text("\n".join(lines)+"\n")
    with (root/"paired_metrics.csv").open("w",newline="") as stream:
        writer = csv.writer(stream); writer.writerow(["phase","metric","M1b_R1","M1_R1"])
        for phase in ("baseline","pulse","recovery","post_trigger"):
            for key in m1["phases"][phase]: writer.writerow([phase,key,m1b["phases"][phase][key],m1["phases"][phase][key]])
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate",choices=tuple(CANDIDATES),required=True)
    parser.add_argument("--screen",type=Path)
    parser.add_argument("--log",type=Path)
    parser.add_argument("--report",type=Path)
    parser.add_argument("--freeze-output",type=Path)
    parser.add_argument("--paired-root",type=Path)
    args = parser.parse_args()
    try:
        if args.screen:
            value = analyze_run(args.screen.resolve(),args.candidate,args.log)
            if args.report: write_report(args.report,value)
            if args.freeze_output and value["screen_passed"]:
                if args.freeze_output.exists():
                    raise ValueError("frozen candidate already exists")
                write_report(args.freeze_output,value)
            print(json.dumps(value,indent=2,sort_keys=True,allow_nan=False))
            return 0 if value["screen_passed"] else 10 if value["fallback_B_allowed"] else 11
        if args.paired_root:
            value = paired_report(args.paired_root.resolve(),args.candidate)
            print(json.dumps(value,indent=2,sort_keys=True,allow_nan=False))
            return 0
        parser.error("--screen or --paired-root is required")
    except Exception as error:
        print("EXP2C_V4_STOP: {}".format(error),file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())

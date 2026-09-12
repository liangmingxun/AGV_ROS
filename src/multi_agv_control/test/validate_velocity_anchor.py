#!/usr/bin/env python3
"""Read-only replay + sensitivity summary; no ROS imports or vehicle commands.

Usage: python3 validate_velocity_anchor.py /path/to/validation_binary /path/to/run
The replay freezes measured states and R1 acceleration; it cannot predict new
closed-loop tracking error. Simulation calls the production R1 C++ source, but
its one-dimensional plant and gain/lag grid are uncalibrated assumptions.
"""
import csv
import io
import json
import math
import pathlib
import statistics
import subprocess
import sys


def rms(values):
    return math.sqrt(statistics.mean(x * x for x in values))


def replay(binary, run, tau):
    summary = json.loads((run / "summary_metrics.json").read_text())
    start = summary["recovery"]["derating_command_start_stamp"] + 6.82
    end = summary["recovery"]["derating_command_end_stamp"] - 3.88
    with (run / "converted" / "aligned_samples.csv").open() as stream:
        rows = [x for x in csv.DictReader(stream)
                if start < float(x["stamp"]) < end
                and x["evaluation_active"] == "True"
                and x["localization_valid"] == "True"]
    result = {}
    for index in (1, 2, 3):
        prefix = "agv{}_".format(index)
        measured = [float(x[prefix + "s_dot_actual"]) for x in rows]
        acceleration = [float(x[prefix + "channel_input_limited"]) for x in rows]
        original = [float(x[prefix + "s_dot_execute_reference"]) for x in rows]
        inputs = []
        for x, v, u in zip(rows, measured, acceleration):
            lower = max(-.15, float(x[prefix + "mapped_path_velocity_lower"]))
            upper = min(.58, float(x[prefix + "mapped_path_velocity_upper"]))
            if index == 2 and x[prefix + "capability_derating_active"] == "True":
                upper = min(upper, float(x[prefix + "boundary_upper"]) - .005)
            if not all(math.isfinite(z) for z in (v, u, lower, upper)):
                raise ValueError("nonfinite replay input; do not bridge invalid evidence")
            inputs.append("{} {} .01 {} {}".format(v, u, lower, upper))
        output = subprocess.run([str(binary), "--replay", str(tau)],
                                input="\n".join(inputs) + "\n", text=True,
                                capture_output=True, check=True).stdout
        candidate = [float(x) for x in output.splitlines()]
        if len(candidate) != len(rows):
            raise ValueError("replay length mismatch")
        legacy = [v + .01 * u for v, u in zip(measured, acceleration)]
        result[prefix.rstrip("_")] = {}
        for name, values in (("persistent_recorded", original),
                             ("anchor_candidate", candidate),
                             ("unfiltered_measured_plus_dt_u", legacy)):
            result[prefix.rstrip("_")][name] = {
                "mean_command_mps": statistics.mean(values),
                "mean_command_minus_measured_mps": statistics.mean(
                    a - b for a, b in zip(values, measured)),
                "command_step_rms_mps": rms(
                    [b - a for a, b in zip(values, values[1:])]),
            }
    return {"samples": len(rows), "tau_seconds": tau, "robots": result}


def simulation(binary):
    output = subprocess.run([str(binary), "--simulate"], text=True,
                            capture_output=True, check=True).stdout
    rows = list(csv.DictReader(io.StringIO(output)))
    original = {(x["gain"], x["lag"]): x for x in rows if x["mode"] == "persistent"}
    candidates = [x for x in rows if x["mode"] == "anchor"]
    worse_error = sum(float(x["low_progress_rms"]) >
                      float(original[(x["gain"], x["lag"])]["low_progress_rms"])
                      for x in candidates)
    worse_peak = sum(float(x["peak_velocity"]) >
                     float(original[(x["gain"], x["lag"])]["peak_velocity"])
                     for x in candidates)
    if any(int(x["invalid"]) or not all(math.isfinite(float(x[k])) for k in
           ("low_progress_rms", "peak_velocity", "command_step_rms")) for x in rows):
        raise ValueError("invalid simulation state")
    return {"model": "uncalibrated_1D_plant_with_production_R1",
            "candidate_cases": len(candidates),
            "cases_with_worse_progress_rms": worse_error,
            "cases_with_higher_peak_velocity": worse_peak,
            "rows": rows}


if __name__ == "__main__":
    binary = pathlib.Path(sys.argv[1]).resolve()
    run = pathlib.Path(sys.argv[2]).resolve()
    report = {"run": run.name, "production_configuration_changed": False,
              "replay_not_a_tracking_error_prediction": True,
              "replay": [replay(binary, run, tau) for tau in (.04, .1, .2)],
              "simulation": simulation(binary)}
    print(json.dumps(report, ensure_ascii=False, indent=2))

#!/usr/bin/env python3

"""Analyze one role-faithful R0.7 clockwise single-car calibration bag."""

import argparse
import json
import math
import os
import statistics

import rosbag


TRACKING_LABEL = "single_car_planar_tracking_v1:progress+9"


def percentile(values, fraction):
    ordered = sorted(values)
    if not ordered:
        return math.nan
    position = fraction * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    ratio = position - lower
    return ordered[lower] * (1.0 - ratio) + ordered[upper] * ratio


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--robot-index", required=True, type=int, choices=(1, 2, 3))
    parser.add_argument("--output", required=True)
    parser.add_argument("--current-negative-scale", required=True, type=float)
    parser.add_argument("--lateral-gain", required=True, type=float)
    parser.add_argument("--heading-gain", required=True, type=float)
    parser.add_argument("--steady-start-progress", type=float, default=0.75)
    parser.add_argument("--steady-end-progress", type=float, default=1.60)
    return parser.parse_args()


def main():
    args = parse_args()
    tracking_topic = f"/agv{args.robot_index}/s_pretest/tracking_state"
    feedback_topic = f"/agv{args.robot_index}/chassis_feedback"
    result_topic = f"/agv{args.robot_index}/s_pretest/result"
    samples = []
    feedback = []
    result = "MISSING"
    with rosbag.Bag(args.bag) as bag:
        for topic, message, stamp in bag.read_messages(
                topics=[tracking_topic, feedback_topic, result_topic]):
            time_value = stamp.to_sec()
            if topic == tracking_topic:
                label = message.layout.dim[0].label if message.layout.dim else ""
                if label != TRACKING_LABEL or len(message.data) != 10:
                    continue
                values = [float(value) for value in message.data]
                if (args.steady_start_progress <= values[0] <=
                        args.steady_end_progress):
                    samples.append((time_value, values))
            elif topic == feedback_topic:
                feedback.append((
                    time_value,
                    float(message.wheel_linear_velocity_left_applied),
                    float(message.wheel_linear_velocity_right_applied),
                    float(message.wheel_linear_velocity_left_actual),
                    float(message.wheel_linear_velocity_right_actual),
                ))
            else:
                result = str(message.data)

    if result != "STOP_CONFIRMED":
        raise RuntimeError(f"calibration run result is {result}, not STOP_CONFIRMED")
    if len(samples) < 300:
        raise RuntimeError(
            f"only {len(samples)} steady-arc samples; at least 300 are required")

    lateral = [row[1][2] for row in samples]
    heading = [row[1][3] for row in samples]
    angular_ff = [row[1][5] for row in samples]
    angular_raw = [row[1][7] for row in samples]
    left_raw = [row[1][8] for row in samples]
    right_raw = [row[1][9] for row in samples]
    unscaled_ff = [value / args.current_negative_scale for value in angular_ff]
    feedback_correction = [raw - ff for raw, ff in zip(angular_raw, angular_ff)]
    median_unscaled_ff = statistics.median(unscaled_ff)
    if abs(median_unscaled_ff) < 0.03:
        raise RuntimeError("steady arc has insufficient angular feedforward")
    unconstrained_scale = (
        args.current_negative_scale +
        statistics.median(feedback_correction) / median_unscaled_ff
    )
    maximum_one_run_change = 0.08
    candidate_scale = min(
        args.current_negative_scale + maximum_one_run_change,
        max(args.current_negative_scale - maximum_one_run_change,
            unconstrained_scale),
    )
    candidate_scale = min(1.25, max(0.75, candidate_scale))

    start_time = samples[0][0]
    end_time = samples[-1][0]
    selected_feedback = [row for row in feedback if start_time <= row[0] <= end_time]
    actual_applied_ratios = []
    actual_peak = math.nan
    if selected_feedback:
        actual_peak = max(
            max(abs(row[3]), abs(row[4])) for row in selected_feedback)
        for row in selected_feedback:
            applied = 0.5 * (abs(row[1]) + abs(row[2]))
            actual = 0.5 * (abs(row[3]) + abs(row[4]))
            if applied > 0.02:
                actual_applied_ratios.append(actual / applied)

    lateral_median = statistics.median(lateral)
    heading_median = statistics.median(heading)
    lateral_rms = rms(lateral)
    heading_rms = rms(heading)
    scale_change = candidate_scale - args.current_negative_scale
    candidate_was_step_limited = (
        abs(unconstrained_scale - args.current_negative_scale) >
        maximum_one_run_change + 1.0e-9)
    tracking_is_small = (
        lateral_rms <= 0.025 and heading_rms <= math.radians(5.0))
    confidence = (
        len(samples) >= 700 and
        percentile([abs(value - lateral_median) for value in lateral], 0.95) < 0.025 and
        percentile([abs(value - heading_median) for value in heading], 0.95) <
        math.radians(6.0) and
        (not math.isfinite(actual_peak) or actual_peak < 0.15)
    )
    if confidence and tracking_is_small:
        # Residual feedback in an already accurate run is not sufficient
        # evidence for changing feedforward. Preserve the parameter that
        # produced the accepted absolute tracking result.
        decision = "CURRENT_PARAMETERS_READY_FOR_THREE_CAR_SHORT_ARC"
        recommended_scale = args.current_negative_scale
    elif confidence:
        decision = "APPLY_CANDIDATE_AND_REPEAT_SINGLE_CAR_ONCE"
        recommended_scale = candidate_scale
    else:
        decision = "REPEAT_SINGLE_CAR_BEFORE_APPLYING"
        recommended_scale = args.current_negative_scale

    report = {
        "schema": "single_car_circle_calibration_v1",
        "robot_index": args.robot_index,
        "bag": os.path.abspath(args.bag),
        "result": result,
        "steady_progress_m": [
            args.steady_start_progress, args.steady_end_progress],
        "steady_samples": len(samples),
        "lateral_error": {
            "median_m": lateral_median,
            "rms_m": lateral_rms,
            "p95_abs_m": percentile([abs(value) for value in lateral], 0.95),
            "maximum_abs_m": max(abs(value) for value in lateral),
        },
        "heading_error": {
            "median_deg": math.degrees(heading_median),
            "rms_deg": math.degrees(heading_rms),
            "p95_abs_deg": math.degrees(
                percentile([abs(value) for value in heading], 0.95)),
            "maximum_abs_deg": math.degrees(max(abs(value) for value in heading)),
        },
        "wheel_demand_peak_mps": max(
            max(abs(value) for value in left_raw),
            max(abs(value) for value in right_raw)),
        "wheel_actual_peak_mps": actual_peak,
        "actual_to_applied_speed_ratio_median": (
            statistics.median(actual_applied_ratios)
            if actual_applied_ratios else math.nan),
        "current": {
            "angular_feedforward_scale_negative": args.current_negative_scale,
            "lateral_gain": args.lateral_gain,
            "heading_gain": args.heading_gain,
        },
        "candidate": {
            "angular_feedforward_scale_negative": candidate_scale,
            "unclamped_estimate": unconstrained_scale,
            "change": scale_change,
            "lateral_gain": args.lateral_gain,
            "heading_gain": args.heading_gain,
        },
        "decision": decision,
        "candidate_was_step_limited": candidate_was_step_limited,
        "tracking_is_small": tracking_is_small,
        "recommended_angular_feedforward_scale_negative": recommended_scale,
        "note": (
            "The candidate is diagnostic only when current absolute tracking "
            "is already small. Gains are intentionally unchanged; formation "
            "gains cannot be identified by a single-car run."
        ),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as stream:
        json.dump(report, stream, indent=2, sort_keys=True)
        stream.write("\n")
    candidate_path = os.path.splitext(args.output)[0] + "_candidate.yaml"
    with open(candidate_path, "w", encoding="utf-8") as stream:
        stream.write(f"agv{args.robot_index}:\n")
        stream.write("  tracker:\n")
        stream.write(
            "    angular_feedforward_scale_negative: "
            f"{recommended_scale:.9f}\n")
        stream.write(f"    lateral_gain: {args.lateral_gain:.9f}\n")
        stream.write(f"    heading_gain: {args.heading_gain:.9f}\n")
    print(json.dumps(report, indent=2, sort_keys=True))
    print(f"candidate_yaml={candidate_path}")


if __name__ == "__main__":
    main()

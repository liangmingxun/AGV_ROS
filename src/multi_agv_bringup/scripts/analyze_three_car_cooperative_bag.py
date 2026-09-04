#!/usr/bin/env python3
"""Offline camera-truth formation metrics for the bounded three-AGV gates."""

import argparse
import bisect
import json
import math
import sys

import rosbag


TOPICS = [f"/pose_provider/agv{i}/base_pose_raw" for i in range(1, 4)]
REFERENCE_TOPIC = "/multi_agv/bounded_pretest/path_reference"
BASE_TO_SUPPORT_X = -0.01783
PATH_AMPLITUDE_S = 0.05
PATH_LONGITUDINAL_LENGTH = 1.0
PATH_CIRCLE_RADIUS = 0.5
PATH_CIRCLE_DIRECTION = -1.0
PATH_CIRCLE_ENTRY_STRAIGHT = 0.20
PATH_CIRCLE_CURVATURE_RAMP = 0.40
PATH_LOOKUP_SAMPLES = 20001
CHASSIS_REFERENCE_SAMPLES = 10001
SUPPORT_OFFSETS = (
    (0.1732050807568877, 0.0),
    (-0.0866025403784439, 0.1500000000000000),
    (-0.0866025403784439, -0.1500000000000000),
)


class ChassisReferenceModel:
    """Python equivalent of SCurvePath/SupportGeometry/PlanarSupportTracker."""

    def __init__(self, mode):
        self.mode = mode
        if mode == "circle":
            self.amplitude = 0.0
            self.wave_number = 0.0
            self.length = (PATH_CIRCLE_ENTRY_STRAIGHT +
                           PATH_CIRCLE_CURVATURE_RAMP +
                           2.0 * math.pi * PATH_CIRCLE_RADIUS)
            self.heading_step = (
                self.length / (CHASSIS_REFERENCE_SAMPLES - 1))
            self.chassis_heading = [
                self._build_chassis_heading(robot_index)
                for robot_index in range(3)
            ]
            return
        self.amplitude = PATH_AMPLITUDE_S if mode == "s" else 0.0
        self.wave_number = 2.0 * math.pi / PATH_LONGITUDINAL_LENGTH
        self.xi = [
            PATH_LONGITUDINAL_LENGTH * index /
            (PATH_LOOKUP_SAMPLES - 1)
            for index in range(PATH_LOOKUP_SAMPLES)
        ]
        self.arc_length = [0.0]
        for index in range(1, PATH_LOOKUP_SAMPLES):
            lower = self.xi[index - 1]
            upper = self.xi[index]
            midpoint = 0.5 * (lower + upper)
            segment = (upper - lower) * (
                self._raw_speed(lower) +
                4.0 * self._raw_speed(midpoint) +
                self._raw_speed(upper)) / 6.0
            self.arc_length.append(self.arc_length[-1] + segment)
        self.length = self.arc_length[-1]
        self.heading_step = (
            self.length / (CHASSIS_REFERENCE_SAMPLES - 1))
        self.chassis_heading = [
            self._build_chassis_heading(robot_index)
            for robot_index in range(3)
        ]

    def _raw_speed(self, xi):
        slope = (self.amplitude * self.wave_number *
                 math.cos(self.wave_number * xi))
        return math.hypot(1.0, slope)

    def _xi_for_arc_length(self, progress):
        bounded = min(max(progress, 0.0), self.length)
        index = bisect.bisect_right(self.arc_length, bounded)
        if index <= 0:
            return 0.0
        if index >= len(self.arc_length):
            return PATH_LONGITUDINAL_LENGTH
        lower = index - 1
        ratio = ((bounded - self.arc_length[lower]) /
                 (self.arc_length[index] - self.arc_length[lower]))
        return self.xi[lower] + ratio * (self.xi[index] - self.xi[lower])

    def _center_sample(self, progress):
        if self.mode == "circle":
            bounded = min(max(progress, 0.0), self.length)
            signed_curvature = (PATH_CIRCLE_DIRECTION /
                                PATH_CIRCLE_RADIUS)
            if bounded <= PATH_CIRCLE_ENTRY_STRAIGHT:
                heading = 0.0
                curvature = 0.0
            elif bounded < (PATH_CIRCLE_ENTRY_STRAIGHT +
                            PATH_CIRCLE_CURVATURE_RAMP):
                t = ((bounded - PATH_CIRCLE_ENTRY_STRAIGHT) /
                     PATH_CIRCLE_CURVATURE_RAMP)
                heading = (signed_curvature *
                           PATH_CIRCLE_CURVATURE_RAMP *
                           (t ** 3 - 0.5 * t ** 4))
                curvature = signed_curvature * (3.0 * t ** 2 -
                                                 2.0 * t ** 3)
            else:
                heading = (0.5 * signed_curvature *
                           PATH_CIRCLE_CURVATURE_RAMP +
                           signed_curvature *
                           (bounded - PATH_CIRCLE_ENTRY_STRAIGHT -
                            PATH_CIRCLE_CURVATURE_RAMP))
                curvature = signed_curvature
            tangent = (math.cos(heading), math.sin(heading))
            normal = (-tangent[1], tangent[0])
            # Reference support positions are read from PathReference. The
            # analyzer needs only tangent/curvature to reconstruct the frozen
            # tag-independent chassis heading.
            return (0.0, 0.0), tangent, normal, curvature
        xi = self._xi_for_arc_length(progress)
        phase = self.wave_number * xi
        slope = (self.amplitude * self.wave_number * math.cos(phase))
        second = (-self.amplitude * self.wave_number ** 2 *
                  math.sin(phase))
        speed = math.hypot(1.0, slope)
        tangent = (1.0 / speed, slope / speed)
        normal = (-tangent[1], tangent[0])
        curvature = second / speed ** 3
        return ((xi, self.amplitude * math.sin(phase)), tangent,
                normal, curvature)

    def _support_sample(self, robot_index, progress):
        position, tangent, normal, curvature = self._center_sample(progress)
        q_tangent, q_normal = SUPPORT_OFFSETS[robot_index]
        tangent_coefficient = 1.0 - curvature * q_normal
        normal_coefficient = curvature * q_tangent
        support_position = (
            position[0] + q_tangent * tangent[0] + q_normal * normal[0],
            position[1] + q_tangent * tangent[1] + q_normal * normal[1])
        first_derivative = (
            tangent_coefficient * tangent[0] + normal_coefficient * normal[0],
            tangent_coefficient * tangent[1] + normal_coefficient * normal[1])
        support_heading = math.atan2(first_derivative[1],
                                     first_derivative[0])
        return support_position, first_derivative, support_heading

    def _heading_derivative(self, robot_index, progress, heading):
        first_derivative = self._support_sample(robot_index, progress)[1]
        return (-math.sin(heading) * first_derivative[0] +
                math.cos(heading) * first_derivative[1]) / BASE_TO_SUPPORT_X

    def _build_chassis_heading(self, robot_index):
        # BASE_TO_SUPPORT_X is negative, matching the C++ tracker: integrate
        # backwards from the final support heading boundary condition.
        heading = [0.0] * CHASSIS_REFERENCE_SAMPLES
        heading[-1] = self._support_sample(robot_index, self.length)[2]
        step = -self.heading_step
        for index in range(CHASSIS_REFERENCE_SAMPLES - 1, 0, -1):
            progress = index * self.heading_step
            value = heading[index]
            k1 = self._heading_derivative(robot_index, progress, value)
            k2 = self._heading_derivative(
                robot_index, progress + 0.5 * step,
                value + 0.5 * step * k1)
            k3 = self._heading_derivative(
                robot_index, progress + 0.5 * step,
                value + 0.5 * step * k2)
            k4 = self._heading_derivative(
                robot_index, progress + step, value + step * k3)
            heading[index - 1] = value + step * (
                k1 + 2.0 * k2 + 2.0 * k3 + k4) / 6.0
        return heading

    def heading(self, robot_index, progress):
        table_position = min(max(progress, 0.0), self.length) / self.heading_step
        left = min(int(table_position), CHASSIS_REFERENCE_SAMPLES - 2)
        ratio = min(max(table_position - left, 0.0), 1.0)
        left_heading = self.chassis_heading[robot_index][left]
        delta = wrap(
            self.chassis_heading[robot_index][left + 1] - left_heading)
        return wrap(left_heading + ratio * delta)


def yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(value):
    return math.atan2(math.sin(value), math.cos(value))


def percentile(values, fraction):
    values = sorted(values)
    if not values:
        return None
    return values[int(round((len(values) - 1) * fraction))]


def nearest(samples, stamp, tolerance=0.05):
    times = [item[0] for item in samples]
    index = bisect.bisect_left(times, stamp)
    choices = []
    if index < len(samples):
        choices.append(samples[index])
    if index:
        choices.append(samples[index - 1])
    if not choices:
        return None
    value = min(choices, key=lambda item: abs(item[0] - stamp))
    return value if abs(value[0] - stamp) <= tolerance else None


def rms(values):
    return math.sqrt(sum(value * value for value in values) / len(values))


def calculate_metrics(samples, pairs, baseline_distance, baseline_heading,
                      reference_baseline):
    distance_errors = []
    heading_errors = []
    per_pair = {}
    for left, right in pairs:
        key = f"agv{left + 1}_agv{right + 1}"
        de = []
        he = []
        for row, reference, _ in samples:
            distance = math.hypot(row[left][1] - row[right][1],
                                  row[left][2] - row[right][2])
            reference_distance = math.hypot(
                reference[left][0] - reference[right][0],
                reference[left][1] - reference[right][1])
            de.append((distance - baseline_distance[key]) -
                      (reference_distance - reference_baseline[key]))
            he.append(wrap(
                (row[left][3] - row[right][3]) -
                (reference[left][2] - reference[right][2]) -
                baseline_heading[key]))
        distance_errors.extend(de)
        heading_errors.extend(he)
        per_pair[key] = {
            "distance_baseline_m": baseline_distance[key],
            "distance_drift_rms_m": rms(de),
            "distance_drift_max_abs_m": max(abs(value) for value in de),
            "relative_heading_rms_deg": math.degrees(rms(he)),
            "relative_heading_max_abs_deg": math.degrees(
                max(abs(value) for value in he)),
        }

    summary = {
        "pairwise_distance_drift_rms_m": rms(distance_errors),
        "pairwise_distance_drift_max_abs_m": max(
            abs(value) for value in distance_errors),
        "relative_heading_rms_deg": math.degrees(rms(heading_errors)),
        "relative_heading_max_abs_deg": math.degrees(
            max(abs(value) for value in heading_errors)),
    }
    return per_pair, summary


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument(
        "--mode", choices=("straight", "s", "circle"), required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--evaluation-start-progress", type=float, default=0.02,
        help=("exclude only the startup interval before this path progress "
              "from the quality gate; full-run metrics remain reported"))
    args = parser.parse_args()
    if args.evaluation_start_progress < 0.0:
        parser.error("--evaluation-start-progress must be non-negative")

    streams = {topic: [] for topic in TOPICS}
    raw_references = []
    with rosbag.Bag(args.bag) as bag:
        for topic, message, receipt in bag.read_messages(
                topics=TOPICS + [REFERENCE_TOPIC]):
            stamp = message.header.stamp.to_sec()
            if stamp <= 0.0:
                stamp = receipt.to_sec()
            if topic == REFERENCE_TOPIC:
                raw_references.append((
                    stamp, message.load_path_progress_reference,
                    [(support.x, support.y)
                     for support in message.support_pose_reference]))
            else:
                streams[topic].append((stamp, message.pose.position.x,
                                       message.pose.position.y,
                                       yaw(message.pose.orientation)))

    reference_model = ChassisReferenceModel(args.mode)
    references = []
    for stamp, progress, supports in raw_references:
        pose = []
        for robot_index, support in enumerate(supports):
            heading = reference_model.heading(robot_index, progress)
            pose.append((
                support[0] - BASE_TO_SUPPORT_X * math.cos(heading),
                support[1] - BASE_TO_SUPPORT_X * math.sin(heading),
                heading))
        references.append((stamp, pose, progress))

    counts = {topic: len(values) for topic, values in streams.items()}
    if min(counts.values()) < 10:
        result = {"status": "INSUFFICIENT_CAMERA_DATA", "counts": counts}
        with open(args.output, "w", encoding="utf-8") as output:
            json.dump(result, output, indent=2, sort_keys=True)
        print(json.dumps(result, indent=2, sort_keys=True))
        return 2

    synchronized = []
    for first in streams[TOPICS[0]]:
        row = [first]
        for topic in TOPICS[1:]:
            sample = nearest(streams[topic], first[0])
            if sample is None:
                break
            row.append(sample)
        if len(row) == 3:
            reference = nearest(references, first[0], tolerance=0.10)
            if reference is not None:
                synchronized.append((row, reference[1], reference[2]))
    if len(synchronized) < 10:
        raise RuntimeError("fewer than 10 synchronized three-camera samples")

    baseline_count = min(30, max(5, len(synchronized) // 10))
    pairs = ((0, 1), (0, 2), (1, 2))
    baseline_distance = {}
    baseline_heading = {}
    reference_baseline = {}
    for left, right in pairs:
        distances = []
        headings = []
        for row, reference, _ in synchronized[:baseline_count]:
            distances.append(math.hypot(row[left][1] - row[right][1],
                                        row[left][2] - row[right][2]))
            headings.append(wrap(
                (row[left][3] - row[right][3]) -
                (reference[left][2] - reference[right][2])))
        key = f"agv{left + 1}_agv{right + 1}"
        baseline_distance[key] = percentile(distances, 0.5)
        baseline_heading[key] = percentile(headings, 0.5)
        reference_baseline[key] = percentile([
            math.hypot(reference[left][0] - reference[right][0],
                       reference[left][1] - reference[right][1])
            for _, reference, _ in synchronized[:baseline_count]], 0.5)

    evaluation_samples = [
        sample for sample in synchronized
        if sample[2] >= args.evaluation_start_progress
    ]
    if len(evaluation_samples) < 10:
        raise RuntimeError(
            "fewer than 10 synchronized samples in the quality evaluation "
            "window")

    full_run_pairs, full_run_summary = calculate_metrics(
        synchronized, pairs, baseline_distance, baseline_heading,
        reference_baseline)
    per_pair, summary = calculate_metrics(
        evaluation_samples, pairs, baseline_distance, baseline_heading,
        reference_baseline)

    distance_limit = 0.020 if args.mode == "straight" else 0.030
    heading_limit_deg = 3.0 if args.mode == "straight" else 5.0

    def gate_status(metrics):
        return ("PASS" if
                metrics["pairwise_distance_drift_max_abs_m"] <=
                distance_limit and
                metrics["relative_heading_max_abs_deg"] <=
                heading_limit_deg else "REVIEW")

    status = gate_status(summary)
    result = {
        "status": status,
        "mode": args.mode,
        "camera_role": "same_sensor_raw_observation_for_fused_closed_loop",
        "reference_heading_model":
            "tracker_chassis_reference_with_base_to_support",
        "synchronized_samples": len(synchronized),
        "evaluation_samples": len(evaluation_samples),
        "evaluation_window": {
            "basis": "load_path_progress_reference",
            "start_progress_m": args.evaluation_start_progress,
            "purpose": "quality_gate_excluding_startup_transient",
        },
        "camera_counts": counts,
        "thresholds": {
            "pairwise_distance_drift_max_abs_m": distance_limit,
            "relative_heading_max_abs_deg": heading_limit_deg,
        },
        "full_run_gate_status": gate_status(full_run_summary),
        "full_run_summary": full_run_summary,
        "full_run_pairs": full_run_pairs,
        "quality_gate_scope": "evaluation_window",
        "summary": summary,
        "pairs": per_pair,
        "interpretation": {
            "full_run_metrics": "safety_and_startup_review",
            "evaluation_window_metrics": "formal_formation_quality_gate",
        },
    }
    with open(args.output, "w", encoding="utf-8") as output:
        json.dump(result, output, indent=2, sort_keys=True)
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if status == "PASS" else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:  # keep failure machine-readable in the runner
        print(f"ERROR: {error}", file=sys.stderr)
        sys.exit(2)

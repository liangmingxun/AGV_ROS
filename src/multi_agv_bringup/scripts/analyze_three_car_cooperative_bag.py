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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    parser.add_argument("--mode", choices=("straight", "s"), required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    streams = {topic: [] for topic in TOPICS}
    references = []
    with rosbag.Bag(args.bag) as bag:
        for topic, message, receipt in bag.read_messages(
                topics=TOPICS + [REFERENCE_TOPIC]):
            stamp = message.header.stamp.to_sec()
            if stamp <= 0.0:
                stamp = receipt.to_sec()
            if topic == REFERENCE_TOPIC:
                pose = []
                for support in message.support_pose_reference:
                    # Convert the planned support centre to base_link using
                    # the same frozen longitudinal transform as the tracker.
                    pose.append((
                        support.x - BASE_TO_SUPPORT_X * math.cos(support.theta),
                        support.y - BASE_TO_SUPPORT_X * math.sin(support.theta),
                        support.theta))
                references.append((stamp, pose))
            else:
                streams[topic].append((stamp, message.pose.position.x,
                                       message.pose.position.y,
                                       yaw(message.pose.orientation)))

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
                synchronized.append((row, reference[1]))
    if len(synchronized) < 10:
        raise RuntimeError("fewer than 10 synchronized three-camera samples")

    baseline_count = min(30, max(5, len(synchronized) // 10))
    pairs = ((0, 1), (0, 2), (1, 2))
    baseline_distance = {}
    baseline_heading = {}
    for left, right in pairs:
        distances = []
        headings = []
        for row, reference in synchronized[:baseline_count]:
            distances.append(math.hypot(row[left][1] - row[right][1],
                                        row[left][2] - row[right][2]))
            headings.append(wrap(
                (row[left][3] - row[right][3]) -
                (reference[left][2] - reference[right][2])))
        key = f"agv{left + 1}_agv{right + 1}"
        baseline_distance[key] = percentile(distances, 0.5)
        baseline_heading[key] = percentile(headings, 0.5)

    distance_errors = []
    heading_errors = []
    per_pair = {}
    for left, right in pairs:
        key = f"agv{left + 1}_agv{right + 1}"
        de = []
        he = []
        reference_baseline = percentile([
            math.hypot(reference[left][0] - reference[right][0],
                       reference[left][1] - reference[right][1])
            for _, reference in synchronized[:baseline_count]], 0.5)
        for row, reference in synchronized:
            distance = math.hypot(row[left][1] - row[right][1],
                                  row[left][2] - row[right][2])
            reference_distance = math.hypot(
                reference[left][0] - reference[right][0],
                reference[left][1] - reference[right][1])
            de.append((distance - baseline_distance[key]) -
                      (reference_distance - reference_baseline))
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

    distance_limit = 0.020 if args.mode == "straight" else 0.030
    heading_limit_deg = 3.0 if args.mode == "straight" else 5.0
    distance_max = max(abs(value) for value in distance_errors)
    heading_max_deg = math.degrees(max(abs(value) for value in heading_errors))
    status = ("PASS" if distance_max <= distance_limit and
              heading_max_deg <= heading_limit_deg else "REVIEW")
    result = {
        "status": status,
        "mode": args.mode,
        "camera_role": "same_sensor_raw_observation_for_fused_closed_loop",
        "synchronized_samples": len(synchronized),
        "camera_counts": counts,
        "thresholds": {
            "pairwise_distance_drift_max_abs_m": distance_limit,
            "relative_heading_max_abs_deg": heading_limit_deg,
        },
        "summary": {
            "pairwise_distance_drift_rms_m": rms(distance_errors),
            "pairwise_distance_drift_max_abs_m": distance_max,
            "relative_heading_rms_deg": math.degrees(rms(heading_errors)),
            "relative_heading_max_abs_deg": heading_max_deg,
        },
        "pairs": per_pair,
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

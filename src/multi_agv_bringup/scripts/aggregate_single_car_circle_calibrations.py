#!/usr/bin/env python3

"""Combine three accepted single-car circle reports into one review YAML."""

import argparse
import json
import os


def parse_args():
    parser = argparse.ArgumentParser()
    for robot in range(1, 4):
        parser.add_argument(f"--robot{robot}", required=True)
    parser.add_argument("--output", required=True)
    return parser.parse_args()


def main():
    args = parse_args()
    reports = []
    for robot in range(1, 4):
        path = getattr(args, f"robot{robot}")
        with open(path, encoding="utf-8") as stream:
            report = json.load(stream)
        if report.get("robot_index") != robot:
            raise RuntimeError(f"{path} is not a Robot{robot} report")
        decision = report.get("decision")
        if decision not in (
                "CURRENT_PARAMETERS_READY_FOR_THREE_CAR_SHORT_ARC",
                "CANDIDATE_READY_FOR_THREE_CAR_SHORT_ARC"):
            raise RuntimeError(
                f"Robot{robot} report requires repetition before aggregation")
        reports.append(report)
    scales = [
        (report["current"]["angular_feedforward_scale_negative"]
         if report.get("decision") ==
         "CURRENT_PARAMETERS_READY_FOR_THREE_CAR_SHORT_ARC"
         else report["candidate"]["angular_feedforward_scale_negative"])
        for report in reports]
    lateral = [report["candidate"]["lateral_gain"] for report in reports]
    heading = [report["candidate"]["heading_gain"] for report in reports]
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, "w", encoding="utf-8") as stream:
        stream.write("# Review-only candidate from three role-faithful 90-degree runs.\n")
        stream.write("tracker:\n")
        stream.write(
            "  angular_feedforward_scale_negative: [" +
            ", ".join(f"{value:.9f}" for value in scales) + "]\n")
        stream.write(
            "  lateral_gain_per_robot: [" +
            ", ".join(f"{value:.9f}" for value in lateral) + "]\n")
        stream.write(
            "  heading_gain_per_robot: [" +
            ", ".join(f"{value:.9f}" for value in heading) + "]\n")
        stream.write("sources:\n")
        for report in reports:
            stream.write(f"  - {report['bag']}\n")
    print(f"candidate={args.output}")
    print("Next gate: one three-car short-arc run; do not skip directly to a full lap.")


if __name__ == "__main__":
    main()

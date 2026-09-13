#!/usr/bin/env python3

import argparse
import json
import sys

from multi_agv_analysis.conversion import convert_bag
from multi_agv_analysis.io_utils import load_yaml


def main():
    parser = argparse.ArgumentParser(
        description="Export frozen ROS topics and causal aligned samples.")
    parser.add_argument("bag")
    parser.add_argument("output_dir")
    parser.add_argument("--maximum-alignment-age", type=float, default=0.2)
    parser.add_argument("--parquet", action="store_true")
    parser.add_argument(
        "--measured-payload-pose-topic",
        default="/pose_provider/load/pose_filtered")
    parser.add_argument("--measured-payload-transform",
                        help="Recorded SE(2) YAML: source_frame, target_frame, x, y, yaw")
    arguments = parser.parse_args()
    try:
        result = convert_bag(
            arguments.bag, arguments.output_dir,
            maximum_alignment_age=arguments.maximum_alignment_age,
            parquet=arguments.parquet,
            measured_payload_pose_topic=
                arguments.measured_payload_pose_topic,
            measured_payload_transform=(load_yaml(arguments.measured_payload_transform)
                if arguments.measured_payload_transform else None))
    except Exception as error:  # CLI boundary: preserve a useful nonzero exit.
        print("ERROR: {}".format(error), file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())

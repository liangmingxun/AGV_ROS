#!/usr/bin/env python3

import argparse
import json
import sys

from multi_agv_analysis.conversion import convert_bag


def main():
    parser = argparse.ArgumentParser(
        description="Export frozen ROS topics and causal aligned samples.")
    parser.add_argument("bag")
    parser.add_argument("output_dir")
    parser.add_argument("--maximum-alignment-age", type=float, default=0.2)
    parser.add_argument("--parquet", action="store_true")
    arguments = parser.parse_args()
    try:
        result = convert_bag(
            arguments.bag, arguments.output_dir,
            maximum_alignment_age=arguments.maximum_alignment_age,
            parquet=arguments.parquet)
    except Exception as error:  # CLI boundary: preserve a useful nonzero exit.
        print("ERROR: {}".format(error), file=sys.stderr)
        return 2
    print(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    sys.exit(main())

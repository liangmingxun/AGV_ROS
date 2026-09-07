#!/usr/bin/env python3

"""Generate reproducible draft figures 2--7 from one aligned run."""

import argparse
import sys

from multi_agv_analysis.io_utils import load_yaml
from multi_agv_analysis.paper_pipeline import plot_run


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("aligned_csv")
    parser.add_argument("output_dir")
    parser.add_argument(
        "--axis-config",
        help="optional YAML mapping of axis key to y_min/y_max/y_tick_step")
    args = parser.parse_args()
    try:
        overrides = load_yaml(args.axis_config) if args.axis_config else None
        count = plot_run(args.aligned_csv, args.output_dir, overrides)
        print("PLOTS WRITTEN: {} figure(s)".format(count))
        return 0
    except Exception as error:
        print("PLOT GENERATION FAILED: {}".format(error), file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())

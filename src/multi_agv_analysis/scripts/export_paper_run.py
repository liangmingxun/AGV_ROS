#!/usr/bin/env python3

"""Create paper-logical CSV views from causal aligned samples."""

import argparse
import sys
from pathlib import Path

from multi_agv_analysis.paper_pipeline import export_views


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("converted_dir")
    parser.add_argument("output_dir")
    args = parser.parse_args()
    try:
        for filename in export_views(args.converted_dir, args.output_dir):
            print("WROTE {}".format(Path(args.output_dir) / filename))
        return 0
    except Exception as error:
        print("PAPER CSV EXPORT FAILED: {}".format(error), file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())

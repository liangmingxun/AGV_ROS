#!/usr/bin/env python3

import argparse
import sys

from multi_agv_analysis.camera_quality import summarize_camera_quality
from multi_agv_analysis.io_utils import atomic_dump_json


def main():
    parser = argparse.ArgumentParser(
        description="Summarize Task-15 camera timing and confidence evidence.")
    parser.add_argument("converted_dir")
    parser.add_argument("report")
    args = parser.parse_args()
    try:
        report = summarize_camera_quality(args.converted_dir)
        atomic_dump_json(args.report, report)
        print("CAMERA QUALITY REPORT WRITTEN: {}".format(args.report))
        return 0
    except Exception as error:
        print("CAMERA QUALITY REPORT REFUSED: {}".format(
            error), file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())

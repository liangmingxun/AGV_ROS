#!/usr/bin/env python3

import argparse
import sys

from multi_agv_analysis.io_utils import atomic_dump_json, load_yaml
from multi_agv_analysis.validation import validate_converted_run


def main():
    parser = argparse.ArgumentParser(
        description="Validate a converted run before computing metrics.")
    parser.add_argument("converted_dir")
    parser.add_argument("manifest")
    parser.add_argument("rules")
    parser.add_argument("output")
    arguments = parser.parse_args()
    report = validate_converted_run(
        arguments.converted_dir, arguments.manifest,
        load_yaml(arguments.rules).get("experiment_recording", {}))
    atomic_dump_json(arguments.output, report)
    if report["valid"]:
        print("RUN VALIDATION: PASSED")
        for warning in report.get("warnings", []):
            print("  WARNING {}: {}".format(
                warning["code"], warning["detail"]))
        return 0
    print("RUN VALIDATION: FAILED ({} issue(s))".format(
        len(report["issues"])), file=sys.stderr)
    for issue in report["issues"]:
        print("  {}: {}".format(issue["code"], issue["detail"]),
              file=sys.stderr)
    return 4


if __name__ == "__main__":
    sys.exit(main())

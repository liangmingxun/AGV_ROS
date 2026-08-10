#!/usr/bin/env python3

import argparse
import sys

from multi_agv_analysis.approval import verify_approved_configuration


def main():
    parser = argparse.ArgumentParser(
        description="Verify an exact Task 17/18 configuration approval.")
    parser.add_argument("registry")
    parser.add_argument("approval_id")
    parser.add_argument("configs", nargs="+")
    parser.add_argument("--formal-statistics", action="store_true")
    args = parser.parse_args()
    try:
        result = verify_approved_configuration(
            args.registry, args.approval_id, args.configs,
            args.formal_statistics)
    except Exception as error:
        print("CONFIG APPROVAL: REFUSED: {}".format(error), file=sys.stderr)
        return 4
    print("CONFIG APPROVAL: PASSED ({})".format(result["status"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())

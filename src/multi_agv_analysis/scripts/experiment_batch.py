#!/usr/bin/env python3

import argparse
import sys

from multi_agv_analysis.experiment_design import (
    create_paired_plan,
    next_pending,
    render_run_instructions,
    update_run,
)
from multi_agv_analysis.io_utils import atomic_dump_yaml, load_yaml


def main():
    parser = argparse.ArgumentParser(
        description="Plan and audit paired experiments; never starts motion.")
    sub = parser.add_subparsers(dest="command", required=True)
    create = sub.add_parser("create")
    create.add_argument("spec")
    create.add_argument("plan")
    show = sub.add_parser("next")
    show.add_argument("plan")
    show.add_argument("--approval-id", default="")
    mark = sub.add_parser("mark")
    mark.add_argument("plan")
    mark.add_argument("run_id")
    mark.add_argument("status", choices=["running", "complete", "invalid"])
    mark.add_argument("--manifest", default="")
    mark.add_argument("--metrics", default="")
    mark.add_argument("--notes", default="")
    args = parser.parse_args()
    try:
        if args.command == "create":
            atomic_dump_yaml(args.plan, create_paired_plan(
                load_yaml(args.spec)))
            print("BATCH PLAN CREATED: {}".format(args.plan))
            return 0
        plan = load_yaml(args.plan)
        if args.command == "next":
            run = next_pending(plan)
            if run is None:
                print("BATCH COMPLETE: no pending runs")
                return 0
            print("NEXT RUN: {} {} {}".format(
                run["run_id"], run["block_id"], run["method_id"]))
            for command in render_run_instructions(run, args.approval_id):
                print(command)
            return 0
        updated = update_run(
            plan, args.run_id, args.status, args.manifest,
            args.metrics, args.notes)
        atomic_dump_yaml(args.plan, plan)
        print("RUN UPDATED: {} -> {}".format(
            updated["run_id"], updated["status"]))
        return 0
    except Exception as error:
        print("BATCH REFUSED: {}".format(error), file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())

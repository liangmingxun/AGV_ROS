#!/usr/bin/env python3

import argparse
import sys

from multi_agv_analysis.evidence import (
    STAGES,
    approval_candidate,
    attach_evidence,
    audit_registry,
    create_registry,
    mark_passed,
    sign_stage,
)
from multi_agv_analysis.io_utils import (
    atomic_dump_json,
    atomic_dump_yaml,
    load_yaml,
)


def main():
    parser = argparse.ArgumentParser(
        description="Manage ordered Stage A-D evidence without authorization.")
    sub = parser.add_subparsers(dest="command", required=True)
    create = sub.add_parser("init")
    create.add_argument("registry")
    create.add_argument("project_id")
    attach = sub.add_parser("attach")
    attach.add_argument("registry")
    attach.add_argument("stage", choices=STAGES)
    attach.add_argument("path")
    attach.add_argument("--label", required=True)
    sign = sub.add_parser("sign")
    sign.add_argument("registry")
    sign.add_argument("stage", choices=STAGES)
    sign.add_argument("role", choices=["operator", "reviewer"])
    sign.add_argument("signer")
    sign.add_argument("--notes", default="")
    passed = sub.add_parser("pass")
    passed.add_argument("registry")
    passed.add_argument("stage", choices=STAGES)
    audit = sub.add_parser("audit")
    audit.add_argument("registry")
    audit.add_argument("report")
    candidate = sub.add_parser("candidate")
    candidate.add_argument("registry")
    candidate.add_argument("approval_id")
    candidate.add_argument("output")
    candidate.add_argument("configs", nargs="+")
    args = parser.parse_args()
    try:
        if args.command == "init":
            atomic_dump_yaml(args.registry, create_registry(args.project_id))
            print("EVIDENCE REGISTRY CREATED: {}".format(args.registry))
            return 0
        registry = load_yaml(args.registry)
        if args.command == "attach":
            attach_evidence(
                registry, args.stage, args.path, args.label)
            atomic_dump_yaml(args.registry, registry)
        elif args.command == "sign":
            sign_stage(
                registry, args.stage, args.role, args.signer, args.notes)
            atomic_dump_yaml(args.registry, registry)
        elif args.command == "pass":
            mark_passed(registry, args.stage)
            atomic_dump_yaml(args.registry, registry)
        elif args.command == "audit":
            report = audit_registry(registry)
            atomic_dump_json(args.report, report)
            state = (
                "COMPLETE" if report["complete"] else
                "INCOMPLETE" if report["valid"] else "INVALID")
            print("EVIDENCE AUDIT: {}".format(state))
            return 0 if report["complete"] else 4
        else:
            atomic_dump_yaml(
                args.output,
                approval_candidate(
                    registry, args.approval_id, args.configs))
            print("APPROVAL CANDIDATE WRITTEN: {}".format(args.output))
            return 0
        print("EVIDENCE UPDATED: {} {}".format(
            args.command, getattr(args, "stage", "")))
        return 0
    except Exception as error:
        print("EVIDENCE REFUSED: {}".format(error), file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())

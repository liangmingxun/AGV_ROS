#!/usr/bin/env python3

import argparse
import os
import socket
import subprocess
import sys

from multi_agv_analysis.health import audit_ros_graph, validate_inventory
from multi_agv_analysis.io_utils import atomic_dump_json, load_yaml


def _git(workspace):
    try:
        sha = subprocess.check_output(
            ["git", "-C", workspace, "rev-parse", "HEAD"],
            text=True).strip()
        status = subprocess.check_output(
            ["git", "-C", workspace, "status", "--porcelain"],
            text=True).splitlines()
        return {"available": True, "sha": sha, "dirty": bool(status)}
    except (OSError, subprocess.CalledProcessError) as error:
        return {"available": False, "error": str(error)}


def _master_reachable(ip, port, timeout):
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True
    except OSError:
        return False


def main():
    parser = argparse.ArgumentParser(
        description="Read-only three-host inventory and ROS graph audit.")
    parser.add_argument("inventory")
    parser.add_argument("report")
    parser.add_argument("--offline", action="store_true")
    parser.add_argument("--expect-commands", action="store_true")
    parser.add_argument("--workspace", default=os.getcwd())
    parser.add_argument("--timeout", type=float, default=1.0)
    args = parser.parse_args()
    try:
        inventory = load_yaml(args.inventory)
        report = {
            "schema_version": 1,
            "inventory": validate_inventory(inventory),
            "online_checked": not args.offline,
            "local_git": _git(args.workspace),
            "claim": (
                "Read-only software health evidence; not an emergency-stop "
                "or hardware-watchdog safety proof."),
        }
        issues = list(report["inventory"]["issues"])
        if not args.offline and not issues:
            import rosgraph
            master_ip = inventory["hosts"]["robot1"]["ip"]
            port = int(inventory.get("ros_master_port", 11311))
            report["ros_master_reachable"] = _master_reachable(
                master_ip, port, args.timeout)
            if not report["ros_master_reachable"]:
                issues.append("ROS master TCP endpoint is unreachable")
            master = rosgraph.Master("/fleet_health_check")
            publishers, subscribers, _ = master.getSystemState()
            report["ros_graph"] = audit_ros_graph(
                publishers, subscribers, args.expect_commands)
            issues.extend(report["ros_graph"]["issues"])
        report["issues"] = issues
        report["valid"] = not issues
        atomic_dump_json(args.report, report)
        print("FLEET HEALTH: {}".format(
            "PASSED" if report["valid"] else "FAILED"))
        return 0 if report["valid"] else 4
    except Exception as error:
        print("FLEET HEALTH REFUSED: {}".format(error), file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())

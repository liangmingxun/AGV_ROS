"""Deterministic paired-run planning without executing any controller."""

import datetime
import hashlib
import json
import random
import re


SAFE_ID = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")
TERMINAL = {"complete", "invalid"}


def _identifier(value, label):
    value = str(value)
    if not SAFE_ID.fullmatch(value):
        raise ValueError("{} is not a safe identifier".format(label))
    return value


def create_paired_plan(spec):
    experiment_id = _identifier(spec.get("experiment_id", ""), "experiment_id")
    methods = [_identifier(value, "method") for value in spec.get("methods", [])]
    if len(methods) < 2 or len(set(methods)) != len(methods):
        raise ValueError("paired plan requires at least two unique methods")
    repetitions = int(spec.get("paired_repetitions", 0))
    if repetitions <= 0:
        raise ValueError("paired_repetitions must be positive")
    seed = int(spec.get("random_seed", 0))
    rng = random.Random(seed)
    runs = []
    sequence = 0
    for repetition in range(1, repetitions + 1):
        order = list(methods)
        rng.shuffle(order)
        block_id = "pair_{:02d}".format(repetition)
        for order_index, method in enumerate(order, start=1):
            sequence += 1
            runs.append({
                "sequence": sequence,
                "block_id": block_id,
                "pair_index": repetition,
                "order_in_block": order_index,
                "run_id": "{}_{}_{}".format(
                    experiment_id, block_id, method),
                "method_id": method,
                "status": "pending",
                "manifest": "",
                "metrics": "",
                "notes": "",
            })
    plan = {
        "schema_version": 1,
        "experiment_id": experiment_id,
        "design": "complete_paired_randomized_blocks",
        "methods": methods,
        "paired_repetitions": repetitions,
        "random_seed": seed,
        "created_at": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "formal_statistics_requested": bool(
            spec.get("formal_statistics_requested", False)),
        "runs": runs,
    }
    canonical = json.dumps(
        plan, sort_keys=True, separators=(",", ":")).encode("utf-8")
    plan["plan_sha256"] = hashlib.sha256(canonical).hexdigest()
    return plan


def next_pending(plan):
    active = [run for run in plan["runs"] if run["status"] == "running"]
    if active:
        raise ValueError(
            "one run is already active: {}".format(active[0]["run_id"]))
    return next(
        (run for run in plan["runs"] if run["status"] == "pending"), None)


def update_run(plan, run_id, status, manifest="", metrics="", notes=""):
    if status not in {"running", "complete", "invalid"}:
        raise ValueError("status must be running, complete or invalid")
    run_id = _identifier(run_id, "run_id")
    matching = [run for run in plan["runs"] if run["run_id"] == run_id]
    if len(matching) != 1:
        raise ValueError("run_id is not unique in plan")
    run = matching[0]
    allowed = {
        "pending": {"running", "invalid"},
        "running": TERMINAL,
        "complete": set(),
        "invalid": set(),
    }
    if status not in allowed[run["status"]]:
        raise ValueError(
            "illegal run transition {} -> {}".format(run["status"], status))
    if status == "complete" and (not manifest or not metrics):
        raise ValueError("completed run requires manifest and metrics paths")
    run["status"] = status
    run["manifest"] = str(manifest)
    run["metrics"] = str(metrics)
    run["notes"] = str(notes)
    return run


def render_run_instructions(run, approval_id=""):
    """Return commands for human review; never execute them."""
    values = {
        "run_id": _identifier(run["run_id"], "run_id"),
        "method_id": _identifier(run["method_id"], "method_id"),
        "block_id": _identifier(run["block_id"], "block_id"),
    }
    approval = (
        " approval_id:={}".format(_identifier(approval_id, "approval_id"))
        if approval_id else "")
    return [
        (
            "roslaunch multi_agv_bringup formal_evaluation_window.launch "
            "run_id:={run_id} method_id:={method_id}"
        ).format(**values),
        (
            "roslaunch multi_agv_bringup experiment.launch "
            "run_id:={run_id} method_id:={method_id} "
            "require_approved_config:=true "
            "formal_statistics_requested:=false{approval}"
        ).format(approval=approval, **values),
    ]

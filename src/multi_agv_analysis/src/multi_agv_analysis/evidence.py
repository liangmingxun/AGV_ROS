"""Ordered physical-stage evidence registry; never grants authorization."""

import datetime
from pathlib import Path

from .io_utils import sha256_file


STAGES = ("stage_a", "stage_b", "stage_c", "stage_d", "formal_gate")


def create_registry(project_id):
    if not str(project_id).strip():
        raise ValueError("project_id is required")
    return {
        "schema_version": 1,
        "project_id": str(project_id),
        "created_at": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "warning": (
            "Evidence registry only. Passing entries do not modify ROS "
            "hardware or formal-statistics authorization."),
        "stages": {
            stage: {
                "status": "pending",
                "attachments": [],
                "signoffs": {},
                "notes": "",
            } for stage in STAGES
        },
    }


def _stage(registry, stage):
    if stage not in STAGES:
        raise ValueError("unknown stage: {}".format(stage))
    value = registry.get("stages", {}).get(stage)
    if not isinstance(value, dict):
        raise ValueError("registry has no stage: {}".format(stage))
    return value


def attach_evidence(registry, stage, path, label):
    value = _stage(registry, stage)
    path = Path(path).resolve()
    if not path.is_file():
        raise ValueError("evidence file does not exist: {}".format(path))
    entry = {
        "label": str(label),
        "path": str(path),
        "sha256": sha256_file(path),
    }
    if any(item["path"] == entry["path"] for item in value["attachments"]):
        raise ValueError("evidence path is already attached")
    value["attachments"].append(entry)
    value["status"] = "collecting"
    return entry


def sign_stage(registry, stage, role, signer, notes=""):
    value = _stage(registry, stage)
    if role not in ("operator", "reviewer"):
        raise ValueError("role must be operator or reviewer")
    if not str(signer).strip():
        raise ValueError("signer is required")
    value["signoffs"][role] = {
        "signer": str(signer),
        "signed_at": datetime.datetime.now(
            datetime.timezone.utc).isoformat(),
        "notes": str(notes),
    }
    return value["signoffs"][role]


def audit_registry(registry):
    issues = []
    previous_passed = True
    report = {}
    for stage in STAGES:
        value = _stage(registry, stage)
        attachment_issues = []
        for entry in value.get("attachments", []):
            path = Path(entry.get("path", ""))
            if not path.is_file():
                attachment_issues.append("missing:{}".format(path))
            elif sha256_file(path) != entry.get("sha256"):
                attachment_issues.append("hash_mismatch:{}".format(path))
        roles = value.get("signoffs", {})
        stage_passed = (
            previous_passed and bool(value.get("attachments")) and
            not attachment_issues and
            all(role in roles for role in ("operator", "reviewer")))
        if value.get("status") == "passed" and not stage_passed:
            issues.append(
                "{} is marked passed without complete evidence".format(stage))
        report[stage] = {
            "eligible_to_mark_passed": stage_passed,
            "attachment_count": len(value.get("attachments", [])),
            "attachment_issues": attachment_issues,
            "operator_signed": "operator" in roles,
            "reviewer_signed": "reviewer" in roles,
            "predecessors_passed": previous_passed,
        }
        previous_passed = value.get("status") == "passed" and stage_passed
    complete = (
        not issues and
        all(_stage(registry, stage).get("status") == "passed"
            for stage in STAGES) and
        all(value["eligible_to_mark_passed"] for value in report.values()))
    return {
        "valid": not issues,
        "complete": complete,
        "candidate_eligible": complete,
        "issues": issues,
        "stages": report,
    }


def mark_passed(registry, stage):
    report = audit_registry(registry)
    stage_report = report["stages"][stage]
    if not stage_report["eligible_to_mark_passed"]:
        raise ValueError(
            "stage is not eligible: {}".format(stage_report))
    _stage(registry, stage)["status"] = "passed"


def approval_candidate(registry, approval_id, config_files):
    audit = audit_registry(registry)
    if not audit["valid"]:
        raise ValueError("evidence registry is invalid")
    if any(_stage(registry, stage)["status"] != "passed" for stage in STAGES):
        raise ValueError("all stages must be passed before a candidate")
    hashes = {}
    for value in config_files:
        path = Path(value).resolve()
        if not path.is_file():
            raise ValueError("configuration file missing: {}".format(path))
        if path.name in hashes:
            raise ValueError("duplicate configuration basename")
        hashes[path.name] = sha256_file(path)
    return {
        "schema_version": 1,
        "candidate_approval_id": str(approval_id),
        "status": "candidate_requires_manual_registry_review",
        "formal_statistics_authorized": False,
        "hardware_execution_authorized": False,
        "evidence_project_id": registry.get("project_id", ""),
        "config_sha256": hashes,
        "warning": (
            "This candidate cannot authorize motion or formal statistics. "
            "A reviewer must create a separate approved registry entry."),
    }

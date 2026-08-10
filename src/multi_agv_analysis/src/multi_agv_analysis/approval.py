"""Immutable configuration approval checks for Task 17/18 archives."""

from pathlib import Path

from .io_utils import load_yaml, sha256_file


def verify_approved_configuration(
        registry_path, approval_id, config_files,
        formal_statistics_requested=False):
    registry_path = Path(registry_path).resolve()
    registry = load_yaml(registry_path)
    approvals = registry.get("approvals", {})
    entry = approvals.get(approval_id)
    if not isinstance(entry, dict):
        raise RuntimeError(
            "approval_id is absent from registry: {}".format(approval_id))
    if entry.get("status") not in ("software_rehearsal_only", "approved"):
        raise RuntimeError(
            "configuration approval is not active: {}".format(
                entry.get("status")))
    if (formal_statistics_requested and
            not entry.get("formal_statistics_authorized", False)):
        raise RuntimeError(
            "formal statistics are refused by the selected approval")

    expected = entry.get("config_sha256", {})
    if not isinstance(expected, dict) or not expected:
        raise RuntimeError("approval contains no configuration hashes")
    observed = {}
    for value in config_files:
        path = Path(value).resolve()
        if not path.is_file():
            raise RuntimeError(
                "configuration file does not exist: {}".format(path))
        name = path.name
        digest = sha256_file(path)
        if name in observed and observed[name] != digest:
            raise RuntimeError(
                "configuration basename is ambiguous: {}".format(name))
        observed[name] = digest
    missing = sorted(set(expected) - set(observed))
    extra = sorted(set(observed) - set(expected))
    mismatch = sorted(
        name for name, digest in expected.items()
        if name in observed and observed[name] != str(digest))
    if missing or extra or mismatch:
        raise RuntimeError(
            "configuration approval mismatch: missing={} extra={} "
            "mismatch={}".format(missing, extra, mismatch))
    return {
        "approval_id": approval_id,
        "registry_path": str(registry_path),
        "registry_sha256": sha256_file(registry_path),
        "status": entry["status"],
        "formal_statistics_authorized": bool(
            entry.get("formal_statistics_authorized", False)),
        "notes": entry.get("notes", ""),
    }

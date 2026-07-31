# -*- coding: utf-8 -*-
"""Create a reproducibility manifest without changing any authorization flag."""
from __future__ import annotations

import argparse
import hashlib
import json
import platform
import subprocess
import time
from pathlib import Path

import yaml

from protocol import PROTOCOL_NAME, PROTOCOL_VERSION


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_sha(root: Path) -> str:
    try:
        return subprocess.check_output(
            ["git", "-C", str(root), "rev-parse", "HEAD"],
            stderr=subprocess.DEVNULL,
            text=True,
        ).strip()
    except Exception:
        return "unavailable"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--output", default="")
    parser.add_argument("--test-stage", default="unspecified")
    parser.add_argument("--operator", default="")
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    root = config_path.parent.parent
    config_snapshot = yaml.safe_load(config_path.read_text(encoding="utf-8"))
    sender_dir = Path(__file__).resolve().parent
    source_names = (
        "aruco_udp_sender.py",
        "camera_sources.py",
        "create_manifest.py",
        "ground_reference.py",
        "protocol.py",
        "requirements.txt",
    )
    source_paths = [sender_dir / name for name in source_names]
    source_paths.extend(sorted((sender_dir / "MvImport").glob("*.py")))
    source_sha256 = {
        path.relative_to(sender_dir).as_posix(): sha256(path)
        for path in source_paths
    }
    output = (
        Path(args.output).resolve()
        if args.output
        else config_path.parent / time.strftime("task15_manifest_%Y%m%d_%H%M%S.json")
    )
    manifest = {
        "created_time_unix_ns": time.time_ns(),
        "created_time_local": time.strftime("%Y-%m-%d %H:%M:%S %z"),
        "protocol": PROTOCOL_NAME,
        "protocol_version": PROTOCOL_VERSION,
        "test_stage": args.test_stage,
        "operator": args.operator,
        "config_path": str(config_path),
        "config_sha256": sha256(config_path),
        "config_snapshot": config_snapshot,
        "source_sha256": source_sha256,
        "git_sha": git_sha(root),
        "host": platform.node(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "authorization_statement": "manifest creation does not authorize calibration, extrinsics, or hardware motion",
    }
    output.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

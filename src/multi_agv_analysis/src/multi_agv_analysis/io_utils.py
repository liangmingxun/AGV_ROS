"""Small dependency-free helpers shared by the command-line tools."""

import csv
import datetime
import hashlib
import json
import math
import os
import tempfile
from pathlib import Path

import yaml


def stamp_to_sec(stamp):
    if stamp is None:
        return math.nan
    if hasattr(stamp, "to_sec"):
        return float(stamp.to_sec())
    return float(stamp)


def finite_float(value, default=math.nan):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def bool_value(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return bool(value)
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def read_csv(path):
    with open(path, newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def write_csv(path, rows, fieldnames=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = list(rows)
    if fieldnames is None:
        fieldnames = []
        seen = set()
        for row in rows:
            for key in row:
                if key not in seen:
                    fieldnames.append(key)
                    seen.add(key)
    with open(path, "w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


class _SnapshotSafeLoader(yaml.SafeLoader):
    """Allow only the inert DateTime representation produced by rosparam dump."""


def _xmlrpc_datetime(loader, node):
    value = loader.construct_mapping(node, deep=True)
    if set(value) != {"value"} or not isinstance(value["value"], str):
        raise yaml.constructor.ConstructorError(
            None, None, "invalid XML-RPC DateTime snapshot", node.start_mark)
    try:
        datetime.datetime.strptime(value["value"], "%Y%m%dT%H:%M:%S")
    except ValueError as error:
        raise yaml.constructor.ConstructorError(
            None, None, "invalid XML-RPC DateTime value", node.start_mark) from error
    return value["value"]


_SnapshotSafeLoader.add_constructor(
    "tag:yaml.org,2002:python/object:xmlrpc.client.DateTime", _xmlrpc_datetime)


def load_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.load(stream, Loader=_SnapshotSafeLoader) or {}


def atomic_dump_yaml(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(value, stream, allow_unicode=True, sort_keys=True)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def atomic_dump_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(
        prefix=path.name + ".", dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2,
                      sort_keys=True, allow_nan=False)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()

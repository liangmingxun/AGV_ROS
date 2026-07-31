# -*- coding: utf-8 -*-
"""UDP v3 codec for the Windows ArUco sender.

The wire format is canonical UTF-8 JSON wrapped in an application-level CRC32
field.  The receiver package contains an intentionally identical codec.
"""
from __future__ import annotations

import json
import re
import zlib
from typing import Any, Dict

PROTOCOL_NAME = "multi_agv_aruco_pose"
PROTOCOL_VERSION = 3
MAX_PACKET_BYTES = 8192
_CRC_RE = re.compile(r"^[0-9a-fA-F]{8}$")


def canonical_json_bytes(value: Any) -> bytes:
    return json.dumps(
        value,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
        allow_nan=False,
    ).encode("utf-8")


def encode_packet(body: Dict[str, Any]) -> bytes:
    if not isinstance(body, dict):
        raise TypeError("packet body must be a dict")
    body_bytes = canonical_json_bytes(body)
    crc32 = f"{zlib.crc32(body_bytes) & 0xFFFFFFFF:08x}"
    packet = canonical_json_bytes({"body": body, "crc32": crc32})
    if len(packet) > MAX_PACKET_BYTES:
        raise ValueError(f"UDP packet is too large: {len(packet)} bytes")
    return packet


def decode_packet(packet: bytes) -> Dict[str, Any]:
    if not isinstance(packet, (bytes, bytearray)):
        raise TypeError("packet must be bytes")
    if len(packet) > MAX_PACKET_BYTES:
        raise ValueError("packet exceeds maximum size")
    envelope = json.loads(bytes(packet).decode("utf-8"))
    if not isinstance(envelope, dict) or set(envelope) != {"body", "crc32"}:
        raise ValueError("invalid packet envelope")
    body = envelope["body"]
    if not isinstance(body, dict):
        raise ValueError("packet body must be a mapping")
    expected = envelope["crc32"]
    if not isinstance(expected, str) or not _CRC_RE.fullmatch(expected):
        raise ValueError("invalid CRC32 field")
    actual = f"{zlib.crc32(canonical_json_bytes(body)) & 0xFFFFFFFF:08x}"
    if expected.lower() != actual:
        raise ValueError("CRC32 mismatch")
    if body.get("protocol") != PROTOCOL_NAME:
        raise ValueError("unsupported protocol")
    version = body.get("version")
    if isinstance(version, bool) or not isinstance(version, int):
        raise ValueError("protocol version must be an integer")
    if version != PROTOCOL_VERSION:
        raise ValueError("unsupported protocol version")
    return body

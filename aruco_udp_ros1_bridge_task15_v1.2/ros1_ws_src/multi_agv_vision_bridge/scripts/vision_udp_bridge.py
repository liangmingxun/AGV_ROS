#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Validate Windows ArUco UDP v3 and publish Task 15 frozen raw inputs.

This node is a transport/interface bridge only.  It does not transform tags to
base_link/load center, filter, publish transforms, compute aggregate state, or emit any
motion command.
"""
from __future__ import annotations

import math
import select
import socket
import threading
import time
import zlib
from collections import deque
from dataclasses import dataclass
from typing import Dict, Mapping, Optional

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Float64, String, UInt64

from multi_agv_vision_bridge.protocol import MAX_PACKET_BYTES, decode_packet
from multi_agv_vision_bridge.validation import (
    EXPECTED_ENTITY_IDS,
    EntitySpec,
    GroundReferenceSpec,
    PlaneCalibrationLimits,
    TimestampPolicy,
    ValidatedPacket,
    Workspace,
    validate_body,
)


@dataclass
class EntityState:
    entity_id: str
    aruco_id: int
    plane_id: str
    marker_height_m: float
    pose_topic: str
    confidence_topic: str
    detected_topic: str
    last_seen_monotonic: float = 0.0
    last_quality: float = 0.0
    last_capture_time_unix_ns: int = 0
    last_send_time_unix_ns: int = 0
    last_receipt_time_unix_ns: int = 0
    detected_in_last_packet: bool = False


class VisionUdpBridge:
    def __init__(self) -> None:
        self.bind_ip = str(rospy.get_param("~bind_ip", "192.168.6.101"))
        self.bind_port = int(rospy.get_param("~bind_port", 15001))
        if self.bind_ip != "192.168.6.101" or self.bind_port != 15001:
            raise rospy.ROSInitException(
                "Task 15 bridge bind endpoint must be 192.168.6.101:15001"
            )
        self.expected_source_ip = str(
            rospy.get_param("~expected_source_ip", "192.168.6.100")
        )
        self.expected_source_port = int(
            rospy.get_param("~expected_source_port", 15000)
        )
        if (
            self.expected_source_ip != "192.168.6.100"
            or self.expected_source_port != 15000
        ):
            raise rospy.ROSInitException(
                "Task 15 Windows source endpoint must be 192.168.6.100:15000"
            )
        self.frame_id = str(rospy.get_param("~frame_id", "world"))
        if self.frame_id != "world":
            raise rospy.ROSInitException("Task 15 bridge frame_id must be world")

        ground_cfg = rospy.get_param("~ground_reference", {})
        if not isinstance(ground_cfg, dict):
            raise rospy.ROSInitException("~ground_reference must be a mapping")
        marker_ids = ground_cfg.get("marker_ids")
        if (
            not isinstance(marker_ids, list)
            or len(marker_ids) != 4
            or any(
                isinstance(value, bool) or not isinstance(value, int) or value < 0
                for value in marker_ids
            )
            or len(set(marker_ids)) != 4
        ):
            raise rospy.ROSInitException(
                "~ground_reference.marker_ids must contain four unique IDs"
            )
        try:
            self.ground_reference_spec = GroundReferenceSpec(
                mode=str(ground_cfg["mode"]),
                marker_ids=tuple(marker_ids),
                origin_id=int(ground_cfg["origin_id"]),
                x_axis_id=int(ground_cfg["x_axis_id"]),
                y_side_id=int(ground_cfg["y_side_id"]),
                marker_length_m=float(ground_cfg["marker_length_m"]),
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise rospy.ROSInitException(
                f"incomplete ground_reference configuration: {exc}"
            )
        roles = {
            self.ground_reference_spec.origin_id,
            self.ground_reference_spec.x_axis_id,
            self.ground_reference_spec.y_side_id,
        }
        if (
            self.ground_reference_spec.mode != "ground_aruco_dynamic"
            or len(roles) != 3
            or not roles.issubset(set(self.ground_reference_spec.marker_ids))
            or not math.isfinite(self.ground_reference_spec.marker_length_m)
            or self.ground_reference_spec.marker_length_m <= 0.0
        ):
            raise rospy.ROSInitException("invalid ground_reference configuration")

        timestamp_mode = str(rospy.get_param("~timestamp/mode", "receipt"))
        if timestamp_mode not in ("receipt", "capture_synced"):
            raise rospy.ROSInitException("timestamp.mode must be receipt or capture_synced")
        if timestamp_mode == "capture_synced" and bool(rospy.get_param("/use_sim_time", False)):
            raise rospy.ROSInitException("capture_synced is incompatible with /use_sim_time")
        self.timestamp_policy = TimestampPolicy(
            mode=timestamp_mode,
            maximum_source_processing_sec=float(
                rospy.get_param("~timestamp/maximum_source_processing_sec", 0.10)
            ),
            maximum_transport_delay_sec=float(
                rospy.get_param("~timestamp/maximum_transport_delay_sec", 0.05)
            ),
            maximum_future_offset_sec=float(
                rospy.get_param("~timestamp/maximum_future_offset_sec", 0.02)
            ),
        )
        timestamp_limits = (
            self.timestamp_policy.maximum_source_processing_sec,
            self.timestamp_policy.maximum_transport_delay_sec,
            self.timestamp_policy.maximum_future_offset_sec,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in timestamp_limits):
            raise rospy.ROSInitException("timestamp limits must be non-negative")

        self.camera_timeout = float(rospy.get_param("~camera_timeout_sec", 0.50))
        self.diagnostic_detection_timeout = float(
            rospy.get_param("~diagnostic_detection_timeout_sec", 0.25)
        )
        if not math.isfinite(self.camera_timeout) or self.camera_timeout <= 0.0:
            raise rospy.ROSInitException("camera_timeout_sec must be finite and positive")
        if (
            not math.isfinite(self.diagnostic_detection_timeout)
            or self.diagnostic_detection_timeout <= 0.0
        ):
            raise rospy.ROSInitException(
                "diagnostic_detection_timeout_sec must be finite and positive"
            )
        self.publish_raw_json = bool(rospy.get_param("~publish_raw_json", False))

        workspace_enabled = bool(rospy.get_param("~workspace/enabled", False))
        self.workspace = Workspace(
            enabled=workspace_enabled,
            x_min_m=float(rospy.get_param("~workspace/x_min_m", -1000.0)),
            x_max_m=float(rospy.get_param("~workspace/x_max_m", 1000.0)),
            y_min_m=float(rospy.get_param("~workspace/y_min_m", -1000.0)),
            y_max_m=float(rospy.get_param("~workspace/y_max_m", 1000.0)),
        )
        workspace_bounds = (
            self.workspace.x_min_m,
            self.workspace.x_max_m,
            self.workspace.y_min_m,
            self.workspace.y_max_m,
        )
        if (
            not all(math.isfinite(value) for value in workspace_bounds)
            or self.workspace.x_min_m > self.workspace.x_max_m
            or self.workspace.y_min_m > self.workspace.y_max_m
        ):
            raise rospy.ROSInitException("workspace bounds are invalid")

        plane_cfg = rospy.get_param("~planes", {})
        if not isinstance(plane_cfg, dict) or not plane_cfg:
            raise rospy.ROSInitException("~planes must be a non-empty mapping")
        if set(plane_cfg) != {"agv_marker_plane", "load_marker_plane"}:
            raise rospy.ROSInitException(
                "~planes must contain exactly agv_marker_plane and load_marker_plane"
            )
        plane_heights: Dict[str, float] = {}
        self.calibration_limits: Dict[str, PlaneCalibrationLimits] = {}
        for plane_id, item in plane_cfg.items():
            if not isinstance(plane_id, str) or not plane_id or not isinstance(item, dict):
                raise rospy.ROSInitException("invalid plane configuration")
            height = float(item.get("height_m", 0.0))
            if not math.isfinite(height):
                raise rospy.ROSInitException(f"non-finite height for plane {plane_id}")
            plane_heights[plane_id] = height
            try:
                limits = PlaneCalibrationLimits(
                    height_m=height,
                    maximum_rmse_m=float(item["max_calibration_rmse_m"]),
                    maximum_point_error_m=float(
                        item["max_calibration_point_error_m"]
                    ),
                    minimum_inliers=int(item["min_inliers"]),
                    minimum_inlier_ratio=float(item["min_inlier_ratio"]),
                    maximum_validation_rmse_m=float(
                        item["max_validation_rmse_m"]
                    ),
                    maximum_validation_point_error_m=float(
                        item["max_validation_point_error_m"]
                    ),
                    minimum_validation_points=int(item["min_validation_points"]),
                    minimum_fit_points=int(item["min_calibration_points"]),
                )
            except (KeyError, TypeError, ValueError) as exc:
                raise rospy.ROSInitException(
                    f"incomplete calibration limits for plane {plane_id}: {exc}"
                )
            floating_limits = (
                limits.maximum_rmse_m,
                limits.maximum_point_error_m,
                limits.minimum_inlier_ratio,
                limits.maximum_validation_rmse_m,
                limits.maximum_validation_point_error_m,
            )
            if (
                not all(math.isfinite(value) for value in floating_limits)
                or limits.maximum_rmse_m < 0.0
                or limits.maximum_point_error_m < 0.0
                or not 0.0 <= limits.minimum_inlier_ratio <= 1.0
                or limits.maximum_validation_rmse_m < 0.0
                or limits.maximum_validation_point_error_m < 0.0
                or limits.minimum_inliers <= 0
                or limits.minimum_validation_points <= 0
                or limits.minimum_fit_points <= 0
                or limits.minimum_inliers > limits.minimum_fit_points
            ):
                raise rospy.ROSInitException(
                    f"invalid calibration limits for plane {plane_id}"
                )
            self.calibration_limits[plane_id] = limits

        entity_items = rospy.get_param("~entities", [])
        if not isinstance(entity_items, list) or len(entity_items) != 4:
            raise rospy.ROSInitException("~entities must contain exactly four mappings")
        self.states: Dict[str, EntityState] = {}
        self.entity_specs: Dict[str, EntitySpec] = {}
        self.pose_publishers = {}
        self.confidence_publishers = {}
        self.detected_publishers = {}
        aruco_ids = set()
        topics = set()
        for item in entity_items:
            if not isinstance(item, dict):
                raise rospy.ROSInitException("each entity mapping must be a dictionary")
            entity_id = item.get("entity_id")
            if not isinstance(entity_id, str) or entity_id not in EXPECTED_ENTITY_IDS:
                raise rospy.ROSInitException(f"invalid entity_id: {entity_id}")
            aruco_id = item.get("aruco_id")
            if isinstance(aruco_id, bool) or not isinstance(aruco_id, int) or aruco_id < 0:
                raise rospy.ROSInitException(f"invalid aruco_id for {entity_id}")
            plane_id = item.get("plane_id")
            expected_plane_id = (
                "load_marker_plane" if entity_id == "load" else "agv_marker_plane"
            )
            if (
                not isinstance(plane_id, str)
                or plane_id not in plane_heights
                or plane_id != expected_plane_id
            ):
                raise rospy.ROSInitException(f"invalid plane_id for {entity_id}")
            pose_topic = str(item.get("pose_topic", ""))
            confidence_topic = str(item.get("confidence_topic", ""))
            detected_topic = str(item.get("detected_topic", f"/vision/aruco/{entity_id}_detected"))
            expected_pose_topic = f"/camera/world/{entity_id}_tag_pose"
            expected_confidence_topic = f"/camera/world/{entity_id}_confidence"
            if (
                pose_topic != expected_pose_topic
                or confidence_topic != expected_confidence_topic
                or not detected_topic
            ):
                raise rospy.ROSInitException(
                    f"Task 15 frozen topics are invalid for {entity_id}"
                )
            if aruco_id in aruco_ids:
                raise rospy.ROSInitException("duplicate ArUco ID")
            for topic in (pose_topic, confidence_topic, detected_topic):
                if topic in topics:
                    raise rospy.ROSInitException(f"duplicate topic: {topic}")
                topics.add(topic)
            aruco_ids.add(aruco_id)
            state = EntityState(
                entity_id=entity_id,
                aruco_id=aruco_id,
                plane_id=plane_id,
                marker_height_m=plane_heights[plane_id],
                pose_topic=pose_topic,
                confidence_topic=confidence_topic,
                detected_topic=detected_topic,
            )
            self.states[entity_id] = state
            self.entity_specs[entity_id] = EntitySpec(
                entity_id=entity_id,
                aruco_id=aruco_id,
                plane_id=plane_id,
                marker_height_m=plane_heights[plane_id],
            )
            self.pose_publishers[entity_id] = rospy.Publisher(
                pose_topic, PoseStamped, queue_size=10, latch=False
            )
            self.confidence_publishers[entity_id] = rospy.Publisher(
                confidence_topic, Float64, queue_size=10, latch=False
            )
            self.detected_publishers[entity_id] = rospy.Publisher(
                detected_topic, Bool, queue_size=2, latch=False
            )
        if set(self.states) != EXPECTED_ENTITY_IDS:
            raise rospy.ROSInitException("entities must be agv1/agv2/agv3/load")
        if aruco_ids.intersection(self.ground_reference_spec.marker_ids):
            raise rospy.ROSInitException(
                "entity ArUco IDs must be distinct from ground-reference IDs"
            )

        self.alive_publisher = rospy.Publisher(
            "/vision/aruco/alive", Bool, queue_size=2, latch=True
        )
        self.diagnostic_publisher = rospy.Publisher(
            "/vision/aruco/diagnostics", DiagnosticArray, queue_size=5, latch=False
        )
        self.calibration_epoch_publisher = rospy.Publisher(
            "/vision/aruco/calibration_epoch", UInt64, queue_size=1, latch=True
        )
        self.raw_json_publisher = (
            rospy.Publisher("/vision/aruco/raw_json", String, queue_size=2, latch=False)
            if self.publish_raw_json
            else None
        )

        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 262144)
        self.socket.bind((self.bind_ip, self.bind_port))
        self.socket.setblocking(False)

        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.thread = threading.Thread(target=self._receive_loop, name="vision_udp_rx", daemon=True)

        self.current_session: Optional[str] = None
        self.closed_sessions = deque(maxlen=16)
        self.closed_session_set = set()
        self.last_seq: Optional[int] = None
        self.last_packet_monotonic = 0.0
        self.last_capture_time_unix_ns = 0
        self.last_send_time_unix_ns = 0
        self.last_receipt_time_unix_ns = 0
        self.last_processing_ms = 0.0
        self.last_source_processing_sec = 0.0
        self.last_transport_delay_sec = 0.0
        self.last_future_offset_sec = 0.0
        self.last_ground_reference: Dict[str, object] = {}
        self.last_calibration: Dict[str, float] = {}
        self.calibration_epoch_unix_ns = 0
        self.calibration_epoch_token = 0
        self.calibration_epoch_session = ""
        self.packet_count = 0
        self.sequence_reject_count = 0
        self.sequence_gap_count = 0
        self.invalid_count = 0
        self.source_reject_count = 0
        self.session_reject_count = 0

        self.status_timer = rospy.Timer(rospy.Duration(0.05), self._status_timer)
        self.diagnostic_timer = rospy.Timer(rospy.Duration(1.0), self._diagnostic_timer)
        rospy.on_shutdown(self.shutdown)

    def start(self) -> None:
        rospy.loginfo(
            "Task15 vision UDP v3 bridge listening on %s:%d source=%s:%d timestamp=%s",
            self.bind_ip,
            self.bind_port,
            self.expected_source_ip,
            self.expected_source_port,
            self.timestamp_policy.mode,
        )
        self.alive_publisher.publish(Bool(data=False))
        for publisher in self.detected_publishers.values():
            publisher.publish(Bool(data=False))
        self.thread.start()

    def _receive_loop(self) -> None:
        while not self.stop_event.is_set() and not rospy.is_shutdown():
            try:
                readable, _, _ = select.select([self.socket], [], [], 0.1)
                if not readable:
                    continue
                packet, source = self.socket.recvfrom(MAX_PACKET_BYTES + 1)
                receipt_time_unix_ns = time.time_ns()
                receipt_stamp = rospy.Time.now()
                if source != (self.expected_source_ip, self.expected_source_port):
                    with self.lock:
                        self.source_reject_count += 1
                    rospy.logwarn_throttle(
                        5.0,
                        "rejected vision UDP source: %s:%d",
                        source[0],
                        source[1],
                    )
                    continue
                body = decode_packet(packet)
                validated = validate_body(
                    body,
                    expected_source_ip=self.expected_source_ip,
                    frame_id=self.frame_id,
                    entity_specs=self.entity_specs,
                    ground_reference_spec=self.ground_reference_spec,
                    calibration_limits=self.calibration_limits,
                    workspace=self.workspace,
                    timestamp_policy=self.timestamp_policy,
                    receipt_time_unix_ns=receipt_time_unix_ns,
                )
                self._handle_packet(validated, packet, receipt_time_unix_ns, receipt_stamp)
            except (ValueError, UnicodeError, KeyError, TypeError, OverflowError) as exc:
                with self.lock:
                    self.invalid_count += 1
                rospy.logwarn_throttle(2.0, "invalid vision UDP packet: %s", exc)
            except OSError as exc:
                if self.stop_event.is_set() or rospy.is_shutdown():
                    break
                with self.lock:
                    self.invalid_count += 1
                rospy.logwarn_throttle(2.0, "vision UDP socket error: %s", exc)
            except Exception as exc:
                # A malformed network input or unexpected conversion failure must
                # not silently terminate the only vision receive thread.
                with self.lock:
                    self.invalid_count += 1
                rospy.logerr_throttle(
                    2.0,
                    "unexpected vision UDP receive failure (%s): %s",
                    type(exc).__name__,
                    exc,
                )

    def _remember_closed_session(self, session: str) -> None:
        if session in self.closed_session_set:
            return
        if len(self.closed_sessions) == self.closed_sessions.maxlen:
            oldest = self.closed_sessions.popleft()
            self.closed_session_set.discard(oldest)
        self.closed_sessions.append(session)
        self.closed_session_set.add(session)

    def _accept_sequence(self, packet: ValidatedPacket) -> bool:
        with self.lock:
            if self.current_session is None:
                self.current_session = packet.session_id
                self.last_seq = None
                rospy.loginfo("new Windows vision session: %s", packet.session_id)
            elif packet.session_id != self.current_session:
                if packet.session_id in self.closed_session_set:
                    self.session_reject_count += 1
                    return False
                self._remember_closed_session(self.current_session)
                self.current_session = packet.session_id
                self.last_seq = None
                rospy.loginfo("new Windows vision session: %s", packet.session_id)
            if self.last_seq is not None:
                if packet.seq <= self.last_seq:
                    self.sequence_reject_count += 1
                    return False
                if packet.seq > self.last_seq + 1:
                    self.sequence_gap_count += packet.seq - self.last_seq - 1
            self.last_seq = packet.seq
            return True

    def _pose_stamp(self, packet: ValidatedPacket, receipt_stamp: rospy.Time) -> rospy.Time:
        if self.timestamp_policy.mode == "receipt":
            return receipt_stamp
        return rospy.Time.from_sec(packet.capture_time_unix_ns * 1e-9)

    def _handle_packet(
        self,
        packet: ValidatedPacket,
        raw_packet: bytes,
        receipt_time_unix_ns: int,
        receipt_stamp: rospy.Time,
    ) -> None:
        if not self._accept_sequence(packet):
            return
        calibration_epoch_unix_ns = int(
            packet.ground_reference["calibrated_at_unix_ns"]
        )
        calibration_epoch_changed = (
            calibration_epoch_unix_ns != self.calibration_epoch_unix_ns
            or packet.session_id != self.calibration_epoch_session
        )
        if calibration_epoch_changed:
            epoch_identity = (
                f"{packet.session_id}:{calibration_epoch_unix_ns}".encode("utf-8")
            )
            calibration_epoch_token = zlib.crc32(epoch_identity) & 0xFFFFFFFF
            if calibration_epoch_token == 0:
                calibration_epoch_token = 1
            previous_epoch = self.calibration_epoch_unix_ns
            with self.lock:
                self.calibration_epoch_unix_ns = calibration_epoch_unix_ns
                self.calibration_epoch_token = calibration_epoch_token
                self.calibration_epoch_session = packet.session_id
            self.calibration_epoch_publisher.publish(
                UInt64(data=calibration_epoch_unix_ns)
            )
            if previous_epoch:
                rospy.logwarn(
                    "ground-reference calibration epoch changed: %d -> %d; "
                    "Task15 filters must reset and an active formal run is invalid",
                    previous_epoch,
                    calibration_epoch_unix_ns,
                )
            else:
                rospy.loginfo(
                    "accepted ground-reference calibration epoch=%d token=%d",
                    calibration_epoch_unix_ns,
                    calibration_epoch_token,
                )
        now_monotonic = time.monotonic()
        detected_entities = {detection.entity_id for detection in packet.detections}
        with self.lock:
            self.last_packet_monotonic = now_monotonic
            self.last_capture_time_unix_ns = packet.capture_time_unix_ns
            self.last_send_time_unix_ns = packet.send_time_unix_ns
            self.last_receipt_time_unix_ns = receipt_time_unix_ns
            self.last_processing_ms = packet.processing_ms
            self.last_source_processing_sec = packet.source_processing_sec
            self.last_transport_delay_sec = packet.transport_delay_sec
            self.last_future_offset_sec = packet.future_offset_sec
            self.last_ground_reference = dict(packet.ground_reference)
            self.last_calibration = dict(packet.calibration)
            self.packet_count += 1
            for entity_id, state in self.states.items():
                state.detected_in_last_packet = entity_id in detected_entities

        stamp = self._pose_stamp(packet, receipt_stamp)
        # Frozen order: confidence first, then matching PoseStamped.
        for detection in packet.detections:
            state = self.states[detection.entity_id]
            self.confidence_publishers[detection.entity_id].publish(
                Float64(data=detection.quality)
            )
            message = PoseStamped()
            message.header.stamp = stamp
            # ROS1 publishers own Header.seq and overwrite application values.
            # A versioned frame_id is therefore the ordering-safe, in-band
            # world-frame generation identity consumed by Task15.
            message.header.frame_id = (
                f"{self.frame_id}@{self.calibration_epoch_token:08x}"
            )
            message.pose.position.x = detection.x_m
            message.pose.position.y = detection.y_m
            message.pose.position.z = state.marker_height_m
            message.pose.orientation.z = math.sin(0.5 * detection.yaw_rad)
            message.pose.orientation.w = math.cos(0.5 * detection.yaw_rad)
            self.pose_publishers[detection.entity_id].publish(message)
            with self.lock:
                state.last_seen_monotonic = now_monotonic
                state.last_quality = detection.quality
                state.last_capture_time_unix_ns = packet.capture_time_unix_ns
                state.last_send_time_unix_ns = packet.send_time_unix_ns
                state.last_receipt_time_unix_ns = receipt_time_unix_ns

        for entity_id, publisher in self.detected_publishers.items():
            publisher.publish(Bool(data=entity_id in detected_entities))
        if self.raw_json_publisher is not None:
            self.raw_json_publisher.publish(String(data=raw_packet.decode("utf-8")))

    def _status_timer(self, _event) -> None:
        now = time.monotonic()
        with self.lock:
            camera_alive = (
                self.last_packet_monotonic > 0.0
                and now - self.last_packet_monotonic <= self.camera_timeout
            )
        self.alive_publisher.publish(Bool(data=camera_alive))
        if not camera_alive:
            for publisher in self.detected_publishers.values():
                publisher.publish(Bool(data=False))

    def _diagnostic_timer(self, _event) -> None:
        now = time.monotonic()
        array = DiagnosticArray()
        array.header.stamp = rospy.Time.now()
        with self.lock:
            packet_age = (
                now - self.last_packet_monotonic
                if self.last_packet_monotonic > 0.0
                else float("inf")
            )
            receiver = DiagnosticStatus()
            receiver.name = "multi_agv_vision_bridge/udp_v3_receiver"
            receiver.hardware_id = (
                f"{self.expected_source_ip}:{self.expected_source_port}"
            )
            receiver.level = (
                DiagnosticStatus.OK if packet_age <= self.camera_timeout else DiagnosticStatus.ERROR
            )
            receiver.message = "receiving" if receiver.level == DiagnosticStatus.OK else "camera UDP timeout"
            receiver.values = [
                KeyValue("bind", f"{self.bind_ip}:{self.bind_port}"),
                KeyValue("expected_source_ip", self.expected_source_ip),
                KeyValue("expected_source_port", str(self.expected_source_port)),
                KeyValue("timestamp_mode", self.timestamp_policy.mode),
                KeyValue("packet_age_sec", f"{packet_age:.3f}"),
                KeyValue("packet_count", str(self.packet_count)),
                KeyValue("sequence_reject_count", str(self.sequence_reject_count)),
                KeyValue("sequence_gap_count", str(self.sequence_gap_count)),
                KeyValue("invalid_packets", str(self.invalid_count)),
                KeyValue("rejected_sources", str(self.source_reject_count)),
                KeyValue("rejected_sessions", str(self.session_reject_count)),
                KeyValue("session_id", self.current_session or "none"),
                KeyValue(
                    "calibration_epoch_unix_ns",
                    str(self.calibration_epoch_unix_ns),
                ),
                KeyValue(
                    "calibration_epoch_token",
                    str(self.calibration_epoch_token),
                ),
                KeyValue("last_seq", str(self.last_seq) if self.last_seq is not None else "none"),
                KeyValue("capture_time_unix_ns", str(self.last_capture_time_unix_ns)),
                KeyValue("send_time_unix_ns", str(self.last_send_time_unix_ns)),
                KeyValue("receipt_time_unix_ns", str(self.last_receipt_time_unix_ns)),
                KeyValue("processing_ms", f"{self.last_processing_ms:.3f}"),
                KeyValue("source_processing_sec", f"{self.last_source_processing_sec:.6f}"),
                KeyValue("transport_delay_sec", f"{self.last_transport_delay_sec:.6f}"),
                KeyValue("future_offset_sec", f"{self.last_future_offset_sec:.6f}"),
                KeyValue(
                    "transport_delay_interpretation",
                    "clock-synchronized" if self.timestamp_policy.mode == "capture_synced" else "diagnostic-only-unsynchronized",
                ),
            ]
            for key, value in sorted(self.last_calibration.items()):
                receiver.values.append(KeyValue(f"calibration.{key}", str(value)))
            for key, value in sorted(self.last_ground_reference.items()):
                receiver.values.append(
                    KeyValue(f"ground_reference.{key}", str(value))
                )
            array.status.append(receiver)

            for entity_id in sorted(self.states):
                state = self.states[entity_id]
                age = (
                    now - state.last_seen_monotonic
                    if state.last_seen_monotonic > 0.0
                    else float("inf")
                )
                fresh_for_diagnostic = packet_age <= self.camera_timeout and age <= self.diagnostic_detection_timeout
                status = DiagnosticStatus()
                status.name = f"multi_agv_vision_bridge/{entity_id}"
                status.hardware_id = f"aruco:{state.aruco_id}"
                status.level = DiagnosticStatus.OK if fresh_for_diagnostic else DiagnosticStatus.WARN
                status.message = "raw tag recently detected" if fresh_for_diagnostic else "raw tag stale/not detected"
                status.values = [
                    KeyValue("entity_id", state.entity_id),
                    KeyValue("plane_id", state.plane_id),
                    KeyValue("marker_height_m", f"{state.marker_height_m:.6f}"),
                    KeyValue("pose_topic", state.pose_topic),
                    KeyValue("confidence_topic", state.confidence_topic),
                    KeyValue("detected_in_last_packet", str(state.detected_in_last_packet)),
                    KeyValue("diagnostic_pose_age_sec", f"{age:.3f}"),
                    KeyValue("last_quality", f"{state.last_quality:.3f}"),
                    KeyValue("last_capture_time_unix_ns", str(state.last_capture_time_unix_ns)),
                    KeyValue("last_send_time_unix_ns", str(state.last_send_time_unix_ns)),
                    KeyValue("last_receipt_time_unix_ns", str(state.last_receipt_time_unix_ns)),
                    KeyValue(
                        "freshness_authority",
                        "diagnostic only; Task15 adapter uses PoseStamped timestamps",
                    ),
                ]
                array.status.append(status)
        self.diagnostic_publisher.publish(array)

    def shutdown(self) -> None:
        self.stop_event.set()
        try:
            self.socket.close()
        except OSError:
            pass
        if self.thread.is_alive():
            self.thread.join(timeout=1.0)


def main() -> None:
    rospy.init_node("vision_udp_bridge")
    bridge = VisionUdpBridge()
    bridge.start()
    rospy.spin()


if __name__ == "__main__":
    main()

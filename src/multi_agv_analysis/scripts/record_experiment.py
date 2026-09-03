#!/usr/bin/env python3

import datetime
import os
import platform
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rosgraph
import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, TriggerResponse

from multi_agv_analysis.io_utils import (
    atomic_dump_json, atomic_dump_yaml, sha256_file)
from multi_agv_analysis.approval import verify_approved_configuration


def _git(repo, *arguments):
    return subprocess.check_output(
        ["git", "-C", str(repo)] + list(arguments),
        text=True).strip()


def _system_publishers():
    publishers, _, _ = rosgraph.Master(
        rospy.get_name()).getSystemState()
    return {topic: list(nodes) for topic, nodes in publishers}


def _system_subscribers():
    _, subscribers, _ = rosgraph.Master(
        rospy.get_name()).getSystemState()
    return {topic: list(nodes) for topic, nodes in subscribers}


def _command_authority(publishers):
    result = {}
    for index in range(1, 4):
        topic = "/agv{}/chassis_command".format(index)
        result[topic] = publishers.get(topic, [])
    return result


def _safe_run_id(value):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", value):
        raise RuntimeError(
            "run_id must contain only letters, digits, '.', '_' and '-'")
    return value


class ExperimentRecorder:
    def __init__(self):
        if not rospy.get_param("~arming_authorized", False):
            raise RuntimeError(
                "recording arming_authorized is false; no bag was started")
        self.output_root = Path(os.path.expanduser(
            rospy.get_param("~output_root"))).resolve()
        requested_run_id = rospy.get_param("~run_id", "").strip()
        if not requested_run_id:
            requested_run_id = datetime.datetime.now().strftime(
                "%Y%m%d_%H%M%S")
        self.run_id = _safe_run_id(requested_run_id)
        self.experiment_id = rospy.get_param("~experiment_id")
        self.method_id = rospy.get_param("~method_id")
        self.recording = rospy.get_param("~experiment_recording")
        registered = self.recording.get(
            "registered_method_ids",
            ["M1_R1", "M2a_R1", "M2b_M2b"])
        if self.method_id not in registered:
            raise RuntimeError(
                "method_id is not preregistered: {}".format(self.method_id))
        self.pair_block_id = rospy.get_param("~pair_block_id", "")
        self.payload_state = rospy.get_param("~payload_state", "unloaded")
        self.localization_source = rospy.get_param(
            "~localization_source", "unknown")
        self.operator = rospy.get_param("~operator", "")
        self.interface_version = rospy.get_param(
            "~interface_version", "agv_ros_interfaces_v1")
        self.topics = list(self.recording.get("topics", []))
        self.required_topics = list(
            self.recording.get("required_topics", []))
        self.camera_mode = rospy.get_param("~camera_mode", False)
        self.require_windows_sender_manifest = rospy.get_param(
            "~require_windows_sender_manifest", True)
        self.virtual_load_from_robots = rospy.get_param(
            "~virtual_load_from_robots", False)
        if self.camera_mode:
            self.required_topics.extend(
                self.recording.get(
                    "camera_virtual_load_required_topics"
                    if self.virtual_load_from_robots
                    else "camera_required_topics", []))
        self.required_topics.extend(
            self.recording.get("method_required_topics", {}).get(
                self.method_id, []))
        self.required_topics = list(dict.fromkeys(self.required_topics))
        self.config_files = [
            Path(value).resolve()
            for value in rospy.get_param("~config_files", [])]
        camera_config_values = [
            str(value).strip()
            for value in rospy.get_param("~camera_config_files", [])
            if str(value).strip()]
        if (self.camera_mode and self.require_windows_sender_manifest and
                len(camera_config_values) != 2):
            raise RuntimeError(
                "camera_mode requires Robot1 vision configuration and the "
                "Windows sender manifest in camera_config_files")
        if (self.camera_mode and not self.require_windows_sender_manifest and
                len(camera_config_values) not in (1, 2)):
            raise RuntimeError(
                "camera_mode requires at least the Robot1 vision "
                "configuration in camera_config_files")
        self.windows_sender_manifest_present = (
            self.camera_mode and len(camera_config_values) == 2)
        if self.camera_mode:
            self.config_files.extend(
                Path(value).resolve() for value in camera_config_values)
        self.require_approved_config = rospy.get_param(
            "~require_approved_config", False)
        self.formal_statistics_requested = rospy.get_param(
            "~formal_statistics_requested", False)
        self.approval_id = rospy.get_param("~approval_id", "").strip()
        registry = rospy.get_param("~approval_registry", "").strip()
        self.approval_registry = (
            Path(registry).resolve() if registry else None)
        self.run_dir = self.output_root / self.run_id
        if self.run_dir.exists():
            raise RuntimeError(
                "run directory already exists: {}".format(self.run_dir))
        if not self.topics:
            raise RuntimeError("record topic list is empty")
        self.bag_process = None
        self.bag_node_name = "/task17_bag_{}".format(
            re.sub(r"[^A-Za-z0-9_]", "_", self.run_id))
        self.manifest = {}
        self.shutdown_lock = threading.Lock()
        self.closed = False
        self.stop_service = None
        self.armed_publisher = rospy.Publisher(
            "/experiment_recorder/armed", Bool, queue_size=1, latch=True)
        self.method_publisher = rospy.Publisher(
            "/experiment_recorder/method_id", String, queue_size=1, latch=True)
        self.armed_publisher.publish(Bool(data=False))
        self.method_publisher.publish(String(data=self.method_id))

    def preflight(self):
        deadline = time.monotonic() + float(
            rospy.get_param("~preflight_wait_seconds", 15.0))
        last_problem = "ROS graph is not ready"
        while not rospy.is_shutdown():
            publishers = _system_publishers()
            authority = _command_authority(publishers)
            invalid_authority = {
                topic: nodes for topic, nodes in authority.items()
                if len(nodes) != 1}
            move_base_topics = [
                topic for topic, nodes in authority.items()
                if any("move_base" in node for node in nodes)]
            missing = [
                topic for topic in self.required_topics
                if not publishers.get(topic)]
            if not invalid_authority and not move_base_topics and not missing:
                return authority
            last_problem = (
                "authority={} move_base={} missing={}".format(
                    invalid_authority, move_base_topics, missing))
            if time.monotonic() >= deadline:
                break
            rospy.sleep(0.1)
        raise RuntimeError("preflight timed out: {}".format(last_problem))

    def prepare(self, authority):
        self.run_dir.mkdir(parents=True)
        config_dir = self.run_dir / "config"
        config_dir.mkdir()
        package_path = Path(__file__).resolve()
        repo = package_path
        while repo != repo.parent and not (repo / ".git").exists():
            repo = repo.parent
        if not (repo / ".git").exists():
            # Installed scripts live below devel; ask Git for the workspace
            # containing the configured launch files instead.
            candidate = Path(rospy.get_param("~workspace", "")).resolve()
            if not (candidate / ".git").exists():
                raise RuntimeError("cannot locate Git worktree")
            repo = candidate

        hashes = []
        approval = None
        if self.require_approved_config:
            if not self.approval_id or self.approval_registry is None:
                raise RuntimeError(
                    "approved configuration is required but approval_id or "
                    "approval_registry is missing")
            approval = verify_approved_configuration(
                self.approval_registry, self.approval_id,
                self.config_files, self.formal_statistics_requested)
        elif self.formal_statistics_requested:
            raise RuntimeError(
                "formal statistics require the approved configuration gate")
        for index, source in enumerate(self.config_files, start=1):
            if not source.is_file():
                raise RuntimeError(
                    "configuration file does not exist: {}".format(source))
            destination = config_dir / "{:02d}_{}".format(index, source.name)
            shutil.copy2(source, destination)
            hashes.append({
                "path": str(source),
                "archived_path": str(destination.relative_to(self.run_dir)),
                "sha256": sha256_file(destination),
            })

        parameter_snapshot = self.run_dir / "rosparams.yaml"
        subprocess.check_call(
            ["rosparam", "dump", str(parameter_snapshot)])
        hashes.append({
            "path": "ROS parameter server snapshot",
            "archived_path": parameter_snapshot.name,
            "sha256": sha256_file(parameter_snapshot),
        })
        status = _git(repo, "status", "--porcelain",
                      "--untracked-files=normal")
        self.manifest = {
            "schema_version": 1,
            "run_id": self.run_id,
            "experiment_id": self.experiment_id,
            "method_id": self.method_id,
            "interface_version": self.interface_version,
            "camera_mode": self.camera_mode,
            "virtual_load_from_robots": self.virtual_load_from_robots,
            "windows_sender_manifest_required":
                self.require_windows_sender_manifest,
            "windows_sender_manifest_present":
                self.windows_sender_manifest_present,
            "started_at": datetime.datetime.now(
                datetime.timezone.utc).isoformat(),
            "recording_armed_at": None,
            "finished_at": None,
            "git_sha": _git(repo, "rev-parse", "HEAD"),
            "git_branch": _git(repo, "branch", "--show-current"),
            "git_dirty": bool(status),
            "git_status": status.splitlines(),
            "hostname": platform.node(),
            "python_version": platform.python_version(),
            "ros_distro": os.environ.get("ROS_DISTRO", ""),
            "command_authority": authority,
            "command_authority_violations": [],
            "move_base_command_authority": False,
            "record_topics": self.topics,
            "required_topics": self.required_topics,
            "config_hashes": hashes,
            "configuration_approval": approval,
            "formal_statistics_requested":
                self.formal_statistics_requested,
            "bag": "{}.bag".format(self.run_id),
            "bag_exit_code": None,
            "communication_age_note": (
                "Recorded ages are diagnostics only and do not establish an "
                "independent communication watchdog."),
            "metrics": rospy.get_param("~metrics", {}),
        }
        self._write_metadata()

    def _run_meta(self):
        return {
            "schema_version": 1,
            "run_id": self.run_id,
            "experiment_id": self.experiment_id,
            "method_id": self.method_id,
            "pair_block_id": self.pair_block_id,
            "git_sha": self.manifest.get("git_sha", ""),
            "config_hashes": self.manifest.get("config_hashes", []),
            "hostname": self.manifest.get("hostname", ""),
            "software_versions": {
                "interface": self.interface_version,
                "ros_distro": self.manifest.get("ros_distro", ""),
                "python": self.manifest.get("python_version", ""),
            },
            "stm32_firmware": [
                rospy.get_param(
                    "/agv{}/deployment/stm32_firmware".format(index),
                    "unreported")
                for index in range(1, 4)],
            "path_version": rospy.get_param(
                "~path_version", "s_curve_v1"),
            "payload_state": self.payload_state,
            "localization_source": self.localization_source,
            "battery_voltage_start": None,
            "battery_voltage_end": None,
            "valid_run": False,
            "abort_reason": self.manifest.get("abort_reason", ""),
            "operator": self.operator,
            "start_timestamp": self.manifest.get("started_at"),
            "recording_armed_timestamp": self.manifest.get(
                "recording_armed_at"),
            "end_timestamp": self.manifest.get("finished_at"),
        }

    def _write_metadata(self):
        atomic_dump_yaml(self.run_dir / "manifest.yaml", self.manifest)
        atomic_dump_json(self.run_dir / "run_meta.json", self._run_meta())

    def start(self):
        bag_path = self.run_dir / "{}.bag".format(self.run_id)
        command = ["rosbag", "record", "--buffsize=512", "-O", str(bag_path)]
        command.extend(self.topics)
        command.append("__name:={}".format(
            self.bag_node_name.lstrip("/")))
        self.bag_process = subprocess.Popen(command)
        deadline = time.monotonic() + float(
            rospy.get_param("~connection_wait_seconds", 5.0))
        missing = list(self.required_topics)
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.bag_process.poll() is not None:
                raise RuntimeError(
                    "rosbag exited during startup with status {}".format(
                        self.bag_process.returncode))
            subscribers = _system_subscribers()
            missing = [
                topic for topic in self.required_topics
                if self.bag_node_name not in subscribers.get(topic, [])]
            if not missing:
                break
            rospy.sleep(0.05)
        if missing:
            raise RuntimeError(
                "rosbag did not subscribe to required topics before arming: "
                "{}".format(missing))
        self.manifest["recording_armed_at"] = datetime.datetime.now(
            datetime.timezone.utc).isoformat()
        self._write_metadata()
        rospy.loginfo(
            "Experiment recording armed after all required subscriptions "
            "connected: run_id=%s bag=%s", self.run_id, bag_path)
        self.method_publisher.publish(String(data=self.method_id))
        self.armed_publisher.publish(Bool(data=True))
        self.stop_service = rospy.Service(
            "~stop", Trigger, self.stop_recording)

    def shutdown(self):
        with self.shutdown_lock:
            if self.closed:
                return
            self.armed_publisher.publish(Bool(data=False))
            if (self.bag_process is not None and
                    self.bag_process.poll() is None):
                self.bag_process.send_signal(signal.SIGINT)
                try:
                    self.bag_process.wait(timeout=15.0)
                except subprocess.TimeoutExpired:
                    self.bag_process.terminate()
                    self.bag_process.wait(timeout=5.0)
            if self.manifest:
                self.manifest["finished_at"] = datetime.datetime.now(
                    datetime.timezone.utc).isoformat()
                self.manifest["bag_exit_code"] = (
                    self.bag_process.returncode
                    if self.bag_process is not None else None)
                self._write_metadata()
            self.closed = True

    def stop_recording(self, _request):
        """Synchronously close the bag before acknowledging a stop request."""
        self.shutdown()
        exit_code = (
            self.bag_process.returncode
            if self.bag_process is not None else None)
        success = exit_code == 0
        return TriggerResponse(
            success=success,
            message=("recording closed cleanly" if success else
                     "recording close failed with status {}".format(
                         exit_code)))

    def spin(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.bag_process.poll() is not None:
                if rospy.is_shutdown() or self.bag_process.returncode == 0:
                    if not rospy.is_shutdown():
                        rospy.signal_shutdown("rosbag closed cleanly")
                    return
                raise RuntimeError(
                    "rosbag exited unexpectedly with status {}".format(
                        self.bag_process.returncode))
            # This is intentionally a heartbeat, not only a latched edge.  An
            # algorithm that requires the recorder gate can therefore fail to
            # zero if this process is killed without running shutdown().
            self.method_publisher.publish(String(data=self.method_id))
            self.armed_publisher.publish(Bool(data=True))
            current = _command_authority(_system_publishers())
            if current != self.manifest["command_authority"]:
                violation = {
                    "stamp": rospy.Time.now().to_sec(),
                    "expected": self.manifest["command_authority"],
                    "observed": current,
                }
                if (not self.manifest["command_authority_violations"] or
                        self.manifest["command_authority_violations"][-1][
                            "observed"] != current):
                    self.manifest[
                        "command_authority_violations"].append(violation)
                    self._write_metadata()
                    rospy.logerr(
                        "Command authority changed while recording: %s",
                        current)
                raise RuntimeError(
                    "command authority changed while recording")
            rate.sleep()


def main():
    rospy.init_node("experiment_recorder")
    recorder = None
    try:
        recorder = ExperimentRecorder()
        authority = recorder.preflight()
        recorder.prepare(authority)
        recorder.start()
        rospy.on_shutdown(recorder.shutdown)
        recorder.spin()
        return 0
    except Exception as error:
        rospy.logfatal("Experiment recorder failed: %s", error)
        if recorder is not None:
            recorder.shutdown()
        return 2


if __name__ == "__main__":
    sys.exit(main())

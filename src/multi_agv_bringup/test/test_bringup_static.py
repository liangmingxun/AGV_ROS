#!/usr/bin/env python3
import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE.parent


class BringupStaticTest(unittest.TestCase):
    def test_all_launch_files_are_well_formed_and_do_not_start_move_base(self):
        for launch in (PACKAGE / "launch").glob("*.launch"):
            with self.subTest(launch=launch.name):
                ET.parse(launch)
                self.assertNotIn("move_base", launch.read_text(encoding="utf-8"))

    def test_task17_recording_is_subscription_ready_and_auditable(self):
        launch = (
            PACKAGE / "launch" / "experiment.launch"
        ).read_text(encoding="utf-8")
        topics = (
            PACKAGE / "config" / "record_topics.yaml"
        ).read_text(encoding="utf-8")
        recorder = (
            SOURCE_ROOT / "multi_agv_analysis" / "scripts" /
            "record_experiment.py"
        ).read_text(encoding="utf-8")
        metrics = (
            SOURCE_ROOT / "multi_agv_analysis" / "src" /
            "multi_agv_analysis" / "metrics.py"
        ).read_text(encoding="utf-8")

        self.assertIn('name="arming_authorized" default="false"', launch)
        self.assertIn('name="connection_wait_seconds" value="5.0"', launch)
        self.assertIn("/multi_agv/formal_algorithm_state", topics)
        self.assertIn("/agv1/imu", topics)
        self.assertIn("/agv1/odom", topics)
        self.assertIn("minimum_topic_rates:", topics)
        self.assertIn("/agv1/capability_report: 90.0", topics)
        self.assertIn("recommended_topics:", topics)
        self.assertIn("/multi_agv/experiment_state", topics)
        self.assertIn("rosbag did not subscribe to required topics", recorder)
        self.assertIn('"recording_armed_at": None', recorder)
        self.assertIn("/experiment_recorder/armed", recorder)
        self.assertIn("Bool(data=True)", recorder)
        self.assertIn("command authority changed while recording", recorder)
        self.assertIn('"all_samples_fallback"', metrics)
        self.assertIn('"formal_statistics_ready"', metrics)

    def test_task15_camera_entry_is_decoupled_and_hardware_blocked(self):
        launch = (
            PACKAGE / "launch" / "camera_formal.launch"
        ).read_text(encoding="utf-8")
        config = (
            PACKAGE / "config" / "localization_camera.yaml"
        ).read_text(encoding="utf-8")
        adapter = (
            SOURCE_ROOT / "multi_agv_control" / "src" /
            "camera_pose_adapter_node.cpp"
        ).read_text(encoding="utf-8")
        estimator = (
            SOURCE_ROOT / "multi_agv_control" / "src" /
            "path_state_estimator_node.cpp"
        ).read_text(encoding="utf-8")
        recorded = (
            PACKAGE / "config" / "record_topics.yaml"
        ).read_text(encoding="utf-8")

        self.assertIn('name="calibration_authorized" default="false"', launch)
        self.assertIn(
            "three_agv_tag_extrinsics_measured_load_pending_readonly", config)
        self.assertIn("calibration_authorized: false", config)
        self.assertIn("extrinsics_frozen: false", config)
        self.assertIn("require_confidence: true", config)
        self.assertEqual(config.count("tag_to_target_x:"), 4)
        self.assertIn("SOURCE_CAMERA", estimator)
        self.assertIn("SOURCE_FUSED", estimator)
        self.assertIn('localization_mode_ != "fused"', estimator)
        self.assertIn("retained as raw but rejected", adapter)
        self.assertNotIn("TransformBroadcaster", adapter)
        self.assertIn("/camera/world/agv1_tag_pose", recorded)
        self.assertIn("/pose_provider/load/pose_filtered", recorded)
        self.assertIn("/pose_provider/agv1/base_pose_fused", recorded)

    def test_task16_and_task18_remain_fake_or_rehearsal_only(self):
        m2b = (
            PACKAGE / "config" / "exp2b_M2b.yaml"
        ).read_text(encoding="utf-8")
        registry = (
            PACKAGE / "config" / "approved_config_registry.yaml"
        ).read_text(encoding="utf-8")
        launch = (
            PACKAGE / "launch" / "formal_fake_m2b.launch"
        ).read_text(encoding="utf-8")
        formal_gate = (
            SOURCE_ROOT.parent / "docs" / "test-protocols" /
            "formal-exp2a-gate.md"
        ).read_text(encoding="utf-8")
        self.assertEqual(m2b.count("hardware_execution_authorized: false"), 2)
        self.assertIn("capability_policy: log_only_never_used_by_control",
                      m2b)
        self.assertIn("formal_fake_algorithm.launch", launch)
        self.assertNotIn("serial", launch)
        self.assertNotIn("formal_statistics_authorized: true", registry)
        self.assertIn("software_rehearsal_only", registry)
        self.assertIn("状态：**关闭", formal_gate)

    def test_optional_software_watchdog_is_observer_only(self):
        launch = (
            PACKAGE / "launch" / "software_watchdog.launch"
        ).read_text(encoding="utf-8")
        script = (
            SOURCE_ROOT / "multi_agv_analysis" / "scripts" /
            "software_watchdog.py"
        ).read_text(encoding="utf-8")
        self.assertIn('name="monitoring_enabled" default="false"', launch)
        self.assertIn("/multi_agv/software_watchdog_ok", script)
        self.assertIn("/multi_agv/software_watchdog_diagnostics", script)
        self.assertNotIn("ChassisCommand", script)
        self.assertNotIn("/chassis_command", script)

    def test_each_car_config_has_explicit_identity_frames_and_calibration(self):
        expected_host_ips = {
            1: "10.134.37.53",
            2: "10.134.37.114",
            3: "10.134.37.239",
        }
        for index in range(1, 4):
            text = (PACKAGE / "config" / f"agv{index}_chassis.yaml").read_text(
                encoding="utf-8")
            for marker in (f"robot_id: agv{index}", f"robot_index: {index}",
                           "serial_device:", f"odom_frame: agv{index}/odom",
                           f"base_frame: agv{index}/base_link", "acc_bias:",
                           "gyro_bias:", "wheel_radius:", "wheel_separation:",
                           "max_wheel_linear_velocity_left:", "transport_type:",
                           "serial_startup_timeout_seconds:",
                           "capability_publish_rate: 100.0",
                           "command_timeout_seconds: 0.20",
                           "sensor_feedback_timeout_seconds: 0.15",
                           "odometry_stationary_wheel_velocity_tolerance: "
                           "0.005"):
                self.assertIn(marker, text)
            self.assertIn(
                "host_ip: {}".format(expected_host_ips[index]), text)

    def test_support_offsets_are_launch_calibration_arguments(self):
        for launch_name in ("car1_master.launch", "car2_client.launch",
                            "car3_client.launch", "chassis_single.launch"):
            text = (PACKAGE / "launch" / launch_name).read_text(encoding="utf-8")
            for marker in ("support_x", "support_y", "support_z"):
                self.assertIn(marker, text)
            self.assertIn('-0.01783', text)
            self.assertIn('name="support_z" default="0.0"', text)

        localization = (PACKAGE / "config" / "localization_odom.yaml").read_text(
            encoding="utf-8")
        self.assertEqual(localization.count(
            "base_to_support: {x: -0.01783, y: 0.0, yaw: 0.0}"), 3)
        for transform in (
                "x: 0.237333702114270, y: 0.074560581501146, "
                "yaw: 0.304395797364615",
                "x: -0.153094727127932, y: 0.161541278437113, "
                "yaw: 0.304395797364615",
                "x: -0.033208005692254, y: -0.220070008114273, "
                "yaw: 0.304395797364615"):
            self.assertIn(transform, localization)
        self.assertIn("maximum_rigid_fit_residual: 0.01", localization)
        self.assertIn("maximum_projection_distance: 0.08", localization)
        self.assertIn("maximum_sync_slop: 0.02", localization)
        self.assertIn("synchronization_queue_size: 64", localization)

        estimator_source = (
            SOURCE_ROOT / "multi_agv_control" / "src" /
            "path_state_estimator_node.cpp"
        ).read_text(encoding="utf-8")
        for marker in (
                "std::deque<OdometrySample>",
                "selectSynchronizedSamples",
                "nearestFreshSample",
                "synchronization_queue_size",
                "ros::TransportHints().reliable().tcpNoDelay()",
                "synchronized && sample != nullptr"):
            self.assertIn(marker, estimator_source)
        self.assertNotIn(".unreliable()", estimator_source)

        tracker = (
            PACKAGE / "config" / "pretest_constant_reference.yaml"
        ).read_text(encoding="utf-8")
        self.assertEqual(tracker.count("{x: -0.01783, y: 0.0}"), 3)
        self.assertIn("chassis_reference_samples: 10001", tracker)
        capability = (
            PACKAGE / "config" / "capability_mapping.yaml"
        ).read_text(encoding="utf-8")
        self.assertEqual(capability.count("{x: -0.01783, y: 0.0}"), 3)
        self.assertIn("offset_compensated_drive_axle_reference", capability)

    def test_three_car_fixture_is_short_s_equilateral_and_still_gated(self):
        path = (PACKAGE / "config" / "path_s_curve.yaml").read_text(
            encoding="utf-8")
        geometry = (PACKAGE / "config" / "support_geometry.yaml").read_text(
            encoding="utf-8")
        pretest = (
            PACKAGE / "config" / "pretest_constant_reference.yaml"
        ).read_text(encoding="utf-8")

        self.assertIn("amplitude: 0.05", path)
        self.assertIn("longitudinal_length: 1.0", path)
        self.assertIn("hardware_execution_authorized: false", path)
        for marker in (
                "role: front",
                "q_tangent: 0.230940107675850",
                "role: left_rear",
                "q_tangent: -0.115470053837925",
                "q_normal: 0.20",
                "role: right_rear",
                "q_normal: -0.20"):
            self.assertIn(marker, geometry)
        self.assertIn("hardware_execution_authorized: false", geometry)
        self.assertIn("hardware_execution_authorized: false", pretest)

    def test_urdf_uses_prefix_for_links_and_joints(self):
        urdf_dir = SOURCE_ROOT / "mycar_description" / "urdf"
        base = (urdf_dir / "car_base.urdf.xacro").read_text(encoding="utf-8")
        laser = (urdf_dir / "car_laser.urdf.xacro").read_text(encoding="utf-8")
        self.assertIn('${prefix}base_link', base)
        self.assertIn('${prefix}${wheel_name}_wheel', base)
        self.assertIn('name="chassis_center_x" value="0.04969"', base)
        self.assertIn('name="wheel_separation" value="0.114"', base)
        self.assertIn('<origin xyz="0 ${(wheel_separation / 2) * flag}',
                      base)
        self.assertIn('${prefix}support', laser)

    def test_robot1_single_s_pretest_is_isolated_and_bounded(self):
        launch = (
            PACKAGE / "launch" / "robot1_single_s_pretest.launch"
        ).read_text(encoding="utf-8")
        config = (
            PACKAGE / "config" / "robot1_single_s_pretest.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn('type="single_car_s_pretest_node"', launch)
        self.assertIn('name="enable_commands" default="false"', launch)
        self.assertIn('name="confirm_test_area_clear" default="false"', launch)
        self.assertIn('name="confirm_wheels_on_floor" default="false"', launch)
        self.assertIn('name="localization_source" default="odom"', launch)
        self.assertNotIn("central_odom_pretest", launch)
        self.assertIn("amplitude: 0.05", config)
        self.assertIn("longitudinal_length: 1.0", config)
        self.assertIn("arc_length_speed: 0.05", config)
        self.assertIn("maximum_wheel_linear_velocity: 0.08", config)
        self.assertIn("base_to_support_x: -0.01783", config)
        self.assertIn("lateral_gain: 3.0", config)
        self.assertIn("heading_gain: 2.5", config)
        self.assertIn("maximum_convergence_time: 5.0", config)
        self.assertIn("hard_stop_speed: 0.09", config)

    def test_central_readonly_entry_has_no_command_authority(self):
        launch = (
            PACKAGE / "launch" / "central_odom_pretest.launch"
        ).read_text(encoding="utf-8")
        controller = (
            SOURCE_ROOT / "multi_agv_control" / "src" /
            "multi_agv_controller_node.cpp"
        ).read_text(encoding="utf-8")
        checker = (
            PACKAGE / "scripts" / "check_three_car_readonly_gate.py"
        ).read_text(encoding="utf-8")

        self.assertIn('name="enable_commands" default="false"', launch)
        self.assertIn('name="enable_derating" default="false"', launch)
        self.assertIn("if (command_publication_authorized_)", controller)
        self.assertIn("publishers were not registered", controller)
        self.assertIn("must have no publisher in read-only mode", checker)
        self.assertIn("--require-valid-state", checker)
        self.assertIn("--maximum-serial-feedback-age", checker)
        self.assertIn("packet_seq did not advance", checker)

    def test_three_car_bounded_entry_is_explicit_and_tightly_bounded(self):
        launch = (
            PACKAGE / "launch" /
            "three_car_unloaded_bounded_pretest.launch"
        ).read_text(encoding="utf-8")
        config = (
            PACKAGE / "config" /
            "three_car_unloaded_bounded_pretest.yaml"
        ).read_text(encoding="utf-8")
        node = (
            SOURCE_ROOT / "multi_agv_control" / "src" /
            "three_car_unloaded_bounded_pretest_node.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn(
            'type="three_car_unloaded_bounded_pretest_node"', launch)
        for marker in (
                'name="enable_commands" default="false"',
                'name="confirm_readonly_gate_passed" default="false"',
                'name="confirm_test_area_clear" default="false"',
                'name="confirm_wheels_on_floor" default="false"',
                'name="confirm_unloaded_40cm_fixture" default="false"'):
            self.assertIn(marker, launch)
        self.assertNotIn(
            'file="$(find multi_agv_bringup)/launch/'
            'central_odom_pretest.launch"', launch)

        for marker in (
                "hardware_execution_authorized: true",
                "load_path_speed: 0.05",
                "target_progress: 1.00",
                "required_command_subscribers: 2",
                "readiness_stable_samples: 20",
                "minimum_battery_voltage: 10.5",
                "maximum_feedback_receive_age: 0.25",
                "maximum_serial_feedback_age: 0.25",
                "maximum_stamp_spread: 0.02",
                "maximum_wheel_linear_velocity: 0.08"):
            self.assertIn(marker, config)
        for marker in (
                "rejectCompetingPublishers",
                "publishRepeatedStop",
                "waitForStopped",
                "virtual load pose or path state is invalid",
                "control loop overrun reported",
                "STM32 serial feedback is stale or future-dated",
                "chassis feedback receive age",
                "STM32 packet sequence age",
                "all six wheels must be stopped before motion"):
            self.assertIn(marker, node)

    def test_three_car_host_and_orchestration_scripts_are_safety_gated(self):
        start = (
            PACKAGE / "scripts" / "start_three_car_chassis.sh"
        ).read_text(encoding="utf-8")
        run = (
            PACKAGE / "scripts" / "run_three_car_unloaded_pretest.sh"
        ).read_text(encoding="utf-8")
        local_clock = (
            PACKAGE / "scripts" / "check_local_clock_sync.sh"
        ).read_text(encoding="utf-8")
        motion_gate = (
            PACKAGE / "scripts" / "require_three_car_motion_gate.sh"
        ).read_text(encoding="utf-8")
        self.assertNotIn("working tree is not clean", start)
        self.assertNotIn("working tree is not clean", run)
        self.assertIn(
            "working tree contains uncommitted changes; continuing by request",
            start)
        self.assertIn(
            "working tree contains uncommitted changes; continuing by request",
            run)
        self.assertIn("/deployment/git_sha", start)
        self.assertIn("/deployment/clean_worktree", start)
        self.assertIn("check_local_clock_sync.sh", start)
        self.assertIn("/deployment/clock_sync_source", start)
        self.assertIn("/deployment/clock_checked_at", start)
        self.assertIn("Leap status", local_clock)
        self.assertIn("CLOCK_SYNC_SOURCE=\"chrony\"", local_clock)
        self.assertIn("CLOCK_SYNC_SOURCE=\"timedatectl\"", local_clock)
        self.assertIn("AGV_MAX_LOCAL_CLOCK_OFFSET_SECONDS", local_clock)
        self.assertIn("serial device", start)
        self.assertIn("--confirm-area-clear", run)
        self.assertIn("--confirm-wheels-on-floor", run)
        self.assertIn("--confirm-unloaded-40cm-fixture", run)
        self.assertIn("check_three_car_readonly_gate.py", run)
        self.assertIn("require_three_car_motion_gate.sh", run)
        self.assertIn("--require-valid-state", motion_gate)
        self.assertIn("PASSED ${passes} CONSECUTIVE WINDOW", motion_gate)
        self.assertIn("/reset_odometry", run)
        self.assertIn("rosbag record", run)
        self.assertIn(
            "/multi_agv/three_car_unloaded_pretest_result_code", run)
        self.assertIn("motion node exited without a valid result code", run)
        self.assertIn("target_progress: 1.00", (
            PACKAGE / "config" /
            "three_car_unloaded_bounded_pretest.yaml"
        ).read_text(encoding="utf-8"))

    def test_three_car_straight_entry_is_independent_and_records_imu(self):
        path = (
            PACKAGE / "config" / "path_straight_1m.yaml"
        ).read_text(encoding="utf-8")
        localization = (
            PACKAGE / "config" / "localization_odom_straight.yaml"
        ).read_text(encoding="utf-8")
        launch = (
            PACKAGE / "launch" /
            "three_car_unloaded_straight_pretest.launch"
        ).read_text(encoding="utf-8")
        estimator_launch = (
            PACKAGE / "launch" /
            "odom_state_estimator_straight.launch"
        ).read_text(encoding="utf-8")
        wrapper = (
            PACKAGE / "scripts" /
            "run_three_car_unloaded_straight_pretest.sh"
        ).read_text(encoding="utf-8")
        straight_config = (
            PACKAGE / "config" /
            "three_car_unloaded_straight_pretest.yaml"
        ).read_text(encoding="utf-8")
        runner = (
            PACKAGE / "scripts" / "run_three_car_unloaded_pretest.sh"
        ).read_text(encoding="utf-8")

        self.assertIn("amplitude: 0.0", path)
        self.assertIn("longitudinal_length: 1.0", path)
        self.assertEqual(localization.count("yaw: 0.0}"), 6)
        self.assertIn("maximum_state_age: 0.20", localization)
        self.assertIn("maximum_sync_slop: 0.02", localization)
        self.assertIn("odom_state_estimator_straight.launch", launch)
        self.assertIn("localization_odom_straight.yaml", estimator_launch)
        self.assertIn("path_straight_1m.yaml", launch)
        self.assertIn(
            "three_car_unloaded_straight_pretest.yaml", launch)
        self.assertIn("AGV_THREE_CAR_PATH_MODE=straight", wrapper)
        self.assertIn("load_path_speed: 0.08", straight_config)
        self.assertIn("maximum_wheel_linear_velocity: 0.11",
                      straight_config)
        self.assertIn("maximum_motion_time: 18.0", straight_config)
        self.assertIn('motion_launch=\"three_car_unloaded_straight_pretest.launch\"',
                      runner)
        self.assertIn("reset_all_odometry", runner)
        self.assertIn("post_gate_odometry_reset=true", runner)
        for topic in ("/agv1/imu", "/agv2/imu", "/agv3/imu"):
            self.assertIn(topic, runner)


if __name__ == "__main__":
    unittest.main()

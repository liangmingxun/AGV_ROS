#!/usr/bin/env python3
import pathlib
import unittest
import xml.etree.ElementTree as ET


PACKAGE = pathlib.Path(__file__).resolve().parents[1]
SOURCE_ROOT = PACKAGE.parent


class BringupStaticTest(unittest.TestCase):
    def test_zero_odometry_reconstructs_30cm_supports(self):
        import math
        import yaml

        config = PACKAGE / "config"
        supports = yaml.safe_load((config / "support_geometry.yaml").read_text())[
            "support_geometry"]["supports"]
        for filename in ("localization_odom.yaml", "localization_odom_straight.yaml"):
            robots = yaml.safe_load((config / filename).read_text())["robots"]
            points = []
            expected_offsets = (-0.01783, 0.09908, 0.09908)
            for index, (robot, support) in enumerate(zip(robots, supports)):
                self.assertEqual(robot["robot_id"], support["robot_id"])
                base = robot["base_to_support"]
                self.assertEqual(
                    base,
                    {"x": expected_offsets[index], "y": 0.0, "yaw": 0.0})
                pose = robot["world_to_odom"]
                c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
                x = pose["x"] + c * base["x"] - s * base["y"]
                y = pose["y"] + s * base["x"] + c * base["y"]
                self.assertAlmostEqual(c*x + s*y, support["q_tangent"], places=12)
                self.assertAlmostEqual(-s*x + c*y, support["q_normal"], places=12)
                points.append((x, y))
            for i, j in ((0, 1), (0, 2), (1, 2)):
                self.assertAlmostEqual(math.dist(points[i], points[j]), 0.30, places=12)
            for axis in (0, 1):
                self.assertAlmostEqual(sum(p[axis] for p in points)/3, 0.0, places=12)

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
        self.assertIn("extrinsics_frozen: true", config)
        self.assertEqual(config.count("enabled: true"), 3)
        self.assertEqual(config.count("enabled: false"), 1)
        self.assertIn("if (!stream.enabled)", adapter)
        self.assertIn("require_confidence: true", config)
        self.assertEqual(config.count("tag_to_target_x:"), 4)
        self.assertEqual(config.count("tag_to_target_y: 0.0972"), 2)
        self.assertIn("tag_to_target_y: -0.13274", config)
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

    def test_serial_008_speed_scale_is_consistent_and_operator_authorized(self):
        runtime = (
            PACKAGE / "config" / "formal_serial_m1_r1_runtime.yaml"
        ).read_text(encoding="utf-8")
        terminal_path = (
            PACKAGE / "config" / "path_s_curve_terminal_straight.yaml"
        ).read_text(encoding="utf-8")
        m1 = (
            PACKAGE / "config" / "exp2a_M1_serial_008.yaml"
        ).read_text(encoding="utf-8")
        m2a = (
            PACKAGE / "config" / "exp2a_M2a_serial_008.yaml"
        ).read_text(encoding="utf-8")
        serial_launch = (
            PACKAGE / "launch" / "formal_serial_m1_r1.launch"
        ).read_text(encoding="utf-8")
        chassis_launch = (
            PACKAGE / "launch" / "chassis_single.launch"
        ).read_text(encoding="utf-8")
        chassis_entry = (
            PACKAGE / "scripts" / "start_three_car_chassis.sh"
        ).read_text(encoding="utf-8")
        experiment_entry = (
            PACKAGE / "scripts" / "run_m1_r1_serial_unloaded.sh"
        ).read_text(encoding="utf-8")
        authorization = (
            PACKAGE / "config" / "formal_serial_m1_r1_authorization.yaml"
        ).read_text(encoding="utf-8")
        formal_source = (
            SOURCE_ROOT / "multi_agv_control" / "src" /
            "formal_fake_algorithm_node.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn(
            "PILOT_AUTHORIZED_0p15_APPLIED_0p18_PRELIMIT_ABORT",
            runtime)
        self.assertIn("command_publication_authorized: false", runtime)
        self.assertIn("serial_execution_authorized: true", runtime)
        self.assertEqual(
            authorization.count("hardware_execution_authorized: true"), 2)
        self.assertNotIn("hardware_execution_authorized: false", authorization)
        self.assertIn("emergency_abort_limit: 0.18", runtime)
        self.assertIn("minimum_battery_voltage: 9.5", runtime)
        self.assertIn("minimum_battery_voltage_duration: 0.5", runtime)
        self.assertIn("velocity: 0.08", runtime)
        self.assertIn("velocity: [0.08, 0.08, 0.08]", runtime)
        self.assertIn("target_progress: 1.2042352285", runtime)
        self.assertIn(
            "model: sine_single_period_with_straight_exit", terminal_path)
        self.assertIn("longitudinal_length: 1.0", terminal_path)
        self.assertIn("exit_straight_length: 0.18", terminal_path)
        self.assertIn("velocity_lower_bound: -0.15", runtime)
        self.assertIn("velocity_upper_bound: 0.58", runtime)
        for config in (m1, m2a):
            self.assertIn("physical_upper: [0.105, 0.105, 0.105]", config)
            self.assertIn("inner_margin: [0.005, 0.005, 0.005]", config)
            self.assertIn("initial_upper: [0.095, 0.095, 0.095]", config)
            self.assertIn("nominal_upper: [0.095, 0.095, 0.095]", config)
        self.assertIn("exp2a_M1_serial_008.yaml", serial_launch)
        self.assertIn("path_s_curve_terminal_straight.yaml", serial_launch)
        self.assertNotIn("capability_mapping.yaml", serial_launch)
        self.assertIn(
            "wheel_separation: [0.135484339, 0.139284482, 0.136802843]",
            runtime)
        self.assertEqual(runtime.count("{x: -0.01783, y: 0.0}"), 1)
        self.assertEqual(runtime.count("{x: 0.09908, y: 0.0}"), 2)
        self.assertIn("confirm_unloaded_30cm_fixture: false", runtime)
        self.assertIn('formal_available_wheel_limit" default="0.08"',
                      chassis_launch)
        self.assertIn("--confirm-formal-0p15-wheel-envelope", chassis_entry)
        self.assertIn('formal_wheel_limit="0.15"', chassis_entry)
        self.assertIn("pre-limit demand abort threshold", experiment_entry)
        self.assertIn("--confirm-unloaded-30cm-fixture", experiment_entry)
        self.assertIn(
            'path_config="src/multi_agv_bringup/config/'
            'path_s_curve_terminal_straight.yaml"', experiment_entry)
        self.assertIn('evaluation_target:="$target_progress"',
                      experiment_entry)
        self.assertIn('nominal_common_velocity="$(awk', experiment_entry)
        self.assertIn('/^[[:space:]]*leader:/', experiment_entry)
        self.assertIn(
            'nominal_common_velocity:="$nominal_common_velocity"',
            experiment_entry)
        self.assertGreaterEqual(
            experiment_entry.count('upper_config:="${workspace}/${upper_config}"'),
            2)
        self.assertGreaterEqual(
            experiment_entry.count('lower_config:="${workspace}/${lower_config}"'),
            2)
        self.assertNotIn("nominal_common_velocity:=0.08", experiment_entry)
        self.assertNotIn("plot_paper_experiments.py", experiment_entry)
        self.assertIn("stop_runtime_nodes", experiment_entry)
        self.assertIn("PHYSICAL_TASK_STATUS=PASSED", experiment_entry)
        self.assertIn("POSTPROCESS_STATUS=PASSED", experiment_entry)
        self.assertIn("POSTPROCESS_STATUS=FAILED", experiment_entry)
        self.assertIn("bag and run directory are retained", experiment_entry)
        self.assertIn("Offline retry:", experiment_entry)
        self.assertGreater(
            experiment_entry.index("process_experiment_run.py"),
            experiment_entry.index("rosservice call /experiment_recorder/stop"))
        self.assertGreater(
            experiment_entry.index("process_experiment_run.py"),
            experiment_entry.index("stop_runtime_nodes"))
        self.assertIn("fleet_wheel_scale_ = 1.0", formal_source)
        self.assertIn("all three pre-limit demands", formal_source)
        self.assertIn("low_battery_since_", formal_source)
        self.assertIn(
            "low_voltage_duration >= minimum_battery_voltage_duration_",
            formal_source)

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
            1: "192.168.6.101",
            2: "192.168.6.102",
            3: "192.168.6.103",
        }
        expected_serial_devices = {
            1: "/dev/chassis_driver",
            2: "/dev/ttyACM0",
            3: "/dev/chassis_driver",
        }
        expected_wheel_calibration = {
            1: ("0.135484339", "1.129384768", "1.115373526"),
            2: ("0.139284482", "1.091612663", "1.102973507"),
            3: ("0.136802843", "1.134911374", "1.137011300"),
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
            self.assertIn(
                "serial_device: {}".format(expected_serial_devices[index]),
                text)
            track, left_scale, right_scale = expected_wheel_calibration[index]
            self.assertIn("wheel_separation: {}".format(track), text)
            self.assertIn(
                "wheel_feedback_scale_left: {}".format(left_scale), text)
            self.assertIn(
                "wheel_feedback_scale_right: {}".format(right_scale), text)

        chassis_entry = (
            PACKAGE / "scripts" / "start_three_car_chassis.sh"
        ).read_text(encoding="utf-8")
        network_entry = (
            PACKAGE / "scripts" / "setup_ros_network.sh"
        ).read_text(encoding="utf-8")
        for address in expected_host_ips.values():
            self.assertIn(address, chassis_entry)
        self.assertIn('192.168.6.101', network_entry)
        self.assertNotIn('10.134.37.', chassis_entry + network_entry)

    def test_support_offsets_are_launch_calibration_arguments(self):
        expected_launch_offsets = {
            "car1_master.launch": "-0.01783",
            "car2_client.launch": "0.09908",
            "car3_client.launch": "0.09908",
            "chassis_single.launch": "-0.01783",
        }
        for launch_name, expected_offset in expected_launch_offsets.items():
            text = (PACKAGE / "launch" / launch_name).read_text(encoding="utf-8")
            for marker in ("support_x", "support_y", "support_z"):
                self.assertIn(marker, text)
            self.assertIn(
                'name="support_x" default="{}"'.format(expected_offset), text)
            self.assertIn('name="support_z" default="0.0"', text)

        localization = (PACKAGE / "config" / "localization_odom.yaml").read_text(
            encoding="utf-8")
        self.assertEqual(localization.count(
            "base_to_support: {x: -0.01783, y: 0.0, yaw: 0.0}"), 1)
        self.assertEqual(localization.count(
            "base_to_support: {x: 0.09908, y: 0.0, yaw: 0.0}"), 2)
        for transform in (
                "x: 0.182252857360210, y: 0.057256423777858, "
                "yaw: 0.304395797364615",
                "x: -0.222103903348248, y: 0.087452054972220, "
                "yaw: 0.304395797364615",
                "x: -0.132188862271489, y: -0.198756409941319, "
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
        self.assertIn('if (localization_mode_ == "fused")',
                      estimator_source)
        self.assertIn("pose_transport_hints", estimator_source)
        self.assertNotIn(".unreliable()", estimator_source)
        self.assertIn(
            "static_cast<std::uint32_t>(synchronization_queue_size_)",
            estimator_source)
        self.assertIn(
            "Cooperative odometry is state, not disposable telemetry",
            estimator_source)

        tracker = (
            PACKAGE / "config" / "pretest_constant_reference.yaml"
        ).read_text(encoding="utf-8")
        self.assertEqual(tracker.count("{x: -0.01783, y: 0.0}"), 1)
        self.assertEqual(tracker.count("{x: 0.09908, y: 0.0}"), 2)
        self.assertIn("chassis_reference_samples: 10001", tracker)
        capability = (
            PACKAGE / "config" / "capability_mapping.yaml"
        ).read_text(encoding="utf-8")
        self.assertEqual(capability.count("{x: -0.01783, y: 0.0}"), 1)
        self.assertEqual(capability.count("{x: 0.09908, y: 0.0}"), 2)
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
                "q_tangent: 0.1732050807568877",
                "role: left_rear",
                "q_tangent: -0.0866025403784439",
                "q_normal: 0.1500000000000000",
                "role: right_rear",
                "q_normal: -0.1500000000000000"):
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

    def test_single_car_s_pretest_is_generic_isolated_and_bounded(self):
        launch = (
            PACKAGE / "launch" / "single_car_s_pretest.launch"
        ).read_text(encoding="utf-8")
        capture = (
            PACKAGE / "launch" / "single_car_camera_s_capture.launch"
        ).read_text(encoding="utf-8")
        config = (
            PACKAGE / "config" / "single_car_s_pretest.yaml"
        ).read_text(encoding="utf-8")
        self.assertIn('type="single_car_s_pretest_node"', launch)
        self.assertIn('name="robot_index" default="0"', launch)
        self.assertIn('name="robot_index"', capture)
        self.assertIn("/camera/world/agv$(arg robot_index)_tag_pose", capture)
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
        self.assertEqual(config.count("base_to_support_x: 0.09908"), 2)
        self.assertIn("lateral_gain: 3.0", config)
        self.assertIn("heading_gain: 2.5", config)
        self.assertIn(
            "profile: robot1_camera_wheel_turn_candidate_v1", config)
        self.assertIn("wheel_separation: 0.135484339", config)
        self.assertIn(
            "angular_feedforward_scale_positive: 0.803566413", config)
        self.assertIn(
            "angular_feedforward_scale_negative: 0.845593913", config)
        self.assertIn(
            "profile: robot2_camera_s_frozen_v1", config)
        self.assertIn("wheel_separation: 0.139284482", config)
        self.assertIn(
            "angular_feedforward_scale_positive: 0.780810219", config)
        self.assertIn(
            "angular_feedforward_scale_negative: 0.918767785", config)
        self.assertIn("hard_stop_consecutive_samples: 5", config)
        self.assertIn(
            "profile: robot3_camera_s_frozen_v1", config)
        self.assertIn("lateral_gain: 4.0", config)
        self.assertIn("heading_gain: 3.0", config)
        self.assertIn("curvature_preview_seconds: 0.05", config)
        self.assertIn("wheel_separation: 0.136802843", config)
        self.assertIn(
            "angular_feedforward_scale_positive: 0.850009371", config)
        self.assertIn(
            "angular_feedforward_scale_negative: 0.883228287", config)
        self.assertIn("hard_stop_speed: 0.105", config)
        self.assertIn("sustained_speed: 0.10", config)
        self.assertIn("sustained_duration: 0.10", config)
        self.assertIn("post_stop_record_seconds: 2.0", config)
        self.assertIn("hard_stop_speed: 0.09", config)
        self.assertIn("hard_stop_consecutive_samples: 2", config)
        self.assertIn("emergency_stop_speed: 0.12", config)

        runner = (SOURCE_ROOT.parent /
                  "run_single_car_camera_s_closed_loop.sh").read_text(
                      encoding="utf-8")
        self.assertIn('--robot-index {1|2|3}', runner)
        self.assertIn('ROBOT1_IP="192.168.6.101"', runner)
        self.assertIn(
            'TRACKER_PROFILE="robot1_camera_wheel_turn_candidate_v1"',
            runner)
        self.assertIn(
            'TRACKER_PROFILE="robot2_camera_s_frozen_v1"',
            runner)
        self.assertIn(
            'TRACKER_FEEDFORWARD_SCALE_POSITIVE="0.780810219"', runner)
        self.assertIn(
            'TRACKER_FEEDFORWARD_SCALE_NEGATIVE="0.918767785"', runner)
        self.assertIn('TRACKER_WHEEL_SEPARATION="0.139284482"', runner)
        self.assertIn(
            'WHEEL_HARD_STOP_CONSECUTIVE_SAMPLES="5"', runner)
        self.assertIn(
            'TRACKER_PROFILE="robot3_camera_s_frozen_v1"',
            runner)
        self.assertIn(
            'TRACKER_FEEDFORWARD_SCALE_POSITIVE="0.850009371"', runner)
        self.assertIn(
            'TRACKER_FEEDFORWARD_SCALE_NEGATIVE="0.883228287"', runner)
        self.assertIn('TRACKER_WHEEL_SEPARATION="0.136802843"', runner)
        freeze = (PACKAGE / "config" /
                  "robot2_camera_s_freeze_v1.yaml").read_text(
                      encoding="utf-8")
        self.assertIn("freeze_id: robot2_camera_s_frozen_v1", freeze)
        self.assertIn("status: frozen", freeze)
        for run_id in ("20260810_202045", "20260810_202854",
                       "20260810_203055"):
            self.assertIn(run_id, freeze)
        robot3_freeze = (PACKAGE / "config" /
                         "robot3_camera_s_freeze_v1.yaml").read_text(
                             encoding="utf-8")
        self.assertIn("freeze_id: robot3_camera_s_frozen_v1",
                      robot3_freeze)
        self.assertIn("status: frozen", robot3_freeze)
        for run_id in ("20260811_115644", "20260811_133644",
                       "20260811_133735"):
            self.assertIn(run_id, robot3_freeze)
        for index, mode in ((1, "local"), (2, "remote"), (3, "remote")):
            wrapper = (SOURCE_ROOT.parent /
                       f"run_robot{index}_camera_s_closed_loop.sh").read_text(
                           encoding="utf-8")
            self.assertIn(f"--robot-index {index}", wrapper)
            self.assertIn(f"--chassis-mode {mode}", wrapper)

        calibration = (SOURCE_ROOT.parent /
                       "run_single_car_camera_wheel_calibration.sh").read_text(
                           encoding="utf-8")
        calibrator = (PACKAGE / "scripts" /
                      "camera_wheel_turn_calibration.py").read_text(
                          encoding="utf-8")
        self.assertIn("--robot-index {1|2|3}", calibration)
        self.assertIn("--confirm-test-area-clear", calibration)
        self.assertIn("--confirm-wheels-on-floor", calibration)
        self.assertIn("parameters_automatically_applied=false", calibration)
        self.assertIn("--cycles 3", calibration)
        self.assertIn("--straight-duration 7.0", calibration)
        self.assertIn("--arc-duration 6.0", calibration)
        for index in (1, 2, 3):
            wrapper = (SOURCE_ROOT.parent /
                       f"run_robot{index}_camera_wheel_calibration.sh").read_text(
                           encoding="utf-8")
            self.assertIn(f"--robot-index {index}", wrapper)
        robot1_calibration = (SOURCE_ROOT.parent /
                              "run_robot1_camera_wheel_calibration.sh").read_text(
                                  encoding="utf-8")
        self.assertIn("car1_master.launch", robot1_calibration)
        self.assertIn("trap cleanup EXIT INT TERM HUP", robot1_calibration)
        self.assertIn("REVIEW ONLY", calibrator)
        self.assertIn("component_status", calibrator)
        self.assertIn("geometry_outlier_cycles", calibrator)
        self.assertIn("wheel_feedback_scale_left_candidate", calibrator)
        self.assertIn("angular_feedforward_scale_positive_candidate", calibrator)

        environment = (SOURCE_ROOT.parent / "setup_robot_ros.sh").read_text(
            encoding="utf-8")
        camera_entry = (SOURCE_ROOT.parent /
                        "start_camera_formal.sh").read_text(encoding="utf-8")
        for address in ("192.168.6.101", "192.168.6.102",
                        "192.168.6.103"):
            self.assertIn(address, environment)
        self.assertIn('source "${SCRIPT_DIR}/setup_robot_ros.sh" 1',
                      camera_entry)
        self.assertIn("calibration_authorized:=true", camera_entry)

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
        shared_adapter = (
            SOURCE_ROOT / "multi_agv_control" / "src" /
            "planar_support_tracker.cpp"
        ).read_text(encoding="utf-8")

        self.assertIn(
            'type="three_car_unloaded_bounded_pretest_node"', launch)
        for marker in (
                'name="enable_commands" default="false"',
                'name="confirm_readonly_gate_passed" default="false"',
                'name="confirm_test_area_clear" default="false"',
                'name="confirm_wheels_on_floor" default="false"',
                'name="confirm_unloaded_30cm_fixture" default="false"'):
            self.assertIn(marker, launch)
        self.assertNotIn(
            'file="$(find multi_agv_bringup)/launch/'
            'central_odom_pretest.launch"', launch)

        for marker in (
                "hardware_execution_authorized: true",
                "load_path_speed: 0.05",
                "start_delay_seconds: 3.0",
                "startup_ramp_seconds: 1.0",
                "target_progress: 1.00",
                "validation_profile: camera_fused_s_1p00",
                "wheel_separation: [0.135484339, 0.139284482, 0.136802843]",
                "angular_feedforward_scale_positive:",
                "angular_feedforward_scale_negative:",
                "lateral_gain_per_robot: [4.0, 3.0, 4.0]",
                "heading_gain_per_robot: [3.0, 2.5, 3.0]",
                "angular_feedforward_scale_positive: [0.78, 0.780810219, 0.850009371]",
                "angular_feedforward_scale_negative: [0.89, 0.918767785, 0.883228287]",
                "formation_longitudinal_gain: 0.25",
                "formation_lateral_gain: 0.50",
                "formation_heading_gain: 0.50",
                "curvature_preview_seconds_positive: [0.05, 0.0, 0.05]",
                "curvature_preview_seconds_negative: [0.05, 0.0, 0.05]",
                "required_command_subscribers: 2",
                "readiness_stable_samples: 20",
                "minimum_battery_voltage: 10.0",
                "maximum_feedback_receive_age: 0.25",
                "maximum_serial_feedback_age: 0.25",
                "maximum_stamp_spread: 0.02",
                "maximum_wheel_linear_velocity: 0.08"):
            self.assertIn(marker, config)
        self.assertIn("minimum_wheel_command_scale: 0.75", config)
        for marker in (
                "rejectCompetingPublishers",
                "publishRepeatedStop",
                "waitForStopped",
                "virtual load pose or path state is invalid",
                "control loop overrun reported",
                "remote STM32 timestamp diagnostic only",
                "local receive and packet-progress gates remain authoritative",
                "chassis feedback receive age",
                "STM32 packet sequence age",
                "curvature_preview_seconds_positive_",
                "curvature_preview_seconds_negative_",
                "formation_longitudinal_gain_",
                "formation_lateral_gain_",
                "formation_heading_gain_",
                "limitTrackingCommands",
                "Uniform three-car wheel-command scaling active",
                "startup_ramp_seconds_",
                "all six wheels must be stopped before motion"):
            self.assertIn(marker, node)
        self.assertIn("FleetPlanarExecutionAdapter", node)
        for marker in (
                "mean_world_error",
                "relative_heading_error",
                "angular_feedforward_scale_positive",
                "formation_lateral_gain"):
            self.assertIn(marker, shared_adapter)
        self.assertIn(
            "feedback_transport_hints.reliable().tcpNoDelay()", node)

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
        self.assertIn("--confirm-unloaded-30cm-fixture", run)
        self.assertIn("check_three_car_readonly_gate.py", run)
        self.assertIn("require_three_car_motion_gate.sh", run)
        self.assertIn("--require-valid-state", motion_gate)
        self.assertIn("PASSED ${passes} CONSECUTIVE WINDOW", motion_gate)
        self.assertNotIn("/reset_odometry", run)
        self.assertIn("rosbag record", run)
        self.assertIn(
            "camera_role=absolute_position_and_heading_closed_loop_authority",
            run)
        self.assertIn("virtual_load=rigid_fit_from_three_fused_support_centres",
                      run)
        self.assertIn("/pose_provider/agv1/base_pose_raw", run)
        self.assertIn("analyze_three_car_cooperative_bag.py", run)
        self.assertIn(
            "rosparam delete /three_car_unloaded_bounded_pretest", run)
        self.assertIn(
            "parameter_snapshot_timing=after_motion_node_parameter_load_before_start_delay",
            run)
        self.assertIn(
            "curvature_preview_seconds_negative", run)
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
        self.assertNotIn("odom_state_estimator_straight.launch", launch)
        self.assertIn("localization_odom_straight.yaml", estimator_launch)
        self.assertIn("path_straight_1m.yaml", launch)
        self.assertIn(
            "three_car_unloaded_straight_pretest.yaml", launch)
        self.assertIn("AGV_THREE_CAR_PATH_MODE=straight", wrapper)
        self.assertIn("load_path_speed: 0.03", straight_config)
        self.assertIn("target_progress: 0.30", straight_config)
        self.assertIn("maximum_wheel_linear_velocity: 0.06",
                      straight_config)
        self.assertIn("maximum_motion_time: 15.0", straight_config)
        self.assertIn('motion_launch=\"three_car_unloaded_straight_pretest.launch\"',
                      runner)
        self.assertNotIn("reset_all_odometry", runner)
        self.assertIn("post_gate_odometry_reset=false", runner)
        self.assertIn("camera_fused_virtual_load_state_estimator.launch",
                      runner)
        for topic in ("/agv1/imu", "/agv2/imu", "/agv3/imu"):
            self.assertIn(topic, runner)

    def test_three_car_clockwise_half_metre_circle_has_smooth_entry(self):
        path = (PACKAGE / "config" /
                "path_circle_r0p5_cw_smooth.yaml").read_text(
            encoding="utf-8")
        override = (
            PACKAGE / "config" / "three_car_unloaded_circle_pretest.yaml"
        ).read_text(encoding="utf-8")
        launch = (
            PACKAGE / "launch" / "three_car_unloaded_circle_pretest.launch"
        ).read_text(encoding="utf-8")
        runner = (
            PACKAGE / "scripts" / "run_three_car_unloaded_pretest.sh"
        ).read_text(encoding="utf-8")
        wrapper = (SOURCE_ROOT.parent /
                   "run_three_car_cooperative_circle_validation.sh").read_text(
                       encoding="utf-8")

        self.assertIn("model: circle_smooth_entry", path)
        self.assertIn("circle_radius: 0.5", path)
        self.assertIn("circle_direction: -1.0", path)
        self.assertIn("entry_straight_length: 0.20", path)
        self.assertIn("curvature_ramp_length: 0.40", path)
        self.assertIn("longitudinal_length: 3.741592653589793", path)
        self.assertIn("camera_fused_circle_r0p5_cw_smooth", override)
        self.assertIn("load_path_speed: 0.05", override)
        self.assertIn("target_progress: 3.741592653589793", override)
        self.assertIn("maximum_motion_time: 90.0", override)
        self.assertIn("path_circle_r0p5_cw_smooth.yaml", launch)
        self.assertIn("three_car_unloaded_circle_pretest.yaml", launch)
        self.assertIn("AGV_THREE_CAR_PATH_MODE=circle", wrapper)
        self.assertIn("--confirm-circle-area-clear", runner)
        self.assertIn(
            'motion_launch="three_car_unloaded_circle_pretest.launch"',
            runner)

    def test_three_car_closed_loop_uses_fused_virtual_load(self):
        launch = (
            PACKAGE / "launch" /
            "camera_fused_virtual_load_state_estimator.launch"
        ).read_text(encoding="utf-8")
        config = (
            PACKAGE / "config" /
            "localization_camera_three_car_closed_loop.yaml"
        ).read_text(encoding="utf-8")
        estimator = (
            SOURCE_ROOT / "multi_agv_control" / "src" /
            "path_state_estimator_node.cpp"
        ).read_text(encoding="utf-8")
        self.assertIn("localization_camera.yaml", launch)
        self.assertIn("localization_camera_three_car_closed_loop.yaml",
                      launch)
        self.assertIn("derive_virtual_load_from_robots: true", config)
        self.assertIn("auto_anchor_from_robot_poses: true", config)
        self.assertIn("output_frame: three_car_path", config)
        self.assertIn("maximum_state_age: 0.22", config)
        self.assertIn("maximum_sync_slop: 0.02", config)
        self.assertIn("synchronization_queue_size: 64", config)
        self.assertIn("localization_mode_ == \"fused\"", estimator)
        self.assertIn("pose_transport_hints", estimator)
        self.assertNotIn(".unreliable()", estimator)
        self.assertNotIn(".maxDatagramSize(1400)", estimator)
        self.assertIn(
            "static_cast<std::uint32_t>(synchronization_queue_size_)",
            estimator)
        self.assertIn("initializeCameraPathAnchor", estimator)
        self.assertIn("fitRigidLoadPose", estimator)

    def test_camera_formation_initialization_is_sequential_and_bounded(self):
        launch = (
            PACKAGE / "launch" / "three_car_camera_formation_init.launch"
        ).read_text(encoding="utf-8")
        config = (
            PACKAGE / "config" / "three_car_camera_formation_init.yaml"
        ).read_text(encoding="utf-8")
        node = (
            PACKAGE / "scripts" /
            "initialize_three_car_camera_formation.py"
        ).read_text(encoding="utf-8")
        runner = (
            SOURCE_ROOT.parent / "run_three_car_camera_formation_init.sh"
        ).read_text(encoding="utf-8")
        runner_30cm = (
            SOURCE_ROOT.parent /
            "run_three_car_camera_formation_init_30cm.sh"
        ).read_text(encoding="utf-8")

        for marker in (
                'name="confirm_test_area_clear" default="false"',
                'name="confirm_wheels_on_floor" default="false"',
                'name="confirm_automatic_formation" default="false"'):
            self.assertIn(marker, launch)
        for marker in (
                "side_length: 0.30",
                "base_to_support_x: [-0.01783, 0.09908, 0.09908]",
                "maximum_linear_speed: 0.025",
                "near_target_speed: 0.010",
                "docking_speed: 0.005",
                "final_side_tolerance: 0.030",
                "position_tolerance: 0.020",
                "refinement_position_tolerance: 0.012",
                "maximum_refinement_passes: 2",
                "final_heading_recovery_distance: 0.030",
                "final_heading_settle_position_margin: 0.005",
                "maximum_no_progress_seconds: 12.0",
                "maximum_wheel_speed: 0.05",
                "maximum_initial_target_distance: 0.65",
                "minimum_robot_separation: 0.22",
                "minimum_battery_voltage: 10.0",
                "minimum_battery_voltage_duration: 0.5",
                "required_command_subscribers: 1"):
            self.assertIn(marker, config)
        for marker in (
                "_reject_competing_publishers",
                "all six wheels must be stopped before initialization",
                "battery remained below the bound",
                "_refine_final_triangle",
                "FORMATION_REFINE",
                "for index in (1, 2)",
                "_check_separation",
                "base_link separation",
                "_check_stationary",
                'phase + "_APPROACH_REVERSE"',
                'phase + "_FINAL_HEADING"',
                "latched final-heading phase",
                'phase + "_POSITION_RECOVERY"',
                "failed to make position progress",
                "all-six-wheel stop confirmation failed",
                "FORMATION_CONFIRMED"):
            self.assertIn(marker, node)
        self.assertIn("--confirm-automatic-formation", runner)
        self.assertIn("0.30 m equilateral triangle", runner)
        self.assertNotIn("rosbag record", runner)
        self.assertNotIn("experiment_data", runner)
        self.assertIn('/pose_provider/agv${index}/base_pose_fused', runner)
        self.assertIn("/multi_agv/formation_init/result_code", runner)
        self.assertIn("side_length:", runner_30cm)
        self.assertIn("0\\.30", runner_30cm)
        self.assertIn("run_three_car_camera_formation_init.sh", runner_30cm)

    def test_robot3_single_wheel_check_is_lifted_bounded_and_recorded(self):
        runner = (
            SOURCE_ROOT.parent / "run_robot3_single_wheel_check.sh"
        ).read_text(encoding="utf-8")
        node = (
            PACKAGE / "scripts" / "robot3_single_wheel_check.py"
        ).read_text(encoding="utf-8")
        for marker in (
                "--confirm-wheels-lifted",
                "--confirm-emergency-stop-ready",
                "--wheel",
                "formal_available_wheel_limit",
                'limit" != "0.08"',
                "rosbag record",
                "chassis_command.csv",
                "chassis_feedback.csv"):
            self.assertIn(marker, runner)
        for marker in (
                "SPEED = 0.01",
                "DURATION = 0.5",
                "OVERSPEED = 0.03",
                "LEFT_FORWARD",
                "RIGHT_FORWARD",
                "LEFT_REVERSE",
                "RIGHT_REVERSE",
                'choices=("left", "right", "both")',
                "inactive wheel moved unexpectedly",
                "SINGLE_WHEEL_STOP"):
            self.assertIn(marker, node)

    def test_robot3_imu_static_capture_is_stationary_and_recorded(self):
        runner = (
            SOURCE_ROOT.parent / "run_robot3_imu_static_capture.sh"
        ).read_text(encoding="utf-8")
        for marker in (
                "duration_seconds=120",
                "--confirm-robot-stationary",
                "--confirm-no-motion-command",
                "--confirm-emergency-stop-ready",
                "starting a private roscore",
                "car3_client.launch",
                "formal_available_wheel_limit:=0.08",
                "/agv3/chassis_controller",
                "has_publisher /agv3/chassis_command",
                "wheel velocity exceeds 0.01 m/s",
                "rosbag record",
                "/agv3/imu",
                "/agv3/chassis_feedback",
                "imu_static_summary.txt",
                "VALID_STATIONARY_CAPTURE"):
            self.assertIn(marker, runner)


if __name__ == "__main__":
    unittest.main()

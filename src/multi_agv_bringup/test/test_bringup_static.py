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
                           "command_timeout_seconds: 0.20",
                           "sensor_feedback_timeout_seconds: 0.15"):
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
        self.assertNotIn("central_odom_pretest", launch)
        self.assertIn("amplitude: 0.05", config)
        self.assertIn("longitudinal_length: 1.0", config)
        self.assertIn("arc_length_speed: 0.05", config)
        self.assertIn("maximum_wheel_linear_velocity: 0.08", config)
        self.assertIn("base_to_support_x: -0.01783", config)

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
                "minimum_battery_voltage: 10.8",
                "maximum_serial_feedback_age: 0.15",
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
                "all six wheels must be stopped before motion"):
            self.assertIn(marker, node)

    def test_three_car_host_and_orchestration_scripts_are_safety_gated(self):
        start = (
            PACKAGE / "scripts" / "start_three_car_chassis.sh"
        ).read_text(encoding="utf-8")
        run = (
            PACKAGE / "scripts" / "run_three_car_unloaded_pretest.sh"
        ).read_text(encoding="utf-8")
        self.assertIn("working tree is not clean", start)
        self.assertIn("/deployment/git_sha", start)
        self.assertIn("serial device", start)
        self.assertIn("--confirm-area-clear", run)
        self.assertIn("--confirm-wheels-on-floor", run)
        self.assertIn("--confirm-unloaded-40cm-fixture", run)
        self.assertIn("check_three_car_readonly_gate.py", run)
        self.assertEqual(run.count("--require-valid-state"), 2)
        self.assertIn("/reset_odometry", run)
        self.assertIn("rosbag record", run)
        self.assertIn("target_progress: 1.00", (
            PACKAGE / "config" /
            "three_car_unloaded_bounded_pretest.yaml"
        ).read_text(encoding="utf-8"))


if __name__ == "__main__":
    unittest.main()

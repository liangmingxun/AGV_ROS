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
                           "serial_startup_timeout_seconds:"):
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


if __name__ == "__main__":
    unittest.main()

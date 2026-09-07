#!/usr/bin/env python3

import csv
import tempfile
import unittest
from pathlib import Path

from multi_agv_analysis.io_utils import (
    atomic_dump_json, load_yaml, write_csv)
from multi_agv_analysis.paper_pipeline import export_views, plot_run
from multi_agv_analysis.publication_plots import (
    plot_experiment1, plot_experiment2a, plot_experiment2b,
    plot_experiment3, write_statistics)


SCRIPT = Path(__file__).parents[1] / "scripts" / "plot_paper_experiments.py"


class PaperPipelineTest(unittest.TestCase):
    def test_cli_exposes_stable_numbered_test_folders(self):
        script = SCRIPT.read_text(encoding="utf-8")
        for folder in (
                "01_M1_R1_complete_method",
                "02a_M1_vs_M2a_capability_derating",
                "02b_M1_vs_M2b_complete_method",
                "03_R1_R2_R3_disturbance_rejection"):
            self.assertIn(folder, script)
        self.assertIn("--numbered-folders", script)

    @staticmethod
    def _publication_fixture(path):
        rows = []
        for index in range(16):
            row = {
                "stamp": 10.0 + index * 0.1,
                "evaluation_active": True,
                "load_pose_x": index * 0.01,
                "load_pose_y": 0.01,
                "load_x_reference": index * 0.01,
                "load_y_reference": 0.0,
                "load_s_reference": index * 0.01,
                "load_velocity_reference": 0.08,
                "common_boundary_upper": 0.09,
                "public_reference_velocity": 0.08,
            }
            for robot in range(1, 4):
                row.update({
                    "agv{}_mapped_path_velocity_upper".format(robot):
                        0.12 - 0.005 * robot,
                    "agv{}_boundary_upper".format(robot): 0.095,
                    "agv{}_support_pose_x".format(robot):
                        index * 0.01 + 0.001 * robot,
                    "agv{}_support_pose_y".format(robot): 0.01 * robot,
                    "agv{}_support_reference_x".format(robot):
                        index * 0.01,
                    "agv{}_support_reference_y".format(robot):
                        0.01 * robot,
                    "agv{}_composite_error".format(robot):
                        0.001 * robot,
                    "agv{}_disturbance_estimate".format(robot):
                        0.002 * robot,
                    "agv{}_channel_input_raw".format(robot):
                        0.01 * robot,
                    "agv{}_s_actual".format(robot):
                        index * 0.01 + 0.0005 * robot,
                    "agv{}_s_dot_actual".format(robot): 0.08,
                    "agv{}_s_dot_execute_reference".format(robot): 0.079,
                })
                for side in ("left", "right"):
                    prefix = "agv{}_wheel_{}".format(robot, side)
                    row[prefix + "_pre_limit"] = 0.07 + 0.002 * robot
                    row[prefix + "_applied"] = 0.07
                    row[prefix + "_actual"] = 0.069
                    row[prefix + "_reported_limit"] = (
                        0.10 if robot == 2 and 5 <= index <= 9 else 0.15)
            rows.append(row)
        write_csv(path, rows)

    def test_compact_publication_set_writes_chinese_png_pdf_and_tables(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "aligned_samples.csv"
            self._publication_fixture(source)
            output = root / "paper_figures"
            self.assertEqual(plot_experiment1(source, output / "experiment1"), 4)
            self.assertEqual(plot_experiment2a(
                source, source, output / "experiment2a", local_window=(0.4, 1.0)), 4)
            self.assertEqual(plot_experiment2b(
                source, source, output / "experiment2b", event=(0.5, 0.9)), 3)
            self.assertEqual(plot_experiment3(
                source, source, source, output / "experiment3",
                disturbance=(0.5, 0.9)), 2)
            expected = {
                "experiment1": (
                    "experiment1_trajectory", "experiment1_boundary_velocity",
                    "experiment1_tracking_error", "experiment1_wheel_execution"),
                "experiment2a": (
                    "experiment2a_boundary_comparison", "experiment2a_robot2_wheel",
                    "experiment2a_formation_error", "experiment2a_local_trajectory"),
                "experiment2b": (
                    "experiment2b_boundary_comparison", "experiment2b_execution_error",
                    "experiment2b_local_trajectory"),
                "experiment3": (
                    "experiment3_tracking_error", "experiment3_disturbance_response"),
            }
            for directory, names in expected.items():
                for name in names:
                    self.assertTrue((output / directory / (name + ".png")).is_file())
                    self.assertTrue((output / directory / (name + ".pdf")).is_file())
            metrics = root / "summary_metrics.json"
            atomic_dump_json(metrics, {
                "task": {"completion_time": 12.0},
                "path": {"progress_rmse": 0.01,
                         "progress_max_absolute": 0.02},
                "geometry": {"load_position_rmse": 0.01,
                             "rigid_fit_residual_max": 0.003,
                             "support": {"agv1": {"position_max": 0.02}}},
                "wheel": {"maximum_pre_limit_demand": 0.12,
                          "limited_time": 0.0},
                "internal": {"channel_input_max_absolute": 0.2},
            })
            table = output / "experiment2a" / "experiment2a_statistics.csv"
            self.assertEqual(write_statistics(
                [("M1", metrics), ("M2a", metrics), ("M4", metrics)],
                table, "experiment2a"), 3)
            with open(table, encoding="utf-8") as stream:
                table_rows = list(csv.DictReader(stream))
                methods = [row["method"] for row in table_rows]
            self.assertEqual(methods, ["M1", "M2a", "M4"])
            self.assertTrue(all(row["experiment"] == "experiment2a"
                                for row in table_rows))
            r4_table = output / "experiment3" / "experiment3_statistics.csv"
            self.assertEqual(write_statistics(
                [("R4", metrics)], r4_table, "experiment3"), 1)
            with open(r4_table, encoding="utf-8") as stream:
                self.assertEqual(next(csv.DictReader(stream))["method"], "R4")
            source_text = Path(
                plot_experiment1.__code__.co_filename).read_text(encoding="utf-8")
            self.assertNotIn("0.230940107", source_text)
            self.assertNotIn("0.115470053", source_text)

    def test_single_run_pipeline_writes_six_chinese_adaptive_figures(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root / "aligned_samples.csv"
            self._publication_fixture(source)
            output = root / "plots"
            self.assertEqual(plot_run(source, output), 6)
            for name in (
                    "figure2_trajectory",
                    "figure3_capability_boundary_reference",
                    "figure4_channel_constraints",
                    "figure5_robot2_wheel_chain",
                    "figure6_cooperative_errors",
                    "figure7_progress_recovery"):
                self.assertTrue((output / (name + ".png")).is_file())
                self.assertTrue((output / (name + ".pdf")).is_file())
            metadata = load_yaml(output / "plot_metadata.json")
            self.assertEqual(metadata["figures"], 6)
            self.assertEqual(
                metadata["axes"]["figure6.position_error"]["unit"], "m")
            self.assertEqual(
                metadata["axes"]["figure6.position_error"]["y_min"], 0.0)
            self.assertEqual(
                metadata["axes"]["figure7.progress_error"]["unit"], "m")
            self.assertGreaterEqual(
                len(metadata["axes"]["figure6.position_error"]["ticks"]), 5)
            source_text = Path(
                plot_run.__code__.co_filename).read_text(encoding="utf-8")
            font_source = Path(
                plot_experiment1.__code__.co_filename).read_text(
                    encoding="utf-8")
            self.assertIn("FigureCanvasCairo", font_source)
            self.assertIn("TrueType font is missing table", font_source)
            for marker in (
                    "S路径与三车支撑点轨迹",
                    "路径域能力与M1动态边界",
                    "M1参考动态边界与R1执行/实测速度",
                    "Robot2轮速执行链",
                    "等效载荷、支撑点及构型误差",
                    "路径进度误差与速度误差",
                    "dpi=320"):
                self.assertIn(marker, source_text)

    def test_logical_views_are_rebuilt_from_aligned_source(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            converted = root / "converted"
            converted.mkdir()
            write_csv(converted / "aligned_samples.csv", [{
                "stamp": 1.0,
                "load_s_reference": 0.0,
                "agv1_robot_pose_x": 0.1,
                "common_velocity_reference": 0.05,
                "agv1_boundary_upper": 0.08,
                "agv1_wheel_left_raw": 0.04,
                "agv1_position_error": 0.001,
                "m2b_input_raw_1": 0.0,
            }])
            write_csv(converted / "capability_report.csv", [{
                "stamp": 1.0, "robot_id": 1,
                "wheel_velocity_limit": 0.08,
            }])
            written = export_views(converted, root)
            self.assertEqual(len(written), 6)
            for filename in written:
                self.assertTrue((root / filename).is_file())
            with open(root / "wheel_chain.csv", encoding="utf-8") as stream:
                row = next(csv.DictReader(stream))
            self.assertEqual(row["agv1_wheel_left_raw"], "0.04")

    def test_missing_logical_group_is_rejected(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            write_csv(root / "aligned_samples.csv", [{"stamp": 1.0}])
            write_csv(root / "capability_report.csv", [])
            with self.assertRaises(RuntimeError):
                export_views(root, root / "output")

    def test_m1_run_does_not_require_m2b_internal_view(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            converted = root / "converted"
            converted.mkdir()
            write_csv(converted / "aligned_samples.csv", [{
                "stamp": 1.0,
                "load_s_reference": 0.0,
                "agv1_robot_pose_x": 0.1,
                "common_velocity_reference": 0.05,
                "agv1_boundary_upper": 0.08,
                "agv1_wheel_left_raw": 0.04,
                "agv1_position_error": 0.001,
            }])
            write_csv(converted / "controller_state.csv", [{
                "method_id": "M1_R1",
            }])
            write_csv(converted / "capability_report.csv", [])

            written = export_views(converted, root)
            self.assertNotIn("m2b_internal.csv", written)
            self.assertFalse((root / "m2b_internal.csv").exists())


if __name__ == "__main__":
    unittest.main()

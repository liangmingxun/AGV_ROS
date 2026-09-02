#!/usr/bin/env python3

import csv
import tempfile
import unittest
from pathlib import Path

from multi_agv_analysis.io_utils import write_csv
from multi_agv_analysis.paper_pipeline import export_views


class PaperPipelineTest(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()

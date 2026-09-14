#!/usr/bin/env python3

import importlib.util
import math
import unittest
from pathlib import Path


SCRIPT = (Path(__file__).resolve().parents[1] / "scripts" /
          "diagnose_exp2c_fake_pairs.py")
SPEC = importlib.util.spec_from_file_location("exp2c_diagnostic", str(SCRIPT))
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class Exp2cDiagnosticTest(unittest.TestCase):

    def test_time_and_path_integrals_use_selected_coordinate(self):
        rows = [
            {"stamp": "0", "agv2_s_tracking_actual": "2.0", "value": "-2"},
            {"stamp": "1", "agv2_s_tracking_actual": "2.2", "value": "-2"},
            {"stamp": "2", "agv2_s_tracking_actual": "2.5", "value": "-2"},
        ]
        getter = lambda row: MODULE.finite(row["value"])
        self.assertAlmostEqual(MODULE.integral(rows, getter, True), 4.0)
        self.assertAlmostEqual(
            MODULE.integral(rows, getter, True, path_domain=True), 1.0)

    def test_fully_safe_tie_is_not_misattributed_to_capability(self):
        records = [{
            "row": {"agv2_s_tracking_actual": "2.4",
                    "agv2_disturbance_window": "1"},
            "per_robot": {robot: {
                "capability_margin": 1.0, "input_margin": 1.0,
                "wheel_margin": 1.0, "position_margin": 1.0,
                "failure_margin": 1.0} for robot in range(1, 4)},
            "observed_by_robot": {robot: 1.0 for robot in range(1, 4)},
            "dominant_value": 1.0, "dominant_robot": 0,
            "dominant_margin": "none_all_safe",
        }]
        result = MODULE.phase_margin_summary(records, "active")
        self.assertEqual(result["no_margin_active_fraction"], 1.0)
        self.assertEqual(result["dominance_fraction_by_margin_type"]
                         ["capability_margin"], 0.0)
        self.assertEqual(result["reconstruction_error"]["maximum_absolute"], 0.0)

    def test_percentile_and_stats_are_population_statistics(self):
        result = MODULE.stats([0.0, 1.0, 2.0, 3.0])
        self.assertEqual(result["mean"], 1.5)
        self.assertAlmostEqual(result["std"], math.sqrt(1.25))
        self.assertAlmostEqual(result["p05"], 0.15)


if __name__ == "__main__":
    unittest.main()

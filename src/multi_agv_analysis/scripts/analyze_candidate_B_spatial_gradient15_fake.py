#!/usr/bin/env python3
"""Candidate B v2 mean-preserving 15% spatial-gradient fake analysis."""
import importlib.util
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location(
    "candidate_b_spatial_base",
    HERE / "analyze_candidate_B_spatial_composite_fake.py")
BASE = importlib.util.module_from_spec(spec)
spec.loader.exec_module(BASE)

BASE.IDENTITY = "candidate_B_spatial_gradient15_fake_validation"
BASE.PROFILE = yaml.safe_load((
    BASE.CONFIG / "formal_fake_candidate_B_spatial_gradient15.yaml").read_text()
)["formal_fake_runtime"]["candidate_b_spatial_composite"]
BASE.COMPARISON_FILENAME = "candidate_b_spatial_gradient15_comparison.json"
BASE.REPORT_FILENAME = (
    "candidate-B-spatial-gradient15-rho0p85-av0p020-fake-results.md")
BASE.ENABLE_SPATIAL_DOMAIN_RMSE = True
BASE.SPATIAL_DOMAIN_GRID_SAMPLES = 5001

if __name__ == "__main__":
    raise SystemExit(BASE.main())

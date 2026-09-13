"""One display contract for automatic run plots and paper comparisons."""
from pathlib import Path

from .io_utils import load_yaml


RAW_DISPLAY_PROFILE = {"display": {
    "smoothing_window_seconds": 0.0,
    "wheel_feedback_smoothing_window_seconds": 0.0,
    "show_raw_samples": False,
    "maximum_plot_rate_hz": 1000.0,
}}
SMOOTHED_DISPLAY_PROFILE = {"display": {
    "smoothing_window_seconds": 0.80,
    "wheel_feedback_smoothing_window_seconds": 0.80,
    "show_raw_samples": False,
    "maximum_plot_rate_hz": 25.0,
}}


def shared_axes():
    return load_yaml(Path(__file__).resolve().parents[2] /
                     "config" / "paper_axis_defaults.yaml")

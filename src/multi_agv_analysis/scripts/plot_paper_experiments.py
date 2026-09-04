#!/usr/bin/env python3

"""Generate the compact physical-experiment figure set from selected ROS runs."""

import argparse
import sys
from pathlib import Path

from multi_agv_analysis.publication_plots import (
    plot_experiment1, plot_experiment2a, plot_experiment2b,
    plot_experiment3, write_statistics)


NUMBERED_FOLDERS = {
    "experiment1": "01_M1_R1_complete_method",
    "experiment2a": "02a_M1_vs_M2a_capability_derating",
    "experiment2b": "02b_M1_vs_M2b_complete_method",
    "experiment3": "03_R1_R2_R3_disturbance_rejection",
}


def _entry(value):
    if "=" not in value:
        raise argparse.ArgumentTypeError("统计项格式必须是 METHOD=summary_metrics.json")
    method, path = value.split("=", 1)
    return method, path


def main():
    parser = argparse.ArgumentParser(
        description="从Robot1/ROS处理后数据生成中文论文图片（PNG+PDF）")
    parser.add_argument("--output-dir", default="paper_figures")
    parser.add_argument(
        "--numbered-folders", action="store_true",
        help="按实验编号和标题命名输出子目录")
    parser.add_argument("--experiment1")
    parser.add_argument("--experiment2a-m1")
    parser.add_argument("--experiment2a-m2a")
    parser.add_argument("--experiment2b-m1")
    parser.add_argument("--experiment2b-m2b")
    parser.add_argument("--experiment3-r1")
    parser.add_argument("--experiment3-r2")
    parser.add_argument("--experiment3-r3")
    parser.add_argument("--experiment2a-event", nargs=2, type=float,
                        metavar=("START", "END"))
    parser.add_argument("--experiment2b-event", nargs=2, type=float,
                        metavar=("START", "END"))
    parser.add_argument("--experiment3-disturbance", nargs=2, type=float,
                        metavar=("START", "END"))
    parser.add_argument("--experiment2a-zoom", nargs=2, type=float,
                        metavar=("START", "END"))
    parser.add_argument("--experiment2b-zoom", nargs=2, type=float,
                        metavar=("START", "END"))
    parser.add_argument("--experiment2a-stat", action="append", type=_entry,
                        default=[])
    parser.add_argument("--experiment3-stat", action="append", type=_entry,
                        default=[])
    args = parser.parse_args()
    root = Path(args.output_dir)
    folders = (NUMBERED_FOLDERS if args.numbered_folders else {
        name: name for name in NUMBERED_FOLDERS})
    written = 0
    try:
        if args.experiment1:
            written += plot_experiment1(
                args.experiment1, root / folders["experiment1"])
        exp2a = (args.experiment2a_m1, args.experiment2a_m2a)
        if any(exp2a):
            if not all(exp2a):
                raise RuntimeError("实验2a必须同时提供M1和M2a run")
            written += plot_experiment2a(
                *exp2a, root / folders["experiment2a"],
                tuple(args.experiment2a_event) if args.experiment2a_event else None,
                tuple(args.experiment2a_zoom) if args.experiment2a_zoom else None)
        exp2b = (args.experiment2b_m1, args.experiment2b_m2b)
        if any(exp2b):
            if not all(exp2b):
                raise RuntimeError("实验2b必须同时提供M1和M2b run")
            written += plot_experiment2b(
                *exp2b, root / folders["experiment2b"],
                tuple(args.experiment2b_event) if args.experiment2b_event else None,
                tuple(args.experiment2b_zoom) if args.experiment2b_zoom else None)
        exp3 = (args.experiment3_r1, args.experiment3_r2, args.experiment3_r3)
        if any(exp3):
            if not all(exp3):
                raise RuntimeError("实验3必须同时提供R1、R2和R3 run")
            written += plot_experiment3(
                *exp3, root / folders["experiment3"],
                tuple(args.experiment3_disturbance)
                if args.experiment3_disturbance else None)
        if args.experiment2a_stat:
            write_statistics(args.experiment2a_stat,
                             root / folders["experiment2a"] /
                             "experiment2a_statistics.csv",
                             "experiment2a")
        if args.experiment3_stat:
            write_statistics(args.experiment3_stat,
                             root / folders["experiment3"] /
                             "experiment3_statistics.csv",
                             "experiment3")
        if not written and not args.experiment2a_stat and not args.experiment3_stat:
            raise RuntimeError("至少指定一个实验run或统计项")
        print("论文出图完成：{}组图片（每组PNG+PDF）".format(written))
        print("output_dir={}".format(root.resolve()))
        return 0
    except Exception as error:
        print("论文出图失败：{}".format(error), file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())

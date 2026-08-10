#!/usr/bin/env python3

import argparse
import re
import sys
from pathlib import Path

from multi_agv_analysis.io_utils import (
    atomic_dump_json,
    load_yaml,
    write_csv,
)
from multi_agv_analysis.reporting import aggregate_plan, markdown_table


def _plots(report, directory):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    methods = report["methods"]
    for metric, summaries in report["summary"].items():
        values = [
            [run["metrics"][metric] for run in report["runs"]
             if run["method_id"] == method]
            for method in methods]
        figure, axis = plt.subplots(figsize=(5.2, 3.3))
        axis.boxplot(values, labels=methods, showmeans=True)
        axis.set_ylabel(metric)
        axis.grid(axis="y", alpha=0.25)
        figure.tight_layout()
        filename = re.sub(r"[^A-Za-z0-9_.-]+", "_", metric) + ".svg"
        figure.savefig(directory / filename)
        plt.close(figure)


def main():
    parser = argparse.ArgumentParser(
        description="Aggregate completed paired runs into paper-ready data.")
    parser.add_argument("plan")
    parser.add_argument("report_json")
    parser.add_argument("summary_markdown")
    parser.add_argument("--metric", action="append", required=True)
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--require-formal-statistics", action="store_true")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--comparison-csv")
    parser.add_argument("--plot-dir")
    args = parser.parse_args()
    try:
        report = aggregate_plan(
            load_yaml(args.plan), args.metric, args.baseline,
            args.require_formal_statistics, args.seed)
        atomic_dump_json(args.report_json, report)
        summary_path = Path(args.summary_markdown)
        summary_path.parent.mkdir(parents=True, exist_ok=True)
        summary_path.write_text(
            markdown_table(report), encoding="utf-8")
        if args.comparison_csv:
            rows = []
            for value in report["comparisons"]:
                rows.append({
                    "metric": value["metric"],
                    "baseline": value["baseline"],
                    "comparison": value["comparison"],
                    "paired_blocks": value["paired_blocks"],
                    "mean_difference": value["difference"]["mean"],
                    "ci95_lower":
                        value["mean_difference_bootstrap_95"][0],
                    "ci95_upper":
                        value["mean_difference_bootstrap_95"][1],
                    "sign_flip_p": value["two_sided_sign_flip_p"],
                })
            write_csv(args.comparison_csv, rows)
        if args.plot_dir:
            _plots(report, args.plot_dir)
        print("PAIRED REPORT WRITTEN: {}".format(args.report_json))
        return 0
    except Exception as error:
        print("AGGREGATION REFUSED: {}".format(error), file=sys.stderr)
        return 3


if __name__ == "__main__":
    sys.exit(main())

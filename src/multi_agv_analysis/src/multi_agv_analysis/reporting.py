"""Paired experiment aggregation with deterministic resampling."""

import itertools
import math
import random
from collections import defaultdict

from .io_utils import finite_float, load_yaml


def nested_value(value, dotted):
    for key in dotted.split("."):
        if not isinstance(value, dict) or key not in value:
            return math.nan
        value = value[key]
    return finite_float(value)


def _quantile(values, probability):
    ordered = sorted(values)
    if not ordered:
        return None
    position = probability * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    weight = position - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def describe(values):
    values = [float(value) for value in values if math.isfinite(float(value))]
    if not values:
        return {"n": 0}
    mean = sum(values) / len(values)
    variance = (
        sum((value - mean) ** 2 for value in values) / (len(values) - 1)
        if len(values) > 1 else 0.0)
    return {
        "n": len(values),
        "mean": mean,
        "standard_deviation": math.sqrt(variance),
        "median": _quantile(values, 0.5),
        "q1": _quantile(values, 0.25),
        "q3": _quantile(values, 0.75),
        "minimum": min(values),
        "maximum": max(values),
    }


def bootstrap_mean_interval(values, seed, samples=10000, level=0.95):
    if not values:
        return [None, None]
    rng = random.Random(seed)
    estimates = []
    for _ in range(samples):
        sample = [values[rng.randrange(len(values))]
                  for _ in range(len(values))]
        estimates.append(sum(sample) / len(sample))
    tail = (1.0 - level) / 2.0
    return [_quantile(estimates, tail), _quantile(estimates, 1.0 - tail)]


def sign_flip_p_value(differences, seed, maximum_exact_pairs=16,
                      monte_carlo_samples=100000):
    differences = [value for value in differences if value != 0.0]
    if not differences:
        return 1.0
    observed = abs(sum(differences) / len(differences))
    exceed = 0
    total = 0
    if len(differences) <= maximum_exact_pairs:
        signs = itertools.product((-1.0, 1.0), repeat=len(differences))
    else:
        rng = random.Random(seed)
        signs = (
            tuple(rng.choice((-1.0, 1.0)) for _ in differences)
            for _ in range(monte_carlo_samples))
    for sign in signs:
        statistic = abs(sum(
            factor * value for factor, value in zip(sign, differences)) /
            len(differences))
        exceed += int(statistic >= observed - 1.0e-15)
        total += 1
    return exceed / total


def aggregate_plan(plan, metric_paths, baseline, require_formal=False,
                   seed=0):
    methods = list(plan.get("methods", []))
    if baseline not in methods:
        raise ValueError("baseline method is absent from plan")
    complete = [run for run in plan.get("runs", [])
                if run.get("status") == "complete"]
    if not complete:
        raise ValueError("plan contains no completed runs")
    by_block = defaultdict(dict)
    run_values = []
    for run in complete:
        metrics = load_yaml(run["metrics"])
        if metrics.get("run_id") != run["run_id"]:
            raise ValueError("metrics run_id mismatch: {}".format(run["run_id"]))
        if metrics.get("method_id") != run["method_id"]:
            raise ValueError(
                "metrics method mismatch: {}".format(run["run_id"]))
        ready = bool(metrics.get(
            "evaluation_window", {}).get("formal_statistics_ready", False))
        if require_formal:
            manifest = load_yaml(run["manifest"])
            approval = manifest.get("configuration_approval")
            manifest_ready = (
                manifest.get("formal_statistics_requested") is True and
                isinstance(approval, dict) and
                approval.get("formal_statistics_authorized") is True)
            if not ready or not manifest_ready:
                raise ValueError(
                    "run is not formal-statistics ready: {}".format(
                        run["run_id"]))
        values = {path: nested_value(metrics, path) for path in metric_paths}
        missing = [path for path, value in values.items()
                   if not math.isfinite(value)]
        if missing:
            raise ValueError(
                "run {} lacks metrics {}".format(run["run_id"], missing))
        by_block[run["block_id"]][run["method_id"]] = values
        run_values.append({
            "run_id": run["run_id"],
            "block_id": run["block_id"],
            "method_id": run["method_id"],
            "formal_statistics_ready": ready,
            "metrics": values,
        })
    complete_blocks = {
        block: values for block, values in by_block.items()
        if set(values) == set(methods)}
    if not complete_blocks:
        raise ValueError("no complete paired block is available")

    summary = {}
    comparisons = []
    for path_index, path in enumerate(metric_paths):
        summary[path] = {}
        for method in methods:
            values = [
                block[method][path] for block in complete_blocks.values()]
            summary[path][method] = describe(values)
        for method_index, method in enumerate(methods):
            if method == baseline:
                continue
            differences = [
                block[method][path] - block[baseline][path]
                for block in complete_blocks.values()]
            comparisons.append({
                "metric": path,
                "baseline": baseline,
                "comparison": method,
                "paired_blocks": len(differences),
                "difference_definition": "comparison_minus_baseline",
                "difference": describe(differences),
                "mean_difference_bootstrap_95": bootstrap_mean_interval(
                    differences, seed + 1000 * path_index + method_index),
                "two_sided_sign_flip_p": sign_flip_p_value(
                    differences, seed + 2000 * path_index + method_index),
            })
    return {
        "schema_version": 1,
        "design": "complete_paired_randomized_blocks",
        "plan_sha256": plan.get("plan_sha256", ""),
        "baseline": baseline,
        "methods": methods,
        "complete_paired_blocks": len(complete_blocks),
        "require_formal_statistics": require_formal,
        "summary": summary,
        "comparisons": comparisons,
        "runs": run_values,
    }


def markdown_table(report):
    lines = [
        "| Metric | Method | n | Mean | SD | Median | IQR |",
        "|---|---:|---:|---:|---:|---:|---:|",
    ]
    for metric, methods in report["summary"].items():
        for method, value in methods.items():
            lines.append(
                "| {} | {} | {} | {:.6g} | {:.6g} | {:.6g} | "
                "[{:.6g}, {:.6g}] |".format(
                    metric, method, value["n"], value["mean"],
                    value["standard_deviation"], value["median"],
                    value["q1"], value["q3"]))
    return "\n".join(lines) + "\n"

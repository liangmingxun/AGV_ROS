#!/usr/bin/env python3
"""Export deterministic vectors from the approved paper reference programs.

This script deliberately imports a user-supplied source path.  It never
substitutes locally generated vectors when an authoritative source is absent.
"""

import argparse
import csv
import importlib.util
import os
import sys
from pathlib import Path

import numpy as np

os.environ.setdefault("MPLBACKEND", "Agg")


def load_module(path: Path):
    if not path.is_file():
        raise FileNotFoundError(f"authoritative reference not found: {path}")
    spec = importlib.util.spec_from_file_location("agv_reference", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import reference: {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def flatten(name, value, row):
    arr = np.asarray(value)
    if arr.ndim == 0:
        row[name] = float(arr)
        return
    for index in np.ndindex(arr.shape):
        row[name + "_" + "_".join(map(str, index))] = float(arr[index])


def export(data, keys, count, output):
    rows = []
    available = len(np.asarray(data["t"]))
    for k in range(min(count, available)):
        row = {"step": k}
        for key in keys:
            if key not in data:
                raise KeyError(f"reference output is missing required key: {key}")
            flatten(key, np.asarray(data[key])[k], row)
        rows.append(row)
    if not rows:
        raise RuntimeError("reference produced no samples")
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--kind", choices=["exp1", "exp2-m1", "exp2-m2"], required=True)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--count", type=int, required=True)
    args = parser.parse_args()
    if args.count <= 0:
        parser.error("--count must be positive")

    module = load_module(args.source.resolve())
    if args.kind == "exp1":
        data = module.Experiment1Simulator(
            module.Params(output_dir=str(args.output.parent))).run()
        keys = [
            "t", "lower", "upper", "lower_s", "upper_s", "z", "ups",
            "phi", "vL", "vL_dot", "sL", "x1", "x2", "es", "ev",
            "psi", "Gamma", "GammaInvUps", "r", "u", "theta_hat",
            "Dhat",
        ]
    else:
        model = module.Experiment2Model(
            module.Config(output_dir=str(args.output.parent)))
        method = "M1" if args.kind == "exp2-m1" else "M2"
        data = module.simulate_method(
            model, method, record=True, reduced=False)
        keys = [
            "t", "lower", "upper", "lower_s", "upper_s", "z", "ups",
            "phi", "v_ref", "s_ref", "capacity_reported", "rho", "beta",
            "a_ref", "zeta_z", "zeta_v", "zeta_phi", "x1", "x2", "es",
            "ev", "psi", "Gamma", "GammaInvUps", "r", "u_raw", "u_act",
            "theta_hat", "paper_delta_inv", "paper_zeta",
        ]
    export(data, keys, args.count, args.output)


if __name__ == "__main__":
    main()

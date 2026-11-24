#!/usr/bin/env python3
"""Sample size from the pilot (Module 17 of instructions.md, step 3).

    python3 analysis/sample_size.py [logs] [--block P] [--delta 0.10]

For each info-aware method and each (map, pose) pair of Block P:

    r = (d_95[method] - d_95[nearest]) / d_95[nearest]

Pooled over the three maps (unless they are obviously different), n_needed() gives the
number of poses per map for a two-sided paired Wilcoxon test at alpha = 0.05 / 3 (Holm
worst case across 3 maps), power 0.8, to detect a 10% median difference. The decision uses
the conservative n_safe (80% upper confidence bound on the sd). Runs that did not reach
95% coverage use their total distance and are flagged (pre-registration rule).
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
from scipy.stats import chi2, norm

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


def n_needed(r, delta=0.10, alpha=0.05 / 3, power=0.8, are=0.864):
    """As given in instructions.md Module 17 step 3. Returns (n_point, n_safe)."""
    # alpha/3: Holm worst case across 3 maps
    # are: Wilcoxon asymptotic relative efficiency vs. t-test
    r = np.asarray(r)
    k = len(r) - 1
    sd = r.std(ddof=1)
    sd_upper = sd * np.sqrt(k / chi2.ppf(0.20, k))   # 80% upper bound on sd
    z = norm.ppf(1 - alpha / 2) + norm.ppf(power)
    n_point = (z * sd / delta) ** 2 / are
    n_safe = (z * sd_upper / delta) ** 2 / are
    return int(np.ceil(n_point)), int(np.ceil(n_safe))


def relative_differences(runs, method, baseline="nearest", value="d_95_or_total"):
    """DataFrame (map, pose, r, censored) for one info-aware method."""
    from load import paired
    w = paired(runs, value=value, baseline=baseline, keys=("map", "pose"))
    if method not in w:
        return w.iloc[0:0]
    w = w[w[method].notna()].copy()
    w["r"] = (w[method] - w[baseline]) / w[baseline]
    w["censored"] = w.get(f"{method}_censored", False).astype(bool) | w.get(f"{baseline}_censored", False).astype(bool)
    return w[["map", "pose", "r", "censored"]]


def analyse(runs, delta=0.10) -> dict:
    out = {"delta": delta, "methods": {}}
    for method in [m for m in runs["method"].cat.categories if m != "nearest"]:
        rd = relative_differences(runs, method)
        if len(rd) < 3:
            out["methods"][method] = {"n_pairs": int(len(rd)), "note": "fewer than 3 pairs"}
            continue
        per_map = {mp: {"n": int(len(g)), "mean_r": float(g["r"].mean()), "sd_r": float(g["r"].std(ddof=1))
                        if len(g) > 1 else None} for mp, g in rd.groupby("map")}
        n_point, n_safe = n_needed(rd["r"].to_numpy(), delta=delta)
        out["methods"][method] = {
            "n_pairs": int(len(rd)), "n_censored": int(rd["censored"].sum()),
            "mean_r": float(rd["r"].mean()), "median_r": float(rd["r"].median()),
            "sd_r_pooled": float(rd["r"].std(ddof=1)), "per_map": per_map,
            "n_point": n_point, "n_safe": n_safe,
        }
    safes = [v["n_safe"] for v in out["methods"].values() if "n_safe" in v]
    out["n_safe_max"] = max(safes) if safes else None
    if out["n_safe_max"] is None:
        out["decision"] = "undecided: not enough pilot pairs"
    elif out["n_safe_max"] <= 10:
        out["decision"] = "Block Q at 10 poses per map; the scaling ablation (Module 21) stays in"
    else:
        out["decision"] = ("Block Q at 15 poses per map; the scaling ablation is dropped"
                           + ("" if out["n_safe_max"] <= 15 else
                              f" (n_safe = {out['n_safe_max']} > 15: 15 is the most the schedule allows; "
                              "report the lower power)"))
    return out


def main(argv=None) -> int:
    from load import load_runs
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("logs", nargs="?", default=os.path.join(HERE, "..", "logs"))
    ap.add_argument("--block", default="P")
    ap.add_argument("--delta", type=float, default=0.10)
    ap.add_argument("--out", help="also write the result as JSON")
    a = ap.parse_args(argv)
    runs = load_runs(a.logs, a.block)
    if runs.empty:
        print(f"no Block {a.block} runs in {a.logs}")
        return 1
    res = analyse(runs, a.delta)
    print(json.dumps(res, indent=2))
    if a.out:
        os.makedirs(os.path.dirname(os.path.abspath(a.out)), exist_ok=True)
        with open(a.out, "w") as f:
            json.dump(res, f, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())

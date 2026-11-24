#!/usr/bin/env python3
"""Rebuild every figure and every reported number from logs/ (Module 20, Gate G5).

    python3 analysis/make_figures.py [--logs logs] [--out analysis/out]

Writes analysis/out/numbers.json (everything the report quotes, Module 22 pulls from it)
and the figures:

    fig_q1_coverage.png        Q1: coverage vs. distance per map (Block Q)
    fig_q2_latency.png         Q2: decision latency per method and power mode (Block E)
    fig_q3_pareto.png          Q3: d_95 vs. planner energy above idle (Block E)
    fig_q3_break_even.png      Q3: mission energy vs. k_loco, break-even k*
    fig_h2_scaling.png         H2: latency vs. n_frontiers (Block E)
    fig_h2_ablation.png        Module 21, if Block A runs exist (exploratory)
    tables/*.csv               the per-cell tables behind the numbers

Deterministic: fixed bootstrap seeds, sorted inputs, no timestamps in numbers.json, so
deleting analysis/out/ and running this again gives the same numbers on any machine.
Blocks without runs are skipped and listed under numbers.json "missing".
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, HERE)

import h2_scaling  # noqa: E402
import q1_quality  # noqa: E402
import q2_compute  # noqa: E402
import q3_tradeoff  # noqa: E402
from load import load_idle, load_runs  # noqa: E402


def clean(o):
    """JSON-safe: numpy scalars to Python, NaN/inf to None, floats rounded to 6 significant digits."""
    if isinstance(o, dict):
        return {str(k): clean(v) for k, v in sorted(o.items(), key=lambda kv: str(kv[0]))}
    if isinstance(o, (list, tuple)):
        return [clean(v) for v in o]
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (float, np.floating)):
        f = float(o)
        return None if not math.isfinite(f) else float(f"{f:.6g}")
    if isinstance(o, np.bool_):
        return bool(o)
    return o


def logs_digest(runs) -> str:
    h = hashlib.sha256()
    for rid, out in sorted(zip(runs["run_id"], runs["outcome"])) if not runs.empty else []:
        h.update(f"{rid}:{out}\n".encode())
    return h.hexdigest()[:16]


def build(logs: str, out: str) -> dict:
    os.makedirs(os.path.join(out, "tables"), exist_ok=True)
    allruns = load_runs(logs)
    idle = load_idle(logs)
    numbers = {"inputs": {"n_runs": int(len(allruns)), "logs_digest": logs_digest(allruns),
                          "n_idle_baselines": int(len(idle))}, "missing": [], "figures": []}
    if not allruns.empty:
        valid = allruns[allruns["outcome"] != "invalid"]
        numbers["inputs"]["runs_per_block"] = valid["block"].value_counts().sort_index().to_dict()
        numbers["inputs"]["invalid_runs"] = int((allruns["outcome"] == "invalid").sum())
        numbers["inputs"]["outcomes"] = valid.groupby(["block", "outcome"]).size().rename("n").reset_index() \
            .assign(k=lambda d: d["block"] + "/" + d["outcome"]).set_index("k")["n"].to_dict()
    else:
        valid = allruns

    def block(b):
        if valid.empty:
            return valid
        r = valid[valid["block"] == b]
        if len(r):
            r = r.copy()
            r["method"] = r["method"].cat.remove_unused_categories()
        return r

    q = block("Q")
    if len(q):
        numbers["q1"] = q1_quality.analyse(q)
        p = q1_quality.figure(q, os.path.join(out, "fig_q1_coverage.png"))
        numbers["figures"] += [os.path.basename(p)] if p else []
        q.drop(columns=["dir"]).to_csv(os.path.join(out, "tables", "block_q_runs.csv"), index=False)
    else:
        numbers["missing"].append("Block Q (Q1/H1)")

    e = block("E")
    if len(e):
        q2, en = q2_compute.analyse(e, idle)
        numbers["q2"] = q2
        en.to_csv(os.path.join(out, "tables", "block_e_energy.csv"), index=False)
        p = q2_compute.figure(e, os.path.join(out, "fig_q2_latency.png"))
        numbers["figures"] += [os.path.basename(p)] if p else []
        numbers["q3"] = q3_tradeoff.analyse(en)
        numbers["figures"] += [os.path.basename(x) for x in q3_tradeoff.figures(en, numbers["q3"], out)]
        h2, cyc = h2_scaling.analyse(e)
        numbers["h2"] = h2
        p = h2_scaling.figure(cyc, h2, os.path.join(out, "fig_h2_scaling.png"), float(e["replan_period_s"].iloc[0]))
        numbers["figures"] += [os.path.basename(p)] if p else []
    else:
        numbers["missing"].append("Block E (Q2, Q3/H3, H2)")

    a = block("A")
    if len(a):
        h2a, cyca = h2_scaling.analyse(a)
        numbers["ablation_h2"] = {"exploratory": True, "slopes": h2a,
                                  "deadline_miss_rate": {str(m): float(g["deadline_miss_rate"].mean())
                                                         for m, g in a.groupby("method", observed=True)}}
        p = h2_scaling.figure(cyca, h2a, os.path.join(out, "fig_h2_ablation.png"),
                              float(a["replan_period_s"].iloc[0]), title_block="A (exploratory)")
        numbers["figures"] += [os.path.basename(p)] if p else []
    else:
        numbers["missing"].append("Block A (optional ablation)")

    numbers = clean(numbers)
    with open(os.path.join(out, "numbers.json"), "w", newline="\n") as f:
        json.dump(numbers, f, indent=2, sort_keys=True)
        f.write("\n")
    return numbers


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--logs", default=os.path.join(REPO, "logs"))
    ap.add_argument("--out", default=os.path.join(HERE, "out"))
    a = ap.parse_args(argv)
    n = build(a.logs, a.out)
    print(f"{a.out}/numbers.json: {n['inputs']['n_runs']} runs, figures: {', '.join(n['figures']) or 'none'}"
          + (f"; missing: {', '.join(n['missing'])}" if n["missing"] else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())

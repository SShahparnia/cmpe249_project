#!/usr/bin/env python3
"""Daily check (Module 19 of instructions.md, step 5).

    python3 scripts/status.py [configs/schedules/Q.csv ...] [--logs logs] [--seed N]

Prints planned vs. done per cell (block, method, map, power mode) for the given schedules
(default: every configs/schedules/*.csv), the count of each outcome, the invalid rate,
and plots one random coverage curve and one random power trace to logs/daily/<date>/.
Anything odd goes into docs/run_log.md by hand.
"""
from __future__ import annotations

import argparse
import collections
import csv
import datetime as dt
import glob
import os
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(REPO, "scripts"))
from run_batch import DATA, cell_key, existing_runs  # noqa: E402


def planned_cells(paths):
    cells = collections.Counter()
    keys = set()
    for p in paths:
        with open(p, newline="") as f:
            for r in csv.DictReader(f):
                k = cell_key(r)
                keys.add(k)
                cells[(k[0], k[1], k[2], k[4])] += 1
    return cells, keys


def summarize(paths, logs):
    planned, keys = planned_cells(paths)
    runs = existing_runs(logs)
    done = collections.Counter()
    outcomes = collections.Counter()
    for k, lst in runs.items():
        for _, o in lst:
            outcomes[o or "unknown"] += 1
        if k in keys and any(o in DATA for _, o in lst):
            done[(k[0], k[1], k[2], k[4])] += 1
    lines = [f"{'block':5} {'method':8} {'map':13} {'power_mode':11} {'done':>4} / planned"]
    for c in sorted(planned):
        flag = "" if done[c] == planned[c] else "  <-"
        lines.append(f"{c[0]:5} {c[1]:8} {c[2]:13} {c[3]:11} {done[c]:4} / {planned[c]}{flag}")
    total_runs = sum(outcomes.values())
    inv = outcomes.get("invalid", 0)
    lines.append("")
    lines.append("outcomes: " + ", ".join(f"{k} {v}" for k, v in sorted(outcomes.items())))
    lines.append(f"cells done: {sum(done.values())} / {sum(planned.values())}; invalid rate "
                 f"{100.0 * inv / total_runs if total_runs else 0:.1f}% of {total_runs} runs (Gate G4: < 5%)")
    return "\n".join(lines), runs


def plots(logs, runs, seed):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    sys.path.insert(0, os.path.join(REPO, "analysis"))
    from energy import read_csv
    rng = np.random.default_rng(seed)
    out = os.path.join(logs, "daily", dt.date.today().strftime("%Y%m%d"))
    os.makedirs(out, exist_ok=True)
    ids = [rid for lst in runs.values() for rid, o in lst if o in DATA]
    with_cov = [r for r in ids if os.path.exists(os.path.join(logs, r, "coverage.csv"))]
    with_pow = [r for r in ids if os.path.exists(os.path.join(logs, r, "power.csv"))]
    written = []
    if with_cov:
        rid = with_cov[rng.integers(len(with_cov))]
        c = read_csv(os.path.join(logs, rid, "coverage.csv"))
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.plot(c["distance_m"], c["coverage_frac"])
        ax.axhline(0.95, ls="--", c="gray")
        ax.set_xlabel("distance (m)")
        ax.set_ylabel("coverage")
        ax.set_title(rid, fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "coverage_random.png"), dpi=110)
        plt.close(fig)
        written.append(f"coverage: {rid}")
    if with_pow:
        rid = with_pow[rng.integers(len(with_pow))]
        p = read_csv(os.path.join(logs, rid, "power.csv"))
        fig, ax = plt.subplots(figsize=(6, 4))
        ax.plot(p["t_wall"] - p["t_wall"][0], p["vdd_in_mW"] / 1000.0, lw=0.6)
        ax.set_xlabel("t (s, wall)")
        ax.set_ylabel("VDD_IN (W)")
        ax.set_title(rid, fontsize=8)
        fig.tight_layout()
        fig.savefig(os.path.join(out, "power_random.png"), dpi=110)
        plt.close(fig)
        written.append(f"power: {rid}")
    return out, written


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("schedules", nargs="*")
    ap.add_argument("--logs", default=os.path.join(REPO, "logs"))
    ap.add_argument("--seed", type=int, help="for the random picks (default: today's date)")
    ap.add_argument("--no-plots", action="store_true")
    a = ap.parse_args(argv)
    paths = a.schedules or sorted(glob.glob(os.path.join(REPO, "configs", "schedules", "*.csv")))
    if not paths:
        print("no schedules: run scripts/make_schedule.py first")
        return 1
    text, runs = summarize(paths, a.logs)
    print(text)
    if not a.no_plots:
        out, written = plots(a.logs, runs, a.seed if a.seed is not None else int(dt.date.today().strftime("%Y%m%d")))
        print(f"\nplots in {out}: " + ("; ".join(written) or "none (no finished runs yet)"))
    return 0


if __name__ == "__main__":
    sys.exit(main())

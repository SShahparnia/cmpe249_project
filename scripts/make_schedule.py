#!/usr/bin/env python3
"""Write run schedules (Module 19 of instructions.md).

    python3 scripts/make_schedule.py --block P                                  # 27 pilot runs
    python3 scripts/make_schedule.py --block Q --poses 10 --seed 7 --machines laptop-a,laptop-b
    python3 scripts/make_schedule.py --block E --poses 5 --power-modes 7W,MAXN_SUPER
    python3 scripts/make_schedule.py --block A --poses 5 --power-modes MAXN_SUPER

Output: configs/schedules/<block>.csv, or one file per session for Block E / A
(configs/schedules/E_<power_mode>.csv: a session runs one power mode, Module 19 step 2).
Columns: order, block, method, map, pose, power_mode, machine.

Order: shuffled with numpy.random.default_rng(seed), then repaired so that no method runs
more than --max-streak times in a row. Block E / A: the (map, pose) order is shuffled and
the method order is shuffled independently within every (map, pose). Machines are assigned
round-robin in schedule order (Blocks P / Q; E / A always run on the Jetson).
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
METHODS = ["nearest", "mrtsp", "ariadne"]
MAPS = ["m1_house", "m2_office", "m3_warehouse"]
DEFAULT_POSES = {"P": 3, "Q": 10, "E": 5, "A": 5}


def break_streaks(rows, key="method", max_streak=2):
    """Deterministically move rows so that no value of `key` repeats more than max_streak times."""
    rows = list(rows)
    i = 0
    while i < len(rows):
        if i >= max_streak and all(rows[i - k][key] == rows[i][key] for k in range(1, max_streak + 1)):
            j = next((j for j in range(i + 1, len(rows)) if rows[j][key] != rows[i][key]), None)
            if j is None:          # only this value is left: move it earlier instead
                j = next((j for j in range(i - max_streak - 1, -1, -1)
                          if rows[j][key] != rows[i][key]
                          and (j == 0 or rows[j - 1][key] != rows[i][key])
                          and rows[j + 1][key] != rows[i][key]), None)
                if j is None:
                    break
                rows.insert(j + 1, rows.pop(i))
            else:
                rows[i], rows[j] = rows[j], rows[i]
        i += 1
    return rows


def max_streak(rows, key="method"):
    best = cur = 0
    prev = None
    for r in rows:
        cur = cur + 1 if r[key] == prev else 1
        prev = r[key]
        best = max(best, cur)
    return best


def schedule(block, n_poses, power_mode, seed, machines, maps=None, methods=None, streak=2):
    rng = np.random.default_rng(seed)
    maps, methods = maps or MAPS, methods or METHODS
    if block in ("P", "Q"):
        rows = [{"block": block, "method": m, "map": mp, "pose": p, "power_mode": power_mode}
                for mp in maps for p in range(n_poses) for m in methods]
        rows = [rows[i] for i in rng.permutation(len(rows))]
        rows = break_streaks(rows, max_streak=streak)
    else:
        cells = [(mp, p) for mp in maps for p in range(n_poses)]
        rows = []
        for ci in rng.permutation(len(cells)):
            mp, p = cells[ci]
            for mi in rng.permutation(len(methods)):
                rows.append({"block": block, "method": methods[mi], "map": mp, "pose": p, "power_mode": power_mode})
        rows = break_streaks(rows, max_streak=streak)
    for i, r in enumerate(rows):
        r["order"] = i
        r["machine"] = machines[i % len(machines)] if block in ("P", "Q") else "jetson"
    return rows


def write(path, rows):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    cols = ["order", "block", "method", "map", "pose", "power_mode", "machine"]
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        for r in rows:
            w.writerow({c: r[c] for c in cols})


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--block", choices=["P", "Q", "E", "A"], required=True)
    ap.add_argument("--poses", type=int, help="poses per map (indices 0..n-1); default P 3, Q 10, E 5, A 5")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--machines", default="laptop", help="comma-separated machine names (Blocks P, Q)")
    ap.add_argument("--power-modes", default="laptop", help="Block E / A: comma-separated nvpmodel names")
    ap.add_argument("--maps", help="default: the three maps; Block A: m2_office_2x")
    ap.add_argument("--max-streak", type=int, default=2)
    ap.add_argument("--out-dir", default=os.path.join(REPO, "configs", "schedules"))
    a = ap.parse_args(argv)

    n = a.poses or DEFAULT_POSES[a.block]
    maps = a.maps.split(",") if a.maps else (["m2_office_2x"] if a.block == "A" else MAPS)
    machines = a.machines.split(",")
    modes = a.power_modes.split(",")
    if a.block in ("E", "A") and modes == ["laptop"]:
        ap.error("Block E / A needs --power-modes (nvpmodel names from docs/power_modes.md)")
    if a.block in ("P", "Q") and modes != ["laptop"]:
        ap.error("Blocks P and Q run on laptops (power_mode laptop)")
    for k, mode in enumerate(modes):
        rows = schedule(a.block, n, mode, a.seed + k, machines, maps, streak=a.max_streak)
        name = f"{a.block}.csv" if a.block in ("P", "Q") else f"{a.block}_{mode}.csv"
        path = os.path.join(a.out_dir, name)
        write(path, rows)
        print(f"wrote {os.path.relpath(path, REPO)}: {len(rows)} runs, longest method streak {max_streak(rows)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

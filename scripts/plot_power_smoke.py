#!/usr/bin/env python3
"""Plot idle vs. stress VDD_IN traces and report the achieved sample rate
(Module 2, steps 7-8). Runs anywhere with numpy + matplotlib.

Usage: python3 scripts/plot_power_smoke.py idle.csv stress.csv --out docs/figures/power_smoke.png
"""
import argparse
import csv

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402


def load(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    t = np.array([float(r["t_wall"]) for r in rows])
    p = np.array([float(r["vdd_in_mW"]) if r["vdd_in_mW"] != "" else np.nan for r in rows])
    keep = ~np.isnan(p)
    return t[keep], p[keep]


ap = argparse.ArgumentParser()
ap.add_argument("idle")
ap.add_argument("stress")
ap.add_argument("--out", default="docs/figures/power_smoke.png")
args = ap.parse_args()

fig, ax = plt.subplots(figsize=(8, 4))
print("| phase | rows | median dt (ms) | achieved rate (Hz) | mean VDD_IN (mW) | sd (mW) |")
print("|---|---|---|---|---|---|")
stats = {}
for name, path, color in [("idle", args.idle, "#4C78A8"), ("stress", args.stress, "#E45756")]:
    t, p = load(path)
    dt = np.diff(t)
    med = float(np.median(dt)) if len(dt) else float("nan")
    stats[name] = p
    print(f"| {name} | {len(t)} | {med * 1e3:.1f} | {1 / med:.1f} | {p.mean():.0f} | {p.std(ddof=1):.0f} |")
    ax.plot(t - t[0], p, lw=0.8, color=color, label=f"{name} (mean {p.mean():.0f} mW)")

sep = stats["stress"].mean() - stats["idle"].mean()
print(f"\nstress - idle = {sep:.0f} mW")
if np.median(np.diff(load(args.idle)[0])) > 0.15:
    print("NOTE: achieved rate is well below 10 Hz; Module 14 may need to read the INA3221 sysfs nodes directly.")

ax.set_xlabel("time since phase start (s)")
ax.set_ylabel("VDD_IN (mW)")
ax.set_title("Jetson power smoke test: idle vs. stress-ng --cpu 6")
ax.legend()
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(args.out, dpi=150)
print(f"saved {args.out}")

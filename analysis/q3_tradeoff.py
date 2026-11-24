"""Q3 / H3: the quality-energy trade-off, Block E (Module 20).

Pareto plot per map: x = planner energy above idle (J), y = d_95 (m), one point per
method x power mode (medians over poses), bootstrap 95% CI whiskers on both axes.

Break-even, per map x power mode x info-aware method, paired by pose:
    dE = E_board[info] - E_board[nearest]      extra compute-side energy (J)
    dD = d[nearest] - d[info]                  distance saved (m)
    k* = median(dE) / median(dD)               J/m; info-aware wins when k_loco > k*
If median(dD) <= 0 there is no break-even (nearest wins at any k_loco). If median(dE) <= 0
while median(dD) > 0, the info-aware method uses less board energy AND drives less: it
dominates at any k_loco >= 0 (k* <= 0, reported as `dominates`). CI: paired bootstrap
over poses. Mission energy E_mission = E_board + k_loco * d is swept over K_LOCO_RANGE.
"""
from __future__ import annotations

import math
import os

import numpy as np
import pandas as pd

from stats import bootstrap_ci, ratio_of_medians_ci

# TEAM DECISION (docs/preregistration.md §2): sweep range, and the smallest realistic k_loco for a
# TurtleBot-class base with its citation. Until K_LOCO_MIN is set, h3_supported stays None.
K_LOCO_RANGE = (0.0, 30.0)
K_LOCO_MIN = None
COLORS = {"nearest": "#4c72b0", "mrtsp": "#dd8452", "ariadne": "#55a868", "roadmap": "#c44e52"}


def analyse(en: pd.DataFrame) -> dict:
    out = {"k_loco_range_J_per_m": list(K_LOCO_RANGE), "k_loco_min_J_per_m": K_LOCO_MIN,
           "points": {}, "break_even": {}}
    if en.empty:
        return out
    for (method, mp, pm), g in en.groupby(["method", "map", "power_mode"], observed=True):
        ex = g["energy_above_idle_J"].to_numpy(float)
        dd = g["d"].to_numpy(float)
        out["points"].setdefault(mp, {}).setdefault(pm, {})[str(method)] = {
            "n": int(len(g)),
            "energy_above_idle_J": float(np.nanmedian(ex)) if np.isfinite(ex).any() else None,
            "energy_ci": list(bootstrap_ci(ex)),
            "d_m": float(np.median(dd)), "d_ci": list(bootstrap_ci(dd)),
            "energy_board_J": float(g["energy_board_J"].median()),
        }
    for (mp, pm), g in en.groupby(["map", "power_mode"]):
        base = g[g["method"] == "nearest"].set_index("pose")
        for method in [m for m in g["method"].unique() if m != "nearest"]:
            o = g[g["method"] == method].set_index("pose")
            common = base.index.intersection(o.index)
            if len(common) == 0:
                continue
            dE = o.loc[common, "energy_board_J"].to_numpy(float) - base.loc[common, "energy_board_J"].to_numpy(float)
            dD = base.loc[common, "d"].to_numpy(float) - o.loc[common, "d"].to_numpy(float)
            k, (lo, hi) = ratio_of_medians_ci(dE, dD)
            res = {"n_pairs": int(len(common)), "dE_median_J": float(np.median(dE)), "dD_median_m": float(np.median(dD)),
                   "k_star_J_per_m": None if not math.isfinite(k) else k,
                   "k_star_ci": [None if not math.isfinite(v) else v for v in (lo, hi)],
                   "break_even_exists": bool(np.median(dD) > 0),
                   "dominates": bool(np.median(dD) > 0 and np.median(dE) <= 0)}
            res["h3_supported"] = None if K_LOCO_MIN is None else \
                bool(res["break_even_exists"] and math.isfinite(hi) and hi < K_LOCO_MIN)
            out["break_even"].setdefault(mp, {}).setdefault(pm, {})[str(method)] = res
    return out


def figures(en: pd.DataFrame, res: dict, out_dir: str) -> list:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    paths = []
    if en.empty:
        return paths
    os.makedirs(out_dir, exist_ok=True)
    maps = sorted(res["points"])
    fig, axes = plt.subplots(1, len(maps), figsize=(5 * len(maps), 4.2), squeeze=False)
    modes = sorted(en["power_mode"].unique())
    mk = {pm: m for pm, m in zip(modes, ["o", "s", "^", "D"])}
    for ax, mp in zip(axes[0], maps):
        for pm, by_m in res["points"][mp].items():
            for method, p in by_m.items():
                if p["energy_above_idle_J"] is None:
                    continue
                x, y = p["energy_above_idle_J"], p["d_m"]
                xe = [[x - p["energy_ci"][0]], [p["energy_ci"][1] - x]]
                ye = [[y - p["d_ci"][0]], [p["d_ci"][1] - y]]
                ax.errorbar(x, y, xerr=xe, yerr=ye, fmt=mk[pm], color=COLORS.get(method), capsize=3,
                            label=f"{method}, {pm}")
        ax.set_title(mp)
        ax.set_xlabel("planner energy above idle (J)")
        ax.set_ylabel("d_95 (m)")
    axes[0][-1].legend(fontsize=7)
    fig.suptitle("Q3: quality vs. energy (medians over poses, 95% bootstrap CI), Block E")
    fig.tight_layout()
    p = os.path.join(out_dir, "fig_q3_pareto.png")
    fig.savefig(p, dpi=140)
    plt.close(fig)
    paths.append(p)

    # Mission energy vs k_loco
    k = np.linspace(*K_LOCO_RANGE, 61)
    fig, axes = plt.subplots(len(modes), len(maps), figsize=(5 * len(maps), 3.6 * len(modes)), squeeze=False)
    for i, pm in enumerate(modes):
        for j, mp in enumerate(maps):
            ax = axes[i][j]
            for method, p in res["points"].get(mp, {}).get(pm, {}).items():
                ax.plot(k, p["energy_board_J"] + k * p["d_m"], color=COLORS.get(method), label=method)
            for method, b in res["break_even"].get(mp, {}).get(pm, {}).items():
                ks = b["k_star_J_per_m"]
                if ks is not None and K_LOCO_RANGE[0] <= ks <= K_LOCO_RANGE[1]:
                    ax.axvline(ks, color=COLORS.get(method), ls=":", lw=1)
                    lo, hi = b["k_star_ci"]
                    if lo is not None and hi is not None:
                        ax.axvspan(max(lo, k[0]), min(hi, k[-1]), color=COLORS.get(method), alpha=0.08)
            ax.set_title(f"{mp}, {pm}", fontsize=9)
            ax.set_xlabel("k_loco (J/m)")
            ax.set_ylabel("E_mission (J)")
    axes[0][-1].legend(fontsize=7)
    fig.suptitle("Q3: mission energy E_board + k_loco * d, break-even k* (dotted, CI shaded)")
    fig.tight_layout()
    p = os.path.join(out_dir, "fig_q3_break_even.png")
    fig.savefig(p, dpi=140)
    plt.close(fig)
    paths.append(p)
    return paths

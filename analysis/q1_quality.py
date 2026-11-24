"""Q1 / H1: exploration quality, Block Q (Module 20).

For each map and info-aware method: paired differences d_95[method] - d_95[nearest] over
poses (censored runs use their total distance, flagged), bootstrap 95% CI of the median
difference, Wilcoxon signed-rank p, Holm across the 3 maps (per method). Figure: coverage
vs. distance, one panel per map, median line and IQR band per method.
"""
from __future__ import annotations

import os

import numpy as np
import pandas as pd

from load import load_coverage, paired
from stats import holm, paired_effect

COLORS = {"nearest": "#4c72b0", "mrtsp": "#dd8452", "ariadne": "#55a868", "roadmap": "#c44e52"}


def analyse(runs: pd.DataFrame, value="d_95_or_total") -> dict:
    out = {}
    if runs.empty:
        return out
    methods = [m for m in runs["method"].cat.categories if m != "nearest"]
    maps = sorted(runs["map"].unique())
    for method in methods:
        cells = {}
        for mp in maps:
            w = paired(runs[runs["map"] == mp], value=value, keys=("pose",))
            if method not in w or "nearest" not in w:
                continue
            cens = w.get(f"{method}_censored", pd.Series(False, index=w.index)).fillna(False).astype(bool) | \
                w.get("nearest_censored", pd.Series(False, index=w.index)).fillna(False).astype(bool)
            cells[mp] = paired_effect(w["nearest"].to_numpy(), w[method].to_numpy(), cens.to_numpy())
        adj = holm([cells[mp]["p"] for mp in cells])
        for mp, pa in zip(cells, adj):
            cells[mp]["p_holm"] = float(pa)
            c = cells[mp]
            c["h1_supported"] = bool(np.isfinite(pa) and pa < 0.05 and c["ci_high"] < 0)
        out[method] = cells
    out["_outcomes"] = {f"{mp}/{m}": runs[(runs["map"] == mp) & (runs["method"] == m)]["outcome"].value_counts().to_dict()
                        for mp in maps for m in runs["method"].cat.categories}
    return out


def figure(runs: pd.DataFrame, path: str, grid_step=0.5):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cov = load_coverage(runs)
    if cov.empty:
        return None
    maps = sorted(cov["map"].unique())
    fig, axes = plt.subplots(1, len(maps), figsize=(5 * len(maps), 4), squeeze=False)
    for ax, mp in zip(axes[0], maps):
        c = cov[cov["map"] == mp]
        dmax = c.groupby("run_id")["distance_m"].max().median() * 1.3
        grid = np.arange(0, dmax + grid_step, grid_step)
        for method, g in c.groupby("method", observed=True):
            curves = []
            for _, r in g.groupby("run_id"):
                # a run that ended keeps its last coverage to the right of its end
                curves.append(np.interp(grid, r["distance_m"], r["coverage_frac"], right=r["coverage_frac"].iloc[-1]))
            if not curves:
                continue
            a = np.vstack(curves)
            q25, q50, q75 = np.percentile(a, [25, 50, 75], axis=0)
            col = COLORS.get(method, None)
            ax.plot(grid, q50, color=col, label=f"{method} (n={len(curves)})")
            ax.fill_between(grid, q25, q75, color=col, alpha=0.2, lw=0)
        ax.axhline(0.95, ls="--", c="gray", lw=0.8)
        ax.set_title(mp)
        ax.set_xlabel("distance travelled (m)")
        ax.set_ylim(0, 1.0)
    axes[0][0].set_ylabel("coverage (fraction of GT reference cells)")
    axes[0][-1].legend(loc="lower right", fontsize=8)
    fig.suptitle("Q1: coverage vs. distance (median, IQR band), Block Q")
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path

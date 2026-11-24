"""H2: does decision time grow with the number of frontiers? (Module 20; reused by Module 21)

Per method x power mode: scatter of decision latency vs. n_frontiers over all planning
cycles, log-log OLS slope with a cluster bootstrap 95% CI over runs, and the
replan_period_s deadline as a horizontal line. Note: n_frontiers is clusters for nearest
and mrtsp but frontier cells for ariadne (docs/planner_timing.md), so slopes compare across
power modes within a method, not across methods.
"""
from __future__ import annotations

import os

import numpy as np
import pandas as pd

from load import load_cycles
from stats import loglog_slope

COLORS = {"nearest": "#4c72b0", "mrtsp": "#dd8452", "ariadne": "#55a868", "roadmap": "#c44e52"}


def analyse(runs: pd.DataFrame) -> tuple[dict, pd.DataFrame]:
    cyc = load_cycles(runs)
    out = {}
    if cyc.empty:
        return out, cyc
    for (method, pm), g in cyc.groupby(["method", "power_mode"], observed=True):
        s = loglog_slope(g)
        s["h2_supported"] = bool(np.isfinite(s["ci_low"]) and s["ci_low"] > 0)
        out.setdefault(str(method), {})[pm] = s
    return out, cyc


def figure(cyc: pd.DataFrame, res: dict, path: str, replan_period_s: float = 1.0, title_block="E"):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    if cyc.empty:
        return None
    methods = [m for m in res]
    modes = sorted(cyc["power_mode"].unique())
    fig, axes = plt.subplots(1, len(methods), figsize=(4.6 * len(methods), 4), squeeze=False)
    for ax, m in zip(axes[0], methods):
        for k, pm in enumerate(modes):
            g = cyc[(cyc["method"] == m) & (cyc["power_mode"] == pm) & (cyc["n_frontiers"] > 0)]
            if g.empty:
                continue
            ax.scatter(g["n_frontiers"], 1000 * g["latency_s"], s=4, alpha=0.3, label=pm,
                       color=plt.cm.viridis(k / max(1, len(modes) - 1)))
            s = res[m].get(pm)
            if s and np.isfinite(s["slope"]):
                xs = np.linspace(g["n_frontiers"].min(), g["n_frontiers"].max(), 20)
                ax.plot(xs, 1000 * np.exp(s["intercept"]) * xs ** s["slope"],
                        color=plt.cm.viridis(k / max(1, len(modes) - 1)), lw=1.5,
                        label=f"{pm}: slope {s['slope']:.2f} [{s['ci_low']:.2f}, {s['ci_high']:.2f}]")
        ax.axhline(1000 * replan_period_s, ls="--", c="red", lw=0.8)
        ax.set_xscale("log")
        ax.set_yscale("log")
        ax.set_title(m)
        ax.set_xlabel("n_frontiers")
        ax.legend(fontsize=7)
    axes[0][0].set_ylabel("decision latency (ms)")
    fig.suptitle(f"H2: latency vs. number of frontiers (log-log), Block {title_block}; red = replan_period_s")
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path

"""Q2: compute and energy, Block E (Module 20).

Per method x map x power mode: decision latency (median and p95 over all cycles), deadline-
miss rate (latency > replan_period_s), planner CPU time, board energy and energy above idle.

Idle baseline for a run: the idle runs of the same power mode closest in time before and
after it (within `max_gap_h`), averaged when both exist (docs/block_e_procedure.md step 6).
"""
from __future__ import annotations

import os

import numpy as np
import pandas as pd
import yaml

import energy
from load import load_cycles


def match_idle(run_t0, run_t1, power_mode, idle: pd.DataFrame, max_gap_h=12.0):
    """-> (mean idle power W, [idle ids]) or (nan, [])."""
    if idle.empty:
        return float("nan"), []
    cand = idle[idle["power_mode"] == power_mode]
    before = cand[cand["t_wall_end"] <= run_t0]
    after = cand[cand["t_wall_start"] >= run_t1]
    picks = []
    if len(before):
        b = before.loc[before["t_wall_end"].idxmax()]
        if run_t0 - b["t_wall_end"] <= max_gap_h * 3600:
            picks.append(b)
    if len(after):
        f = after.loc[after["t_wall_start"].idxmin()]
        if f["t_wall_start"] - run_t1 <= max_gap_h * 3600:
            picks.append(f)
    if not picks:
        return float("nan"), []
    return float(np.mean([p["power_idle_W"] for p in picks])), [p["idle_id"] for p in picks]


def per_run_energy(runs: pd.DataFrame, idle: pd.DataFrame) -> pd.DataFrame:
    rows = []
    for _, r in runs.iterrows():
        try:
            e = energy.run_energy(r["dir"])
        except ValueError:
            continue
        pw = energy.read_csv(os.path.join(r["dir"], "power.csv"))
        with open(os.path.join(r["dir"], "manifest.yaml")) as f:
            m = yaml.safe_load(f) or {}
        t0 = float(m.get("t_wall_start") or pw["t_wall"][0])
        t1 = float(m.get("t_wall_end") or pw["t_wall"][-1])
        p_idle, ids = match_idle(t0, t1, r["power_mode"], idle)
        rows.append({"run_id": r["run_id"], "method": r["method"], "map": r["map"], "pose": r["pose"],
                     "power_mode": r["power_mode"], "d": r["d_95_or_total"], "censored": r["censored"],
                     "energy_board_J": e["energy_board_J"], "window_s": e["energy_window_s"],
                     "power_idle_W": p_idle, "idle_ids": ",".join(ids),
                     "energy_above_idle_J": e["energy_board_J"] - p_idle * (t1 - t0) if np.isfinite(p_idle) else np.nan,
                     "planner_cpu_s": energy.cpu_seconds(energy.read_csv(os.path.join(r["dir"], "procs.csv")),
                                                         m.get("planner_process", ""), t0, t1)})
    return pd.DataFrame(rows)


def analyse(runs: pd.DataFrame, idle: pd.DataFrame) -> tuple[dict, pd.DataFrame]:
    out = {}
    if runs.empty:
        return out, pd.DataFrame()
    cyc = load_cycles(runs)
    en = per_run_energy(runs, idle)
    for (method, mp, pm), g in runs.groupby(["method", "map", "power_mode"], observed=True):
        c = cyc[(cyc["method"] == method) & (cyc["map"] == mp) & (cyc["power_mode"] == pm)] if not cyc.empty else cyc
        e = en[(en["method"] == method) & (en["map"] == mp) & (en["power_mode"] == pm)] if not en.empty else en
        lat = c["latency_s"].to_numpy() if len(c) else np.array([])
        replan = float(g["replan_period_s"].iloc[0])
        out.setdefault(str(method), {}).setdefault(mp, {})[pm] = {
            "n_runs": int(len(g)), "n_cycles": int(len(lat)),
            "latency_median_s": float(np.median(lat)) if len(lat) else None,
            "latency_p95_s": float(np.percentile(lat, 95)) if len(lat) else None,
            "deadline_miss_rate": float(np.mean(lat > replan)) if len(lat) else None,
            "replan_period_s": replan,
            "planner_cpu_s_median": float(e["planner_cpu_s"].median()) if len(e) else None,
            "energy_board_J_median": float(e["energy_board_J"].median()) if len(e) else None,
            "energy_above_idle_J_median": float(e["energy_above_idle_J"].median()) if len(e) and e["energy_above_idle_J"].notna().any() else None,
            "runs_without_idle": int(e["energy_above_idle_J"].isna().sum()) if len(e) else 0,
        }
    return out, en


def figure(runs: pd.DataFrame, path: str):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    cyc = load_cycles(runs)
    if cyc.empty:
        return None
    modes = sorted(cyc["power_mode"].unique())
    methods = [m for m in cyc["method"].cat.categories if m in set(cyc["method"])] \
        if hasattr(cyc["method"], "cat") else sorted(cyc["method"].unique())
    fig, ax = plt.subplots(figsize=(1.8 * len(methods) * len(modes) + 2, 4))
    data, labels = [], []
    for m in methods:
        for pm in modes:
            data.append(1000 * cyc[(cyc["method"] == m) & (cyc["power_mode"] == pm)]["latency_s"].to_numpy())
            labels.append(f"{m}\n{pm}")
    ax.boxplot(data, showfliers=False)       # tick labels set separately: matplotlib 3.5 and 3.9+ differ
    ax.set_xticks(range(1, len(labels) + 1))
    ax.set_xticklabels(labels)
    ax.set_yscale("log")
    replan = float(runs["replan_period_s"].iloc[0]) * 1000
    ax.axhline(replan, ls="--", c="red", lw=0.8, label=f"replan_period_s = {replan:.0f} ms")
    ax.set_ylabel("decision latency (ms, wall)")
    ax.set_title("Q2: decision latency per planning cycle, Block E")
    ax.legend(fontsize=8)
    fig.tight_layout()
    os.makedirs(os.path.dirname(path), exist_ok=True)
    fig.savefig(path, dpi=140)
    plt.close(fig)
    return path

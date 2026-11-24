#!/usr/bin/env python3
"""Load every run under logs/ into one tidy table (Module 20; also used by the pilot, Module 17).

    from load import load_runs, load_cycles, load_coverage
    runs = load_runs("logs")                  # one row per run folder

    python3 analysis/load.py [logs] [--block Q]   # print a per-cell summary

A run folder is any logs/<run_id>/ with a manifest.yaml whose method is not `none`.
Idle baselines (logs/idle_<mode>_<ts>/) come from load_idle(). Columns of load_runs():

  run_id, block, method, map, pose, power_mode, outcome, reason, censored,
  d_95, t_95, d_95_or_total, t_95_or_total, final_coverage, total_distance_m, total_time_s,
  oscillation_count, rtf, wall_duration_s, n_cycles, latency_median_s, latency_p95_s,
  deadline_miss_rate, plus every energy_* / planner_cpu_s key found in summary.json.

`censored` is True when 95% coverage was not reached (timeout, stuck). For paired tests the
pre-registration (Module 18) uses d_95_or_total: the run's total distance stands in for
d_95, and the run stays flagged as censored.
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np
import pandas as pd
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
METHOD_ORDER = ["nearest", "mrtsp", "ariadne", "roadmap"]
MAP_ORDER = ["m1_house", "m2_office", "m3_warehouse", "m2_office_2x"]


def _yaml(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def _json(path):
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        return json.load(f)


def read_csv(path) -> pd.DataFrame:
    if not os.path.exists(path) or os.path.getsize(path) == 0:
        return pd.DataFrame()
    try:
        return pd.read_csv(path)
    except pd.errors.EmptyDataError:
        return pd.DataFrame()


def cycle_latencies(run_dir: str) -> pd.DataFrame:
    """cycles.csv with latency_s = wall_end - wall_start (decision latency, Module 20 Q2)."""
    c = read_csv(os.path.join(run_dir, "cycles.csv"))
    if c.empty:
        return c
    c["latency_s"] = c["t_wall_end"] - c["t_wall_start"]
    return c


def run_dirs(logs: str):
    if not os.path.isdir(logs):
        return []
    out = []
    for name in sorted(os.listdir(logs)):
        d = os.path.join(logs, name)
        if os.path.isfile(os.path.join(d, "manifest.yaml")) and not name.startswith("idle_"):
            out.append(d)
    return out


def load_runs(logs: str = os.path.join(REPO, "logs"), block: str | None = None) -> pd.DataFrame:
    rows = []
    for d in run_dirs(logs):
        m = _yaml(os.path.join(d, "manifest.yaml"))
        if m.get("method") in (None, "none") or (block and m.get("block") != block):
            continue
        s = _json(os.path.join(d, "summary.json"))
        replan = float((m.get("common") or {}).get("replan_period_s", 1.0))
        cyc = cycle_latencies(d)
        lat = cyc["latency_s"].to_numpy() if not cyc.empty else np.array([])
        outcome = m.get("outcome") or s.get("outcome")
        row = {
            "run_id": m.get("run_id", os.path.basename(d)),
            "block": m.get("block"), "method": m.get("method"), "map": m.get("map"),
            "pose": m.get("pose_index"), "power_mode": m.get("power_mode"),
            "outcome": outcome, "reason": m.get("reason"),
            "censored": outcome in ("timeout", "stuck"),
            "d_95": s.get("d_95"), "t_95": s.get("t_95"),
            "final_coverage": s.get("final_coverage"), "total_distance_m": s.get("total_distance_m"),
            "total_time_s": s.get("total_time_s"), "oscillation_count": s.get("oscillation_count"),
            "rtf": m.get("rtf"), "wall_duration_s": m.get("wall_duration_s"),
            "n_cycles": int(len(lat)),
            "latency_median_s": float(np.median(lat)) if len(lat) else np.nan,
            "latency_p95_s": float(np.percentile(lat, 95)) if len(lat) else np.nan,
            "deadline_miss_rate": float(np.mean(lat > replan)) if len(lat) else np.nan,
            "replan_period_s": replan,
            "dir": d,
        }
        for k, v in s.items():
            if k.startswith("energy_") or k in ("planner_cpu_s", "power_mean_W", "power_idle_W", "power_sample_hz"):
                row[k] = v
        rows.append(row)
    df = pd.DataFrame(rows)
    if df.empty:
        return df
    for col in ("d_95", "t_95", "final_coverage", "total_distance_m", "total_time_s"):
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df["d_95_or_total"] = df["d_95"].where(df["d_95"].notna(), df["total_distance_m"])
    df["t_95_or_total"] = df["t_95"].where(df["t_95"].notna(), df["total_time_s"])
    df["method"] = pd.Categorical(df["method"], [m for m in METHOD_ORDER if m in set(df["method"])])
    return df.sort_values(["block", "map", "pose", "method", "power_mode"]).reset_index(drop=True)


def load_idle(logs: str = os.path.join(REPO, "logs")) -> pd.DataFrame:
    """One row per idle baseline: power_mode, mean VDD_IN (W), duration, start time."""
    sys.path.insert(0, HERE)
    import energy
    rows = []
    for name in sorted(os.listdir(logs)) if os.path.isdir(logs) else []:
        d = os.path.join(logs, name)
        if not name.startswith("idle_") or not os.path.isfile(os.path.join(d, "power.csv")):
            continue
        m = _yaml(os.path.join(d, "manifest.yaml")) if os.path.exists(os.path.join(d, "manifest.yaml")) else {}
        pw = energy.read_csv(os.path.join(d, "power.csv"))
        rows.append({"idle_id": name, "power_mode": m.get("power_mode", name.split("_")[1]),
                     "t_wall_start": float(pw["t_wall"][0]), "t_wall_end": float(pw["t_wall"][-1]),
                     "power_idle_W": energy.mean_power_W(pw["t_wall"], pw["vdd_in_mW"]), "dir": d})
    return pd.DataFrame(rows)


def load_cycles(runs: pd.DataFrame) -> pd.DataFrame:
    """All planning cycles of the given runs, with run keys attached (for H2)."""
    parts = []
    for _, r in runs.iterrows():
        c = cycle_latencies(r["dir"])
        if c.empty:
            continue
        for k in ("run_id", "block", "method", "map", "pose", "power_mode"):
            c[k] = r[k]
        parts.append(c)
    return pd.concat(parts, ignore_index=True) if parts else pd.DataFrame()


def load_coverage(runs: pd.DataFrame) -> pd.DataFrame:
    """coverage.csv of the given runs (t_ros made relative to the run's start)."""
    parts = []
    for _, r in runs.iterrows():
        c = read_csv(os.path.join(r["dir"], "coverage.csv"))
        if c.empty:
            continue
        c["t_s"] = c["t_ros"] - c["t_ros"].iloc[0]
        for k in ("run_id", "method", "map", "pose", "power_mode", "block"):
            c[k] = r[k]
        parts.append(c)
    return pd.concat(parts, ignore_index=True) if parts else pd.DataFrame()


def paired(runs: pd.DataFrame, value: str = "d_95_or_total", baseline: str = "nearest",
           keys=("map", "pose", "power_mode")) -> pd.DataFrame:
    """Wide table: one row per (map, pose[, power_mode]), one column per method, plus a
    `<method>_censored` flag. Only pairs where the baseline and the method both ran."""
    keys = [k for k in keys if k in runs.columns]
    wide = runs.pivot_table(index=keys, columns="method", values=value, aggfunc="first", observed=True)
    cens = runs.pivot_table(index=keys, columns="method", values="censored", aggfunc="first", observed=True)
    cens.columns = [f"{c}_censored" for c in cens.columns]
    out = wide.join(cens).reset_index()
    return out[out[baseline].notna()] if baseline in out else out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("logs", nargs="?", default=os.path.join(REPO, "logs"))
    ap.add_argument("--block")
    a = ap.parse_args(argv)
    runs = load_runs(a.logs, a.block)
    if runs.empty:
        print("no runs found")
        return 0
    g = runs.groupby(["block", "map", "method", "power_mode"], observed=True)
    print(g.agg(n=("run_id", "size"), complete=("outcome", lambda s: int((s == "complete").sum())),
                d95_median=("d_95", "median"), latency_p95=("latency_p95_s", "median")).to_string())
    return 0


if __name__ == "__main__":
    sys.exit(main())

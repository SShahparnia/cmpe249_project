#!/usr/bin/env python3
"""Energy and CPU-time helpers (Module 14). Everyone computes energy with these.

    from energy import energy_J, run_energy

    python3 analysis/energy.py logs/<run_id> [--idle logs/idle_<mode>_<ts>]

Definitions (instructions.md Module 14):

    E_run      = integral of VDD_IN over [t0, t1]                    (trapezoid, J)
    P_idle     = mean VDD_IN of the idle-stack baseline of the same session and power mode
    E_above    = E_run - P_idle * (t1 - t0)
    CPU time   = last minus first (utime + stime) of a process in procs.csv

[t0, t1] is the planner's active window in wall time: manifest `t_wall_start` /
`t_wall_end` when the orchestrator wrote them, otherwise the first / last power sample.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np

_trapz = getattr(np, "trapezoid", None) or np.trapz       # numpy >= 2 renamed trapz


def energy_J(t_wall, power_mW, t0, t1) -> float:
    """Trapezoid integral of power (mW) over [t0, t1] (s), in joules. The window is clipped
    to the samples (no extrapolation); fewer than two samples inside gives 0."""
    t = np.asarray(t_wall, dtype=float)
    p = np.asarray(power_mW, dtype=float)
    m = (t >= t0) & (t <= t1) & np.isfinite(p)
    if m.sum() < 2:
        return 0.0
    return float(_trapz(p[m] / 1000.0, t[m]))


def mean_power_W(t_wall, power_mW, t0=-np.inf, t1=np.inf) -> float:
    """Time-weighted mean power (W) over [t0, t1]: energy / covered duration."""
    t = np.asarray(t_wall, dtype=float)
    m = (t >= t0) & (t <= t1) & np.isfinite(np.asarray(power_mW, dtype=float))
    if m.sum() < 2:
        return float("nan")
    tt = t[m]
    return energy_J(t_wall, power_mW, tt[0], tt[-1]) / (tt[-1] - tt[0])


def energy_above_idle(e_run_J, p_idle_W, t0, t1) -> float:
    return float(e_run_J - p_idle_W * (t1 - t0))


def read_csv(path: str) -> dict:
    """CSV -> {column: array} (floats where possible). Missing or empty file -> {}."""
    if not os.path.exists(path):
        return {}
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        return {}
    out = {}
    for k in rows[0]:
        vals = [r[k] for r in rows]
        try:
            out[k] = np.array([float(v) if v not in ("", None) else np.nan for v in vals])
        except ValueError:
            out[k] = np.array(vals, dtype=str)
    return out


def cpu_seconds(procs: dict, name: str, t0=-np.inf, t1=np.inf) -> float:
    """CPU seconds (user + system) used by processes matching `name` in [t0, t1]. Each pid
    contributes last - first within the window, so a restarted node is summed correctly."""
    if not procs or "name" not in procs:
        return 0.0
    m = (procs["name"] == name) & (procs["t_wall"] >= t0) & (procs["t_wall"] <= t1)
    total = 0.0
    for pid in np.unique(procs["pid"][m]):
        k = m & (procs["pid"] == pid)
        cpu = procs["utime_s"][k] + procs["stime_s"][k]
        total += float(cpu.max() - cpu.min())
    return total


def _manifest(run_dir: str) -> dict:
    p = os.path.join(run_dir, "manifest.yaml")
    if not os.path.exists(p):
        return {}
    import yaml
    with open(p) as f:
        return yaml.safe_load(f) or {}


def idle_power_W(idle_dir: str) -> float:
    pw = read_csv(os.path.join(idle_dir, "power.csv"))
    if not pw:
        raise ValueError(f"{idle_dir}: no power.csv")
    return mean_power_W(pw["t_wall"], pw["vdd_in_mW"])


def run_energy(run_dir: str, idle_dir: str | None = None, planner_proc: str | None = None) -> dict:
    """Energy summary for one run folder (keys go into summary.json as energy_*)."""
    man = _manifest(run_dir)
    pw = read_csv(os.path.join(run_dir, "power.csv"))
    if not pw:
        raise ValueError(f"{run_dir}: no power.csv")
    t = pw["t_wall"]
    t0 = float(man.get("t_wall_start") or t[0])
    t1 = float(man.get("t_wall_end") or t[-1])
    e_run = energy_J(t, pw["vdd_in_mW"], t0, t1)
    out = {
        "energy_window_s": t1 - t0,
        "energy_board_J": e_run,
        "energy_cpu_gpu_cv_J": energy_J(t, pw["vdd_cpu_gpu_cv_mW"], t0, t1),
        "energy_soc_J": energy_J(t, pw["vdd_soc_mW"], t0, t1),
        "power_mean_W": mean_power_W(t, pw["vdd_in_mW"], t0, t1),
        "power_sample_hz": float((len(t) - 1) / (t[-1] - t[0])) if len(t) > 1 else float("nan"),
    }
    if idle_dir:
        p_idle = idle_power_W(idle_dir)
        out.update({"energy_idle_dir": os.path.basename(os.path.normpath(idle_dir)),
                    "power_idle_W": p_idle,
                    "energy_above_idle_J": energy_above_idle(e_run, p_idle, t0, t1)})
    proc = planner_proc or man.get("planner_process")
    if proc:
        out["planner_cpu_s"] = cpu_seconds(read_csv(os.path.join(run_dir, "procs.csv")), proc, t0, t1)
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run_dir")
    ap.add_argument("--idle", help="idle-baseline folder of the same session and power mode")
    ap.add_argument("--planner-proc", help="procs.csv name of the planner (default: manifest planner_process)")
    ap.add_argument("--write", action="store_true", help="merge the result into <run_dir>/summary.json")
    a = ap.parse_args(argv)
    res = run_energy(a.run_dir, a.idle, a.planner_proc)
    print(json.dumps(res, indent=2))
    if a.write:
        path = os.path.join(a.run_dir, "summary.json")
        summary = {}
        if os.path.exists(path):
            with open(path) as f:
                summary = json.load(f)
        summary.update(res)
        with open(path, "w") as f:
            json.dump(summary, f, indent=2)
            f.write("\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())

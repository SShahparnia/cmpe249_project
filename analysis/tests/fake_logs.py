"""Synthetic run folders in the Section 0 formats, for testing the analysis without a sim.

    from fake_logs import make_block
    make_block(logs_dir, block="P", poses=range(3), effects={"mrtsp": -0.15, "ariadne": -0.05})

Each run gets manifest.yaml, summary.json, coverage.csv, cycles.csv, goals.csv,
nav2_results.csv and pose.csv; Block E/A runs also get power.csv and procs.csv. Distances
follow d_95[method] = d_95[nearest] * (1 + effect + noise), so tests know the true effect.
"""
from __future__ import annotations

import csv
import json
import math
import os

import numpy as np
import yaml

MAPS = {"m1_house": 60.0, "m2_office": 45.0, "m3_warehouse": 80.0}      # nearest d_95 baseline (m)
PLANNER_PROC = {"nearest": "explore", "mrtsp": "frontier_explorer", "ariadne": "rl_planner"}
LATENCY = {"nearest": (0.004, 0.00005), "mrtsp": (0.010, 0.0004), "ariadne": (0.150, 0.0003)}   # base, per frontier
PLANNER_W = {"nearest": 0.15, "mrtsp": 0.35, "ariadne": 1.6}            # extra board power while planning (W)


def _w(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


def make_run(logs, block, method, map_name, pose, power_mode, d95, outcome="complete", seed=0,
             t0_wall=1.8e9, speed=0.22, idle_W=5.0):
    rng = np.random.default_rng(seed)
    run_id = f"{block}_{method}_{map_name}_p{pose:02d}_{power_mode}_20261201-{pose:02d}{seed % 60:02d}00"
    d = os.path.join(logs, run_id)
    os.makedirs(os.path.join(d, "maps"), exist_ok=True)
    t_ros0 = 50.0
    total_d = d95 * (1.25 if outcome == "complete" else 1.0)
    total_t = total_d / speed
    # coverage vs distance: 1 - exp(-k d) scaled so that it reaches 0.95 at d95 when complete
    k = -math.log(0.05) / d95
    ts = np.arange(0.0, total_t + 1e-9, 5.0)
    dist = ts * speed
    cov = 1.0 - np.exp(-k * dist)
    if outcome != "complete":
        cov = np.minimum(cov, 0.93)
    _w(os.path.join(d, "coverage.csv"), ["t_ros", "distance_m", "coverage_frac"],
       [[t_ros0 + t, dd, c] for t, dd, c in zip(ts, dist, cov)])
    # pose.csv at 1 Hz along a straight line (enough for loaders)
    tp = np.arange(0.0, total_t, 1.0)
    _w(os.path.join(d, "pose.csv"), ["t_wall", "t_ros", "x", "y", "yaw"],
       [[t0_wall + t, t_ros0 + t, speed * t, 0.0, 0.0] for t in tp])
    # planning cycles: n_frontiers rises then falls; latency grows with it
    base, slope = LATENCY[method]
    n_cyc = max(5, int(total_t / 2.0))
    cyc, goals = [], []
    for i in range(n_cyc):
        tr = t_ros0 + i * total_t / n_cyc
        nf = int(5 + 25 * math.sin(math.pi * i / n_cyc)) + int(rng.integers(0, 3))
        lat = base + slope * nf * (nf if method == "mrtsp" else 1) + rng.exponential(0.002)
        tw = t0_wall + (tr - t_ros0)
        gx, gy = float(rng.uniform(-5, 5)), float(rng.uniform(-5, 5))
        cyc.append([i, tw, tw + lat, tr, tr + lat, nf, gx, gy])
        goals.append([tw + lat, tr + lat, i, gx, gy])
    _w(os.path.join(d, "cycles.csv"), ["cycle_id", "t_wall_start", "t_wall_end", "t_ros_start", "t_ros_end",
                                       "n_frontiers", "goal_x", "goal_y"], cyc)
    _w(os.path.join(d, "goals.csv"), ["t_wall", "t_ros", "cycle_id", "goal_x", "goal_y"], goals)
    _w(os.path.join(d, "nav2_results.csv"), ["t_wall", "t_ros", "goal_id", "status"],
       [[g[0] + 1, g[1] + 1, f"{i:032x}", 4] for i, g in enumerate(goals[::3])])
    manifest = {"run_id": run_id, "block": block, "method": method, "map": map_name, "pose_index": pose,
                "start_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0}, "power_mode": power_mode,
                "outcome": outcome, "reason": "synthetic", "t_ros_start": t_ros0,
                "t_wall_start": t0_wall, "t_wall_end": t0_wall + total_t,
                "rtf": 1.0, "wall_duration_s": total_t, "sim_duration_s": total_t,
                "planner_process": PLANNER_PROC[method], "common": {"replan_period_s": 1.0}}
    with open(os.path.join(d, "manifest.yaml"), "w") as f:
        yaml.safe_dump(manifest, f)
    summary = {"t_95": (d95 / speed) if outcome == "complete" else None,
               "d_95": d95 if outcome == "complete" else None,
               "final_coverage": float(cov[-1]), "total_distance_m": float(total_d),
               "total_time_s": float(total_t), "outcome": outcome, "oscillation_count": int(rng.integers(0, 4)),
               "oscillation_threshold_m": 1.0}
    if block in ("E", "A"):
        tw = np.arange(t0_wall - 1, t0_wall + total_t + 1, 0.1)
        busy = np.isin(np.floor((tw - t0_wall) / 2.0) % 2, [0])
        p = 1000 * (idle_W + PLANNER_W[method] * busy) + rng.normal(0, 20, len(tw))
        _w(os.path.join(d, "power.csv"),
           ["t_wall", "vdd_in_mW", "vdd_cpu_gpu_cv_mW", "vdd_soc_mW", "temp_cpu_C", "temp_gpu_C"],
           [[t, v, 0.4 * v, 0.25 * v, 50, 49] for t, v in zip(tw, p)])
        tpp = np.arange(t0_wall, t0_wall + total_t, 1.0)
        cpu_rate = {"nearest": 0.02, "mrtsp": 0.06, "ariadne": 0.45}[method]
        _w(os.path.join(d, "procs.csv"), ["t_wall", "pid", "name", "utime_s", "stime_s"],
           [[t, 4242, PLANNER_PROC[method], cpu_rate * (t - t0_wall), 0.0] for t in tpp])
    with open(os.path.join(d, "summary.json"), "w") as f:
        json.dump(summary, f)
    return run_id


def make_idle(logs, power_mode, watts, t0_wall=1.8e9 - 600, seconds=300):
    d = os.path.join(logs, f"idle_{power_mode}_20261201-080000")
    os.makedirs(d, exist_ok=True)
    tw = np.arange(t0_wall, t0_wall + seconds, 0.1)
    _w(os.path.join(d, "power.csv"),
       ["t_wall", "vdd_in_mW", "vdd_cpu_gpu_cv_mW", "vdd_soc_mW", "temp_cpu_C", "temp_gpu_C"],
       [[t, 1000 * watts, 400 * watts, 250 * watts, 45, 44] for t in tw])
    with open(os.path.join(d, "manifest.yaml"), "w") as f:
        yaml.safe_dump({"run_id": os.path.basename(d), "block": "idle", "method": "none",
                        "power_mode": power_mode, "outcome": "idle"}, f)
    return d


def make_block(logs, block="P", poses=range(3), effects=None, power_modes=("laptop",), maps=None,
               noise=0.05, seed=1, timeouts=()):
    """effects: {method: relative change of d_95 vs nearest}. timeouts: {(method, map, pose)}."""
    effects = effects or {"mrtsp": -0.15, "ariadne": -0.05}
    rng = np.random.default_rng(seed)
    ids = []
    for pm in power_modes:
        for map_name in (maps or MAPS):
            for pose in poses:
                base = MAPS[map_name] * (1 + 0.2 * rng.standard_normal())
                for method in ["nearest"] + list(effects):
                    eff = 0.0 if method == "nearest" else effects[method]
                    d95 = max(5.0, base * (1 + eff + noise * rng.standard_normal()))
                    out = "timeout" if (method, map_name, pose) in timeouts else "complete"
                    ids.append(make_run(logs, block, method, map_name, pose, pm, d95, out,
                                        seed=int(rng.integers(1 << 30))))
    return ids

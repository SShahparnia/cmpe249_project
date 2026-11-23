#!/usr/bin/env python3
"""Coverage evaluator (Module 13 of instructions.md).

Library (used live by the recorder, Module 12):

    ref = CoverageReference.from_map_dir("maps/m1_house", start_pose=(x0, y0, yaw0))
    frac = ref.fraction(data, width, height, resolution, origin_x, origin_y, origin_yaw)

CLI (official offline number):

    python analysis/coverage.py logs/<run_id> [--maps-dir maps] [--osc-threshold 1.0]
                                              [--overlay out.png]

Writes logs/<run_id>/coverage.csv and logs/<run_id>/summary.json.

Only depends on numpy and PyYAML so it runs both on the Mac and inside the ROS
container / on the Jetson (Ubuntu 22.04, Python 3.10, numpy 1.21).

Frames:
  world : the Gazebo / ground-truth frame (gt.yaml origin is in this frame).
  map   : the SLAM `map` frame. It starts at the robot's spawn pose, so
          p_world = R(yaw0) @ p_map + (x0, y0) with (x0, y0, yaw0) the start pose
          from the manifest.
  grid  : a snapshot's cell frame, p_map = R(origin_yaw) @ p_grid + origin.
Grids are indexed [row, col] with row 0 at the grid origin (lowest y), like
nav_msgs/OccupancyGrid. Images (gt.pgm) have row 0 at the top and are flipped
on load.
"""
from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import os
import sys
from dataclasses import dataclass

import numpy as np
import yaml

# SLAM cell values counted as "known" (instructions.md 13, step 2).
# VERIFY against our slam_toolbox config; use the same values everywhere.
FREE_MAX = 0      # 0 <= value <= FREE_MAX is free
OCC_MIN = 65      # value >= OCC_MIN is occupied; -1 is unknown, 1..64 counts as unknown

COVERAGE_TARGET = 0.95
MIN_STEP_M = 0.01             # path-length jitter filter (instructions.md 12)
DEFAULT_OSC_THRESHOLD_M = 1.0 # placeholder; fixed in the pilot (Module 17), frozen in Module 18
NAV2_RESOLVED = (4, 6)        # action_msgs/GoalStatus: SUCCEEDED, ABORTED


# --------------------------------------------------------------------------- GT

def read_pgm(path: str) -> np.ndarray:
    """Read a binary (P5) or ASCII (P2) PGM, 8-bit. Returns uint8 [rows, cols], row 0 = top."""
    with open(path, "rb") as f:
        raw = f.read()
    tokens, pos = [], 0
    while len(tokens) < 4:
        # skip whitespace and comments
        while pos < len(raw) and raw[pos:pos + 1].isspace():
            pos += 1
        if raw[pos:pos + 1] == b"#":
            while pos < len(raw) and raw[pos:pos + 1] not in (b"\n", b"\r"):
                pos += 1
            continue
        start = pos
        while pos < len(raw) and not raw[pos:pos + 1].isspace():
            pos += 1
        tokens.append(raw[start:pos])
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if maxval > 255:
        raise ValueError(f"{path}: 16-bit PGM not supported")
    if magic == b"P5":
        pos += 1  # single whitespace after maxval
        img = np.frombuffer(raw, dtype=np.uint8, count=w * h, offset=pos)
    elif magic == b"P2":
        img = np.array(raw[pos:].split()[: w * h], dtype=np.uint8)
    else:
        raise ValueError(f"{path}: not a PGM (magic {magic!r})")
    return img.reshape(h, w).copy()


def write_pgm(path: str, img: np.ndarray) -> None:
    """Write uint8 [rows, cols] (row 0 = top) as binary PGM. Used by tests and helpers."""
    img = np.asarray(img, dtype=np.uint8)
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (img.shape[1], img.shape[0]))
        f.write(img.tobytes())


def _dilate8(mask: np.ndarray) -> np.ndarray:
    """One-cell 8-neighbour dilation without scipy."""
    p = np.pad(mask, 1, constant_values=False)
    out = np.zeros_like(mask)
    h, w = mask.shape
    for dr in (0, 1, 2):
        for dc in (0, 1, 2):
            out |= p[dr:dr + h, dc:dc + w]
    return out


@dataclass
class GroundTruth:
    occupied: np.ndarray    # bool [rows, cols], row 0 = lowest y
    free: np.ndarray        # bool
    reachable: np.ndarray   # bool, reachable free space (Module 10 step 4)
    resolution: float
    origin: tuple           # (x, y, yaw) of cell [0, 0]'s lower-left corner, world frame

    @classmethod
    def load(cls, map_dir: str) -> "GroundTruth":
        with open(os.path.join(map_dir, "gt.yaml")) as f:
            meta = yaml.safe_load(f)
        img = read_pgm(os.path.join(map_dir, meta.get("image", "gt.pgm")))
        if meta.get("negate", 0):
            p = img / 255.0
        else:
            p = (255.0 - img) / 255.0
        occ = p > float(meta.get("occupied_thresh", 0.65))
        free = p < float(meta.get("free_thresh", 0.25))
        reach = np.load(os.path.join(map_dir, "gt_reachable.npy")).astype(bool)
        if reach.shape != img.shape:
            raise ValueError(f"gt_reachable.npy shape {reach.shape} != gt.pgm shape {img.shape}")
        ox, oy, oyaw = (list(meta["origin"]) + [0.0])[:3]
        # Image row 0 is the top; flip so row 0 is the origin row (lowest y).
        return cls(np.flipud(occ), np.flipud(free), np.flipud(reach),
                   float(meta["resolution"]), (float(ox), float(oy), float(oyaw)))

    def reference_mask(self) -> np.ndarray:
        """Cells that count in the denominator: reachable free cells plus the obstacle
        cells bordering them (what a lidar can see). Unreachable free space is excluded."""
        reach_free = self.reachable & self.free
        return reach_free | (self.occupied & _dilate8(reach_free))

    def cell_centres_world(self, mask: np.ndarray) -> np.ndarray:
        rows, cols = np.nonzero(mask)
        r = self.resolution
        gx, gy = (cols + 0.5) * r, (rows + 0.5) * r
        ox, oy, oyaw = self.origin
        c, s = math.cos(oyaw), math.sin(oyaw)
        return np.column_stack([ox + c * gx - s * gy, oy + s * gx + c * gy])


# --------------------------------------------------------------------- coverage

def known_mask(values: np.ndarray, free_max: int = FREE_MAX, occ_min: int = OCC_MIN) -> np.ndarray:
    v = values.astype(np.int16)
    return ((v >= 0) & (v <= free_max)) | (v >= occ_min)


class CoverageReference:
    """GT reference cells pre-transformed into the SLAM map frame, so each snapshot
    costs one rigid transform plus an array lookup."""

    def __init__(self, ref_world_xy: np.ndarray, start_pose, free_max=FREE_MAX, occ_min=OCC_MIN):
        x0, y0, yaw0 = start_pose
        c, s = math.cos(yaw0), math.sin(yaw0)
        dx, dy = ref_world_xy[:, 0] - x0, ref_world_xy[:, 1] - y0
        # p_map = R(-yaw0) @ (p_world - t0)
        self.pts_map = np.column_stack([c * dx + s * dy, -s * dx + c * dy])
        self.free_max, self.occ_min = free_max, occ_min

    @classmethod
    def from_map_dir(cls, map_dir: str, start_pose, **kw) -> "CoverageReference":
        gt = GroundTruth.load(map_dir)
        return cls(gt.cell_centres_world(gt.reference_mask()), start_pose, **kw)

    @property
    def n_cells(self) -> int:
        return len(self.pts_map)

    def known(self, data, width, height, resolution, origin_x, origin_y, origin_yaw=0.0) -> np.ndarray:
        """Bool per reference cell: is it known in this SLAM grid (nearest-neighbour lookup)?"""
        grid = np.asarray(data).reshape(int(height), int(width))
        c, s = math.cos(origin_yaw), math.sin(origin_yaw)
        dx, dy = self.pts_map[:, 0] - origin_x, self.pts_map[:, 1] - origin_y
        col = np.floor((c * dx + s * dy) / resolution).astype(np.int64)
        row = np.floor((-s * dx + c * dy) / resolution).astype(np.int64)
        inside = (col >= 0) & (col < grid.shape[1]) & (row >= 0) & (row < grid.shape[0])
        out = np.zeros(self.n_cells, dtype=bool)
        out[inside] = known_mask(grid[row[inside], col[inside]], self.free_max, self.occ_min)
        return out

    def fraction(self, data, width, height, resolution, origin_x, origin_y, origin_yaw=0.0) -> float:
        if self.n_cells == 0:
            return float("nan")
        return float(self.known(data, width, height, resolution,
                                origin_x, origin_y, origin_yaw).mean())

    def fraction_snapshot(self, snap) -> float:
        return self.fraction(snap["data"], snap["width"], snap["height"], snap["resolution"],
                             snap["origin_x"], snap["origin_y"], snap.get("origin_yaw", 0.0))


# ------------------------------------------------------------------ run metrics

def path_length_cumulative(x: np.ndarray, y: np.ndarray, min_step: float = MIN_STEP_M) -> np.ndarray:
    """Cumulative path length per pose sample. Moves under `min_step` from the last
    counted point are ignored (jitter), but slow motion still adds up because the
    anchor only advances once the robot has moved `min_step` from it."""
    cum = np.zeros(len(x))
    if len(x) == 0:
        return cum
    ax, ay, total = x[0], y[0], 0.0
    for i in range(1, len(x)):
        d = math.hypot(x[i] - ax, y[i] - ay)
        if d >= min_step:
            total += d
            ax, ay = x[i], y[i]
        cum[i] = total
    return cum


def first_crossing(t, cov, dist, target=COVERAGE_TARGET):
    """First (t, d) where coverage reaches `target`, linearly interpolated between
    snapshots. (None, None) if never reached."""
    idx = np.nonzero(np.asarray(cov) >= target)[0]
    if len(idx) == 0:
        return None, None
    i = int(idx[0])
    if i == 0:
        return float(t[0]), float(dist[0])
    f = (target - cov[i - 1]) / (cov[i] - cov[i - 1])
    return (float(t[i - 1] + f * (t[i] - t[i - 1])),
            float(dist[i - 1] + f * (dist[i] - dist[i - 1])))


def oscillation_count(goals_t, goals_xy, result_t, result_status, threshold_m):
    """Goal switches where the new goal is more than `threshold_m` from the current one
    and the current goal was neither reached nor aborted (no Nav2 SUCCEEDED/ABORTED
    result since it was first set). Re-sends of (almost) the same goal do not reset it."""
    order = np.argsort(goals_t, kind="stable")
    gt_, gxy = np.asarray(goals_t)[order], np.asarray(goals_xy)[order]
    resolved_t = np.asarray(result_t)[np.isin(result_status, NAV2_RESOLVED)]
    count = 0
    if len(gt_) == 0:
        return 0
    cur_xy, cur_t = gxy[0], gt_[0]
    for t, xy in zip(gt_[1:], gxy[1:]):
        if math.hypot(xy[0] - cur_xy[0], xy[1] - cur_xy[1]) <= threshold_m:
            continue
        if not np.any((resolved_t > cur_t) & (resolved_t <= t)):
            count += 1
        cur_xy, cur_t = xy, t
    return count


# ------------------------------------------------------------------------ I/O

def read_csv_columns(path: str) -> dict:
    """CSV -> {column: float array}. Missing file -> {}."""
    if not os.path.exists(path):
        return {}
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        with open(path, newline="") as f:
            header = next(csv.reader(f), [])
        return {k: np.array([], dtype=float) for k in header}
    return {k: np.array([float(r[k]) if r[k] not in ("", None) else np.nan for r in rows])
            for k in rows[0]}


def load_snapshots(run_dir: str) -> list:
    snaps = []
    for p in glob.glob(os.path.join(run_dir, "maps", "map_*.npz")):
        with np.load(p) as z:
            snaps.append({k: (z[k] if k == "data" else z[k].item()) for k in z.files})
    snaps.sort(key=lambda s: s["t_ros"])
    return snaps


def evaluate_run(run_dir: str, maps_dir: str = "maps",
                 osc_threshold_m: float = DEFAULT_OSC_THRESHOLD_M,
                 free_max: int = FREE_MAX, occ_min: int = OCC_MIN) -> dict:
    """Compute coverage.csv rows and the summary for one run folder (does not write)."""
    with open(os.path.join(run_dir, "manifest.yaml")) as f:
        manifest = yaml.safe_load(f) or {}
    sp = manifest["start_pose"]
    start_pose = (float(sp["x"]), float(sp["y"]), float(sp["yaw"]))
    ref = CoverageReference.from_map_dir(os.path.join(maps_dir, manifest["map"]), start_pose,
                                         free_max=free_max, occ_min=occ_min)
    warnings = []

    pose = read_csv_columns(os.path.join(run_dir, "pose.csv"))
    snaps = load_snapshots(run_dir)
    if not snaps:
        warnings.append("no map snapshots")
    if not pose or len(pose.get("t_ros", [])) == 0:
        warnings.append("no pose samples")
        pose = {"t_ros": np.array([]), "x": np.array([]), "y": np.array([])}

    # Time zero: manifest t_ros_start (planner start) if the orchestrator wrote it,
    # else the first recorded pose / snapshot.
    starts = [pose["t_ros"][0]] if len(pose["t_ros"]) else []
    starts += [snaps[0]["t_ros"]] if snaps else []
    t0 = float(manifest["t_ros_start"]) if manifest.get("t_ros_start") is not None \
        else (min(starts) if starts else 0.0)

    keep = pose["t_ros"] >= t0
    pt, px, py = pose["t_ros"][keep], pose["x"][keep], pose["y"][keep]
    cum = path_length_cumulative(px, py)

    def dist_at(t):
        return float(np.interp(t, pt, cum, left=0.0)) if len(pt) else 0.0

    rows = []
    for s in snaps:
        if s["t_ros"] < t0:
            continue
        rows.append((float(s["t_ros"]), dist_at(s["t_ros"]), ref.fraction_snapshot(s)))
    t_arr = np.array([r[0] for r in rows])
    d_arr = np.array([r[1] for r in rows])
    c_arr = np.array([r[2] for r in rows])

    t95, d95 = first_crossing(t_arr, c_arr, d_arr) if len(rows) else (None, None)

    goals = read_csv_columns(os.path.join(run_dir, "goals.csv"))
    nav = read_csv_columns(os.path.join(run_dir, "nav2_results.csv"))
    osc = oscillation_count(
        goals.get("t_ros", np.array([])),
        np.column_stack([goals.get("goal_x", np.array([])), goals.get("goal_y", np.array([]))]),
        nav.get("t_ros", np.array([])), nav.get("status", np.array([])), osc_threshold_m)

    t_end = max([t0] + ([float(pt[-1])] if len(pt) else []) + ([float(t_arr[-1])] if len(t_arr) else []))
    outcome = manifest.get("outcome")
    if outcome == "complete" and t95 is None:
        warnings.append("manifest says complete but offline coverage never reached the target")
    if outcome in ("timeout", "stuck") and t95 is not None:
        warnings.append(f"manifest says {outcome} but offline coverage reached the target")

    summary = {
        "t_95": None if t95 is None else t95 - t0,
        "d_95": d95,
        "final_coverage": float(c_arr[-1]) if len(c_arr) else None,
        "total_distance_m": float(cum[-1]) if len(cum) else 0.0,
        "total_time_s": t_end - t0,
        "outcome": outcome,
        "oscillation_count": osc,
        "oscillation_threshold_m": osc_threshold_m,
        "coverage_target": COVERAGE_TARGET,
        "n_reference_cells": ref.n_cells,
        "n_snapshots": len(rows),
        "t_ros_zero": t0,
        "known_thresholds": {"free_max": free_max, "occ_min": occ_min},
        "coverage_warnings": warnings,
    }
    return {"rows": rows, "summary": summary, "manifest": manifest, "ref": ref, "snaps": snaps}


def write_outputs(run_dir: str, result: dict) -> None:
    with open(os.path.join(run_dir, "coverage.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_ros", "distance_m", "coverage_frac"])
        for t, d, c in result["rows"]:
            w.writerow([f"{t:.3f}", f"{d:.4f}", f"{c:.6f}"])
    # summary.json is shared with the analysis step: merge, do not clobber other keys.
    path = os.path.join(run_dir, "summary.json")
    summary = {}
    if os.path.exists(path):
        with open(path) as f:
            summary = json.load(f)
    summary.update(result["summary"])
    with open(path, "w") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")


def write_overlay(path: str, result: dict) -> None:
    """Final SLAM map (grey) with GT reference cells: green = known, red = missed.
    For the one-run-per-map alignment check (instructions.md 13, step 1)."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    ref, snaps = result["ref"], result["snaps"]
    fig, ax = plt.subplots(figsize=(8, 8))
    if snaps:
        s = snaps[-1]
        grid = np.asarray(s["data"]).reshape(int(s["height"]), int(s["width"]))
        img = np.full(grid.shape, 0.75)
        img[(grid >= 0) & (grid <= FREE_MAX)] = 1.0
        img[grid >= OCC_MIN] = 0.0
        r, oyaw = s["resolution"], s.get("origin_yaw", 0.0)
        if abs(oyaw) > 1e-6:
            print("overlay: snapshot origin_yaw != 0, background drawn unrotated", file=sys.stderr)
        ext = [s["origin_x"], s["origin_x"] + grid.shape[1] * r,
               s["origin_y"], s["origin_y"] + grid.shape[0] * r]
        ax.imshow(img, cmap="gray", vmin=0, vmax=1, origin="lower", extent=ext)
        known = ref.known(s["data"], s["width"], s["height"], r, s["origin_x"], s["origin_y"], oyaw)
    else:
        known = np.zeros(ref.n_cells, dtype=bool)
    p = ref.pts_map
    ax.scatter(p[known, 0], p[known, 1], s=1, c="tab:green", label="GT ref, known")
    ax.scatter(p[~known, 0], p[~known, 1], s=1, c="tab:red", label="GT ref, missed")
    ax.set_aspect("equal")
    ax.set_xlabel("x (m, SLAM map frame)")
    ax.set_ylabel("y (m, SLAM map frame)")
    ax.legend(loc="upper right", markerscale=8)
    ax.set_title(os.path.basename(os.path.normpath(result["manifest"].get("run_id", ""))) or "coverage overlay")
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("run_dir", help="logs/<run_id>")
    ap.add_argument("--maps-dir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "maps"),
                    help="folder holding <map>/gt.yaml, gt.pgm, gt_reachable.npy (default: repo maps/)")
    ap.add_argument("--osc-threshold", type=float, default=DEFAULT_OSC_THRESHOLD_M,
                    help="oscillation goal-switch distance X in metres (set in the pilot)")
    ap.add_argument("--free-max", type=int, default=FREE_MAX)
    ap.add_argument("--occ-min", type=int, default=OCC_MIN)
    ap.add_argument("--overlay", metavar="PNG", help="also save an alignment overlay figure")
    a = ap.parse_args(argv)

    result = evaluate_run(a.run_dir, a.maps_dir, a.osc_threshold, a.free_max, a.occ_min)
    write_outputs(a.run_dir, result)
    if a.overlay:
        write_overlay(a.overlay, result)
    s = result["summary"]
    print(f"final_coverage={s['final_coverage']} t_95={s['t_95']} d_95={s['d_95']} "
          f"distance={s['total_distance_m']:.2f} m oscillations={s['oscillation_count']} "
          f"outcome={s['outcome']}")
    for w in s["coverage_warnings"]:
        print(f"warning: {w}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

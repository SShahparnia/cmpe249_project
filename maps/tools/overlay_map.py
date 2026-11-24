#!/usr/bin/env python3
"""Overlay a SLAM map on a map's ground truth (Module 10, "Done when" figure).

    python3 maps/tools/overlay_map.py maps/<map> --slam saved_map.yaml \
        [--start-pose X Y YAW] [--out maps/<map>/overlay_slam.png]

--slam is a map_server map (map_saver_cli output: .yaml + image) in the SLAM `map`
frame. That frame starts at the robot's spawn pose, so --start-pose (world frame,
default 0 0 0) moves it into the world frame. For a recorder run folder use
`python3 analysis/coverage.py logs/<run_id> --overlay out.png` instead.

The figure shows GT occupied cells (black), SLAM occupied cells (red, drawn on top) and
SLAM free cells (light blue). It prints the fraction of SLAM occupied cells that lie
within one cell of a GT occupied cell: a quick number for "do they line up".
"""
from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np
import yaml
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "analysis"))
import coverage as cov  # noqa: E402


def load_map_server(path: str):
    """map_server yaml -> (occupied bool, free bool, resolution, (x, y, yaw)); row 0 = lowest y."""
    with open(path) as f:
        meta = yaml.safe_load(f)
    img_path = os.path.join(os.path.dirname(path), meta["image"])
    if img_path.lower().endswith(".pgm"):
        img = cov.read_pgm(img_path)
    else:
        from PIL import Image
        img = np.asarray(Image.open(img_path).convert("L"))
    p = img / 255.0 if meta.get("negate", 0) else (255.0 - img) / 255.0
    occ = np.flipud(p > float(meta.get("occupied_thresh", 0.65)))
    free = np.flipud(p < float(meta.get("free_thresh", 0.196)))
    ox, oy, oyaw = (list(meta["origin"]) + [0.0])[:3]
    return occ, free, float(meta["resolution"]), (float(ox), float(oy), float(oyaw))


def cells_to_world(mask, res, origin, start_pose):
    """Cell centres of `mask` (SLAM map frame grid) -> world xy."""
    rows, cols = np.nonzero(mask)
    gx, gy = (cols + 0.5) * res, (rows + 0.5) * res
    ox, oy, oyaw = origin
    c, s = math.cos(oyaw), math.sin(oyaw)
    mx, my = ox + c * gx - s * gy, oy + s * gx + c * gy           # map frame
    x0, y0, yaw0 = start_pose
    c, s = math.cos(yaw0), math.sin(yaw0)
    return np.column_stack([x0 + c * mx - s * my, y0 + s * mx + c * my])


def agreement(gt: "cov.GroundTruth", slam_occ_world: np.ndarray, tol_cells: int = 1) -> float:
    """Fraction of SLAM occupied points within `tol_cells` of a GT occupied cell."""
    near = ndimage.binary_dilation(gt.occupied, iterations=tol_cells, structure=np.ones((3, 3), bool))
    ox, oy, _ = gt.origin
    col = np.floor((slam_occ_world[:, 0] - ox) / gt.resolution).astype(int)
    row = np.floor((slam_occ_world[:, 1] - oy) / gt.resolution).astype(int)
    inside = (row >= 0) & (row < near.shape[0]) & (col >= 0) & (col < near.shape[1])
    if not inside.any():
        return float("nan")
    hit = np.zeros(len(col), dtype=bool)
    hit[inside] = near[row[inside], col[inside]]
    return float(hit.mean())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map_dir")
    ap.add_argument("--slam", required=True, help="map_server .yaml in the SLAM map frame")
    ap.add_argument("--start-pose", type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=("X", "Y", "YAW"))
    ap.add_argument("--out", help="default: <map_dir>/overlay_slam.png")
    ap.add_argument("--title")
    a = ap.parse_args(argv)

    gt = cov.GroundTruth.load(a.map_dir)
    occ, free, res, origin = load_map_server(a.slam)
    p_occ = cells_to_world(occ, res, origin, a.start_pose)
    p_free = cells_to_world(free, res, origin, a.start_pose)
    score = agreement(gt, p_occ)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    r = gt.resolution
    ext = [gt.origin[0], gt.origin[0] + gt.occupied.shape[1] * r, gt.origin[1], gt.origin[1] + gt.occupied.shape[0] * r]
    fig, ax = plt.subplots(figsize=(9, 9 * gt.occupied.shape[0] / gt.occupied.shape[1] + 0.5))
    bg = np.where(gt.occupied, 0.0, np.where(gt.reachable, 0.93, 0.8))
    ax.imshow(bg, origin="lower", extent=ext, cmap="gray", vmin=0, vmax=1, interpolation="nearest")
    ax.scatter(p_free[:, 0], p_free[:, 1], s=0.3, c="#9ecae1", alpha=0.35, linewidths=0, label="SLAM free")
    ax.scatter(p_occ[:, 0], p_occ[:, 1], s=1.2, c="#d62728", linewidths=0, label="SLAM occupied")
    ax.set_xlim(ext[0], ext[1])
    ax.set_ylim(ext[2], ext[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x (m, world)")
    ax.set_ylabel("y (m, world)")
    ax.legend(loc="upper right", markerscale=6, fontsize=8)
    name = os.path.basename(os.path.normpath(a.map_dir))
    ax.set_title(a.title or f"{name}: SLAM map over ground truth ({100 * score:.1f}% of SLAM occupied "
                            f"cells within 1 cell of GT occupied)", fontsize=10)
    fig.tight_layout()
    out = a.out or os.path.join(a.map_dir, "overlay_slam.png")
    fig.savefig(out, dpi=130)
    plt.close(fig)
    print(f"{out}: agreement {score:.3f} ({len(p_occ)} SLAM occupied cells)")
    return 0


if __name__ == "__main__":
    sys.exit(main())

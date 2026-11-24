#!/usr/bin/env python3
"""Start-pose generator (Module 11 of instructions.md).

    python3 harness/scripts/gen_start_poses.py --map m1_house [--n 20] [--seed 249]

Reads maps/<map>/gt.pgm (+ gt.yaml) and gt_reachable.npy, keeps the reachable free cells
that are at least --clearance (0.35 m: TB4 radius ~0.17 m plus margin, which also leaves
room for the dock the TB4 sim spawns 0.157 m behind the robot) from any cell that is not
reachable free, then samples:

  - a cell uniformly from those, and a point uniformly inside that cell,
  - yaw uniformly in [-pi, pi),
  - rejecting points closer than --min-sep (1.5 m) to a pose already chosen,

all from numpy.random.default_rng(seed), so the same inputs always give the same poses.
Writes configs/start_poses/<map>.yaml and a figure configs/start_poses/<map>.png.

Blocks use the first N poses (pilot 0..2, Block Q 0..9 or 0..14, Block E 0..4), so a
pose that fails the sim check (harness/scripts/check_start_poses.py) is replaced by
running with --replace <i>: pose i is redrawn from the same stream (the next sample
that passes), the others stay unchanged, and the replacement is listed in the file.
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys

import numpy as np
import yaml
from scipy import ndimage

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
sys.path.insert(0, os.path.join(REPO, "analysis"))
import coverage as cov  # noqa: E402

MAX_DRAWS = 200_000


def git_commit() -> str:
    """HEAD, with "-dirty" when this script is uncommitted or modified (the file's sha256 is
    stored next to it, so the exact generator is always pinned)."""
    me = os.path.relpath(os.path.abspath(__file__), REPO)
    try:
        sha = subprocess.check_output(["git", "-C", REPO, "rev-parse", "HEAD"], text=True).strip()
        tracked = subprocess.call(["git", "-C", REPO, "ls-files", "--error-unmatch", me],
                                  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0
        changed = subprocess.call(["git", "--no-optional-locks", "-C", REPO, "diff", "--quiet", "HEAD", "--", me]) != 0
        return sha + ("-dirty" if (not tracked or changed) else "")
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def file_sha256(path) -> str:
    import hashlib
    with open(path, "rb") as f:
        return hashlib.sha256(f.read().replace(b"\r\n", b"\n")).hexdigest()


def safe_cells(gt: "cov.GroundTruth", clearance: float) -> np.ndarray:
    """Reachable free cells whose centre is >= clearance from every other cell's centre."""
    ok = gt.reachable & gt.free
    dist = ndimage.distance_transform_edt(ok) * gt.resolution   # to the nearest not-ok cell centre
    return ok & (dist >= clearance + gt.resolution / 2)


class Sampler:
    """One deterministic stream of candidate poses; `next()` returns the next accepted pose."""

    def __init__(self, gt, safe, seed, min_sep):
        self.gt, self.min_sep = gt, min_sep
        self.rows, self.cols = np.nonzero(safe)
        if len(self.rows) == 0:
            raise ValueError("no cell satisfies the clearance")
        self.rng = np.random.default_rng(seed)
        self.draws = 0

    def candidate(self):
        self.draws += 1
        i = self.rng.integers(len(self.rows))
        u, v = self.rng.random(2)
        yaw = self.rng.uniform(-math.pi, math.pi)
        r, g = self.gt.resolution, self.gt
        gx, gy = (self.cols[i] + u) * r, (self.rows[i] + v) * r
        ox, oy, oyaw = g.origin
        c, s = math.cos(oyaw), math.sin(oyaw)
        return ox + c * gx - s * gy, oy + s * gx + c * gy, yaw

    def next(self, chosen, banned=()):
        while self.draws < MAX_DRAWS:
            x, y, yaw = self.candidate()
            if any(math.hypot(x - p[0], y - p[1]) < self.min_sep for p in chosen):
                continue
            if any(math.hypot(x - p[0], y - p[1]) < 1e-9 for p in banned):
                continue
            return x, y, yaw
        raise RuntimeError(f"gave up after {MAX_DRAWS} draws: the map is too small for this many poses "
                           f"at min_sep, lower --n or --min-sep")


def generate(gt, n, seed, clearance, min_sep):
    sampler = Sampler(gt, safe_cells(gt, clearance), seed, min_sep)
    poses = []
    for _ in range(n):
        poses.append(sampler.next(poses))
    return poses, sampler


def replace(gt, poses, idx, seed, clearance, min_sep, n_extra_draws=0):
    """Redraw pose `idx`: continue the same stream past the original n poses and take the
    first sample that keeps min_sep from all the other poses."""
    sampler = Sampler(gt, safe_cells(gt, clearance), seed, min_sep)
    first = []
    for _ in range(len(poses)):          # replay the original stream
        first.append(sampler.next(first))
    for _ in range(n_extra_draws):       # skip replacements already used
        sampler.candidate()
    others = [p for j, p in enumerate(poses) if j != idx]
    new = sampler.next(others, banned=[poses[idx]])
    return new, sampler.draws


def plot(path, gt, poses, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    r = gt.resolution
    ext = [gt.origin[0], gt.origin[0] + gt.occupied.shape[1] * r, gt.origin[1], gt.origin[1] + gt.occupied.shape[0] * r]
    bg = np.where(gt.occupied, 0.0, np.where(gt.reachable, 0.95, 0.8))
    fig, ax = plt.subplots(figsize=(8, 8 * gt.occupied.shape[0] / gt.occupied.shape[1] + 0.4))
    ax.imshow(bg, origin="lower", extent=ext, cmap="gray", vmin=0, vmax=1, interpolation="nearest")
    for i, (x, y, yaw) in enumerate(poses):
        color = "tab:red" if i < 3 else ("tab:blue" if i < 10 else "tab:purple")
        ax.add_patch(plt.Circle((x, y), 0.17, color=color, alpha=0.6))
        ax.arrow(x, y, 0.35 * math.cos(yaw), 0.35 * math.sin(yaw), head_width=0.12, color=color)
        ax.text(x + 0.2, y + 0.2, str(i), fontsize=7, color=color)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color="tab:red", label="0-2 (pilot)"), Patch(color="tab:blue", label="3-9"),
                       Patch(color="tab:purple", label="10+")], loc="upper right", fontsize=8)
    ax.set_aspect("equal")
    ax.set_title(title, fontsize=10)
    ax.set_xlabel("x (m, world)")
    ax.set_ylabel("y (m, world)")
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--map", required=True)
    ap.add_argument("--n", type=int, default=20)
    ap.add_argument("--seed", type=int, default=249)
    ap.add_argument("--clearance", type=float, default=0.35, help="m from any non-reachable cell")
    ap.add_argument("--min-sep", type=float, default=1.5, help="m between poses")
    ap.add_argument("--maps-dir", default=os.path.join(REPO, "maps"))
    ap.add_argument("--out-dir", default=os.path.join(REPO, "configs", "start_poses"))
    ap.add_argument("--replace", type=int, action="append", default=[], metavar="I",
                    help="redraw pose I of the existing file (it failed the sim check)")
    ap.add_argument("--reason", default="failed the sim spawn check", help="note stored with --replace")
    a = ap.parse_args(argv)

    gt = cov.GroundTruth.load(os.path.join(a.maps_dir, a.map))
    out = os.path.join(a.out_dir, f"{a.map}.yaml")
    os.makedirs(a.out_dir, exist_ok=True)

    if a.replace:
        with open(out) as f:
            doc = yaml.safe_load(f)
        poses = [(p["x"], p["y"], p["yaw"]) for p in doc["poses"]]
        g = doc["generator"]
        extra = sum(int(r.get("extra_draws", 0)) for r in doc.get("replaced", []))
        for idx in a.replace:
            new, draws = replace(gt, poses, idx, doc["seed"], g["clearance_m"], g["min_sep_m"], extra)
            doc.setdefault("replaced", []).append({
                "index": idx, "old": dict(zip(("x", "y", "yaw"), [round(float(v), 4) for v in poses[idx]])),
                "reason": a.reason, "extra_draws": draws - g["draws"] - extra})
            extra = draws - g["draws"]
            poses[idx] = new
    else:
        poses, sampler = generate(gt, a.n, a.seed, a.clearance, a.min_sep)
        doc = {"map": a.map, "seed": a.seed, "generator_commit": git_commit(),
               "generator": {"script": "harness/scripts/gen_start_poses.py",
                             "script_sha256": file_sha256(os.path.abspath(__file__)), "clearance_m": a.clearance,
                             "min_sep_m": a.min_sep, "safe_cells": int(safe_cells(gt, a.clearance).sum()),
                             "draws": sampler.draws},
               "frame": "world (Gazebo); the SLAM map frame starts here"}
    doc["poses"] = [{"x": round(float(x), 4), "y": round(float(y), 4), "yaw": round(float(yaw), 4)}
                    for x, y, yaw in poses]

    with open(out, "w", newline="\n") as f:
        f.write(f"# Start poses for {a.map} (Module 11). Frozen after prereg-v1. Blocks use the first N:\n"
                f"# pilot 0-2, Block Q 0-9 (or 0-14), Block E 0-4. Regenerate: {doc['generator']['script']} "
                f"--map {a.map} --n {len(poses)} --seed {doc['seed']}\n")
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None, width=120)
    plot(os.path.join(a.out_dir, f"{a.map}.png"), gt, poses,
         f"{a.map}: {len(poses)} start poses (seed {doc['seed']}, clearance {doc['generator']['clearance_m']} m, "
         f"min separation {doc['generator']['min_sep_m']} m)")
    print(f"wrote {out}: {len(poses)} poses")
    return 0


if __name__ == "__main__":
    sys.exit(main())

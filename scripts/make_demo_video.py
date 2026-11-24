#!/usr/bin/env python3
"""Demo video: one run per method side by side, with the power trace underneath (Module 22).

    python3 scripts/make_demo_video.py \
        --run logs/E_nearest_m1_house_p00_MAXN_SUPER_... --video nearest.mp4 --video-start 1796000000.0 \
        --run logs/E_mrtsp_m1_house_p00_MAXN_SUPER_...   --video mrtsp.mp4   --video-start 1796001000.0 \
        --run logs/E_ariadne_m1_house_p00_MAXN_SUPER_... --video ariadne.mp4 --video-start 1796002000.0 \
        --out demo_m1.mp4

Each --video is a screen capture of RViz for that run; --video-start is the Unix time
(t_wall) of the capture's first frame (OBS / ffmpeg can stamp it; or start the capture
with `date +%s.%N` printed). Each run is played from its planner start (manifest
t_wall_start), cut to the shortest run, scaled to the same height and stacked side by
side. Under each video, the board power from that run's power.csv is drawn with a cursor
synchronized by t_wall. The run IDs are written to <out>.txt for the video description.
Needs ffmpeg and matplotlib.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np
import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(REPO, "analysis"))
from energy import read_csv  # noqa: E402

FPS = 10
H = 540            # height of each RViz panel
PANEL_H = 180      # height of each power panel


def power_frames(run_dir, t0, duration, width, out_dir, label):
    """PNG frames of the power trace with a moving cursor, one per 1/FPS s."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    p = read_csv(os.path.join(run_dir, "power.csv"))
    t = p["t_wall"] - t0
    w = p["vdd_in_mW"] / 1000.0
    m = (t >= 0) & (t <= duration)
    os.makedirs(out_dir, exist_ok=True)
    fig, ax = plt.subplots(figsize=(width / 100, PANEL_H / 100), dpi=100)
    ax.plot(t[m], w[m], lw=0.6, color="#444")
    ax.set_xlim(0, duration)
    ax.set_ylim(max(0.0, w[m].min() - 0.5), w[m].max() + 0.5)
    ax.set_ylabel("W", fontsize=7)
    ax.tick_params(labelsize=6)
    ax.set_title(label, fontsize=7)
    cursor = ax.axvline(0, color="red", lw=1)
    fig.tight_layout()
    for i, ts in enumerate(np.arange(0, duration, 1.0 / FPS)):
        cursor.set_xdata([ts, ts])
        fig.savefig(os.path.join(out_dir, f"f{i:06d}.png"))
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--run", action="append", required=True)
    ap.add_argument("--video", action="append", required=True)
    ap.add_argument("--video-start", action="append", type=float, required=True)
    ap.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    if not (len(a.run) == len(a.video) == len(a.video_start)):
        ap.error("give one --video and one --video-start per --run")
    if not shutil.which("ffmpeg"):
        ap.error("ffmpeg not found")
    mans = []
    for r in a.run:
        with open(os.path.join(r, "manifest.yaml")) as f:
            mans.append(yaml.safe_load(f))
    starts = [float(m["t_wall_start"]) for m in mans]
    duration = min(float(m["t_wall_end"]) - s for m, s in zip(mans, starts))
    width = int(H * 16 / 9)
    with tempfile.TemporaryDirectory() as tmp:
        inputs, filters = [], []
        for i, (r, v, vs, s, m) in enumerate(zip(a.run, a.video, a.video_start, starts, mans)):
            offset = max(0.0, s - vs)            # skip to the planner start inside the capture
            inputs += ["-ss", f"{offset:.3f}", "-t", f"{duration:.3f}", "-i", v]
            fdir = os.path.join(tmp, f"p{i}")
            power_frames(r, s, duration, width, fdir, f"{m['method']}: VDD_IN (board power)")
            inputs += ["-framerate", str(FPS), "-i", os.path.join(fdir, "f%06d.png")]
            filters.append(f"[{2 * i}:v]scale={width}:{H},fps={FPS},setsar=1[v{i}];"
                           f"[{2 * i + 1}:v]scale={width}:{PANEL_H},setsar=1[p{i}];"
                           f"[v{i}][p{i}]vstack=inputs=2[c{i}]")
        n = len(a.run)
        graph = ";".join(filters) + ";" + "".join(f"[c{i}]" for i in range(n)) + f"hstack=inputs={n}[out]"
        cmd = ["ffmpeg", "-y", *inputs, "-filter_complex", graph, "-map", "[out]", "-c:v", "libx264",
               "-pix_fmt", "yuv420p", "-r", str(FPS), a.out]
        subprocess.check_call(cmd)
    with open(a.out + ".txt", "w") as f:
        f.write("Run IDs (left to right):\n" + "\n".join(os.path.basename(os.path.normpath(r)) for r in a.run) + "\n")
    print(f"wrote {a.out} ({duration:.0f} s) and {a.out}.txt")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
"""Spawn the TB4 at each start pose once and check it (Module 11 step 4).

    python3 harness/scripts/check_start_poses.py --map m1_house [--poses 0-19] [--settle 10]

For every pose: launch sim + slam_toolbox (no Nav2) with harness/launch/tb4_sim.launch.py
at that pose, then check

  1. not in collision: after `--settle` s of sim time the robot has not been pushed:
     /odom moved less than 3 cm and turned less than 2 degrees;
  2. nothing inside the robot: no /scan return closer than 0.20 m (body radius ~0.17 m);
  3. SLAM starts: slam_toolbox publishes /map with at least 500 known cells.

Results are appended to configs/start_poses/<map>_check.csv (one row per pose per check
run), so the file documents which poses were validated and when. A failing pose is
replaced with gen_start_poses.py --replace <i>, then checked again.

Run it inside the ROS environment with the repo built (exploration_msgs is not needed).
It uses its own ROS_DOMAIN_ID / IGN_PARTITION unless they are set, so it does not talk to
a sim that is already running.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import os
import signal
import subprocess
import sys
import time

import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
LAUNCH = os.path.join(REPO, "harness", "launch", "tb4_sim.launch.py")
HEADER = ["date", "map", "pose", "x", "y", "yaw", "pushed_m", "turned_deg", "min_scan_m",
          "map_known_cells", "sim_s_waited", "ok", "note"]


def parse_range(s: str, n: int):
    if not s:
        return list(range(n))
    out = []
    for part in s.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b or a) + 1))
    return out


def probe(settle_s: float, timeout_wall: float) -> dict:
    """Collect /odom, /scan and /map from the running sim (runs in this process)."""
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan

    rclpy.init()
    node = Node("start_pose_check", parameter_overrides=[Parameter("use_sim_time", value=True)])
    st = {"odom0": None, "odom": None, "t0": None, "min_scan": math.inf, "known": 0}

    def on_odom(m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if st["odom0"] is None:
            st["odom0"], st["t0"] = (p.x, p.y, yaw), t
        st["odom"] = (p.x, p.y, yaw, t)

    def on_scan(m):
        if st["t0"] is None:
            return
        vals = [r for r in m.ranges if math.isfinite(r) and m.range_min <= r <= m.range_max]
        if vals:
            st["min_scan"] = min(st["min_scan"], min(vals))

    def on_map(m):
        st["known"] = sum(1 for v in m.data if v >= 0)

    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(Odometry, "/odom", on_odom, 10)
    node.create_subscription(LaserScan, "/scan", on_scan, qos_profile_sensor_data)
    node.create_subscription(OccupancyGrid, "/map", on_map, latched)
    t_end = time.time() + timeout_wall
    waited = 0.0
    try:
        while time.time() < t_end:
            rclpy.spin_once(node, timeout_sec=0.2)
            if st["odom"] is not None:
                waited = st["odom"][3] - st["t0"]
                if waited >= settle_s and st["known"] >= 500:
                    break
    finally:
        node.destroy_node()
        rclpy.shutdown()
    st["waited"] = waited
    return st


def stop(proc):
    if proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, 15), (signal.SIGTERM, 5), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(wait)
            return
        except subprocess.TimeoutExpired:
            continue


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--map", required=True)
    ap.add_argument("--poses", default="", help="e.g. 0-2,5 (default: all)")
    ap.add_argument("--settle", type=float, default=10.0, help="sim seconds to watch for pushes")
    ap.add_argument("--timeout", type=float, default=600.0, help="wall seconds per pose")
    ap.add_argument("--software-gl", action="store_true", help="LIBGL_ALWAYS_SOFTWARE=1 (WSL)")
    a = ap.parse_args(argv)

    with open(os.path.join(REPO, "configs", "start_poses", f"{a.map}.yaml")) as f:
        poses = yaml.safe_load(f)["poses"]
    env = dict(os.environ)
    env.setdefault("ROS_DOMAIN_ID", "78")
    env.setdefault("IGN_PARTITION", "start_pose_check")
    if a.software_gl:
        env["LIBGL_ALWAYS_SOFTWARE"] = "1"
    os.environ.update({k: env[k] for k in ("ROS_DOMAIN_ID", "IGN_PARTITION")})
    log_dir = os.path.join(REPO, "logs", "start_pose_check", a.map)
    os.makedirs(log_dir, exist_ok=True)
    out_csv = os.path.join(REPO, "configs", "start_poses", f"{a.map}_check.csv")
    new = not os.path.exists(out_csv)
    n_bad = 0
    with open(out_csv, "a", newline="") as fcsv:
        w = csv.writer(fcsv)
        if new:
            w.writerow(HEADER)
        for i in parse_range(a.poses, len(poses)):
            p = poses[i]
            cmd = ["ros2", "launch", LAUNCH, f"world:={a.map}", "gui:=false", "slam:=true", "nav2:=false",
                   "rviz:=false", f"x:={p['x']}", f"y:={p['y']}", f"yaw:={p['yaw']}"]
            with open(os.path.join(log_dir, f"p{i:02d}.log"), "w") as flog:
                proc = subprocess.Popen(cmd, stdout=flog, stderr=subprocess.STDOUT, env=env, start_new_session=True)
                try:
                    st = probe(a.settle, a.timeout)
                finally:
                    stop(proc)       # the whole process group: launch, Gazebo, bridges, SLAM
            note = []
            if st["odom"] is None:
                pushed = turned = float("nan")
                note.append("no /odom")
            else:
                x0, y0, yaw0 = st["odom0"]
                x1, y1, yaw1, _ = st["odom"]
                pushed = math.hypot(x1 - x0, y1 - y0)
                turned = abs(math.degrees(math.atan2(math.sin(yaw1 - yaw0), math.cos(yaw1 - yaw0))))
            if not pushed < 0.03 or not turned < 2.0:
                note.append("pushed (collision?)")
            if st["min_scan"] < 0.20:
                note.append(f"scan return at {st['min_scan']:.2f} m")
            if st["known"] < 500:
                note.append("no SLAM map")
            if st["waited"] < a.settle:
                note.append(f"only {st['waited']:.1f} s sim time in {a.timeout:.0f} s wall")
            ok = not note
            n_bad += not ok
            row = [dt.datetime.now().strftime("%Y-%m-%d %H:%M"), a.map, i, p["x"], p["y"], p["yaw"],
                   f"{pushed:.4f}", f"{turned:.2f}",
                   "inf" if math.isinf(st["min_scan"]) else f"{st['min_scan']:.3f}",
                   st["known"], f"{st['waited']:.1f}", "yes" if ok else "NO", "; ".join(note)]
            w.writerow(row)
            fcsv.flush()
            print(f"{a.map} p{i:02d}: {'ok' if ok else 'FAIL ' + '; '.join(note)} "
                  f"(pushed {pushed:.3f} m, turned {turned:.1f} deg, min scan {st['min_scan']:.2f} m, "
                  f"{st['known']} known cells)", flush=True)
    return 1 if n_bad else 0


if __name__ == "__main__":
    sys.exit(main())

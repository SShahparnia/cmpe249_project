#!/usr/bin/env python3
"""Smoke test for harness/recorder.py (Module 12), no Gazebo needed.

Run inside the ROS environment after building exploration_msgs:
    colcon build --base-paths harness && source install/setup.bash
    python3 harness/tests/smoke_recorder.py [--seconds 15] [--signal INT|TERM]

Starts the real recorder as a subprocess, fakes its inputs from this process
(/clock at sim time, moving TF map->base_link, a growing /map, PlanningCycle
messages, Nav2 action status changes), stops the recorder with a signal, then
checks every output file and runs the offline coverage evaluator on the folder.
Exit code 0 = pass.
"""
import argparse
import csv
import os
import signal
import subprocess
import sys
import tempfile
import time

import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from action_msgs.msg import GoalInfo, GoalStatus, GoalStatusArray
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

from exploration_msgs.msg import PlanningCycle

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "analysis"))
sys.path.insert(0, os.path.join(REPO, "analysis", "tests"))
import coverage as cov  # noqa: E402
from test_coverage import COLS, GT_ORIGIN, RES, ROWS, make_gt_grid, write_map_dir  # noqa: E402

SIM_T0 = 1000.0
LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)


def tmsg(t):
    return TimeMsg(sec=int(t), nanosec=int(round((t - int(t)) * 1e9)) % 1_000_000_000)


class FakeWorld(Node):
    def __init__(self):
        super().__init__("fake_world")
        self.t0_wall = time.time()
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.tf = TransformBroadcaster(self)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", LATCHED)
        self.cycle_pub = self.create_publisher(PlanningCycle, "/exploration/planning_cycle", 10)
        self.status_pub = self.create_publisher(GoalStatusArray, "/navigate_to_pose/_action/status", LATCHED)
        self.cov = []
        self.create_subscription(Float32, "/exploration/coverage_live", lambda m: self.cov.append(m.data), 10)
        self.cycle_id = 0
        self.goals = []          # [(uuid bytes, status)]
        self.last = {"map": -1e9, "cycle": -1e9}

    def sim_now(self):
        return SIM_T0 + (time.time() - self.t0_wall)

    def step(self):
        t = self.sim_now()
        self.clock_pub.publish(Clock(clock=tmsg(t)))
        # Robot drives along +x at 0.2 m/s in the map frame.
        tf = TransformStamped()
        tf.header.stamp, tf.header.frame_id, tf.child_frame_id = tmsg(t), "map", "base_link"
        tf.transform.translation.x = 0.2 * (t - SIM_T0)
        tf.transform.rotation.w = 1.0
        self.tf.sendTransform(tf)

        if t - self.last["map"] >= 2.0:          # slam_toolbox-like map updates
            self.last["map"] = t
            k = min(COLS, int(3 * (t - SIM_T0)))   # known columns grow over time
            grid = np.full((ROWS, COLS), -1, dtype=np.int8)
            grid[:, :k] = 0
            m = OccupancyGrid()
            m.header.stamp, m.header.frame_id = tmsg(t), "map"
            m.info.resolution, m.info.width, m.info.height = RES, COLS, ROWS
            m.info.origin.position.x, m.info.origin.position.y = GT_ORIGIN
            m.info.origin.orientation.w = 1.0
            m.data = grid.ravel().tolist()
            self.map_pub.publish(m)

        if t - self.last["cycle"] >= 1.0:        # one planning cycle per second, new goal each time
            self.last["cycle"] = t
            c = PlanningCycle(cycle_id=self.cycle_id, stamp_start=tmsg(t - 0.05), stamp_end=tmsg(t),
                              wall_start=time.time() - 0.05, wall_end=time.time(), n_frontiers=10 + self.cycle_id)
            c.goal.x, c.goal.y = 2.0 * self.cycle_id, 0.0
            self.cycle_pub.publish(c)
            # Nav2: the previous goal succeeds, the new one is accepted then executing.
            if self.goals:
                self.goals[-1][1] = GoalStatus.STATUS_SUCCEEDED
            self.goals.append([os.urandom(16), GoalStatus.STATUS_ACCEPTED])
            self.publish_status()
            self.goals[-1][1] = GoalStatus.STATUS_EXECUTING
            self.publish_status()
            self.cycle_id += 1

    def publish_status(self):
        arr = GoalStatusArray()
        for uid, st in self.goals[-3:]:
            gs = GoalStatus(status=st, goal_info=GoalInfo())
            gs.goal_info.goal_id.uuid = list(uid)
            arr.status_list.append(gs)
        self.status_pub.publish(arr)


def read_csv(path):
    with open(path, newline="") as f:
        return list(csv.reader(f))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--signal", choices=["INT", "TERM"], default="INT")
    ap.add_argument("--keep", action="store_true", help="keep the temp run folder")
    a = ap.parse_args()

    tmp = tempfile.mkdtemp(prefix="recorder_smoke_")
    maps_dir = os.path.join(tmp, "maps")
    occ, reach = make_gt_grid()
    write_map_dir(os.path.join(maps_dir, "m_test"), occ, reach)
    run = os.path.join(tmp, "logs", "P_nearest_m_test_p00_mac_20261123-000000")
    os.makedirs(run)
    with open(os.path.join(run, "manifest.yaml"), "w") as f:
        yaml.safe_dump({"run_id": os.path.basename(run), "map": "m_test", "pose_index": 0,
                        "start_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                        "t_ros_start": SIM_T0, "outcome": "timeout"}, f)

    rclpy.init()
    world = FakeWorld()
    log = open(os.path.join(tmp, "recorder.log"), "w")
    rec = subprocess.Popen([sys.executable, os.path.join(REPO, "harness", "recorder.py"),
                            "--out", run, "--maps-dir", maps_dir],
                           stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    t_end = time.time() + a.seconds
    while time.time() < t_end:
        world.step()
        rclpy.spin_once(world, timeout_sec=0.02)
    rec.send_signal(getattr(signal, "SIG" + a.signal))
    try:
        rc = rec.wait(timeout=10)
    except subprocess.TimeoutExpired:
        rec.kill()
        rc = "killed (did not exit within 10 s)"
    log.close()

    failures = []

    def check(cond, msg):
        print(("PASS " if cond else "FAIL ") + msg)
        if not cond:
            failures.append(msg)

    check(rc == 0, f"recorder exited cleanly on SIG{a.signal} (rc={rc})")
    for name, header in [("pose", ["t_wall", "t_ros", "x", "y", "yaw"]),
                         ("goals", ["t_wall", "t_ros", "cycle_id", "goal_x", "goal_y"]),
                         ("cycles", ["cycle_id", "t_wall_start", "t_wall_end", "t_ros_start", "t_ros_end",
                                     "n_frontiers", "goal_x", "goal_y"]),
                         ("nav2_results", ["t_wall", "t_ros", "goal_id", "status"])]:
        rows = read_csv(os.path.join(run, f"{name}.csv"))
        check(rows[0] == header, f"{name}.csv header")
        check(len(rows) > 2, f"{name}.csv has rows ({len(rows) - 1})")
        check(all(len(r) == len(header) for r in rows), f"{name}.csv rows complete (no torn last line)")

    pose = np.array(read_csv(os.path.join(run, "pose.csv"))[1:], dtype=float)
    dt = np.diff(pose[:, 1])
    rate = 1.0 / np.median(dt) if len(dt) else 0.0
    check(8.0 <= rate <= 12.0, f"pose rate ~10 Hz sim time ({rate:.1f} Hz)")
    check(pose[0, 1] >= SIM_T0 - 1e-6, "pose t_ros is sim time, not wall time")
    check(np.all(dt > 0), "pose t_ros strictly increasing (no duplicate TF samples)")

    statuses = [int(r[3]) for r in read_csv(os.path.join(run, "nav2_results.csv"))[1:]]
    check({1, 2, 4} <= set(statuses), f"nav2 status changes logged (accepted/executing/succeeded: {sorted(set(statuses))})")
    nav = read_csv(os.path.join(run, "nav2_results.csv"))[1:]
    pairs = [(r[2], r[3]) for r in nav]
    check(len(pairs) == len(set(pairs)), "nav2_results: one row per (goal, status) change, no repeats")

    snaps = sorted(f for f in os.listdir(os.path.join(run, "maps")))
    check(len(snaps) >= int(a.seconds // 5), f"map snapshots every 5 s + final ({len(snaps)})")
    check(not any(".tmp" in s for s in snaps), "no leftover temp snapshot files")
    with np.load(os.path.join(run, "maps", snaps[-1])) as z:
        keys = set(z.files)
    need = {"data", "resolution", "origin_x", "origin_y", "origin_yaw", "width", "height", "t_ros", "t_wall"}
    check(need <= keys, f"snapshot keys ({sorted(keys)})")

    check(len(world.cov) > 0 and world.cov[-1] > world.cov[0],
          f"coverage_live published and growing ({[round(c, 3) for c in world.cov[:1] + world.cov[-1:]]})")

    res = cov.evaluate_run(run, maps_dir)
    cov.write_outputs(run, res)
    s = res["summary"]
    check(s["final_coverage"] is not None and s["final_coverage"] > 0.5,
          f"offline evaluator runs on recorder output (final_coverage={s['final_coverage']:.3f})")
    check(abs(s["total_distance_m"] - 0.2 * s["total_time_s"]) < 0.3,
          f"path length matches 0.2 m/s ({s['total_distance_m']:.2f} m over {s['total_time_s']:.1f} s)")
    if world.cov:
        check(abs(world.cov[-1] - s["final_coverage"]) < 0.05, "live and offline coverage agree")

    world.destroy_node()
    rclpy.shutdown()
    print(f"\nrun folder: {run}" if a.keep or failures else "")
    if not (a.keep or failures):
        import shutil
        shutil.rmtree(tmp)
    print("RESULT:", "FAIL" if failures else "PASS")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())

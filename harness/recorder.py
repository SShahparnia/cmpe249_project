#!/usr/bin/env python3
"""Recorder node (Module 12 of instructions.md).

    python3 harness/recorder.py --out logs/<run_id> [--map m1_house --start-pose X Y YAW]

Writes into --out (formats in docs/log_format.md):
    pose.csv          TF map -> base_link, 10 Hz (sim time)
    cycles.csv        /exploration/planning_cycle, every message
    goals.csv         one row per planning cycle (the chosen goal)
    nav2_results.csv  /navigate_to_pose/_action/status, every status change
    maps/map_<seq>.npz  /map snapshot every 5 s sim time, plus one at shutdown
and publishes std_msgs/Float32 /exploration/coverage_live (same function as the
offline evaluator, analysis/coverage.py) for the orchestrator's stop condition.

The map name and start pose for live coverage come from <out>/manifest.yaml (written
by the orchestrator first) unless given on the command line. Without ground truth the
recorder still logs everything and just does not publish coverage.

CSV files are flushed every second (wall clock) and closed cleanly on SIGINT/SIGTERM.
Snapshots are written to a temp file and renamed, so a crash never leaves a broken .npz.
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import sys
import time

import numpy as np
import yaml

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time

from action_msgs.msg import GoalStatusArray
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32
from tf2_ros import Buffer, TransformException, TransformListener

from exploration_msgs.msg import PlanningCycle

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
sys.path.insert(0, os.path.join(REPO, "analysis"))
from coverage import CoverageReference  # noqa: E402

HEADERS = {
    "pose": ["t_wall", "t_ros", "x", "y", "yaw"],
    "goals": ["t_wall", "t_ros", "cycle_id", "goal_x", "goal_y"],
    "cycles": ["cycle_id", "t_wall_start", "t_wall_end", "t_ros_start", "t_ros_end",
               "n_frontiers", "goal_x", "goal_y"],
    "nav2_results": ["t_wall", "t_ros", "goal_id", "status"],
}

LATCHED = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
# Action status can change several times within one spin (accepted -> executing);
# a deeper queue keeps every change. Transient local still delivers the latest on join.
STATUS_QOS = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)


def stamp_s(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class Recorder(Node):
    def __init__(self, a):
        super().__init__("exploration_recorder",
                         parameter_overrides=[Parameter("use_sim_time", value=not a.wall_time)])
        self.a = a
        os.makedirs(os.path.join(a.out, "maps"), exist_ok=True)
        self.files, self.writers = {}, {}
        for name, header in HEADERS.items():
            f = open(os.path.join(a.out, f"{name}.csv"), "w", newline="")
            self.files[name] = f
            self.writers[name] = csv.writer(f)
            self.writers[name].writerow(header)
        self.flush()

        self.coverage = self._load_coverage_reference()
        self.cov_pub = self.create_publisher(Float32, "/exploration/coverage_live", 10)

        # Pose from TF.
        self.tf_buffer = Buffer(cache_time=Duration(seconds=30))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last_pose_stamp = None
        self.create_timer(1.0 / a.pose_rate, self.on_pose_timer)

        # Planning cycles.
        self.create_subscription(PlanningCycle, "/exploration/planning_cycle", self.on_cycle, 100)

        # Nav2 action status: one row per (goal_id, status) change.
        self.goal_status = {}
        self.create_subscription(GoalStatusArray, a.nav2_status_topic, self.on_nav2_status, STATUS_QOS)

        # Map: keep the latest; snapshot on a sim-time timer.
        self.latest_map = None
        self.map_seq = 0
        self.create_subscription(OccupancyGrid, a.map_topic, self.on_map, LATCHED)
        self.create_timer(a.snapshot_period, self.on_snapshot_timer)

        # Flush on the wall clock so files stay fresh even if /clock stalls.
        self.create_timer(1.0, self.flush, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(
            f"recording to {a.out} (sim_time={not a.wall_time}, "
            f"live coverage={'on' if self.coverage else 'off'})")

    # ----------------------------------------------------------- setup helpers

    def _load_coverage_reference(self):
        a, map_name, start = self.a, self.a.map, self.a.start_pose
        mpath = os.path.join(a.out, "manifest.yaml")
        if (map_name is None or start is None) and os.path.exists(mpath):
            with open(mpath) as f:
                m = yaml.safe_load(f) or {}
            map_name = map_name or m.get("map")
            sp = m.get("start_pose")
            if start is None and sp:
                start = (float(sp["x"]), float(sp["y"]), float(sp["yaw"]))
        if map_name is None or start is None:
            self.get_logger().warn("no map / start pose (manifest or CLI): live coverage disabled")
            return None
        map_dir = os.path.join(a.maps_dir, map_name)
        try:
            ref = CoverageReference.from_map_dir(map_dir, tuple(start))
        except (OSError, KeyError, ValueError) as e:
            self.get_logger().warn(f"cannot load ground truth from {map_dir} ({e}): live coverage disabled")
            return None
        self.get_logger().info(f"live coverage: {map_name}, {ref.n_cells} reference cells, start {start}")
        return ref

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    # --------------------------------------------------------------- callbacks

    def on_pose_timer(self):
        try:
            tf = self.tf_buffer.lookup_transform(self.a.map_frame, self.a.base_frame, Time())
        except TransformException:
            return
        t_ros = stamp_s(tf.header.stamp)
        if t_ros == self.last_pose_stamp:
            return          # no new TF since the last sample
        self.last_pose_stamp = t_ros
        tr = tf.transform.translation
        self.writers["pose"].writerow(
            [repr(time.time()), repr(t_ros), repr(tr.x), repr(tr.y), repr(yaw_from_quat(tf.transform.rotation))])

    def on_cycle(self, m: PlanningCycle):
        t_end = stamp_s(m.stamp_end)
        self.writers["cycles"].writerow(
            [m.cycle_id, repr(m.wall_start), repr(m.wall_end), repr(stamp_s(m.stamp_start)), repr(t_end),
             m.n_frontiers, repr(m.goal.x), repr(m.goal.y)])
        self.writers["goals"].writerow([repr(m.wall_end), repr(t_end), m.cycle_id, repr(m.goal.x), repr(m.goal.y)])

    def on_nav2_status(self, m: GoalStatusArray):
        t_wall, t_ros = time.time(), self.now_s()
        for st in m.status_list:
            gid = bytes(st.goal_info.goal_id.uuid).hex()
            if self.goal_status.get(gid) != st.status:
                self.goal_status[gid] = st.status
                self.writers["nav2_results"].writerow([repr(t_wall), repr(t_ros), gid, st.status])

    def on_map(self, m: OccupancyGrid):
        self.latest_map = m
        if self.coverage is not None:
            info = m.info
            frac = self.coverage.fraction(
                np.asarray(m.data, dtype=np.int8), info.width, info.height, info.resolution,
                info.origin.position.x, info.origin.position.y, yaw_from_quat(info.origin.orientation))
            self.cov_pub.publish(Float32(data=float(frac)))

    def on_snapshot_timer(self):
        self.save_snapshot()

    # ------------------------------------------------------------------ output

    def save_snapshot(self):
        m = self.latest_map
        if m is None:
            return
        info = m.info
        path = os.path.join(self.a.out, "maps", f"map_{self.map_seq:05d}.npz")
        tmp = path[:-4] + ".tmp.npz"
        np.savez_compressed(
            tmp,
            data=np.asarray(m.data, dtype=np.int8),
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            origin_yaw=float(yaw_from_quat(info.origin.orientation)),
            width=int(info.width), height=int(info.height),
            t_ros=self.now_s(), t_wall=time.time(),
            map_stamp=stamp_s(m.header.stamp),    # extra key: when slam_toolbox published it
        )
        os.replace(tmp, path)
        self.map_seq += 1

    def flush(self):
        for f in self.files.values():
            f.flush()

    def close(self):
        try:
            self.save_snapshot()       # final map state
        except Exception as e:  # noqa: BLE001 - never lose the CSVs over a snapshot
            print(f"recorder: final snapshot failed: {e}", file=sys.stderr)
        for f in self.files.values():
            f.flush()
            os.fsync(f.fileno())
            f.close()


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="Exploration run recorder (Module 12)")
    ap.add_argument("--out", required=True, help="run folder, logs/<run_id>")
    ap.add_argument("--map", help="map name for live coverage (default: from manifest.yaml)")
    ap.add_argument("--start-pose", type=float, nargs=3, metavar=("X", "Y", "YAW"),
                    help="spawn pose in the world frame (default: from manifest.yaml)")
    ap.add_argument("--maps-dir", default=os.path.join(REPO, "maps"))
    ap.add_argument("--map-frame", default="map")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--map-topic", default="/map")
    ap.add_argument("--nav2-status-topic", default="/navigate_to_pose/_action/status")
    ap.add_argument("--pose-rate", type=float, default=10.0, help="Hz, sim time")
    ap.add_argument("--snapshot-period", type=float, default=5.0, help="s, sim time")
    ap.add_argument("--wall-time", action="store_true", help="do not use /clock (testing only)")
    # Ignore ROS args (--ros-args ...) so this can also be started from a launch file.
    return ap.parse_known_args(argv)[0]


def main(argv=None):
    a = parse_args(argv)
    rclpy.init(args=sys.argv)
    node = Recorder(a)
    stop = {"flag": False}

    def on_term(signum, _frame):
        stop["flag"] = True
    signal.signal(signal.SIGTERM, on_term)
    signal.signal(signal.SIGINT, on_term)

    try:
        while rclpy.ok() and not stop["flag"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.close()
        node.get_logger().info(f"closed {a.out} ({node.map_seq} snapshots)")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

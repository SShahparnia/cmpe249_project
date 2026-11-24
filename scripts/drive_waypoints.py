#!/usr/bin/env python3
"""Drive the robot through a list of map-frame waypoints with Nav2, one goal at a time.

For demos and manual test runs (e.g. a recorder / coverage check), not for experiments.

    python3 scripts/drive_waypoints.py [--loops 1] -- 2,3 5,6.5 0,6.5 -4,6.5 -3,2.5 0,1

(the `--` lets waypoints start with a minus sign)

A goal that fails (aborted / rejected) is skipped and the next one is sent.
"""
import argparse
import math
import sys
import time

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

NAMES = {GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED", GoalStatus.STATUS_ABORTED: "ABORTED",
         GoalStatus.STATUS_CANCELED: "CANCELED"}


def parse_xy(s):
    x, y = s.split(",")
    return float(x), float(y)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("waypoints", nargs="+", type=parse_xy, help="x,y in the map frame (m)")
    ap.add_argument("--loops", type=int, default=1)
    ap.add_argument("--frame", default="map")
    a = ap.parse_args()

    rclpy.init()
    node = Node("drive_waypoints", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    if not client.wait_for_server(timeout_sec=60):
        print("navigate_to_pose server not available", file=sys.stderr)
        return 1

    plan = [wp for _ in range(a.loops) for wp in a.waypoints]
    ok = 0
    for i, (x, y) in enumerate(plan):
        nxt = plan[i + 1] if i + 1 < len(plan) else None
        yaw = math.atan2(nxt[1] - y, nxt[0] - x) if nxt else 0.0   # face the next waypoint
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = a.frame
        goal.pose.pose.position.x, goal.pose.pose.position.y = x, y
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)

        last = {"t": 0.0}

        def feedback(msg):
            if time.time() - last["t"] >= 10:
                last["t"] = time.time()
                f = msg.feedback
                print(f"    {f.distance_remaining:5.2f} m to go, robot at "
                      f"({f.current_pose.pose.position.x:.2f}, {f.current_pose.pose.position.y:.2f}), "
                      f"{f.navigation_time.sec} s sim", flush=True)

        print(f"[{i + 1}/{len(plan)}] goal ({x:.2f}, {y:.2f})", flush=True)
        t0 = time.time()
        fut = client.send_goal_async(goal, feedback_callback=feedback)
        rclpy.spin_until_future_complete(node, fut)
        handle = fut.result()
        if not handle.accepted:
            print("    rejected", flush=True)
            continue
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(node, res)
        status = res.result().status
        ok += status == GoalStatus.STATUS_SUCCEEDED
        print(f"    {NAMES.get(status, status)} after {time.time() - t0:.0f} s wall", flush=True)

    print(f"done: {ok}/{len(plan)} waypoints reached")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())

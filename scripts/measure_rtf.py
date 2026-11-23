#!/usr/bin/env python3
"""Module 3, step 4: measure the real-time factor (sim seconds per wall second)
from /clock while sim + SLAM + Nav2 are running. Simulator-agnostic.

Usage (with the stack running):  python3 scripts/measure_rtf.py [--seconds 60]
"""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock

ap = argparse.ArgumentParser()
ap.add_argument("--seconds", type=float, default=60.0)
args = ap.parse_args()

rclpy.init()
node = Node("measure_rtf")
samples = []
node.create_subscription(
    Clock, "/clock",
    lambda m: samples.append((time.time(), m.clock.sec + m.clock.nanosec * 1e-9)),
    qos_profile_sensor_data)

print(f"sampling /clock for {args.seconds:.0f} s wall ...")
t_end = time.time() + args.seconds
while time.time() < t_end:
    rclpy.spin_once(node, timeout_sec=0.1)

if len(samples) < 2:
    print("no /clock messages received; is the sim running and ROS_DOMAIN_ID set?")
else:
    (w0, s0), (w1, s1) = samples[0], samples[-1]
    rtf = (s1 - s0) / (w1 - w0)
    print(f"wall {w1 - w0:.1f} s, sim {s1 - s0:.1f} s, /clock msgs {len(samples)}")
    print(f"real-time factor = {rtf:.3f}")
node.destroy_node()
rclpy.shutdown()

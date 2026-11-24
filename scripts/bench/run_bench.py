#!/usr/bin/env python3
"""Unattended runner for the shared exploration benchmark (Modules 5 and 6, benchmark runs).

Benchmark: mertgulerx/autonomous-exploration-demo-benchmark @ 9dbf7bd (ROS 2 Jazzy, Gazebo Harmonic).
Set up with scripts/bench/setup_jazzy_bench.sh, then build the benchmark workspace and our overlay
(commands in docs/benchmark_runs.md, "Reproduce").

What it reproduces from the benchmark (launch.sh + rviz/src/exploration_control_panel.cpp):
  - world + robot spawn at the benchmark's per-world pose, then navigation_with_slam.launch.py
  - explorer started with the same commands the RViz panel uses (set A), or ours (sets B, C)
  - distance = sum of /odom xy steps while the run is active, steps > 2.0 m ignored (panel rule)
  - time = wall clock from explorer start (panel rule); sim time is recorded as well

What differs, on purpose:
  - no RViz and no frontier_path_tracker (GUI / path display only)
  - a fresh simulator for every run instead of the panel's Map Reset / Return Start
  - the run ends on the explorer's own completion signal (or a timeout / stuck rule),
    not when a person clicks End

Usage (inside Ubuntu 24.04):
  python3 scripts/bench/run_bench.py --sets A B C --reps 5 --world bookstore
Results: ~/bench_runs/results.csv, one folder of logs per run.
"""

import argparse
import csv
import datetime as dt
import math
import os
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.action import ActionClient
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Empty
from nav2_msgs.action import NavigateToPose
from explore_lite_msgs.msg import ExploreStatus

try:
    from exploration_msgs.msg import PlanningCycle
except ImportError:  # overlay not sourced
    PlanningCycle = None

HOME = Path.home()
BENCH = HOME / 'bench' / 'autonomous-exploration-demo-benchmark'
OVERLAY = HOME / 'eb_ws'
REPO = HOME / 'cmpe249_project'
OUT = HOME / 'bench_runs'

# launch.sh spawn poses
SPAWN = {
    'bookstore': ('1.23', '6.35', '0.11', '-3.10'),
    'corridor': ('-8.11', '0.31', '0.10', '0.00'),
    'warehouse': ('-12.97', '8.23', '0.10', '1.58'),
}

# Explorer definitions. overlay=True sources ~/eb_ws (our pinned, timing-patched planners).
SETS = {
    'A': dict(
        name='m_explore_ros2 (benchmark vendored + benchmark config)',
        overlay=False, done='explore_status',
        cmd='ros2 launch explore_lite explore.launch.py use_sim_time:=true'),
    'B': dict(
        name='nearest (our pinned m-explore-ros2 + configs/methods/nearest.yaml)',
        overlay=True, done='explore_status',
        cmd=('ros2 run explore_lite explore --ros-args -r __node:=explore_node '
             f'--params-file {REPO}/configs/methods/nearest.yaml -p use_sim_time:=true')),
    'C': dict(
        name='mrtsp greedy (our pinned frontier_exploration_ros2; benchmark config + configs/methods/mrtsp.yaml)',
        overlay=True, done='fer2_complete',
        cmd=('ros2 run frontier_exploration_ros2 frontier_explorer --ros-args -r __node:=frontier_explorer '
             f'--params-file {BENCH}/config/frontier_exploration_ros2/config.yaml '
             f'--params-file {REPO}/configs/methods/mrtsp.yaml '
             # benchmark robot (MOGI bot) frame, as in the benchmark config; ours is written for the TB4
             '-p robot_base_frame:=base_footprint -p use_sim_time:=true')),
    'D': dict(
        name='frontier_exploration_ros2 (benchmark vendored + benchmark config, mrtsp_solver dp)',
        overlay=False, done='fer2_complete',
        cmd=('ros2 launch frontier_exploration_ros2 frontier_explorer.launch.py use_sim_time:=true '
             'params_file:=config/frontier_exploration_ros2/config.yaml')),
}

CLEANUP_PATTERNS = [
    'gz sim', 'ruby.*gz', 'parameter_bridge', 'image_bridge', 'robot_state_publisher',
    'slam_toolbox', 'nav2_', 'lifecycle_manager', 'explore_lite', 'frontier_explorer',
    'topic_tools', 'interactive_marker_twist_server', 'ekf_node', 'rviz2', 'ros2 launch',
]


def source_cmd(overlay):
    s = f'source /opt/ros/jazzy/setup.bash && source {BENCH}/install/setup.bash'
    if overlay:
        s += f' && source {OVERLAY}/install/setup.bash'
    return s


def start(cmd, overlay, log_path):
    log = open(log_path, 'w')
    # Start from an environment without any sourced ROS workspace, so a process with
    # overlay=False really gets only the benchmark install (the runner itself has the overlay).
    env = {k: v for k, v in os.environ.items() if k not in ROS_ENV_KEYS and not k.startswith('ROS_')}
    env['PATH'] = '/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
    return subprocess.Popen(
        ['bash', '-c', f'{source_cmd(overlay)} && exec {cmd}'],
        cwd=BENCH, env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


ROS_ENV_KEYS = {'AMENT_PREFIX_PATH', 'CMAKE_PREFIX_PATH', 'COLCON_PREFIX_PATH', 'LD_LIBRARY_PATH',
                'PYTHONPATH', 'PKG_CONFIG_PATH', 'GZ_SIM_RESOURCE_PATH', 'GZ_SIM_SYSTEM_PLUGIN_PATH'}


def stop(proc, grace=8.0):
    if proc is None or proc.poll() is not None:
        return
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(timeout=grace)
    except (subprocess.TimeoutExpired, ProcessLookupError):
        try:
            os.killpg(proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


def cleanup_leftovers():
    killed = []
    for pat in CLEANUP_PATTERNS:
        if subprocess.run(['pgrep', '-f', pat], capture_output=True).returncode == 0:
            killed.append(pat)
            subprocess.run(['pkill', '-TERM', '-f', pat])
    if killed:
        time.sleep(2)
        for pat in killed:
            subprocess.run(['pkill', '-KILL', '-f', pat])
        time.sleep(1)
    return killed


def tree_cpu_seconds(root_pid):
    """utime+stime of every live process in the session of root_pid."""
    total = 0.0
    clk = os.sysconf('SC_CLK_TCK')
    for d in os.listdir('/proc'):
        if not d.isdigit():
            continue
        try:
            with open(f'/proc/{d}/stat') as f:
                fields = f.read().rsplit(')', 1)[1].split()
            # after ')': [0]=state [1]=ppid [2]=pgrp [3]=session [11]=utime [12]=stime
            if int(fields[3]) == root_pid:
                total += (int(fields[11]) + int(fields[12])) / clk
        except (OSError, IndexError, ValueError):
            continue
    return total


class Metrics(Node):
    def __init__(self):
        super().__init__('bench_runner_metrics')
        self.reset()
        self.create_subscription(Odometry, '/odom', self.on_odom, 50)
        self.create_subscription(Clock, '/clock', self.on_clock, 1)
        latched = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ExploreStatus, '/explore/status', self.on_status, latched)
        self.create_subscription(Empty, '/exploration_complete', self.on_fer2_done, latched)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
        if PlanningCycle is not None:
            self.create_subscription(PlanningCycle, '/exploration/planning_cycle', self.on_cycle, 100)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def reset(self):
        self.active = False
        self.distance = 0.0
        self.last_xy = None
        self.sim_now = None
        self.done_signal = None
        self.known_m2 = None
        self.cycles = []
        self.last_move_sim = None

    def on_clock(self, msg):
        self.sim_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    def on_odom(self, msg):
        if not self.active:
            return
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        if self.last_xy is None:
            self.last_xy = (x, y)
            return
        step = math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)
        if not math.isfinite(step) or step > 2.0:
            return
        self.distance += step
        if step > 0.002:
            self.last_move_sim = self.sim_now

    def on_status(self, msg):
        if self.active and msg.status == ExploreStatus.EXPLORATION_COMPLETE:
            self.done_signal = 'explore_status'

    def on_fer2_done(self, _msg):
        if self.active:
            self.done_signal = 'fer2_complete'

    def on_map(self, msg):
        known = sum(1 for v in msg.data if v != -1)
        self.known_m2 = known * msg.info.resolution ** 2

    def on_cycle(self, msg):
        if self.active:
            self.cycles.append((msg.cycle_id, msg.wall_end - msg.wall_start, msg.n_frontiers))


def spin_for(_node, seconds):
    # The node is spun by a background executor thread (main()); just wait.
    time.sleep(seconds)


def run_once(node, set_key, world, rep, args, writer):
    spec = SETS[set_key]
    stamp = dt.datetime.now().strftime('%Y%m%d-%H%M%S')
    run_id = f'bench_{world}_{set_key}_r{rep:02d}_{stamp}'
    run_dir = OUT / run_id
    run_dir.mkdir(parents=True)
    leftovers = cleanup_leftovers()
    node.reset()

    x, y, z, yaw = SPAWN[world]
    spawn = start(f'ros2 launch bme_ros2_navigation spawn_robot.launch.py world:={world} '
                  f'x:={x} y:={y} z:={z} yaw:={yaw} rviz:=false', False, run_dir / 'spawn.log')
    spin_for(node, 5.0)  # launch.sh: 5 s head start
    navstack = start('ros2 launch bme_ros2_navigation navigation_with_slam.launch.py rviz:=false', False,
                     run_dir / 'nav.log')
    explorer = None
    row = dict(run_id=run_id, set=set_key, explorer=spec['name'], world=world, rep=rep,
               outcome='invalid', end_reason='', distance_m='', time_wall_s='', time_sim_s='',
               rtf='', known_area_m2='', explorer_cpu_s='', n_cycles='', cycle_ms_median='',
               cycle_ms_p95='', leftovers_killed=';'.join(leftovers))
    try:
        # Wait for Nav2 + a first map, like an operator would before pressing Start.
        t0 = time.monotonic()
        ready = False
        while time.monotonic() - t0 < args.ready_timeout:
            time.sleep(0.2)
            if node.known_m2 is not None and node.nav.server_is_ready():
                ready = True
                break
        if not ready:
            row['end_reason'] = 'nav2 or /map not ready'
            return row
        spin_for(node, args.settle)

        explorer = start(spec['cmd'], spec['overlay'], run_dir / 'explorer.log')
        node.active = True
        wall0 = time.monotonic()
        while node.sim_now is None:
            time.sleep(0.05)
        sim0 = node.sim_now
        node.last_move_sim = sim0
        cpu_max = 0.0
        last_cpu = 0.0
        while True:
            time.sleep(0.1)
            now_w = time.monotonic()
            if now_w - last_cpu > 2.0:
                cpu_max = max(cpu_max, tree_cpu_seconds(explorer.pid))
                last_cpu = now_w
            sim_el = (node.sim_now or sim0) - sim0
            if node.done_signal:
                row['outcome'], row['end_reason'] = 'complete', node.done_signal
                break
            if explorer.poll() is not None:
                row['outcome'], row['end_reason'] = 'stuck', f'explorer exited ({explorer.returncode})'
                break
            if sim_el > args.timeout_sim:
                row['outcome'], row['end_reason'] = 'timeout', f'sim time > {args.timeout_sim} s'
                break
            if (node.sim_now or sim0) - node.last_move_sim > args.stuck_sim:
                row['outcome'], row['end_reason'] = 'stuck', f'no motion for {args.stuck_sim} s sim'
                break
            if spawn.poll() is not None or navstack.poll() is not None:
                row['outcome'], row['end_reason'] = 'invalid', 'sim or nav stack exited'
                break
        wall_el = time.monotonic() - wall0
        sim_el = (node.sim_now or sim0) - sim0
        cpu_max = max(cpu_max, tree_cpu_seconds(explorer.pid))
        spin_for(node, 1.0)  # last /map
        lat = sorted(c[1] * 1000.0 for c in node.cycles)
        row.update(
            distance_m=f'{node.distance:.2f}', time_wall_s=f'{wall_el:.1f}', time_sim_s=f'{sim_el:.1f}',
            rtf=f'{sim_el / wall_el:.3f}' if wall_el > 0 else '',
            known_area_m2=f'{node.known_m2:.1f}' if node.known_m2 is not None else '',
            explorer_cpu_s=f'{cpu_max:.1f}', n_cycles=len(lat),
            cycle_ms_median=f'{lat[len(lat) // 2]:.2f}' if lat else '',
            cycle_ms_p95=f'{lat[min(len(lat) - 1, int(0.95 * len(lat)))]:.2f}' if lat else '')
        if node.cycles:
            with open(run_dir / 'cycles.csv', 'w', newline='') as f:
                w = csv.writer(f)
                w.writerow(['cycle_id', 'latency_s', 'n_frontiers'])
                w.writerows(node.cycles)
        return row
    finally:
        node.active = False
        stop(explorer)
        stop(navstack)
        stop(spawn)
        cleanup_leftovers()
        writer.writerow(row)
        print(f"[{row['run_id']}] {row['outcome']} ({row['end_reason']}) "
              f"d={row['distance_m']} m t_wall={row['time_wall_s']} s t_sim={row['time_sim_s']} s "
              f"rtf={row['rtf']}", flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--sets', nargs='+', default=['A', 'B', 'C'], choices=sorted(SETS))
    ap.add_argument('--reps', type=int, default=5)
    ap.add_argument('--world', default='bookstore', choices=sorted(SPAWN))
    ap.add_argument('--timeout-sim', type=float, default=1200.0)
    ap.add_argument('--stuck-sim', type=float, default=180.0)
    ap.add_argument('--ready-timeout', type=float, default=120.0)
    ap.add_argument('--settle', type=float, default=5.0)
    ap.add_argument('--max-invalid-retries', type=int, default=2)
    args = ap.parse_args()

    OUT.mkdir(exist_ok=True)
    results = OUT / 'results.csv'
    fields = ['run_id', 'set', 'explorer', 'world', 'rep', 'outcome', 'end_reason', 'distance_m',
              'time_wall_s', 'time_sim_s', 'rtf', 'known_area_m2', 'explorer_cpu_s', 'n_cycles',
              'cycle_ms_median', 'cycle_ms_p95', 'leftovers_killed']
    new = not results.exists()
    rclpy.init()
    node = Metrics()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
    with open(results, 'a', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        if new:
            writer.writeheader()
        # interleave sets per repetition so no set runs in one long streak
        for rep in range(1, args.reps + 1):
            for s in args.sets:
                for attempt in range(args.max_invalid_retries + 1):
                    row = run_once(node, s, args.world, rep, args, writer)
                    f.flush()
                    if row['outcome'] != 'invalid':
                        break
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())

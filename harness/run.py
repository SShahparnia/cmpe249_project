#!/usr/bin/env python3
"""Run orchestrator (Module 16 of instructions.md): one exploration run, start to finish.

    python3 harness/run.py --block Q --method mrtsp --map m1_house --pose 3 --power-mode laptop
    python3 harness/run.py --block E --method nearest --map m2_office --pose 0 --power-mode 15W \
        --jetson --sim-ssh laptop                      # Block E: run ON THE JETSON, Gazebo on the laptop
    python3 harness/run.py --method none --power-mode 15W --jetson --sim-ssh laptop --duration 300
                                                       # idle-stack baseline (scripts/idle_baseline.sh)
    python3 harness/run.py ... --dry-run               # write the manifest and print the commands only

In order (instructions.md Module 16):
  1. run_id (Section 0.1) and logs/<run_id>/ with stdout/ and params/.
  2. On a Jetson power mode: `nvpmodel -q` must report that mode, else refuse to run.
  3. manifest.yaml, first part: run, pose, power mode, git commits of the repo and every
     submodule, sha256 of every config used, host, chrony offset, sensor setup, start time.
  4. Launch, each in its own process group with stdout/stderr to stdout/<name>.log:
     sim (harness/launch/tb4_sim.launch.py, robot at the start pose), slam_toolbox (async)
     and Nav2 with configs/ params; wait for Nav2 (else `invalid`); remove the charging dock
     (the TB4 sim spawns the robot docked, see below); power logger (--jetson); recorder; the planner.
  5. Monitor once per second (configs/common.yaml): coverage_live >= target -> complete
     (after a grace period); sim time past timeout_sim_s -> timeout; planner exited or no
     planning cycle for stuck_no_cycle_sim_s -> stuck; an infrastructure process died,
     /clock stalled or the pose jumped -> invalid.
  6. Shutdown in reverse order: SIGINT to each group, 10 s, SIGKILL. Then every process
     still carrying this run's EB_RUN_ID in its environment is killed and listed in the
     manifest (`leftovers_killed`), so back-to-back runs never inherit orphans.
  7. manifest.yaml, second part: end times, outcome, reason, measured real-time factor.
  8. analysis/coverage.py on the folder (coverage.csv, summary.json).

Exit code: 0 for complete / timeout / stuck / idle (all count as data), 2 for invalid,
1 for a refusal before anything was launched.

Each method's replanning rate and smallest frontier come from configs/common.yaml: the
orchestrator writes the effective parameter file for the run into logs/<run_id>/params/
(method file + those overrides) and launches the planner with it.
"""
from __future__ import annotations

import argparse
import copy
import datetime as dt
import hashlib
import math
import os
import platform
import re
import shlex
import signal
import socket
import subprocess
import sys
import time

import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
BLOCKS = ("P", "Q", "E", "A")
METHODS = ("nearest", "mrtsp", "ariadne", "roadmap", "none")
LAPTOP_MODES = ("laptop", "mac")
DATA_OUTCOMES = ("complete", "timeout", "stuck", "idle")

# Process names the power logger tracks (procs.csv). The planner's entry is also stored as
# manifest `planner_process` for analysis/energy.py.
STACK_PROCS = ["async_slam_toolbox_node", "sync_slam_toolbox_node", "controller_server", "planner_server",
               "bt_navigator", "behavior_server", "smoother_server", "velocity_smoother",
               "waypoint_follower", "lifecycle_manager", "recorder"]


# ===================================================================== pure logic

def make_run_id(block, method, map_name, pose, power_mode, when: dt.datetime) -> str:
    ts = when.strftime("%Y%m%d-%H%M%S")
    if method == "none":
        return f"idle_{power_mode}_{ts}"
    return f"{block}_{method}_{map_name}_p{pose:02d}_{power_mode}_{ts}"


METHOD_SPECS = {
    # node key in the params file, replanning parameter, frontier-size parameter (and unit),
    # planner process name for procs.csv
    "nearest": {"node": "explore_node", "rate": "planner_frequency", "frontier": ("min_frontier_size", "m"),
                "process": "explore"},
    "mrtsp": {"node": "frontier_explorer", "rate": "map_processing_rate_hz",
              "frontier": ("min_frontier_size_cells", "cells"), "process": "frontier_explorer"},
    "ariadne": {"node": "rl_planner", "rate": "replanning_frequency", "frontier": None, "process": "rl_planner"},
}


def effective_params(method: str, method_cfg: dict, common: dict, map_resolution: float = 0.05) -> dict:
    """Method parameter file + the common overrides (Module 15 rules 2 and 4)."""
    spec = METHOD_SPECS[method]
    out = copy.deepcopy(method_cfg)
    p = out.setdefault(spec["node"], {}).setdefault("ros__parameters", {})
    p[spec["rate"]] = round(1.0 / float(common["replan_period_s"]), 6)
    if spec["frontier"]:
        name, unit = spec["frontier"]
        m = float(common["min_frontier_size_m"])
        p[name] = m if unit == "m" else int(round(m / map_resolution))
    for node in out.values():
        if isinstance(node, dict) and "ros__parameters" in node:
            node["ros__parameters"]["use_sim_time"] = True
    return out


def parse_nvpmodel(text: str) -> str | None:
    """`nvpmodel -q` -> mode name ("NV Power Mode: 15W" -> "15W")."""
    m = re.search(r"NV Power Mode:\s*(\S+)", text)
    return m.group(1) if m else None


def parse_chrony_offset(text: str) -> float | None:
    """`chronyc tracking` -> system clock offset in seconds (+ = fast of NTP time)."""
    m = re.search(r"System time\s*:\s*([0-9.eE+-]+)\s+seconds\s+(fast|slow)", text)
    if not m:
        return None
    v = float(m.group(1))
    return v if m.group(2) == "fast" else -v


class OutcomeMonitor:
    """Decides when a run ends and how (Section 0.5). Fed once per second by the ROS monitor;
    no ROS imports, so it is unit-tested on its own."""

    def __init__(self, common: dict, t_ros_start: float, wall_start: float):
        self.c = common
        self.t0 = t_ros_start
        self.last_cycle = t_ros_start
        self.complete_at = None
        self.last_clock = None
        self.last_clock_wall = wall_start

    def on_cycle(self, t_ros: float):
        self.last_cycle = max(self.last_cycle, t_ros)

    def step(self, wall: float, sim: float | None, coverage: float | None, planner_alive: bool,
             dead_infra: list, pose_jump_m: float = 0.0):
        """-> None (keep going) or (outcome, reason)."""
        c = self.c
        if sim is not None and sim != self.last_clock:
            self.last_clock, self.last_clock_wall = sim, wall
        if dead_infra:
            return "invalid", f"infrastructure process died: {', '.join(dead_infra)}"
        if wall - self.last_clock_wall > float(c["clock_stall_wall_s"]):
            return "invalid", f"/clock did not advance for {wall - self.last_clock_wall:.0f} s wall"
        if pose_jump_m > float(c["max_pose_jump_m"]):
            return "invalid", f"robot pose jumped {pose_jump_m:.2f} m between two samples"
        if sim is None:
            return None
        if self.complete_at is not None:
            if sim - self.complete_at >= float(c["complete_grace_s"]):
                return "complete", f"coverage_live >= {c['coverage_target']}"
            return None
        if coverage is not None and coverage >= float(c["coverage_target"]):
            self.complete_at = sim
            return None
        if sim - self.t0 > float(c["timeout_sim_s"]):
            return "timeout", f"sim time {sim - self.t0:.0f} s > timeout_sim_s {c['timeout_sim_s']}"
        if not planner_alive:
            return "stuck", "planner process exited before the coverage target"
        if sim - self.last_cycle > float(c["stuck_no_cycle_sim_s"]):
            return "stuck", f"no planning cycle for {sim - self.last_cycle:.0f} s sim time"
        return None


# ================================================================ small helpers

def sha256(path: str) -> str | None:
    if not os.path.isfile(path):
        return None
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def run_text(cmd, timeout=10) -> str:
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout).stdout
    except (OSError, subprocess.SubprocessError):
        return ""


def git_state() -> dict:
    head = run_text(["git", "-C", REPO, "rev-parse", "HEAD"]).strip()
    # --no-optional-locks: never take the index lock (someone may be committing in this clone)
    dirty = bool(run_text(["git", "--no-optional-locks", "-C", REPO, "status", "--porcelain",
                           "--untracked-files=no"]).strip())
    subs = {}
    for line in run_text(["git", "-C", REPO, "submodule", "status"]).splitlines():
        parts = line.strip().lstrip("+-U").split()
        if len(parts) >= 2:
            subs[parts[1]] = {"commit": parts[0], "modified": line.startswith("+")}
    return {"commit": head or None, "dirty": dirty, "submodules": subs}


def detect_sensor_setup() -> dict:
    """What the installed TB4 xacros actually render (scripts/sim/tune_sensor_rates.sh edits them)."""
    base = "/opt/ros/humble/share"
    out = {}
    oakd = os.path.join(base, "turtlebot4_description/urdf/sensors/oakd.urdf.xacro")
    lidar = os.path.join(base, "turtlebot4_description/urdf/sensors/rplidar.urdf.xacro")
    cliff = os.path.join(base, "irobot_create_description/urdf/sensors/cliff_sensor.urdf.xacro")
    try:
        with open(oakd) as f:
            out["camera"] = "present" if 'sensor name="rgbd_camera"' in f.read() else "removed"
        with open(lidar) as f:
            m = re.search(r'update_rate="([0-9.]+)"', f.read())
            out["lidar_hz"] = float(m.group(1)) if m else None
        with open(cliff) as f:
            m = re.search(r'name="update_rate"\s+value="([0-9.]+)"', f.read())
            out["hazard_hz"] = float(m.group(1)) if m else None
    except OSError:
        return {"detected": False}
    out["detected"] = True
    return out


def load_yaml(path):
    with open(path) as f:
        return yaml.safe_load(f) or {}


def write_yaml(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w", newline="\n") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None, width=120)
    os.replace(tmp, path)


def rel(p):
    return os.path.relpath(p, REPO).replace(os.sep, "/")


# ===================================================================== commands

class Plan:
    """Everything a run needs, resolved before anything is launched (also used by --dry-run)."""

    def __init__(self, a, now=None):
        self.a = a
        self.common_path = os.path.join(REPO, "configs", "common.yaml")
        self.common = load_yaml(self.common_path)
        self.overrides = {}
        if getattr(a, "timeout_sim", None):
            self.overrides["timeout_sim_s"] = float(a.timeout_sim)
            self.common["timeout_sim_s"] = float(a.timeout_sim)
        now = now or dt.datetime.now()
        self.started = now
        self.run_id = make_run_id(a.block, a.method, a.map, a.pose, a.power_mode, now)
        self.run_dir = os.path.join(a.logs_dir, self.run_id)
        self.poses_path = os.path.join(REPO, "configs", "start_poses", f"{a.map}.yaml")
        poses = load_yaml(self.poses_path)["poses"]
        if not 0 <= a.pose < len(poses):
            raise SystemExit(f"pose {a.pose} not in {rel(self.poses_path)} ({len(poses)} poses)")
        self.start_pose = {k: float(poses[a.pose][k]) for k in ("x", "y", "yaw")}
        self.slam_params = os.path.join(REPO, self.common["slam_params"])
        self.nav2_params = os.path.join(REPO, self.common["nav2_params"])
        self.method_cfg_path = None
        self.params_path = None
        if a.method not in ("none", "roadmap"):
            self.method_cfg_path = os.path.join(REPO, "configs", "methods", f"{a.method}.yaml")
            self.params_path = os.path.join(self.run_dir, "params", f"{a.method}.yaml")
        if a.method == "roadmap":
            raise SystemExit("method `roadmap` has no config: the fallback was not triggered (Module 7)")

    # ----------------------------------------------------------- commands
    def sim_cmd(self):
        sp = self.start_pose
        cmd = ["ros2", "launch", os.path.join(REPO, "harness", "launch", "tb4_sim.launch.py"),
               f"world:={self.a.map}", "gui:=false", "slam:=false", "nav2:=false", "rviz:=false",
               f"x:={sp['x']}", f"y:={sp['y']}", f"yaw:={sp['yaw']}"]
        if self.a.sim_ssh:
            remote_repo = self.a.sim_repo or REPO
            # setsid makes the launch its own process group; its id goes to a pid file so
            # remote_stop_cmd() can signal exactly that group.
            inner = (f"cd {shlex.quote(remote_repo)} && source scripts/ros_env.sh && "
                     + ("export LIBGL_ALWAYS_SOFTWARE=1 && " if self.a.software_gl else "")
                     + f"export EB_RUN_ID={self.run_id} && setsid "
                     + " ".join(shlex.quote(c.replace(REPO, remote_repo)) for c in cmd)
                     + f" & echo $! > {self.remote_pidfile()} && wait")
            return ["ssh", "-tt", self.a.sim_ssh, f"bash -lc {shlex.quote(inner)}"]
        return cmd

    def remove_dock_cmd(self):
        cmd = ["ign", "service", "-s", f"/world/{self.a.map}/remove", "--reqtype", "ignition.msgs.Entity",
               "--reptype", "ignition.msgs.Boolean", "--timeout", "10000",
               "--req", 'name: "standard_dock" type: MODEL']
        if self.a.sim_ssh:
            inner = "source scripts/ros_env.sh 2>/dev/null; " + " ".join(shlex.quote(c) for c in cmd)
            return ["ssh", self.a.sim_ssh, f"cd {shlex.quote(self.a.sim_repo or REPO)} && bash -c {shlex.quote(inner)}"]
        return cmd

    def remote_pidfile(self):
        return f"/tmp/explorebench_sim_{self.run_id}.pid"

    def remote_stop_cmd(self):
        f = self.remote_pidfile()
        inner = (f"test -f {f} || exit 0; g=$(cat {f}); kill -INT -- -$g 2>/dev/null; "
                 f"for i in $(seq 10); do kill -0 -- -$g 2>/dev/null || break; sleep 1; done; "
                 f"kill -KILL -- -$g 2>/dev/null; rm -f {f}; true")
        return ["ssh", self.a.sim_ssh, f"bash -c {shlex.quote(inner)}"]

    def slam_cmd(self):
        return ["ros2", "launch", "turtlebot4_navigation", "slam.launch.py", "use_sim_time:=true",
                "sync:=false", f"params:={self.slam_params}"]

    def nav2_cmd(self):
        return ["ros2", "launch", "turtlebot4_navigation", "nav2.launch.py", "use_sim_time:=true",
                f"params_file:={self.nav2_params}"]

    def recorder_cmd(self):
        return [sys.executable, os.path.join(REPO, "harness", "recorder.py"), "--out", self.run_dir,
                "--snapshot-period", str(self.common["map_snapshot_period_s"])]

    def planner_process(self):
        return METHOD_SPECS[self.a.method]["process"] if self.a.method in METHOD_SPECS else None

    def power_cmd(self):
        procs = STACK_PROCS + ([self.planner_process()] if self.planner_process() else [])
        if self.a.method == "ariadne":
            procs.append("adapter_node")
        return [sys.executable, os.path.join(REPO, "harness", "power_logger.py"), "--out", self.run_dir,
                "--procs", ",".join(procs), "--source", self.a.power_source]

    def planner_cmd(self):
        m, p = self.a.method, self.params_path
        if m == "nearest":
            return ["ros2", "run", "explore_lite", "explore", "--ros-args", "--params-file", p]
        if m == "mrtsp":
            pkg = self._share("frontier_exploration_ros2")
            return ["ros2", "run", "frontier_exploration_ros2", "frontier_explorer", "--ros-args",
                    "--params-file", os.path.join(pkg, "config", "params.yaml"), "--params-file", p,
                    "-p", "use_sim_time:=true"]
        if m == "ariadne":
            return ["ros2", "launch", "ariadne_adapter", "ariadne.launch.py", f"params_file:={p}",
                    "use_sim_time:=true"]
        return None

    @staticmethod
    def _share(pkg):
        try:
            from ament_index_python.packages import get_package_share_directory
            return get_package_share_directory(pkg)
        except Exception:   # noqa: BLE001  (dry run without ROS)
            return f"<share of {pkg}>"

    # ----------------------------------------------------------- manifest
    def config_hashes(self) -> dict:
        files = [self.common_path, self.slam_params, self.nav2_params, self.poses_path,
                 os.path.join(REPO, "maps", self.a.map, f"{self.a.map}.sdf"),
                 os.path.join(REPO, "maps", self.a.map, "gt.pgm"),
                 os.path.join(REPO, "maps", self.a.map, "gt_reachable.npy")]
        if self.method_cfg_path:
            files.append(self.method_cfg_path)
        out = {rel(f): sha256(f) for f in files}
        if self.params_path and os.path.exists(self.params_path):
            out[f"logs/{self.run_id}/params/{os.path.basename(self.params_path)}"] = sha256(self.params_path)
        return out

    def manifest_start(self) -> dict:
        a = self.a
        chrony = run_text(["chronyc", "tracking"])
        return {
            "run_id": self.run_id,
            "block": "idle" if a.method == "none" else a.block,
            "method": a.method,
            "map": a.map,
            "pose_index": a.pose,
            "start_pose": self.start_pose,
            "power_mode": a.power_mode,
            "machine": {"hostname": socket.gethostname(), "platform": platform.platform(),
                        "python": platform.python_version(), "jetson": bool(a.jetson),
                        "sim_host": a.sim_ssh or "local", "software_gl": bool(a.software_gl),
                        "ros_distro": os.environ.get("ROS_DISTRO"),
                        "rmw": os.environ.get("RMW_IMPLEMENTATION"),
                        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID")},
            "chrony_offset_s": parse_chrony_offset(chrony),
            "git": git_state(),
            "config_sha256": self.config_hashes(),
            "common": self.common,
            "protocol_overrides": self.overrides,     # non-empty = not a protocol run (tests, pilot tuning)
            "sensor_setup_detected": detect_sensor_setup(),
            "planner_process": self.planner_process(),
            "started": self.started.isoformat(timespec="seconds"),
            "t_wall_launch": time.time(),
        }


# ===================================================================== processes

class Procs:
    def __init__(self, run_dir, run_id, env):
        self.run_dir, self.env = run_dir, dict(env, EB_RUN_ID=run_id)
        self.run_id = run_id
        self.procs = {}          # name -> (Popen, log file, log offset)

    def start(self, name, cmd, extra_env=None):
        log = open(os.path.join(self.run_dir, "stdout", f"{name}.log"), "w")
        log.write("$ " + " ".join(shlex.quote(c) for c in cmd) + "\n")
        log.flush()
        env = dict(self.env, **(extra_env or {}))
        p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, env=env, start_new_session=True,
                             stdin=subprocess.DEVNULL)
        self.procs[name] = [p, log, 0]
        return p

    def alive(self, name) -> bool:
        return name in self.procs and self.procs[name][0].poll() is None

    def died_nodes(self, name) -> list:
        """New 'process has died' lines in a launch log since the last call."""
        if name not in self.procs:
            return []
        entry = self.procs[name]
        path = os.path.join(self.run_dir, "stdout", f"{name}.log")
        try:
            with open(path, errors="replace") as f:
                f.seek(entry[2])
                text = f.read()
                entry[2] = f.tell()
        except OSError:
            return []
        return re.findall(r"\[ERROR\] \[([^\]]+)\]: process has died", text)

    def log_contains(self, name, text) -> bool:
        try:
            with open(os.path.join(self.run_dir, "stdout", f"{name}.log"), errors="replace") as f:
                return text in f.read()
        except OSError:
            return False

    def stop(self, name, grace=10.0):
        if name not in self.procs:
            return
        p, log, _ = self.procs[name]
        if p.poll() is None:
            for sig, wait in ((signal.SIGINT, grace), (signal.SIGKILL, 3.0)):
                try:
                    os.killpg(p.pid, sig)
                except (ProcessLookupError, PermissionError):
                    break
                try:
                    p.wait(wait)
                    break
                except subprocess.TimeoutExpired:
                    continue
        log.close()

    def kill_leftovers(self) -> list:
        """Kill every process that still carries this run's EB_RUN_ID (Linux /proc)."""
        killed = []
        if not os.path.isdir("/proc"):
            return killed
        tag = f"EB_RUN_ID={self.run_id}".encode()
        for d in os.listdir("/proc"):
            if not d.isdigit() or int(d) == os.getpid():
                continue
            try:
                with open(f"/proc/{d}/environ", "rb") as f:
                    if tag not in f.read().split(b"\0"):
                        continue
                with open(f"/proc/{d}/comm") as f:
                    comm = f.read().strip()
                os.kill(int(d), signal.SIGKILL)
                killed.append(f"{comm}[{d}]")
            except (OSError, ProcessLookupError):
                continue
        return killed


# ===================================================================== ROS monitor

class RosMonitor:
    """Subscribes to what the monitor loop needs. Imported only when a run really starts."""

    def __init__(self):
        import rclpy
        from rclpy.node import Node
        from rclpy.parameter import Parameter
        from rclpy.qos import qos_profile_sensor_data
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import Float32
        from tf2_ros import Buffer, TransformListener

        rclpy.init(args=None)
        self.rclpy = rclpy
        self.node = Node("explorebench_run", parameter_overrides=[Parameter("use_sim_time", value=True)])
        self.sim = None
        self.coverage = None
        self.cycles = []
        self.node.create_subscription(Clock, "/clock", self._on_clock, qos_profile_sensor_data)
        self.node.create_subscription(Float32, "/exploration/coverage_live", self._on_cov, 10)
        try:
            from exploration_msgs.msg import PlanningCycle
            self.node.create_subscription(PlanningCycle, "/exploration/planning_cycle", self._on_cycle, 100)
        except ImportError:
            self.node.get_logger().warn("exploration_msgs not built: stuck detection uses planner exit only")
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self.node)
        self.last_pose = None

    def _on_clock(self, m):
        self.sim = m.clock.sec + m.clock.nanosec * 1e-9

    def _on_cov(self, m):
        self.coverage = float(m.data)

    def _on_cycle(self, m):
        self.cycles.append(m.stamp_end.sec + m.stamp_end.nanosec * 1e-9)

    def spin(self, seconds):
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.rclpy.spin_once(self.node, timeout_sec=max(0.0, min(0.1, t_end - time.time())))

    def pose_jump(self) -> float:
        from rclpy.time import Time
        try:
            t = self.tf.lookup_transform("map", "base_link", Time())
        except Exception:   # noqa: BLE001
            return 0.0
        p = (t.transform.translation.x, t.transform.translation.y, t.header.stamp.sec + t.header.stamp.nanosec * 1e-9)
        jump = 0.0
        if self.last_pose is not None and 0 < p[2] - self.last_pose[2] <= 2.0:
            jump = math.hypot(p[0] - self.last_pose[0], p[1] - self.last_pose[1])
        self.last_pose = p
        return jump

    def wait_nav2(self, timeout_wall_base, procs, infra) -> tuple:
        """Wait until this run's Nav2 is up: the navigate_to_pose server answers AND our own
        Nav2 launch log reports "Managed nodes are active" (so a stray server from another
        process on the same ROS domain cannot pass for it). The wall timeout is stretched by
        1 / RTF (measured from /clock while waiting), so a slow machine is not marked invalid
        for being slow. -> (ok, reason)."""
        from nav2_msgs.action import NavigateToPose
        from rclpy.action import ActionClient
        client = ActionClient(self.node, NavigateToPose, "navigate_to_pose")
        w0, s0 = time.time(), None
        while True:
            self.spin(1.0)
            if client.server_is_ready() and procs.log_contains("nav2", "Managed nodes are active"):
                return True, f"Nav2 ready after {time.time() - w0:.0f} s wall"
            for name in infra:
                if not procs.alive(name):
                    return False, f"{name} exited while waiting for Nav2"
            if self.sim is not None and s0 is None:
                s0, ws0 = self.sim, time.time()
            rtf = 1.0
            if s0 is not None and time.time() - ws0 > 5:
                rtf = max(0.05, min(1.0, (self.sim - s0) / (time.time() - ws0)))
            if time.time() - w0 > timeout_wall_base / rtf:
                return False, f"Nav2 not ready after {time.time() - w0:.0f} s wall (RTF ~{rtf:.2f})"

    def set_param(self, node: str, name: str, value: str, timeout_wall=120.0) -> str:
        """Set a string parameter on another node through its set_parameters service."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        c = self.node.create_client(SetParameters, f"{node}/set_parameters")
        t0 = time.time()
        while not c.service_is_ready():
            self.spin(0.5)
            if time.time() - t0 > timeout_wall:
                return f"{node}/set_parameters not available"
        req = SetParameters.Request()
        req.parameters = [Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                                                     string_value=value))]
        fut = c.call_async(req)
        while not fut.done() and time.time() - t0 < timeout_wall:
            self.spin(0.2)
        if not fut.done():
            return f"{node} {name}: no reply"
        r = fut.result().results[0]
        return f"{node} {name}={value}: {'ok' if r.successful else 'refused ' + r.reason}"

    def close(self):
        self.node.destroy_node()
        if self.rclpy.ok():
            self.rclpy.shutdown()


# ===================================================================== main run

def run(a) -> int:
    plan = Plan(a)
    common = plan.common
    if a.power_mode not in LAPTOP_MODES:
        q = run_text(["nvpmodel", "-q"])
        actual = parse_nvpmodel(q)
        if actual != a.power_mode and not a.dry_run:
            print(f"refusing to run: --power-mode {a.power_mode} but nvpmodel -q reports {actual!r}",
                  file=sys.stderr)
            return 1
    os.makedirs(os.path.join(plan.run_dir, "stdout"), exist_ok=True)
    if plan.params_path:
        os.makedirs(os.path.dirname(plan.params_path), exist_ok=True)
        write_yaml(plan.params_path, effective_params(a.method, load_yaml(plan.method_cfg_path), common))
    manifest = plan.manifest_start()
    mpath = os.path.join(plan.run_dir, "manifest.yaml")
    write_yaml(mpath, manifest)

    steps = [("sim", plan.sim_cmd()), ("slam", plan.slam_cmd()), ("nav2", plan.nav2_cmd())]
    if a.jetson:
        steps.append(("power_logger", plan.power_cmd()))
    steps.append(("recorder", plan.recorder_cmd()))
    if a.method != "none":
        steps.append(("planner", plan.planner_cmd()))
    if a.dry_run:
        for name, cmd in steps:
            print(f"{name:13s} {' '.join(shlex.quote(c) for c in cmd)}")
        print(f"manifest: {mpath}")
        return 0

    procs = Procs(plan.run_dir, plan.run_id, os.environ)
    mon = None
    outcome, reason, notes = "invalid", "orchestrator error", []
    t_ros_start = t_wall_start = None
    try:
        mon = RosMonitor()
        sim_env = {"LIBGL_ALWAYS_SOFTWARE": "1"} if a.software_gl and not a.sim_ssh else None
        procs.start("sim", plan.sim_cmd(), sim_env)
        procs.start("slam", plan.slam_cmd())
        procs.start("nav2", plan.nav2_cmd())
        infra = ["sim", "slam", "nav2"]
        ok, why = mon.wait_nav2(float(a.nav2_timeout or common["nav2_ready_timeout_s"]), procs, infra)
        notes.append(why)
        if not ok:
            raise RuntimeError(why)
        # The TB4 sim spawns the robot docked, facing its dock. The Create 3 undock behaviour
        # does not survive a slow sim (it is cancelled and then blocks Nav2's velocities), so the
        # dock model is removed from the world instead: the robot starts free at its start pose.
        # backup_only lifts the Create 3's cap on driving backwards (Nav2's backup recovery).
        # Both are the same for every method (docs/common_rules.md rule 5).
        notes.append(mon.set_param("/motion_control", "safety_override",
                                   str(common.get("robot_safety_override", "backup_only"))))
        rc = subprocess.call(plan.remove_dock_cmd(), stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                             timeout=90)
        notes.append(f"dock removed (ign service exit {rc})")
        if a.jetson:
            procs.start("power_logger", plan.power_cmd())
            infra.append("power_logger")
        procs.start("recorder", plan.recorder_cmd())
        infra.append("recorder")
        mon.spin(2.0)
        t_ros_start, t_wall_start = mon.sim, time.time()
        manifest.update({"t_ros_start": t_ros_start, "t_wall_start": t_wall_start})
        write_yaml(mpath, manifest)      # the recorder and the evaluator read t_ros_start from here
        if a.method != "none":
            procs.start("planner", plan.planner_cmd())

        om = OutcomeMonitor(common, t_ros_start if t_ros_start is not None else 0.0, time.time())
        idle_end = time.time() + float(a.duration) if a.method == "none" else None
        n_seen = 0
        planner_died = []
        while True:
            mon.spin(1.0)
            for t in mon.cycles[n_seen:]:
                om.on_cycle(t)
            n_seen = len(mon.cycles)
            dead = [n for n in infra if not procs.alive(n)]
            for n in ("sim", "slam", "nav2"):
                died = [d for d in procs.died_nodes(n) if d not in common.get("ignore_died", [])]
                dead += [f"{n}/{d}" for d in died]
            planner_died += procs.died_nodes("planner")
            planner_alive = a.method == "none" or (procs.alive("planner") and not planner_died)
            res = om.step(time.time(), mon.sim, mon.coverage, planner_alive, dead, mon.pose_jump())
            if idle_end and res is None and time.time() >= idle_end:
                res = ("idle", f"idle baseline, {a.duration:.0f} s wall")
            if res:
                outcome, reason = res
                break
    except KeyboardInterrupt:
        outcome, reason = "invalid", "interrupted by the operator (Ctrl-C)"
    except Exception as e:  # noqa: BLE001 - any orchestrator failure is an infrastructure failure
        outcome, reason = "invalid", f"{type(e).__name__}: {e}"
    finally:
        t_wall_end = time.time()
        t_ros_end = mon.sim if mon else None
        n_cycles = len(mon.cycles) if mon else 0
        last_cov = mon.coverage if mon else None
        for name in ("planner", "recorder", "power_logger", "nav2", "slam"):
            procs.stop(name)
        if a.sim_ssh and "sim" in procs.procs:
            try:
                subprocess.call(plan.remote_stop_cmd(), stdout=subprocess.DEVNULL,
                                stderr=subprocess.DEVNULL, timeout=40)
            except subprocess.TimeoutExpired:
                notes.append("remote sim stop timed out")
        procs.stop("sim")
        time.sleep(1.0)
        leftovers = procs.kill_leftovers()
        if mon:
            mon.close()

    rtf = None
    if t_ros_start is not None and t_ros_end is not None and t_wall_start:
        rtf = (t_ros_end - t_ros_start) / max(t_wall_end - t_wall_start, 1e-9)
    manifest.update({
        "outcome": outcome, "reason": reason, "notes": notes,
        "t_wall_end": t_wall_end, "t_ros_end": t_ros_end,
        "sim_duration_s": None if t_ros_start is None or t_ros_end is None else t_ros_end - t_ros_start,
        "wall_duration_s": None if t_wall_start is None else t_wall_end - t_wall_start,
        "rtf": rtf, "n_planning_cycles": n_cycles, "coverage_live_last": last_cov,
        "leftovers_killed": leftovers,
        "ended": dt.datetime.now().isoformat(timespec="seconds"),
    })
    write_yaml(mpath, manifest)
    print(f"{plan.run_id}: {outcome} ({reason})" + (f", RTF {rtf:.2f}" if rtf else ""))
    if leftovers:
        print(f"killed leftovers: {', '.join(leftovers)}", file=sys.stderr)

    if a.method != "none" and t_ros_start is not None:
        osc = str(common.get("oscillation_threshold_m", 1.0))
        subprocess.call([sys.executable, os.path.join(REPO, "analysis", "coverage.py"), plan.run_dir,
                         "--osc-threshold", osc])
    return 0 if outcome in DATA_OUTCOMES else 2


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--block", choices=BLOCKS, default="P")
    ap.add_argument("--method", choices=METHODS, required=True)
    ap.add_argument("--map", default="m1_house")
    ap.add_argument("--pose", type=int, default=0)
    ap.add_argument("--power-mode", required=True, help="laptop | mac | an nvpmodel name (docs/power_modes.md)")
    ap.add_argument("--jetson", action="store_true", help="running on the Jetson: also start the power logger")
    ap.add_argument("--sim-ssh", help="ssh host that runs Gazebo (Block E: the laptop)")
    ap.add_argument("--sim-repo", help="repo path on the --sim-ssh host (default: same as here)")
    ap.add_argument("--software-gl", action="store_true", help="render the sim on the CPU (WSL, no GPU)")
    ap.add_argument("--power-source", choices=["jtop", "sysfs", "fake"], default="jtop")
    ap.add_argument("--duration", type=float, default=300.0, help="--method none: wall seconds of idle logging")
    ap.add_argument("--nav2-timeout", type=float, help="override nav2_ready_timeout_s (wall, before RTF scaling)")
    ap.add_argument("--timeout-sim", type=float,
                    help="override timeout_sim_s (tests and pilot tuning only; recorded as protocol_overrides)")
    ap.add_argument("--logs-dir", default=os.path.join(REPO, "logs"))
    ap.add_argument("--dry-run", action="store_true", help="write the manifest and print the commands only")
    a = ap.parse_args(argv)
    if a.method == "none":
        a.block = "P"           # unused in the idle run_id
    return a


def main(argv=None) -> int:
    return run(parse_args(argv))


if __name__ == "__main__":
    sys.exit(main())

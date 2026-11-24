#!/usr/bin/env python3
"""Power and compute logger (Module 14 of instructions.md). Runs ON THE JETSON.

    python3 harness/power_logger.py --out logs/<run_id> \
        --procs explore,frontier_explorer,rl_planner,async_slam_tool,controller_server,planner_server,bt_navigator,recorder

Writes into --out (schemas: docs/log_format.md, Section 0.4), all on one t_wall clock:

    power.csv  t_wall, vdd_in_mW, vdd_cpu_gpu_cv_mW, vdd_soc_mW, temp_cpu_C, temp_gpu_C
               every sample of the power source (jtop interval, default 0.1 s)
    procs.csv  t_wall, pid, name, utime_s, stime_s   once per second per matching process
               (cumulative CPU seconds from /proc/<pid>/stat)
    gpu.csv    t_wall, gpu_util_pct, gpu_freq_MHz, ram_used_MB   once per second

--procs matches a process when a name equals its /proc/<pid>/comm, or appears as a word
in its command line (ROS 2 Python nodes all have comm "python3", so the script name in the
command line is what identifies them). The `name` column is the --procs entry that matched.
/proc is rescanned every --rescan seconds, so a node that restarts is picked up again.

Sources (--source):
  jtop   (default) jetson-stats. VERIFY the rail names once with
         `python3 scripts/power_smoke_test.py --list-rails`.
  sysfs  the INA3221 hwmon nodes and thermal zones directly (Module 2 step 8 fallback,
         if jtop's real sample rate is too slow). VERIFY the hwmon path on the board.
  fake   synthetic values, for testing the logger on any Linux machine.

Stops cleanly on SIGINT/SIGTERM (files flushed every second and closed), or after
--duration seconds.
"""
from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import re
import signal
import sys
import threading
import time

POWER_HEADER = ["t_wall", "vdd_in_mW", "vdd_cpu_gpu_cv_mW", "vdd_soc_mW", "temp_cpu_C", "temp_gpu_C"]
PROCS_HEADER = ["t_wall", "pid", "name", "utime_s", "stime_s"]
GPU_HEADER = ["t_wall", "gpu_util_pct", "gpu_freq_MHz", "ram_used_MB"]
RAILS = ["VDD_IN", "VDD_CPU_GPU_CV", "VDD_SOC"]
CLK = os.sysconf("SC_CLK_TCK") if hasattr(os, "sysconf") else 100


# ---------------------------------------------------------------- /proc helpers

def cpu_times(pid: int):
    """(utime_s, stime_s) of a process, or None if it is gone."""
    try:
        with open(f"/proc/{pid}/stat") as f:
            fields = f.read().rsplit(")", 1)[1].split()
    except (OSError, IndexError):
        return None
    return int(fields[11]) / CLK, int(fields[12]) / CLK      # fields 14, 15 in `man proc`


def _read(path: str) -> str:
    try:
        with open(path, "rb") as f:
            return f.read().replace(b"\0", b" ").decode("utf-8", "replace").strip()
    except OSError:
        return ""


def match_processes(names, self_pid=None) -> dict:
    """{pid: matched name} for every process whose comm equals a name or whose command
    line contains the name as a word (path components and '__node:=' count as separators)."""
    out = {}
    pats = {n: re.compile(r"(^|[\s/=:])" + re.escape(n) + r"($|[\s.:_]|\.py)") for n in names}
    for d in os.listdir("/proc"):
        if not d.isdigit():
            continue
        pid = int(d)
        if pid == self_pid:
            continue
        comm = _read(f"/proc/{pid}/comm")
        cmd = _read(f"/proc/{pid}/cmdline")
        for n in names:
            if comm == n or (cmd and pats[n].search(cmd)):
                out[pid] = n
                break
    return out


# ----------------------------------------------------------------------- sources

class FakeSource:
    def __init__(self, interval):
        self.interval, self.t0 = interval, time.time()

    def ok(self):
        return True

    def wait(self):
        time.sleep(self.interval)

    def power(self):
        t = time.time() - self.t0
        base = 5000 + 400 * math.sin(t)
        return [base, 0.35 * base, 0.25 * base, 45.0 + 0.1 * t, 44.0 + 0.1 * t]

    def gpu(self):
        return [10.0, 306.0, 2100.0]

    def close(self):
        pass


class JtopSource:
    def __init__(self, interval):
        from jtop import jtop
        self.j = jtop(interval=interval)
        self.j.start()

    def ok(self):
        return self.j.ok()          # blocks until the next sample

    def wait(self):
        pass

    @staticmethod
    def _temp(temps: dict, key: str):
        for k, v in temps.items():
            if k.lower() == key:
                if isinstance(v, dict):
                    return v.get("temp") if v.get("online", True) else None
                return v
        return None

    def power(self):
        rails = self.j.power.get("rail", {})
        vals = [rails.get(r, {}).get("power") for r in RAILS]
        t = self.j.temperature
        return vals + [self._temp(t, "cpu"), self._temp(t, "gpu")]

    def gpu(self):
        # jtop 4.x layout. VERIFY units on the board: freq 'cur' is reported in kHz, RAM 'used' in kB.
        g = next(iter(self.j.gpu.values()), {}) if self.j.gpu else {}
        load = g.get("status", {}).get("load")
        freq = g.get("freq", {}).get("cur")
        ram = self.j.memory.get("RAM", {}).get("used")
        return [load, None if freq is None else freq / 1000.0, None if ram is None else ram / 1024.0]

    def close(self):
        self.j.close()


class SysfsSource:
    """INA3221 hwmon + thermal zones. Power = bus voltage (mV) * current (mA) / 1000."""

    HWMON_GLOB = "/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*"

    def __init__(self, interval):
        self.interval = interval
        self.chan = {}
        for hw in glob.glob(self.HWMON_GLOB):
            for lab in glob.glob(os.path.join(hw, "in*_label")):
                name = _read(lab).upper()
                n = re.search(r"in(\d+)_label", lab).group(1)
                if name in RAILS:
                    self.chan[name] = (os.path.join(hw, f"in{n}_input"), os.path.join(hw, f"curr{n}_input"))
        missing = [r for r in RAILS if r not in self.chan]
        if missing:
            print(f"power_logger: sysfs rails not found: {missing} (looked in {self.HWMON_GLOB})", file=sys.stderr)
        self.zones = {}
        for z in glob.glob("/sys/class/thermal/thermal_zone*"):
            t = _read(os.path.join(z, "type")).lower()
            for key in ("cpu", "gpu"):
                if t.startswith(key):
                    self.zones[key] = os.path.join(z, "temp")

    def ok(self):
        return True

    def wait(self):
        time.sleep(self.interval)

    def power(self):
        vals = []
        for r in RAILS:
            if r in self.chan:
                try:
                    vals.append(float(_read(self.chan[r][0])) * float(_read(self.chan[r][1])) / 1000.0)
                except ValueError:
                    vals.append(None)
            else:
                vals.append(None)
        temps = []
        for key in ("cpu", "gpu"):
            v = _read(self.zones.get(key, ""))
            temps.append(float(v) / 1000.0 if v else None)
        return vals + temps

    def gpu(self):
        load = _read("/sys/devices/platform/gpu.0/load") or _read("/sys/devices/gpu.0/load")
        freq = glob.glob("/sys/class/devfreq/*gpu*/cur_freq") or glob.glob("/sys/class/devfreq/17000000.gpu/cur_freq")
        mem = {}
        for line in _read("/proc/meminfo").splitlines():
            k, _, v = line.partition(":")
            mem[k] = float(v.split()[0]) if v.split() else 0.0
        used_mb = (mem.get("MemTotal", 0) - mem.get("MemAvailable", 0)) / 1024.0 if mem else None
        return [float(load) / 10.0 if load else None,                 # load is per mille
                float(_read(freq[0])) / 1e6 if freq else None,          # Hz -> MHz
                used_mb]

    def close(self):
        pass


# ------------------------------------------------------------------------ logger

def fmt(v):
    if v is None:
        return ""
    return repr(float(v)) if isinstance(v, float) else str(v)


class PowerLogger:
    def __init__(self, out, procs, source, rescan_s=5.0, proc_period_s=1.0):
        os.makedirs(out, exist_ok=True)
        self.files = {k: open(os.path.join(out, f"{k}.csv"), "w", newline="") for k in ("power", "procs", "gpu")}
        self.w = {k: csv.writer(f) for k, f in self.files.items()}
        self.w["power"].writerow(POWER_HEADER)
        self.w["procs"].writerow(PROCS_HEADER)
        self.w["gpu"].writerow(GPU_HEADER)
        self.flush()
        self.procs, self.src = procs, source
        self.rescan_s, self.proc_period_s = rescan_s, proc_period_s
        self.matched, self.last_scan, self.last_proc = {}, 0.0, 0.0
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.n_power = 0

    def flush(self):
        for f in self.files.values():
            f.flush()

    def sample_procs(self, now):
        if now - self.last_scan >= self.rescan_s:
            self.matched = match_processes(self.procs, os.getpid()) if self.procs else {}
            self.last_scan = now
        for pid, name in sorted(self.matched.items()):
            ct = cpu_times(pid)
            if ct is not None:
                self.w["procs"].writerow([repr(now), pid, name, repr(ct[0]), repr(ct[1])])
        self.w["gpu"].writerow([repr(now)] + [fmt(v) for v in self.src.gpu()])

    def run(self, duration=None):
        t_end = time.time() + duration if duration else None
        last_flush = time.time()
        while not self.stop.is_set() and self.src.ok():
            now = time.time()
            if t_end and now >= t_end:
                break
            self.w["power"].writerow([repr(now)] + [fmt(v) for v in self.src.power()])
            self.n_power += 1
            if now - self.last_proc >= self.proc_period_s:
                self.last_proc = now
                self.sample_procs(now)
            if now - last_flush >= 1.0:
                self.flush()
                last_flush = now
            self.src.wait()

    def close(self):
        for f in self.files.values():
            f.flush()
            os.fsync(f.fileno())
            f.close()
        self.src.close()


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--out", required=True, help="logs/<run_id>")
    ap.add_argument("--procs", default="", help="comma-separated process names to track")
    ap.add_argument("--source", choices=["jtop", "sysfs", "fake"], default="jtop")
    ap.add_argument("--interval", type=float, default=0.1, help="power sample period (s); jtop treats it as a request")
    ap.add_argument("--rescan", type=float, default=5.0, help="seconds between /proc rescans")
    ap.add_argument("--duration", type=float, help="stop after this many seconds")
    a = ap.parse_args(argv)

    src = {"jtop": JtopSource, "sysfs": SysfsSource, "fake": FakeSource}[a.source](a.interval)
    procs = [p.strip() for p in a.procs.split(",") if p.strip()]
    lg = PowerLogger(a.out, procs, src, a.rescan)

    def on_signal(_sig, _frame):
        lg.stop.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    t0 = time.time()
    try:
        lg.run(a.duration)
    finally:
        lg.close()
    dt = time.time() - t0
    print(f"power_logger: {lg.n_power} power rows in {dt:.1f} s ({lg.n_power / max(dt, 1e-9):.1f} Hz) -> {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""Tests for harness/power_logger.py (Module 14). Linux only (/proc); no Jetson needed.

    python3 -m pytest harness/tests/test_power_logger.py
"""
import csv
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "harness"))
LOGGER = os.path.join(REPO, "harness", "power_logger.py")

LINUX = sys.platform.startswith("linux")


def rows(path):
    with open(path, newline="") as f:
        return list(csv.reader(f))


@unittest.skipUnless(LINUX, "needs /proc")
class TestPowerLogger(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.out = os.path.join(self._tmp.name, "run")
        # A busy "planner" and an idle one, recognisable by their command line.
        busy = "import time\nwhile True: sum(range(10000))"
        idle = "import time\nwhile True: time.sleep(1)"
        self.busy = subprocess.Popen([sys.executable, "-c", busy, "fake_planner_busy"])
        self.idle = subprocess.Popen([sys.executable, "-c", idle, "fake_planner_idle"])

    def tearDown(self):
        for p in (self.busy, self.idle):
            p.kill()
            p.wait()
        self._tmp.cleanup()

    def run_logger(self, seconds=3.0, sig=signal.SIGINT):
        p = subprocess.Popen([sys.executable, LOGGER, "--out", self.out, "--source", "fake",
                              "--procs", "fake_planner_busy,fake_planner_idle", "--rescan", "1"])
        time.sleep(seconds)
        p.send_signal(sig)
        self.assertEqual(p.wait(10), 0)

    def test_files_and_schemas(self):
        self.run_logger()
        power, procs, gpu = (rows(os.path.join(self.out, f"{k}.csv")) for k in ("power", "procs", "gpu"))
        self.assertEqual(power[0], ["t_wall", "vdd_in_mW", "vdd_cpu_gpu_cv_mW", "vdd_soc_mW", "temp_cpu_C", "temp_gpu_C"])
        self.assertEqual(procs[0], ["t_wall", "pid", "name", "utime_s", "stime_s"])
        self.assertEqual(gpu[0], ["t_wall", "gpu_util_pct", "gpu_freq_MHz", "ram_used_MB"])
        self.assertGreater(len(power), 15)          # ~10 Hz for 3 s
        self.assertGreaterEqual(len(gpu), 3)        # 1 Hz
        # one t_wall base: every file's timestamps fall in the same window
        t = [float(r[0]) for f in (power, procs, gpu) for r in f[1:]]
        self.assertLess(max(t) - min(t), 10)

    def test_cpu_time_of_busy_vs_idle(self):
        self.run_logger(seconds=4.0)
        procs = rows(os.path.join(self.out, "procs.csv"))[1:]
        cpu = {}
        for t, pid, name, ut, st in procs:
            cpu.setdefault(name, []).append(float(ut) + float(st))
        self.assertIn("fake_planner_busy", cpu)
        self.assertIn("fake_planner_idle", cpu)
        busy = max(cpu["fake_planner_busy"]) - min(cpu["fake_planner_busy"])
        idle = max(cpu["fake_planner_idle"]) - min(cpu["fake_planner_idle"])
        self.assertGreater(busy, 1.0)               # ~1 CPU-second per second
        self.assertLess(idle, 0.2)

    def test_sigterm_leaves_closed_files(self):
        self.run_logger(seconds=1.5, sig=signal.SIGTERM)
        for k in ("power", "procs", "gpu"):
            with open(os.path.join(self.out, f"{k}.csv"), "rb") as f:
                self.assertTrue(f.read().endswith(b"\n"))

    def test_match_processes(self):
        import power_logger as pl
        m = pl.match_processes(["fake_planner_busy", "no_such_process_xyz"])
        self.assertIn(self.busy.pid, m)
        self.assertNotIn(self.idle.pid, m)
        self.assertEqual(m[self.busy.pid], "fake_planner_busy")


if __name__ == "__main__":
    unittest.main()

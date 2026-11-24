"""Tests for harness/run.py (Module 16): the parts that need no ROS.

    python -m pytest harness/tests/test_run.py
"""
import datetime as dt
import os
import re
import sys
import tempfile
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "harness"))
import run  # noqa: E402

with open(os.path.join(REPO, "configs", "common.yaml")) as _f:
    COMMON = yaml.safe_load(_f)


class TestNames(unittest.TestCase):
    def test_run_id_format(self):
        rid = run.make_run_id("Q", "mrtsp", "m1_house", 3, "laptop", dt.datetime(2026, 12, 1, 2, 15, 0))
        self.assertEqual(rid, "Q_mrtsp_m1_house_p03_laptop_20261201-021500")   # instructions.md 0.1 example
        self.assertRegex(rid, r"^[PQEA]_(nearest|mrtsp|ariadne|roadmap)_m\d_[a-z_0-9]+_p\d{2}_[A-Za-z0-9_]+_\d{8}-\d{6}$")

    def test_idle_run_id(self):
        rid = run.make_run_id("P", "none", "m1_house", 0, "15W", dt.datetime(2026, 12, 1, 9, 0, 0))
        self.assertEqual(rid, "idle_15W_20261201-090000")


class TestEffectiveParams(unittest.TestCase):
    def load(self, m):
        with open(os.path.join(REPO, "configs", "methods", f"{m}.yaml")) as f:
            return yaml.safe_load(f)

    def test_rates_follow_replan_period(self):
        common = dict(COMMON, replan_period_s=0.5)
        for m, node, key in (("nearest", "explore_node", "planner_frequency"),
                             ("mrtsp", "frontier_explorer", "map_processing_rate_hz"),
                             ("ariadne", "rl_planner", "replanning_frequency")):
            p = run.effective_params(m, self.load(m), common)
            self.assertEqual(p[node]["ros__parameters"][key], 2.0, m)
            self.assertTrue(p[node]["ros__parameters"]["use_sim_time"])

    def test_frontier_size(self):
        common = dict(COMMON, min_frontier_size_m=0.3)
        self.assertEqual(run.effective_params("nearest", self.load("nearest"), common)
                         ["explore_node"]["ros__parameters"]["min_frontier_size"], 0.3)
        self.assertEqual(run.effective_params("mrtsp", self.load("mrtsp"), common)
                         ["frontier_explorer"]["ros__parameters"]["min_frontier_size_cells"], 6)

    def test_method_file_is_not_modified(self):
        cfg = self.load("nearest")
        before = yaml.safe_dump(cfg)
        run.effective_params("nearest", cfg, dict(COMMON, replan_period_s=0.25))
        self.assertEqual(yaml.safe_dump(cfg), before)

    def test_gain_scale_kept(self):
        p = run.effective_params("nearest", self.load("nearest"), COMMON)
        self.assertEqual(p["explore_node"]["ros__parameters"]["gain_scale"], 0.0)


class TestParsers(unittest.TestCase):
    def test_nvpmodel(self):
        self.assertEqual(run.parse_nvpmodel("NV Power Mode: 15W\n0\n"), "15W")
        self.assertEqual(run.parse_nvpmodel("NV Power Mode: MAXN_SUPER\n2\n"), "MAXN_SUPER")
        self.assertIsNone(run.parse_nvpmodel("command not found"))

    def test_chrony(self):
        text = ("Reference ID    : C0A8F902 (192.168.249.2)\n"
                "System time     : 0.000012345 seconds slow of NTP time\n")
        self.assertAlmostEqual(run.parse_chrony_offset(text), -1.2345e-5)
        self.assertAlmostEqual(run.parse_chrony_offset(text.replace("slow", "fast")), 1.2345e-5)
        self.assertIsNone(run.parse_chrony_offset(""))


class TestOutcomeMonitor(unittest.TestCase):
    C = dict(COMMON, timeout_sim_s=100, stuck_no_cycle_sim_s=20, clock_stall_wall_s=10,
             complete_grace_s=5, coverage_target=0.95, max_pose_jump_m=1.0)

    def mon(self):
        return run.OutcomeMonitor(self.C, t_ros_start=1000.0, wall_start=0.0)

    def drive(self, m, steps):
        """steps: list of (wall, sim, coverage, planner_alive, dead, jump) -> first result."""
        for s in steps:
            r = m.step(*s)
            if r:
                return r
        return None

    def test_complete_after_grace(self):
        m = self.mon()
        self.assertIsNone(m.step(1, 1001, 0.96, True, []))
        self.assertIsNone(m.step(2, 1004, 0.96, False, []))       # grace: planner exit ignored
        self.assertEqual(m.step(3, 1006, 0.96, False, [])[0], "complete")

    def test_timeout(self):
        m = self.mon()
        m.on_cycle(1090)
        self.assertIsNone(m.step(1, 1099, 0.5, True, []))
        self.assertEqual(m.step(2, 1101, 0.5, True, [])[0], "timeout")

    def test_stuck_planner_exit(self):
        r = self.mon().step(1, 1010, 0.4, False, [])
        self.assertEqual(r[0], "stuck")

    def test_stuck_no_cycles(self):
        m = self.mon()
        m.on_cycle(1005)
        self.assertIsNone(m.step(1, 1024, 0.4, True, []))
        self.assertEqual(m.step(2, 1026, 0.4, True, [])[0], "stuck")

    def test_cycles_keep_it_alive(self):
        m = self.mon()
        for k in range(10):
            m.on_cycle(1000 + 10 * k)
            self.assertIsNone(m.step(k, 1000 + 10 * k + 5, 0.4, True, []))

    def test_invalid_infra(self):
        r = self.mon().step(1, 1001, 0.1, True, ["slam"])
        self.assertEqual(r[0], "invalid")
        self.assertIn("slam", r[1])

    def test_invalid_clock_stall(self):
        m = self.mon()
        self.assertIsNone(m.step(1, 1001, 0.1, True, []))
        self.assertIsNone(m.step(9, 1001, 0.1, True, []))
        self.assertEqual(m.step(12, 1001, 0.1, True, [])[0], "invalid")

    def test_invalid_pose_jump(self):
        self.assertEqual(self.mon().step(1, 1001, 0.1, True, [], 2.5)[0], "invalid")

    def test_infra_failure_beats_method_outcomes(self):
        r = self.mon().step(1, 2000, 0.1, False, ["nav2"])
        self.assertEqual(r[0], "invalid")


class TestDryRun(unittest.TestCase):
    def test_dry_run_writes_manifest_and_params(self):
        maps = [m for m in ("m2_office", "m1_house", "m3_warehouse")
                if os.path.exists(os.path.join(REPO, "configs", "start_poses", f"{m}.yaml"))]
        if not maps:
            self.skipTest("no start poses committed yet")
        with tempfile.TemporaryDirectory() as tmp:
            rc = run.main(["--block", "Q", "--method", "nearest", "--map", maps[0], "--pose", "2",
                           "--power-mode", "laptop", "--logs-dir", tmp, "--dry-run"])
            self.assertEqual(rc, 0)
            (rid,) = os.listdir(tmp)
            self.assertTrue(re.match(rf"Q_nearest_{maps[0]}_p02_laptop_\d{{8}}-\d{{6}}$", rid))
            with open(os.path.join(tmp, rid, "manifest.yaml")) as f:
                man = yaml.safe_load(f)
            self.assertEqual(man["pose_index"], 2)
            self.assertEqual(set(man["start_pose"]), {"x", "y", "yaw"})
            self.assertEqual(man["planner_process"], "explore")
            self.assertIn("configs/common.yaml", man["config_sha256"])
            self.assertTrue(os.path.exists(os.path.join(tmp, rid, "params", "nearest.yaml")))


if __name__ == "__main__":
    unittest.main()

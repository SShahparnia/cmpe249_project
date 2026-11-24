"""Tests for scripts/make_schedule.py, run_batch.py and status.py (Module 19)."""
import collections
import os
import subprocess
import sys
import tempfile
import textwrap
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
SCRIPTS = os.path.abspath(os.path.join(HERE, ".."))
sys.path.insert(0, SCRIPTS)
import make_schedule as ms  # noqa: E402
import run_batch as rb  # noqa: E402
import status  # noqa: E402

# A stand-in for harness/run.py: writes a manifest whose outcome comes from OUTCOMES
# (cell "method,map,pose" -> list of outcomes, one per attempt).
FAKE_RUNNER = textwrap.dedent('''
    import argparse, json, os, sys, time, yaml
    ap = argparse.ArgumentParser()
    for k in ("--block", "--method", "--map", "--pose", "--power-mode", "--logs-dir"):
        ap.add_argument(k)
    a, _ = ap.parse_known_args()
    plan = json.load(open(os.environ["FAKE_PLAN"]))
    key = f"{a.method},{a.map},{a.pose}"
    seen = plan.setdefault("_seen", {})
    n = seen.get(key, 0)
    seen[key] = n + 1
    json.dump(plan, open(os.environ["FAKE_PLAN"], "w"))
    outs = plan.get(key, ["complete"])
    outcome = outs[min(n, len(outs) - 1)]
    rid = f"{a.block}_{a.method}_{a.map}_p{int(a.pose):02d}_{a.power_mode}_{time.time_ns()}"
    os.makedirs(os.path.join(a.logs_dir, rid))
    yaml.safe_dump({"run_id": rid, "block": a.block, "method": a.method, "map": a.map,
                    "pose_index": int(a.pose), "power_mode": a.power_mode, "outcome": outcome,
                    "reason": "fake"}, open(os.path.join(a.logs_dir, rid, "manifest.yaml"), "w"))
    sys.exit(0 if outcome != "invalid" else 2)
''')


class TestSchedule(unittest.TestCase):
    def test_block_q_counts_and_streaks(self):
        rows = ms.schedule("Q", 10, "laptop", 7, ["a", "b"])
        self.assertEqual(len(rows), 90)
        cells = collections.Counter((r["method"], r["map"]) for r in rows)
        self.assertEqual(set(cells.values()), {10})
        self.assertLessEqual(ms.max_streak(rows), 2)
        self.assertEqual(collections.Counter(r["machine"] for r in rows), {"a": 45, "b": 45})

    def test_deterministic(self):
        a = ms.schedule("Q", 10, "laptop", 7, ["x"])
        b = ms.schedule("Q", 10, "laptop", 7, ["x"])
        c = ms.schedule("Q", 10, "laptop", 8, ["x"])
        self.assertEqual(a, b)
        self.assertNotEqual([r["method"] for r in a], [r["method"] for r in c])

    def test_block_e_groups_methods_per_pose(self):
        rows = ms.schedule("E", 5, "7W", 7, ["x"], streak=3)
        self.assertEqual(len(rows), 45)
        self.assertTrue(all(r["machine"] == "jetson" and r["power_mode"] == "7W" for r in rows))
        orders = set()
        for i in range(0, 45, 3):
            trio = rows[i:i + 3]
            self.assertEqual(len({(r["map"], r["pose"]) for r in trio}), 1)
            orders.add(tuple(r["method"] for r in trio))
        self.assertGreater(len(orders), 1)          # method order varies across poses

    def test_break_streaks_keeps_rows(self):
        rows = [{"method": m, "i": i} for i, m in enumerate("aaaaabbbcc")]
        fixed = ms.break_streaks(rows)
        self.assertEqual(sorted(r["i"] for r in fixed), list(range(10)))
        self.assertLessEqual(ms.max_streak(fixed), 3)


class TestRunBatch(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        t = self._tmp.name
        self.logs = os.path.join(t, "logs")
        self.runner = os.path.join(t, "fake_run.py")
        with open(self.runner, "w") as f:
            f.write(FAKE_RUNNER)
        self.plan = os.path.join(t, "plan.json")
        self.sched = os.path.join(t, "P.csv")
        ms.write(self.sched, ms.schedule("P", 1, "laptop", 7, ["laptop"], maps=["m1_house"]))
        self.run_log = os.path.join(t, "run_log.md")

    def tearDown(self):
        self._tmp.cleanup()

    def batch(self, plan):
        import json
        with open(self.plan, "w") as f:
            json.dump(plan, f)
        env = dict(os.environ, FAKE_PLAN=self.plan)
        subprocess.check_call([sys.executable, os.path.join(SCRIPTS, "run_batch.py"), self.sched,
                               "--machine", "laptop", "--logs-dir", self.logs, "--runner", self.runner,
                               "--who", "test", "--run-log", self.run_log], env=env,
                              stdout=subprocess.DEVNULL)
        return rb.existing_runs(self.logs)

    def test_all_complete(self):
        runs = self.batch({})
        self.assertEqual(len(runs), 3)
        self.assertTrue(all(o == "complete" for lst in runs.values() for _, o in lst))

    def test_invalid_retried_then_succeeds(self):
        runs = self.batch({"mrtsp,m1_house,0": ["invalid", "invalid", "complete"]})
        mrtsp = runs[("P", "mrtsp", "m1_house", 0, "laptop")]
        self.assertEqual([o for _, o in mrtsp], ["invalid", "invalid", "complete"])
        with open(self.run_log) as f:
            text = f.read()
        self.assertEqual(text.count("invalid: fake"), 2)
        self.assertIn("rerun 2 of row", text)

    def test_retry_limit(self):
        runs = self.batch({"nearest,m1_house,0": ["invalid"]})
        self.assertEqual(len(runs[("P", "nearest", "m1_house", 0, "laptop")]), 3)   # 1 + 2 retries

    def test_skips_done_cells(self):
        self.batch({})
        runs = self.batch({})                     # second pass: nothing new
        self.assertEqual(sum(len(v) for v in runs.values()), 3)

    def test_status(self):
        self.batch({"ariadne,m1_house,0": ["stuck"]})
        text, _ = status.summarize([self.sched], self.logs)
        self.assertIn("cells done: 3 / 3", text)
        self.assertIn("stuck 1", text)


if __name__ == "__main__":
    unittest.main()

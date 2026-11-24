"""Tests for analysis/load.py, sample_size.py and pilot_report.py (Module 17)."""
import os
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import fake_logs  # noqa: E402
import load  # noqa: E402
import pilot_report  # noqa: E402
import sample_size  # noqa: E402


class TestSampleSizeFormula(unittest.TestCase):
    def test_matches_hand_calculation(self):
        # sd = 0.1 exactly, k = 8 -> z = norm(1 - 0.05/6) + norm(0.8)
        from scipy.stats import chi2, norm
        r = np.array([-0.2, -0.1, 0.0, -0.1, -0.2, 0.0, -0.1, -0.15, -0.05])
        sd = r.std(ddof=1)
        z = norm.ppf(1 - 0.05 / 6) + norm.ppf(0.8)
        expect_point = int(np.ceil((z * sd / 0.1) ** 2 / 0.864))
        expect_safe = int(np.ceil((z * sd * np.sqrt(8 / chi2.ppf(0.2, 8)) / 0.1) ** 2 / 0.864))
        self.assertEqual(sample_size.n_needed(r), (expect_point, expect_safe))

    def test_more_noise_needs_more_poses(self):
        a = sample_size.n_needed(np.linspace(-0.1, 0.1, 9))
        b = sample_size.n_needed(np.linspace(-0.3, 0.3, 9))
        self.assertLess(a[1], b[1])


class TestPilotPipeline(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.logs = self._tmp.name

    def tearDown(self):
        self._tmp.cleanup()

    def test_load_runs(self):
        fake_logs.make_block(self.logs, "P", poses=range(3))
        runs = load.load_runs(self.logs, "P")
        self.assertEqual(len(runs), 27)
        self.assertEqual(set(runs["method"]), {"nearest", "mrtsp", "ariadne"})
        self.assertTrue((runs["latency_median_s"] > 0).all())
        self.assertTrue(runs["d_95"].notna().all())

    def test_relative_differences_recover_effect(self):
        fake_logs.make_block(self.logs, "P", poses=range(3), effects={"mrtsp": -0.2, "ariadne": 0.0}, noise=0.01)
        runs = load.load_runs(self.logs, "P")
        rd = sample_size.relative_differences(runs, "mrtsp")
        self.assertEqual(len(rd), 9)
        self.assertAlmostEqual(rd["r"].mean(), -0.2, delta=0.03)

    def test_censored_runs_use_total_distance(self):
        fake_logs.make_block(self.logs, "P", poses=range(3), timeouts={("mrtsp", "m1_house", 1)})
        runs = load.load_runs(self.logs, "P")
        t = runs[(runs["method"] == "mrtsp") & (runs["map"] == "m1_house") & (runs["pose"] == 1)].iloc[0]
        self.assertTrue(t["censored"])
        self.assertTrue(np.isnan(t["d_95"]))
        self.assertEqual(t["d_95_or_total"], t["total_distance_m"])
        res = sample_size.analyse(runs)
        self.assertEqual(res["methods"]["mrtsp"]["n_censored"], 1)

    def test_decision_rule(self):
        fake_logs.make_block(self.logs, "P", poses=range(3), noise=0.02)
        res = sample_size.analyse(load.load_runs(self.logs, "P"))
        self.assertIn("10 poses", res["decision"])
        with tempfile.TemporaryDirectory() as noisy:
            fake_logs.make_block(noisy, "P", poses=range(3), noise=0.4, seed=3)
            res = sample_size.analyse(load.load_runs(noisy, "P"))
            self.assertIn("15 poses", res["decision"])

    def test_report_is_written(self):
        fake_logs.make_block(self.logs, "P", poses=range(3))
        text = pilot_report.build(self.logs, "P")
        for needle in ("## 1. Results", "## 2. Sample size", "timeout_sim_s", "replan_period_s",
                       "Oscillation threshold", "Wall time per run", "n_safe"):
            self.assertIn(needle, text)
        results = text.split("## 1. Results")[1].split("## 2.")[0]
        self.assertEqual(results.count("| m1_house |"), 9)


if __name__ == "__main__":
    unittest.main()

"""Tests for the analysis pipeline (Module 20): stats, Q1, Q2, Q3, H2 and make_figures."""
import json
import math
import os
import shutil
import sys
import tempfile
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, HERE)
import fake_logs  # noqa: E402
import make_figures  # noqa: E402
import stats  # noqa: E402


class TestStats(unittest.TestCase):
    def test_bootstrap_ci_covers_median(self):
        rng = np.random.default_rng(0)
        x = rng.normal(3.0, 1.0, 40)
        lo, hi = stats.bootstrap_ci(x)
        self.assertLess(lo, np.median(x))
        self.assertGreater(hi, np.median(x))
        self.assertEqual((lo, hi), stats.bootstrap_ci(x))        # seeded: reproducible

    def test_holm(self):
        adj = stats.holm([0.01, 0.04, 0.03])
        np.testing.assert_allclose(adj, [0.03, 0.06, 0.06])
        adj = stats.holm([0.5, math.nan, 0.01])
        self.assertTrue(math.isnan(adj[1]))
        np.testing.assert_allclose(adj[[0, 2]], [0.5, 0.02])

    def test_wilcoxon(self):
        self.assertLess(stats.wilcoxon_p(np.arange(1, 11)), 0.01)
        self.assertTrue(math.isnan(stats.wilcoxon_p([0, 0, 0])))

    def test_loglog_slope_recovers_exponent(self):
        import pandas as pd
        rng = np.random.default_rng(1)
        rows = []
        for run in range(8):
            n = rng.integers(2, 200, 50)
            rows.append(pd.DataFrame({"n_frontiers": n, "latency_s": 1e-4 * n ** 2.0 * np.exp(rng.normal(0, 0.05, 50)),
                                      "run_id": f"r{run}"}))
        s = stats.loglog_slope(pd.concat(rows))
        self.assertAlmostEqual(s["slope"], 2.0, delta=0.05)
        self.assertLess(s["ci_low"], 2.0)
        self.assertGreater(s["ci_high"], 2.0)

    def test_ratio_of_medians(self):
        k, (lo, hi) = stats.ratio_of_medians_ci([10, 12, 8, 11, 9], [2, 2, 2, 2, 2])
        self.assertAlmostEqual(k, 5.0)
        self.assertLessEqual(lo, 5.0)
        self.assertGreaterEqual(hi, 5.0)
        k, _ = stats.ratio_of_medians_ci([1, 1, 1], [-1, -2, -1])
        self.assertTrue(math.isinf(k))


class TestPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls._tmp = tempfile.TemporaryDirectory()
        cls.logs = os.path.join(cls._tmp.name, "logs")
        os.makedirs(cls.logs)
        fake_logs.make_block(cls.logs, "Q", poses=range(10), effects={"mrtsp": -0.25, "ariadne": 0.0},
                             noise=0.04, seed=11, timeouts={("ariadne", "m3_warehouse", 4)})
        fake_logs.make_block(cls.logs, "E", poses=range(5), effects={"mrtsp": -0.25, "ariadne": -0.02},
                             power_modes=("7W", "MAXN_SUPER"), noise=0.04, seed=12)
        fake_logs.make_block(cls.logs, "A", poses=range(5), maps=["m2_office"], power_modes=("MAXN_SUPER",), seed=13)
        for pm, w in (("7W", 4.0), ("MAXN_SUPER", 6.5)):
            fake_logs.make_idle(cls.logs, pm, w)
        cls.out = os.path.join(cls._tmp.name, "out")
        cls.numbers = make_figures.build(cls.logs, cls.out)

    @classmethod
    def tearDownClass(cls):
        cls._tmp.cleanup()

    def test_q1_detects_effect(self):
        q1 = self.numbers["q1"]
        for mp in ("m1_house", "m2_office", "m3_warehouse"):
            c = q1["mrtsp"][mp]
            self.assertEqual(c["n"], 10)
            self.assertLess(c["ci_high"], 0)
            self.assertAlmostEqual(c["median_rel"], -0.25, delta=0.06)
            self.assertTrue(c["h1_supported"])
            self.assertGreaterEqual(c["p_holm"], c["p"])
        self.assertEqual(q1["ariadne"]["m3_warehouse"]["n_censored"], 1)

    def test_q2_latency_and_energy(self):
        q2 = self.numbers["q2"]
        a = q2["ariadne"]["m1_house"]["7W"]
        n = q2["nearest"]["m1_house"]["7W"]
        self.assertGreater(a["latency_median_s"], n["latency_median_s"])
        self.assertIsNotNone(a["energy_above_idle_J_median"])
        self.assertEqual(a["runs_without_idle"], 0)
        self.assertGreater(a["energy_above_idle_J_median"], n["energy_above_idle_J_median"])
        self.assertGreater(a["planner_cpu_s_median"], n["planner_cpu_s_median"])

    def test_q3_break_even(self):
        be = self.numbers["q3"]["break_even"]["m1_house"]["7W"]
        self.assertTrue(be["mrtsp"]["break_even_exists"])
        self.assertIsNotNone(be["mrtsp"]["k_star_J_per_m"])
        self.assertEqual(be["mrtsp"]["n_pairs"], 5)
        self.assertIsNone(be["mrtsp"]["h3_supported"])     # K_LOCO_MIN not set yet

    def test_h2_slopes(self):
        h2 = self.numbers["h2"]
        self.assertTrue(h2["mrtsp"]["7W"]["h2_supported"])
        self.assertGreater(h2["mrtsp"]["7W"]["slope"], h2["nearest"]["7W"]["slope"])
        self.assertTrue(self.numbers["ablation_h2"]["exploratory"])

    def test_figures_written(self):
        for f in ("fig_q1_coverage.png", "fig_q2_latency.png", "fig_q3_pareto.png", "fig_q3_break_even.png",
                  "fig_h2_scaling.png", "fig_h2_ablation.png"):
            self.assertIn(f, self.numbers["figures"])
            self.assertTrue(os.path.getsize(os.path.join(self.out, f)) > 10_000)
        self.assertEqual(self.numbers["missing"], [])

    def test_rebuild_is_identical(self):
        out2 = os.path.join(self._tmp.name, "out2")
        make_figures.build(self.logs, out2)
        with open(os.path.join(self.out, "numbers.json")) as f1, open(os.path.join(out2, "numbers.json")) as f2:
            self.assertEqual(json.load(f1), json.load(f2))
        shutil.rmtree(out2)

    def test_missing_blocks_are_listed(self):
        with tempfile.TemporaryDirectory() as tmp:
            fake_logs.make_block(tmp, "Q", poses=range(3))
            n = make_figures.build(tmp, os.path.join(tmp, "out"))
            self.assertIn("Block E (Q2, Q3/H3, H2)", n["missing"])
            self.assertIn("q1", n)


if __name__ == "__main__":
    unittest.main()

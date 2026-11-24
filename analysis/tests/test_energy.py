"""Tests for analysis/energy.py (Module 14)."""
import csv
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import energy  # noqa: E402


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        w.writerows(rows)


class TestEnergy(unittest.TestCase):
    def test_constant_power(self):
        t = np.arange(0, 10.01, 0.1)
        self.assertAlmostEqual(energy.energy_J(t, np.full_like(t, 5000.0), 2.0, 7.0), 25.0, places=6)

    def test_ramp_is_exact_with_trapezoid(self):
        t = np.linspace(0, 4, 41)
        p = 1000.0 * t                        # 0..4 W
        self.assertAlmostEqual(energy.energy_J(t, p, 0, 4), 8.0, places=9)

    def test_window_outside_samples(self):
        t = np.arange(0, 5.0, 0.5)
        self.assertEqual(energy.energy_J(t, np.ones_like(t), 10, 20), 0.0)

    def test_nan_samples_are_skipped(self):
        t = np.arange(0, 4.01, 1.0)
        p = np.array([1000, np.nan, 1000, 1000, 1000.0])
        self.assertAlmostEqual(energy.energy_J(t, p, 0, 4), 4.0)

    def test_above_idle(self):
        self.assertAlmostEqual(energy.energy_above_idle(100.0, 4.0, 10.0, 30.0), 20.0)

    def test_cpu_seconds_handles_restart(self):
        procs = {
            "t_wall": np.array([0, 1, 2, 3, 4, 5.0]),
            "pid": np.array([10, 10, 10, 11, 11, 11]),
            "name": np.array(["planner"] * 6),
            "utime_s": np.array([1.0, 1.5, 2.0, 0.0, 0.4, 0.8]),
            "stime_s": np.array([0.0, 0.1, 0.2, 0.0, 0.0, 0.1]),
        }
        self.assertAlmostEqual(energy.cpu_seconds(procs, "planner"), 1.2 + 0.9)
        self.assertAlmostEqual(energy.cpu_seconds(procs, "planner", 0, 1), 0.6)
        self.assertEqual(energy.cpu_seconds(procs, "slam"), 0.0)

    def test_run_energy_folder(self):
        with tempfile.TemporaryDirectory() as tmp:
            run, idle = os.path.join(tmp, "run"), os.path.join(tmp, "idle")
            os.makedirs(run)
            os.makedirs(idle)
            hdr = ["t_wall", "vdd_in_mW", "vdd_cpu_gpu_cv_mW", "vdd_soc_mW", "temp_cpu_C", "temp_gpu_C"]
            write_csv(os.path.join(run, "power.csv"), hdr,
                      [[100 + 0.1 * i, 6000, 2000, 1500, 50, 49] for i in range(301)])     # 30 s at 6 W
            write_csv(os.path.join(idle, "power.csv"), hdr,
                      [[0 + 0.1 * i, 4000, 1000, 1500, 45, 44] for i in range(101)])       # 4 W idle
            write_csv(os.path.join(run, "procs.csv"), ["t_wall", "pid", "name", "utime_s", "stime_s"],
                      [[100 + i, 7, "explore", 0.5 * i, 0.1 * i] for i in range(31)])
            with open(os.path.join(run, "manifest.yaml"), "w") as f:
                f.write("t_wall_start: 105.0\nt_wall_end: 125.0\nplanner_process: explore\n")
            r = energy.run_energy(run, idle)
            self.assertAlmostEqual(r["energy_window_s"], 20.0)
            self.assertAlmostEqual(r["energy_board_J"], 120.0, places=6)
            self.assertAlmostEqual(r["power_idle_W"], 4.0, places=6)
            self.assertAlmostEqual(r["energy_above_idle_J"], 40.0, places=6)
            self.assertAlmostEqual(r["planner_cpu_s"], 0.6 * 20, places=6)
            self.assertAlmostEqual(r["power_sample_hz"], 10.0, places=6)


if __name__ == "__main__":
    unittest.main()

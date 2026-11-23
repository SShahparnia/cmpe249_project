"""Tests for analysis/coverage.py (Module 13).

Run from the repo root with either:
    python -m pytest analysis/tests
    python -m unittest discover -s analysis/tests
"""
import csv
import json
import math
import os
import sys
import tempfile
import unittest

import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import coverage as cov  # noqa: E402

RES = 0.05
ROWS, COLS = 20, 30          # world GT grid, row 0 = lowest y
GT_ORIGIN = (-1.0, -0.5)     # world position of cell [0, 0]'s lower-left corner
ROOM_INTERIOR = (slice(6, 10), slice(21, 26))   # 4 x 5 = 20 free but unreachable cells


def make_gt_grid():
    """Outer wall ring, open interior, and a closed room (walls rows 5..10, cols 20..26)
    whose inside is free but unreachable. Returns (occupied, reachable) with row 0 = low y."""
    occ = np.zeros((ROWS, COLS), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    occ[5, 20:27] = occ[10, 20:27] = occ[5:11, 20] = occ[5:11, 26] = True
    reach = ~occ
    reach[ROOM_INTERIOR] = False
    return occ, reach


def write_map_dir(map_dir, occ, reach):
    os.makedirs(map_dir, exist_ok=True)
    grid = np.where(occ, 0, 254).astype(np.uint8)          # black = occupied, white = free
    cov.write_pgm(os.path.join(map_dir, "gt.pgm"), np.flipud(grid))   # images: row 0 = top
    np.save(os.path.join(map_dir, "gt_reachable.npy"), np.flipud(reach))
    with open(os.path.join(map_dir, "gt.yaml"), "w") as f:
        yaml.safe_dump({"image": "gt.pgm", "resolution": RES,
                        "origin": [GT_ORIGIN[0], GT_ORIGIN[1], 0.0], "negate": 0,
                        "occupied_thresh": 0.65, "free_thresh": 0.25}, f)


def expected_reference():
    """Hand-derived: every cell except the closed room's interior counts
    (all wall cells touch reachable free space, at least diagonally)."""
    ref = np.ones((ROWS, COLS), dtype=bool)
    ref[ROOM_INTERIOR] = False
    return ref


def slam_values(known_world):
    """SLAM grid values for a world-aligned known mask: known cells get 0 or 100,
    unknown cells get -1 or 50 (50 is below OCC_MIN, so it must count as unknown)."""
    rng = np.random.default_rng(0)
    vals = np.where(rng.random(known_world.shape) < 0.5, 0, 100)
    unk = np.where(rng.random(known_world.shape) < 0.5, -1, 50)
    return np.where(known_world, vals, unk).astype(np.int8)


class TmpMapMixin:
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = self._tmp.name
        self.maps_dir = os.path.join(self.tmp, "maps")
        occ, reach = make_gt_grid()
        write_map_dir(os.path.join(self.maps_dir, "m_test"), occ, reach)

    def tearDown(self):
        self._tmp.cleanup()


class TestGroundTruth(TmpMapMixin, unittest.TestCase):
    def test_reference_excludes_unreachable_free_space(self):
        gt = cov.GroundTruth.load(os.path.join(self.maps_dir, "m_test"))
        np.testing.assert_array_equal(gt.reference_mask(), expected_reference())
        self.assertEqual(int(gt.reference_mask().sum()), ROWS * COLS - 20)

    def test_pgm_round_trip_with_comment(self):
        img = (np.arange(12, dtype=np.uint8) * 20).reshape(3, 4)
        p = os.path.join(self.tmp, "c.pgm")
        with open(p, "wb") as f:
            f.write(b"P5\n# made by map_saver\n4 3\n255\n" + img.tobytes())
        np.testing.assert_array_equal(cov.read_pgm(p), img)


class TestSyntheticFraction(TmpMapMixin, unittest.TestCase):
    """A fake SLAM map with a known fraction filled must return that fraction."""

    def test_known_fraction(self):
        ref_mask = expected_reference()
        known_world = np.zeros((ROWS, COLS), dtype=bool)
        known_world[:, :10] = True                    # left third known
        expected = (ref_mask & known_world).sum() / ref_mask.sum()
        self.assertAlmostEqual(expected, 200 / 580)

        ref = cov.CoverageReference.from_map_dir(os.path.join(self.maps_dir, "m_test"), (0.0, 0.0, 0.0))
        data = slam_values(known_world)
        frac = ref.fraction(data.ravel(), COLS, ROWS, RES, GT_ORIGIN[0], GT_ORIGIN[1], 0.0)
        self.assertAlmostEqual(frac, expected, places=12)

    def test_empty_and_full(self):
        ref = cov.CoverageReference.from_map_dir(os.path.join(self.maps_dir, "m_test"), (0.0, 0.0, 0.0))
        unknown = np.full((ROWS, COLS), -1, dtype=np.int8)
        self.assertEqual(ref.fraction(unknown, COLS, ROWS, RES, *GT_ORIGIN), 0.0)
        full = np.zeros((ROWS, COLS), dtype=np.int8)
        self.assertEqual(ref.fraction(full, COLS, ROWS, RES, *GT_ORIGIN), 1.0)

    def test_cells_outside_slam_grid_are_unknown(self):
        ref = cov.CoverageReference.from_map_dir(os.path.join(self.maps_dir, "m_test"), (0.0, 0.0, 0.0))
        # SLAM grid only covers world columns 0..9, all known.
        small = np.zeros((ROWS, 10), dtype=np.int8)
        frac = ref.fraction(small, 10, ROWS, RES, *GT_ORIGIN)
        self.assertAlmostEqual(frac, 200 / 580)


class TestFrameAlignment(TmpMapMixin, unittest.TestCase):
    """Shift / rotate the SLAM frame and confirm the result does not change after alignment."""

    def setUp(self):
        super().setUp()
        rng = np.random.default_rng(249)
        self.known_world = rng.random((ROWS, COLS)) < 0.6
        ref_mask = expected_reference()
        self.expected = (ref_mask & self.known_world).sum() / ref_mask.sum()

    def ref(self, start_pose):
        return cov.CoverageReference.from_map_dir(os.path.join(self.maps_dir, "m_test"), start_pose)

    def test_shifted_slam_origin(self):
        ref = self.ref((0.0, 0.0, 0.0))
        data = slam_values(self.known_world)
        for pad in (0, 3, 17):   # slam_toolbox grows the map and moves its origin
            padded = np.pad(data, pad, constant_values=-1)
            frac = ref.fraction(padded, COLS + 2 * pad, ROWS + 2 * pad, RES,
                                GT_ORIGIN[0] - pad * RES, GT_ORIGIN[1] - pad * RES, 0.0)
            self.assertAlmostEqual(frac, self.expected, places=12, msg=f"pad={pad}")

    def test_translated_start_pose(self):
        x0, y0 = 0.35, -0.20      # robot spawned here; SLAM map frame starts here
        ref = self.ref((x0, y0, 0.0))
        data = slam_values(self.known_world)
        frac = ref.fraction(data, COLS, ROWS, RES, GT_ORIGIN[0] - x0, GT_ORIGIN[1] - y0, 0.0)
        self.assertAlmostEqual(frac, self.expected, places=12)

    def test_rotated_start_pose(self):
        # Spawn yaw = +90 deg. Then x_map = y_w - y0 and y_map = -(x_w - x0), so the
        # map-frame grid is the world grid transposed and flipped (built here by hand).
        x0, y0 = 0.40, 0.10
        ref = self.ref((x0, y0, math.pi / 2))
        data_m = slam_values(self.known_world).T[::-1, :]
        origin_x = GT_ORIGIN[1] - y0
        origin_y = x0 - GT_ORIGIN[0] - COLS * RES
        for pad in (0, 5):
            padded = np.pad(data_m, pad, constant_values=-1)
            frac = ref.fraction(padded, ROWS + 2 * pad, COLS + 2 * pad, RES,
                                origin_x - pad * RES, origin_y - pad * RES, 0.0)
            self.assertAlmostEqual(frac, self.expected, places=12, msg=f"pad={pad}")


class TestRunMetrics(unittest.TestCase):
    def test_path_length_filters_jitter_but_keeps_slow_motion(self):
        rng = np.random.default_rng(1)
        jx, jy = rng.normal(0, 0.002, 600), rng.normal(0, 0.002, 600)    # 1 min stationary @ 10 Hz
        self.assertLess(cov.path_length_cumulative(jx, jy)[-1], 0.05)
        x = np.arange(0, 1.0 + 1e-9, 0.005)                              # 5 mm steps, 1 m
        # A per-segment filter would give 0 here. The anchor filter can drop at most the
        # last < MIN_STEP_M of the path.
        total = cov.path_length_cumulative(x, np.zeros_like(x))[-1]
        self.assertLessEqual(1.0 - total, cov.MIN_STEP_M)
        self.assertLessEqual(total, 1.0 + 1e-9)

    def test_first_crossing_interpolates(self):
        t = np.array([0.0, 5.0, 10.0, 15.0])
        c = np.array([0.2, 0.9, 1.0, 1.0])
        d = np.array([0.0, 2.0, 4.0, 6.0])
        t95, d95 = cov.first_crossing(t, c, d, 0.95)
        self.assertAlmostEqual(t95, 7.5)
        self.assertAlmostEqual(d95, 3.0)
        self.assertEqual(cov.first_crossing(t, c * 0.5, d, 0.95), (None, None))

    def test_oscillation_count(self):
        goals_t = [1, 2, 3, 4, 5, 6]
        goals_xy = [(0, 0), (0.1, 0),   # A, then A re-sent (same goal)
                    (5, 0),             # B: A succeeded at t=1.5 -> not an oscillation
                    (5, 5),             # C: B never resolved -> oscillation
                    (5.3, 5),           # C nudged (< X) -> ignored
                    (0, 5)]             # D: C aborted at t=5.5 -> not an oscillation
        res_t, res_s = [1.5, 3.5, 5.5], [4, 5, 6]   # t=3.5 is a CANCELED, does not count
        self.assertEqual(cov.oscillation_count(goals_t, goals_xy, res_t, res_s, 1.0), 1)
        self.assertEqual(cov.oscillation_count([], np.zeros((0, 2)), [], [], 1.0), 0)


class TestEndToEnd(TmpMapMixin, unittest.TestCase):
    def write_run(self, outcome="complete"):
        run = os.path.join(self.tmp, "logs", "Q_nearest_m_test_p00_laptop_20261201-000000")
        os.makedirs(os.path.join(run, "maps"))
        with open(os.path.join(run, "manifest.yaml"), "w") as f:
            yaml.safe_dump({"run_id": os.path.basename(run), "map": "m_test", "pose_index": 0,
                            "start_pose": {"x": 0.0, "y": 0.0, "yaw": 0.0},
                            "t_ros_start": 100.0, "outcome": outcome}, f)
        with open(os.path.join(run, "pose.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_wall", "t_ros", "x", "y", "yaw"])
            for i in range(0, 201):          # 100 s..120 s, 0.1 m/s along x
                t = 100.0 + i * 0.1
                w.writerow([1e9 + t, t, 0.01 * i, 0.0, 0.0])
        with open(os.path.join(run, "goals.csv"), "w", newline="") as f:
            csv.writer(f).writerow(["t_wall", "t_ros", "cycle_id", "goal_x", "goal_y"])
        with open(os.path.join(run, "nav2_results.csv"), "w", newline="") as f:
            w = csv.writer(f)          # goal_id is a hex UUID string, as the recorder writes it
            w.writerow(["t_wall", "t_ros", "goal_id", "status"])
            w.writerow([1e9 + 101.0, 101.0, "64c3aacfa6283e77a939501312125c45", 2])
            w.writerow([1e9 + 104.0, 104.0, "64c3aacfa6283e77a939501312125c45", 4])
        # Snapshots every 5 s: known columns grow 0 -> 30.
        for seq, (t, k) in enumerate([(100.0, 0), (105.0, 10), (110.0, 25), (115.0, 30), (120.0, 30)]):
            known = np.zeros((ROWS, COLS), dtype=bool)
            known[:, :k] = True
            np.savez(os.path.join(run, "maps", f"map_{seq:05d}.npz"), data=slam_values(known).ravel(),
                     resolution=RES, origin_x=GT_ORIGIN[0], origin_y=GT_ORIGIN[1], origin_yaw=0.0,
                     width=COLS, height=ROWS, t_ros=t, t_wall=1e9 + t)
        with open(os.path.join(run, "summary.json"), "w") as f:
            json.dump({"from_analysis": 42}, f)
        return run

    def test_cli_writes_coverage_and_summary(self):
        run = self.write_run()
        self.assertEqual(cov.main([run, "--maps-dir", self.maps_dir]), 0)

        with open(os.path.join(run, "coverage.csv"), newline="") as f:
            rows = list(csv.reader(f))
        self.assertEqual(rows[0], ["t_ros", "distance_m", "coverage_frac"])
        self.assertEqual(len(rows), 6)
        c = [float(r[2]) for r in rows[1:]]
        self.assertAlmostEqual(c[1], 200 / 580, places=5)
        self.assertAlmostEqual(c[-1], 1.0)

        with open(os.path.join(run, "summary.json")) as f:
            s = json.load(f)
        self.assertEqual(s["from_analysis"], 42)          # merged, not overwritten
        # Coverage at 110 s: cols < 25 minus the room cells with cols 21..24 (4 x 4 = 16).
        c110 = (ROWS * 25 - 16) / 580
        self.assertAlmostEqual(c[2], c110, places=5)
        f = (0.95 - c110) / (1.0 - c110)
        self.assertAlmostEqual(s["t_95"], 10.0 + 5.0 * f, places=3)
        self.assertAlmostEqual(s["d_95"], 1.0 + 0.5 * f, places=3)
        self.assertAlmostEqual(s["total_distance_m"], 2.0, places=6)
        self.assertAlmostEqual(s["total_time_s"], 20.0, places=6)
        self.assertEqual(s["final_coverage"], 1.0)
        self.assertEqual(s["outcome"], "complete")
        self.assertEqual(s["oscillation_count"], 0)
        self.assertEqual(s["coverage_warnings"], [])

    def test_warns_when_live_and_offline_disagree(self):
        run = self.write_run(outcome="stuck")
        res = cov.evaluate_run(run, self.maps_dir)
        self.assertTrue(any("stuck" in w for w in res["summary"]["coverage_warnings"]))

    def test_overlay_figure(self):
        run = self.write_run()
        res = cov.evaluate_run(run, self.maps_dir)
        out = os.path.join(self.tmp, "overlay.png")
        cov.write_overlay(out, res)
        self.assertGreater(os.path.getsize(out), 1000)


if __name__ == "__main__":
    unittest.main()

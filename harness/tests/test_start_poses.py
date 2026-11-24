"""Tests for harness/scripts/gen_start_poses.py (Module 11)."""
import math
import os
import sys
import tempfile
import unittest

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "harness", "scripts"))
sys.path.insert(0, os.path.join(REPO, "analysis"))
sys.path.insert(0, os.path.join(REPO, "analysis", "tests"))
import coverage as cov  # noqa: E402
import gen_start_poses as gsp  # noqa: E402
from test_coverage import write_map_dir  # noqa: E402

RES = 0.05


def room(rows=160, cols=200):
    """10 x 8 m room with a 2 x 2 m pillar in the middle."""
    occ = np.zeros((rows, cols), dtype=bool)
    occ[0, :] = occ[-1, :] = occ[:, 0] = occ[:, -1] = True
    occ[60:100, 80:120] = True
    return occ, ~occ


class TestStartPoses(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.maps = os.path.join(self._tmp.name, "maps")
        self.out = os.path.join(self._tmp.name, "poses")
        occ, reach = room()
        write_map_dir(os.path.join(self.maps, "m_t"), occ, reach)
        self.gt = cov.GroundTruth.load(os.path.join(self.maps, "m_t"))

    def tearDown(self):
        self._tmp.cleanup()

    def gen(self, *extra):
        gsp.main(["--map", "m_t", "--n", "12", "--maps-dir", self.maps, "--out-dir", self.out, *extra])
        with open(os.path.join(self.out, "m_t.yaml")) as f:
            return yaml.safe_load(f)

    def test_clearance_and_separation(self):
        doc = self.gen()
        poses = [(p["x"], p["y"]) for p in doc["poses"]]
        self.assertEqual(len(poses), 12)
        bad = ~(self.gt.reachable & self.gt.free)
        rows, cols = np.nonzero(bad)
        ox, oy, _ = self.gt.origin
        bx, by = ox + (cols + 0.5) * RES, oy + (rows + 0.5) * RES
        for x, y in poses:
            self.assertGreaterEqual(np.min(np.hypot(bx - x, by - y)), 0.35 - 1e-9)
        for i in range(len(poses)):
            for j in range(i):
                self.assertGreaterEqual(math.dist(poses[i], poses[j]), 1.5)
        self.assertTrue(all(-math.pi <= p["yaw"] < math.pi for p in doc["poses"]))

    def test_deterministic(self):
        a = self.gen()["poses"]
        b = self.gen()["poses"]
        self.assertEqual(a, b)
        c = self.gen("--seed", "250")["poses"]
        self.assertNotEqual(a, c)

    def test_replace_keeps_the_others(self):
        before = self.gen()["poses"]
        doc = self.gen("--replace", "2", "--reason", "test")
        after = doc["poses"]
        self.assertNotEqual(after[2], before[2])
        self.assertEqual([p for i, p in enumerate(after) if i != 2], [p for i, p in enumerate(before) if i != 2])
        self.assertEqual(doc["replaced"][0]["index"], 2)
        # a second replacement draws further along the stream, not the same point again
        doc2 = self.gen("--replace", "2", "--reason", "test again")
        self.assertNotEqual(doc2["poses"][2], after[2])
        self.assertEqual(len(doc2["replaced"]), 2)


if __name__ == "__main__":
    unittest.main()

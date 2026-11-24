"""Tests for maps/tools/world_to_gt.py (Module 10).

    python -m pytest maps/tools/tests
"""
import math
import os
import sys
import tempfile
import textwrap
import unittest

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "analysis"))
import coverage as cov  # noqa: E402
import world_to_gt as w2g  # noqa: E402

RES = 0.05


def box_model(name, x, y, sx, sy, sz, yaw=0.0):
    return f"""
    <model name="{name}"><static>true</static><pose>{x} {y} {sz / 2} 0 0 {yaw}</pose>
      <link name="link"><collision name="c"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision></link>
    </model>"""


def walls(prefix, cx, cy, w, h, t=0.1, z=1.0, door=None):
    """Four walls of a w x h rectangle centred at (cx, cy). door = width of a gap in the south wall."""
    out = [box_model(f"{prefix}_n", cx, cy + h / 2, w + t, t, z),
           box_model(f"{prefix}_e", cx + w / 2, cy, t, h + t, z),
           box_model(f"{prefix}_w", cx - w / 2, cy, t, h + t, z)]
    if door:
        seg = (w + t - door) / 2
        out += [box_model(f"{prefix}_s1", cx - (w + t) / 2 + seg / 2, cy - h / 2, seg, t, z),
                box_model(f"{prefix}_s2", cx + (w + t) / 2 - seg / 2, cy - h / 2, seg, t, z)]
    else:
        out.append(box_model(f"{prefix}_s", cx, cy - h / 2, w + t, t, z))
    return "".join(out)


def make_world(tmp, extra="", outer_door=None):
    import trimesh
    mdir = os.path.join(tmp, "models", "crate")
    os.makedirs(os.path.join(mdir, "meshes"))
    trimesh.creation.box(extents=[0.4, 0.4, 0.4]).export(os.path.join(mdir, "meshes", "crate.obj"))
    with open(os.path.join(mdir, "model.config"), "w") as f:
        f.write("<model><name>crate</name><sdf version='1.6'>model.sdf</sdf></model>")
    with open(os.path.join(mdir, "model.sdf"), "w") as f:
        f.write("""<sdf version="1.6"><model name="crate"><pose>0 0 0.2 0 0 0</pose><link name="l">
          <collision name="c"><geometry><mesh><uri>model://crate/meshes/crate.obj</uri>
          <scale>2 2 1</scale></mesh></geometry></collision></link></model></sdf>""")
    world = f"""<?xml version="1.0"?>
    <sdf version="1.6"><world name="t">
      <model name="ground"><static>true</static><link name="l"><collision name="c"><geometry>
        <plane><normal>0 0 1</normal><size>100 100</size></plane></geometry></collision></link></model>
      {walls("room", 0, 0, 6.0, 4.0, door=outer_door)}
      {walls("closet", 2.0, 1.0, 1.0, 1.0)}
      {box_model("low", -2.0, -1.0, 1.0, 1.0, 0.10)}
      {box_model("col", 0.0, 1.0, 0.4, 0.4, 1.0, yaw=math.pi / 4)}
      <include><uri>model://crate</uri><name>crate_1</name><pose>-2.0 1.0 0.2 0 0 0</pose></include>
      {extra}
    </world></sdf>"""
    with open(os.path.join(tmp, "t.sdf"), "w") as f:
        f.write(textwrap.dedent(world))
    cfg = {"world": "t.sdf", "model_paths": ["models"], "resolution": RES, "seed": [0.0, -1.0], "margin": 0.5}
    with open(os.path.join(tmp, "gt_config.yaml"), "w") as f:
        yaml.safe_dump(cfg, f)
    return w2g.load_config(tmp)


class TestWorldToGt(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.tmp = self._tmp.name

    def tearDown(self):
        self._tmp.cleanup()

    def at(self, out, x, y, key):
        r, c = out["grid"].cell(x, y)
        return bool(out[key][int(r), int(c)])

    def test_occupancy_and_reachability(self):
        cfg = make_world(self.tmp)
        out = w2g.build(self.tmp, cfg)
        self.assertEqual(out["world"].warnings, [])
        # walls are occupied, open floor is reachable
        self.assertTrue(self.at(out, 0.0, 2.0, "occ"))
        self.assertTrue(self.at(out, -3.0, 0.0, "occ"))
        self.assertTrue(self.at(out, 1.0, -1.0, "reach"))
        # closed closet: free inside but unreachable
        self.assertFalse(self.at(out, 2.0, 1.0, "occ"))
        self.assertFalse(self.at(out, 2.0, 1.0, "reach"))
        # outside the building: free, unreachable
        self.assertFalse(self.at(out, -3.3, 0.0, "occ"))
        self.assertFalse(self.at(out, -3.3, 0.0, "reach"))
        # low box (0.10 m): below the lidar, so free; robot cannot drive in, so not reachable
        self.assertFalse(self.at(out, -2.0, -1.0, "occ"))
        self.assertTrue(self.at(out, -2.0, -1.0, "blocking"))
        self.assertFalse(self.at(out, -2.0, -1.0, "reach"))
        self.assertTrue(self.at(out, -2.0, -1.6, "reach"))

    def test_rotated_box(self):
        out = w2g.build(self.tmp, make_world(self.tmp))
        self.assertTrue(self.at(out, 0.0, 1.0, "occ"))            # centre
        self.assertTrue(self.at(out, 0.0, 1.0 + 0.26, "occ"))     # a 45-degree corner reaches 0.28 m
        self.assertFalse(self.at(out, 0.18, 1.18, "occ"))         # axis-aligned corner: outside when rotated

    def test_included_scaled_mesh(self):
        out = w2g.build(self.tmp, make_world(self.tmp))
        # 0.4 m cube scaled x2 in x, y -> 0.8 m footprint centred at (-2, 1)
        self.assertTrue(self.at(out, -2.0 + 0.35, 1.0, "occ"))    # filled interior, not just outline
        self.assertTrue(self.at(out, -2.0, 1.0, "occ"))
        self.assertFalse(self.at(out, -2.0 + 0.5, 1.0, "occ"))

    def test_files_roundtrip_through_coverage_loader(self):
        cfg = make_world(self.tmp)
        out = w2g.build(self.tmp, cfg)
        info = w2g.write(self.tmp, cfg, out)
        gt = cov.GroundTruth.load(self.tmp)
        np.testing.assert_array_equal(gt.occupied, out["occ"])
        np.testing.assert_array_equal(gt.reachable, out["reach"])
        self.assertAlmostEqual(gt.origin[0], out["grid"].x0, places=6)
        self.assertEqual(info["cells"]["reachable_free"], int(out["reach"].sum()))
        self.assertEqual(info["cells"]["reference"], int(gt.reference_mask().sum()))
        self.assertEqual(info["cells"]["free"] + info["cells"]["occupied"], out["grid"].rows * out["grid"].cols)
        # Reference = reachable free plus the obstacle cells that border it: the closet interior
        # and outside stay out, the room walls are in.
        ref = gt.reference_mask()
        r, c = out["grid"].cell(2.0, 1.0)
        self.assertFalse(ref[r, c])
        # the south wall (y = -2.05 .. -1.95) has reference cells on its inner face, none outside
        col = [out["grid"].cell(0.0, y) for y in np.arange(-2.2, -1.8, RES / 2)]
        wall_ref = [bool(ref[r, c] and gt.occupied[r, c]) for r, c in col]
        self.assertTrue(any(wall_ref))
        r, c = out["grid"].cell(0.0, -2.2)
        self.assertFalse(ref[r, c])

    def test_open_world_is_rejected(self):
        cfg = make_world(self.tmp, outer_door=1.0)
        with self.assertRaises(ValueError):
            w2g.build(self.tmp, cfg)

    def test_seed_in_obstacle_is_rejected(self):
        cfg = make_world(self.tmp)
        cfg["seed"] = [0.0, 1.0]
        with self.assertRaises(ValueError):
            w2g.build(self.tmp, cfg)


class TestHelpers(unittest.TestCase):
    def test_pose_matrix_yaw(self):
        T = w2g.pose_matrix("1 2 3 0 0 1.5707963")
        np.testing.assert_allclose(T @ [1, 0, 0, 1], [1, 3, 3, 1], atol=1e-6)

    def test_closed_components(self):
        square = np.array([[[0, 0], [1, 0]], [[1, 0], [1, 1]], [[1, 1], [0, 1]], [[0, 1], [0, 0]]], float)
        dangling = np.array([[[5, 5], [6, 5]]], float)
        closed = w2g.closed_components(np.concatenate([square, dangling]))
        self.assertEqual(closed.tolist(), [True, True, True, True, False])

    def test_evenodd_fill_ring(self):
        g = w2g.Grid(0.0, 0.0, 1.0, 10, 10)
        outer = [[1, 1], [9, 1], [9, 9], [1, 9]]
        inner = [[3, 3], [7, 3], [7, 7], [3, 7]]
        segs = [[p, q] for loop in (outer, inner) for p, q in zip(loop, loop[1:] + loop[:1])]
        m = g.blank()
        w2g.fill_evenodd(m, g, np.array(segs, float))
        self.assertTrue(m[1, 1])        # in the ring
        self.assertFalse(m[5, 5])       # in the hole
        self.assertFalse(m[0, 0])       # outside

    def test_collada_unit(self):
        with tempfile.NamedTemporaryFile("w", suffix=".DAE", delete=False) as f:
            f.write('<COLLADA><asset><unit meter="0.010000" name="centimeter"/></asset></COLLADA>')
        try:
            self.assertAlmostEqual(w2g._collada_unit(f.name), 0.01)
        finally:
            os.unlink(f.name)


if __name__ == "__main__":
    unittest.main()

"""Tests for maps/tools/convert_aws_world.py (Module 10)."""
import os
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
import convert_aws_world as cv  # noqa: E402

MODEL = """<?xml version="1.0"?><sdf version="1.6"><model name="aws_thing"><pose>{pose}</pose>
  <link name="link"><inertial><mass>80</mass><inertia><ixx>1</ixx><iyy>0</iyy><izz>1</izz></inertia></inertial>
  <collision name="c"><geometry><mesh><uri>model://aws_thing/meshes/c.DAE</uri></mesh></geometry></collision>
  <visual name="v"><geometry><mesh><uri>model://aws_thing/meshes/v.DAE</uri></mesh></geometry></visual></link>
  <static>0</static></model></sdf>"""

WORLD = """<?xml version="1.0"?><sdf version="1.6"><world name="default">
  <physics name="default_physics" default="0" type="ode"><max_step_size>0.001</max_step_size></physics>
  <scene><ambient>0.4 0.4 0.4 1</ambient><background>0.7 0.7 0.7 1</background></scene>
  <model name="Thing_001"><static>true</static><include><uri>model://aws_thing</uri></include>
    <pose frame="">1.5 -2 0.01 0 -0 1.5708</pose></model>
  <!-- <model name="Gone"><include><uri>model://aws_thing</uri></include></model> -->
  <light name="lamp" type="point"><pose frame="">0 0 2 0 0 0</pose><diffuse>0.5 0.5 0.5 1</diffuse></light>
</world></sdf>"""


class TestConvert(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        t = self._tmp.name
        os.makedirs(os.path.join(t, "repo", "models", "aws_thing"))
        os.makedirs(os.path.join(t, "repo", "worlds", "sub"))
        self.world = os.path.join(t, "repo", "worlds", "sub", "w.world")
        with open(self.world, "w") as f:
            f.write(WORLD)
        self.model_sdf = os.path.join(t, "repo", "models", "aws_thing", "model.sdf")

    def tearDown(self):
        self._tmp.cleanup()

    def convert(self, model_pose="0 0 0 0 0 0"):
        with open(self.model_sdf, "w") as f:
            f.write(MODEL.format(pose=model_pose))
        text, stats = cv.convert(self.world, "m_test", floor=(-2, 2, -2, 2))
        return ET.fromstring(text), stats

    def test_world_structure(self):
        root, stats = self.convert()
        w = root.find("world")
        self.assertEqual(w.get("name"), "m_test")
        self.assertEqual(w.find("physics").get("type"), "ignored")
        plugins = {p.get("name") for p in w.findall("plugin")}
        self.assertIn("ignition::gazebo::systems::Physics", plugins)
        self.assertEqual(stats["includes"], 1)          # the commented-out model stays out
        self.assertEqual(stats["lights"], 1)
        self.assertIsNone(w.find("light/pose").get("frame"))
        ground = w.find("model[@name='ground_plane']")
        self.assertEqual(len(ground.findall("link/visual")), 4)    # 2 x 2 tiles of 2 m

    def test_inlined_model(self):
        root, _ = self.convert()
        m = root.find("world/model[@name='Thing_001']")
        self.assertIsNotNone(m)
        self.assertEqual(m.findtext("static"), "true")
        self.assertEqual(m.findtext("pose").split()[:2], ["1.5", "-2"])
        self.assertEqual(len(m.findall(".//inertial")), 0)         # invalid inertia removed
        self.assertEqual(m.findtext("link/visual/geometry/mesh/uri"), "model://aws_thing/meshes/v.DAE")
        self.assertEqual(len([s for s in m.findall("static")]), 1)

    def test_nonzero_model_pose_is_refused(self):
        with self.assertRaises(NotImplementedError):
            self.convert("0 0 0.5 0 0 0")


if __name__ == "__main__":
    unittest.main()

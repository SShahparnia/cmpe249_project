"""Tests for analysis/fill_report.py and scripts/make_demo_video.py (Module 22)."""
import os
import shutil
import subprocess
import sys
import tempfile
import unittest

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "analysis"))
sys.path.insert(0, HERE)
import fake_logs  # noqa: E402
import fill_report  # noqa: E402
import make_figures  # noqa: E402

TEMPLATE = os.path.join(REPO, "docs", "report", "report_template.md")


class TestFill(unittest.TestCase):
    def test_formats(self):
        self.assertEqual(fill_report.fmt(0.1234, "pct"), "12.3%")
        self.assertEqual(fill_report.fmt(0.01234, "ms"), "12.3 ms")
        self.assertEqual(fill_report.fmt(0.0004, "p"), "< 0.001")
        self.assertEqual(fill_report.fmt(None, "f2"), "n/a")
        self.assertEqual(fill_report.fmt(True, None), "yes")

    def test_missing_keys_are_reported(self):
        text, missing = fill_report.fill("a {{x.y|f1}} b {{z}}", {"x": {"y": 1.234}})
        self.assertEqual(text, "a 1.2 b **MISSING(z)**")
        self.assertEqual(missing, ["z"])

    def test_template_fills_completely_from_pipeline(self):
        with tempfile.TemporaryDirectory() as tmp:
            logs = os.path.join(tmp, "logs")
            os.makedirs(logs)
            fake_logs.make_block(logs, "Q", poses=range(6), seed=21)
            fake_logs.make_block(logs, "E", poses=range(4), power_modes=("7W", "MAXN_SUPER"), seed=22)
            for pm in ("7W", "MAXN_SUPER"):
                fake_logs.make_idle(logs, pm, 5.0)
            numbers = make_figures.build(logs, os.path.join(tmp, "out"))
            with open(TEMPLATE, encoding="utf-8") as f:
                text, missing = fill_report.fill(f.read(), numbers)
            self.assertEqual(missing, [], f"template keys not produced by make_figures: {missing}")
            self.assertNotIn("{{", text)


@unittest.skipUnless(shutil.which("ffmpeg"), "needs ffmpeg")
class TestDemoVideo(unittest.TestCase):
    def test_side_by_side(self):
        with tempfile.TemporaryDirectory() as tmp:
            args = []
            for i, method in enumerate(("nearest", "mrtsp")):
                rid = fake_logs.make_run(tmp, "E", method, "m1_house", 0, "7W", 6.0, seed=i)
                run = os.path.join(tmp, rid)
                with open(os.path.join(run, "manifest.yaml")) as f:
                    t0 = float(yaml.safe_load(f)["t_wall_start"])
                vid = os.path.join(tmp, f"{method}.mp4")
                subprocess.check_call(["ffmpeg", "-loglevel", "error", "-f", "lavfi", "-i",
                                       "testsrc=size=320x240:rate=10", "-t", "40", "-pix_fmt", "yuv420p", vid])
                args += ["--run", run, "--video", vid, "--video-start", str(t0 - 2.0)]
            out = os.path.join(tmp, "demo.mp4")
            subprocess.check_call([sys.executable, os.path.join(REPO, "scripts", "make_demo_video.py"), *args,
                                   "--out", out], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.assertGreater(os.path.getsize(out), 10_000)
            with open(out + ".txt") as f:
                self.assertIn("E_nearest_m1_house_p00_7W", f.read())


if __name__ == "__main__":
    unittest.main()

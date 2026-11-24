# harness

ROS 2 pieces of the benchmark. Build only this folder (the submodules in `methods/` are built separately):

```bash
colcon build --base-paths harness && source install/setup.bash
```

| Path | Module | What |
|------|--------|------|
| `exploration_msgs/` | 9 | `PlanningCycle.msg`, published on `/exploration/planning_cycle` by each method's timing patch |
| `recorder.py` | 12 | `python3 harness/recorder.py --out logs/<run_id>`: writes `pose.csv`, `goals.csv`, `cycles.csv`, `nav2_results.csv`, `maps/map_<seq>.npz`; publishes `/exploration/coverage_live`. Formats: `docs/log_format.md` |
| `power_logger.py` | 14 | Jetson: `python3 harness/power_logger.py --out logs/<run_id> --procs <names>`: writes `power.csv`, `procs.csv`, `gpu.csv` (jtop, or `--source sysfs` / `fake`) |
| `run.py` | 16 | one run end to end: `python3 harness/run.py --block Q --method mrtsp --map m1_house --pose 3 --power-mode laptop` (`--dry-run` to see the commands). Writes the manifest, launches sim, SLAM, Nav2, recorder, planner, monitors the outcome, shuts down, runs the coverage evaluator |
| `scripts/gen_start_poses.py` | 11 | start poses per map → `configs/start_poses/<map>.yaml` |
| `scripts/check_start_poses.py` | 11 | spawns the robot at each start pose and checks it (not pushed, nothing inside it, SLAM starts) |
| `tests/smoke_recorder.py` | 12 | Recorder test without Gazebo: fakes `/clock`, TF, `/map`, planning cycles and Nav2 status, stops the recorder with SIGINT/SIGTERM, checks every file and runs `analysis/coverage.py` on it |

The recorder reads `map` and `start_pose` from `<run>/manifest.yaml` for live coverage (or `--map`, `--start-pose`); without ground truth it still records everything.

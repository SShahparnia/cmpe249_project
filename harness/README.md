# harness

ROS 2 pieces of the benchmark. Build only this folder (the submodules in `methods/` are built separately):

```bash
colcon build --base-paths harness && source install/setup.bash
```

| Path | Module | What |
|------|--------|------|
| `exploration_msgs/` | 9 | `PlanningCycle.msg`, published on `/exploration/planning_cycle` by each method's timing patch |
| `recorder.py` | 12 | `python3 harness/recorder.py --out logs/<run_id>`: writes `pose.csv`, `goals.csv`, `cycles.csv`, `nav2_results.csv`, `maps/map_<seq>.npz`; publishes `/exploration/coverage_live`. Formats: `docs/log_format.md` |
| `tests/smoke_recorder.py` | 12 | Recorder test without Gazebo: fakes `/clock`, TF, `/map`, planning cycles and Nav2 status, stops the recorder with SIGINT/SIGTERM, checks every file and runs `analysis/coverage.py` on it |

The recorder reads `map` and `start_pose` from `<run>/manifest.yaml` for live coverage (or `--map`, `--start-pose`); without ground truth it still records everything.

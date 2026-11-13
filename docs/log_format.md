# Log format

Source of truth for run IDs, folder layout, timestamps, CSV schemas and outcomes. Copied from `instructions.md` Section 0. If you add a column, append it at the end of the table and note it in the changelog below. Do not rename columns after `prereg-v1`.

## 0.1 Names

Use exactly these strings everywhere: file names, CLI flags, CSV columns and plot labels.

| Thing | Allowed values |
|-------|----------------|
| `method` | `nearest`, `mrtsp`, `ariadne` (or `roadmap` if the fallback is triggered) |
| `map` | `m1_house`, `m2_office`, `m3_warehouse` (and `m2_office_2x` for the ablation) |
| `block` | `P` (pilot), `Q` (quality), `E` (energy), `A` (ablation) |
| `power_mode` | `laptop` for Blocks P and Q; for Block E, the nvpmodel name as listed in `docs/power_modes.md` (e.g. `7W`, `MAXN_SUPER`) |
| `pose` | integer index into `configs/start_poses/<map>.yaml`, starting at 0 |

Run ID format: `<block>_<method>_<map>_p<pose>_<power_mode>_<YYYYmmdd-HHMMSS>`, for example `Q_mrtsp_m1_house_p03_laptop_20261201-021500`.

## 0.2 Run folder layout

Every run writes exactly one folder, `logs/<run_id>/`:

```
logs/<run_id>/
  manifest.yaml        # written by the orchestrator (Module 16)
  pose.csv             # recorder (Module 12)
  goals.csv            # recorder
  cycles.csv           # recorder, from the timing topic (Module 9)
  nav2_results.csv     # recorder
  maps/                # recorder: map_<seq>.npz snapshots
  power.csv            # power logger (Module 14), Block E/A only
  procs.csv            # power logger, Block E/A only
  gpu.csv              # power logger, Block E/A only
  stdout/              # one .log file per launched process
  coverage.csv         # coverage evaluator (Module 13), offline
  summary.json         # coverage evaluator + analysis, offline
```

## 0.3 Timestamps

- Every CSV row has **both** `t_wall` (Unix seconds from `time.time()`, float) and, where ROS is involved, `t_ros` (ROS time in seconds, which is sim time when `use_sim_time` is true).
- Power and CPU data only have `t_wall`. The recorder logs both clocks, so the analysis can map between them.
- In Block E, run the recorder **on the Jetson**, the same machine as the power logger, so `t_wall` comes from one clock. Still install `chrony` on both machines and sync the laptop to the Jetson (or both to the same server). Record the offset (`chronyc tracking`) in the manifest.

## 0.4 CSV schemas

| File | Columns |
|------|---------|
| `pose.csv` | `t_wall, t_ros, x, y, yaw` (map frame, metres, radians) |
| `goals.csv` | `t_wall, t_ros, cycle_id, goal_x, goal_y` |
| `cycles.csv` | `cycle_id, t_wall_start, t_wall_end, t_ros_start, t_ros_end, n_frontiers, goal_x, goal_y` |
| `nav2_results.csv` | `t_wall, t_ros, goal_id, status` (status uses `action_msgs/GoalStatus` codes: 4 succeeded, 5 canceled, 6 aborted) |
| `power.csv` | `t_wall, vdd_in_mW, vdd_cpu_gpu_cv_mW, vdd_soc_mW, temp_cpu_C, temp_gpu_C` |
| `procs.csv` | `t_wall, pid, name, utime_s, stime_s` (cumulative, from `/proc/<pid>/stat`) |
| `gpu.csv` | `t_wall, gpu_util_pct, gpu_freq_MHz, ram_used_MB` |
| `coverage.csv` | `t_ros, distance_m, coverage_frac` |

If you need a new column, add it at the end and note it in `docs/log_format.md`. Do not rename existing columns after `prereg-v1`.

## 0.5 Run outcomes

Every run ends with exactly one `outcome` in the manifest. Keep these distinct, because the statistics treat them differently:

| Outcome | Meaning | What happens |
|---------|---------|--------------|
| `complete` | Reached 95% coverage | Counts as data |
| `timeout` | Hit the time limit before 95% | **Counts as data** (a method failure, recorded as censored) |
| `stuck` | Planner reported no frontiers or stopped before 95% | **Counts as data** (a method failure) |
| `invalid` | Infrastructure failure: sim crash, DDS drop, node died, robot spawned in collision | Rerun with the same pose; log it in `docs/run_log.md` |

Never mark a run `invalid` just because the method did badly. That would bias the results toward the method that fails most.

## Changelog

| date | change | who |
|------|--------|-----|

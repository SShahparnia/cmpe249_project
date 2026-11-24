# Baseline provenance (Module 8)

Status: **draft, 2026-11-23.** The repo, commit, config and code-definition parts were filled by reading the pinned source. The nearest and mrtsp result cells come from our own benchmark runs on 2026-11-23 ([benchmark_runs.md](benchmark_runs.md)). The ariadne cells stay empty until Module 7 is run. **Do not fill them with anything but our own runs.**

"Pass?" rule (decided 2026-11-24, Shervan): the published value lies within 2 sd of our 5-run mean, for distance and for time. Time compares our sim time with their wall time, which assumes the publishing machine ran near real time (not stated in the benchmark). For `nearest` the comparison uses the benchmark build + benchmark config (set A in [benchmark_runs.md](benchmark_runs.md)); our `nearest.yaml` changes several parameters and has no published number to compare with.

Gate G1: three methods pass, or two pass and the fallback is chosen and passes its own demo.

| Method | Paper | Repo | Commit | Config file | Published number | Our 5-run result (mean ± sd) | Pass? |
|--------|-------|------|--------|-------------|------------------|------------------------------|-------|
| `nearest` | Yamauchi 1997 (frontier-based exploration); explore_lite lineage | [robo-friends/m-explore-ros2](https://github.com/robo-friends/m-explore-ros2) | `326cf8a0b487c34246bb8f3326afbcd69576dc60` | [configs/methods/nearest.yaml](../configs/methods/nearest.yaml) | Benchmark, package config: Bookstore 50.73 m / 96 s; Warehouse 338.61 m / 587 s ([literature.md](literature.md)) | Bookstore, benchmark build + config: 47.83 ± 5.82 m, 91.4 ± 12.4 s sim (5/5 complete). Our nearest config: 35.73 ± 8.66 m, 172.5 ± 73.7 s sim (4/5 complete, 1 stuck). Warehouse: not run | Bookstore: yes, benchmark build + config (distance and time within 1 sd) |
| `mrtsp` | Liu et al. 2025, Sci. Rep. 15:12261 | [mertgulerx/frontier_exploration_ros2](https://github.com/mertgulerx/frontier_exploration_ros2) | `11864e4ac6789dc81fd71bad51e43339c01deca2` (v1.6.2-5) | [configs/methods/mrtsp.yaml](../configs/methods/mrtsp.yaml) | Benchmark, greedy: Bookstore 36.60 m / 63 s; Warehouse 273.52 m / 499 s | Bookstore, our pinned build, greedy: 33.77 ± 1.73 m, 65.9 ± 4.8 s sim (5/5 complete). Warehouse: not run | Bookstore: yes (distance 1.6 sd, time within 1 sd) |
| `ariadne` | Cao et al. 2023, ICRA; Cao et al. 2024, RA-L | [marmotlab/ARiADNE](https://github.com/marmotlab/ARiADNE) (model, `new_version`) + [marmotlab/ARiADNE-ROS-Planner](https://github.com/marmotlab/ARiADNE-ROS-Planner) (`humble`) | `ea8bc6adb967dff29438dae3df089f4ec9b42920` / `c41b4c949946e9058598fc129fec8271f0489fba` | [configs/methods/ariadne.yaml](../configs/methods/ariadne.yaml) | Paper: TODO (full text). ROS demo: 948 m / 545 s, 0.17 s mean planning time (CMU indoor, i7-12700H) | TODO | TODO |
| `roadmap` (fallback) | TODO | [suchetanrs/roadmap-explorer](https://github.com/suchetanrs/roadmap-explorer) | `203ff2602e1a1e41eec7ef663eacc21266e270c6` | not written (only if the fallback is triggered) | Benchmark: Bookstore 39.28 m / 72 s; Warehouse 286.28 m / 641 s | n/a | n/a |

Patches applied to every method: `methods/patches/<submodule>-timing.patch` (Module 9, timing only, see [planner_timing.md](planner_timing.md)).

---

## nearest: m-explore-ros2 (Module 5)

Paths relative to `methods/m-explore-ros2/explore/`, line numbers at the pinned commit (without our patch).

**Frontier detection.** `src/frontier_search.cpp:26-96` `searchFrom`: BFS over the costmap from the robot's cell (4-connected, `:68`). A frontier cell is `NO_INFORMATION` with a 4-connected `FREE_SPACE` neighbour (`:173-190`). Clusters grow over the 8-neighbourhood (`:129`). Clusters smaller than `min_frontier_size` metres are dropped (`:79-80`, size in cells × resolution).

**Cost (step 2).** `src/frontier_search.cpp:192-197`:

```
cost = potential_scale * min_distance * resolution  -  gain_scale * size * resolution
```

Frontiers are sorted by ascending cost (`:91-93`). The first non-blacklisted one wins (`src/explore.cpp:255-259`).

- **Distance is straight-line**, not a path cost: `min_distance` is the Euclidean distance from the robot's map cell to the **closest cell** of the frontier (`src/frontier_search.cpp:151-159`, reference = robot cell passed in at `:78`).
- **Units quirk:** `min_distance` is already in metres, and the code multiplies it by `resolution` again (`:194-195`). With `gain_scale = 0` this only rescales every cost by the same factor, so the ordering is unchanged.
- **The goal is the centroid, not the closest cell.** The selected frontier's `centroid` is sent to Nav2 (`src/explore.cpp:268`, `:304`), while it was ranked by its closest cell. For a long or curved frontier the robot may be sent somewhere farther than "nearest" suggests. Report this as part of the definition.
- `orientation_scale` is declared and read (`src/explore.cpp:69, 78`) but never passed to `FrontierSearch` (`:89-91`), so it has no effect.

**Making it nearest (step 3).** `gain_scale: 0.0`, so cost = `potential_scale * min_distance * res` and the lowest straight-line distance to the closest frontier cell wins. Saved in [configs/methods/nearest.yaml](../configs/methods/nearest.yaml).

**Replanning and commitment (step 4).**
- `planner_frequency` drives `makePlan` through a **wall-clock** timer (`create_wall_timer`, `src/explore.cpp:132-134`). Under sim time with a real-time factor below 1, it replans more often per simulated second than the parameter suggests. Note this for Module 15.
- Every cycle recomputes and re-sorts all frontiers. If the best frontier is the same goal (within 1 cm, `same_point`, `:43-50`) and a goal is active, nothing is sent (`:296-298`).
- **Progress timeout:** if the goal is the same and the distance has not dropped for `progress_timeout` s, the goal is blacklisted and replanning runs immediately (`:270-288`).
- **Blacklist on abort:** a Nav2 ABORTED result blacklists the frontier (`:390-404`; on Humble, unconditionally). The blacklist tolerance is 5 cells in x and y (`:359-374`).
- After SUCCEEDED, it replans immediately (`:385-389`, `:426`).
- It stops when no frontiers are left or all are blacklisted (`:240-267`). `return_to_init` is set to false in our config.

**Benchmark (step 5).** Done on Bookstore, 2026-11-23 ([benchmark_runs.md](benchmark_runs.md)). Benchmark build + config: 47.83 ± 5.82 m, 91.4 ± 12.4 s sim, published 50.73 m / 96 s. Our pinned build + `nearest.yaml`: 35.73 ± 8.66 m, 172.5 ± 73.7 s sim, 4/5 complete (run 1 stuck after 1,599 cycles without moving). Our pinned build with its package default config gives 47.60 ± 8.54 m, the same as the benchmarked copy. `nearest.yaml` changes `planner_frequency` and `min_frontier_size` as well as `gain_scale`. Warehouse not run yet.

---

## mrtsp: frontier_exploration_ros2 (Module 6)

Full paper-to-code mapping: [mrtsp_mapping.md](mrtsp_mapping.md).

- **Solver:** `greedy` in our config. The package default `dp` (bounded-horizon DP) is the package author's extension, not part of Liu et al. The published benchmark numbers are greedy. **TEAM DECISION:** confirm greedy.
- **Planning loop for Module 9:** starts at the frontier snapshot in `try_send_next_goal` (`src/frontier_explorer_core_dispatch.cpp:74`) and ends before `send_frontier_goal` (`:173`). Preemption (`consider_preempt_active_goal`, `:399`) can also reselect goals and is **not** timed yet (see [planner_timing.md](planner_timing.md)).
- **Commitment logic kept as is:** visible-reveal-gain preemption (on), blocked-goal skip (on), near-goal completion at 0.5 m, suppression off, post-goal settle off (package defaults).
- **Deviations from the paper:** TODO, from the mapping table once the paper column is filled.

---

## ariadne: ARiADNE-ROS-Planner (Module 7)

Hours and step results are tracked in [ariadne_log.md](ariadne_log.md). Facts found by reading the code (2026-11-23):

- **ROS version (Step C.1): ROS 2 Humble.** The `humble` branch is an `ament_python` package `rl_planner` (`src/rl_planner/package.xml`, `setup.py`). No ROS 1 port or `ros1_bridge` is needed.
- **Inputs (Step C.2):** `/projected_map` (`nav_msgs/OccupancyGrid`, BEST_EFFORT) and `/state_estimation` (`nav_msgs/Odometry`, robot position in the map frame) (`rl_planner/rl_planner.py:108-119`). The authors produce the map with `octomap_server` from a point cloud (`sensor_scan`) at 0.4 m, and the pose from CMU's SLAM.
- **Output:** `/way_point` (`geometry_msgs/PointStamped`, frame `map`), meant for CMU's waypoint follower, not Nav2 (`:121`, `:302-308`). Also `/runtime` (Float32 planning time, `:122`).
- **Planning loop:** a timer at `replanning_frequency` Hz calls `run()` (`:129`, `:137`). Graph update and frontier extraction run in `update_planning_state` (`:200`, `agent.py:115-134`), then network inference (`:210-230`), then the waypoint is published (`:251`).
- **Checkpoint:** `src/rl_planner/rl_planner/model/checkpoint.pth` (51 MB) ships with the ROS planner. Inference runs on CPU (`self.device = 'cpu'`, `:32`).
- **Training vs. deployment parameters:** training (`methods/ARiADNE/parameter.py`) uses CELL_SIZE 0.4 m, NODE_RESOLUTION 4.0 m, SENSOR_RANGE 16 m, UTILITY_RANGE 0.8 × range, MIN_UTILITY 2. The authors' ROS launch already uses 0.4 m / 2.0 m / 20 m / 0.5 × range / 3. Our TB4 lidar range is about 12 m (VERIFY), and slam_toolbox gives 0.05 m.
- **Jetson blocker found:** the authors' launch sets `LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6`. That path does not exist on aarch64. Our launch (`harness/ariadne_adapter/launch/ariadne.launch.py`) drops it.

**Adapter (Step C.3):** `harness/ariadne_adapter`. It only converts interfaces:

| ARiADNE needs | We have | Adapter |
|---|---|---|
| `/projected_map` | slam_toolbox `/map` | republishes it, optionally downsampled (`projected_map_resolution`) |
| `/state_estimation` (Odometry) | TF `map -> base_link` | TF lookup at 10 Hz, published as Odometry |
| publishes `/way_point` | Nav2 `navigate_to_pose` | sends each new waypoint as a goal; identical repeats are not re-sent; after a non-success result the same waypoint may be sent again |

**Decision needed: map resolution fed to ARiADNE.** The default in `configs/methods/ariadne.yaml` is `projected_map_resolution: 0.4` (coarse cell = occupied if any fine cell is occupied, else free if any is free, else unknown). This matches what the network was trained on and what the authors feed it, but it is a change to the input. The alternative, `0.0`, feeds the raw 0.05 m map with `map_resolution: 0.05`, which is 64× more cells than the network was designed for. Record the choice, date and reason here: TODO.

**Fallback decision (end of Week 4).** TODO: date, decision (ARiADNE / roadmap), reason.

---

## Deviations and patches

| Method | Patch | What it changes | Logic changed? |
|--------|-------|-----------------|----------------|
| nearest | `methods/patches/m-explore-ros2-timing.patch` | publishes `PlanningCycle` | no (review pending) |
| mrtsp | `methods/patches/frontier_exploration_ros2-timing.patch` | publishes `PlanningCycle` | no (review pending) |
| ariadne | `methods/patches/ARiADNE-ROS-Planner-timing.patch` | publishes `PlanningCycle` | no (review pending) |
| ariadne | `harness/ariadne_adapter` (not a patch) | interface conversion, optional map downsampling | input resolution, see the decision above |

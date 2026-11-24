# Common experiment rules (Module 15)

Every run uses `configs/common.yaml`. The orchestrator (`harness/run.py`, Module 16) reads it and passes the values to each method, so the methods differ only in how they choose the next goal. Frozen after `prereg-v1`.

## Rules

1. **Same for every method:** SLAM parameters (`configs/slam_toolbox.yaml`, async node), Nav2 parameters (`configs/nav2.yaml`), robot (TB4 standard), sensors (below), `timeout_sim_s`, `coverage_target`, map-snapshot period, start poses (`configs/start_poses/`).
2. **Replanning rate:** where a method has a rate parameter, it is set to `1 / replan_period_s` (1.0 Hz until the pilot fixes `replan_period_s`).
3. **Goal commitment is the method's own.** Blacklisting, progress timeouts, preemption and "replan only when the goal is done" are kept and documented below, never overridden. This is what "the method" means in our results.
4. **Smallest frontier:** `min_frontier_size_m = 0.3` wherever a method has the parameter.
5. **Robot start:** the TB4 sim spawns the robot docked, facing its charging dock 0.16 m ahead. The Create 3 undock behaviour does not survive a slow sim: in a WSL test run (2026-11-23, RTF ~0.1) it was cancelled and then left the base ignoring Nav2's velocity commands. So, after Nav2 is up, the orchestrator removes the dock model from the world (`ign service /world/<map>/remove`) and sets the Create 3's `safety_override` to `backup_only` (`robot_safety_override` in `configs/common.yaml`), which lifts the base's cap on driving backwards so Nav2's backup recovery works; it also drops the cliff check while reversing, and none of our worlds has a drop-off. Same for every method. Distance and time are counted from the planner start.
6. **Per-method parameters** live in `configs/methods/<method>.yaml`, listing only values that differ from the package's own config file. Every such value is in the table below with its reason.

## Sensor setup (TEAM DECISION before `prereg-v1`)

The TB4's sensors are rendered by Gazebo. Without a GPU in the sim (WSL, Mac container) the depth camera crashes or stalls the sim, and stock sensor rates give a low real-time factor (`docs/sim_setup.md`). Whatever is chosen is applied on **every** sim machine with `scripts/sim/tune_sensor_rates.sh`, written into `configs/common.yaml` (`sensor_setup`), and recorded in each manifest.

| Option | Camera | Lidar | Hazard sensors | Changes SLAM input? | Measured RTF |
|--------|--------|-------|----------------|---------------------|--------------|
| A (current default) | removed | 62 Hz (stock) | 62 Hz (stock) | no (SLAM and Nav2 do not use the camera) | Joey, WSL2: 0.22 |
| B | removed | 62 Hz | 5 Hz | no (hazards are not used by SLAM or Nav2) | TODO |
| C (Shervan's Mac) | removed | 10 Hz | 5 Hz | yes (closer to the real RPLIDAR A1, 5.5 to 10 Hz) | Mac: 0.92 to 1.0 |

## Per-method table

| | `nearest` | `mrtsp` | `ariadne` |
|---|---|---|---|
| Package / node | m-explore-ros2 `explore_lite` / `explore_node` | frontier_exploration_ros2 / `frontier_explorer` | ARiADNE-ROS-Planner `rl_planner` + `harness/ariadne_adapter` |
| Replanning parameter | `planner_frequency` = 1.0 Hz | `map_processing_rate_hz` = 1.0 Hz | `replanning_frequency` = 1.0 Hz |
| Clock of that timer | **wall clock** (`create_wall_timer`) | ROS timer (sim time) | ROS timer (sim time) |
| When it replans | every period, also while a goal is active | when no goal is active, plus visible-reveal-gain preemption while one is | every period |
| Goal commitment (kept) | same goal (within 1 cm) is not re-sent; goal blacklisted after `progress_timeout` 30 s without progress or on Nav2 ABORTED (5-cell tolerance); immediate replan on SUCCEEDED | goal done at 0.5 m (near-goal completion); blocked goals skipped; preemption only if the new frontier reveals more | waypoint reached within `waypoint_threshold` 2.0 m; adapter does not re-send an identical waypoint, re-sends after a non-success result |
| Stops when | no frontiers left, or all blacklisted | no frontiers left | planner reports exploration complete |
| Frontier size filter | `min_frontier_size` 0.3 m | `min_frontier_size_cells` 6 (= 0.3 m at 0.05 m) | n/a (graph-based utility, `min_utility` 3) |

### Parameters that differ from the package defaults

Package values: `methods/m-explore-ros2/explore/config/params.yaml`, `methods/frontier_exploration_ros2/config/params.yaml`, `methods/ARiADNE-ROS-Planner/src/rl_planner/launch/rl_planner.launch.py`. Values equal to the package's are not listed (for example mrtsp's `map_processing_rate_hz` is already 1.0).

| Method | Parameter | Package value | Ours | Reason |
|--------|-----------|---------------|------|--------|
| nearest | `gain_scale` | 1.0 | 0.0 | makes it nearest-frontier (Module 5) |
| nearest | `planner_frequency` | 0.15 | 1.0 | rule 2 |
| nearest | `visualize` | true | false | RViz markers only; saves CPU |
| nearest | `min_frontier_size` | 0.75 | 0.3 | rule 4 |
| nearest | `return_to_init` | true | false | a run ends at frontier exhaustion, not at the start pose |
| mrtsp | `mrtsp_solver` | dp | greedy | the paper's MRTSP traversal; `dp` is the package author's extension. TEAM DECISION in `docs/baseline_provenance.md` |
| mrtsp | `min_frontier_size_cells` | 5 | 6 | rule 4 |
| mrtsp | `robot_base_frame` | base_footprint | base_link | TB4 TF tree: the recorder, nearest and ariadne all use base_link. VERIFY that the TB4 sim publishes it (it does in the URDF) |
| ariadne | `replanning_frequency` | 2.5 (authors' launch) | 1.0 | rule 2 |
| ariadne | `sensor_range` | 20.0 | 12.0 | TB4 RPLIDAR range (`r_max` 12 m in the TB4 URDF) |
| ariadne | `publish_graph` | true | false | RViz only; saves CPU on the Jetson |
| ariadne | `projected_map_resolution` (adapter) | n/a | 0.4 | grid the network was trained on. TEAM DECISION in `docs/baseline_provenance.md` |

## Known threat: wall-clock replanning in `nearest`

explore_lite's planning timer runs on the wall clock. At a real-time factor below 1 it replans more often per simulated second than the other two methods, which use sim-time timers. Mitigations: run the sim at an RTF the machine can hold close to 1 (sensor setup above), record the measured RTF in every manifest (orchestrator), and report decision latency in wall time (Module 9). The threat goes in the report's threats-to-validity section.

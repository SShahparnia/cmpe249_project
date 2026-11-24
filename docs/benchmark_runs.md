# Benchmark runs: nearest and MRTSP (Modules 5 and 6, step "benchmark run")

Run 2026-11-23 by Aaron on a Windows 11 laptop in WSL2 (Ubuntu 24.04, 10 cores, 12 GB), software rendering, no GPU. Raw results: [results/benchmark_bookstore_2026-11-23.csv](results/benchmark_bookstore_2026-11-23.csv). Per-run logs and `cycles.csv` stay on that machine (`~/bench_runs/`, not in git).

## Setup

- **Benchmark:** [mertgulerx/autonomous-exploration-demo-benchmark](https://github.com/mertgulerx/autonomous-exploration-demo-benchmark) @ `9dbf7bd`. ROS 2 Jazzy, Gazebo Harmonic 8.15, MOGI bot with a 10 m lidar, Bookstore world (225 m²), the benchmark's spawn pose.
- **Install:** [scripts/bench/setup_jazzy_bench.sh](../scripts/bench/setup_jazzy_bench.sh). **Runner:** [scripts/bench/run_bench.py](../scripts/bench/run_bench.py).
- **Our planners:** pinned submodules with `methods/patches/*-timing.patch`, built in an overlay workspace on top of the benchmark. Both patches compile on Jazzy.
- 5 runs per set, sets interleaved (A, B, C, A, B, C, ...). Fresh simulator for every run.

| Set | Planner build | Config |
|-----|---------------|--------|
| A | benchmark's vendored `m-explore-ros2` | benchmark config (`gain_scale 1.0`, `planner_frequency 0.15`, `min_frontier_size 0.75`) |
| B | our pinned `m-explore-ros2` @ 326cf8a | [configs/methods/nearest.yaml](../configs/methods/nearest.yaml) (`gain_scale 0`, `planner_frequency 1.0`, `min_frontier_size 0.3`) |
| C | our pinned `frontier_exploration_ros2` @ 11864e4 | benchmark `config.yaml`, then [configs/methods/mrtsp.yaml](../configs/methods/mrtsp.yaml) (`mrtsp_solver greedy`, `min_frontier_size_cells 6`), `robot_base_frame base_footprint` |
| A0 | our pinned `m-explore-ros2` @ 326cf8a | its package default `params.yaml` (`gain_scale 1.0`, `planner_frequency 0.15`, `return_to_init true`) |

**A0 is set A's first attempt, run by mistake.** The runner's child processes inherited our overlay, so "set A" started our pinned build with its package config. This was found from the logs (`Getting initial pose`, planning-cycle output) and fixed: processes for set A now start from a clean environment, and the rerun was checked to use `build/explore_lite/explore` from the benchmark workspace. A0 is kept as extra data, labelled as what it is.

### Metric definitions (same as the benchmark's RViz panel)

- **Distance:** sum of `/odom` xy steps from explorer start; single steps > 2.0 m are ignored.
- **Time:** the benchmark publishes wall-clock time. WSL ran the sim at 0.49 to 0.67× real time, so wall times here are inflated. **Compare sim time against the published times**, which assumes the publishing machine ran near real time (not stated in the benchmark).
- **End of run:** the explorer's own completion signal (`explore/status = exploration_complete`, or `/exploration_complete`). The benchmark instead ends when the operator clicks End. A run is `stuck` if the robot does not move for 180 s sim time, `timeout` after 1200 s sim time.
- **Known area:** cells != -1 in the final `/map` × resolution². Every completed run reached 178.0 to 178.6 m², so complete runs covered the same map.

### Differences from the benchmark procedure

No RViz, no `frontier_path_tracker` (display only). A fresh simulator per run instead of Map Reset / Return Start. Automatic end of run (above). These are recorded here so the comparison is not over-claimed.

## Results (Bookstore, mean ± sd over 5 runs)

| Set | Outcomes | Distance (m) | Sim time (s) | Wall time (s) | Cycle latency median / p95 (ms) | Published (1 run each, wall time) |
|-----|----------|--------------|--------------|---------------|---------------------------------|-----------------------------------|
| A: benchmark m-explore | 5 complete | **47.83 ± 5.82** | **91.4 ± 12.4** | 169.0 ± 22.8 | n/a (unpatched) | m_explore_ros2: 50.73 m / 96 s |
| B: our nearest | 4 complete, 1 stuck | 35.73 ± 8.66 (complete only: 38.38 ± 7.28) | 172.5 ± 73.7 | 275.3 ± 106.4 | 60.9 / 83.6 (means over runs) | none |
| C: our MRTSP greedy | 5 complete | **33.77 ± 1.73** | **65.9 ± 4.8** | 119.9 ± 9.8 | 112.0 / 173.5 | greedy MRTSP: 36.60 m / 63 s |
| A0: our m-explore, default config | 5 complete | 47.60 ± 8.54 | 107.9 ± 47.5 | 187.4 ± 80.7 | 56.6 / 82.0 | (compare with m_explore_ros2 above) |

Per-run values are in the CSV. One extra C run was `invalid` (Nav2 / `/map` not ready within 120 s) and was rerun with the same settings, as Section 0.5 allows.

## Reading

- **m-explore-ros2 reproduces.** The published 50.73 m / 96 s lies within one sd of set A (47.83 ± 5.82 m, 91.4 ± 12.4 s sim). A0 (our pinned build, same kind of config) gives the same distance (47.60 m), so our pinned version behaves like the benchmarked one.
- **MRTSP greedy reproduces.** Published 36.60 m / 63 s against our 33.77 ± 1.73 m / 65.9 ± 4.8 s sim: time within one sd, distance about 1.6 sd shorter than published.
- **MRTSP is shorter than nearest (Module 6 sanity check), and far more consistent.** 33.77 ± 1.73 m vs 35.73 ± 8.66 m (or 38.38 ± 7.28 m over the 4 complete nearest runs), and 66 vs 173 s sim. With 5 unpaired runs this is a sanity check, not a test.
- **Nearest run 1 got stuck.** No motion for 180 s at 167 m² known (about 94% of the final area), with 1,599 planning cycles: replanning without moving. It counts as a method failure, not a rerun (Section 0.5).
- **Set B is not a one-parameter change from set A.** `nearest.yaml` also changes `planner_frequency` (0.15 → 1.0) and `min_frontier_size` (0.75 → 0.3), on top of `gain_scale` (1 → 0). Keep that in mind before attributing the A vs B difference to the cost function.
- **Planning latency (laptop, not Jetson):** MRTSP cycles take about twice as long as nearest's (112 vs 61 ms median). Not a Block E result.

## Reproduce

On Ubuntu 24.04 (a machine or a WSL2 distro):

```bash
bash scripts/bench/setup_jazzy_bench.sh            # installs (sudo), clones benchmark @ 9dbf7bd and this repo
cd ~/bench/autonomous-exploration-demo-benchmark && source /opt/ros/jazzy/setup.bash && colcon build --symlink-install
cd ~/cmpe249_project && bash scripts/apply_patches.sh
mkdir -p ~/eb_ws/src && cd ~/eb_ws/src
ln -s ~/cmpe249_project/harness/exploration_msgs .
ln -s ~/cmpe249_project/methods/m-explore-ros2/explore .
ln -s ~/cmpe249_project/methods/m-explore-ros2/explore_lite_msgs .
ln -s ~/cmpe249_project/methods/frontier_exploration_ros2 .
cd ~/eb_ws && source ~/bench/autonomous-exploration-demo-benchmark/install/setup.bash && \
  colcon build --symlink-install --allow-overriding explore_lite explore_lite_msgs frontier_exploration_ros2
source ~/eb_ws/install/setup.bash
python3 ~/cmpe249_project/scripts/bench/run_bench.py --sets A B C --reps 5 --world bookstore
```

Results go to `~/bench_runs/results.csv`. About 3 to 6 minutes per run at real-time factor 0.5 to 0.65.

## Not covered here

- ARiADNE: not part of the benchmark; Module 7 Steps A to C happen in our Humble stack.
- Warehouse world (1,500 m²): not run yet.
- The benchmark's own `frontier_exploration_ros2` copy with its config (solver `dp`, set D in the runner): not run. Note that the benchmark's shipped MRTSP config uses `mrtsp_solver: dp` even though its results table labels the row "greedy mrtsp".

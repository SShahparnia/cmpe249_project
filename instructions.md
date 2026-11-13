# instructions.md: How to Build Each Module

This file turns [plan.md](plan.md) into step-by-step build instructions. `plan.md` says **what** happens and **when**; this file says **how** to build each piece and **how to know it is done**.

How to use it:

- Modules are numbered in build order. Each one lists its owner, its plan phase, what it depends on, the steps, the files it produces, and a **Done when** checklist.
- Do not start a module until the modules it depends on are done.
- Section 0 defines the shared conventions (folder layout, names, log formats). Every module must follow them, because the analysis script reads every run the same way.
- Anything marked **VERIFY** is something we have not confirmed yet. Check it against the real repo, board or paper and write down the answer.

---

## Module index

| # | Module | Owner | Phase | Depends on |
|---|--------|-------|-------|------------|
| 0 | Shared conventions (read first) | all | 0 | none |
| 1 | Repository skeleton | Shervan | 0 | 0 |
| 2 | Jetson setup and power-mode inventory | Shervan, Aaron | 0 | 1 |
| 3 | Laptop simulation stack and Jetson link | Shervan, Joey | 0 | 1 |
| 4 | Literature notes | Aaron (all) | 0 | none |
| 5 | Baseline A: nearest frontier (m-explore-ros2) | Joey | 1 | 3 |
| 6 | Baseline B: MRTSP (frontier_exploration_ros2) | Aaron | 1 | 3, 5 |
| 7 | Baseline C: ARiADNE (or roadmap-explorer fallback) | Joey, Aaron | 1 | 2, 3 |
| 8 | Baseline provenance table | all | 1 | 5, 6, 7 |
| 9 | Planner timing instrumentation | Aaron | 2 | 5, 6, 7 |
| 10 | Map set and ground-truth grids | Joey | 2 | 3 |
| 11 | Start-pose generator | Joey | 2 | 10 |
| 12 | Recorder node | Shervan | 2 | 9 |
| 13 | Coverage evaluator | Shervan | 2 | 10, 12 |
| 14 | Power and compute logger | Aaron | 2 | 2 |
| 15 | Common experiment config | all | 2 | 5, 6, 7 |
| 16 | Run orchestrator | Shervan | 2 | 11 to 15 |
| 17 | Pilot and sample-size decision | all | 3 | 16 |
| 18 | Pre-registration | all | 3 | 17 |
| 19 | Batch runner (Blocks Q and E) | Shervan | 4 | 16, 18 |
| 20 | Analysis pipeline | Aaron (all) | 5 | 19 |
| 21 | Scaling ablation (optional) | Aaron, Shervan | 6 | 19, 20 |
| 22 | Write-up, video, slides | all | 7 | 20 |

---

## Module 0: Shared conventions (read before writing any code)

### 0.1 Names

Use exactly these strings everywhere: file names, CLI flags, CSV columns and plot labels.

| Thing | Allowed values |
|-------|----------------|
| `method` | `nearest`, `mrtsp`, `ariadne` (or `roadmap` if the fallback is triggered) |
| `map` | `m1_house`, `m2_office`, `m3_warehouse` (and `m2_office_2x` for the ablation) |
| `block` | `P` (pilot), `Q` (quality), `E` (energy), `A` (ablation) |
| `power_mode` | `laptop` for Blocks P and Q; for Block E, the nvpmodel name as listed in `docs/power_modes.md` (e.g. `7W`, `MAXN_SUPER`) |
| `pose` | integer index into `configs/start_poses/<map>.yaml`, starting at 0 |

Run ID format: `<block>_<method>_<map>_p<pose>_<power_mode>_<YYYYmmdd-HHMMSS>`, for example `Q_mrtsp_m1_house_p03_laptop_20261201-021500`.

### 0.2 Run folder layout

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

### 0.3 Timestamps

- Every CSV row has **both** `t_wall` (Unix seconds from `time.time()`, float) and, where ROS is involved, `t_ros` (ROS time in seconds, which is sim time when `use_sim_time` is true).
- Power and CPU data only have `t_wall`. The recorder logs both clocks, so the analysis can map between them.
- In Block E, run the recorder **on the Jetson**, the same machine as the power logger, so `t_wall` comes from one clock. Still install `chrony` on both machines and sync the laptop to the Jetson (or both to the same server). Record the offset (`chronyc tracking`) in the manifest.

### 0.4 CSV schemas

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

### 0.5 Run outcomes

Every run ends with exactly one `outcome` in the manifest. Keep these distinct, because the statistics treat them differently:

| Outcome | Meaning | What happens |
|---------|---------|--------------|
| `complete` | Reached 95% coverage | Counts as data |
| `timeout` | Hit the time limit before 95% | **Counts as data** (a method failure, recorded as censored) |
| `stuck` | Planner reported no frontiers or stopped before 95% | **Counts as data** (a method failure) |
| `invalid` | Infrastructure failure: sim crash, DDS drop, node died, robot spawned in collision | Rerun with the same pose; log it in `docs/run_log.md` |

Never mark a run `invalid` just because the method did badly. That would bias the results toward the method that fails most.

---

## Module 1: Repository skeleton

**Owner:** Shervan · **Phase:** 0 · **Depends on:** Module 0

### Steps

1. Create the repo and the folder structure:
   ```
   project/
     methods/          # git submodules, one per planner
       patches/        # our .patch files applied on top of submodules
     harness/          # ROS 2 package(s): recorder, timing msgs, launch files
     maps/             # world files + ground-truth grids
     configs/          # start poses, per-method params, common params
     analysis/         # offline scripts
     scripts/          # setup helpers, batch runners
     docs/
     logs/             # git-ignored
     README.md
   ```
2. Add `.gitignore` with at least: `logs/`, `build/`, `install/`, `log/`, `__pycache__/`, `*.pyc`, `.venv/`.
3. Add each third-party repo as a submodule pinned to one commit:
   ```bash
   git submodule add <repo-url> methods/<name>
   cd methods/<name> && git checkout <commit> && cd ../..
   git add methods/<name> && git commit -m "Pin <name> at <short-sha>"
   ```
   Do this for m-explore-ros2, frontier_exploration_ros2, ARiADNE, ARiADNE-ROS-Planner, and (in case of fallback) roadmap-explorer.
4. Never edit code inside a submodule directly. Any change goes in `methods/patches/<name>-<what>.patch` and gets applied by a setup script (`scripts/apply_patches.sh`). This keeps "the original method" and "our changes" separate, which the provenance table needs.
5. Create `docs/log_format.md` with the contents of Section 0, and `docs/run_log.md` (empty, with a header row: `date | who | run_id | what happened | action`).
6. Write a README with a "Quick start" section. It will grow as Modules 2 and 3 finish.

### Done when

- [ ] Fresh `git clone --recursive` gives all submodules at the pinned commits.
- [ ] `scripts/apply_patches.sh` runs cleanly on a fresh clone (even if there are no patches yet).

---

## Module 2: Jetson setup and power-mode inventory

**Owners:** Shervan, Aaron · **Phase:** 0 · **Depends on:** Module 1

### Steps

1. **Flash.** Flash JetPack 6.x with the Super configuration using SDK Manager or the SD-card image. In `docs/jetson_setup.md`, write down: JetPack version, L4T version (`cat /etc/nv_tegra_release`), the image file name or SDK Manager version, the storage device, and every command you ran.
2. **Power modes.**
   ```bash
   sudo nvpmodel -q            # current mode
   cat /etc/nvpmodel.conf      # every mode defined on this board, with IDs
   ```
   Write `docs/power_modes.md` with a table: mode ID, name, CPU cores online, max CPU freq, max GPU freq. **VERIFY** which modes exist; do not assume the IDs. Pick the lowest-power and highest-power mode for Block E. To switch: `sudo nvpmodel -m <id>` (some changes need a reboot; note which).
3. **Clocks and fan.** Do **not** run `jetson_clocks` during experiments. It locks clocks at maximum and overrides what the power mode is supposed to do. Pick one fan profile, set it the same way for every block, and write it down.
4. **ROS 2.** Install ROS 2 Humble (JetPack 6 is Ubuntu 22.04, so Humble debs work). Then install `ros-humble-slam-toolbox` and `ros-humble-navigation2` / `ros-humble-nav2-bringup`.
5. **jtop.** `sudo pip3 install -U jetson-stats`, then reboot. Check that `jtop` opens and shows power rails.
6. **Power-logging smoke test.** Write `scripts/power_smoke_test.py`:
   ```python
   import csv, time, sys
   from jtop import jtop

   out = sys.argv[1]
   with jtop(interval=0.1) as jetson, open(out, "w", newline="") as f:
       w = csv.writer(f)
       w.writerow(["t_wall", "vdd_in_mW"])
       while jetson.ok():
           rails = jetson.power["rail"]
           w.writerow([time.time(), rails["VDD_IN"]["power"]])
   ```
   **VERIFY** the rail names (`print(jetson.power)` once). On Orin Nano they are typically `VDD_IN`, `VDD_CPU_GPU_CV`, `VDD_SOC`.
7. Log 5 minutes idle, then 5 minutes under `stress-ng --cpu 6 --timeout 300` (install `stress-ng`). Plot both. The stress trace must sit clearly above idle. Save the plot as `docs/figures/power_smoke.png`.
8. **Check the real sample rate.** jtop's `interval` is a request, not a guarantee. Compute the median gap between rows. If it is much slower than 10 Hz, write that down; Module 14 might need to read the INA3221 sysfs nodes directly instead (`/sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*/`; **VERIFY** the path on our board).

### Done when

- [ ] `docs/jetson_setup.md` lets someone re-flash the board without asking questions.
- [ ] `docs/power_modes.md` lists the real modes and the two chosen for Block E.
- [ ] `power_smoke.png` shows idle vs. stress clearly separated, and the achieved sample rate is recorded.

---

## Module 3: Laptop simulation stack and Jetson link

**Owners:** Shervan, Joey · **Phase:** 0 · **Depends on:** Module 1

### Steps

1. **Laptop install:** Ubuntu 22.04, ROS 2 Humble, TurtleBot 4 simulator (`ros-humble-turtlebot4-simulator`), `slam_toolbox`, Nav2.
2. **Note the simulator.** On Humble the TurtleBot 4 sim runs on **Ignition Gazebo (Fortress)**, not Gazebo Classic. The AWS RoboMaker worlds (Module 10) were written for Gazebo Classic. Check this now so it does not surprise you in Week 4. Options, in order of preference: (a) find a maintained Ignition/Fortress port of the AWS worlds, (b) convert the SDF and models yourself, (c) use the TurtleBot 4 model in Gazebo Classic. Record the choice in `docs/sim_setup.md`.
3. **Launch the full stack once** in the default TB4 world: simulation + `slam_toolbox` (online async) + Nav2, then drive with teleop and watch the map grow in RViz.
4. **Measure the real-time factor** while SLAM and Nav2 are running. Write it down. If it is well below 1.0, every run takes longer in wall time than in sim time; plan the batch schedule in Module 19 around this.
5. **Jetson link.**
   - Connect laptop and Jetson by Ethernet with static IPs.
   - Set the same `ROS_DOMAIN_ID` on both, and the same RMW (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` is a good default; install it on both).
   - From the Jetson: `ros2 topic list` shows the sim topics; `ros2 topic hz /scan` and `ros2 topic echo /clock --once` work.
   - From the laptop: a test topic published on the Jetson is visible.
   - Launch `slam_toolbox` **on the Jetson** with `use_sim_time:=true` against the laptop's sim, and confirm the map builds. This is the Block E layout: **Gazebo on the laptop; SLAM, Nav2, planner, recorder and power logger on the Jetson.**
6. Put the exact launch commands in the README under "Quick start".

### Done when

- [ ] Every team member has launched sim + SLAM + Nav2 from the README on their own laptop (this is Gate G0 together with Module 2).
- [ ] SLAM on the Jetson builds a map from laptop sim data with `use_sim_time` working.
- [ ] Real-time factor recorded in `docs/sim_setup.md`.

---

## Module 4: Literature notes

**Owner:** Aaron leads, all read · **Phase:** 0 · **Depends on:** none

### Steps

1. Create `docs/literature.md` with one row per surveyed work (the 11 works in proposal Section 2):

   | Paper | Claim | Environments | # runs | How compute is reported | Hardware | Code link | Notes |
   |-------|-------|--------------|--------|-------------------------|----------|-----------|-------|

2. Split the 11 papers among the three of you. Read the full paper, not the abstract.
3. For each paper, answer explicitly in Notes: "Does it report exploration quality **and** compute or energy on an embedded board, as a trade-off?" If any paper says yes, post it to the team the same day. It changes the novelty argument.
4. For the three baseline papers (nearest frontier / explore_lite lineage, Liu et al. 2025 MRTSP, ARiADNE), also record the exact numbers you will try to reproduce in Module 5 to 7: metric name, environment, mean, spread, number of runs.

### Done when

- [ ] All 11 rows filled.
- [ ] Gap statement re-checked and either confirmed or rewritten.
- [ ] Target numbers for each baseline written down before reproduction starts.

---

## Module 5: Baseline A, nearest frontier (m-explore-ros2)

**Owner:** Joey · **Phase:** 1 · **Depends on:** Module 3

### Steps

1. Build the pinned submodule in a colcon workspace and run the package's own TurtleBot demo to completion.
2. **Read the cost function.** In `explore_lite`, find where frontiers are scored (look in the frontier search and the explore node's planning loop). Write down:
   - Which parameters weight distance vs. frontier size (expected: `potential_scale`, `gain_scale`, `orientation_scale`; **VERIFY**).
   - How "distance" is computed: straight-line distance from the robot to the nearest frontier cell, or a path cost. **VERIFY** in the code and quote the line numbers in `docs/baseline_provenance.md`.
3. **Make it nearest-frontier.** Set the information/size term to zero (expected `gain_scale: 0.0`) and keep the distance term. Save as `configs/methods/nearest.yaml`.
4. Note the method's own replanning rate (`planner_frequency`) and progress/blacklist logic (`progress_timeout`, blacklisting of unreachable frontiers). You will need these for Module 15.
5. **Benchmark run.** Run the `autonomous-exploration-demo-benchmark` environment 5 times with this config. Record the metrics the benchmark publishes, using its definitions, and compare with its published numbers.
6. Write everything into the `nearest` row of the provenance table (Module 8).

### Done when

- [ ] Demo completes.
- [ ] Definition of "nearest" written down with code references.
- [ ] 5 benchmark runs done, mean and spread compared with the published value.

---

## Module 6: Baseline B, MRTSP (frontier_exploration_ros2)

**Owner:** Aaron · **Phase:** 1 · **Depends on:** Modules 3, 5

### Steps

1. Build the pinned submodule. Run its MRTSP mode in the same benchmark environment as Module 5, 5 times.
2. **Map paper to code.** Read Liu et al. (Sci. Rep. 2025). Make a table in `docs/mrtsp_mapping.md`:

   | Paper term (symbol, equation number) | Meaning | Code location | Parameter name | Value we use | Same / different |
   |---|---|---|---|---|---|

   Pay attention to: how the TSP is solved (exact, heuristic, greedy), which distance is used (Euclidean or path), what information gain means, and how often the tour is recomputed.
3. Every "different" row becomes a sentence in the report's threats-to-validity section. This is a third-party implementation, so we claim to evaluate "this implementation of MRTSP", not the paper's exact code.
4. **Sanity checks:**
   - On the same environment and seeds, MRTSP total path length should be shorter than nearest's. If it is not, check the config before concluding anything.
   - Results should be consistent with the benchmark's published numbers.
5. Note where the planning loop starts and ends in the code (for Module 9) and whether its cost grows with the number of frontiers (for H2).
6. Save the config as `configs/methods/mrtsp.yaml` and fill the provenance row.

### Done when

- [ ] 5 runs done, path shorter than nearest (or the reason it is not understood and written down).
- [ ] Paper-to-code mapping table complete.

---

## Module 7: Baseline C, ARiADNE (with fallback)

**Owners:** Joey, Aaron · **Phase:** 1 · **Depends on:** Modules 2, 3

Hard deadline: **end of Week 4**. Track hours spent in `docs/ariadne_log.md`.

### Step A: does the model work? (authors' Python environment)

1. Create an isolated Python environment matching the authors' requirements (Python and PyTorch versions from their README).
2. Download the pretrained checkpoint they provide.
3. Run their test script on their test set. Compare mean trajectory length with the number in the paper.
4. Pass condition: within about 10% of the paper. Record the number of test episodes and the result.

### Step B: does it run on the Jetson?

1. Install a PyTorch build for JetPack 6 (from NVIDIA's Jetson wheels; **VERIFY** the version that matches our JetPack).
2. Load the model, run one inference on a dummy input, and record: load time, inference time, and memory used (from jtop). Remember the 8 GB is shared between CPU and GPU, and SLAM + Nav2 also need memory.

### Step C: does it run in our ROS 2 stack?

1. Open `ARiADNE-ROS-Planner`. **VERIFY first** which ROS version it targets. If it is ROS 1, budget the port (or a `ros1_bridge` setup) explicitly; it counts against the Week 4 deadline.
2. List what it needs as input (occupancy map topic, robot pose/TF, any graph or sensor-range parameters) and what it outputs (waypoint topic or action).
3. Write a small adapter node `harness/ariadne_adapter` that takes its waypoint output and sends it as a `NavigateToPose` goal to Nav2. Keep the adapter thin: no logic that would change which goal is chosen.
4. Run it on the default TB4 world until the map is mostly covered.

### Fallback decision (end of Week 4)

- If Step A **or** Step C fails: switch to **roadmap-explorer**. Build it, run its own demo, record it in the provenance table, and use `roadmap` as the method name from then on.
- Write the decision, the date, and the reason in `docs/baseline_provenance.md`. Do not keep debugging ARiADNE in Week 5.

### Done when

- [ ] Either ARiADNE passes A, B and C, or the fallback is chosen and its demo runs.
- [ ] Jetson memory and inference time recorded.

---

## Module 8: Baseline provenance table

**Owners:** all · **Phase:** 1 · **Depends on:** Modules 5 to 7

Create `docs/baseline_provenance.md`:

| Method | Paper | Repo | Commit | Config file | Published number | Our 5-run result (mean ± sd) | Pass? |
|--------|-------|------|--------|-------------|------------------|------------------------------|-------|

Under the table, one section per method with: the exact definitions found in Modules 5 to 7, every deviation from the paper, and any patch applied (with its file name in `methods/patches/`).

**Done when (Gate G1):** three methods pass, or two pass and the fallback is chosen and passes its own demo.

---

## Module 9: Planner timing instrumentation

**Owner:** Aaron · **Phase:** 2 · **Depends on:** Modules 5 to 7

Decision latency and H2 ("decision time grows with number of frontiers") need each planner to report when a planning cycle starts and ends. None of the methods do this the same way, so we add one shared topic.

### Steps

1. In `harness/`, create a message package `exploration_msgs` with `PlanningCycle.msg`:
   ```
   uint64 cycle_id
   builtin_interfaces/Time stamp_start   # ROS time
   builtin_interfaces/Time stamp_end
   float64 wall_start                    # time.time() / std::chrono system_clock
   float64 wall_end
   uint32 n_frontiers
   geometry_msgs/Point goal
   ```
2. For each method, find the function that goes from "map in" to "goal out" (you located it in Modules 5 to 7). Write a minimal patch that:
   - records both clocks right before frontier extraction starts,
   - records both clocks right after the goal is chosen (before it is sent to Nav2),
   - publishes one `PlanningCycle` on `/exploration/planning_cycle`.
3. The patch must only add timing. It must not change any logic. Review each patch with a second person.
4. Save as `methods/patches/<method>-timing.patch` and add it to `apply_patches.sh`.
5. Sanity check: run each method for 2 minutes and `ros2 topic echo /exploration/planning_cycle`. Durations should be positive and `n_frontiers` should change as the map grows.

### Done when

- [ ] All three methods publish `PlanningCycle` messages.
- [ ] Each patch reviewed by a second team member and confirmed to be timing-only.

---

## Module 10: Map set and ground-truth grids

**Owner:** Joey · **Phase:** 2 · **Depends on:** Module 3

### Steps

1. **M1 AWS Small House** and **M3 AWS Small Warehouse:** import using the approach chosen in Module 3 step 2. Put the world files and any models under `maps/<map>/`. Record the source repo and commit.
2. **M2 office corridor:** build once (corridors plus several rooms, with doors). Commit it. After the `prereg-v1` tag it is frozen; add a note in `maps/m2_office/README.md` saying so.
3. **Ground-truth occupancy grid** for each map, at the same resolution as `slam_toolbox` (default 0.05 m; **VERIFY** our config):
   - Option A (preferred): rasterize the world geometry at lidar height into a 2D grid with a script or a 2D-map plugin.
   - Option B: drive a full teleop mapping run with perfect (ground-truth) odometry, save with `nav2_map_server`'s map saver, and clean it by hand.
   - Save as `maps/<map>/gt.pgm` + `gt.yaml` (standard map_server format, with origin).
4. **Reachable free space.** From any start pose, flood-fill free cells to get the set of cells the robot can actually reach. Free cells that are not reachable (inside closed rooms, inside furniture) must not count in the coverage denominator. Save as `maps/<map>/gt_reachable.npy` (boolean mask). Also add the obstacle cells that border reachable free space; these are what a lidar can "see".
5. Write the cell counts (free, occupied, reachable) to `maps/<map>/README.md`.

### Done when

- [ ] All three maps launch in sim with the TB4.
- [ ] Each has `gt.pgm`, `gt.yaml`, `gt_reachable.npy`, overlaid on a teleop SLAM map in a figure to show they line up.

---

## Module 11: Start-pose generator

**Owner:** Joey · **Phase:** 2 · **Depends on:** Module 10

### Steps

1. Write `harness/scripts/gen_start_poses.py --map <map> --n 20 --seed 249`:
   - Load `gt.pgm` and `gt_reachable.npy`.
   - Erode the reachable free space by the robot radius plus a margin (TB4 radius is about 0.17 m; use 0.35 m clearance).
   - Sample `(x, y)` uniformly from the eroded cells and `yaw` uniformly in `[-pi, pi)` with `numpy.random.default_rng(seed)`.
   - Reject poses closer than 1.5 m to an already chosen pose, so poses are spread out.
2. Generate **20** per map (enough for Block Q at 15 plus spares). Blocks use the first N: pilot uses 0 to 2, Block Q uses 0 to 9 (or 0 to 14), Block E uses 0 to 4.
3. Write to `configs/start_poses/<map>.yaml`:
   ```yaml
   map: m1_house
   seed: 249
   generator_commit: <git sha>
   poses:
     - {x: 1.23, y: -0.45, yaw: 0.78}
     - ...
   ```
4. Spawn the robot at each pose once in sim and confirm it is not in collision and SLAM starts. Replace any bad pose with the next sampled one, and note it.
5. Commit the YAML files. They never change after `prereg-v1`.

### Done when

- [ ] 20 validated poses per map committed, with a figure showing them on each map.

---

## Module 12: Recorder node

**Owner:** Shervan · **Phase:** 2 · **Depends on:** Module 9

A ROS 2 Python node, `harness/recorder.py`, started by the orchestrator with `--out logs/<run_id>`.

### Subscriptions and what to write

| Source | Rate | Writes |
|--------|------|--------|
| TF `map -> base_link` (lookup on a timer) | 10 Hz | `pose.csv` |
| `/exploration/planning_cycle` | every message | `cycles.csv`, and a row in `goals.csv` |
| Nav2 `navigate_to_pose` action status (`/navigate_to_pose/_action/status`) | every change | `nav2_results.csv` |
| `/map` (`nav_msgs/OccupancyGrid`) | snapshot every 5 s sim time | `maps/map_<seq>.npz` |

### Details

- Each map snapshot `.npz` contains: `data` (int8 array), `resolution`, `origin_x`, `origin_y`, `origin_yaw`, `width`, `height`, `t_ros`, `t_wall`. Save the origin every time; `slam_toolbox` grows the map and moves the origin.
- Path length is computed offline from `pose.csv` (sum of distances between consecutive poses, ignoring jumps under 1 cm to filter jitter). Do not trust a live counter.
- Flush CSV files every second so a crash still leaves usable data.
- Also publish a live `std_msgs/Float32` on `/exploration/coverage_live` using the same coverage function as Module 13. The orchestrator uses this to stop the run; the offline evaluator remains the official number.
- Use `use_sim_time:=true`. Get `t_wall` from `time.time()`.

### Done when

- [ ] A 3-minute run produces every file in Section 0.2 (except power/coverage) with the right columns.
- [ ] Killing the recorder with SIGINT leaves valid, closed files.

---

## Module 13: Coverage evaluator

**Owner:** Shervan · **Phase:** 2 · **Depends on:** Modules 10, 12

`analysis/coverage.py`, usable as a library (for the live topic) and as a CLI: `python analysis/coverage.py logs/<run_id>`.

### Algorithm

1. **Align frames.** The SLAM `map` frame starts at the robot's spawn pose; the ground-truth grid is in the world frame. Read the start pose from the manifest and build the transform `world <- map`. Check alignment visually on one run per map (overlay the final SLAM map on GT).
2. For each snapshot:
   - Mark SLAM cells as **known** if value is `0` (free) or `>= 65` (occupied); `-1` is unknown. (**VERIFY** the thresholds against our `slam_toolbox` config and use the same ones everywhere.)
   - Transform the GT reference cells (reachable free + visible obstacle cells from Module 10) into SLAM grid coordinates, and look up whether each one is known. Use nearest-neighbour lookup.
   - `coverage_frac = known GT reference cells / total GT reference cells`.
3. For each snapshot time, compute `distance_m` = path length from `pose.csv` up to that `t_ros`.
4. Write `coverage.csv`, then compute and write to `summary.json`:
   - `t_95`, `d_95` (first time/distance where `coverage_frac >= 0.95`, linearly interpolated between snapshots; `null` if never reached),
   - `final_coverage`, `total_distance_m`, `total_time_s`, `outcome`.
5. **Oscillation metric** (needed for the pre-registration): from `goals.csv`, count goal switches where the new goal is more than X m from the previous one and the previous goal was neither reached nor aborted. Pick X in the pilot and freeze it in Module 18.

### Tests

- A synthetic test: a fake GT map and a fake SLAM map with a known fraction filled must return that fraction.
- A frame test: shift the fake SLAM origin and confirm the result does not change after alignment.

### Done when

- [ ] Tests pass.
- [ ] Final coverage on a manual full-teleop run is close to 100%.

---

## Module 14: Power and compute logger

**Owner:** Aaron · **Phase:** 2 · **Depends on:** Module 2

`harness/power_logger.py --out logs/<run_id> --procs <comma-separated process names>`, runs on the Jetson.

### What it logs

1. **Power** (`power.csv`): from jtop, every sample: `VDD_IN`, `VDD_CPU_GPU_CV`, `VDD_SOC`, plus CPU and GPU temperature. Use the rate measured in Module 2.
2. **Per-process CPU** (`procs.csv`): once per second, for each process whose name matches the list (planner node, `slam_toolbox`, Nav2 servers, recorder):
   ```python
   import os
   CLK = os.sysconf("SC_CLK_TCK")
   def cpu_times(pid):
       with open(f"/proc/{pid}/stat") as f:
           fields = f.read().rsplit(")", 1)[1].split()
       return int(fields[11]) / CLK, int(fields[12]) / CLK   # utime, stime
   ```
   (After the `)` split, field index 11 is `utime` and 12 is `stime`, matching fields 14 and 15 in `man proc`.) Re-scan `/proc` for matching names every few seconds in case a node restarts.
3. **GPU** (`gpu.csv`): GPU utilization, GPU frequency and RAM used from jtop, once per second.

### Idle-stack baseline

Script `scripts/idle_baseline.sh <power_mode>`: sim running on the laptop, SLAM + Nav2 running on the Jetson, robot stationary, **no planner**, 5 minutes. Save to `logs/idle_<power_mode>_<timestamp>/`. Repeat at the start of each Block E session (Module 19).

### Energy helper

Put this in `analysis/energy.py` so everyone computes energy the same way:

```python
import numpy as np
def energy_J(t_wall, power_mW, t0, t1):
    m = (t_wall >= t0) & (t_wall <= t1)
    return np.trapz(power_mW[m] / 1000.0, t_wall[m])
```

Energy above idle for a run: `E_run - P_idle_mean * (t1 - t0)`, where `P_idle_mean` comes from the idle baseline of the same session and power mode.

### Done when

- [ ] A 3-minute run produces `power.csv`, `procs.csv`, `gpu.csv` on one `t_wall` base.
- [ ] Idle baseline recorded for both Block E power modes.
- [ ] Planner process CPU time is clearly above zero during a run and near zero when it is not running.

---

## Module 15: Common experiment config

**Owners:** all · **Phase:** 2 · **Depends on:** Modules 5 to 7

Create `configs/common.yaml`. Every method reads from it (directly, or by the orchestrator passing these values as parameters):

```yaml
replan_period_s: 1.0         # deadline for one planning cycle; set from the pilot
goal_commitment: method_default   # see below
timeout_sim_s: 1200          # per run; set from pilot (e.g. 2x the slowest pilot completion)
coverage_target: 0.95
map_snapshot_period_s: 5.0
slam_params: configs/slam_toolbox.yaml
nav2_params: configs/nav2.yaml
min_frontier_size_m: 0.3
```

Rules:

- **Same** SLAM params, Nav2 params, robot, sensor, timeout and coverage target for every method.
- Set each method's replanning rate to `1 / replan_period_s` where the method has such a parameter.
- Goal commitment: where a method has its own logic (blacklisting, progress timeout, re-planning only on goal completion), **keep it and document it** in a table in `docs/common_rules.md`. Do not override method logic.
- Commit per-method params in `configs/methods/<method>.yaml`.

### Done when

- [ ] `docs/common_rules.md` has one row per method showing replanning rate, commitment logic, and any parameter that differs from the method's defaults (with the reason).

---

## Module 16: Run orchestrator

**Owner:** Shervan · **Phase:** 2 · **Depends on:** Modules 11 to 15

`harness/run.py --block <b> --method <m> --map <map> --pose <i> --power-mode <p> [--jetson]`

### What it does, in order

1. Build the `run_id` (Section 0.1), create `logs/<run_id>/`.
2. If `--power-mode` is a Jetson mode: check the current mode with `nvpmodel -q` and **refuse to run** if it does not match.
3. Write the first part of `manifest.yaml`: `run_id`, block, method, map, pose index and values, power mode, git commit of the main repo, commit of each submodule (`git submodule status`), hashes of all config files used, hostname, chrony offset, start time.
4. Launch, each in its own process group (`subprocess.Popen(..., start_new_session=True)`), with stdout/stderr to `stdout/<name>.log`:
   1. Gazebo with the map and the robot at the start pose.
   2. `slam_toolbox`, Nav2.
   3. Wait until Nav2 is active (lifecycle check or `/navigate_to_pose` action server available). Timeout 60 s, else `invalid`.
   4. Power logger (Jetson runs only).
   5. Recorder.
   6. The planner.
5. **Monitor** loop, once per second:
   - `coverage_live >= 0.95` → `complete`, but keep running 5 s more so the final snapshot is saved.
   - sim time > `timeout_sim_s` → `timeout`.
   - planner has exited, or no new planning cycle for 60 s sim time while coverage < 95% → `stuck`.
   - any infrastructure process died, `/clock` stopped advancing for 10 s wall, or robot pose jumped impossibly → `invalid`.
6. **Shutdown** in reverse order: SIGINT to each process group, wait 10 s, then SIGKILL. Afterwards, check no `gzserver`/`ign`/`ros2` processes are left (`pkill` leftovers and note it in the manifest if anything had to be killed).
7. Finish the manifest: end time, outcome, reason string.
8. Run `analysis/coverage.py` on the folder to produce `coverage.csv` and `summary.json`.

### Done when (Gate G2)

- [ ] On a fresh clone, `run.py` works for every method on every map.
- [ ] The output folder has every file in Section 0.2.
- [ ] 10 back-to-back runs leave no orphan processes.

---

## Module 17: Pilot and sample-size decision

**Owners:** all · **Phase:** 3 · **Depends on:** Module 16

### Steps

1. **Block P:** 3 methods × 3 maps × poses 0, 1, 2 on laptops = 27 runs.
2. For each map, pose, and info-aware method, compute the relative paired difference:
   `r = (d_95[method] - d_95[nearest]) / d_95[nearest]`
3. Estimate the sample size, in `analysis/sample_size.py`:
   ```python
   import numpy as np
   from scipy.stats import norm, chi2

   def n_needed(r, delta=0.10, alpha=0.05 / 3, power=0.8, are=0.864):
       # alpha/3: Holm worst case across 3 maps
       # are: Wilcoxon asymptotic relative efficiency vs. t-test
       r = np.asarray(r)
       k = len(r) - 1
       sd = r.std(ddof=1)
       sd_upper = sd * np.sqrt(k / chi2.ppf(0.20, k))   # 80% upper bound on sd
       z = norm.ppf(1 - alpha / 2) + norm.ppf(power)
       n_point = (z * sd / delta) ** 2 / are
       n_safe = (z * sd_upper / delta) ** 2 / are
       return int(np.ceil(n_point)), int(np.ceil(n_safe))
   ```
   With 3 poses per map the sd estimate is very noisy, so **pool across the three maps** (9 differences per method) unless the maps are obviously different, and decide on the conservative `n_safe`.
4. Decide using the rule in plan.md: if 10 per map is enough, Block Q stays at 10 and the ablation stays in; otherwise Block Q goes to 15 and the ablation is dropped.
5. Use the pilot to also fix: `timeout_sim_s` (about 2× the slowest completed pilot run), `replan_period_s`, the oscillation threshold X, and expected wall time per run (for scheduling).
6. Pilot runs are **not** part of the main results. Keep them in `logs/` and report them only as the pilot.

### Done when

- [ ] `docs/pilot_report.md`: the 27 results, sd estimates, the sample-size numbers and the decision.

---

## Module 18: Pre-registration

**Owners:** all · **Phase:** 3 · **Depends on:** Module 17

Write `docs/preregistration.md` with:

1. **Hypotheses H1 to H3**, copied from the proposal, each with the exact test that confirms or rejects it.
2. **Primary metrics:** `d_95` (path length to 95% coverage) for quality; planner energy above idle, decision latency (median, p95) and deadline-miss rate for compute. Give the formula for each.
3. **Secondary metrics:** `t_95`, final coverage, oscillation count (with X), CPU time, GPU utilization.
4. **Termination rule** and outcome definitions (Section 0.5).
5. **How timeouts enter the stats:** state it now. Recommended: for paired tests, assign the run's `d_95` as the timeout distance and flag it; report the number of timeouts per cell separately.
6. **Run counts** per block, pose indices used, power modes.
7. **Analysis:** bootstrap 95% CI on the median paired difference (10,000 resamples, fixed seed), Wilcoxon signed-rank, Holm correction across maps. The analysis script path and the commit it is at.
8. **Invalid-run rule:** rerun with the same pose; rerun limit of 2 per cell; if more, report the cell as missing.

Then:
```bash
git add docs/preregistration.md
git commit -m "Pre-registration v1"
git tag -a prereg-v1 -m "Protocol frozen"
git push origin prereg-v1
```

### Done when (Gate G3)

- [ ] Tag pushed, all three have replied "acknowledged" in the team channel.

---

## Module 19: Batch runner for Blocks Q and E

**Owner:** Shervan · **Phase:** 4 · **Depends on:** Modules 16, 18

### Steps

1. `scripts/make_schedule.py --block Q --poses 10 --seed 7` writes `configs/schedules/Q.csv` with one row per run (method, map, pose, power mode, machine). Shuffle the order with the fixed seed so a method never runs in a long consecutive streak.
2. For Block E, generate one schedule per **session** (one power mode per session). Within a session, randomize method order across poses.
3. `scripts/run_batch.py configs/schedules/Q.csv --machine <laptop-name>`:
   - Skips rows that already have a `complete`, `timeout` or `stuck` run.
   - Retries `invalid` runs at the end, at most 2 extra attempts per row.
   - Writes `logs/batch_<date>.log` with one line per run.
4. **Block E session procedure** (write it as a checklist in `docs/block_e_procedure.md`):
   1. `sudo nvpmodel -m <id>`, reboot.
   2. Wait for the CPU temperature to stabilize (less than 1 °C change over 5 min), and record it.
   3. Run the idle-stack baseline (Module 14).
   4. Run the session schedule.
   5. Run a second idle baseline at the end to detect drift.
5. **Daily check** (10 minutes): `scripts/status.py` prints planned vs. done per cell, counts of each outcome, and plots one random coverage curve and one random power trace to `logs/daily/<date>/`. Write anything odd in `docs/run_log.md`.

### Done when (Gate G4)

- [ ] All planned runs present, fewer than 5% invalid, every rerun in `docs/run_log.md`.

---

## Module 20: Analysis pipeline

**Owner:** Aaron leads, all · **Phase:** 5 · **Depends on:** Module 19

### Structure

```
analysis/
  load.py          # walk logs/, read manifest + summary + CSVs into one tidy DataFrame
  coverage.py      # Module 13
  energy.py        # Module 14
  stats.py         # bootstrap, Wilcoxon, Holm
  q1_quality.py
  q2_compute.py
  q3_tradeoff.py
  h2_scaling.py
  make_figures.py  # calls everything above, writes analysis/out/
```

`python analysis/make_figures.py` must rebuild every figure and every number in the report from `logs/`. Write all numbers the report quotes to `analysis/out/numbers.json` and pull them into the report from there, so nothing is typed by hand.

### Q1: quality (Block Q)

- For each map and info-aware method: paired difference `d_95[method] - d_95[nearest]` per pose.
- Bootstrap 95% CI of the median difference (resample poses), Wilcoxon signed-rank p-value, then Holm across the 3 maps (per method).
- Figure: coverage vs. distance, one panel per map, median line and IQR band per method.

### Q2: compute and energy (Block E)

Per method × map × power mode:

- Decision latency from `cycles.csv`: `wall_end - wall_start`, median and p95.
- Deadline-miss rate: fraction of cycles with latency > `replan_period_s`.
- Planner CPU time: last minus first `utime + stime` for the planner process in `procs.csv`.
- Board energy over the run and energy above idle (Module 14).

### Q3: the trade-off

- **Pareto plot** per map: x = planner energy above idle (J), y = `d_95` (m), one point per method × power mode, with bootstrap CI whiskers on both axes. Use Block E runs so energy and distance come from the same runs.
- **Break-even:** `E_mission = E_board + k_loco * distance`. For an info-aware method vs. nearest:
  - `dE = E_board[info] - E_board[nearest]` (extra compute-side energy, J)
  - `dD = d[nearest] - d[info]` (distance saved, m)
  - If `dD > 0`, the break-even is `k* = dE / dD` (J/m). Info-aware is worth it when the robot's locomotion cost `k_loco > k*`. If `dD <= 0`, there is no break-even: nearest wins at any `k_loco`.
  - Sweep `k_loco` over a range justified from the literature for a TurtleBot-class base (cite the source in the report), plot `E_mission` vs. `k_loco` per method, and mark `k*` with a bootstrap CI.

### H2

- Scatter of decision latency vs. `n_frontiers` (from `cycles.csv`), per method, Block E, per power mode. Fit a line (or log-log slope) and report the slope with a CI. Draw the `replan_period_s` deadline as a horizontal line.

### Done when (Gate G5)

- [ ] Deleting `analysis/out/` and running `make_figures.py` rebuilds everything.
- [ ] A second team member runs it on their machine and gets the same numbers.

---

## Module 21: Scaling ablation (optional)

**Owners:** Aaron, Shervan · **Phase:** 6 · **Depends on:** Modules 19, 20

Decide at the start of Week 11. Only run it if Blocks Q and E are complete and Module 20 is passing.

1. Build `m2_office_2x`: M2 scaled to twice the area (repeat the layout, do not redesign it). Generate GT and start poses with Modules 10 and 11.
2. **Block A:** 3 methods × 5 poses on the Jetson, high power mode (and low, if time allows).
3. Reuse `h2_scaling.py`: does MRTSP latency cross `replan_period_s` as `n_frontiers` grows?
4. Report it as exploratory: it was decided after the pilot, so state that it is not part of the pre-registered tests unless the pre-registration already listed it.

---

## Module 22: Write-up, video, slides

**Owners:** all · **Phase:** 7 · **Depends on:** Module 20

1. **Report**, in the order given in plan.md Phase 7. Every number comes from `analysis/out/numbers.json`. Threats to validity must include at least: sim vs. real locomotion, third-party MRTSP implementation (link `docs/mrtsp_mapping.md`), single board, modeled `k_loco`, and the Gazebo version / world-port choice from Module 3.
2. **Demo video:** pick one pose on M1, record one run per method (screen capture RViz), place them side by side, and overlay the power trace from `power.csv` synchronized by `t_wall`. Keep the run IDs in the video description.
3. **Slides:** open with the research question and the Pareto figure. Architecture goes at the end or in backup.
4. **Archive:** zip `logs/` with a checksum, store it outside git, and put the link plus checksum in the README.

---

## Quick dependency summary

```
0 → 1 → 2 ─────────────────────────┐
        └→ 3 → 5 → 6               │
           │    └──→ 7 → 8 (G1)    │
           │              │        │
           │              ▼        ▼
           └→ 10 → 11    9 → 12   14
                  │        │       │
                  └→ 13 ◄──┘       │
                       │   15      │
                       ▼   ▼       ▼
                         16 (G2) ◄─┘
                          │
                          ▼
                     17 → 18 (G3) → 19 (G4) → 20 (G5) → 21 (opt) → 22
```

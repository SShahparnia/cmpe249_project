# Project Proposal: ExploreBench

**A Controlled Comparison of Frontier-Selection Strategies for Autonomous Exploration and Mapping, with Embedded Deployment on the NVIDIA Jetson Orin Nano**

CMPE 249, Fall 2026
**Team:** Shervan Shahparnia, Joey Manivong, Aaron Sam
**Track:** System (primary), Deployment (secondary)
**Repository:** https://github.com/SShahparnia/cmpe249_project

## 1. Problem Formulation

### 1.1 Domain problem

A mobile robot is placed at an unknown pose in an unknown indoor environment with no prior map and no operator. It must map the space in as little time and distance as possible. This is autonomous exploration, the capability behind search-and-rescue robots, warehouse inventory systems, and inspection drones.

The decision the robot repeats is where to drive next. Because the map changes with every observation, the optimal policy is intractable, so practical systems use a greedy heuristic over candidate viewpoints. The dominant family selects among frontiers, which are cells on the boundary between known-free and unknown space.

**The gap.** Practitioners default to nearest-frontier greedy selection, mostly because that is what `explore_lite` and the standard ROS packages implement. The literature comparing it to alternatives is fragmented across incompatible simulators, robots, and metrics, usually with under ten runs and no variance estimates. Brugali et al. (IJRR 2025) document this fragmentation systematically. We found no recent study that holds everything but the selection rule fixed, reports paired significance testing, or measures what each strategy costs on embedded hardware. We propose no new algorithm. We ask whether the community's default is the right default, when it stops being one, and what each alternative costs on a 15 W SoC.

### 1.2 Input and output specification

Per decision cycle, the exploration node consumes and produces:

| Dir | Signal | Type | Source |
|---|---|---|---|
| In | Occupancy grid | `nav_msgs/OccupancyGrid`, {-1 unknown, 0 free, 100 occupied}, 5 cm | `slam_toolbox`, or ground-truth grid in the primary condition |
| In | Robot pose | `geometry_msgs/PoseStamped`, map frame | TF, with Gazebo ground truth logged in parallel |
| In | LiDAR scan | `sensor_msgs/LaserScan`, 2D, range R | Simulated TurtleBot 4 |
| In | Path costs | `dict[frontier_id, float]`, planned path length | Nav2 planner server |
| Out | Next goal | `geometry_msgs/PoseStamped` via `NavigateToPose` | Our node to Nav2 |
| Out | Telemetry | CSV per tick, JSON per episode | Our metrics recorder |

Every strategy is a plugin behind one interface, `score(frontiers, grid, robot_pose, path_costs) -> dict[id, float]`, highest score wins. All strategies therefore share identical frontier detection, commitment logic, navigation, and logging. The only variable across conditions is the scoring function, which is what makes the comparison controlled rather than anecdotal.

An episode ends when no frontier cluster remains that is both above the size threshold and reachable per the global planner, or when a sim-time budget expires. Budget-exhausted episodes are logged and reported separately as a real failure mode, not discarded.

### 1.3 Target metrics and success criteria

| Class | Metrics |
|---|---|
| Efficiency | Coverage vs. simulated time and vs. distance travelled; time and distance to 80, 90, 95 percent coverage; metres travelled per m² mapped |
| Map quality | Cell classification error and free-space IoU vs. ground truth at matched coverage; SLAM absolute trajectory error; loop-closure count |
| Platform | Decision latency per detection and scoring call; latency vs. mapped area; power and peak memory via `tegrastats` at 7 W, 15 W, 25 W |
| Robustness | Failure rate (deadlock, unreachable goals, budget exhaustion); oscillation index (goal switches per goal reached) |

Ground-truth reachable free area is extracted once per world by flood-filling a high-resolution grid rendered from the SDF geometry, and unit-tested against hand-measured worlds before any coverage number is trusted.

**Research questions.** 
RQ1: how do strategies compare on efficiency, and does the ranking depend on environment topology? 
RQ2: does the cost-utility weight lambda have a robust setting, or is the optimum environment-dependent? 
RQ3: does exploration speed trade against map quality with SLAM in the loop? 
RQ4: what is each strategy's decision latency on embedded hardware, and where does information gain become infeasible? 
RQ5: how much does goal commitment (hysteresis) affect efficiency?

**Success criteria.** 
(1) One command reproduces any figure from the released code and logs. 
(2) Coverage-time and coverage-distance curves for at least three strategies across at least three environments, with bootstrap confidence intervals over at least ten seeds each. 
(3) An evidence-backed answer to RQ1. 
(4) Jetson latency reported per strategy with an explicit infeasibility point. 
(5) Failure modes, negative results, and threats to validity documented. None of these require a strategy to win. A null result reported with a power analysis satisfies every criterion.

## 2. Proposed Technical Approach

### 2.1 Architecture

Gazebo supplies sensors, odometry, and ground-truth pose. `slam_toolbox` publishes the occupancy grid. Our `exploration_node` runs a five-stage loop: frontier detection, candidate scoring by the active strategy plugin, commitment filter, goal dispatch to Nav2 with a watchdog, and termination check. A separate `metrics_recorder` node observes the map, both pose sources, and internal exploration events, and writes the logs.

For RQ4 the setup is hardware-in-the-loop. Gazebo stays on a workstation while the entire ROS 2 stack, meaning `slam_toolbox`, Nav2, and our node, runs on the Jetson over the local network. Every latency number therefore comes from the real embedded target.

### 2.2 Components

We adopt Gazebo Harmonic, ROS 2 Jazzy, the TurtleBot 4 model, `slam_toolbox` in asynchronous mode, and Nav2. We write the exploration node (frontier detection, four strategy plugins, commitment logic) and the entire evaluation layer (ground-truth extraction, metrics recorder, batch runner, statistics, figures). About 70 percent of the code we write is in those last two components. That split is deliberate: enough implementation to be real engineering, not so much that we reimplement a navigation stack and run out of semester before the experiment starts.

Frontier detection each cycle: identify free cells with an unknown 8-neighbour, reject cells closer to an obstacle than the robot's inscribed radius using a distance transform, cluster by connected components, discard clusters below a size threshold, and emit a centroid and goal pose per cluster. Complexity is O(N) in grid cells, with the label image cached for incremental updates.

**The four strategies.** With `d(f)` as cost-to-go and `I(f)` as estimated information gain:

| | Strategy | Rule and purpose |
|---|---|---|
| S0 | Random | Uniform over reachable free cells. Calibrates the scale of every other strategy's improvement. Commonly omitted, so we include it. |
| S1 | Nearest frontier | `U = -d(f)`. The community default. Built two ways, Euclidean and true planned path length, since Euclidean cost is wrong whenever a wall intervenes. |
| S2 | Max information gain | `U = I(f)`, with two estimators reported separately. Count: unknown cells within range R, cheap but ignores occlusion. Raycast: K rays from f to R stopping at the first occupied cell, respects occlusion at O(K·R) per candidate, and is the estimator profiled in RQ4. |
| S3 | Cost-utility | `U = I(f) - lambda·d(f)`, with I normalized to [0,1] per iteration. S1 and S2 are the limiting cases as lambda goes to infinity and to zero. |

Commitment applies identically to all strategies. Naive per-cycle replanning makes the robot oscillate between near-tied candidates and waste large amounts of time, so we add a bonus to the held goal and reconsider only on arrival, invalidation, or watchdog timeout.

### 2.3 Intended modifications and advanced capabilities

**(a) The lambda sweep turns four algorithms into a design space.** Sweeping lambda over {0, 0.25, 0.5, 1, 2, 4, infinity} changes the question from "which of these four is best" to "what does the cost-information weighting do, and does its optimum transfer across environments." This is the most informative figure we expect to produce.

**(b) Controlled design at a sample size the field does not use.** Randomized start poses, per-seed pairing across strategies, bootstrap confidence intervals, Wilcoxon signed-rank tests with Holm-Bonferroni correction, and effect sizes rather than bare p-values. The largest sample size in any comparable recent paper we surveyed was ten trials per scenario; we run twenty seeds per cell. Metrics and the statistical plan are frozen at the end of Week 8, before the main experiment runs.

**(c) Embedded latency as a first-class result.** Every state-of-the-art planner we surveyed treats computation as a design constraint. TARE restructures the map representation for it, PIPE headlines an 83.3 percent computation reduction, EPIC abandons the volumetric grid for memory reasons, and Lewis et al. (2026) make frontier-detection complexity their contribution. None of them reports where a strategy stops meeting a real-time budget on a 15 W SoC. We measure that curve, then try to push the crossing point out with incremental frontier updates or coarse-to-fine scoring and report before and after.

### 2.4 Data sources

There is no training dataset. This is a systems and evaluation project, not a learning one, so our data is the environments and the episode logs they produce. Five worlds: E1 small house and E2 warehouse from the open-source AWS RoboMaker set, plus E3 maze, E4 cluttered open hall, and E5 loop-heavy corridors generated procedurally by our own script. The AWS worlds target Gazebo Classic, so Week 1 budgets time for SDF compatibility work.

| Experiment | Design | Runs |
|---|---|---|
| Primary (RQ1, RQ2) | 4 strategies × 5 environments × 20 seeds, ground-truth localization | 400 |
| Secondary (RQ3) | Identical, with `slam_toolbox` in the loop | 200 |
| Lambda sweep, sensor range, commitment, Euclidean vs. path cost | Four ablations | 730 |
| **Total** | | **≈ 1,330** |

At 6 to 10 minutes of simulated time per episode and a headless real-time factor of 2 to 4, that is roughly 45 to 90 machine-hours single-stream, which four parallel Gazebo instances across two machines cover in three nights. Fallback if the harness is slower than estimated: drop to 10 seeds and 3 environments, a 60 percent cut, decided at the start of Week 9 and not later. Running the primary condition with ground-truth localization is what lets us attribute RQ3's map-quality differences to the exploration policy rather than to SLAM.

## 3. Device Availability and Maintainer

**Device.** One NVIDIA Jetson Orin Nano Super 8 GB developer kit, borrowed from Prof. Liu's storage and shared by the team. If spare boards are available we will borrow additional units so that bring-up and profiling can run in parallel, but the plan assumes a single shared board. The board runs JetPack 7.x on Ubuntu 24.04 and supports 7 W, 15 W, and 25 W power modes, which gives us a latency-versus-power curve at no extra implementation cost. Simulation, batch runs, and analysis happen on team members' own x86-64 Linux workstations.

Because one board is shared, Jetson access is scheduled rather than assumed. Bring-up starts in Week 1 rather than Week 12, the board is flashed once with a container image that all three of us can reproduce, and the Week 12 profiling runs are booked as blocks so that two people are never blocked waiting on it. Profiling is the only task that truly needs the hardware; everything else runs in simulation.

The 8 GB is unified memory shared between CPU and GPU. JetPack and the ROS stack consume 1.5 to 2 GB, leaving roughly 6 GB usable. A 100 by 100 m world at 5 cm resolution is 4 million cells, and naive Python frontier processing with intermediate array copies will hit that ceiling. Writing memory-conscious code is part of the intended learning outcome, and the ceiling is itself a reportable RQ4 result. No training hardware is required, since nothing in this project trains a model.

**Maintainers.** Jetson board, JetPack flashing, containers, and the profiling harness: Shervan Shahparnia and Aaron Sam. Simulation, worlds, batch runner, and CI: Shervan Shahparnia and Joey Manivong. Exploration algorithms and strategy plugins: Joey Manivong and Aaron Sam. Repository maintainer and point of contact: Shervan Shahparnia. Hardware loaned by Prof. Liu. Every subsystem has two owners and every member owns two subsystems, so no component has a single point of failure.

**Agentic tooling (Claude Code).** We intend to use Claude Code as a development assistant in this repository, run from each member's own workstation against a normal working copy, not on the Jetson and not in any unattended mode. It helps with ROS 2 boilerplate, the batch runner and log-parsing scripts, plotting and statistics code, and test scaffolding. We will not use it for the experimental design, metric definitions, statistical plan, or interpretation of results, since those are the graded intellectual content and must be ours. Every change is a reviewed pull request with a human author who can explain it, and anything touching the frozen metrics or analysis code after Week 8 requires sign-off from both evaluation owners.

## Key references

Full annotated survey in `docs/LITERATURE_SURVEY.md`. Brugali, Muratore and De Luca (2025), IJRR. Placed et al. (2023), IEEE T-RO 39(3). Cao et al. (2023), Science Robotics 8(80). Ho et al. (2025), ICRA. Baek et al. (2025), arXiv 2503.07504. Liu et al. (2025), Scientific Reports 15:12261. Calzolari et al. (2025), arXiv 2504.11907. Zhan et al. (2025), IEEE RA-L. Geng et al. (2025), IEEE RA-L. Lewis, Basiri and Lima (2026), arXiv 2604.03008. Method sources for S1 to S3: Yamauchi (1997); Gonzalez-Banos and Latombe (2002); Burgard et al. (2005); Holz et al. (2010).

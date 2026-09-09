# Project Proposal — ExploreBench

**A Controlled Comparison of Frontier-Selection Strategies for Autonomous Exploration and Mapping, with Embedded Deployment on the NVIDIA Jetson Orin Nano**

CMPE 249 · Fall 2026 · **Team:** Shervan Shahparnia, Joey Manivong, Aaron Sam
**Track:** System (primary) + Deployment (secondary) · **Repo:** https://github.com/SShahparnia/cmpe249_project

---

## 1. Problem Formulation

### 1.1 Domain problem

A mobile robot is placed at an unknown pose in an unknown indoor environment with no prior map and no operator. It must map the environment in as little time and distance as possible. This is **autonomous exploration**, the enabling capability behind search-and-rescue robots, warehouse inventory systems, and inspection drones.

The decision the robot makes repeatedly is *where to drive next*. Because the map changes with every observation, the optimal policy is intractable, so every practical system uses a greedy heuristic over candidate viewpoints. The dominant family selects among **frontiers** — cells on the boundary between known-free and unknown space, which are by construction the places from which new information is observable.

**The gap.** Practitioners overwhelmingly default to *nearest-frontier greedy* selection, largely because it is what `explore_lite` and the standard ROS packages implement. The literature comparing it to alternatives is fragmented across incompatible simulators, robots, and metrics, typically with under ten runs and no variance estimates — fragmentation documented systematically by Brugali et al. (IJRR 2025). Our survey found no recent study that holds everything but the selection rule fixed, reports variance and paired significance testing, or measures what each strategy costs on embedded hardware. We propose no new algorithm. We ask whether the community's default is the right default, when it stops being one, and what each alternative costs on a 15 W SoC.

### 1.2 Input / output specification

**System level**, per decision cycle:

| | Signal | Type | Source |
|---|---|---|---|
| In | Occupancy grid `M_t` | `nav_msgs/OccupancyGrid`, `{-1 unknown, 0 free, 100 occupied}`, 5 cm | `slam_toolbox`, or ground-truth grid in the primary condition |
| In | Robot pose `x_t` | `geometry_msgs/PoseStamped`, map frame | TF; Gazebo ground truth logged in parallel |
| In | LiDAR scan | `sensor_msgs/LaserScan`, 2D, range `R` | Simulated TurtleBot 4 |
| In | Path costs | `dict[frontier_id, float]`, planned path length | Nav2 planner server |
| Out | Next goal `g` | `geometry_msgs/PoseStamped` via `NavigateToPose` action | Our node → Nav2 |
| Out | Per-tick telemetry | CSV row: coverage, distance, sim time, chosen frontier, per-stage latency | Our metrics recorder |
| Out | Per-episode summary | JSON: thresholds crossed, termination reason, failures, map quality | Our metrics recorder |

**Strategy level.** Every strategy is a plugin behind one interface, which is what makes the comparison controlled rather than anecdotal:

```python
class Strategy(Protocol):
    def score(self, frontiers: list[FrontierCluster], grid: OccupancyGrid,
              robot_pose: Pose, path_costs: dict[int, float]) -> dict[int, float]:
        """Return a utility score per frontier id. Highest wins."""
```

All strategies therefore share identical frontier detection, commitment logic, navigation, and logging. The only variable across conditions is the scoring function.

**Termination.** An episode ends when no frontier cluster remains that is both larger than `min_frontier_cells` and reachable per the global planner, or when a hard sim-time budget expires. Budget-exhausted episodes are recorded and reported separately as a legitimate failure mode, not discarded.

### 1.3 Target metrics

| Class | Metric |
|---|---|
| **Efficiency (primary)** | Coverage `C(t)` and `C(s)` — fraction of ground-truth reachable free area mapped, vs. simulated time and vs. distance |
| | `T_80/T_90/T_95` and `D_80/D_90/D_95` — time and distance to each coverage threshold |
| | `η` — exploration cost, metres travelled per m² mapped |
| **Map quality (RQ3)** | Cell classification error and free-space IoU vs. ground truth, **at matched coverage** |
| | ATE of the SLAM estimate vs. Gazebo ground truth; loop-closure count |
| **Platform (RQ4)** | Decision latency per detection call and per scoring call, as a distribution, on workstation and Jetson |
| | Latency vs. mapped area — where each strategy crosses the real-time budget |
| | Power and peak memory via `tegrastats` at 7 W / 15 W / 25 W |
| **Robustness** | Failure rate (deadlock, unreachable-goal loops, budget exhaustion); oscillation index (goal switches per goal reached) |

Ground-truth reachable free area is extracted once per world by flood-filling a high-resolution grid rendered from the world's SDF geometry, unit-tested against hand-measured worlds before any coverage number is trusted.

### 1.4 Research questions

| ID | Question | Hypothesis |
|---|---|---|
| **RQ1** | How do strategies compare on efficiency, and does the ranking depend on environment topology? | Greedy is competitive in corridors and mazes where proximity correlates with information; information-gain wins in open halls. No strategy dominates all five environments. |
| **RQ2** | Does the cost–utility weight λ admit a robust setting? | An intermediate λ beats both endpoints in most environments, but the optimum scales with environment size, so no single λ transfers. |
| **RQ3** | Does exploration speed trade against map quality with SLAM in the loop? | At matched coverage, faster strategies show higher classification error and larger ATE, because fewer revisits mean fewer loop closures. |
| **RQ4** | What is each strategy's decision latency on embedded hardware? | Raycast information gain scales super-linearly with mapped area and exceeds a 1 Hz budget past a threshold area at 15 W; nearest-frontier stays effectively constant-time. |
| **RQ5** | How much does goal commitment affect efficiency? | Commitment improves all strategies by removing oscillation between near-tied candidates, most for information-gain, whose scores are noisiest. |

### 1.5 Success criteria

1. A single command reproduces any figure in the report from the released code and logs.
2. Coverage-time and coverage-distance curves for **at least three strategies across at least three environments**, with bootstrap confidence intervals over **at least ten seeds each**.
3. An evidence-backed answer to RQ1, including "the ranking depends on environment topology in the following way," which is a positive result.
4. Jetson decision latency reported for every strategy, with an explicit statement of where each becomes infeasible.
5. The report documents failure modes, negative results, and threats to validity.

**None of these require a strategy to "win."** A well-executed null result — "the strategies are statistically indistinguishable here, and this power analysis shows our sample size could have detected a 15% difference" — satisfies every criterion.

---

## 2. Proposed Technical Approach

### 2.1 High-level architecture

```
  Gazebo Harmonic ──► slam_toolbox ──► /map (OccupancyGrid)
   (sensors, odom)                          │
        │ /ground_truth/pose                │
        └──────────────► exploration_node (ours) ◄┘
                          1. frontier detection   (grid → clusters)
                          2. candidate scoring    (STRATEGY PLUGIN — the only variable)
                          3. commitment / hysteresis filter
                          4. goal dispatch + watchdog
                          5. termination check
                                   │ NavigateToPose (action)
                                   ▼
                          Nav2  (planner · controller · recoveries)
                                   │ /cmd_vel
                                   ▼
                             Gazebo robot

  metrics_recorder (ours) — subscribes to /map, both pose sources, and internal
                            exploration events → CSV per tick + JSON per episode
```

**Hardware-in-the-loop for RQ4.** Gazebo stays on a workstation; the *entire ROS 2 stack* — `slam_toolbox`, Nav2, and our exploration node — runs on the Jetson Orin Nano over the local network, so every latency number comes from the real embedded target rather than a workstation pretending to be one.

### 2.2 Components: adopted vs. written

| Layer | Choice | Ours? |
|---|---|---|
| Simulator | Gazebo Harmonic (LTS) — paired with Jazzy, headless batch, free ground truth | adopt |
| Middleware | ROS 2 Jazzy Jalisco (LTS) — native on Ubuntu 24.04, the base for JetPack 7.x | adopt |
| Robot | TurtleBot 4 model — official Gazebo/Jazzy stack; 2D LiDAR suffices | adopt |
| SLAM | `slam_toolbox`, asynchronous — publishes a standard `OccupancyGrid` | adopt |
| Navigation | Nav2 — planning, control, recoveries via `NavigateToPose` | adopt |
| **Exploration** | frontier detection, four strategy plugins, commitment logic | **ours** |
| **Evaluation** | ground-truth extraction, metrics recorder, batch runner, statistics, figures | **ours** |

Roughly **70% of the code we write is in the last two rows** — enough implementation to be real engineering, not so much that we reimplement a navigation stack and run out of semester before the experiment starts.

### 2.3 Frontier detection

Per cycle: (1) identify frontier cells — free cells with at least one unknown 8-neighbour; (2) reject cells whose distance to the nearest obstacle is below the robot's inscribed radius, using a distance transform, preventing goals the planner can never reach; (3) cluster survivors by connected components; (4) discard clusters below `min_frontier_cells`, the primary defence against never terminating; (5) emit a centroid, size, and goal pose per cluster, headed into the unknown. Complexity is `O(N)` in grid cells, with the previous label image cached for incremental updates.

### 2.4 The four strategies

Let `d(f)` be cost-to-go and `I(f)` estimated information gain at frontier `f`.

| | Strategy | Rule | Purpose |
|---|---|---|---|
| **S0** | Random | uniform over reachable free cells | Calibrates the scale of every other strategy's improvement. Cheapest to implement, most commonly omitted. |
| **S1** | Nearest frontier | `U(f) = −d(f)` | The community default. Implemented **two ways** — Euclidean and true planned path length — with the gap as a sub-experiment, since Euclidean cost is systematically wrong whenever a wall intervenes. |
| **S2** | Max information gain | `U(f) = I(f)` | Two estimators reported separately. **Count:** unknown cells within range `R` — cheap, ignores occlusion. **Raycast:** `K` rays from `f` to `R`, each stopping at the first occupied cell — respects occlusion, costs `O(K·R)` per candidate, and is the estimator profiled in RQ4. |
| **S3** | Cost–utility | `U(f) = I(f) − λ·d(f)` | `I` normalized to `[0,1]` per iteration so λ is interpretable across environments. **S1 and S2 are the limiting cases** as λ → ∞ and λ → 0. |

**S4 — RRT-based frontier detection** is a stretch goal only if Week 11 arrives on schedule.

**Commitment / hysteresis, applied identically to all strategies.** Naive per-cycle replanning makes the robot oscillate between near-tied candidates and waste enormous time. We add a bonus β to the currently held goal and reconsider only when the robot is within `r_commit` of it, when new map data invalidates it, or when the watchdog fires.

### 2.5 Intended technical modifications and advanced capabilities

Three things distinguish this from configuring an existing package.

**(a) The λ sweep turns four algorithms into a design space.** Sweeping `λ ∈ {0, 0.25, 0.5, 1, 2, 4, ∞}` reframes the question from "which of these four is best" to "what does the cost–information weighting actually do, and does its optimum transfer across environments." This is the single most informative figure we expect to produce, and it makes the study a characterization rather than a bake-off.

**(b) Controlled experimental design at a sample size the field does not use.** Randomized start poses, per-seed pairing across strategies, bootstrap confidence intervals, Wilcoxon signed-rank tests with Holm–Bonferroni correction, and effect sizes rather than bare p-values. The largest sample size in any comparable recent paper our survey found was ten trials per scenario; we run twenty seeds per cell in the primary matrix. Metrics and the statistical plan are **frozen at the end of Week 8, before the main experiment runs**, to prevent post-hoc metric selection.

**(c) Embedded latency as a first-class result.** Every surveyed SOTA planner treats computation as a design constraint — TARE restructures the representation for it, PIPE headlines an 83.3% computation reduction, EPIC abandons the volumetric grid for memory reasons, and Lewis et al. (2026) make frontier-detection complexity the contribution. None reports where a strategy stops meeting a real-time budget on a 15 W SoC. We measure that curve directly, then attempt to push the crossing point out with an optimized implementation (incremental frontier updates, hierarchical coarse-to-fine scoring, or a GPU kernel) and report before/after.

### 2.6 Data sources

There is **no training dataset** — this is a systems and evaluation project, not a learning one. Our data is the environments and the episode logs they generate.

| ID | Environment | Property under test | Source |
|---|---|---|---|
| E1 | Small house / apartment | Many rooms and doorways; high frontier count, short distances | AWS RoboMaker Small House (open source) |
| E2 | Warehouse | Large, structured, long aisles; proximity and information decoupled | AWS RoboMaker Small Warehouse (open source) |
| E3 | Maze | Deceptive Euclidean-vs-path distance; high branching factor | Procedural (ours) |
| E4 | Cluttered open hall | Frontiers on all sides; greedy should degrade | Procedural, parameterized obstacle density (ours) |
| E5 | Loop-heavy corridors | Varies loop-closure opportunity, for RQ3 | Procedural (ours) |

The AWS worlds were authored for Gazebo Classic, so Week 1 budgets time for SDF compatibility work. Generating E3–E5 from a Python script gives parameterized difficulty at negligible cost.

**Generated dataset, released with the code:** ~1,330 episode logs (CSV + JSON).

| Experiment | Design | Runs |
|---|---|---|
| Primary (RQ1, RQ2) | 4 strategies × 5 envs × 20 seeds, ground-truth localization | 400 |
| Secondary (RQ3) | identical, with `slam_toolbox` in the loop | 200 |
| λ sweep (RQ2) | 7 λ × 3 envs × 10 seeds | 210 |
| Sensor range (RQ1) | 3 ranges × 2 envs × 3 strategies × 10 seeds | 180 |
| Commitment β (RQ5) | 4 β × 3 envs × 2 strategies × 10 seeds | 240 |
| Euclidean vs. path cost (S1) | 2 variants × 5 envs × 10 seeds | 100 |
| **Total** | | **≈ 1,330** |

At 6–10 min of simulated time per episode and a headless real-time factor of 2–4×, that is roughly 45–90 machine-hours single-stream — four parallel Gazebo instances across two machines over three nights. **Pre-defined fallback:** if the harness is slower than estimated, drop to 10 seeds and 3 environments (−60%), decided at the start of Week 9 and not later.

Running the primary condition with ground-truth localization is what lets us attribute RQ3's map-quality differences to the exploration policy rather than to SLAM.

---

## 3. Device Availability and Maintainer

### 3.1 Devices

| Device | Spec | Availability | Used for |
|---|---|---|---|
| **NVIDIA Jetson Orin Nano Super 8 GB devkit** | 8 GB unified LPDDR5 shared CPU/GPU; 7 W / 15 W / 25 W modes; JetPack 7.x on Ubuntu 24.04 | **One per member, borrowed from Prof. Liu** | RQ4: hardware-in-the-loop deployment of `slam_toolbox` + Nav2 + our exploration node; latency, memory, power profiling |
| Team development workstations | x86-64 Linux, discrete GPU | Personal machines | Gazebo host, headless batch runs, analysis and figures |

**Why a board per member matters.** JetPack/ROS dependency problems are a known time sink and our classic schedule risk. With three boards, bring-up runs in parallel from Week 1 instead of serialized behind one shared device, a mis-flashed board does not stop the study, and the Week 12 profiling can be split three ways across power modes.

**Memory reality.** The 8 GB is *unified* memory shared between CPU and GPU; JetPack and the ROS stack consume 1.5–2 GB, leaving roughly 6 GB usable. A 100 × 100 m world at 5 cm resolution is 4 M cells, and naive Python frontier processing with intermediate array copies will hit that ceiling. Writing memory-conscious code is part of the intended learning outcome, and the ceiling is itself a reportable RQ4 result. **No training hardware is required** — nothing in this project trains a model, so the 8 GB is an inference and real-time-execution budget rather than a constraint to work around.

### 3.2 Maintainers

| Responsibility | Owner(s) |
|---|---|
| **Jetson boards, JetPack flashing, containers, profiling harness** | Shervan Shahparnia and Aaron Sam |
| Simulation, worlds, batch runner, CI | Shervan Shahparnia and Joey Manivong |
| Exploration algorithms and strategy plugins | Joey Manivong and Aaron Sam |
| **Repository maintainer / point of contact** | Shervan Shahparnia — https://github.com/SShahparnia/cmpe249_project |
| Hardware loaned by | Prof. Liu |

Every subsystem has two owners and every member owns two subsystems, so no component has a single point of failure and every interface has someone on both sides of it.

### 3.3 Agentic tooling (Claude Code)

We intend to use Claude Code as a development assistant in this repository, run from each member's own workstation against a normal working copy — not on the Jetson, and not in any autonomous or unattended mode.

**Where it helps:** ROS 2 boilerplate (package scaffolding, launch files, parameter YAML), the batch runner and log-parsing scripts, plotting and statistics code, and test scaffolding for the pure-Python frontier-detection core.

**Where we will not use it.** Not for the experimental design, metric definitions, statistical plan, or interpretation of results — those are the graded intellectual content and must be ours. Every change is a reviewed pull request with a human author who can explain it, and anything touching the frozen metrics or analysis code after Gate G2 (end of Week 8) requires sign-off from both evaluation owners, since silently rewritten analysis code is exactly the failure mode pre-registration exists to prevent.

---

## Key references

Full annotated survey in [`LITERATURE_SURVEY.md`](LITERATURE_SURVEY.md).

Brugali et al. (2025), IJRR · Placed et al. (2023), IEEE T-RO · Cao et al. (2023), Science Robotics · Ho et al. (2025), ICRA · Baek et al. (2025), arXiv:2503.07504 · Liu et al. (2025), Scientific Reports 15:12261 · Geng et al. (2025), IEEE RA-L · Lewis et al. (2026), arXiv:2604.03008 · Yamauchi (1997); González-Baños & Latombe (2002); Burgard et al. (2005); Holz et al. (2010) as method sources for S1–S3.

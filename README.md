# ExploreBench

**A Controlled Comparison of Frontier-Selection Strategies for Autonomous Exploration and Mapping, with Embedded Deployment on the NVIDIA Jetson Orin Nano**

CMPE 249 Intelligent Autonomous Systems, Fall 2026

**Repository:** https://github.com/SShahparnia/cmpe249_project

---

## Team members

- **Shervan Shahparnia**: simulation and infrastructure; evaluation and Jetson deployment
- **Joey Manivong**: simulation and infrastructure; exploration algorithms
- **Aaron Sam**: exploration algorithms; evaluation and Jetson deployment

Each subsystem has two owners and each member owns two subsystems, so no component has a single point of failure.

---

## Selected track

**System** (primary). Integration of autonomy components with ROS 2, Nav2, `slam_toolbox`, Gazebo, and a Jetson target, evaluated as a real-time pipeline.

**Deployment** (secondary). Decision latency, memory, and power characterization of each strategy on the Jetson Orin Nano across its 7 W / 15 W / 25 W modes.

---

## Abstract

Autonomous exploration, which means mapping an unknown environment with no human in the loop, underpins search-and-rescue robots, warehouse inventory systems, and inspection drones. The decision such a robot makes repeatedly is *where to go next*. Decades of work have produced a family of frontier-selection strategies for that decision, but the comparative literature is fragmented: results are reported on different simulators, robots, and metrics, often on a handful of runs with no variance estimates. Practitioners default to nearest-frontier greedy selection largely because it is what the standard open-source packages implement.

We build a reproducible benchmarking harness for 2D autonomous exploration in ROS 2 Jazzy and Gazebo Harmonic, implement four frontier-selection strategies behind one common interface, and run a controlled study of roughly 1,300 simulated episodes across five environment topologies with randomized start poses. We report coverage-over-time and coverage-over-distance with bootstrap confidence intervals, measure whether exploration speed trades against map quality when real SLAM is in the loop, characterize the cost-utility weight λ as a continuous design knob rather than four discrete algorithms, and profile every strategy's decision latency, memory, and power on a Jetson Orin Nano running the full stack hardware-in-the-loop.

Our contribution is not a new exploration algorithm. It is a rigorous, reproducible, embedded-aware comparison of existing ones, plus an open-source harness that makes future comparisons cheap.

---

## Assignment 1 deliverables

| Req. | Deliverable | Location |
|---|---|---|
| **A** | GitHub repo and README (title, team, abstract, track) | this file |
| **B** | Literature and SOTA survey: 10 papers, 2023 to 2026 | [`docs/LITERATURE_SURVEY.md`](docs/LITERATURE_SURVEY.md) |
| **C** | Project proposal document (submitted on Canvas) | [`docs/PROJECT_PROPOSAL.md`](docs/PROJECT_PROPOSAL.md) · `CMPE249_Assignment1_Proposal.docx` |
| **D** | AI novelty and feasibility audit | [`docs/AI_NOVELTY_AUDIT.md`](docs/AI_NOVELTY_AUDIT.md) |

---

## Quick start

Target stack: **Ubuntu 22.04, ROS 2 Humble, Ignition Gazebo Fortress, TurtleBot 4** on the laptop; **JetPack 6 (Ubuntu 22.04), ROS 2 Humble** on the Jetson. See [`docs/sim_setup.md`](docs/sim_setup.md) for why it is Humble and not Jazzy. Build instructions per module are in [`instructions.md`](instructions.md).

### 1. Clone (both machines)

```bash
git clone --recursive https://github.com/SShahparnia/cmpe249_project.git
cd cmpe249_project
scripts/setup_git_hooks.sh      # once per clone: enables .githooks/ (strips AI attribution from commits)
scripts/apply_patches.sh        # our patches on top of the pinned submodules in methods/
```

### 2. Install

```bash
bash scripts/laptop/setup_laptop.sh     # laptop: ROS 2 Humble desktop, TB4 sim, slam_toolbox, Nav2, CycloneDDS
bash scripts/jetson/setup_jetson.sh     # Jetson: ROS 2 Humble base, slam_toolbox, Nav2, CycloneDDS, jtop, stress-ng
```

Both add `source <repo>/scripts/ros_env.sh` to `~/.bashrc` (sets `ROS_DOMAIN_ID=49`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`). Jetson flashing and power-mode setup: [`docs/jetson_setup.md`](docs/jetson_setup.md).

### 3. Full stack on the laptop (default TB4 world)

```bash
# terminal 1: sim + SLAM + Nav2 + RViz
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
# terminal 2: drive, and watch the map grow in RViz
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# terminal 3: real-time factor -> docs/sim_setup.md
python3 scripts/measure_rtf.py --seconds 60
```

### 4. Laptop <-> Jetson link

```bash
bash scripts/set_static_ip.sh <iface> laptop    # on the laptop  (192.168.249.1)
bash scripts/set_static_ip.sh <iface> jetson    # on the Jetson  (192.168.249.2)
# then on both: add  export EB_IFACE=<iface>  to ~/.bashrc above the ros_env.sh line
```

### 5. SLAM on the Jetson against the laptop sim (Block E layout)

```bash
# laptop: sim only
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=false nav2:=false rviz:=true
# Jetson: link checks, then SLAM with sim time
bash scripts/jetson/link_test.sh
ros2 launch turtlebot4_navigation slam.launch.py use_sim_time:=true
# laptop: drive with teleop; the /map from the Jetson should grow in RViz
```

### Repository layout

```
methods/      planner submodules pinned to one commit each; our changes go in methods/patches/
harness/      ROS 2 packages: recorder, timing msgs, launch files
maps/         world files + ground-truth grids
configs/      start poses, per-method params, common params
analysis/     offline scripts
scripts/      setup helpers, batch runners
docs/         setup notes, log format, run log, provenance
logs/         run outputs (git-ignored)
```

Pinned planners:

| Submodule | Repo | Branch | Commit |
|-----------|------|--------|--------|
| `methods/m-explore-ros2` | robo-friends/m-explore-ros2 | main | `326cf8a` |
| `methods/frontier_exploration_ros2` | mertgulerx/frontier_exploration_ros2 | main | `11864e4` |
| `methods/ARiADNE` | marmotlab/ARiADNE | new_version | `ea8bc6a` |
| `methods/ARiADNE-ROS-Planner` | marmotlab/ARiADNE-ROS-Planner | humble (ROS 2 port) | `c41b4c9` |
| `methods/roadmap-explorer` | suchetanrs/roadmap-explorer | main (Humble tested) | `203ff26` |

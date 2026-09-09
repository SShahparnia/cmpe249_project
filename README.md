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

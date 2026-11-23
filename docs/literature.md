# Literature notes (Module 4)

Owner: Aaron (all read). Status: **draft, 2026-11-23.**

This table is seeded from [LITERATURE_SURVEY.md](LITERATURE_SURVEY.md), the README files of the pinned baseline repos, and the published benchmark. Rows 1 to 10 are the ten survey papers. Row 11 is ARiADNE, added because it is baseline C. `instructions.md` says "11 works in proposal Section 2", but the survey lists 10 (plus four classic method sources), so **confirm the list as a team**.

Every cell marked **TODO (full text)** has to be filled by whoever reads that paper in full. Module 4 step 2 says to read the full paper, not the abstract, and the survey was written before that reading. Do not copy numbers into this file from memory; copy them from the paper with a page or table reference.

Split (suggested, change freely): Aaron 1, 6, 9, 10, 11 · Joey 3, 4, 5 · Shervan 2, 7, 8.

## Table

| # | Paper | Claim | Environments | # runs | How compute is reported | Hardware | Code link | Notes |
|---|-------|-------|--------------|--------|-------------------------|----------|-----------|-------|
| 1 | Brugali, Muratore, De Luca 2025, IJRR | Systematic mapping study: exploration papers use incompatible simulators, robots and metrics | n/a (survey) | n/a | n/a | n/a | none | Motivation for a controlled comparison. Trade-off on embedded board? n/a (survey). |
| 2 | Placed et al. 2023, IEEE T-RO 39(3) | Active-SLAM survey; frontier / information-theoretic / learning taxonomy | n/a (survey) | n/a | n/a | n/a | none | Taxonomy for related work. Trade-off on embedded board? n/a (survey). |
| 3 | Cao et al. 2023, Science Robotics 8(80) (TARE) | Multi-resolution representation keeps exploration fast in large worlds | TODO (full text) | TODO | TODO | TODO | github.com/caochao39/tare_planner | Restructures representation for compute. Trade-off on embedded board? TODO. |
| 4 | Ho et al. 2025, ICRA (MapEx) | Map prediction gives a better information-gain estimate; +25.4% vs nearest frontier, +12.4% vs another prediction method (survey) | KTH floorplans (survey) | TODO | TODO | TODO | github.com/castacks/MapEx | Gives a scale for nearest frontier. Trade-off on embedded board? TODO. |
| 5 | Baek et al. 2025, arXiv 2503.07504 (PIPE) | Pathwise information gain + map prediction; 83.3% compute reduction on large maps (survey) | TODO | TODO | TODO | TODO | none found | Baselines Nearest / NBV-2D are close to ours. Trade-off on embedded board? TODO. |
| 6 | Liu et al. 2025, Sci. Rep. 15:12261 | Map optimization (bilateral filter + dilation) + multi-term frontier cost ordered as MRTSP | Corridor, maze, cluttered (sim) + real wheeled robot (survey) | 10 per scenario (survey) | TODO | TODO | none (third-party: frontier_exploration_ros2) | **Baseline B source.** Paper-to-code table: [mrtsp_mapping.md](mrtsp_mapping.md). Trade-off on embedded board? TODO. |
| 7 | Calzolari et al. 2025, arXiv 2504.11907 | GNN + safety shield greedy policy; ~95% coverage in 2,000 steps over 1,000 sim envs (survey) | Procedurally generated outdoor (survey) | 1,000 envs (survey) | TODO | TODO | none | Learning-based family. Trade-off on embedded board? TODO. |
| 8 | Zhan et al. 2025, IEEE RA-L | Semantic + geometric exploration with panoramic LiDAR-camera fusion | Isaac Sim + Spot with Ouster OS-1-128 (survey) | TODO | TODO | TODO | none | Recovery state machine as failure warning. Trade-off on embedded board? TODO. |
| 9 | Geng et al. 2025, IEEE RA-L (EPIC) | Drops the volumetric grid; point-cloud observation map + topological graph for large-scale aerial exploration | TODO | TODO | TODO | TODO | github.com/Robotics-STAR-Lab/EPIC | Representation is the compute cost. Trade-off on embedded board? TODO. |
| 10 | Lewis, Basiri, Lima 2026, arXiv 2604.03008 | Frontier detection inside the sensor model, O(\|F\|); GP information gain | TODO | TODO | Complexity is the headline (survey) | Sim: HPC node 32 GB; flight: Intel NUC i7 (survey) | none | Closest precedent for RQ4, but not embedded. Trade-off on embedded board? **No** per survey; confirm. |
| 11 | Cao et al. 2023, ICRA (ARiADNE); Cao et al. 2024, RA-L | Attention-based DRL picks the next waypoint on an informative graph | Training maps from Chen et al. (methods/ARiADNE/maps); ROS demo: CMU indoor / forest / tunnel | TODO | ROS demo: average planning time 0.17 s (README) | ROS demo: Intel i7-12700H mini PC, CPU inference (README) | github.com/marmotlab/ARiADNE, ARiADNE-ROS-Planner | **Baseline C.** Trade-off on embedded board? TODO. |

Step 3 rule: if any paper reports exploration quality **and** compute or energy on an embedded board as a trade-off, post it to the team the same day.

## Gap statement re-check (step 3)

Current gap statement (from the survey): none of the ten papers runs a controlled multi-strategy comparison, and none reports where a strategy stops meeting a real-time budget on a 15 W-class embedded SoC.

- [ ] Re-checked against the full texts by: TODO (who, date). Confirmed / rewritten as: TODO.

## Target numbers for each baseline (step 4)

Write these down **before** reproduction starts (Modules 5 to 7). The published numbers below are copied from the repos as of the pinned commits; paper numbers still need the full text.

### A. nearest (m-explore-ros2)

Source: the `frontier_exploration_ros2` README benchmark table (submodule @ 11864e4, README "Benchmark"), which runs the shared [autonomous-exploration-demo-benchmark](https://github.com/mertgulerx/autonomous-exploration-demo-benchmark).

| Environment | Metric | Published | # runs | Spread |
|-------------|--------|-----------|--------|--------|
| Bookstore (225 m²) | distance traveled | 50.73 m | TODO (benchmark repo) | not reported in README |
| Bookstore | time elapsed | 96 s | TODO | not reported |
| Warehouse (1,500 m²) | distance traveled | 338.61 m | TODO | not reported |
| Warehouse | time elapsed | 587 s | TODO | not reported |

**Caution:** the benchmark ran `m_explore_ros2` with its own parameters, presumably the package config (`gain_scale: 1.0`), not our nearest config (`gain_scale: 0.0`). To reproduce the published number, run the benchmark config. Then run our nearest config separately and report both. VERIFY which parameters the benchmark repo used.

### B. mrtsp (frontier_exploration_ros2)

| Source | Environment | Metric | Published | # runs | Spread |
|--------|-------------|--------|-----------|--------|--------|
| repo benchmark, greedy MRTSP | Bookstore | distance / time | 36.60 m / 63 s | TODO | not reported |
| repo benchmark, greedy MRTSP | Warehouse | distance / time | 273.52 m / 499 s | TODO | not reported |
| repo README, DP solver | Warehouse (presumably) | distance / time | 263.72 m / 442 s | TODO | not reported |
| Liu et al. 2025 | corridor / maze / cluttered | TODO (full text) | TODO | 10 per scenario | TODO |

### C. ariadne (ARiADNE / ARiADNE-ROS-Planner)

| Source | Environment | Metric | Published | # runs | Spread |
|--------|-------------|--------|-----------|--------|--------|
| ARiADNE paper (ICRA 2023) | authors' test set | mean trajectory length | TODO (full text) | TODO | TODO |
| ARiADNE-ROS-Planner README | CMU indoor | distance / time to completion | 948 m / 545 s | 1 (demo) | n/a |
| ARiADNE-ROS-Planner README | CMU indoor | average planning time | 0.17 s on i7-12700H | 1 (demo) | n/a |

- [ ] All 11 rows filled.
- [ ] Gap statement re-checked and either confirmed or rewritten.
- [ ] Target numbers for each baseline written down before reproduction starts.

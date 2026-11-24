<!--
Report template (Module 22). Do not type numbers by hand: every number is a double-brace placeholder
(a numbers.json path and an optional format, see analysis/fill_report.py), filled by `python3 analysis/fill_report.py` after `python3 analysis/make_figures.py`.
Edit the prose here; never edit docs/report/report.md (it is generated). Prose marked TODO is written
once the results are in. Paths below assume Block E power modes 7W and MAXN_SUPER: rename the keys to
the modes chosen in docs/power_modes.md.
-->

# ExploreBench: Is Information-Aware Frontier Selection Worth Its Compute on an Embedded Robot?

Shervan Shahparnia, Joey Manivong, Aaron Sam · CMPE 249, Fall 2026

## Abstract

TODO (written last). State the question, the setup (3 planners, 3 maps, TurtleBot 4 in Gazebo, Jetson Orin Nano), and the three findings with their numbers: Q1 (`d_95` difference), Q2 (latency, energy), Q3 (break-even `k*`).

## 1. Research question

Does choosing frontiers with information-aware planners (MRTSP, ARiADNE) instead of nearest-frontier make exploration shorter enough to pay for their extra computation, when that computation runs on an embedded board? Figure 1 is the answer in one picture.

![Figure 1: d_95 vs. planner energy above idle, Block E](../../analysis/out/fig_q3_pareto.png)

Pre-registered hypotheses (`docs/preregistration.md`, tag `prereg-v1`): H1 (quality), H2 (compute grows with the number of frontiers), H3 (trade-off).

## 2. Related work

TODO, from `docs/literature.md` (11 works). The gap: no surveyed work reports exploration quality and embedded compute/energy as one trade-off.

## 3. Methods compared

Three planners behind one harness, each the authors' or a maintained implementation pinned to one commit (`docs/baseline_provenance.md`): `nearest` (m-explore-ros2, `gain_scale = 0`), `mrtsp` (frontier_exploration_ros2, greedy MRTSP, a third-party implementation of Liu et al. 2025, mapped in `docs/mrtsp_mapping.md`), `ariadne` (ARiADNE-ROS-Planner with our thin Nav2 adapter). Common rules in `docs/common_rules.md`: same SLAM, Nav2, robot, sensors, replanning rate and timeout for every method; each method keeps its own goal-commitment logic.

## 4. Experimental protocol

- Maps: M1 AWS small house, M2 office corridor (ours), M3 AWS small warehouse, converted to Ignition Fortress; ground truth rasterized from the world geometry (`maps/`).
- Start poses: 20 per map, seed 249, ≥ 0.35 m clearance, ≥ 1.5 m apart (`configs/start_poses/`).
- Blocks: Q (quality, laptops): {{inputs.runs_per_block.Q|int}} runs; E (energy, Jetson): {{inputs.runs_per_block.E|int}} runs; invalid runs rerun: {{inputs.invalid_runs|int}}.
- Metrics and statistics exactly as pre-registered: `d_95`, energy above idle, decision latency; bootstrap CI of the median paired difference, Wilcoxon, Holm across maps.

## 5. Results

### 5.1 Q1: exploration quality (Block Q)

![Figure 2: coverage vs. distance](../../analysis/out/fig_q1_coverage.png)

| map | MRTSP vs nearest: median Δd_95 (95% CI) | rel. | Holm p | ARiADNE vs nearest: median Δd_95 (95% CI) | rel. | Holm p |
|-----|----|----|----|----|----|----|
| M1 house | {{q1.mrtsp.m1_house.median_diff|f1}} m ({{q1.mrtsp.m1_house.ci_low|f1}}, {{q1.mrtsp.m1_house.ci_high|f1}}) | {{q1.mrtsp.m1_house.median_rel|pct}} | {{q1.mrtsp.m1_house.p_holm|p}} | {{q1.ariadne.m1_house.median_diff|f1}} m ({{q1.ariadne.m1_house.ci_low|f1}}, {{q1.ariadne.m1_house.ci_high|f1}}) | {{q1.ariadne.m1_house.median_rel|pct}} | {{q1.ariadne.m1_house.p_holm|p}} |
| M2 office | {{q1.mrtsp.m2_office.median_diff|f1}} m ({{q1.mrtsp.m2_office.ci_low|f1}}, {{q1.mrtsp.m2_office.ci_high|f1}}) | {{q1.mrtsp.m2_office.median_rel|pct}} | {{q1.mrtsp.m2_office.p_holm|p}} | {{q1.ariadne.m2_office.median_diff|f1}} m ({{q1.ariadne.m2_office.ci_low|f1}}, {{q1.ariadne.m2_office.ci_high|f1}}) | {{q1.ariadne.m2_office.median_rel|pct}} | {{q1.ariadne.m2_office.p_holm|p}} |
| M3 warehouse | {{q1.mrtsp.m3_warehouse.median_diff|f1}} m ({{q1.mrtsp.m3_warehouse.ci_low|f1}}, {{q1.mrtsp.m3_warehouse.ci_high|f1}}) | {{q1.mrtsp.m3_warehouse.median_rel|pct}} | {{q1.mrtsp.m3_warehouse.p_holm|p}} | {{q1.ariadne.m3_warehouse.median_diff|f1}} m ({{q1.ariadne.m3_warehouse.ci_low|f1}}, {{q1.ariadne.m3_warehouse.ci_high|f1}}) | {{q1.ariadne.m3_warehouse.median_rel|pct}} | {{q1.ariadne.m3_warehouse.p_holm|p}} |

Censored pairs (timeout or stuck, total distance used): MRTSP M1 {{q1.mrtsp.m1_house.n_censored|int}}, M2 {{q1.mrtsp.m2_office.n_censored|int}}, M3 {{q1.mrtsp.m3_warehouse.n_censored|int}}; ARiADNE M1 {{q1.ariadne.m1_house.n_censored|int}}, M2 {{q1.ariadne.m2_office.n_censored|int}}, M3 {{q1.ariadne.m3_warehouse.n_censored|int}}.

TODO: interpretation (H1 per map and method).

### 5.2 Q2: compute and energy on the Jetson (Block E)

![Figure 3: decision latency](../../analysis/out/fig_q2_latency.png)

| method | p95 latency, M1, 7W | p95 latency, M1, MAXN_SUPER | deadline misses, M1, 7W | energy above idle, M1, 7W |
|--------|----|----|----|----|
| nearest | {{q2.nearest.m1_house.7W.latency_p95_s|ms}} | {{q2.nearest.m1_house.MAXN_SUPER.latency_p95_s|ms}} | {{q2.nearest.m1_house.7W.deadline_miss_rate|pct}} | {{q2.nearest.m1_house.7W.energy_above_idle_J_median|f0}} J |
| mrtsp | {{q2.mrtsp.m1_house.7W.latency_p95_s|ms}} | {{q2.mrtsp.m1_house.MAXN_SUPER.latency_p95_s|ms}} | {{q2.mrtsp.m1_house.7W.deadline_miss_rate|pct}} | {{q2.mrtsp.m1_house.7W.energy_above_idle_J_median|f0}} J |
| ariadne | {{q2.ariadne.m1_house.7W.latency_p95_s|ms}} | {{q2.ariadne.m1_house.MAXN_SUPER.latency_p95_s|ms}} | {{q2.ariadne.m1_house.7W.deadline_miss_rate|pct}} | {{q2.ariadne.m1_house.7W.energy_above_idle_J_median|f0}} J |

(Full table for every map: `analysis/out/tables/block_e_energy.csv`.)

### 5.3 H2: does decision time grow with the number of frontiers?

![Figure 4: latency vs. n_frontiers](../../analysis/out/fig_h2_scaling.png)

Log-log slope of latency on `n_frontiers` (95% cluster-bootstrap CI), low power mode: nearest {{h2.nearest.7W.slope|f2}} ({{h2.nearest.7W.ci_low|f2}}, {{h2.nearest.7W.ci_high|f2}}), MRTSP {{h2.mrtsp.7W.slope|f2}} ({{h2.mrtsp.7W.ci_low|f2}}, {{h2.mrtsp.7W.ci_high|f2}}), ARiADNE {{h2.ariadne.7W.slope|f2}} ({{h2.ariadne.7W.ci_low|f2}}, {{h2.ariadne.7W.ci_high|f2}}). `n_frontiers` counts clusters for nearest and MRTSP and frontier cells for ARiADNE, so slopes are compared within a method.

### 5.4 Q3: the trade-off

![Figure 5: mission energy vs. k_loco](../../analysis/out/fig_q3_break_even.png)

Break-even `k*` on M1 at 7W: MRTSP {{q3.break_even.m1_house.7W.mrtsp.k_star_J_per_m|f2}} J/m (CI {{q3.break_even.m1_house.7W.mrtsp.k_star_ci.0|f2}} to {{q3.break_even.m1_house.7W.mrtsp.k_star_ci.1|f2}}), ARiADNE {{q3.break_even.m1_house.7W.ariadne.k_star_J_per_m|f2}} J/m. Information-aware selection pays off when the robot's locomotion cost exceeds `k*`; a negative `k*` means it also used less board energy, so it wins at any `k_loco` (MRTSP dominates on M1 at 7W: {{q3.break_even.m1_house.7W.mrtsp.dominates}}). TODO: compare with the cited `k_loco` range for a TurtleBot-class base.

## 6. Threats to validity

1. **Simulated locomotion.** Distance comes from Gazebo; real wheel slip, battery state and motor efficiency are not modelled, and `k_loco` is modelled, not measured (next item).
2. **Modelled `k_loco`.** Mission energy uses a locomotion cost per metre from the literature (TODO: source), swept over a range rather than measured on our robot.
3. **Third-party MRTSP.** We evaluate this implementation of MRTSP (frontier_exploration_ros2, greedy solver), not the paper's code. Every difference from Liu et al. 2025 is listed in `docs/mrtsp_mapping.md`.
4. **Single board.** One Jetson Orin Nano unit, two power modes, one fan profile, no `jetson_clocks`; unit-to-unit variation is not measured.
5. **Gazebo version and world port.** ROS 2 Humble with Ignition Gazebo Fortress (not Jazzy/Harmonic as first proposed). The AWS worlds were written for Gazebo Classic and converted by us (`maps/tools/convert_aws_world.py`); the lidar is a GPU ray sensor that sees visual geometry, so ground truth uses the visual meshes at lidar height.
6. **Simulation speed and sensors.** Sensor setup (`docs/common_rules.md`) and the measured real-time factor of every run (manifests); nearest's planning timer runs on the wall clock, so at RTF < 1 it replans more often per simulated second.
7. **Start condition.** The TB4 sim spawns the robot docked; the dock is removed before the run (`docs/common_rules.md` rule 5).
8. TODO: anything that changed after `prereg-v1` (deviations), with reasons.

## 7. Conclusion

TODO.

## Appendix A. Architecture

Harness (`harness/run.py`): sim → SLAM + Nav2 → recorder → planner, one process group each; outcomes decided by the monitor (`docs/log_format.md` §0.5). Block E layout: Gazebo on the laptop, everything else on the Jetson. Run data: `logs/` archive (link and checksum in the README).

## Appendix B. Reproducing every number

```bash
python3 analysis/make_figures.py && python3 analysis/fill_report.py
```

Inputs: {{inputs.n_runs|int}} runs, logs digest `{{inputs.logs_digest}}`.

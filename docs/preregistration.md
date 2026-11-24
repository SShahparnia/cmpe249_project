# Pre-registration (Module 18)

> **DRAFT, not frozen.** Before tagging `prereg-v1`:
> 1. replace every `PILOT:` value with the number from `docs/pilot_report.md` (and the same value in `configs/common.yaml`);
> 2. replace the hypothesis wording in Section 1 with the exact text of the final proposal (`plan.md` is not in this repo; the text below is reconstructed from `instructions.md`);
> 3. settle the TEAM DECISIONs listed in Section 9;
> 4. all three members reply "acknowledged" in the team channel, then tag (Section 10).
>
> After the tag, nothing in this file, `configs/`, `maps/`, `analysis/` (statistics code) or the method patches changes. Any later change is a deviation, reported as such.

## 1. Hypotheses and the test for each

| | Hypothesis (VERIFY wording against the proposal) | Data | Test that confirms or rejects it |
|---|---|---|---|
| **H1** (quality) | Information-aware frontier selection (`mrtsp`, `ariadne`) explores with a shorter path than nearest-frontier: lower `d_95` than `nearest` from the same start pose, on each map. | Block Q, laptops | Per map and method: paired differences `d_95[method] − d_95[nearest]` over poses. Two-sided Wilcoxon signed-rank; Holm correction across the 3 maps (separately for each method). H1 holds for a method on a map when the Holm-adjusted p < 0.05 **and** the bootstrap 95% CI of the median difference lies below 0. |
| **H2** (compute) | A planner's decision time grows with the number of frontiers, most steeply for `mrtsp` (O(n²) matrix), and the growth is larger at the low power mode. | Block E, Jetson | Per method and power mode: slope of log(latency) on log(`n_frontiers`) over all planning cycles (`cycles.csv`), with a 95% CI from a cluster bootstrap over runs. H2 holds for a method when the CI excludes 0. Steeper for `mrtsp`: the slope difference CI excludes 0. Low vs high power mode: same, between modes. |
| **H3** (trade-off) | The information-aware methods' extra compute energy is small next to the locomotion energy their shorter paths save, so for a TurtleBot-class base they use less mission energy than `nearest` (break-even `k*` below a realistic `k_loco`). | Block E, Jetson | Per map, method and power mode: `k* = dE / dD` with `dE = E_board[info] − E_board[nearest]`, `dD = d[nearest] − d[info]` (medians over poses; Section 2), and a bootstrap 95% CI over poses. H3 holds when the upper end of the CI of `k*` lies below the smallest realistic `k_loco` for a TurtleBot-class base (`K_LOCO_MIN` in `analysis/q3_tradeoff.py`, with its citation; Section 2). If `dD ≤ 0`, there is no break-even and H3 is rejected for that cell. |

Q1, Q2 and Q3 in the report correspond to H1, H2 and H3.

## 2. Primary metrics (formulas)

| Metric | Formula | Source |
|--------|---------|--------|
| `d_95` | path length (m) from the planner start until GT coverage first reaches 0.95, linearly interpolated between map snapshots. Path length = sum of steps between consecutive 10 Hz poses, ignoring moves under 1 cm from the last counted point. | `analysis/coverage.py` → `summary.json` |
| Coverage | known GT reference cells / all GT reference cells. Reference = reachable free cells + obstacle cells bordering them (`maps/<map>/gt_reachable.npy`). SLAM cell known if value 0 or ≥ 65. | `analysis/coverage.py` |
| Planner energy above idle | `E_run − P_idle · (t1 − t0)`; `E_run` = trapezoid integral of `VDD_IN` over the planner's active window [t0, t1] (manifest `t_wall_start`, `t_wall_end`); `P_idle` = mean `VDD_IN` of the idle-stack baseline of the same session and power mode. | `analysis/energy.py` |
| Decision latency | `wall_end − wall_start` of each `PlanningCycle`; reported as median and p95 per run, then median over runs. | `cycles.csv` |
| Deadline-miss rate | fraction of planning cycles with latency > `replan_period_s`. | `cycles.csv` |
| Mission energy | `E_mission = E_board + k_loco · d`, with `k_loco` swept over **PILOT/TEAM: range and citation for a TurtleBot-class base** (J/m). | `analysis/q3_tradeoff.py` |

## 3. Secondary metrics

`t_95` (s, sim time, same interpolation as `d_95`); final coverage; oscillation count with **X = PILOT: m** (a goal switch farther than X while the current goal was neither reached nor aborted); planner CPU time (last − first `utime + stime` of the planner process in `procs.csv`, s); GPU utilization (mean of `gpu.csv`); per-run real-time factor.

## 4. Termination and outcomes

Every run ends with exactly one outcome (`docs/log_format.md` Section 0.5), decided by `harness/run.py` from `configs/common.yaml`:

| Outcome | Rule | Counts as data? |
|---------|------|-----------------|
| `complete` | live coverage ≥ 0.95 (run continues 5 s sim, then stops) | yes |
| `timeout` | sim time since the planner start > `timeout_sim_s` = **PILOT: s** | yes (censored) |
| `stuck` | planner exited, or no planning cycle for 60 s sim time, before 0.95 | yes (censored) |
| `invalid` | sim / SLAM / Nav2 / recorder / power logger died, `/clock` stalled 10 s wall, impossible pose jump (> 1 m between samples), Nav2 not ready | no: rerun (Section 8) |

A run is never marked `invalid` because the method did badly.

## 5. Timeouts and stuck runs in the statistics

For the paired tests, a censored run's `d_95` is replaced by its **total distance** at the end of the run (`d_95_or_total` in `analysis/load.py`) and the pair is flagged. This is conservative for the censored method (its true `d_95` is at least that long). The number of censored runs per (method, map, power mode) cell is reported next to every test. `t_95` is treated the same way with total time.

## 6. Run counts

| Block | Where | Methods | Maps | Poses (indices) | Power modes | Runs |
|-------|-------|---------|------|-----------------|-------------|------|
| P (pilot) | laptops | 3 | 3 | 0–2 | `laptop` | 27, not part of the results |
| Q (quality) | laptops | 3 | 3 | 0–9 (or 0–14, **PILOT decision**) | `laptop` | 90 (or 135) |
| E (energy) | Jetson (Gazebo on the laptop) | 3 | 3 | 0–4 | lowest and highest from `docs/power_modes.md`: **TODO names** | 90 |
| A (ablation, optional) | Jetson | 3 | `m2_office_2x` | 0–4 | high (and low if time) | 15 (30) |

Poses come from `configs/start_poses/<map>.yaml` (seed 249, frozen). Run order is shuffled with a fixed seed (`scripts/make_schedule.py --seed 7`).

## 7. Analysis

- Paired unit: (map, pose[, power mode]). Effect: median of the paired differences.
- **Bootstrap 95% CI** of the median difference: resample poses with replacement, 10,000 resamples, percentile interval, seed 249 (`analysis/stats.py`, `bootstrap_ci`).
- **Wilcoxon signed-rank**, two-sided, zero differences dropped (`scipy.stats.wilcoxon`, `zero_method="wilcox"`).
- **Holm** correction across the 3 maps, separately for each method and metric.
- H2 slopes: OLS of log latency on log `n_frontiers`; CI by bootstrap over runs (cycles within a run are not independent).
- Software: `analysis/make_figures.py` rebuilds every number in `analysis/out/numbers.json` and every figure from `logs/`. **Commit: TODO (the `prereg-v1` tag).**

## 8. Invalid runs

Rerun with the same pose. At most 2 reruns per cell (method, map, pose, power mode). If all three attempts are invalid, the cell is reported as missing, and that pose is dropped from every paired test on that map and power mode (for all methods). Every invalid run and rerun gets a row in `docs/run_log.md`.

## 9. Decisions to settle before the tag

| Decision | Where | Status |
|----------|-------|--------|
| `replan_period_s`, `timeout_sim_s`, oscillation X, Block Q size | `docs/pilot_report.md` → `configs/common.yaml` | TODO after the pilot |
| Sensor setup (camera, lidar and hazard rates) | `docs/common_rules.md`, `configs/common.yaml` | TODO |
| MRTSP solver (`greedy` vs `dp`) | `docs/baseline_provenance.md` | TODO |
| ARiADNE map resolution (0.4 m vs raw), or the roadmap fallback | `docs/baseline_provenance.md` | TODO |
| Block E power modes | `docs/power_modes.md` | TODO |
| `k_loco` range and its source | Section 2 | TODO |
| Hypothesis wording | Section 1 | TODO (copy from the proposal) |

## 10. Freezing

```bash
git add docs/preregistration.md configs/ docs/pilot_report.md
git commit -m "Pre-registration v1"
git tag -a prereg-v1 -m "Protocol frozen"
git push origin prereg-v1
```

| member | acknowledged (date) |
|--------|---------------------|
| Shervan | |
| Joey | |
| Aaron | |

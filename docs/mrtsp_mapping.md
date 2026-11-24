# MRTSP: paper to code mapping (Module 6)

Paper: Liu, C. et al. (2025). *Enhancing autonomous exploration for robotics via real-time map optimization and improved frontier costs.* Scientific Reports 15:12261. https://www.nature.com/articles/s41598-025-97231-9

Code: `methods/frontier_exploration_ros2` @ `11864e4` (v1.6.2-5), a **third-party** implementation by M. Güler, not the authors' code. We evaluate "this implementation of MRTSP".

Status: **draft, 2026-11-23.** The code column was read from source at the pinned commit. The paper column (symbol, equation number, "same / different") is **TODO (full text)**: fill it while reading the paper, not from the package README. The README's own formulas do not match the code in three places (section 2), so the README is not a reliable stand-in for the paper.

## 1. Mapping table

All paths are relative to `methods/frontier_exploration_ros2/`.

| Paper term (symbol, eq.) | Meaning | Code location | Parameter | Value we use | Same / different |
|---|---|---|---|---|---|
| TODO | Map optimization: bilateral filter on the occupancy image | `src/decision_map.cpp`; switch `src/frontier_explorer_core_init.cpp:181` | `frontier_map_optimization_enabled`, `sigma_s`, `sigma_r` | true, 2.0 cells, 30.0 (package) | TODO |
| TODO | Map optimization: free-space dilation | `src/decision_map.cpp` | `dilation_kernel_radius_cells` | 1 (package) | TODO |
| TODO | Frontier extraction (WFD two-level BFS) | `src/frontier_search.cpp` (map BFS ~L589-L720) | `occ_threshold`, `min_frontier_size_cells` | 65, 6 (ours; package 5) | TODO |
| TODO | Information gain P(V_i, V_j) | `src/mrtsp_ordering.cpp:54-58`: `size` of the target frontier cluster, in cells | `weight_gain_ws` | 1.0 (package) | TODO |
| TODO | Gain weight | `src/mrtsp_ordering.cpp:126-128, 177-179`: **gain = size ^ weight_gain_ws** (an exponent, applied only when != 1.0) | `weight_gain_ws` | 1.0 | TODO. The README says it multiplies. |
| TODO | Path cost d(V_i, V_j), frontier to frontier | `src/mrtsp_ordering.cpp:60-72`: `max(dm + du, dn + dv) - r_s`, all **Euclidean** | `sensor_effective_range_m` (r_s) | 1.5 (package) | TODO |
| TODO | Path cost d(V_0, V_j), robot to frontier | `src/mrtsp_ordering.cpp:74-87`: same form; `dm` = **BFS distance through free cells** from the robot to the frontier center (`src/frontier_search.cpp:452, 593, 654, 708`), Euclidean only as a fallback | `sensor_effective_range_m` | 1.5 | TODO |
| TODO | Distance weight | `src/mrtsp_ordering.cpp:141, 198`: multiplies the path cost | `weight_distance_wd` | 1.0 (package) | TODO |
| TODO | Lower-bound motion time t_lb | `src/mrtsp_ordering.cpp:89-108`: **distance / vmax + heading / wmax** (sum) | `max_linear_speed_vmax`, `max_angular_speed_wmax` | 0.50, 1.0 (package). VERIFY TB4 limits | TODO. The README says min. |
| TODO | Start-row cost M(0, j) | `src/mrtsp_ordering.cpp:110-142`: `wd * d / gain + t_lb / sqrt(gain)` | as above | | TODO. The README says `+ t_lb`. |
| TODO | Frontier-to-frontier cost M(i, j) | `src/mrtsp_ordering.cpp:190-199`: `wd * d / gain` | as above | | TODO |
| TODO | How the TSP is solved | greedy: `src/mrtsp_ordering.cpp:206` `greedy_mrtsp_order` (nearest-neighbour walk over M). dp: `src/mrtsp_solver.cpp` bounded-horizon DP over a pruned pool | `mrtsp_solver` | **greedy** (ours; package default dp) | TODO. dp is the package author's extension, not from the paper. |
| TODO | How often the tour is recomputed | `src/frontier_explorer_node.cpp` map-processing timer -> `try_send_next_goal` (`src/frontier_explorer_core_dispatch.cpp:32`) when no goal is active; plus preemption (`consider_preempt_active_goal`, `:399`) | `map_processing_rate_hz`, `goal_preemption_enabled` | 1.0 Hz, true (package) | TODO |
| TODO | Only the first frontier is dispatched | `src/frontier_explorer_core_frontiers.cpp:389-398` (`select_frontier` takes `.front()`) | | | TODO |
| (not in paper?) | Visible-reveal-gain preemption | `src/frontier_explorer_core_dispatch.cpp:399` | `goal_preemption_*` | package defaults | TODO. Probably an addition by the package author. |
| (not in paper?) | Frontier suppression / failure memory | `src/frontier_suppression.cpp` | `frontier_suppression_enabled` | false (package) | TODO |

Every row whose last column ends up "different" becomes one sentence in the report's threats-to-validity section.

## 2. README vs. code (found while reading, 2026-11-23)

The package README "Algorithm and Mathematics" section describes the cost differently from the code at the pinned commit:

1. **Gain weight.** README: `M = (wd * d) / (ws * P)`. Code: `M = (wd * d) / P^ws`. Identical at the default `ws = 1.0`, different for any other value.
2. **Start-row time term.** README: `M(0, j) = wd * d / (ws * P) + t_lb`. Code: `wd * d / P + t_lb / sqrt(P)`.
3. **Lower-bound time.** README: `t_lb = min(L / vmax, |dyaw| / wmax)`. Code: `L / vmax + |dyaw| / wmax`.

Also:

- The README lists the DP defaults as `dp_solver_candidate_limit: 15, dp_planning_horizon: 10`. The shipped `config/params.yaml` uses 12 and 8. Not relevant if we run greedy.
- Robot-to-frontier BFS distance adds one cell per step on an 8-connected grid, so a diagonal step counts as `res`, not `res * sqrt(2)`. Diagonal paths are underestimated by up to about 29%.
- `nearest` mode was deprecated in v1.6.0 (README version history), so our nearest baseline comes from m-explore-ros2, not from this package.

## 3. Module 6 sanity checks

Runs: 2026-11-23, Bookstore, details in [benchmark_runs.md](benchmark_runs.md).

- [x] 5 runs of MRTSP (greedy) in the benchmark environment: 33.77 ± 1.73 m, 65.9 ± 4.8 s sim, 5/5 complete.
- [x] MRTSP total path shorter than nearest in the same environment and start pose: 33.77 ± 1.73 m vs. 35.73 ± 8.66 m (our nearest config) and 47.83 ± 5.82 m (benchmark m-explore config). Runs are not seed-paired; the sim is not deterministic.
- [x] Results consistent with the published greedy numbers (36.60 m / 63 s): time within 1 sd, distance 1.6 sd shorter.
- [ ] Warehouse runs: not done.
- [ ] Does planning cost grow with the number of frontiers (H2)? Greedy builds a full (n+1) x (n+1) matrix and walks it, so O(n²) per cycle on a cache miss (the order cache in `build_mrtsp_frontier_sequence` skips recomputation when frontiers and the pose bucket are unchanged). Confirm empirically with the Module 9 timing topic.

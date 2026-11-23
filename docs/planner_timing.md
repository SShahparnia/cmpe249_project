# Planner timing instrumentation (Module 9)

Owner: Aaron. Status: **patches written 2026-11-23. Not compiled, not run, not reviewed.**

Every method publishes one `exploration_msgs/PlanningCycle` per planning cycle on `/exploration/planning_cycle`. The recorder (Module 12) writes them to `cycles.csv` and `goals.csv`.

## Message

`harness/exploration_msgs/msg/PlanningCycle.msg`: `cycle_id`, `stamp_start` / `stamp_end` (ROS time, which is sim time under `use_sim_time`), `wall_start` / `wall_end` (Unix seconds), `n_frontiers`, `goal` (map frame).

## Build

The patched planners depend on `exploration_msgs`, so build it in the same colcon workspace before them:

```bash
scripts/apply_patches.sh          # applies methods/patches/*-timing.patch
colcon build --base-paths harness methods --packages-up-to explore_lite frontier_exploration_ros2 rl_planner ariadne_adapter
```

VERIFY: the exact `colcon` invocation once Module 3's workspace layout is settled. `methods/` also contains packages we do not build (for example `map_merge`), so use `--packages-up-to` or `--packages-skip`.

## Where each cycle starts and ends

| Method | Patch | Start (both clocks) | End (both clocks) | `n_frontiers` |
|--------|-------|---------------------|-------------------|---------------|
| nearest | `m-explore-ros2-timing.patch` | `Explore::makePlan`, right before `search_.searchFrom` | after the goal is chosen and the blacklist check passed, before the "same goal" early return and before `async_send_goal` | frontier clusters after the size filter |
| mrtsp | `frontier_exploration_ros2-timing.patch` | `try_send_next_goal`, right before `get_frontier_snapshot` | after `select_frontier_sequence`, before `send_frontier_goal` | clusters after suppression filtering |
| ariadne | `ARiADNE-ROS-Planner-timing.patch` | `Runner.run`, right before `check_valid_node` / `update_planning_state` | after the next waypoint is popped, before `/way_point` is published | **frontier cells** in the local updating window (`agent.frontier`), not clusters |

## Known limits (decide in review)

1. **`n_frontiers` is not the same unit across methods.** nearest and mrtsp count clusters; ariadne counts downsampled frontier cells in its local window. H2 fits slopes per method, so this is fine within a method but not across methods. Write that in the report.
2. **Cycles that end without a goal are not published.** "No frontiers" (exploration complete), "all suppressed" and ARiADNE's "exploration completed" return early. The orchestrator detects `stuck` from the absence of cycles, so this is intended.
3. **nearest publishes a cycle even when it keeps the same goal.** m-explore-ros2 replans every `1 / planner_frequency` s while a goal is active, so it publishes at that rate. mrtsp only plans when no goal is active (and in preemption). Its cycle count is therefore lower and not comparable as a rate; latency per cycle is still comparable.
4. **mrtsp preemption is not timed.** `consider_preempt_active_goal` (`src/frontier_explorer_core_dispatch.cpp:399`) can recompute a frontier sequence while a goal is active. That work is real compute that the patch does not capture. Option: add a second start/end pair there. Decide in review.
5. **nearest blacklist recursion.** When a goal is blacklisted, `makePlan` calls itself. Only the inner call publishes, so the outer search time is not counted.
6. **Cache hits.** mrtsp reuses cached frontier snapshots and MRTSP orders when nothing changed, so some cycles are near-zero latency. That is the method's real behaviour; keep those cycles.
7. **Wall clock vs. steady clock.** `wall_*` uses `system_clock` / `time.time()` as Section 0.3 requires. An NTP or chrony step during a run would distort a latency; chrony slews by default, so this should not happen. VERIFY `chronyc tracking` in the manifest.

## Step 5 sanity check (TODO, needs ROS)

Run each method for 2 minutes and `ros2 topic echo /exploration/planning_cycle`:

- [ ] nearest: durations positive, `n_frontiers` changes as the map grows
- [ ] mrtsp: same
- [ ] ariadne: same

## Review (step 3)

Each patch is reviewed by a second team member and confirmed to be timing-only.

| Patch | Reviewer | Date | Timing-only? | Notes |
|-------|----------|------|--------------|-------|
| `m-explore-ros2-timing.patch` | TODO | | | |
| `frontier_exploration_ros2-timing.patch` | TODO | | | |
| `ARiADNE-ROS-Planner-timing.patch` | TODO | | | |

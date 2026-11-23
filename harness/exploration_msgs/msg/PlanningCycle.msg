# One planning cycle of an exploration planner: from "map in" to "goal out".
# Published on /exploration/planning_cycle by our timing patches
# (methods/patches/*-timing.patch). See instructions.md Module 9.
#
# A cycle starts right before frontier extraction / graph update and ends right
# after the goal is chosen, before it is sent to Nav2.

uint64 cycle_id                       # per-process counter, starts at 0
builtin_interfaces/Time stamp_start   # ROS time (sim time when use_sim_time is true)
builtin_interfaces/Time stamp_end
float64 wall_start                    # Unix seconds: time.time() / std::chrono::system_clock
float64 wall_end
uint32 n_frontiers                    # frontier clusters (nearest, mrtsp) or frontier cells (ariadne); see docs/baseline_provenance.md
geometry_msgs/Point goal              # chosen goal in the map frame

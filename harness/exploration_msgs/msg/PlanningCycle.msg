# One planning cycle of an exploration planner (Module 9 of instructions.md).
# Published on /exploration/planning_cycle by each method's timing patch:
# start = right before frontier extraction, end = right after the goal is chosen
# (before it is sent to Nav2). The patch adds timing only, never logic.
uint64 cycle_id
builtin_interfaces/Time stamp_start   # ROS time (sim time when use_sim_time is true)
builtin_interfaces/Time stamp_end
float64 wall_start                    # Unix seconds: time.time() / std::chrono::system_clock
float64 wall_end
uint32 n_frontiers
geometry_msgs/Point goal              # chosen goal, map frame

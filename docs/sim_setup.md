# Simulation setup (Module 3)

## Stack

| Item | Value |
|------|-------|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| Simulator | Ignition Gazebo **Fortress** (what `ros-humble-turtlebot4-simulator` uses). Confirmed: `ign gazebo --versions` = 6.18.0 in the Mac container (2026-11-23) |
| Robot | TurtleBot 4 (standard), `turtlebot4_ignition_bringup` |
| SLAM | `slam_toolbox`, online async |
| Nav | Nav2 (`nav2_bringup` via `turtlebot4_navigation`) |
| RMW | CycloneDDS (`rmw_cyclonedds_cpp`), `ROS_DOMAIN_ID=49` (see `scripts/ros_env.sh`) |

> **Version note.** The README abstract and proposal say ROS 2 Jazzy + Gazebo Harmonic. JetPack 6 is Ubuntu 22.04, which only gets Humble debs, so `instructions.md` (and these scripts) use **Humble + Fortress** on both machines. TODO: the team confirms this and updates the README abstract.

## World-port decision (step 2)

The AWS RoboMaker worlds (M1 small house, M3 small warehouse) were written for Gazebo Classic; the TB4 sim on Humble runs on Ignition Fortress. Options, in order of preference:

1. A maintained Ignition/Fortress port of the AWS worlds.
2. Convert the SDF and models ourselves.
3. Run the TB4 model in Gazebo Classic.

| Decision | Source repo + commit | Who / date | Notes |
|----------|----------------------|------------|-------|
| TODO | | | |

## Real-time factor (step 4)

Measured with `python3 scripts/measure_rtf.py --seconds 60` while sim + SLAM + Nav2 run and the robot is driving under teleop.

| Machine (CPU / GPU / RAM) | World | Headless? | RTF | Who / date |
|---------------------------|-------|-----------|-----|------------|
| Apple M6 Mac, 16 GB, OrbStack container (12 vCPU, 7.8 GB, no GPU, llvmpipe) | default TB4 (`warehouse`), **stock sensors** | GUI on Xvfb | **0.007 typical**, bursts to 0.3; stalls for minutes while driving | Shervan / 2026-11-23 |
| same | same, stock sensors | server only (`gui:=false`) + RViz | 0.145 idle; stalls once driving | Shervan / 2026-11-23 |
| same | same, **tuned sensors** (`scripts/sim/tune_sensor_rates.sh`) | server only + RViz, Gazebo GUI attached | **0.92 to 1.0** while driving under Nav2 | Shervan / 2026-11-23 |
| TODO (GPU PC) | default TB4 (`warehouse`) | | | |

If RTF is well below 1.0, plan the Module 19 batch schedule around wall time = sim time / RTF.

### Mac findings (2026-11-23)

- **Cause of the slowness:** Gazebo renders the TB4's sensors in software (Mesa llvmpipe). Stock Humble TB4 has the rplidar, 4 cliff and 7 IR-intensity sensors at 62 Hz plus the OAK-D RGB-D camera at 30 Hz. The camera is the worst: with software OpenGL a single camera render could block the Gazebo server for minutes (one llvmpipe thread busy, every other thread waiting), which froze `/clock`.
- **Fix used:** `sudo bash scripts/sim/tune_sensor_rates.sh` (in the container, before launching): lidar 10 Hz (real RPLIDAR A1 is ~5.5 to 10 Hz), cliff and IR 5 Hz, camera removed (SLAM and Nav2 do not use it). `revert` restores the stock files. It edits the installed xacros in the container, so it is lost on `scripts/mac/sim.sh down`; rerun it after `up`.
- **Launch:** `ros2 launch harness/launch/tb4_sim.launch.py gui:=false slam:=true nav2:=true rviz:=true` runs Gazebo without its GUI. To watch in 3D, attach a GUI client to the running server (`ign gazebo -g`, with the TB4 `IGN_GUI_PLUGIN_PATH`); it is a separate process and did not slow the server.
- **Result:** two Nav2 waypoint tours (`scripts/drive_waypoints.py`), 6/6 and 8/8 goals reached, 24.4 m in 137 s wall for the first, x from -12 to 12 m across the warehouse for the second, RTF ~0.95 throughout.
- **TEAM DECISION NEEDED:** sensor rates change SLAM input. If we adopt the tuned rates, every sim machine (Mac and GPU PC) must use the same values, they go into `docs/common_rules.md` (Module 15) and every manifest, and they are frozen at `prereg-v1`. If we do, the Mac may be usable for experiments, not only development.

## Jetson link (step 5)

- Link: direct Ethernet, laptop `192.168.249.1/24`, Jetson `192.168.249.2/24` (`scripts/set_static_ip.sh`).
- Check: `bash scripts/jetson/link_test.sh` on the Jetson while the laptop sim runs.
- SLAM on the Jetson with `use_sim_time:=true` against the laptop sim builds a map: TODO (who / date / screenshot).
- Block E layout: **Gazebo on the laptop; SLAM, Nav2, planner, recorder and power logger on the Jetson.**

## Who has launched the stack from the README (Gate G0)

| Member | Laptop | Date | OK? |
|--------|--------|------|-----|
| Shervan | | | |
| Joey | | | |
| Aaron | | | |

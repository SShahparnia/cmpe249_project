# Simulation setup (Module 3)

## Stack

| Item | Value |
|------|-------|
| OS | Ubuntu 22.04 |
| ROS 2 | Humble |
| Simulator | Ignition Gazebo **Fortress** (what `ros-humble-turtlebot4-simulator` uses). TODO confirm with `ign gazebo --versions` |
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
| TODO | default TB4 (`warehouse`) | | | |

If RTF is well below 1.0, plan the Module 19 batch schedule around wall time = sim time / RTF.

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

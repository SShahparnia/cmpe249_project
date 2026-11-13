# Mac sim machine (Apple Silicon)

The project has no Ubuntu laptop. The simulation stack (Ubuntu 22.04, ROS 2 Humble, TurtleBot 4 sim on Ignition Gazebo Fortress, slam_toolbox, Nav2) runs on a Mac inside a container. Gazebo and RViz are shown in a browser desktop.

Plan for sim machines (see `docs/sim_setup.md`):

1. **Mac container (this doc):** development, debugging, the pilot, and the Gazebo host for the Block E Jetson link.
2. **A teammate's Windows PC with an NVIDIA GPU, booted from an Ubuntu 22.04 USB stick:** the main machine for the Block P/Q batches if the Mac's real-time factor is too low. It uses the normal `scripts/laptop/setup_laptop.sh` path.

## One-time setup

1. Install [OrbStack](https://orbstack.dev) (Docker Desktop also works). Open it once and choose **Docker**.
2. Build the image. This takes about 15 to 30 minutes and makes an image of about 6.3 GB:
   ```bash
   scripts/mac/sim.sh build
   ```
   The image runs the same `scripts/laptop/setup_laptop.sh` as an Ubuntu laptop, so the package list stays in one place.

## Daily use

```bash
scripts/mac/sim.sh up        # start the container and browser desktop
scripts/mac/sim.sh desktop   # open http://localhost:6080/vnc.html (Gazebo and RViz show up here)
scripts/mac/sim.sh shell     # a bash shell inside, with ROS env sourced; open as many as you need
scripts/mac/sim.sh down      # stop the container (the image is kept)
```

Inside `sim.sh shell`, the README Quick start step 3 commands work unchanged:

```bash
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=true nav2:=true rviz:=true
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # in a second shell
python3 scripts/measure_rtf.py --seconds 60              # in a third shell
```

The repo is bind-mounted at `/workspace/cmpe249_project`. Edit files on the Mac and build or run them inside the container. `colcon build` output (`build/`, `install/`, `log/`) lands in the repo and is git-ignored; `ros_env.sh` sources `install/setup.bash` when it exists.

## Limits

- **No GPU.** Gazebo renders with Mesa llvmpipe (software OpenGL 4.5), so the real-time factor (RTF) is likely below 1.0. Record it in `docs/sim_setup.md`.
- **Fixed RTF for experiments.** If the Mac runs below real time, planner compute (wall time) looks cheaper in sim time. For every method, cap Gazebo at an RTF the machine can always hold, record the measured RTF per run in `manifest.yaml`, and report planner timing in wall time (Module 9).
- **Memory.** OrbStack gives the VM about half the Mac's RAM by default (7.8 GB on a 16 GB Mac). If processes get killed, raise it in OrbStack, under Settings, then System, then Memory.
- **Jetson link (Block E).** DDS multicast discovery does not leave the container, so the laptop-to-Jetson setup in README step 4 and 5 does not work as written. The plan is a Zenoh bridge (`zenoh-bridge-ros2dds`) on both ends over one TCP port, plus a USB-C Ethernet adapter on the Mac. TODO: build and test once the Jetson is connected.
- **Power-mode label.** Runs on the Mac use `power_mode` = `mac` in place of `laptop` (Section 0.1 of `instructions.md`).

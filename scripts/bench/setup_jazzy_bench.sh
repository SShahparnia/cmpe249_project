#!/usr/bin/env bash
# Modules 5 and 6, step "benchmark run": install what the shared exploration benchmark needs.
#
# The benchmark (mertgulerx/autonomous-exploration-demo-benchmark) targets Ubuntu 24.04 +
# ROS 2 Jazzy + Gazebo Harmonic, not our Humble/Fortress project stack, so it gets its own
# machine or WSL distro. Package list = the benchmark's docker/Dockerfile (at the pinned
# commit below) plus ros-jazzy-desktop, which its base image osrf/ros:jazzy-desktop provides.
#
# Run once, on Ubuntu 24.04, as a normal user (it calls sudo itself):
#   bash scripts/bench/setup_jazzy_bench.sh 2>&1 | tee ~/bench_setup_$(date +%Y%m%d).log
# Only installs (everything that needs sudo). Building and running need no sudo.
set -euxo pipefail

BENCH_URL=https://github.com/mertgulerx/autonomous-exploration-demo-benchmark.git
BENCH_SHA=9dbf7bd869a9da5571c1b004a652e4b7988f348b
BENCH_DIR="$HOME/bench/autonomous-exploration-demo-benchmark"
REPO_URL=https://github.com/SShahparnia/cmpe249_project.git
REPO_DIR="$HOME/cmpe249_project"
REPO_BRANCH=aaron-dev

. /etc/os-release
if [[ "$VERSION_ID" != "24.04" ]]; then
  echo "Expected Ubuntu 24.04 for ROS 2 Jazzy. Found $VERSION_ID." >&2
  exit 1
fi

# --- ROS 2 Jazzy apt source ---
sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release git
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update

# --- Packages: benchmark Dockerfile list + desktop base ---
sudo apt-get install -y \
  ros-jazzy-desktop \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  xauth x11-apps mesa-utils libgl1-mesa-dri libglu1-mesa libxkbcommon0 \
  libwayland-client0 wayland-protocols \
  ros-jazzy-ros-gz \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-topic-tools \
  ros-jazzy-interactive-marker-twist-server \
  ros-jazzy-xacro \
  ros-jazzy-pcl-ros \
  python3-numpy python3-yaml python3-psutil
sudo rosdep init 2>/dev/null || true
rosdep update --rosdistro jazzy

# --- Sources: benchmark at the pinned commit, and our repo (pinned planner submodules) ---
mkdir -p "$(dirname "$BENCH_DIR")"
if [[ ! -d "$BENCH_DIR/.git" ]]; then
  git clone "$BENCH_URL" "$BENCH_DIR"
fi
git -C "$BENCH_DIR" fetch --quiet origin
git -C "$BENCH_DIR" checkout --quiet "$BENCH_SHA"
git -C "$BENCH_DIR" submodule update --init --recursive

if [[ ! -d "$REPO_DIR/.git" ]]; then
  git clone --recursive -b "$REPO_BRANCH" "$REPO_URL" "$REPO_DIR"
fi

# --- rosdep for everything we will build (needs sudo for apt) ---
set +u
source /opt/ros/jazzy/setup.bash
set -u
rosdep install -r -y --ignore-src --rosdistro jazzy --from-paths \
  "$BENCH_DIR/exploration_packages" "$BENCH_DIR/simulation" "$BENCH_DIR/rviz" \
  "$REPO_DIR/methods/m-explore-ros2" "$REPO_DIR/methods/frontier_exploration_ros2" \
  "$REPO_DIR/harness/exploration_msgs"

set +x
echo
echo "Done. Installs finished; tell Claude to continue with the build and runs."
echo "Check: ros2 --help >/dev/null && gz sim --versions   (expect 8.x = Harmonic)"

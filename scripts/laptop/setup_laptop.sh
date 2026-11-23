#!/usr/bin/env bash
# Module 3, step 1: laptop simulation stack on Ubuntu 22.04.
# ROS 2 Humble + TurtleBot 4 simulator (Ignition Gazebo Fortress) + slam_toolbox + Nav2.
# Run on each team member's Ubuntu laptop:
#   bash scripts/laptop/setup_laptop.sh 2>&1 | tee laptop_setup_$(date +%Y%m%d).log
set -euxo pipefail

. /etc/os-release
if [[ "$VERSION_ID" != "22.04" ]]; then
  echo "Expected Ubuntu 22.04 for ROS 2 Humble. Found $VERSION_ID." >&2
  exit 1
fi

# --- ROS 2 Humble apt source ---
sudo apt-get update
sudo apt-get install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update

# --- ROS 2 desktop + sim stack ---
sudo apt-get install -y \
  ros-humble-desktop \
  ros-humble-turtlebot4-simulator \
  ros-humble-turtlebot4-navigation \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-teleop-twist-keyboard \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  chrony
sudo rosdep init 2>/dev/null || true
rosdep update

# --- Shell environment ---
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LINE="source $REPO/scripts/ros_env.sh"
grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc

set +x
echo
echo "Done. Open a new terminal, then follow README 'Quick start' step 3."
echo "Simulator check:  ign gazebo --versions   (expect 6.x = Fortress)"

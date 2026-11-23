#!/usr/bin/env bash
# Module 2 steps 4-5 (+ chrony from Section 0.3): software setup on a freshly
# flashed Jetson Orin Nano (JetPack 6.x = Ubuntu 22.04 -> ROS 2 Humble).
# Run ON THE JETSON:  bash scripts/jetson/setup_jetson.sh 2>&1 | tee jetson_setup_$(date +%Y%m%d).log
# Every command it runs is echoed, so the log can be pasted into docs/jetson_setup.md.
set -euxo pipefail

. /etc/os-release
if [[ "$VERSION_ID" != "22.04" ]]; then
  echo "Expected Ubuntu 22.04 (JetPack 6). Found $VERSION_ID. Stop and check the flash." >&2
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

# --- ROS 2 (headless: ros-base, no desktop/Gazebo on the Jetson) ---
sudo apt-get install -y \
  ros-humble-ros-base \
  ros-humble-slam-toolbox \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-turtlebot4-navigation \
  ros-humble-rmw-cyclonedds-cpp \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep
sudo rosdep init 2>/dev/null || true
rosdep update

# --- Measurement tools ---
sudo apt-get install -y stress-ng chrony python3-pip python3-matplotlib python3-numpy
sudo pip3 install -U jetson-stats

# --- Shell environment (same as the laptop; see scripts/ros_env.sh) ---
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
LINE="source $REPO/scripts/ros_env.sh"
grep -qxF "$LINE" ~/.bashrc || echo "$LINE" >> ~/.bashrc

set +x
echo
echo "Done. REBOOT now so jtop's service starts:  sudo reboot"
echo "After reboot: run  jtop  (check power rails show), then  bash scripts/jetson/collect_info.sh"

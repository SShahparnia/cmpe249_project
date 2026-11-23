#!/usr/bin/env bash
# Module 2 steps 1-3: print everything docs/jetson_setup.md and
# docs/power_modes.md need, as Markdown. Run ON THE JETSON after setup + reboot:
#   bash scripts/jetson/collect_info.sh | tee jetson_info.md
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "## Board info (collected $(date -Is) on $(hostname))"
echo
echo '```'
echo "# /etc/nv_tegra_release"; cat /etc/nv_tegra_release 2>/dev/null || echo "(missing)"
echo; echo "# nvidia-l4t-core / jetpack package versions"
dpkg-query -W -f='${Package} ${Version}\n' nvidia-l4t-core nvidia-jetpack 2>/dev/null || true
echo; echo "# model"; tr -d '\0' < /proc/device-tree/model 2>/dev/null; echo
echo; echo "# kernel"; uname -a
echo; echo "# OS"; grep PRETTY_NAME /etc/os-release
echo; echo "# root storage"; findmnt -no SOURCE,SIZE / 2>/dev/null; lsblk -dno NAME,MODEL,SIZE,TRAN 2>/dev/null
echo; echo "# RAM"; free -h | head -2
echo; echo "# current power mode (sudo nvpmodel -q)"; sudo nvpmodel -q 2>&1
echo; echo "# fan (nvfancontrol)"; grep -E '^\s*FAN_DEFAULT' /etc/nvfancontrol.conf 2>/dev/null || echo "(no /etc/nvfancontrol.conf)"
systemctl is-active nvfancontrol 2>/dev/null | sed 's/^/nvfancontrol service: /'
echo; echo "# jetson_clocks must NOT be active during experiments:"
sudo jetson_clocks --show 2>/dev/null | head -20 || echo "(jetson_clocks not found)"
echo; echo "# ROS"; (source /opt/ros/humble/setup.bash && printenv ROS_DISTRO && ros2 pkg list 2>/dev/null | grep -cE '^(slam_toolbox|nav2_bringup|rmw_cyclonedds_cpp)$' | sed 's/^/required ROS pkgs found: /') 2>/dev/null
echo; echo "# jetson-stats"; pip3 show jetson-stats 2>/dev/null | grep -E '^(Name|Version)'
echo '```'
echo
echo "## Power modes defined in /etc/nvpmodel.conf (VERIFY against the raw file)"
echo
python3 "$HERE/parse_nvpmodel.py" /etc/nvpmodel.conf

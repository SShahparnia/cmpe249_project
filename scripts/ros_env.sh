# Shared ROS 2 environment for the laptop AND the Jetson (Module 3, step 5).
# Source it from ~/.bashrc:   source <repo>/scripts/ros_env.sh
# Both machines must agree on ROS_DOMAIN_ID and RMW_IMPLEMENTATION.

_EB_REPO="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")/.." && pwd)"

source /opt/ros/humble/setup.bash
[ -f "$_EB_REPO/install/setup.bash" ] && source "$_EB_REPO/install/setup.bash"

export ROS_DOMAIN_ID=49                       # team-wide; change here only
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  # roadmap-explorer also recommends CycloneDDS

# Pin DDS to the laptop<->Jetson Ethernet link when EB_IFACE is set
# (e.g. export EB_IFACE=enp3s0 before sourcing). Unset = CycloneDDS picks.
if [ -n "${EB_IFACE:-}" ]; then
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$EB_IFACE\"/></Interfaces></General></Domain></CycloneDDS>"
fi

export EB_REPO="$_EB_REPO"
unset _EB_REPO

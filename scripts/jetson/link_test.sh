#!/usr/bin/env bash
# Module 3, step 5: laptop<->Jetson ROS 2 link checks. Run ON THE JETSON while
# the laptop runs the TB4 sim (README Quick start step 5a).
set -uo pipefail
LAPTOP=192.168.249.1
pass() { echo "PASS  $*"; }
fail() { echo "FAIL  $*"; FAILED=1; }
FAILED=0

ping -c 3 -W 1 "$LAPTOP" > /dev/null && pass "ping $LAPTOP" || fail "ping $LAPTOP"
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID  RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION"

topics="$(timeout 15 ros2 topic list 2>/dev/null)"
for t in /clock /scan /tf /odom; do
  grep -qx "$t" <<< "$topics" && pass "topic $t visible" || fail "topic $t not visible"
done

timeout 10 ros2 topic echo /clock --once > /dev/null 2>&1 && pass "/clock echo" || fail "/clock echo"
echo "--- /scan rate (10 s) ---"
timeout 10 ros2 topic hz /scan 2>&1 | tail -3

echo "--- reverse direction ---"
echo "Publishing /eb_link_test from the Jetson for 30 s. On the LAPTOP run:"
echo "    ros2 topic echo /eb_link_test --once"
timeout 30 ros2 topic pub -r 2 /eb_link_test std_msgs/msg/String "{data: 'hello from jetson'}" > /dev/null 2>&1

(( FAILED )) && { echo "Some checks failed."; exit 1; }
echo "All Jetson-side checks passed."

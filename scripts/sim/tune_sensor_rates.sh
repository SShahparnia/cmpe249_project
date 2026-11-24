#!/usr/bin/env bash
# Lower the TurtleBot 4 sim's rendered-sensor rates so Gazebo runs usably without a GPU.
#
# Stock Humble TB4 renders 17 sensors in Gazebo: rplidar, 4 cliff and 7 IR-intensity
# rays at 62 Hz, the OAK-D RGB-D camera at 30 Hz, 4 dock IR emitters at 1 Hz. With
# software OpenGL (Mac container) that is ~800 renders per sim second and RTF ~0.01.
#
#   sudo bash scripts/sim/tune_sensor_rates.sh          # apply (then relaunch the sim)
#   sudo bash scripts/sim/tune_sensor_rates.sh revert   # restore the stock files
#
# Edits the installed xacros in /opt/ros/humble (originals kept as *.orig). Inside the
# Mac container this lasts until `scripts/mac/sim.sh down`.
# NOT decided for experiments: sensor rates change SLAM input. If the team adopts this,
# every sim machine must use the same rates and they get recorded in the manifest.
set -euo pipefail

LIDAR_HZ=${LIDAR_HZ:-10}     # real RPLIDAR A1: ~5.5-10 Hz
HAZARD_HZ=${HAZARD_HZ:-5}    # cliff + IR intensity (Create 3 hazard sensors)
CAMERA_HZ=${CAMERA_HZ:-0}    # OAK-D, unused by SLAM / Nav2 here; 0 = remove the sensor
# (with software OpenGL a single camera render can stall Gazebo for minutes)

TB4=/opt/ros/humble/share/turtlebot4_description/urdf/sensors
CR3=/opt/ros/humble/share/irobot_create_description/urdf/sensors
FILES=("$TB4/rplidar.urdf.xacro" "$TB4/oakd.urdf.xacro" "$CR3/cliff_sensor.urdf.xacro" "$CR3/ir_intensity.urdf.xacro")

if [[ "${1:-}" == "revert" ]]; then
  for f in "${FILES[@]}"; do [[ -f "$f.orig" ]] && cp "$f.orig" "$f"; done
  echo "restored stock sensor rates"; exit 0
fi

for f in "${FILES[@]}"; do [[ -f "$f.orig" ]] || cp "$f" "$f.orig"; cp "$f.orig" "$f"; done
sed -i -E "s/update_rate=\"62\.0\"/update_rate=\"${LIDAR_HZ}.0\"/" "$TB4/rplidar.urdf.xacro"
if [[ "$CAMERA_HZ" == "0" ]]; then
  # Drop the RGB-D sensor entirely (0 would mean "as fast as possible" in Gazebo).
  sed -i -E '/<sensor name="rgbd_camera"/,/<\/sensor>/d' "$TB4/oakd.urdf.xacro"
else
  sed -i -E "s#<update_rate>30</update_rate>#<update_rate>${CAMERA_HZ}</update_rate>#" "$TB4/oakd.urdf.xacro"
fi
sed -i -E "s/name=\"update_rate\"( +)value=\"62\"/name=\"update_rate\"\1value=\"${HAZARD_HZ}\"/" "$CR3/cliff_sensor.urdf.xacro"
sed -i -E "s/update_rate:=62\.0/update_rate:=${HAZARD_HZ}.0/" "$CR3/ir_intensity.urdf.xacro"

grep -H -n -E 'update_rate' "${FILES[@]}" | grep -v -E '\$\{update_rate\}' | sed -E 's#.*/urdf/sensors/##'
echo "lidar ${LIDAR_HZ} Hz, cliff/IR ${HAZARD_HZ} Hz, camera ${CAMERA_HZ} Hz. Relaunch the sim."

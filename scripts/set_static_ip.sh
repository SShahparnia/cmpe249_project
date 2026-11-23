#!/usr/bin/env bash
# Module 3, step 5: static IP on the laptop<->Jetson Ethernet link (NetworkManager).
#   laptop:  bash scripts/set_static_ip.sh <iface> laptop
#   jetson:  bash scripts/set_static_ip.sh <iface> jetson
# Find <iface> with:  ip -br link   (e.g. enp3s0 on the laptop, eno1 / enP8p1s0 on the Jetson)
set -euo pipefail
IFACE="${1:?usage: set_static_ip.sh <iface> laptop|jetson}"
ROLE="${2:?usage: set_static_ip.sh <iface> laptop|jetson}"
case "$ROLE" in
  laptop) ADDR=192.168.249.1/24 ;;
  jetson) ADDR=192.168.249.2/24 ;;
  *) echo "role must be laptop or jetson" >&2; exit 1 ;;
esac
CON=explorebench-link
sudo nmcli con delete "$CON" 2>/dev/null || true
sudo nmcli con add type ethernet ifname "$IFACE" con-name "$CON" \
  ipv4.method manual ipv4.addresses "$ADDR" ipv4.never-default yes ipv6.method disabled
sudo nmcli con up "$CON"
ip -br addr show "$IFACE"
echo "Now: export EB_IFACE=$IFACE in ~/.bashrc (before sourcing scripts/ros_env.sh)."

#!/usr/bin/env bash
# Idle-stack baseline (Module 14): sim running (on the laptop in Block E), SLAM + Nav2 running
# here, robot stationary, no planner, power logger on, for 5 minutes. Writes
# logs/idle_<power_mode>_<timestamp>/ (power.csv, procs.csv, gpu.csv, manifest.yaml).
#
#   bash scripts/idle_baseline.sh <power_mode> [extra harness/run.py args]
#   bash scripts/idle_baseline.sh 15W --sim-ssh laptop          # Block E: Gazebo on the laptop
#
# It is harness/run.py with --method none, so the stack is launched exactly as in a real run.
# Run it at the start and at the end of every Block E session (docs/block_e_procedure.md).
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ $# -lt 1 ]]; then
  sed -n '2,10p' "$0"; exit 1
fi
MODE=$1; shift
exec python3 "$REPO/harness/run.py" --method none --power-mode "$MODE" --jetson \
  --map "${EB_IDLE_MAP:-m1_house}" --pose 0 --duration "${EB_IDLE_SECONDS:-300}" "$@"

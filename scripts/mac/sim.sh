#!/usr/bin/env bash
# Mac sim machine: Ubuntu 22.04 + ROS 2 Humble + TB4 sim in a container (OrbStack or Docker Desktop).
#   scripts/mac/sim.sh build     # build the image (first time ~15-30 min, ~5-7 GB)
#   scripts/mac/sim.sh up        # start the container + browser desktop
#   scripts/mac/sim.sh shell     # bash inside it, ROS env sourced (run as many as you like)
#   scripts/mac/sim.sh desktop   # open http://localhost:6080/vnc.html
#   scripts/mac/sim.sh down      # stop and remove the container (image is kept)
# See docs/mac_setup.md.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
export PATH="$PATH:$HOME/.orbstack/bin:/usr/local/bin"
command -v docker >/dev/null || { echo "docker not found: start OrbStack (open -a OrbStack) first" >&2; exit 1; }
DC=(docker compose -f "$REPO/docker/mac/compose.yaml")
URL="http://localhost:6080/vnc.html?autoconnect=1&resize=remote"

case "${1:-}" in
  build)   "${DC[@]}" build ;;
  up)      "${DC[@]}" up -d && echo "desktop: $URL" ;;
  shell)   docker exec -it cmpe249-sim bash ;;   # interactive bash sources ~/.bashrc -> ros_env.sh
  exec)    shift; docker exec cmpe249-sim bash -c "source /workspace/cmpe249_project/scripts/ros_env.sh; $*" ;;
  desktop) open "$URL" ;;
  down)    "${DC[@]}" down ;;
  status)  "${DC[@]}" ps ;;
  *) sed -n '2,8p' "$0"; exit 1 ;;
esac

#!/usr/bin/env bash
# Module 2, steps 7-8: 5 min idle + 5 min stress-ng, then plot and report the
# real sample rate. Run ON THE JETSON, with nothing else heavy running.
#   bash scripts/jetson/power_smoke_run.sh [seconds_per_phase]
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DUR="${1:-300}"
OUT="$REPO/logs/power_smoke_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT"

echo "power mode: $(sudo nvpmodel -q 2>/dev/null | head -1)" | tee "$OUT/info.txt"
echo "[1/2] idle for ${DUR}s"
python3 "$REPO/scripts/power_smoke_test.py" "$OUT/idle.csv" --duration "$DUR"

echo "[2/2] stress-ng --cpu 6 for ${DUR}s"
stress-ng --cpu 6 --timeout "${DUR}s" > "$OUT/stress-ng.log" 2>&1 &
STRESS=$!
python3 "$REPO/scripts/power_smoke_test.py" "$OUT/stress.csv" --duration "$DUR"
wait "$STRESS" || true

python3 "$REPO/scripts/plot_power_smoke.py" "$OUT/idle.csv" "$OUT/stress.csv" \
  --out "$REPO/docs/figures/power_smoke.png" | tee -a "$OUT/info.txt"
echo "Raw data in $OUT; copy the summary above into docs/jetson_setup.md."

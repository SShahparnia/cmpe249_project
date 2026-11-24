#!/usr/bin/env bash
# Archive logs/ outside git (Module 22, step 4).
#
#   bash scripts/archive_logs.sh [out_dir]          # default out_dir: the parent of the repo
#
# Writes <out_dir>/explorebench_logs_<date>.tar.gz and a .sha256 next to it, then prints the
# README lines to paste (link + checksum) after uploading the archive.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="${1:-$(dirname "$REPO")}"
NAME="explorebench_logs_$(date +%Y%m%d)"
[[ -d "$REPO/logs" ]] || { echo "no logs/ folder" >&2; exit 1; }
tar -C "$REPO" -czf "$OUT/$NAME.tar.gz" logs
( cd "$OUT" && sha256sum "$NAME.tar.gz" > "$NAME.tar.gz.sha256" )
SUM=$(cut -d' ' -f1 "$OUT/$NAME.tar.gz.sha256")
N=$(find "$REPO/logs" -maxdepth 1 -mindepth 1 -type d | wc -l)
echo "wrote $OUT/$NAME.tar.gz ($N run folders, $(du -h "$OUT/$NAME.tar.gz" | cut -f1))"
echo
echo "Paste into README.md after uploading:"
echo "| Run data | <link> | sha256 \`$SUM\` | $N folders, commit $(git -C "$REPO" rev-parse --short HEAD) |"

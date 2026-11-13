#!/usr/bin/env bash
# Fail if any commit message reachable from HEAD carries AI-tool attribution
# (Co-Authored-By trailers, "Generated with" footers, session trailers).
# Run before pushing or opening a PR:  scripts/check_no_ai_attribution.sh
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PATTERN='(co-authored-by:.*(claude|anthropic|noreply@anthropic\.com|\bai\b|assistant|copilot|chatgpt|openai))|(generated (with|by).*(claude|anthropic))|(^\s*🤖)|(^\s*claude-[a-z-]*:)'

bad=0
while IFS= read -r sha; do
  if git log -1 --format=%B "$sha" | grep -qiE "$PATTERN"; then
    echo "attribution found in commit $(git log -1 --format='%h %s' "$sha")"
    bad=1
  fi
done < <(git rev-list HEAD)

if (( bad )); then
  echo "FAIL: rewrite those commit messages before pushing." >&2
  exit 1
fi
echo "OK: no AI attribution in $(git rev-list --count HEAD) commits."

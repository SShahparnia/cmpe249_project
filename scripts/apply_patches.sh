#!/usr/bin/env bash
# Apply our patches on top of the pinned submodules.
#
# Patches live in methods/patches/<submodule>-<what>.patch, where <submodule> is
# the directory name under methods/ (e.g. m-explore-ros2-timing.patch).
# Never edit submodule code directly; see instructions.md Module 1, step 4.
#
# Idempotent: a patch that is already applied is skipped.
# Usage: scripts/apply_patches.sh [--reset]
#   --reset  first restore every submodule to its pinned commit (drops local edits)
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PATCH_DIR="$ROOT/methods/patches"
cd "$ROOT"

if [[ "${1:-}" == "--reset" ]]; then
  echo "[apply_patches] resetting submodules to pinned commits"
  git submodule foreach --quiet 'git reset --hard --quiet && git clean -fdq'
  git submodule update --init --recursive
fi

shopt -s nullglob
patches=("$PATCH_DIR"/*.patch)
if (( ${#patches[@]} == 0 )); then
  echo "[apply_patches] no patches in methods/patches/, nothing to do"
  exit 0
fi

# Submodule dir names, longest first, so "ARiADNE-ROS-Planner-x.patch" does not
# match the shorter "ARiADNE" prefix.
subs=()   # (no mapfile: macOS ships bash 3.2)
while IFS= read -r s; do subs+=("$s"); done < <(git config -f .gitmodules --get-regexp 'submodule\..*\.path' \
  | awk '{print $2}' | sed 's#^methods/##' | awk '{print length, $0}' | sort -rn | cut -d' ' -f2-)

applied=0; skipped=0
for p in "${patches[@]}"; do
  name="$(basename "$p")"
  target=""
  for s in "${subs[@]}"; do
    if [[ "$name" == "$s-"* ]]; then target="$s"; break; fi
  done
  if [[ -z "$target" ]]; then
    echo "[apply_patches] ERROR: $name does not start with a known submodule name" >&2
    exit 1
  fi
  dir="methods/$target"
  if git -C "$dir" apply --reverse --check "$p" 2>/dev/null; then
    echo "[apply_patches] already applied: $name"
    skipped=$((skipped + 1))
  elif git -C "$dir" apply --check "$p"; then
    git -C "$dir" apply "$p"
    echo "[apply_patches] applied: $name -> $dir"
    applied=$((applied + 1))
  else
    echo "[apply_patches] ERROR: $name does not apply cleanly to $dir" >&2
    exit 1
  fi
done
echo "[apply_patches] done: $applied applied, $skipped already present"

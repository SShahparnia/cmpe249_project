#!/usr/bin/env bash
# Point this clone at the repo's versioned hooks in .githooks/ and turn on the
# attribution-stripping flag. Every team member runs this once after cloning.
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"
chmod +x .githooks/*
git config core.hooksPath .githooks
git config explorebench.stripAttribution true
echo "Hooks enabled (core.hooksPath=.githooks, explorebench.stripAttribution=true)."

#!/usr/bin/env python3
"""Fill the report template with numbers from analysis/out/numbers.json (Module 22).

    python3 analysis/make_figures.py                  # first: rebuild numbers.json and figures
    python3 analysis/fill_report.py [--template docs/report/report_template.md] [--out docs/report/report.md]

Placeholders in the template:

    {{q1.mrtsp.m1_house.median_rel|pct}}     dotted path into numbers.json, optional format
    {{q2.nearest.m2_office.7W.latency_p95_s|ms}}

Formats: pct (0.123 -> 12.3%), ms (s -> ms, 1 decimal), f0/f1/f2/f3 (fixed decimals),
p (p-value: 3 significant digits, "< 0.001" below that), int, raw (default: 3 significant).
Every number in the report comes from numbers.json, so nothing is typed by hand (Module 20).
A placeholder whose key is missing is an error: the script lists all of them and writes
nothing, unless --allow-missing (then they render as **MISSING(key)**, for drafts).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
PLACEHOLDER = re.compile(r"\{\{\s*([A-Za-z0-9_.\-]+)\s*(?:\|\s*([a-z0-9]+)\s*)?\}\}")


def lookup(numbers: dict, path: str):
    cur = numbers
    for part in path.split("."):
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        elif isinstance(cur, list) and part.isdigit() and int(part) < len(cur):
            cur = cur[int(part)]
        else:
            raise KeyError(path)
    return cur


def fmt(v, kind: str | None) -> str:
    if v is None or (isinstance(v, float) and not math.isfinite(v)):
        return "n/a"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if not isinstance(v, (int, float)):
        return str(v)
    if kind == "pct":
        return f"{100 * v:.1f}%"
    if kind == "ms":
        return f"{1000 * v:.1f} ms"
    if kind in ("f0", "f1", "f2", "f3"):
        return f"{v:.{kind[1]}f}"
    if kind == "p":
        return "< 0.001" if v < 0.001 else f"{v:.3g}"
    if kind == "int":
        return str(int(round(v)))
    return f"{v:.3g}" if isinstance(v, float) else str(v)


def fill(template: str, numbers: dict, allow_missing=False):
    missing = []

    def sub(m):
        key, kind = m.group(1), m.group(2)
        try:
            return fmt(lookup(numbers, key), kind)
        except KeyError:
            missing.append(key)
            return f"**MISSING({key})**"
    text = PLACEHOLDER.sub(sub, template)
    return text, missing


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--numbers", default=os.path.join(HERE, "out", "numbers.json"))
    ap.add_argument("--template", default=os.path.join(REPO, "docs", "report", "report_template.md"))
    ap.add_argument("--out", default=os.path.join(REPO, "docs", "report", "report.md"))
    ap.add_argument("--allow-missing", action="store_true")
    a = ap.parse_args(argv)
    with open(a.numbers) as f:
        numbers = json.load(f)
    with open(a.template, encoding="utf-8") as f:
        template = f.read()
    text, missing = fill(template, numbers, a.allow_missing)
    if missing and not a.allow_missing:
        print("missing numbers (run make_figures.py, or fix the keys):\n  " + "\n  ".join(sorted(set(missing))),
              file=sys.stderr)
        return 1
    with open(a.out, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    print(f"wrote {a.out}" + (f" with {len(set(missing))} MISSING placeholders" if missing else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())

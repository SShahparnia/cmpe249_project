#!/usr/bin/env python3
"""Module 2, step 6: power-logging smoke test (runs ON THE JETSON).

Writes one CSV row per jtop sample: t_wall, then one <rail>_mW column per rail
jtop reports (VDD_IN first). The extra rails are there so we can VERIFY their
names before Module 14 fixes the schema.

Usage:
  python3 scripts/power_smoke_test.py --list-rails          # print jetson.power once
  python3 scripts/power_smoke_test.py out.csv [--duration 300] [--interval 0.1]
"""
import argparse
import csv
import json
import sys
import time

from jtop import jtop

ap = argparse.ArgumentParser()
ap.add_argument("out", nargs="?")
ap.add_argument("--duration", type=float, default=None, help="seconds; default: until Ctrl-C")
ap.add_argument("--interval", type=float, default=0.1, help="requested jtop interval (s)")
ap.add_argument("--list-rails", action="store_true")
args = ap.parse_args()

with jtop(interval=args.interval) as jetson:
    if args.list_rails:
        print(json.dumps(jetson.power, indent=2, default=str))
        sys.exit(0)
    if not args.out:
        ap.error("output CSV path required")

    rails = list(jetson.power["rail"].keys())
    if "VDD_IN" in rails:
        rails.remove("VDD_IN")
        rails.insert(0, "VDD_IN")
    else:
        print(f"WARNING: no VDD_IN rail; rails are {rails}", file=sys.stderr)

    t_end = time.time() + args.duration if args.duration else None
    n = 0
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_wall"] + [f"{r.lower()}_mW" for r in rails])
        try:
            while jetson.ok():
                now = time.time()
                if t_end and now >= t_end:
                    break
                pr = jetson.power["rail"]
                w.writerow([f"{now:.4f}"] + [pr.get(r, {}).get("power", "") for r in rails])
                n += 1
                if n % 50 == 0:
                    f.flush()
        except KeyboardInterrupt:
            pass
    print(f"wrote {n} rows to {args.out}")

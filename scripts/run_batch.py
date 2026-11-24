#!/usr/bin/env python3
"""Run a schedule (Module 19 of instructions.md, step 3).

    python3 scripts/run_batch.py configs/schedules/Q.csv --machine laptop-a
    python3 scripts/run_batch.py configs/schedules/E_7W.csv --machine jetson -- --jetson --sim-ssh laptop
    python3 scripts/run_batch.py configs/schedules/P.csv --machine laptop --dry-run

For each row of the schedule assigned to --machine, in order:
  - skip it if logs/ already holds a run of that cell with outcome complete, timeout or stuck;
  - otherwise run it with harness/run.py (arguments after `--` are passed through).
Rows that end `invalid` are retried after the whole schedule, at most --max-retries extra
attempts per row (earlier invalid attempts in logs/ count). Every run gets one line in
logs/batch_<date>.log; every invalid run and every rerun gets a row in docs/run_log.md.
Stop with Ctrl-C: the current run is shut down cleanly by run.py and marked invalid.
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import os
import subprocess
import sys

import yaml

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
DATA = ("complete", "timeout", "stuck")
KEY = ("block", "method", "map", "pose", "power_mode")


def cell_key(d) -> tuple:
    return (str(d["block"]), str(d["method"]), str(d["map"]), int(d["pose"] if "pose" in d else d["pose_index"]),
            str(d["power_mode"]))


def existing_runs(logs: str) -> dict:
    """cell -> list of (run_id, outcome) found in logs/, oldest first."""
    out = {}
    if not os.path.isdir(logs):
        return out
    for name in sorted(os.listdir(logs)):
        mp = os.path.join(logs, name, "manifest.yaml")
        if name.startswith("idle_") or not os.path.isfile(mp):
            continue
        try:
            with open(mp) as f:
                m = yaml.safe_load(f) or {}
            out.setdefault(cell_key(m), []).append((name, m.get("outcome")))
        except (OSError, yaml.YAMLError, KeyError, TypeError, ValueError):
            continue
    return out


def read_schedule(path, machine):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    rows.sort(key=lambda r: int(r["order"]))
    return [r for r in rows if machine in ("any", r.get("machine", ""), "")]


def run_log_row(who, run_id, what, action, path=os.path.join(REPO, "docs", "run_log.md")):
    with open(path, "a", newline="\n") as f:
        f.write(f"| {dt.date.today().isoformat()} | {who} | {run_id} | {what} | {action} |\n")


def git_user():
    try:
        return subprocess.check_output(["git", "-C", REPO, "config", "user.name"], text=True).strip() or "?"
    except (OSError, subprocess.CalledProcessError):
        return "?"


class Batch:
    def __init__(self, a, passthrough):
        self.a, self.passthrough = a, passthrough
        os.makedirs(a.logs_dir, exist_ok=True)
        self.log_path = os.path.join(a.logs_dir, f"batch_{dt.date.today().strftime('%Y%m%d')}.log")
        self.who = a.who or git_user()

    def log(self, text):
        line = f"{dt.datetime.now().isoformat(timespec='seconds')} {text}"
        print(line, flush=True)
        with open(self.log_path, "a") as f:
            f.write(line + "\n")

    def run_row(self, r, attempt) -> tuple:
        cmd = [sys.executable, self.a.runner, "--block", r["block"], "--method", r["method"],
               "--map", r["map"], "--pose", str(r["pose"]), "--power-mode", r["power_mode"],
               "--logs-dir", self.a.logs_dir] + self.passthrough
        if self.a.dry_run:
            self.log(f"DRY {' '.join(cmd)}")
            return None, "dry-run"
        before = set(os.listdir(self.a.logs_dir))
        subprocess.call(cmd)
        new = sorted(set(os.listdir(self.a.logs_dir)) - before)
        new = [n for n in new if os.path.isfile(os.path.join(self.a.logs_dir, n, "manifest.yaml"))]
        if not new:
            return None, "invalid"
        run_id = new[-1]
        with open(os.path.join(self.a.logs_dir, run_id, "manifest.yaml")) as f:
            m = yaml.safe_load(f) or {}
        outcome = m.get("outcome") or "invalid"
        self.log(f"row {r['order']} attempt {attempt}: {run_id} -> {outcome} ({m.get('reason', '')})")
        if outcome not in DATA:
            run_log_row(self.who, run_id, f"invalid: {m.get('reason', 'no reason recorded')}",
                        "rerun with the same pose" if attempt < self.a.max_retries else
                        "rerun limit reached: cell reported as missing", self.a.run_log)
        elif attempt > 0:
            run_log_row(self.who, run_id, f"rerun {attempt} of row {r['order']}", f"outcome {outcome}",
                        self.a.run_log)
        return run_id, outcome

    def go(self, rows) -> int:
        done = existing_runs(self.a.logs_dir)
        todo, retry = [], []
        for r in rows:
            prev = done.get(cell_key(r), [])
            if any(o in DATA for _, o in prev):
                continue
            n_invalid = sum(1 for _, o in prev if o not in DATA)
            (retry if n_invalid else todo).append((r, n_invalid))
        self.log(f"schedule {self.a.schedule}: {len(rows)} rows for {self.a.machine}, "
                 f"{len(todo)} to run, {len(retry)} earlier invalid, "
                 f"{len(rows) - len(todo) - len(retry)} already done")
        pending = []
        for r, n_inv in todo:
            _, outcome = self.run_row(r, 0)
            if outcome not in DATA and outcome != "dry-run":
                pending.append((r, 1))
        pending += [(r, n) for r, n in retry]
        while pending:
            r, attempt = pending.pop(0)
            if attempt > self.a.max_retries:
                self.log(f"row {r['order']}: {attempt} invalid attempts, giving up (cell missing)")
                continue
            _, outcome = self.run_row(r, attempt)
            if outcome not in DATA and outcome != "dry-run":
                pending.append((r, attempt + 1))
        self.log("batch finished")
        return 0


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    passthrough = []
    if "--" in argv:
        i = argv.index("--")
        argv, passthrough = argv[:i], argv[i + 1:]
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("schedule")
    ap.add_argument("--machine", required=True, help="this machine's name in the schedule, or `any`")
    ap.add_argument("--max-retries", type=int, default=2)
    ap.add_argument("--logs-dir", default=os.path.join(REPO, "logs"))
    ap.add_argument("--who", help="name for docs/run_log.md (default: git user.name)")
    ap.add_argument("--run-log", default=os.path.join(REPO, "docs", "run_log.md"), help=argparse.SUPPRESS)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--runner", default=os.path.join(REPO, "harness", "run.py"), help=argparse.SUPPRESS)
    a = ap.parse_args(argv)
    rows = read_schedule(a.schedule, a.machine)
    return Batch(a, passthrough).go(rows)


if __name__ == "__main__":
    sys.exit(main())

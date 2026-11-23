#!/usr/bin/env python3
"""Turn /etc/nvpmodel.conf into the Markdown table for docs/power_modes.md.

Best-effort parser: the conf format differs between L4T releases, so always
compare the output with the raw file (Module 2, step 2: VERIFY).

Usage: python3 parse_nvpmodel.py [/etc/nvpmodel.conf]
"""
import re
import sys

path = sys.argv[1] if len(sys.argv) > 1 else "/etc/nvpmodel.conf"
try:
    lines = open(path).read().splitlines()
except OSError as e:
    sys.exit(f"cannot read {path}: {e}")

modes = []
cur = None
for raw in lines:
    line = raw.strip()
    m = re.match(r"<\s*POWER_MODEL\s+ID=(\d+)\s+NAME=(\S+)\s*>", line)
    if m:
        cur = {"id": int(m.group(1)), "name": m.group(2), "cores": 0,
               "cpu_max": [], "gpu_max": None}
        modes.append(cur)
        continue
    if line.startswith("<"):           # any other section header ends the mode
        cur = None
        continue
    if cur is None or not line or line.startswith("#"):
        continue
    tok = line.split()
    if tok[0] == "CPU_ONLINE" and len(tok) >= 3 and tok[2] == "1":
        cur["cores"] += 1
    elif tok[0].startswith("CPU") and len(tok) >= 3 and tok[1] == "MAX_FREQ":
        cur["cpu_max"].append(int(tok[2]))
    elif tok[0] == "GPU" and len(tok) >= 3 and tok[1] == "MAX_FREQ":
        cur["gpu_max"] = int(tok[2])

default = None
for raw in lines:
    m = re.match(r"<\s*PM_CONFIG\s+DEFAULT=(\d+)\s*>", raw.strip())
    if m:
        default = int(m.group(1))


def mhz(v):
    if v is None or v < 0:
        return "unlimited / VERIFY"
    # conf values are kHz for CPU and Hz for GPU on Orin; normalise to MHz
    return f"{v / 1e6:.0f}" if v > 1e7 else f"{v / 1e3:.0f}"


print("| Mode ID | Name | CPU cores online | Max CPU freq (MHz) | Max GPU freq (MHz) | Default? |")
print("|---|---|---|---|---|---|")
for md in modes:
    cpu = mhz(max(md["cpu_max"])) if md["cpu_max"] else "VERIFY"
    print(f"| {md['id']} | {md['name']} | {md['cores'] or 'VERIFY'} | {cpu} | "
          f"{mhz(md['gpu_max'])} | {'yes' if md['id'] == default else ''} |")
if not modes:
    print("| (no POWER_MODEL blocks parsed; paste the raw file) | | | | | |")

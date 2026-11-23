# Jetson power modes (Module 2, step 2)

Generated with `bash scripts/jetson/collect_info.sh` (which runs `scripts/jetson/parse_nvpmodel.py /etc/nvpmodel.conf`) and **checked by hand against the raw file**. Do not assume mode IDs; they differ between JetPack releases.

The `name` column is the exact string used as `power_mode` in run IDs and CSVs (Section 0.1).

| Mode ID | Name | CPU cores online | Max CPU freq (MHz) | Max GPU freq (MHz) | Default? | Needs reboot to switch? |
|---|---|---|---|---|---|---|
| TODO | | | | | | |

Switch with `sudo nvpmodel -m <id>`, check with `sudo nvpmodel -q`.

## Chosen for Block E

| Role | Mode ID | Name | Reason |
|------|---------|------|--------|
| Lowest power | TODO | TODO | |
| Highest power | TODO | TODO | |

Verified by: TODO (who, date, L4T version).

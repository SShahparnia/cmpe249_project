# Block E session procedure (Module 19)

One session = one power mode. Follow this checklist top to bottom, every session, and tick it in a copy pasted into `docs/run_log.md` (one row per session: date, who, power mode, temperatures, idle baselines, anything odd).

Layout: **Gazebo on the laptop; SLAM, Nav2, planner, recorder and power logger on the Jetson** (`docs/sim_setup.md`). The orchestrator runs on the Jetson and starts Gazebo on the laptop over ssh.

## Before the session

- [ ] Laptop and Jetson on the direct Ethernet link (`scripts/set_static_ip.sh`), `bash scripts/jetson/link_test.sh` passes.
- [ ] Passwordless ssh from the Jetson to the laptop works: `ssh laptop true` (the name used in `--sim-ssh`).
- [ ] Both clones on the same commit (`git rev-parse HEAD` on both), patches applied, harness built.
- [ ] chrony running on both; `chronyc tracking` on the Jetson shows the laptop as source (or both on the same server). The orchestrator stores the offset in every manifest.
- [ ] Sensor setup on the laptop matches `configs/common.yaml` (`sensor_setup`); the manifest records what was detected.
- [ ] Fan profile: the one written in `docs/jetson_setup.md`. **Never** run `jetson_clocks`.

## Session

1. [ ] `sudo nvpmodel -m <id>` (ID from `docs/power_modes.md`), then reboot the Jetson.
2. [ ] `sudo nvpmodel -q` shows the intended mode name. (`harness/run.py` refuses to run otherwise.)
3. [ ] Wait for the CPU temperature to stabilise: less than 1 °C change over 5 minutes (`jtop`, or `cat /sys/class/thermal/thermal_zone*/temp`). Write the temperature down.
4. [ ] Idle-stack baseline: `bash scripts/idle_baseline.sh <power_mode> --sim-ssh laptop` (5 min, robot stationary, no planner) → `logs/idle_<mode>_<ts>/`.
5. [ ] Run the session schedule:
   ```bash
   python3 scripts/run_batch.py configs/schedules/E_<mode>.csv --machine jetson -- --jetson --sim-ssh laptop
   ```
6. [ ] Second idle baseline at the end (same command as step 4), to detect drift. If the two idle means differ by more than 5%, note it in `docs/run_log.md`; the analysis uses the mean of the two.
7. [ ] `python3 scripts/status.py` and look at the random power trace in `logs/daily/<date>/`.

## If something goes wrong

- A run ends `invalid`: `run_batch.py` reruns it at the end of the schedule (at most 2 extra attempts) and writes a row in `docs/run_log.md`. Do not delete invalid run folders.
- The Jetson rebooted or the mode changed mid-session: stop, restart the session from step 1. Runs already finished with a data outcome are kept (`run_batch.py` skips them).
- Thermal throttling (CPU temperature above ~85 °C or the clock dropping in `jtop`): pause, note it, let it cool to the step-3 temperature, continue.

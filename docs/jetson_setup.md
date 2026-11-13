# Jetson setup (Module 2)

Goal: someone can re-flash and set up the board from this page without asking questions.
Fill every `TODO` while doing it; paste the output of `scripts/jetson/collect_info.sh` at the bottom.

## 1. Flash

| Item | Value |
|------|-------|
| Board | Jetson Orin Nano Developer Kit (8 GB): TODO confirm from `/proc/device-tree/model` |
| JetPack | TODO (6.x, Super configuration) |
| L4T (`cat /etc/nv_tegra_release`) | TODO |
| Method | TODO: SDK Manager vX.Y on host `<host OS>`, **or** SD-card image `<file name>` |
| Storage device | TODO (microSD / NVMe, model, size) |
| Date / who | TODO |

Commands run (in order):

```bash
# TODO: paste exactly what was run, e.g. Etcher/dd command or SDK Manager steps
```

## 2. Software

```bash
git clone --recursive https://github.com/SShahparnia/cmpe249_project.git
cd cmpe249_project
bash scripts/jetson/setup_jetson.sh 2>&1 | tee jetson_setup_$(date +%Y%m%d).log
sudo reboot
```

`setup_jetson.sh` installs ROS 2 Humble (ros-base), `slam_toolbox`, Nav2, `turtlebot4_navigation`, CycloneDDS, `stress-ng`, `chrony`, and `jetson-stats` (jtop), and adds `scripts/ros_env.sh` to `~/.bashrc`.

After reboot: `jtop` opens and shows power rails. TODO confirm.

## 3. Clocks and fan

- **Never run `jetson_clocks` during experiments.** It locks clocks at max and overrides the power mode. `collect_info.sh` prints `jetson_clocks --show` so we can check.
- Fan profile used for **every** block: TODO (e.g. `cool` in `/etc/nvfancontrol.conf`, `FAN_DEFAULT_PROFILE cool`, then `sudo systemctl restart nvfancontrol`).

## 4. Network

```bash
bash scripts/set_static_ip.sh <iface> jetson     # 192.168.249.2/24
echo 'export EB_IFACE=<iface>' >> ~/.bashrc        # must come BEFORE the ros_env.sh line
```

Interface name on our board: TODO.

## 5. Time sync (Section 0.3)

The Jetson is the reference clock in Block E. TODO once Module 3 is done:

- Jetson `/etc/chrony/chrony.conf`: add `allow 192.168.249.0/24` and `local stratum 10`, then `sudo systemctl restart chrony`.
- Laptop `/etc/chrony/chrony.conf`: add `server 192.168.249.2 iburst prefer`, restart chrony.
- Check on the laptop: `chronyc tracking` (record the offset in each run manifest).

## 6. Power-logging smoke test (steps 6–8)

```bash
python3 scripts/power_smoke_test.py --list-rails      # VERIFY rail names
bash scripts/jetson/power_smoke_run.sh                # 5 min idle + 5 min stress-ng --cpu 6
```

Results (paste the table it prints):

| phase | rows | median dt (ms) | achieved rate (Hz) | mean VDD_IN (mW) | sd (mW) |
|---|---|---|---|---|---|
| idle | TODO | | | | |
| stress | TODO | | | | |

- Rail names reported by jtop: TODO (expected `VDD_IN`, `VDD_CPU_GPU_CV`, `VDD_SOC`).
- Power mode during the test: TODO.
- If the achieved rate is well below 10 Hz: INA3221 sysfs path on our board: TODO (`ls /sys/bus/i2c/drivers/ina3221/*/hwmon/hwmon*/`).
- Figure: [`figures/power_smoke.png`](figures/power_smoke.png)

## 7. Raw `collect_info.sh` output

```
TODO: paste
```

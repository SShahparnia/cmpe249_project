# M2 ×2: office corridor at twice the area (`m2_office_2x`, Module 21, optional)

The Module 21 scaling ablation: M2's layout repeated twice along the corridor (`python3 maps/tools/make_office.py --copies 2 --name m2_office_2x`), not redesigned. A 24 m × 10.3 m office with ten rooms. It exists to push `n_frontiers` higher and see whether MRTSP's decision latency crosses `replan_period_s` (H2).

**Exploratory.** It was set up after the pilot was planned; unless `docs/preregistration.md` lists Block A before `prereg-v1`, its results are reported as exploratory, not as a pre-registered test.

| File | What |
|------|------|
| `m2_office_2x.sdf` | the world (same generator and constants as M2) |
| `gt_config.yaml`, `gt.*`, `gt_reachable.npy`, `gt_info.yaml` | ground truth: `python3 maps/tools/world_to_gt.py maps/m2_office_2x` |
| `../../configs/start_poses/m2_office_2x.yaml` | 20 start poses, seed 249 (Block A uses 0 to 4) |

Run it: `python3 scripts/make_schedule.py --block A --poses 5 --power-modes <high mode>`, then `scripts/run_batch.py configs/schedules/A_<mode>.csv --machine jetson -- --jetson --sim-ssh laptop`; `python3 analysis/make_figures.py` adds `fig_h2_ablation.png` and `numbers.ablation_h2`.

## Cell counts (0.05 m cells)

| Grid | Free | Occupied | Reachable free | Coverage reference |
|------|------|----------|----------------|--------------------|
| 524 × 244 (26.2 × 12.2 m) | 104,326 | 23,530 | 74,254 (185.64 m², 2.0× M2) | 79,614 |

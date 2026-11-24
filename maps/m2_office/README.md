# M2: office corridor (`m2_office`)

> **Frozen after the `prereg-v1` tag.** Do not change `m2_office.sdf`, `gt_config.yaml`, the generator constants in `maps/tools/make_office.py`, or the ground-truth files after that tag. If something must change, it is a protocol deviation: record it in `docs/run_log.md` and the report.

Built for this project (Module 10 step 2): a 12 m × 10.3 m single-storey office. A 2 m corridor runs east-west through the middle. Three 4 m rooms lie to the north and two 6 m rooms to the south, each opening onto the corridor through a 0.9 m doorway. The middle and east north rooms also connect through an inner doorway, so there is one loop. A cabinet narrows the corridor to 1.6 m in one place. Furniture (desks, tables, racks, a column) is modelled as solid boxes, so Gazebo's lidar and the ground truth see exactly the same shapes.

| File | What |
|------|------|
| `m2_office.sdf` | the world, generated: `python3 maps/tools/make_office.py` |
| `gt_config.yaml` | ground-truth inputs (seed = corridor centre, the default spawn) |
| `gt.pgm`, `gt.yaml`, `gt_reachable.npy`, `gt_info.yaml`, `gt.png` | ground truth: `python3 maps/tools/world_to_gt.py maps/m2_office` |
| `overlay_slam.png` | SLAM map from a sim run over the ground truth (alignment check, below) |

Launch: `ros2 launch harness/launch/tb4_sim.launch.py world:=m2_office gui:=false slam:=true nav2:=true`.

## Cell counts (0.05 m cells, from `gt_info.yaml`)

| Grid | Free | Occupied | Reachable free | Coverage reference (reachable + bordering obstacles) |
|------|------|----------|----------------|------------------------------------------------------|
| 284 × 244 (14.2 × 12.2 m) | 57,036 | 12,260 | 37,038 (92.6 m²) | 39,752 |

"Free" includes the 1 m margin outside the building, which is never reachable.

## Alignment check (Module 10 "Done when")

`overlay_slam.png`: final map of a TB4 waypoint tour in WSL (2026-11-23; spawn at (0, 0, 0), stock TB4 slam_toolbox, software rendering, RTF ≈ 0.25), drawn by `python3 analysis/coverage.py <run> --overlay`. Green = GT reference cells the SLAM map knows, red = not yet seen. After 41 m the tour had covered 79% of the reference cells (corridor and the north rooms; it was stopped before the south rooms were finished). The corridor walls (y = ±1 m) and the room partitions coincide with the ground truth, so the GT frame and resolution are right. The map also shows scan-matching artefacts: a rotated copy of the east end wall outside the building and smeared walls in the north-west room, from in-place turns at the low real-time factor. They are SLAM errors, not GT errors, and they are one more reason to run the experiments at an RTF close to 1 (`docs/common_rules.md`).

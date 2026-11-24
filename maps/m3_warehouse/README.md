# M3: AWS RoboMaker small warehouse (`m3_warehouse`)

> **Frozen after the `prereg-v1` tag** (world, `gt_config.yaml`, ground truth, submodule commit).

| | |
|---|---|
| Source | [aws-robotics/aws-robomaker-small-warehouse-world](https://github.com/aws-robotics/aws-robomaker-small-warehouse-world), branch `ros2`, commit `ee0af733315e78432408c3cd98d378ecee5f767c` (release 1.0.5) |
| Submodule | `maps/m3_warehouse/aws_world` (models and the Classic world, unchanged) |
| World | `m3_warehouse.sdf` from the **no-roof** variant (the roof only costs rendering time; the lidar never sees it): `python3 maps/tools/convert_aws_world.py maps/m3_warehouse/aws_world/worlds/no_roof_small_warehouse/no_roof_small_warehouse.world --name m3_warehouse --floor -9 9 -13 13` |
| Ground truth | `python3 maps/tools/world_to_gt.py maps/m3_warehouse` (seed (0, 0), central aisle) |
| Licence | MIT-0 (the AWS repo) |

What the conversion changes: as for M1 (Fortress physics and plugins, models inlined, static, no `<inertial>`, world name `m3_warehouse`, tiled visual floor). Meshes, model poses and the light are the AWS originals.

The warehouse models reference meshes as `file://models/<model>/meshes/...`, resolved against the resource path; `harness/launch/tb4_sim.launch.py` puts both `aws_world/models` and `aws_world` on it, and `gt_config.yaml` lists the same two folders.

Visual and collision geometry differ a lot here: the shelves are solid blocks for collision but open racks visually. At 0.19 m the lidar sees through the west shelves (only their end posts are occupied in `gt.pgm`), while the robot cannot drive under them (they are excluded from `gt_reachable.npy`). See `gt.png`: yellow = blocks the robot but not the lidar.

## Alignment check

`overlay_aws_map.png`: AWS's reference map `005` (a cleaned 0.05 m map aligned to the world) over our ground truth. Walls, shelf rows and desks coincide in position, scale and orientation; the score (40.5% within one cell) is low because that map draws object outlines about 0.1 m outside the geometry and includes the lowest shelf boards, which a lidar at a lower height sees. Map `002` is at 0.02 m with its own origin and is not world-aligned. A TB4 SLAM run in our Fortress world gives the final check (`overlay_slam.png`).

## Cell counts (0.05 m cells, from `gt_info.yaml`)

| Grid | Free | Occupied (visual, at 0.19 m) | Reachable free | Coverage reference (reachable + bordering obstacles) |
|------|------|------------------------------|----------------|------------------------------------------------------|
| 320 × 460 (16.0 × 23.0 m) | 128,230 | 18,970 | 94,214 (235.54 m²) | 96,643 |

"Free" includes the 1 m margin and everything outside the building, which is never reachable.

# M1: AWS RoboMaker small house (`m1_house`)

> **Frozen after the `prereg-v1` tag** (world, `gt_config.yaml`, ground truth, submodule commit).

| | |
|---|---|
| Source | [aws-robotics/aws-robomaker-small-house-world](https://github.com/aws-robotics/aws-robomaker-small-house-world), branch `ros2`, commit `ff9631ca6d1db9c1ba656498151464b5ab74aafe` (the repo is archived) |
| Submodule | `maps/m1_house/aws_world` (models and the Classic world, unchanged) |
| World | `m1_house.sdf`, converted for Ignition Fortress: `python3 maps/tools/convert_aws_world.py maps/m1_house/aws_world/worlds/small_house.world --name m1_house --floor -12 12 -8 9` |
| Ground truth | `python3 maps/tools/world_to_gt.py maps/m1_house` (seed (0, 0), living room) |
| Licence | MIT-0 (the AWS repo) |

What the conversion changes (details in `maps/tools/convert_aws_world.py`): Fortress physics and system plugins instead of the Classic ones; every model inlined into the world file, static and without its `<inertial>` block (two of the house models have a zero principal moment of inertia, which makes sdformat reject the whole world; static models never use inertia); world name `m1_house`; a tiled visual floor (`maps/tools/floor.py`). Meshes, model poses and lights are the AWS originals. Loads in Fortress 6.18 with the TB4 (WSL check, 2026-11-23).

## Alignment check

`overlay_aws_map.png`: the SLAM map AWS ships with the world (`maps/turtlebot3_waffle_pi/map.yaml`, built by a TurtleBot3 in Gazebo Classic) over our ground truth. **93.6%** of its occupied cells lie within one cell (5 cm) of a GT occupied cell; the rest are furniture seen at the TurtleBot3's slightly different lidar height. This checks the COLLADA unit scaling and every model pose. A TB4 SLAM run in our Fortress world gives the final check (`overlay_slam.png`, Module 10 "Done when").

## Cell counts (0.05 m cells, from `gt_info.yaml`)

| Grid | Free | Occupied (visual, at 0.19 m) | Reachable free | Coverage reference (reachable + bordering obstacles) |
|------|------|------------------------------|----------------|------------------------------------------------------|
| 420 × 270 (21.0 × 13.5 m) | 103,377 | 10,023 | 61,034 (152.59 m²) | 62,691 |

"Free" includes the 1 m margin and everything outside the building, which is never reachable.

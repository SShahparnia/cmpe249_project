# maps

World files and ground truth for every map (Module 10). Map names are the ones in Section 0.1 of `instructions.md`.

## Layout of one map folder

```
maps/<map>/
  <map>.sdf            Ignition Fortress world. The file name and <world name> must equal <map>:
                       the TB4 bridges subscribe to /world/<map>/...
  gt_config.yaml       inputs to the ground-truth tool (world, model paths, heights, seed)
  gt.pgm, gt.yaml      ground-truth occupancy grid, map_server format (image row 0 = top)
  gt_reachable.npy     bool mask of reachable free cells, same shape and row order as gt.pgm
  gt_info.yaml         cell counts, parameters, sha256 of the world file and of the tool
  gt.png               figure of the grid
  README.md            source, commit, cell counts, notes
```

## Ground truth: `tools/world_to_gt.py`

Option A of Module 10 step 3: the world geometry is cut by horizontal planes and rasterized at the slam_toolbox resolution.

- **Occupancy (`gt.pgm`)** is the cut of the **visual** geometry at lidar height. The TB4 lidar is a Gazebo `gpu_lidar`, which renders visuals, so this is what SLAM can map. Collision meshes are often simplified (the AWS warehouse shelves are solid blocks in collision but open racks in their visuals).
- **Reachability (`gt_reachable.npy`)** uses the **collision** geometry cut at several robot-body heights, plus everything the lidar sees: space the robot cannot drive into is excluded even if the lidar sees through it.

```bash
python3 maps/tools/world_to_gt.py maps/<map>          # rewrites gt.* in maps/<map>/
python3 -m pytest maps/tools/tests                     # synthetic-world tests
```

| Parameter (`gt_config.yaml`) | Value | Why |
|------------------------------|-------|-----|
| `resolution` | 0.05 m | `resolution` in slam_toolbox's `slam.yaml` (TB4 navigation package) |
| `lidar_height` | 0.19 m | From the expanded TB4 standard URDF (`xacro turtlebot4.urdf.xacro gazebo:=ignition`): `base_link` → `shell_link` 0.0942 m, → `rplidar_link` 0.0987 m, total 0.193 m. The wheel axle is 0.040 m above `base_link`, so `base_link` is at floor level within a few mm |
| `body_heights` | 0.05 to 0.33 m | Anything the robot body hits between the floor and the top plate blocks driving, even when the lidar sees over it |
| `seal_radius` | 0.10 m | Gaps narrower than 0.2 m (mesh seams, slots behind furniture) are closed before the flood fill. The TB4 is 0.34 m wide, so it cannot use them |
| `seed` | per map | A world point in the explorable space. The flood fill starts there. The tool refuses to run if that region touches the grid border (world not closed) |
| `lidar_geometry` | visual | `collision` to cut collision shapes for occupancy too (both give the same grid for M2, whose boxes have identical visuals and collisions) |

What counts in the coverage denominator: reachable free cells (`gt_reachable.npy`) plus the obstacle cells that border them, which `analysis/coverage.py` adds itself. Free cells inside furniture, closed rooms or outside the building are excluded.

Known limits of a single-plane cut:

- Thin objects that only exist above or below 0.19 m (table tops, chair seats) do not appear in `gt.pgm`. Their legs do if they cross the plane. slam_toolbox sees the same thing, so they do not distort coverage.
- Mesh collisions that are not closed at the cut are drawn as outlines only. Their insides stay "free" but are not reachable, so they drop out of the denominator anyway.
- COLLADA `<unit meter>` is applied (trimesh ignores it; Gazebo does not). Every map must still be checked against a real SLAM map: the overlay figure in Module 10's "Done when".

## World generators

| Tool | Map |
|------|-----|
| `tools/make_office.py` | M2 office corridor (`--copies 2 --name m2_office_2x` for the Module 21 ablation) |
| `tools/convert_aws_world.py` | M1 house, M3 warehouse from the AWS RoboMaker Classic worlds |
| `tools/floor.py` | shared: the visual floor is a grid of 2 m box tiles. The TB4's cliff sensors are short downward `gpu_lidar` rays; under software rendering (WSL, no GPU) they miss a floor drawn as one big surface (a `<plane>` or one 100 m box) and report a cliff everywhere, and the Create 3's cliff reflex then blocks every Nav2 command. Tiles fix it (tested in WSL, 2026-11-23). |

Dependencies: `maps/tools/requirements.txt`.

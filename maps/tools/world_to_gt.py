#!/usr/bin/env python3
"""Ground-truth grids from a Gazebo world (Module 10 of instructions.md, steps 3 to 5).

    python3 maps/tools/world_to_gt.py maps/<map>            # reads maps/<map>/gt_config.yaml

Option A of step 3: rasterize the world geometry at lidar height into a 2D grid, at the
slam_toolbox resolution. Writes into maps/<map>/:

    gt.pgm, gt.yaml       map_server format (image row 0 = top), every cell free or occupied
    gt_reachable.npy      bool, same shape and row order as gt.pgm: reachable free cells
    gt_info.yaml          cell counts, parameters, and hashes of the world and of this tool
    gt.png                figure: occupied / reachable / unreachable free

Occupancy is the cross-section of the VISUAL geometry at `lidar_height`: the TB4 lidar is
a Gazebo gpu_lidar, which renders visuals, so that is what SLAM can map (set
`lidar_geometry: collision` to use collision shapes instead). Reachability (step 4) uses a
stricter "blocking" grid: the union of the COLLISION cross-sections at `body_heights`
(anything the robot body would hit) and the lidar-height occupancy (anything Nav2 would
see as an obstacle). Blocking cells are grown by `seal_radius`, the free
component containing `seed` is flood-filled (4-connected), grown back by the same radius,
and intersected with the lidar-height free cells. Growing and shrinking by the same radius
closes gaps narrower than 2 * seal_radius (mesh seams, gaps behind furniture) without
eating into open floor. The coverage evaluator adds the obstacle cells that border the
reachable set itself (analysis/coverage.py, GroundTruth.reference_mask).

gt_config.yaml (all paths relative to the map folder):

    world: m2_office.sdf
    model_paths: []                 # folders searched for model://<name>
    resolution: 0.05                # = slam_toolbox resolution (configs/slam_toolbox.yaml)
    lidar_height: 0.19              # TB4 rplidar above the floor
    body_heights: [0.05, 0.12, 0.19, 0.26, 0.33]
    seal_radius: 0.10
    seed: [0.0, 0.0]                # world x, y inside the explorable space
    margin: 1.0                     # m of free border around the geometry
    exclude: []                     # regexes on model instance names to ignore
    lidar_geometry: visual          # visual (what gpu_lidar sees) or collision

Geometry supported: box, cylinder, sphere, mesh (any format trimesh reads; COLLADA
<unit meter> is applied, as Gazebo does), nested <model> and <include model://...>.
Planes (ground) are skipped. Needs numpy, scipy, PyYAML, trimesh; COLLADA needs pycollada.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import hashlib
import math
import os
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np
import yaml
from scipy import ndimage

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, os.path.join(REPO, "analysis"))
import coverage as cov  # noqa: E402  (write_pgm, GroundTruth: the evaluator's own reader)

DEFAULTS = {
    "model_paths": [],
    "resolution": 0.05,
    "lidar_height": 0.19,
    "body_heights": [0.05, 0.12, 0.19, 0.26, 0.33],
    "seal_radius": 0.10,
    "seed": None,
    "margin": 1.0,
    "exclude": [],
    "lidar_geometry": "visual",
}
FREE_PX, OCC_PX = 254, 0


# ------------------------------------------------------------------ transforms

def pose_matrix(text: str | None) -> np.ndarray:
    """SDF <pose>x y z roll pitch yaw</pose> -> 4x4 (fixed-axis XYZ, i.e. R = Rz Ry Rx)."""
    v = [float(t) for t in (text or "").split()] + [0.0] * 6
    x, y, z, r, p, yw = v[:6]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(yw), math.sin(yw)
    T = np.eye(4)
    T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    T[:3, 3] = [x, y, z]
    return T


def _child_pose(el) -> np.ndarray:
    p = el.find("pose")
    return pose_matrix(p.text if p is not None else None)


# ------------------------------------------------------------------- SDF walk

@dataclass
class Shape:
    name: str               # model instance path, e.g. "Bed_01_001/link/collision"
    kind: str               # box, cylinder, sphere, mesh
    params: dict
    T: np.ndarray           # 4x4 world <- geometry
    role: str = "collision" # collision or visual


@dataclass
class World:
    shapes: list = field(default_factory=list)
    warnings: list = field(default_factory=list)
    models_used: set = field(default_factory=set)


def resolve_model_uri(uri: str, model_paths, base_dir: str) -> str | None:
    """model://name[/sub/path], or a relative / file:// path -> absolute path, or None.
    Relative paths are tried against the referring file's folder, then every resource
    path, like Gazebo does (the AWS warehouse models use file://models/<model>/...)."""
    uri = uri.strip()
    if uri.startswith("model://"):
        rest = uri[len("model://"):]
        for mp in model_paths:
            cand = os.path.join(mp, *rest.split("/"))
            if os.path.exists(cand):
                return cand
        return None
    if uri.startswith("file://"):
        uri = uri[len("file://"):]
    if os.path.isabs(uri):
        return uri if os.path.exists(uri) else None
    for d in [base_dir] + list(model_paths):
        cand = os.path.join(d, *uri.split("/"))
        if os.path.exists(cand):
            return cand
    return None


def _model_sdf_file(model_dir: str) -> str:
    cfg = os.path.join(model_dir, "model.config")
    if os.path.exists(cfg):
        try:
            sdfs = ET.parse(cfg).getroot().findall("sdf")
            if sdfs:
                return os.path.join(model_dir, sdfs[-1].text.strip())
        except ET.ParseError:
            pass
    return os.path.join(model_dir, "model.sdf")


def _walk_model(model_el, T_parent, prefix, model_dir, world: World, model_paths, exclude, pose_override=None):
    name = model_el.get("name", "model")
    path = f"{prefix}/{name}" if prefix else name
    if any(re.search(rx, path) for rx in exclude):
        return
    T_model = T_parent @ (pose_override if pose_override is not None else _child_pose(model_el))
    for link in model_el.findall("link"):
        T_link = T_model @ _child_pose(link)
        for role in ("collision", "visual"):
            for el in link.findall(role):
                geom = el.find("geometry")
                if geom is None:
                    continue
                _add_geometry(geom, T_link @ _child_pose(el),
                              f"{path}/{link.get('name', 'link')}/{el.get('name', role)}",
                              model_dir, world, model_paths, role)
    for sub in model_el.findall("model"):
        _walk_model(sub, T_model, path, model_dir, world, model_paths, exclude)
    for inc in model_el.findall("include"):
        _walk_include(inc, T_model, path, model_dir, world, model_paths, exclude)


def _walk_include(inc, T_parent, prefix, base_dir, world: World, model_paths, exclude):
    uri = (inc.findtext("uri") or "").strip()
    mdir = resolve_model_uri(uri, model_paths, base_dir)
    if mdir is None:
        world.warnings.append(f"cannot resolve include {uri!r} (model_paths={model_paths})")
        return
    sdf_file = _model_sdf_file(mdir) if os.path.isdir(mdir) else mdir
    root = ET.parse(sdf_file).getroot()
    model = root.find("model") if root.tag == "sdf" else root
    if model is None:
        world.warnings.append(f"{sdf_file}: no <model>")
        return
    world.models_used.add(uri)
    pose_el = inc.find("pose")
    override = pose_matrix(pose_el.text) if pose_el is not None else None
    name_el = inc.find("name")
    if name_el is not None:
        model = _renamed(model, name_el.text.strip())
    _walk_model(model, T_parent, prefix, os.path.dirname(sdf_file), world, model_paths, exclude,
                pose_override=override)


def _renamed(el, name):
    el = ET.fromstring(ET.tostring(el))
    el.set("name", name)
    return el


def _add_geometry(geom, T, name, model_dir, world: World, model_paths, role="collision"):
    for kind in ("box", "cylinder", "sphere", "mesh", "plane", "capsule", "ellipsoid", "heightmap", "polyline"):
        g = geom.find(kind)
        if g is None:
            continue
        if kind == "box":
            size = [float(v) for v in g.findtext("size", "1 1 1").split()]
            world.shapes.append(Shape(name, "box", {"size": size}, T, role))
        elif kind == "cylinder":
            world.shapes.append(Shape(name, "cylinder", {"radius": float(g.findtext("radius", "1")),
                                                         "length": float(g.findtext("length", "1"))}, T, role))
        elif kind == "sphere":
            world.shapes.append(Shape(name, "sphere", {"radius": float(g.findtext("radius", "1"))}, T, role))
        elif kind == "mesh":
            uri = g.findtext("uri", "").strip()
            f = resolve_model_uri(uri, model_paths, model_dir)
            if f is None:
                world.warnings.append(f"{name}: cannot resolve mesh {uri!r}")
                return
            scale = [float(v) for v in g.findtext("scale", "1 1 1").split()]
            world.shapes.append(Shape(name, "mesh", {"file": f, "scale": scale}, T, role))
        elif kind == "plane":
            pass   # ground plane: never an obstacle at lidar height
        else:
            world.warnings.append(f"{name}: geometry <{kind}> not supported, skipped")
        return


def load_world(world_file: str, model_paths=(), exclude=()) -> World:
    world = World()
    root = ET.parse(world_file).getroot()
    w = root.find("world")
    if w is None:
        raise ValueError(f"{world_file}: no <world>")
    base = os.path.dirname(os.path.abspath(world_file))
    I = np.eye(4)
    for m in w.findall("model"):
        _walk_model(m, I, "", base, world, list(model_paths), list(exclude))
    for inc in w.findall("include"):
        _walk_include(inc, I, "", base, world, list(model_paths), list(exclude))
    return world


# --------------------------------------------------------------- geometry ops

_MESH_CACHE: dict = {}


def _collada_unit(path: str) -> float:
    if not path.lower().endswith(".dae"):
        return 1.0
    with open(path, "rb") as f:
        head = f.read(4096).decode("utf-8", "ignore")
    m = re.search(r'<unit[^>]*meter="([0-9.eE+-]+)"', head)
    return float(m.group(1)) if m else 1.0


def shape_mesh(s: Shape):
    """trimesh.Trimesh in the world frame."""
    import trimesh
    if s.kind == "box":
        m = trimesh.creation.box(extents=s.params["size"])
    elif s.kind == "cylinder":
        m = trimesh.creation.cylinder(radius=s.params["radius"], height=s.params["length"], sections=48)
    elif s.kind == "sphere":
        m = trimesh.creation.icosphere(subdivisions=3, radius=s.params["radius"])
    else:
        f = s.params["file"]
        if f not in _MESH_CACHE:
            loaded = trimesh.load(f, force="mesh")
            loaded.apply_scale(_collada_unit(f))     # Gazebo applies COLLADA <unit>; trimesh does not
            _MESH_CACHE[f] = loaded
        m = _MESH_CACHE[f].copy()
        S = np.diag(list(s.params["scale"]) + [1.0])
        m.apply_transform(S)
    m.apply_transform(s.T)
    return m


def section_segments(mesh, z: float) -> np.ndarray:
    """(n, 2, 2) xy line segments where the plane at height z cuts the mesh."""
    import trimesh
    lo, hi = mesh.bounds[:, 2]
    if not (lo < z < hi):
        return np.zeros((0, 2, 2))
    segs = trimesh.intersections.mesh_plane(mesh, plane_normal=[0, 0, 1], plane_origin=[0, 0, z])
    return np.asarray(segs)[:, :, :2] if len(segs) else np.zeros((0, 2, 2))


def closed_components(segs: np.ndarray, tol: float = 1e-5) -> np.ndarray:
    """Bool per segment: does it belong to a closed loop set (every vertex of its connected
    component has even degree)? Only those are filled; the rest are drawn as outlines."""
    n = len(segs)
    if n == 0:
        return np.zeros(0, dtype=bool)
    pts = np.round(segs.reshape(-1, 2) / tol).astype(np.int64)
    _, vid = np.unique(pts, axis=0, return_inverse=True)
    vid = vid.reshape(-1)
    nv = vid.max() + 1
    parent = np.arange(nv)

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    e = vid.reshape(n, 2)
    for a, b in e:
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[ra] = rb
    roots = np.array([find(v) for v in range(nv)])
    deg = np.bincount(vid, minlength=nv)
    bad_roots = set(roots[deg % 2 == 1].tolist())
    seg_root = roots[e[:, 0]]
    degenerate = e[:, 0] == e[:, 1]
    return np.array([r not in bad_roots for r in seg_root]) & ~degenerate


# ------------------------------------------------------------------ raster ops

@dataclass
class Grid:
    x0: float
    y0: float
    res: float
    rows: int
    cols: int

    def blank(self):
        return np.zeros((self.rows, self.cols), dtype=bool)

    def cell(self, x, y):
        return (np.floor((np.asarray(y) - self.y0) / self.res).astype(int),
                np.floor((np.asarray(x) - self.x0) / self.res).astype(int))


def draw_segments(mask: np.ndarray, g: Grid, segs: np.ndarray) -> None:
    """Mark every cell a segment passes through (dense sampling at res / 4)."""
    if len(segs) == 0:
        return
    a, b = segs[:, 0], segs[:, 1]
    n = np.maximum(2, np.ceil(np.linalg.norm(b - a, axis=1) / (g.res / 4)).astype(int) + 1)
    idx = np.repeat(np.arange(len(segs)), n)
    t = np.concatenate([np.linspace(0, 1, k) for k in n])
    p = a[idx] + (b[idx] - a[idx]) * t[:, None]
    r, c = g.cell(p[:, 0], p[:, 1])
    ok = (r >= 0) & (r < g.rows) & (c >= 0) & (c < g.cols)
    mask[r[ok], c[ok]] = True


def fill_evenodd(mask: np.ndarray, g: Grid, segs: np.ndarray) -> None:
    """Scanline even-odd fill of closed loops (handles holes and nesting) at cell centres."""
    if len(segs) == 0:
        return
    ay, by = segs[:, 0, 1], segs[:, 1, 1]
    ax, bx = segs[:, 0, 0], segs[:, 1, 0]
    r_lo = max(0, int(math.floor((min(ay.min(), by.min()) - g.y0) / g.res)))
    r_hi = min(g.rows - 1, int(math.ceil((max(ay.max(), by.max()) - g.y0) / g.res)))
    xc = g.x0 + (np.arange(g.cols) + 0.5) * g.res
    for r in range(r_lo, r_hi + 1):
        yc = g.y0 + (r + 0.5) * g.res
        cross = ((ay <= yc) & (by > yc)) | ((by <= yc) & (ay > yc))
        if not cross.any():
            continue
        xs = np.sort(ax[cross] + (yc - ay[cross]) * (bx[cross] - ax[cross]) / (by[cross] - ay[cross]))
        for x_in, x_out in zip(xs[0::2], xs[1::2]):
            mask[r, (xc >= x_in) & (xc < x_out)] = True


def rasterize_slice(meshes, z: float, g: Grid) -> np.ndarray:
    occ = g.blank()
    for m in meshes:
        segs = section_segments(m, z)
        if len(segs) == 0:
            continue
        closed = closed_components(segs)
        fill_evenodd(occ, g, segs[closed])
        draw_segments(occ, g, segs)
    return occ


def square(r_cells: int) -> np.ndarray:
    return np.ones((2 * r_cells + 1, 2 * r_cells + 1), dtype=bool)


def reachable_mask(free: np.ndarray, blocking: np.ndarray, seed_rc, seal_cells: int) -> np.ndarray:
    """Free cells the robot can get to from the seed cell (see module docstring)."""
    se = square(seal_cells) if seal_cells > 0 else np.ones((1, 1), dtype=bool)
    drive = ~ndimage.binary_dilation(blocking, structure=se)
    labels, _ = ndimage.label(drive)        # 4-connected
    r, c = seed_rc
    if not (0 <= r < free.shape[0] and 0 <= c < free.shape[1]) or labels[r, c] == 0:
        raise ValueError(f"seed cell {seed_rc} is outside the grid or within seal_radius of an obstacle")
    region = labels == labels[r, c]
    if region[0, :].any() or region[-1, :].any() or region[:, 0].any() or region[:, -1].any():
        raise ValueError("the seed's free region touches the grid border: the world is not closed "
                         "at robot height (open door to the outside?), or the seed is outside")
    return ndimage.binary_dilation(region, structure=se) & free


# ----------------------------------------------------------------------- main

def sha256(path: str) -> str:
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def load_config(map_dir: str) -> dict:
    with open(os.path.join(map_dir, "gt_config.yaml")) as f:
        cfg = dict(DEFAULTS, **(yaml.safe_load(f) or {}))
    if "world" not in cfg:
        raise ValueError("gt_config.yaml needs `world`")
    if cfg["seed"] is None:
        raise ValueError("gt_config.yaml needs `seed: [x, y]` (a world point inside the explorable space)")
    return cfg


def build(map_dir: str, cfg: dict) -> dict:
    world_file = os.path.join(map_dir, cfg["world"])
    model_paths = [os.path.normpath(os.path.join(map_dir, p)) for p in cfg["model_paths"]]
    world = load_world(world_file, model_paths, cfg["exclude"])
    if cfg["lidar_geometry"] not in ("visual", "collision"):
        raise ValueError("lidar_geometry must be visual or collision")
    col = [shape_mesh(s) for s in world.shapes if s.role == "collision"]
    vis = [shape_mesh(s) for s in world.shapes if s.role == "visual"]
    lidar_meshes = vis if cfg["lidar_geometry"] == "visual" else col

    # Grid bounds: everything any slice touches, plus the margin.
    lo, hi = np.array([np.inf, np.inf]), -np.array([np.inf, np.inf])
    for meshes, heights in ((lidar_meshes, [cfg["lidar_height"]]), (col, cfg["body_heights"])):
        for m in meshes:
            zlo, zhi = m.bounds[:, 2]
            if any(zlo < z < zhi for z in heights):
                lo, hi = np.minimum(lo, m.bounds[0, :2]), np.maximum(hi, m.bounds[1, :2])
    if not np.all(np.isfinite(lo)):
        raise ValueError("no geometry crosses the slice heights")
    res, margin = float(cfg["resolution"]), float(cfg["margin"])
    x0 = math.floor((lo[0] - margin) / res) * res
    y0 = math.floor((lo[1] - margin) / res) * res
    g = Grid(x0, y0, res, int(math.ceil((hi[1] + margin - y0) / res)), int(math.ceil((hi[0] + margin - x0) / res)))

    occ = rasterize_slice(lidar_meshes, cfg["lidar_height"], g)
    blocking = occ.copy()
    for z in cfg["body_heights"]:
        blocking |= rasterize_slice(col, z, g)
    free = ~occ
    seal = int(round(float(cfg["seal_radius"]) / res))
    seed_rc = tuple(int(v) for v in g.cell(*cfg["seed"]))
    reach = reachable_mask(free, blocking, seed_rc, seal)
    return {"grid": g, "occ": occ, "blocking": blocking, "reach": reach, "world": world,
            "world_file": world_file, "seed_rc": seed_rc}


def write(map_dir: str, cfg: dict, out: dict) -> dict:
    g, occ, reach = out["grid"], out["occ"], out["reach"]
    img = np.where(occ, OCC_PX, FREE_PX).astype(np.uint8)
    cov.write_pgm(os.path.join(map_dir, "gt.pgm"), np.flipud(img))     # image row 0 = top
    np.save(os.path.join(map_dir, "gt_reachable.npy"), np.flipud(reach))
    meta = {"image": "gt.pgm", "mode": "trinary", "resolution": g.res,
            "origin": [round(g.x0, 6), round(g.y0, 6), 0.0], "negate": 0,
            "occupied_thresh": 0.65, "free_thresh": 0.25}
    with open(os.path.join(map_dir, "gt.yaml"), "w") as f:
        yaml.safe_dump(meta, f, sort_keys=False)

    # Counts through the evaluator's own loader, so they match what coverage.py uses.
    gt = cov.GroundTruth.load(map_dir)
    ref = gt.reference_mask()
    info = {
        "generated": _dt.datetime.now().strftime("%Y-%m-%d %H:%M"),
        "world": cfg["world"],
        "world_sha256": sha256(out["world_file"]),
        "tool_sha256": sha256(os.path.abspath(__file__)),
        "params": {k: cfg[k] for k in ("resolution", "lidar_height", "body_heights", "seal_radius",
                                       "seed", "margin", "exclude", "lidar_geometry")},
        "grid": {"rows": g.rows, "cols": g.cols, "origin": meta["origin"],
                 "size_m": [round(g.cols * g.res, 2), round(g.rows * g.res, 2)]},
        "cells": {
            "free": int((~occ).sum()),
            "occupied": int(occ.sum()),
            "reachable_free": int(reach.sum()),
            "reference": int(ref.sum()),            # coverage denominator
            "reference_obstacle": int((ref & gt.occupied).sum()),
        },
        "areas_m2": {"reachable_free": round(float(reach.sum()) * g.res ** 2, 2),
                     "reference": round(float(ref.sum()) * g.res ** 2, 2)},
        "n_shapes": {r: sum(1 for s in out["world"].shapes if s.role == r) for r in ("collision", "visual")},
        "warnings": out["world"].warnings,
    }
    with open(os.path.join(map_dir, "gt_info.yaml"), "w") as f:
        yaml.safe_dump(info, f, sort_keys=False)
    return info


def figure(map_dir: str, out: dict, title: str) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.colors import ListedColormap

    g, occ, reach = out["grid"], out["occ"], out["reach"]
    blocked_only = out["blocking"] & ~occ
    lab = np.zeros(occ.shape, dtype=np.uint8)            # 0 unreachable free
    lab[reach] = 1
    lab[blocked_only] = 2                                 # low obstacle: robot height, below lidar
    lab[occ] = 3
    cmap = ListedColormap(["#e6e6e6", "#b9e3c6", "#f2c46d", "#222222"])
    ext = [g.x0, g.x0 + g.cols * g.res, g.y0, g.y0 + g.rows * g.res]
    fig, ax = plt.subplots(figsize=(8, 8 * g.rows / max(g.cols, 1) + 0.6))
    ax.imshow(lab, origin="lower", extent=ext, cmap=cmap, vmin=0, vmax=3, interpolation="nearest")
    sx = g.x0 + (out["seed_rc"][1] + 0.5) * g.res
    sy = g.y0 + (out["seed_rc"][0] + 0.5) * g.res
    ax.plot(sx, sy, "r+", ms=12, mew=2)
    from matplotlib.patches import Patch
    ax.legend(handles=[Patch(color="#222222", label="occupied at lidar height"),
                       Patch(color="#b9e3c6", label="reachable free"),
                       Patch(color="#f2c46d", label="blocks the robot, below/above lidar"),
                       Patch(color="#e6e6e6", label="free, not reachable"),
                       plt.Line2D([], [], color="r", marker="+", ls="", label="flood-fill seed")],
              loc="upper center", bbox_to_anchor=(0.5, -0.06), ncol=3, fontsize=8, frameon=False)
    ax.set_xlabel("x (m, world)")
    ax.set_ylabel("y (m, world)")
    ax.set_title(title)
    ax.set_aspect("equal")
    fig.tight_layout()
    fig.savefig(os.path.join(map_dir, "gt.png"), dpi=130)
    plt.close(fig)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("map_dir", help="maps/<map> (holds gt_config.yaml and the world file)")
    ap.add_argument("--no-figure", action="store_true")
    a = ap.parse_args(argv)
    cfg = load_config(a.map_dir)
    out = build(a.map_dir, cfg)
    info = write(a.map_dir, cfg, out)
    if not a.no_figure:
        figure(a.map_dir, out, f"{os.path.basename(os.path.normpath(a.map_dir))}: ground truth "
                               f"at {cfg['lidar_height']} m, {cfg['resolution']} m cells")
    c = info["cells"]
    print(f"{a.map_dir}: {info['grid']['cols']}x{info['grid']['rows']} cells, free={c['free']} "
          f"occupied={c['occupied']} reachable={c['reachable_free']} reference={c['reference']} "
          f"({info['areas_m2']['reachable_free']} m2 reachable)")
    for w in info["warnings"]:
        print(f"warning: {w}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())

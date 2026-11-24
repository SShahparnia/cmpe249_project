"""Visual floor for generated worlds: a grid of thin box tiles (Module 10).

The TB4's cliff sensors are downward gpu_lidar rays 1 to 15 cm long. With software
rendering (WSL, no GPU) they miss a floor drawn as one huge surface (a <plane>, or a single
100 m box: two giant triangles) and report a cliff everywhere, so the Create 3's cliff
reflex blocks every Nav2 command. A floor of 2 m tiles is seen correctly (tested in WSL,
2026-11-23). Collision stays an infinite <plane>.
"""
from __future__ import annotations

import math

TILE = 2.0


def floor_tiles_xml(x0, x1, y0, y1, color="0.8 0.8 0.8 1", indent="        ", tile=TILE) -> str:
    """<visual> elements tiling [x0, x1] x [y0, y1] (expanded outward to whole tiles)."""
    xs = range(int(math.floor(x0 / tile)), int(math.ceil(x1 / tile)))
    ys = range(int(math.floor(y0 / tile)), int(math.ceil(y1 / tile)))
    out = []
    for i in xs:
        for j in ys:
            cx, cy = (i + 0.5) * tile, (j + 0.5) * tile
            out.append(f'{indent}<visual name="floor_{i}_{j}"><pose>{cx:g} {cy:g} -0.01 0 0 0</pose>'
                       f'<geometry><box><size>{tile:g} {tile:g} 0.02</size></box></geometry>'
                       f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>')
    return "\n".join(out) + "\n"

#!/usr/bin/env python3
"""
Export the candidate layout (design_synthesis.py, 'rods' case, inversion-symmetric) as a scene for a 3D viewer.

Writes JSON: plates, and per member a cell box, wedge block, boss, rod (as a line with the turned
profile), and the two bolts. Coordinates are the synthesis frame (mm, z up, origin on the bottom
cap's inner face, centre of the sensor), converted to a viewer frame (x, z, -y) so that z-up
becomes y-up with a rotation (no mirroring).

    python3 hardware/ft_sensor/export_scene.py out.json
"""
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import design_synthesis as ds  # noqa: E402
import sizing as sz  # noqa: E402

# inversion-symmetric layout (design_synthesis.py --symmetry inv), hexagon, 6 mm caps, 75 kg cell
CAND = dict(r=28.7, delta=0.0, c0=52.8, alpha=54.7, psi=-124.2, lenmode=0, sc=1.0, zc=20.0)
CAP_T = 6.0


def v3(a):
    a = np.asarray(a, float)
    return [round(float(a[0]), 3), round(float(a[2]), 3), round(float(-a[1]), 3)]


def box(b):
    # Inversion negates a box's axes, which leaves a reflection (det -1) that a 3D engine cannot
    # turn into a rotation. A box is symmetric, so flipping one axis draws the same box.
    R = b.R.copy()
    if np.linalg.det(R) < 0:
        R[:, 2] = -R[:, 2]
    return dict(c=v3(b.c), ax=[v3(R[:, i]) for i in range(3)], h=[round(float(x), 3) for x in b.h])


def main(out):
    ds.configure(CAP_T, 18.0, 75.0)
    ds.set_envelope(88.85, None)
    ds.REPS = 1
    ms = ds.members_inv(CAND, "rods")
    A = np.array([m["line"] for m in ms]).T
    forces = np.abs(np.linalg.pinv(A)) @ (ds.W_DESIGN_MM / sz.RANGE_F)
    scene = {
        "gap": ds.GAP, "cap": CAP_T, "across_flats": 88.85, "keepout": ds.KEEPOUT,
        "params": CAND, "members": [],
    }
    for m in ms:
        u = m["u"]
        cell = next(b for b in m["boxes"] if b.kind == "cell")
        pad = next(b for b in m["boxes"] if b.name.startswith("pad1"))
        rod = next(b for b in m["boxes"] if b.kind == "rod")
        boss = next(b for b in m["boxes"] if b.name.startswith("boss"))
        span = 2 * rod.h[2]
        start = rod.c - rod.R[:, 2] * span / 2
        h1, h2 = m["h1"], m["h2"]
        scene["members"].append(dict(
            id=m["id"], flipped=bool(m.get("flipped")), u=v3(u), tilt=round(math.degrees(math.acos(abs(u[2]))), 1),
            cell=box(cell), block=box(pad), boss=box(boss),
            rod=dict(start=v3(start), dir=v3(rod.R[:, 2]), span=round(float(span), 2)),
            fixed_hole=v3(h1), free_hole=v3(h2),
            force_per_wrench=[round(float(x), 4) for x in np.abs(np.linalg.pinv(A))[m["id"]]],
        ))
    scene["thickness"] = ds.CT
    scene["cell_dims"] = [ds.CL, ds.CW, ds.CT]
    Path(out).write_text(json.dumps(scene, indent=1))
    print(f"wrote {out}: {len(scene['members'])} members, rod span {scene['members'][0]['rod']['span']} mm")


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "scene.json")

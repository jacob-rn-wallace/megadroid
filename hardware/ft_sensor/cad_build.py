#!/usr/bin/env python3
"""
Build the foot F/T sensor as solid CAD (STEP) from the synthesised layout, and check it.

Needs a CAD kernel:  pip install build123d   (OpenCASCADE; not in the project venv, nothing else uses it)

    python3 hardware/ft_sensor/cad_build.py [--out DIR]

WRITES  (default hardware/ft_sensor/cad/megadroid_ft_v0/)
    sensor_assembly.step            everything, coloured, in the synthesis frame (mm, z up, origin at the
                                    centre of the bottom cap's inner face)
    cap_bottom.step, cap_top.step   machined caps (integral wedges and bosses)
    rod.step                        the turned flexure rod (one drawing, six off)
    load_cell_modified.step         the harvested cell with its two holes reamed to clearance
    fasteners.step                  cell bolt and washer, shim, free-end screw, one of each
    features.csv                    per member: tilt, azimuth and hole entry points for the wedge and boss holes
    checks.txt                      interference, clearance, envelope, hole, tool-access and assembly-path checks

WHAT IS DESIGN HERE AND WHAT IS A CHOICE
The layout (cell positions, tilt, rod length, boss size) comes from design_synthesis.py. Everything below
the "DETAILING CHOICES" line was decided while detailing and has NOT been sized by FEA or tested.

FRAMES  Each member is built in its own frame: z = the cell's bolt axis u, x = along the cell, origin on the
cell's mid-plane at the free hole.  A flipped member (cell on the top cap) is the inversion image of an
ordinary one, so it is built from its own boxes in the same way: nothing is mirrored by hand.
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
from build123d import (Box, Cylinder, Plane, Pos, RegularPolygon, extrude, export_step, Compound, Color, Axis,
                       Polyline, make_face, revolve, fillet)

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import design_synthesis as ds  # noqa: E402

# ---- the layout and the main part dimensions: design/ft_sensor.yaml, build_candidate ------------------------
BC = ds.FT["build_candidate"]
CAND = BC["layout"]
CAP_T = BC["cap_thickness_mm"]
ROD_MIN = BC["min_free_rod_mm"]
RATED_KG = BC["rated_kg"]
HEX_AF = BC["rod"]["hex_af_mm"]  # wrench flats sit BETWEEN neck and stud so tightening torque never goes through a neck
SHOULDER = BC["rod"]["shoulder_mm"]
NECK_D, NECK_L = BC["rod"]["neck_d_mm"], BC["rod"]["neck_l_mm"]
NECK_FILLET = BC["rod"]["neck_fillet_mm"]
BODY_D = BC["rod"]["body_d_mm"]
STUD_D = 6.0                    # M6x1
STUD_B = BC["rod"]["stud"]["into_boss_mm"]
SHIM_NOM = BC["shim_nominal_mm"]
CELL_HOLE_FIXED, CELL_HOLE_FREE = BC["cell_hole_ream_mm"]["fixed"], BC["cell_hole_ream_mm"]["free"]

# ---- DETAILING CHOICES made here (catalogue sizes, tool probes; not sized by FEA, not tested) ---------------
A_END = BC["rod"]["a_end_mm"]                  # rod's cell end: a hex block with a blind M6 thread, wrench flats along all of it
A_THREAD_DEPTH = BC["rod"]["a_thread_depth_mm"]
WASHER = dict(od=12.0, id=6.4, t=1.6)
BOLT_HEAD = dict(d=10.0, k=6.0)       # M6 socket head cap screw, fixed-end bolt
BOLT_LEN = 18.0                       # M6x18, 12.9: 7.9 mm into the wedge (thread strip capacity is ~20 kN)
FREE_HEAD = dict(d=10.0, k=3.9)       # M6 low-head socket screw (DIN 7984), free end, from the cell's underside
FREE_LEN = 14.0                       # M6x14: goes through the cell (8.5) and shim, 5.25 mm into the rod end
HOLE_D = 6.0                          # tapped M6: drawn at the 6.0 major diameter so the screw models without overlap
TAP_DRILL = 5.0                       # what you actually drill before tapping
FIXED_HOLE_DEPTH = 9.5
BOSS_HOLE_DEPTH = 10.0
CELL_EDGE_R = 0.5
KEY_D, KEY_LEN = 7.0, ds.KEY_ACCESS   # 5 mm hex key / short bit: a 7 mm wide probe, as the layout search used
SPANNER_THIN_D = 12.0                 # ground-down thin spanner jaw (hex is 9.24 across corners)
HANDLE_W, HANDLE_L = 9.0, 30.0
SPANNER_D = 17.0                      # standard 8 mm open-end spanner jaw
OVERHANG = 20.0                       # prisms are cut back to the cap plane afterwards
CLOSE_START = 20.0                    # the two cap halves start this far apart (mm along z) in the closing check

DENSITY = {"7075": 2.81e-3, "steel": 7.85e-3, "cell": 2.70e-3}   # g/mm^3


def rod_free_length(m):
    """Cell face to boss face along the rod axis: shim plus the rod's boxes in the layout model."""
    return SHIM_NOM + sum(2 * b.h[2] for b in m["boxes"] if b.name.startswith("rod"))


def frame(origin, x, z):
    return Plane(origin=tuple(float(v) for v in origin), x_dir=tuple(float(v) for v in x),
                 z_dir=tuple(float(v) for v in z)).location


def hexagon_prism(apothem, z0, z1):
    R = apothem / math.cos(math.radians(30))
    s = extrude(RegularPolygon(radius=R, side_count=6), amount=z1 - z0)
    return Pos(0, 0, z0) * s


def prism_from_face(face_centre, ax_x, ax_z, half_x, half_y, length):
    """Rectangular prism: one end on the plane through face_centre with normal ax_z (x along ax_x), extending
    LENGTH in the direction of -ax_z. Returns the solid."""
    b = Pos(0, 0, -length / 2) * Box(2 * half_x, 2 * half_y, length)
    return b.moved(frame(face_centre, ax_x, ax_z))


def cyl_along(origin, axis, x_hint, radius, z0, z1):
    """Cylinder with its axis through origin along `axis`, spanning z0..z1 measured along the axis."""
    c = Pos(0, 0, (z0 + z1) / 2) * Cylinder(radius, z1 - z0)
    return c.moved(frame(origin, x_hint, axis))


def perp(v):
    v = np.asarray(v, float)
    t = np.array([1.0, 0, 0]) if abs(v[0]) < 0.9 else np.array([0, 1.0, 0])
    p = np.cross(v, t)
    return p / np.linalg.norm(p)


# ---- parts ------------------------------------------------------------------------------------------------
def make_rod(l_free):
    """Turned rod, axis z, z = 0 at the cell-end face. Cell end: hex block (A_END long) with a blind M6 thread, the
    cell is screwed to it from below. Boss end: hex (SHOULDER) and an M6 stud that screws into the boss.
    Wrench flats sit on the hex blocks, outboard of the necks, so no tightening torque ever passes a neck."""
    L = l_free - SHIM_NOM
    hex_r = HEX_AF / 2 / math.cos(math.radians(30))
    body_len = L - A_END - 2 * NECK_L - SHOULDER
    assert body_len > 1.0, f"free rod {l_free:.2f} mm too short for this end block and necks (body {body_len:.2f} mm)"
    zb0 = A_END + NECK_L
    zb1 = zb0 + body_len
    zn1 = L - SHOULDER
    pts = [(0, A_END - 0.5), (NECK_D / 2, A_END - 0.5), (NECK_D / 2, zb0), (BODY_D / 2, zb0), (BODY_D / 2, zb1),
           (NECK_D / 2, zb1), (NECK_D / 2, zn1), (STUD_D / 2, zn1), (STUD_D / 2, L + STUD_B), (0, L + STUD_B)]
    core = revolve(make_face(Polyline(*[(q[0], 0.0, q[1]) for q in pts], close=True)), Axis.Z)
    hexA = extrude(RegularPolygon(radius=hex_r, side_count=6), amount=A_END)
    hexB = Pos(0, 0, zn1) * extrude(RegularPolygon(radius=hex_r, side_count=6), amount=SHOULDER)
    rod = core + hexA + hexB
    rod = rod - Pos(0, 0, A_THREAD_DEPTH / 2) * Cylinder(HOLE_D / 2, A_THREAD_DEPTH)       # blind M6 hole, cell end
    try:                                                             # stress-relief fillets where the necks end
        sel = [e for e in rod.edges() if abs(e.length - math.pi * NECK_D) < 1e-3]
        rod = fillet(sel, NECK_FILLET)
    except Exception as ex:
        print("   (neck fillet skipped:", ex, ")")
    return rod, dict(L=L, body_len=body_len, hex_r=hex_r)


def make_cell():
    """Simplified harvested cell: box with two reamed holes. Local: x = length, y = width, z = bolt axis.
    The fixed-end hole (-HOLE) is a close fit for the bolt; the free-end hole (+HOLE) is oversize so the cell can be
    placed over the cap-to-cap alignment tolerance."""
    cell = Box(ds.CL, ds.CW, ds.CT)
    try:
        cell = fillet(cell.edges().filter_by(Axis.Z), CELL_EDGE_R)
    except Exception:
        pass
    for hx, d in ((-ds.HOLE, CELL_HOLE_FIXED), (ds.HOLE, CELL_HOLE_FREE)):
        cell = cell - Pos(hx, 0, 0) * Cylinder(d / 2, ds.CT + 2)
    return cell


def make_screw(head, length):
    """Socket head screw at its own origin: head underside at z = 0, head towards +z, shank towards -z."""
    h = Pos(0, 0, head["k"] / 2) * Cylinder(head["d"] / 2, head["k"])
    shank = Pos(0, 0, -length / 2) * Cylinder(STUD_D / 2, length)
    return h + shank - Pos(0, 0, head["k"] - 1.5) * Cylinder(2.0, 3.0)


def make_washer():
    return Pos(0, 0, WASHER["t"] / 2) * Cylinder(WASHER["od"] / 2, WASHER["t"]) - \
        Pos(0, 0, WASHER["t"] / 2) * Cylinder(WASHER["id"] / 2, WASHER["t"] + 1)


def make_shim():
    t = SHIM_NOM
    return Pos(0, 0, t / 2) * Cylinder(WASHER["od"] / 2, t) - Pos(0, 0, t / 2) * Cylinder(WASHER["id"] / 2, t + 1)


# ---- assembly ---------------------------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(HERE / "cad" / "megadroid_ft_v0"))
    ap.add_argument("--mesh", help="also write a triangle mesh of every part (JSON, viewer frame y-up) for the 3D viewer")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    log = []

    def say(s=""):
        print(s)
        log.append(s)

    ds.configure(CAP_T, ROD_MIN, RATED_KG)
    ds.set_envelope(88.85, None)
    ds.REPS = 1
    ms = ds.members_inv(CAND, "rods")
    assert ms is not None and ds.feasible(ms), "layout no longer feasible: re-run design_synthesis.py"
    GAP = ds.GAP
    apothem = ds.APOTHEM
    say(f"layout: {len(ms)} members, gap {GAP:.3f} mm, caps {CAP_T} mm, sensor height {GAP + 2 * CAP_T:.3f} mm")

    hex_full = hexagon_prism(apothem, -CAP_T, GAP + CAP_T)
    hex_inner = hexagon_prism(apothem - ds.MARGIN, -CAP_T, GAP + CAP_T)
    cap_bot = hexagon_prism(apothem, -CAP_T, 0.0)
    cap_top = hexagon_prism(apothem, GAP, GAP + CAP_T)

    cells, rods, hardware = [], [], []
    wedges = {"bot": [], "top": []}
    bosses = {"bot": [], "top": []}
    holes = {"bot": [], "top": []}
    info = []
    rod_solid = None
    members_data = []

    for m in ms:
        flipped = bool(m.get("flipped"))
        fixed_cap, far_cap = ("top", "bot") if flipped else ("bot", "top")
        u = np.asarray(m["u"], float)
        cell_b = next(b for b in m["boxes"] if b.kind == "cell")
        pad_b = next(b for b in m["boxes"] if b.name.startswith("pad1"))
        boss_b = next(b for b in m["boxes"] if b.name.startswith("boss"))
        h1, h2 = np.asarray(m["h1"], float), np.asarray(m["h2"], float)

        # --- wedge: pad box cross-section, from the cell's fixed-end face out along -u to the cap plane
        face_c = pad_b.c + pad_b.R[:, 2] * pad_b.h[2]
        w = prism_from_face(face_c, pad_b.R[:, 0], pad_b.R[:, 2], pad_b.h[0], pad_b.h[1], OVERHANG)
        # --- boss: from the boss face (towards the cell) out along +u past the far cap plane
        bface = boss_b.c - boss_b.R[:, 2] * boss_b.h[2]
        bs = prism_from_face(bface, boss_b.R[:, 0], -boss_b.R[:, 2], boss_b.h[0], boss_b.h[1], OVERHANG)
        # trim both to the sensor envelope and to their cap
        lo_w, hi_w = (-CAP_T + 0.5, GAP + CAP_T) if not flipped else (-CAP_T, GAP + CAP_T - 0.5)
        zclip = lambda lo, hi: hexagon_prism(apothem, lo, hi)
        wedges[fixed_cap].append((w & zclip(lo_w, hi_w), m))
        lo_b, hi_b = (-CAP_T, GAP + CAP_T - 0.5) if not flipped else (-CAP_T + 0.5, GAP + CAP_T)
        bosses[far_cap].append((bs & zclip(lo_b, hi_b), m))

        # --- holes (tap drill) in wedge and boss
        l_free = rod_free_length(m)
        axis_far = h2 + u * (ds.CT / 2 + l_free)
        holes[fixed_cap].append(("fixed", (h1 - u * ds.CT / 2, -u), m["id"]))
        holes[far_cap].append(("boss", (axis_far, u), m["id"]))

        # --- the cell (centred on its box, x along the cell)
        cell = make_cell()
        cell_loc = frame(cell_b.c, cell_b.R[:, 0], cell_b.R[:, 2])
        # make_cell holes at -HOLE (fixed) and +HOLE (free) along local x; check orientation against h1/h2
        cells.append((cell.moved(cell_loc), m))

        members_data.append(dict(m=m, cell_b=cell_b, h1=h1, h2=h2, u=u, l_free=l_free, flipped=flipped,
                                 fixed_entry=h1 - u * ds.CT / 2, boss_entry=h2 + u * (ds.CT / 2 + l_free)))
        info.append(dict(id=m['id'], flipped=flipped, l_free=l_free))

    def outer_slab(key):
        return Pos(0, 0, -CAP_T - 50) * Box(300, 300, 100) if key == "bot" else Pos(0, 0, GAP + CAP_T + 50) * Box(300, 300, 100)

    def hole_solid(spec, depth):
        (o, ax) = spec
        return cyl_along(o, ax, perp(ax), HOLE_D / 2, 0.0, depth)

    def deepest(cap_solid, key, spec, max_depth):
        """Deepest tapped hole (to 0.5 mm steps) that stays inside the solid with >= 1 mm wall to the cap's outer face."""
        d = max_depth
        while d > 2.0:
            h = hole_solid(spec, d)
            if h.distance_to(outer_slab(key)) >= 1.0 and (h & cap_solid).volume >= 0.995 * h.volume:
                return d
            d -= 0.5
        return 0.0

    def build_cap(base, key):
        cap = base
        for s_, _ in wedges[key]:
            cap = cap + s_
        for s_, _ in bosses[key]:
            cap = cap + s_
        return cap & hex_full

    bot0, top0 = build_cap(cap_bot, "bot"), build_cap(cap_top, "top")
    pre = {"bot": bot0, "top": top0}
    depth = {"fixed": 99.0, "boss": 99.0}
    for key in ("bot", "top"):
        for kind, spec, mid in holes[key]:
            depth[kind] = min(depth[kind], deepest(pre[key], key, spec, 14.0))
    FIXED_DEPTH, BOSS_DEPTH = depth["fixed"], depth["boss"]
    stud_b = BOSS_DEPTH - 0.5                                    # boss-end stud, stops 0.5 mm short of the hole bottom
    eng_boss = stud_b
    under = ds.CT + WASHER["t"]                                  # washer + cell under the fixed-end bolt head
    bolt_len = max(L for L in (6, 8, 10, 12, 14, 16, 18, 20, 25) if L - ds.CT - WASHER["t"] <= FIXED_DEPTH - 0.5)
    eng_fixed = bolt_len - ds.CT - WASHER["t"]
    say("")
    say(f"TAPPED HOLE DEPTHS (deepest with a 1 mm wall to the cap's outer face, same for all six): "
        f"fixed-end {FIXED_DEPTH:.1f} mm, boss {BOSS_DEPTH:.1f} mm")
    say(f"  boss-end stud {stud_b:.1f} mm long -> thread engagement {eng_boss:.1f} mm ({eng_boss / HOLE_D:.2f} x d); "
        f"cell bolt M6x{bolt_len} -> engagement {eng_fixed:.1f} mm ({eng_fixed / HOLE_D:.2f} x d)")
    if min(eng_boss, eng_fixed) < HOLE_D:
        say("  NOTE: engagement under 1 x d in 7075; fine for the load (strip capacity is many kN) but check by pull test")
    rep_b, rep_t, = [], []
    caps = {}
    for key, rep in (("bot", rep_b), ("top", rep_t)):
        cap = pre[key]
        for kind, spec, mid in holes[key]:
            d = FIXED_DEPTH if kind == "fixed" else BOSS_DEPTH
            h = hole_solid(spec, d)
            before = cap.volume
            cap = cap - h
            rep.append((kind, mid, before - cap.volume, h.volume, h.distance_to(outer_slab(key))))
        caps[key] = cap
    bot, top = caps["bot"], caps["top"]
    STUD_B_USED, BOLT_LEN_USED = stud_b, float(bolt_len)
    say("")
    say("HOLE CHECK (volume removed from the cap / volume of the drill; less means the drill left the solid; wall >= 1 mm)")
    for name, rep in (("bottom cap", rep_b), ("top cap", rep_t)):
        for kind, mid, removed, want, wall in rep:
            ok = removed >= want * 0.995 and wall >= 1.0
            say(f"  {name} {kind} hole, member {mid}: removed {removed:.1f} of {want:.1f} mm3, wall to the outer face {wall:.2f} mm  "
                f"{'ok' if ok else 'BREAKS OUT / WALL UNDER 1 mm'}")
    say("")
    # --- the rods and hardware, now that the stud length and bolt length are known
    global STUD_B, BOLT_LEN
    STUD_B, BOLT_LEN = STUD_B_USED, BOLT_LEN_USED
    for md in members_data:
        m, h1, h2, u, l_free = md["m"], md["h1"], md["h2"], md["u"], md["l_free"]
        # --- the rod, A-end hex bearing on the cell's +u face at h2 (through the shim), B hex on the boss face
        if rod_solid is None:
            rod_solid, rinfo = make_rod(l_free)
            say(f"rod: free length cell face to boss face {l_free:.3f} mm; rod {rinfo['L']:.3f} mm shoulder to shoulder, "
                f"body {rinfo['body_len']:.2f} mm")
        a_origin = h2 + u * (ds.CT / 2 + SHIM_NOM)
        rods.append((rod_solid.moved(frame(a_origin, perp(u), u)), m))

        # --- hardware
        bolt = make_screw(BOLT_HEAD, BOLT_LEN).moved(frame(h1 + u * (ds.CT / 2 + WASHER["t"]), perp(u), u))
        wash_f = make_washer().moved(frame(h1 + u * ds.CT / 2, perp(u), u))
        shim = make_shim().moved(frame(h2 + u * ds.CT / 2, perp(u), u))
        screw_free = make_screw(FREE_HEAD, FREE_LEN).moved(frame(h2 - u * ds.CT / 2, perp(u), -u))
        hardware.append(dict(m=m, bolt=bolt, wash_f=wash_f, shim=shim, screw_free=screw_free))


    say(f"cap volumes: bottom {bot.volume / 1000:.2f} cm3, top {top.volume / 1000:.2f} cm3")

    # ---- named parts
    parts = []   # (name, solid, material, member, colour)
    parts.append(("cap_bottom", bot, "7075", None, (0.62, 0.66, 0.72)))
    parts.append(("cap_top", top, "7075", None, (0.55, 0.60, 0.67)))
    for (c, m) in cells:
        parts.append((f"cell_{m['id']}", c, "cell", m["id"], (0.80, 0.72, 0.50)))
    for (r, m) in rods:
        parts.append((f"rod_{m['id']}", r, "7075", m["id"], (0.78, 0.45, 0.30)))
    for h in hardware:
        i = h["m"]["id"]
        parts += [(f"bolt_{i}", h["bolt"], "steel", i, (0.25, 0.25, 0.28)),
                  (f"washer_fixed_{i}", h["wash_f"], "steel", i, (0.45, 0.45, 0.48)),
                  (f"shim_{i}", h["shim"], "steel", i, (0.85, 0.85, 0.2)),
                  (f"screw_free_{i}", h["screw_free"], "steel", i, (0.30, 0.30, 0.34))]

    # ---- checks
    say("")
    say("ENVELOPE (part volume outside the hexagon inset by the wall margin; caps are the envelope itself)")
    worst = 0.0
    for name, s, mat, mid, col in parts:
        if name.startswith("cap"):
            continue
        outside = s.volume - (s & hex_inner).volume
        worst = max(worst, outside)
        if outside > 0.01:
            say(f"  {name}: {outside:.3f} mm3 outside")
    say(f"  worst non-cap overshoot {worst:.3f} mm3" + ("  (all inside)" if worst <= 0.01 else ""))

    say("")
    say("INTERFERENCE (overlap volume, mm3; contacts that share a face show ~0)")
    n_int = 0
    names = [p[0] for p in parts]
    bbs = [p[1].bounding_box() for p in parts]

    def near(a, b, pad=0.2):
        return not (a.max.X + pad < b.min.X or b.max.X + pad < a.min.X or a.max.Y + pad < b.min.Y or
                    b.max.Y + pad < a.min.Y or a.max.Z + pad < b.min.Z or b.max.Z + pad < a.min.Z)

    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            if not near(bbs[i], bbs[j]):
                continue
            try:
                v = (parts[i][1] & parts[j][1]).volume
            except Exception:
                v = float("nan")
            if v > 0.05 or v != v:
                n_int += 1
                say(f"  {names[i]} x {names[j]}: {v:.3f} mm3")
    say(f"  {n_int} overlapping pairs" + ("" if n_int else " -- none"))

    say("")
    say("CLEARANCE (mm). Contacts the design intends (cell on its wedge, rod on its boss, hardware in its holes) are")
    say("not listed; everything else is: member against other members, and each member's loose parts against the")
    say("wedges and bosses that are not its own and against the plain cap faces.")
    loose = ("cell", "washer", "shim", "screw")
    rows = []
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            ni, nj = parts[i], parts[j]
            if ni[3] is None or nj[3] is None or ni[3] == nj[3]:
                continue
            if not near(bbs[i], bbs[j], 4.0):
                continue
            rows.append((ni[1].distance_to(nj[1]), f"{ni[0]} - {nj[0]}"))
    for key, wl in wedges.items():
        for sol, mw in wl:
            for p in parts:
                if p[3] is not None and p[3] != mw["id"] and near(p[1].bounding_box(), sol.bounding_box(), 4.0):
                    rows.append((p[1].distance_to(sol), f"{p[0]} - wedge of member {mw['id']}"))
    for key, bl in bosses.items():
        for sol, mw in bl:
            for p in parts:
                if p[3] is not None and p[3] != mw["id"] and near(p[1].bounding_box(), sol.bounding_box(), 4.0):
                    rows.append((p[1].distance_to(sol), f"{p[0]} - boss of member {mw['id']}"))
    slabs = {"bottom face": Pos(0, 0, -CAP_T / 2) * Box(200, 200, CAP_T), "top face": Pos(0, 0, GAP + CAP_T / 2) * Box(200, 200, CAP_T)}
    for p in parts:
        if p[3] is not None and p[0].split("_")[0] in loose:
            for sn, sl in slabs.items():
                rows.append((p[1].distance_to(sl), f"{p[0]} - cap {sn}"))
    rows.sort()
    for d, nm in rows[:14]:
        say(f"  {d:6.3f}  {nm}")
    say(f"  ({len(rows)} pairs measured; smallest {rows[0][0]:.3f} mm)")

    say("")
    say("TOOL ACCESS (a probe of the tool's footprint must not touch anything but its own fastener)")
    probes = []
    spanner_jobs = []
    for m in ms:
        i = m["id"]
        u = np.asarray(m["u"], float)
        h1, h2 = np.asarray(m["h1"], float), np.asarray(m["h2"], float)
        top_of_head = h1 + u * (ds.CT / 2 + WASHER["t"] + BOLT_HEAD["k"])
        probes.append((f"hex key to bolt {i} (arm dia {KEY_D} mm, {KEY_LEN:g} mm out)", f"bolt_{i}",
                       cyl_along(top_of_head, u, perp(u), KEY_D / 2, 0.0, KEY_LEN)))
        screw_out = h2 - u * (ds.CT / 2 + FREE_HEAD["k"])
        probes.append((f"hex key to free-end screw {i} (arm dia {KEY_D} mm, {KEY_LEN:g} mm out)", f"screw_free_{i}",
                       cyl_along(screw_out, -u, perp(u), KEY_D / 2, 0.0, KEY_LEN)))
        spanner_jobs.append(m)
    bad = 0
    cap_sols = {"cap_bottom": bot, "cap_top": top}
    allp = [(n, sol) for n, sol, *_ in parts]
    for label, own, probe in probes:
        hits = []
        for n, sol in allp:
            if n in (own if isinstance(own, tuple) else (own,)):
                continue
            if not near(probe.bounding_box(), sol.bounding_box(), 0.0):
                continue
            v = (probe & sol).volume
            if v > 0.5:
                hits.append((n, v))
        if hits:
            bad += 1
            say(f"  BLOCKED  {label}: " + ", ".join(f"{n} {v:.1f} mm3" for n, v in hits))
    say(f"  {len(probes) - bad} of {len(probes)} hex-key probes clear" + ("" if not bad else f"; {bad} blocked"))

    why = {}
    # Open-end spanner on the two rod hexes: a jaw disc plus a handle strip, tried in 12 directions around the axis.
    for jaw in (SPANNER_D, SPANNER_THIN_D):
        res = {}
        for m in spanner_jobs:
            i = m["id"]
            u = np.asarray(m["u"], float)
            h2 = np.asarray(m["h2"], float)
            rl = rod_free_length(m) - SHIM_NOM
            a0 = h2 + u * (ds.CT / 2 + SHIM_NOM)
            for end, z0, z1 in (("cell-end hex", 0.2, A_END - 0.2), ("boss-end hex", rl - SHOULDER + 0.2, rl - 0.2)):
                ok_dirs = []
                for k in range(12):
                    ang = math.radians(30 * k)
                    xh = perp(u)
                    yh = np.cross(u, xh)
                    d = math.cos(ang) * xh + math.sin(ang) * yh
                    disc = cyl_along(a0, u, xh, jaw / 2, z0, z1)
                    strip = (Pos(HANDLE_L / 2, 0, (z0 + z1) / 2) * Box(HANDLE_L, HANDLE_W, z1 - z0)).moved(frame(a0, d, u))
                    probe = disc + strip
                    pb = probe.bounding_box()
                    blockers = []
                    for n, sol in allp:
                        if n in (f"rod_{i}", f"screw_free_{i}"):
                            continue
                        if near(pb, sol.bounding_box(), 0.0):
                            v = (probe & sol).volume
                            if v > 0.5:
                                blockers.append(f"{n} {v:.0f}")
                    if not blockers:
                        ok_dirs.append(30 * k)
                    else:
                        why.setdefault((i, end, jaw), {})[30 * k] = blockers
                res[(i, end)] = ok_dirs
        worst = min(len(v) for v in res.values())
        none = [k for k, v in res.items() if not v]
        say(f"  spanner, {jaw:g} mm jaw + {HANDLE_W:g} mm x {HANDLE_L:g} mm handle: free approach directions per hex (of 12): "
            f"min {worst}, max {max(len(v) for v in res.values())}" + (f"; NO approach for {none}" if none else ""))
        for (i, end), v in sorted(res.items()):
            if i < 2:
                say(f"     rod {i} {end}: clear at {v} deg")
                if jaw == SPANNER_THIN_D and end == "cell-end hex" and not v:
                    for a_, bl in sorted(why[(i, end, jaw)].items()):
                        say(f"        {a_:3d} deg blocked by " + ", ".join(bl))

    say("")
    say("OWN-MEMBER CLEARANCE (cell and free-end screw head to the cap that does not carry them)")
    for m in ms:
        fl = bool(m.get("flipped"))
        i = m["id"]
        cell = next(p[1] for p in parts if p[0] == f"cell_{i}")
        scr = next(p[1] for p in parts if p[0] == f"screw_free_{i}")
        carrier, other = (top, bot) if fl else (bot, top)
        say(f"  member {i} ({'cell on top' if fl else 'cell on bottom'}): cell to far cap {cell.distance_to(other):.2f} mm, "
            f"free-end screw head to its own cap {scr.distance_to(carrier):.2f} mm")

    # ---- assembly paths: is there a way to put it together? ------------------------------------------
    say("")
    say("ASSEMBLY PATHS (overlap volume in mm3 along the motion; 0 = free). Rigid six-rod loop: the two halves must close")
    say("in one straight move, then the six free-end screws go in from underneath.")
    say("  1. each cap on the bench: screw its three rods into its bosses, put the shims on, bolt its three cells to its wedges")
    say("  2. close the halves along z")
    say("  3. screw each cell to its rod end from the underside")
    byname = {pn: ps for pn, ps, *_ in parts}
    flip_ids = {m["id"] for m in ms if m.get("flipped")}
    unf_ids = {m["id"] for m in ms if not m.get("flipped")}

    def ov(solid, grp, cut=0.5):
        mb = solid.bounding_box()
        w, who = 0.0, ""
        for pn, ps in grp:
            if near(mb, ps.bounding_box(), 0.0):
                v = (solid & ps).volume
                if v > w:
                    w, who = v, pn
        return w, who

    # step 1: each cap on the bench. Two orders are tried: rods first then cells, or cells first then rods.
    def cap_sets(key):
        cap_solid = bot if key == "bottom" else top
        cell_ids = sorted(unf_ids if key == "bottom" else flip_ids)
        rod_ids = sorted(flip_ids if key == "bottom" else unf_ids)       # rods rooted in this cap's bosses
        return cap_solid, cell_ids, rod_ids

    def rod_into_boss(i, others):
        u = next(np.asarray(m["u"], float) for m in ms if m["id"] == i)
        rod0 = byname[f"rod_{i}"]
        worst = (0.0, "", 0.0)
        for adv in np.linspace(STUD_B, 0.0, 5)[:-1]:
            w, who = ov(Pos(*(-u * adv)) * rod0, others)
            if w > worst[0]:
                worst = (w, who, adv)
        return worst

    def cell_onto_wedge(i, grp):
        u = next(np.asarray(m["u"], float) for m in ms if m["id"] == i)
        cell0 = byname[f"cell_{i}"]
        worst = (0.0, "", 0.0)
        for back in np.arange(15.0, 0.4, -1.0):
            w, who = ov(Pos(*(u * back)) * cell0, grp)
            if w > worst[0]:
                worst = (w, who, back)
        return worst

    order_ok = {}
    for order in ("rods first", "cells first"):
        problems = []
        for key in ("bottom", "top"):
            cap_solid, cell_ids, rod_ids = cap_sets(key)
            placed_cells, placed_rods = [], []
            seq = [("rod", i) for i in rod_ids] + [("cell", i) for i in cell_ids]
            if order == "cells first":
                seq = [("cell", i) for i in cell_ids] + [("rod", i) for i in rod_ids]
            for kind_, i in seq:
                if kind_ == "rod":
                    others = [(f"cap_{key}", cap_solid)] + placed_rods + placed_cells
                    w = rod_into_boss(i, others)
                    placed_rods.append((f"rod_{i}", byname[f"rod_{i}"]))
                    placed_rods.append((f"shim_{i}", byname[f"shim_{i}"]))
                else:
                    grp = [(f"cap_{key}", cap_solid)] + placed_rods + placed_cells
                    w = cell_onto_wedge(i, grp)
                    placed_cells.append((f"cell_{i}", byname[f"cell_{i}"]))
                    placed_cells.append((f"bolt_{i}", byname[f"bolt_{i}"]))
                if w[0] > 0.5:
                    problems.append(f"{kind_} {i} ({key}): blocked by {w[1]} {w[0]:.1f} mm3 at {w[2]:.0f} mm out")
        order_ok[order] = problems
        say(f"  bench order '{order}': " + ("free" if not problems else "; ".join(problems)))
    bench_ok = any(not v for v in order_ok.values())
    say(f"  each cap can be built up on the bench: {'yes' if bench_ok else 'NO'}")

    # step 2: close the halves. The top half = top cap, its three cells, the three rods rooted in it.
    top_ids = flip_ids
    def half(is_top):
        names = [("cap_top" if is_top else "cap_bottom")]
        for i in sorted(flip_ids if is_top else unf_ids):
            names += [f"cell_{i}", f"bolt_{i}", f"washer_fixed_{i}"]
        for i in sorted(unf_ids if is_top else flip_ids):
            names += [f"rod_{i}", f"shim_{i}"]
        return [(n, byname[n]) for n in names]
    top_half, bot_half = half(True), half(False)
    worst = (0.0, "", 0.0)
    first_hit = None
    for dz in np.arange(CLOSE_START, -0.01, -1.0):
        moved = [(n, Pos(0, 0, dz) * sol) for n, sol in top_half]
        for n, sol in moved:
            w, who = ov(sol, bot_half)
            if w > 0.5 and first_hit is None:
                first_hit = (dz, n, who, w)
            if w > worst[0]:
                worst = (w, f"{n} x {who}", dz)
    if worst[0] > 0.5:
        say(f"  closing along z: BLOCKED. First collision {first_hit[1]} x {first_hit[2]} {first_hit[3]:.1f} mm3 at {first_hit[0]:.0f} mm "
            f"from closed; worst {worst[1]} {worst[0]:.1f} mm3 at {worst[2]:.0f} mm")
    else:
        say(f"  closing along z from {CLOSE_START:g} mm apart: free (every part of the top half clears the bottom half to the final seat)")

    # step 3: each free-end screw goes in along +u from underneath
    bad3 = 0
    for m in ms:
        i = m["id"]
        u = np.asarray(m["u"], float)
        scr = byname[f"screw_free_{i}"]
        grp = [(n, s_) for n, s_ in allp if n not in (f"screw_free_{i}", f"cell_{i}", f"rod_{i}", f"shim_{i}")]
        w_tot = (0.0, "")
        for back in np.arange(20.0, 0.4, -2.0):
            w, who = ov(Pos(*(-u * back)) * scr, grp)
            if w > w_tot[0]:
                w_tot = (w, who)
        if w_tot[0] > 0.5:
            bad3 += 1
            say(f"  free-end screw {i}: BLOCKED by {w_tot[1]} {w_tot[0]:.1f} mm3")
    say(f"  free-end screws: {'all free' if not bad3 else f'{bad3} blocked'}")

    # ---- mass
    say("")
    mass = {}
    for name, s, mat, mid, col in parts:
        mass[name] = s.volume * DENSITY[mat]
    tot = sum(mass.values())
    say(f"MASS estimate: caps {mass['cap_bottom']:.0f} + {mass['cap_top']:.0f} g, six cells {sum(v for k, v in mass.items() if k.startswith('cell')):.0f} g, "
        f"six rods {sum(v for k, v in mass.items() if k.startswith('rod')):.0f} g, hardware "
        f"{sum(v for k, v in mass.items() if k.split('_')[0] in ('bolt', 'washer', 'shim', 'screw')):.0f} g, total {tot:.0f} g")

    # ---- machining feature table
    rows = ["member,cell_on,tilt_from_cap_normal_deg,azimuth_deg,u_x,u_y,u_z,wedge_hole_entry_x,wedge_hole_entry_y,wedge_hole_entry_z,"
            "boss_hole_entry_x,boss_hole_entry_y,boss_hole_entry_z"]
    for md in members_data:
        u = md["u"]
        # u points from the cell to its rod; the machining axis of the wedge hole is -u, of the boss hole +u
        tilt = math.degrees(math.acos(abs(u[2])))
        az = math.degrees(math.atan2(u[1], u[0]))
        fe, be = md["fixed_entry"], md["boss_entry"]
        rows.append(",".join(str(x) for x in (md["m"]["id"], "top" if md["flipped"] else "bottom", f"{tilt:.2f}", f"{az:.1f}",
                    f"{u[0]:.4f}", f"{u[1]:.4f}", f"{u[2]:.4f}", *(f"{v:.3f}" for v in fe), *(f"{v:.3f}" for v in be))))
    (out / "features.csv").write_text("\n".join(rows) + "\n")
    for nm, sol in (("cap_bottom", bot), ("cap_top", top)):
        bb = sol.bounding_box()
        say(f"{nm} stock envelope: x {bb.min.X:.1f}..{bb.max.X:.1f}, y {bb.min.Y:.1f}..{bb.max.Y:.1f}, z {bb.min.Z:.1f}..{bb.max.Z:.1f} mm "
            f"(height {bb.max.Z - bb.min.Z:.1f} mm)")

    # ---- export
    def put(name, solids):
        comp = Compound(children=solids) if len(solids) > 1 else solids[0]
        export_step(comp, str(out / name))

    export_step(bot, str(out / "cap_bottom.step"))
    export_step(top, str(out / "cap_top.step"))
    export_step(rod_solid, str(out / "rod.step"))
    export_step(next(p[1] for p in parts if p[0] == "cell_0"), str(out / "load_cell_modified.step"))
    h0 = hardware[0]
    export_step(Compound(children=[h0["bolt"], h0["wash_f"], h0["shim"], h0["screw_free"]]), str(out / "fasteners.step"))
    import copy
    coloured = []
    for name, sol, mat, mid, col in parts:
        sc = copy.copy(sol)
        sc.label, sc.color = name, Color(*col)
        coloured.append(sc)
    export_step(Compound(children=coloured, label="megadroid_ft_sensor"), str(out / "sensor_assembly.step"))
    if args.mesh:
        import json
        mesh = []
        flips = {m["id"]: bool(m.get("flipped")) for m in ms}
        for name, sol, mat, mid, col in parts:
            verts, tris = sol.tessellate(0.04, 0.35)
            v = [[round(p.X, 3), round(p.Z, 3), round(-p.Y, 3)] for p in verts]     # (x, y, z) -> (x, z, -y): a rotation
            mesh.append(dict(name=name, kind=name.split("_")[0], member=mid, flipped=bool(mid is not None and flips[mid]),
                             material=mat, colour=list(col),
                             v=[c for p in v for c in p], i=[int(k) for t in tris for k in t]))
        Path(args.mesh).write_text(json.dumps(dict(gap=GAP, cap=CAP_T, across_flats=88.85, parts=mesh), separators=(",", ":")))
        say(f"mesh -> {args.mesh}: {sum(len(m['i']) // 3 for m in mesh)} triangles")
    (out / "checks.txt").write_text("\n".join(log) + "\n")
    say(f"\nwrote {out}")


if __name__ == "__main__":
    main()

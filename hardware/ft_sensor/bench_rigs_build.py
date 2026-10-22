#!/usr/bin/env python3
"""
Solid CAD (STEP) for the F/T sensor bench tests: H (cantilever, on-axis and off-axis), H2 (guided), H3 (two-cell V)
and R (single strut).  Needs build123d, like cad_build.py (which supplies the cell, rod, screw and shim models).

    python3 hardware/ft_sensor/bench_rigs_build.py [--out DIR]

ONE FRAME FOR ALL FIVE RIGS.  A rigid tower hangs from its top plate; weights hang below.  Frame: mm, z up, x along the
cell, origin on the underside of the top plate.  Swapping a few parts turns the tower into each test:

    H    cell bolted to a block under the top plate at one hole, eyebolt in the other hole, weights on the eyebolt
    H    off-axis: the same cell on the side face of a tall block, so gravity loads it across its width
    H2   cell bolted to the fixed block at one end and to a carriage at the other; the carriage slides on two
         ground shafts (linear bushings), so the free end can move only along the bolt axis
    H3   two cells in an inverted V between an upper and a lower plate, bolt axes at +-45 deg, holes 30 mm apart
    R    one complete sensor member (reamed cell, shim, flexure rod, carriage) with the load line on the rod axis

WHAT IS DESIGN HERE AND WHAT IS A CHOICE.  Cell, rod, screws, shim, rod length: from design/ft_sensor.yaml through
cad_build.py.  Every other dimension (plates, blocks, shafts, eyebolts) is a bench-tooling choice made here, sized
by hand for 75 kg and not by FEA.  Catalogue parts (eyebolt, eye nut, linear bushing, socket screws) are drawn at
nominal size: check the supplier's drawing before ordering.

WRITES  (default hardware/ft_sensor/cad/bench_rigs/)
    rig_H_cantilever.step, rig_H_offaxis.step, rig_H2_guided.step, rig_H3_vee.step, rig_R_strut.step
    parts/*.step     the machined parts, one file each
    parts_list.csv   every part of every rig, with quantity and what it is
    checks.txt       interference between parts, thread engagement, wall checks
"""
import argparse
import copy
import csv
import math
import sys
from pathlib import Path

import numpy as np
from build123d import Box, Cylinder, Torus, Pos, Rot, Compound, Color, export_step, Location

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import cad_build as cb  # noqa: E402
import design_synthesis as ds  # noqa: E402

# ---- tooling choices (bench only; not robot design values) ---------------------------------------------------------
TOP_X, TOP_Y, TOP_T = (-60.0, 90.0), (-60.0, 60.0), 12.0     # top plate
HC = 160.0                                                    # column length = clear height between the plates
COL_D, COL_XY, STUD_D8, STUD_L = 12.0, [(-45, -50), (-45, 50), (75, -50), (75, 50)], 8.0, 10.0
SHAFT_D, SHAFT_Y, SHAFT_X, SHAFT_SOCKET = 8.0, 30.0, 15.0, 8.0
BUSH_OD, BUSH_L = 15.0, 45.0                                  # LM8LUU linear bushing
CARR_X, CARR_Y = (2.0, 28.0), 42.0                            # carriage footprint (x range, half-width in y)
BLOCK_X, BLOCK_Y, BLOCK_T = (-30.0, 0.0), 15.0, 15.0          # blocks A1, A2 under the top plate
A3_H = 50.0                                                   # tall block for the off-axis test
ATTACH_Y = 10.0                                               # the two M6 screws that hold a block to the top plate
CELL_X0 = -15.0                                               # fixed-end hole of the cell, x
CELL_X1 = +15.0                                               # free-end hole
DIAL_XY = (50.0, 0.0)                                         # dial-indicator stem hole in the top plate
CLR6 = 6.4                                                    # M6 clearance hole
M6 = 6.0                                                      # tapped M6 drawn at its major diameter (as in cad_build)
H3_U = dict(x=(-45.0, 45.0), y=(-30.0, 30.0), t=15.0, cell_cx=30.0, face_to_plate=26.0)
H3_SCREWS = [(-35, -20), (-35, 20), (35, -20), (35, 20)]
EYE = dict(collar_d=14.0, collar_t=3.0, ring_R=7.5, ring_r=2.5, shank_l=10.0)
WEDGE_CHECKS = []
TOL_V = 0.05                                                  # mm3: overlaps below this are surface contact


def cylz(x, y, z0, z1, d):
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(d / 2, abs(z1 - z0))


def boxz(x0, x1, y0, y1, z0, z1):
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def place(shape, origin, axis, xhint=None):
    """Move a part built with its own z axis along `axis` and its origin at `origin`."""
    ax = np.asarray(axis, float)
    ax = ax / np.linalg.norm(ax)
    xh = cb.perp(ax) if xhint is None else np.asarray(xhint, float)
    return shape.moved(cb.frame(origin, xh, ax))


# ---- catalogue-size parts ---------------------------------------------------------------------------------------------
def make_eyebolt():
    """Lifting eyebolt (DIN 580 style) drawn at nominal M6 size.  Local z points out of the joint: collar underside
    at z = 0 (touches the part), thread goes down to z = -shank_l, the ring is above the collar."""
    e = EYE
    collar = Pos(0, 0, e["collar_t"] / 2) * Cylinder(e["collar_d"] / 2, e["collar_t"])
    shank = Pos(0, 0, -e["shank_l"] / 2) * Cylinder(M6 / 2, e["shank_l"])
    zc = e["collar_t"] + e["ring_R"] + e["ring_r"]
    ring = Pos(0, 0, zc) * (Rot(90, 0, 0) * Torus(e["ring_R"], e["ring_r"]))
    neck = Pos(0, 0, e["collar_t"] + 1.5) * Cylinder(3.0, 3.0)
    return collar + shank + neck + ring


def make_eyenut(thread_depth=13.0):
    """Eye nut (DIN 582 style), nominal M6.  Local z points out of the joint, face against the part at z = 0, the
    stud comes in from -z.  Blind thread (thread_depth), ring above."""
    body = Pos(0, 0, 6.5) * Cylinder(6.0, 13.0)
    body = body - Pos(0, 0, thread_depth / 2) * Cylinder(M6 / 2, thread_depth)
    zc = 13.0 + EYE["ring_R"] + EYE["ring_r"]
    ring = Pos(0, 0, zc) * (Rot(90, 0, 0) * Torus(EYE["ring_R"], EYE["ring_r"]))
    return body + ring


def make_stud(length):
    return Pos(0, 0, -length / 2) * Cylinder(M6 / 2, length)        # tip at z = 0, runs to -length


def make_bushing():
    t = Pos(0, 0, 0) * Cylinder(BUSH_OD / 2, BUSH_L)
    return t - Cylinder(SHAFT_D / 2, BUSH_L + 1)


# ---- machined parts ---------------------------------------------------------------------------------------------------
def top_plate():
    p = boxz(*TOP_X, *TOP_Y, 0.0, TOP_T)
    for (x, y) in COL_XY:                                              # M8 tapped, column studs
        p -= cylz(x, y, 0.0, STUD_L, STUD_D8)
    for sy in (-SHAFT_Y, SHAFT_Y):                                     # ground-shaft sockets (Ø8 H7, set screw)
        p -= cylz(SHAFT_X, sy, 0.0, SHAFT_SOCKET, SHAFT_D)
    p -= cylz(DIAL_XY[0], DIAL_XY[1], -1.0, TOP_T + 1, 8.0)           # dial-indicator stem (Ø8 h6, clamp screw)
    for y in (-ATTACH_Y, 0.0, ATTACH_Y):                               # block attachment / cell screw, clearance
        p -= cylz(CELL_X0, y, -1.0, TOP_T + 1, CLR6)
    for (x, y) in H3_SCREWS:
        p -= cylz(x, y, -1.0, TOP_T + 1, CLR6)
    return p


def bottom_plate():
    z1 = -HC
    p = boxz(*TOP_X, *TOP_Y, z1 - TOP_T, z1)
    for (x, y) in COL_XY:
        p -= cylz(x, y, z1 - STUD_L, z1, STUD_D8)
    for sy in (-SHAFT_Y, SHAFT_Y):
        p -= cylz(SHAFT_X, sy, z1 - SHAFT_SOCKET, z1, SHAFT_D)
    p -= cylz(SHAFT_X, 0.0, z1 - TOP_T - 1, z1 + 1, 40.0)             # the hanger passes through
    return p


def column():
    c = cylz(0, 0, -HC, 0.0, COL_D)
    return c + cylz(0, 0, 0.0, STUD_L, STUD_D8) + cylz(0, 0, -HC - STUD_L, -HC, STUD_D8)


def shaft():
    return cylz(0, 0, -HC - SHAFT_SOCKET, SHAFT_SOCKET, SHAFT_D)


def block_a(kind):
    """kind 'clear': Ø6.4 through hole for a screw from above (native cell, tapped).  'tapped': blind M6 from the
    underside for the reamed cell's bolt (as the wedge in the sensor).  Two M6 tapped holes from the top attach it."""
    b = boxz(*BLOCK_X, -BLOCK_Y, BLOCK_Y, -BLOCK_T, 0.0)
    for y in (-ATTACH_Y, ATTACH_Y):
        b -= cylz(CELL_X0, y, -10.0, 0.0, M6)
    if kind == "clear":
        b -= cylz(CELL_X0, 0.0, -BLOCK_T - 1, 1.0, CLR6)
    else:
        b -= cylz(CELL_X0, 0.0, -BLOCK_T, -BLOCK_T + 7.5, M6)
    return b


def block_a3():
    """Tall block: the cell is bolted to its +y face, long axis along x, so gravity loads the cell across its width."""
    b = boxz(*BLOCK_X, -BLOCK_Y, BLOCK_Y, -A3_H, 0.0)
    for y in (-ATTACH_Y, ATTACH_Y):
        b -= cylz(CELL_X0, y, -10.0, 0.0, M6)
    hole = Pos(CELL_X0, 0.0, -A3_H / 2) * Rot(90, 0, 0) * Cylinder(CLR6 / 2, 2 * BLOCK_Y + 2)
    return b - hole


def carriage(top_z, kind):
    """Sliding block on the two shafts.  kind 'h2': Ø6.4 through the load axis (stud passes, eye nut below).
    kind 'r': blind M6 (7 deep) in the top face for the rod stud, blind M6 (12 deep) in the bottom face for the
    eyebolt.  A tab with a post carries the dial-indicator target."""
    z0 = top_z - BUSH_L
    c = boxz(*CARR_X, -CARR_Y, CARR_Y, z0, top_z)
    for sy in (-SHAFT_Y, SHAFT_Y):
        c -= cylz(SHAFT_X, sy, z0 - 1, top_z + 1, BUSH_OD)
    if kind == "h2":
        c -= cylz(SHAFT_X, 0.0, z0 - 1, top_z + 1, CLR6)
    else:
        c -= cylz(SHAFT_X, 0.0, top_z - 7.0, top_z, M6)
        c -= cylz(SHAFT_X, 0.0, z0, z0 + 12.0, M6)
    tab = boxz(CARR_X[1], DIAL_XY[0] + 10.0, -8.0, 8.0, top_z - 10.0, top_z)
    tab -= cylz(DIAL_XY[0], 0.0, top_z - 8.0, top_z, M6)
    return c + tab


def dial_post(top_z):
    h = (-12.0) - top_z                                                # top of the post: 12 mm below the plate
    return cylz(DIAL_XY[0], 0.0, top_z, top_z + h, 10.0) + cylz(DIAL_XY[0], 0.0, top_z - 8.0, top_z, M6)


# ---- H3: two cells in an inverted V ---------------------------------------------------------------------------------
def h3_geometry():
    s = c = math.sqrt(0.5)
    zc = -(H3_U["t"] + H3_U["face_to_plate"])
    out = []
    for sx in (+1, -1):
        u = np.array([sx * s, 0.0, c])                      # bolt axis
        l = np.array([sx * c, 0.0, -s])                     # long axis, descending outward
        C = np.array([sx * H3_U["cell_cx"], 0.0, zc])
        out.append(dict(sx=sx, u=u, l=l, C=C, up=C - cb.ds.HOLE * l, lo=C + cb.ds.HOLE * l))
    return out, zc


def wedge_plate(role, geo, zc):
    """Upper or lower plate with two integral 16 x 16 wedge pads parallel to the cell faces (CNC, 3+2 axis)."""
    U = H3_U
    if role == "upper":
        pl = boxz(*U["x"], *U["y"], -U["t"], 0.0)
        top_cut = boxz(-200, 200, -200, 200, -U["t"] - 200, 0.0)
    else:
        z1 = zc - U["face_to_plate"]
        pl = boxz(-62, 62, *U["y"], z1 - U["t"], z1)
        top_cut = boxz(-200, 200, -200, 200, z1 - U["t"], z1 + 200)
    body = pl
    cuts = []
    for g in geo:
        pt = g["up"] if role == "upper" else g["lo"]
        sign = +1.0 if role == "upper" else -1.0           # wedge grows away from the cell along sign*u
        face_c = pt + (cb.ds.CT / 2.0) * sign * g["u"]     # the cell face the wedge touches
        fr = cb.frame(face_c, g["l"], sign * g["u"])       # local z points away from the cell, into the wedge
        box = Pos(0, 0, 20.0) * Box(16.0, 16.0, 40.0)
        wedge = box.moved(fr)
        body = body + wedge
        # counterbore: screw head sits flat on a floor perpendicular to the bolt axis, 9 mm inside the plate surface
        if role == "upper":
            t_floor = (-9.0 - face_c[2]) / (sign * g["u"][2])
        else:
            t_floor = ((zc - U["face_to_plate"] - U["t"] + 9.0) - face_c[2]) / (sign * g["u"][2])
        origin = face_c + t_floor * sign * g["u"]
        cuts.append((origin, sign * g["u"], face_c, fr))
    body = body & top_cut                                   # the wedge boxes end at the plate surface
    for (origin, axis, face_c, fr) in cuts:
        d = float(np.linalg.norm(origin - face_c))
        probe = place(Pos(0, 0, d / 2) * Cylinder(4.2, d), face_c, axis)            # Ø8.4 = clearance hole + 1 mm wall
        inside = (body & probe).volume / probe.volume
        WEDGE_CHECKS.append((role, d, inside))
    for (origin, axis, face_c, fr) in cuts:
        hole = place(Pos(0, 0, 0) * Cylinder(CLR6 / 2, 120.0), face_c, axis)
        cbore = place(Pos(0, 0, 30.0) * Cylinder(5.5, 60.0), origin, axis)
        body = body - hole - cbore
    if role == "upper":
        for (x, y) in H3_SCREWS:
            body -= cylz(x, y, -10.0, 0.0, M6)
    else:
        z1 = zc - U["face_to_plate"] - U["t"]
        body -= cylz(0.0, 0.0, z1, z1 + 12.0, M6)           # the eyebolt
    return body, cuts


# ---- assemblies ---------------------------------------------------------------------------------------------------------
def assemble(rig, out_dir, say, state):
    """Return a list of (name, solid, colour, material, note). The geometry of each rig is built here."""
    parts = []

    def add(name, solid, col, mat, note=""):
        parts.append((name, solid, col, mat, note))

    GREY, ALU, STEEL, YEL, GOLD, RED = (0.55, 0.57, 0.6), (0.78, 0.8, 0.84), (0.25, 0.27, 0.3), (0.95, 0.8, 0.2), (0.85, 0.65, 0.3), (0.8, 0.25, 0.2)
    cell_nat = lambda: (Box(ds.CL, ds.CW, ds.CT) - cylz(-ds.HOLE, 0, -9, 9, M6) - cylz(ds.HOLE, 0, -9, 9, M6))

    add("top_plate", top_plate(), ALU, "6061/7075 plate, 12 mm", "Ø8 sockets for shafts, M8 for columns, Ø8 dial stem")
    for i, (x, y) in enumerate(COL_XY):
        add(f"column_{i}", column().moved(Location((x, y, 0))), STEEL, "steel Ø12", "M8 stud at each end")
    add("bottom_plate", bottom_plate(), ALU, "6061 plate, 12 mm")

    if rig in ("H", "H_off", "H2"):
        zc_cell = -BLOCK_T - ds.CT / 2
        if rig == "H_off":
            add("block_a3", block_a3().moved(Location((0, 0, 0))), ALU, "6061 block 30 x 30 x 50")
            for y in (-ATTACH_Y, ATTACH_Y):
                add(f"attach_screw_{'p' if y > 0 else 'n'}", place(cb.make_screw(cb.BOLT_HEAD, 20.0), (CELL_X0, y, TOP_T), (0, 0, 1)),
                    STEEL, "M6x20 SHCS 12.9")
            cell_c = np.array([0.0, BLOCK_Y + ds.CT / 2, -A3_H / 2])
            cell = cell_nat().moved(cb.frame(cell_c, (1, 0, 0), (0, 1, 0)))
            add("cell", cell, YEL, "harvested cell, native threaded holes")
            add("cell_screw", place(cb.make_screw(cb.BOLT_HEAD, 40.0), (CELL_X0, -BLOCK_Y, -A3_H / 2), (0, -1, 0)),
                STEEL, "M6x40 SHCS 12.9", "from the back face, into the cell's thread")
            add("eyebolt", place(make_eyebolt(), (CELL_X1, BLOCK_Y + ds.CT, -A3_H / 2), (0, 1, 0)), RED,
                "M6 eyebolt DIN 580, nominal", "load line is about 13 mm outboard of the cell: see the rig note")
            state["load_pt"] = (CELL_X1, BLOCK_Y + ds.CT + 13.0, -A3_H / 2)
        else:
            add("block_a1", block_a("clear"), ALU, "6061 block 30 x 30 x 15")
            for y in (-ATTACH_Y, ATTACH_Y):
                add(f"attach_screw_{'p' if y > 0 else 'n'}", place(cb.make_screw(cb.BOLT_HEAD, 20.0), (CELL_X0, y, TOP_T), (0, 0, 1)),
                    STEEL, "M6x20 SHCS 12.9")
            cell_c = np.array([0.0, 0.0, -BLOCK_T - ds.CT / 2])
            add("cell", cell_nat().moved(cb.frame(cell_c, (1, 0, 0), (0, 0, 1))), YEL, "harvested cell, native threaded holes")
            add("cell_screw", place(cb.make_screw(cb.BOLT_HEAD, 35.0), (CELL_X0, 0, TOP_T), (0, 0, 1)), STEEL,
                "M6x35 SHCS 12.9", "head on the top plate, through plate and block into the cell's thread")
            if rig == "H":
                add("eyebolt", place(make_eyebolt(), (CELL_X1, 0, -BLOCK_T - ds.CT), (0, 0, -1)), RED, "M6 eyebolt DIN 580, nominal")
                state["load_pt"] = (CELL_X1, 0, -BLOCK_T - ds.CT - 3.0 - 2 * EYE["ring_R"] - 2.5)
        if rig == "H2":
            top_z = -BLOCK_T - ds.CT
            add("carriage_h2", carriage(top_z, "h2"), ALU, "6061 block", "two Ø15 bores for the bushings; tab for the dial target")
            for i, sy in enumerate((-SHAFT_Y, SHAFT_Y)):
                add(f"shaft_{i}", shaft().moved(Location((SHAFT_X, sy, 0))), STEEL, "ground shaft Ø8 h6 x 176")
                add(f"bushing_{i}", make_bushing().moved(Location((SHAFT_X, sy, top_z - BUSH_L / 2))), GOLD, "LM8LUU Ø8/15/45")
            add("dial_post", dial_post(top_z), STEEL, "Ø10 steel post, M6 stud")
            stud_len = 65.0
            add("stud", make_stud(stud_len).moved(Location((CELL_X1, 0, -BLOCK_T))), STEEL, "M6x65 all-thread",
                "into the cell's thread at the top, through the carriage")
            add("eye_nut", place(make_eyenut(13.0), (CELL_X1, 0, top_z - BUSH_L), (0, 0, -1)), RED, "M6 eye nut DIN 582, nominal")
            state["load_pt"] = (CELL_X1, 0, top_z - BUSH_L - 13.0 - 2 * EYE["ring_R"] - 2.5)

    if rig == "R":
        # A2 block (tapped), reamed cell, shim, rod, carriage with boss face at the rod's end
        add("block_a2", block_a("tapped"), ALU, "6061 block 30 x 30 x 15", "M6 blind 7.5 mm: the fixed-end thread, as in the sensor wedge")
        for y in (-ATTACH_Y, ATTACH_Y):
            add(f"attach_screw_{'p' if y > 0 else 'n'}", place(cb.make_screw(cb.BOLT_HEAD, 20.0), (CELL_X0, y, TOP_T), (0, 0, 1)),
                STEEL, "M6x20 SHCS 12.9")
        z_ct, z_cb = -BLOCK_T, -BLOCK_T - ds.CT
        cell = cb.make_cell().moved(cb.frame((0.0, 0.0, (z_ct + z_cb) / 2), (1, 0, 0), (0, 0, 1)))
        add("cell", cell, YEL, "harvested cell, reamed 6.1 / 7.0")
        add("washer", place(cb.make_washer(), (CELL_X0, 0, z_cb), (0, 0, -1)), STEEL, "6.4 x 12 x 1.6 hardened")
        add("cell_bolt", place(cb.make_screw(cb.BOLT_HEAD, 16.0), (CELL_X0, 0, z_cb - cb.WASHER["t"]), (0, 0, -1)), STEEL,
            "M6x16 SHCS 12.9", "from below into the block, 5.9 mm engaged: same as the sensor")
        add("shim", place(cb.make_shim(), (CELL_X1, 0, z_cb), (0, 0, -1)), GOLD, "0.25 stainless")
        rod, rinfo = cb.make_rod(state["l_free"])
        z_rod0 = z_cb - cb.SHIM_NOM
        add("rod", rod.moved(cb.frame((CELL_X1, 0, z_rod0), (1, 0, 0), (0, 0, -1))), (0.7, 0.72, 0.78), "7075-T6 turned",
            "the rod of the sensor, unchanged")
        add("free_screw", place(cb.make_screw(cb.FREE_HEAD, cb.FREE_LEN), (CELL_X1, 0, z_ct), (0, 0, 1)), STEEL,
            "M6x14 low-head SHCS", "from the cell's upper side into the rod end, as in the sensor")
        z_boss = z_rod0 - rinfo["L"]
        state["top_z"] = z_boss
        add("carriage_r", carriage(z_boss, "r"), ALU, "6061 block", "top: blind M6 for the rod stud; bottom: blind M6 for the eyebolt")
        for i, sy in enumerate((-SHAFT_Y, SHAFT_Y)):
            add(f"shaft_{i}", shaft().moved(Location((SHAFT_X, sy, 0))), STEEL, "ground shaft Ø8 h6 x 176")
            add(f"bushing_{i}", make_bushing().moved(Location((SHAFT_X, sy, z_boss - BUSH_L / 2))), GOLD, "LM8LUU Ø8/15/45")
        add("dial_post", dial_post(z_boss), STEEL, "Ø10 steel post, M6 stud")
        add("eyebolt", place(make_eyebolt(), (CELL_X1, 0, z_boss - BUSH_L), (0, 0, -1)), RED, "M6 eyebolt DIN 580, nominal")
        state["load_pt"] = (CELL_X1, 0, z_boss - BUSH_L - 3.0 - 2 * EYE["ring_R"] - 2.5)

    if rig == "H3":
        geo, zc = h3_geometry()
        WEDGE_CHECKS.clear()
        up, cu = wedge_plate("upper", geo, zc)
        lp, cl = wedge_plate("lower", geo, zc)
        add("h3_upper_plate", up, ALU, "6061 plate 15 mm with two integral wedges", "5-axis or 3+2: pads square to the bolt axes")
        add("h3_lower_plate", lp, ALU, "6061 plate 15 mm with two integral wedges")
        for k, (x, y) in enumerate(H3_SCREWS):
            add(f"plate_screw_{k}", place(cb.make_screw(cb.BOLT_HEAD, 20.0), (x, y, TOP_T), (0, 0, 1)), STEEL, "M6x20 SHCS 12.9")
        z1 = zc - H3_U["face_to_plate"] - H3_U["t"]
        add("eyebolt", place(make_eyebolt(), (0, 0, z1), (0, 0, -1)), RED, "M6 eyebolt DIN 580, nominal")
        state["load_pt"] = (0, 0, z1 - 3.0 - 2 * EYE["ring_R"] - 2.5)
        for g, (o_up, ax_up, fc_up, _), (o_lo, ax_lo, fc_lo, _) in zip(geo, cu, cl):
            k = "pos" if g["sx"] > 0 else "neg"
            cell_fr = cb.frame(g["C"], g["l"], g["u"])
            add(f"cell_{k}", cell_nat().moved(cell_fr), YEL, "harvested cell, native threaded holes")
            for nm, o, ax, fc in (("up", o_up, ax_up, fc_up), ("lo", o_lo, ax_lo, fc_lo)):
                d_head = float(np.linalg.norm(o - fc))                # head underside to the cell face along the axis
                need = d_head + ds.CT + 1.0                           # through the wedge, the cell, 1 mm past
                std = next(L for L in (16, 18, 20, 25, 30, 35, 40, 45, 50, 55, 60) if L >= need)
                add(f"screw_{nm}_{k}", place(cb.make_screw(cb.BOLT_HEAD, float(std)), o, ax), STEEL, f"M6x{std} SHCS 12.9",
                    f"{std - d_head - ds.CT:.1f} mm past the cell")
                state.setdefault("h3_screws", []).append((f"screw_{nm}_{k}", std, d_head))

    return parts


def interference(parts, say, skip=()):
    names = [p[0] for p in parts]
    bbs = [p[1].bounding_box() for p in parts]

    def near(a, b, pad=0.2):
        return not (a.max.X + pad < b.min.X or b.max.X + pad < a.min.X or a.max.Y + pad < b.min.Y or
                    b.max.Y + pad < a.min.Y or a.max.Z + pad < b.min.Z or b.max.Z + pad < a.min.Z)

    n = 0
    for i in range(len(parts)):
        for j in range(i + 1, len(parts)):
            if not near(bbs[i], bbs[j]):
                continue
            try:
                v = (parts[i][1] & parts[j][1]).volume
            except Exception:
                v = float("nan")
            if v > TOL_V or v != v:
                n += 1
                say(f"    OVERLAP {names[i]} x {names[j]}: {v:.2f} mm3")
    say(f"    {n} overlapping pairs" if n else "    no overlapping pairs")
    return n


CONTACTS = {
    "H": [("cell", "block_a1"), ("cell", "eyebolt"), ("block_a1", "top_plate"), ("cell_screw", "cell")],
    "H_off": [("cell", "block_a3"), ("cell", "eyebolt"), ("block_a3", "top_plate"), ("cell_screw", "cell")],
    "H2": [("cell", "block_a1"), ("cell", "carriage_h2"), ("stud", "cell"), ("eye_nut", "carriage_h2"), ("stud", "eye_nut")],
    "R": [("cell", "block_a2"), ("cell", "shim"), ("shim", "rod"), ("rod", "carriage_r"), ("eyebolt", "carriage_r"),
          ("cell_bolt", "block_a2"), ("free_screw", "rod")],
    "H3": [("cell_pos", "h3_upper_plate"), ("cell_pos", "h3_lower_plate"), ("cell_neg", "h3_upper_plate"),
           ("cell_neg", "h3_lower_plate"), ("eyebolt", "h3_lower_plate")],
}


def contacts_and_gaps(rig, parts, say):
    sol = {p[0]: p[1] for p in parts}
    say("  SEATING (distance between parts that must touch; 0 = seated)")
    bad = 0
    for a, b in CONTACTS.get(rig, []):
        if a in sol and b in sol:
            d = sol[a].distance_to(sol[b])
            bad += d > 0.02
            say(f"    {a} - {b}: {d:.3f} mm" + ("" if d <= 0.02 else "   NOT SEATED"))
    touch = {frozenset(c) for c in CONTACTS.get(rig, [])}
    names = list(sol)
    bbs = {n: sol[n].bounding_box() for n in names}
    tight = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if frozenset((a, b)) in touch:
                continue
            A, B = bbs[a], bbs[b]
            if (A.max.X + 3 < B.min.X or B.max.X + 3 < A.min.X or A.max.Y + 3 < B.min.Y or B.max.Y + 3 < A.min.Y or
                    A.max.Z + 3 < B.min.Z or B.max.Z + 3 < A.min.Z):
                continue
            d = sol[a].distance_to(sol[b])
            if d < 1.0:
                tight.append((d, a, b))
    n_touch = sum(1 for t in tight if t[0] <= 0.02)
    tight = [t for t in tight if t[0] > 0.02]
    say(f"  {n_touch} further pairs touch (threads, bearings, plates on columns); not listed")
    say("  TIGHT GAPS (0.02 to 1 mm between parts not listed as seated)")
    for d, a, b in sorted(tight)[:14]:
        say(f"    {a} - {b}: {d:.3f} mm")
    if not tight:
        say("    none")
    return bad


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(HERE / "cad" / "bench_rigs"))
    args = ap.parse_args()
    out = Path(args.out)
    (out / "parts").mkdir(parents=True, exist_ok=True)
    log = []

    def say(s=""):
        print(s)
        log.append(s)

    cb.ds.configure(cb.CAP_T, cb.ROD_MIN, cb.RATED_KG)
    cb.ds.set_envelope(88.85, None)
    cb.ds.REPS = 1
    ms = cb.ds.members_inv(cb.CAND, "rods")
    cb.STUD_B = 6.5                                    # as built by cad_build.main (boss hole 7.0 mm deep)
    l_free = cb.rod_free_length(ms[0])
    say(f"rod free length (cell face to boss face, shim included): {l_free:.2f} mm")

    rigs = [("H", "rig_H_cantilever", "H: cantilever, on-axis"),
            ("H_off", "rig_H_offaxis", "H: cantilever, off-axis (load across the cell's width)"),
            ("H2", "rig_H2_guided", "H2: guided end"),
            ("H3", "rig_H3_vee", "H3: two-cell inverted V"),
            ("R", "rig_R_strut", "R: single sensor member")]
    rows = []
    machined = {}
    for key, fname, title in rigs:
        say(f"\n=== {title}  ({fname}.step)")
        state = {"l_free": l_free}
        parts = assemble(key, out, say, state)
        n = interference(parts, say)
        contacts_and_gaps(key, parts, say)
        if key == "H3":
            for role, d, inside in WEDGE_CHECKS:
                say(f"    wedge hole ({role} plate, {d:.1f} mm deep, 1 mm wall): {'OK' if inside > 0.995 else 'BREAKS OUT'} ({inside:.3f})")
        sol = {p[0]: p[1] for p in parts}
        if "carriage_r" in sol or "carriage_h2" in sol:
            cz = sol.get("carriage_r", sol.get("carriage_h2")).bounding_box()
            say(f"    carriage bottom is {cz.min.Z + HC:.0f} mm above the bottom plate; the load point is "
                f"{state['load_pt'][2] + HC:.0f} mm above it, so the hanger passes through the plate's Ø40 hole")
        bb = Compound(children=[p[1] for p in parts]).bounding_box()
        say(f"    envelope x {bb.min.X:.0f}..{bb.max.X:.0f}, y {bb.min.Y:.0f}..{bb.max.Y:.0f}, z {bb.min.Z:.0f}..{bb.max.Z:.0f} mm; "
            f"load point z {state['load_pt'][2]:.0f}")
        for (nm, std, d) in state.get("h3_screws", []):
            say(f"    {nm}: M6x{std}, head underside {d:.1f} mm from the cell face")
        children = []
        for (nm, sol, col, mat, note) in parts:
            if mat.startswith("6061") or mat.startswith("7075") or nm == "rod":
                if nm not in machined:
                    machined[nm] = sol
                    export_step(Compound(children=[sol]), str(out / "parts" / f"{nm}.step"))
            sc = copy.copy(sol)
            sc.label, sc.color = nm, Color(*col)
            children.append(sc)
            rows.append((key, nm, mat, note))
        export_step(Compound(children=children, label=fname), str(out / f"{fname}.step"))

    with open(out / "parts_list.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["rig", "part", "material_or_spec", "note"])
        w.writerows(rows)
    (out / "checks.txt").write_text("\n".join(log) + "\n")
    say(f"\nwrote {out}")


if __name__ == "__main__":
    main()

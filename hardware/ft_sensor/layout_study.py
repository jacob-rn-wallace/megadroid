#!/usr/bin/env python3
"""
F/T sensor layout study -- compare candidate cell arrangements on paper.

Each cell, with its stiff strut or flexure, is idealised as a two-force member: it
reads the force along one line of action (a point p_i and a unit direction u_i, in
the sensor frame, origin at the sensing-plane centre, z up). A wrench w = [F; M]
applied at the origin loads the cells so that  w = A f,  A's columns being
[u_i ; p_i x u_i]. Reading the wrench back is  w = A f_measured, and the cell
forces for a given wrench are f = pinv(A) w.

WHAT IT REPORTS, per layout
  rank        A must have rank 6 for a 6-axis sensor.
  cond        condition number of A with moment rows scaled by a reference length
              (1 = every axis measured equally well; large = some axis is poorly
              resolved or noise-amplified).
  worst cell  largest cell force over the corners of the assumed peak wrench box,
              as a fraction of the cell rating.
  signal      cell output at that worst load, mV, at the listed excitation.
  resolution  wrench resolution (peak to peak) from the assumed cell noise floor.

WHAT IT DOES NOT DO
  - It does not check that the cells FIT: footprint, strut clearance and the M6 hole
    positions need CAD, and are the next step for any layout that survives here.
  - A bending-beam cell reads force only perpendicular to its face. The line of
    action here is idealised; the real strut or flexure must make that true.
  - The peak loads, the noise floor and the cell rating are assumptions or
    unverified reseller figures (design/ft_sensor.yaml). Treat every number as
    relative between layouts, not absolute.

    python3 hardware/ft_sensor/layout_study.py
    python3 hardware/ft_sensor/layout_study.py --wrenches ft_loads.npz

With --wrenches (from tools/log_ft_loads.py: columns Fx Fy Fz Mx My Mz, N and N.m, at the
ankle) each layout is also scored on the recorded simulation loads. The sensor centre
sits below the ankle, so moments are shifted to it by the lever arm; the shift's sign
is ambiguous without the site convention, so BOTH signs are scored and the worse shown.
"""

import argparse
import itertools
import math
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
FT = yaml.safe_load((ROOT / "design" / "ft_sensor.yaml").read_text())
SENS = yaml.safe_load((ROOT / "design" / "sensors.yaml").read_text())
FORM = SENS["end_of_limb_sensing"]["ft_sensor"]["form_factor"]

G = 9.81
RATED_N = FT["load_cell"]["rating"]["rated_load_kg"] * G
FS_MV = FT["load_cell"]["rating"]["rated_output_mv_per_v"] * FT["load_cell"]["rating"]["excitation_v"]
NOISE_UV = FT["study_assumptions"]["noise_floor_uv"]
PEAK = FT["study_assumptions"]["peak_wrench"]
W_PEAK = np.array([PEAK["fx_n"], PEAK["fy_n"], PEAK["fz_n"],
                   PEAK["mx_nm"], PEAK["my_nm"], PEAK["mz_nm"]], dtype=float)

L_REF = 0.05                                  # m, moment-row scaling for the condition number
R_FLATS = FORM["across_flats_mm"] / 2000.0    # m, inscribed radius
H_SENSOR = FORM["height_mm"] / 1000.0         # m
CELL_W = FT["load_cell"]["dimensions_mm"]["width"] / 1000.0
# Largest radius at which a cell can sit with its whole width inside the hexagon.
R_MAX = R_FLATS - CELL_W / 2.0


def line(point, direction):
    u = np.asarray(direction, float)
    u = u / np.linalg.norm(u)
    p = np.asarray(point, float)
    return np.concatenate([u, np.cross(p, u)])


def polar(r, deg, z=0.0):
    a = math.radians(deg)
    return np.array([r * math.cos(a), r * math.sin(a), z])


def tangent(deg):
    a = math.radians(deg)
    return np.array([-math.sin(a), math.cos(a), 0.0])


# ---- candidate layouts: each returns a 6 x n matrix A -----------------------

def stewart(r_top, r_base, dtop, dbase, height, offset_deg=0.0):
    """Six struts, three pairs. Strut i joins a base point to a top point; the line
    of action passes through the top point."""
    cols = []
    for k in range(3):
        c = 120 * k + offset_deg
        for s in (+1, -1):
            top = polar(r_top, c + s * dtop, height)
            base = polar(r_base, c - s * dbase, 0.0)
            cols.append(line(top, top - base))
    return np.array(cols).T


def three_plus_three(r, vert_deg=(90, 210, 330), tang_deg=(30, 150, 270)):
    """Three vertical cells and three horizontal tangential cells. Decoupled by
    construction: verticals read Fz, Mx, My; tangentials read Fx, Fy, Mz."""
    cols = [line(polar(r, a), (0, 0, 1)) for a in vert_deg]
    cols += [line(polar(r, a), tangent(a)) for a in tang_deg]
    return np.array(cols).T


def four_vertical(r):
    """BIP-style: four vertical cells at the corners. Only Fz, Mx, My."""
    pts = [polar(r, a) for a in (45, 135, 225, 315)]
    return np.array([line(p, (0, 0, 1)) for p in pts]).T


def four_plus_four(r):
    """Four vertical plus four tangential cells (redundant, eight channels)."""
    cols = [line(polar(r, a), (0, 0, 1)) for a in (45, 135, 225, 315)]
    cols += [line(polar(r, a), tangent(a)) for a in (0, 90, 180, 270)]
    return np.array(cols).T


def metrics(A):
    rank = int(np.linalg.matrix_rank(A, tol=1e-9))
    scale = np.diag([1, 1, 1, 1 / L_REF, 1 / L_REF, 1 / L_REF])   # rows: F, then M / L_REF
    cond = float(np.linalg.cond(scale @ A)) if rank == 6 else float("inf")
    pinv = np.linalg.pinv(A)
    # Worst cell force over the corners of the peak-wrench box (an upper bound).
    worst = float(np.max(np.abs(pinv) @ W_PEAK))
    frac = worst / RATED_N
    # Noise: cell noise (uV p-p) -> force -> wrench through A.
    cell_noise_n = NOISE_UV * 1e-3 / FS_MV * RATED_N
    wrench_res = np.sqrt((A ** 2).sum(axis=1)) * cell_noise_n            # per-row root-sum-square
    return dict(rank=rank, cond=cond, worst_n=worst, frac=frac,
                signal_mv=frac * FS_MV, cell_noise_n=cell_noise_n, wrench_res=wrench_res)


SENSOR_BELOW_ANKLE_M = H_SENSOR / 2.0 + 0.010   # sensor centre below the ankle (sweep_foot.py's convention)


def score_series(A, W):
    """Worst-case and 99th-percentile cell force, as a fraction of rating, over the
    recorded wrenches W (n x 6, at the ankle), shifted to the sensor centre."""
    pinv = np.linalg.pinv(A)
    out = []
    for sign in (+1.0, -1.0):
        Ws = W.copy()
        h = SENSOR_BELOW_ANKLE_M
        Ws[:, 3] += sign * (-h * W[:, 1])      # Mx += -h*Fy
        Ws[:, 4] += sign * (h * W[:, 0])       # My += +h*Fx
        f = np.abs(Ws @ pinv.T)                # n x cells
        out.append((f.max() / RATED_N, np.percentile(f.max(axis=1), 99) / RATED_N))
    return max(o[0] for o in out), max(o[1] for o in out)


def best_stewart():
    """Coarse sweep over the Stewart geometry, scored by condition number."""
    best = None
    for r, dtop, dbase, h in itertools.product(
            (0.030, 0.035, R_MAX), (8, 15, 25, 40, 55), (8, 15, 25, 40, 55, 75),
            (0.015, 0.025, 0.035)):
        A = stewart(r, r, dtop, dbase, h)
        m = metrics(A)
        if m["rank"] == 6 and (best is None or m["cond"] < best[0]["cond"]):
            best = (m, dict(r_mm=round(r * 1000, 1), top_deg=dtop, base_deg=dbase,
                            height_mm=round(h * 1000, 1)), A)
    return best


def row(name, A, extra="", W=None):
    m = metrics(A)
    if W is not None and m["rank"] == 6:
        pk, p99 = score_series(A, W)
        extra = f"SIM worst {100 * pk:.1f}% p99 {100 * p99:.1f}% of rating  " + extra
    n = A.shape[1]
    cond = f"{m['cond']:.1f}" if np.isfinite(m["cond"]) else "rank<6"
    wr = m["wrench_res"]
    print(f"{name:<26} {n:>3}  {m['rank']:>4}  {cond:>7}  {m['worst_n']:>6.0f} N "
          f"{100 * m['frac']:>5.1f}%  {m['signal_mv']:>5.2f}  "
          f"{wr[2]:>5.2f}/{wr[0]:>5.2f}  {wr[3]:>5.3f}/{wr[5]:>5.3f}  {extra}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--wrenches", help="npz from tools/log_ft_loads.py")
    args = ap.parse_args()
    W = np.load(args.wrenches)["wrenches"] if args.wrenches else None
    print(f"cell rating {RATED_N:.0f} N ({FT['load_cell']['rating']['rated_load_kg']} kg, UNVERIFIED), "
          f"full-scale {FS_MV:.2f} mV, noise {NOISE_UV} uV p-p -> "
          f"{NOISE_UV * 1e-3 / FS_MV * RATED_N:.2f} N per cell")
    print(f"hexagon {FORM['across_flats_mm']} mm across flats, cell centre radius <= {R_MAX * 1000:.1f} mm, "
          f"height {FORM['height_mm']} mm")
    print(f"assumed peak wrench: Fz {PEAK['fz_n']} N, Fx/Fy {PEAK['fx_n']} N, Mx/My {PEAK['mx_nm']} N.m, "
          f"Mz {PEAK['mz_nm']} N.m\n")
    print(f"{'layout':<26} {'n':>3}  {'rank':>4}  {'cond':>7}  {'worst cell':>13}  {'sig mV':>5}  "
          f"{'Fz/Fx res N':>11}  {'Mx/Mz res N.m':>13}")
    print("-" * 110)

    row("3 vertical + 3 tangential", three_plus_three(R_MAX), f"r={R_MAX * 1000:.0f} mm", W)
    row("  (same, r = 30 mm)", three_plus_three(0.030), "r=30 mm", W)
    row("4 vertical (BIP-style)", four_vertical(R_MAX), "Fz, Mx, My only")
    row("4 vertical + 4 tangential", four_plus_four(R_MAX), "8 channels, redundant", W)

    m, params, A = best_stewart()
    row("Stewart, best of sweep", A, str(params), W)
    # The builder's rough sketch is not parameterised here; add it once its
    # strut angles are read off the CAD.

    print("\nrank<6: cannot measure all six components. cond is on moment-scaled A.")
    print("signal is the worst cell at the peak wrench; resolution is peak to peak at the assumed noise.")
    print("Resolution needed for a ~2 mm CoP at 47 N is about 0.1 N.m on Mx/My.")


if __name__ == "__main__":
    main()

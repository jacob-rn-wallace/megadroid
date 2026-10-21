#!/usr/bin/env python3
"""
Closed-form sizing for the 'rods' layout: bolts, flexure necks, pads.

Hand-calculation screening, not FEA. It answers: for the member forces this layout sees,
what neck diameter and length survive, how long must the rod be, do the cell bolts and joint
hold, and which check governs. Every input marked ASSUMPTION is in design/ft_sensor.yaml
(`sizing_basis`, `design_basis`). Units mm, N, MPa.

THE ROD: shoulder | neck | body | neck | stud into the far cap. The necks are turned thin
sections that act as pivots, so the rod carries axial load only. Its span between the cell face
and the far cap is  shoulder + 2 x neck length + body, which sets the minimum rod length the
layout search must allow.

CHECKS (per neck of diameter d, length l):
  tension/compression   sigma = F / (pi d^2 / 4)      <= yield / SF at the sensing-range load,
                                                        <= yield at the abuse load
  buckling              P_cr = 2 (E I / l) / L_rod     >= 2 x the abuse load (rigid body between two
                                                        rotational springs)
  misalignment bending  sigma_b = E d theta / (2 l),  theta = misalignment / L_rod
  fatigue               alternating stress from the walking range <= endurance / 1.5
  stiffness             neck axial stiffness against the cell's bending stiffness
and for the bolts and pads: preload, joint separation at the fixed end (M = F x 30 mm against
preload x contact length / 6), thread strip, pad bending.

    python3 hardware/ft_sensor/sizing.py
"""

import math
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import design_synthesis as ds  # noqa: E402

ROOT = HERE.parents[1]
FT = yaml.safe_load((ROOT / "design" / "ft_sensor.yaml").read_text())
SB = FT["sizing_basis"]
MAT = SB["materials"]
ROD_MAT = MAT[SB["parts"]["rod"]]
PAD_MAT = MAT[SB["parts"]["cap_and_pads"]]
BOLT = SB["bolt"]
SF, MIS = SB["safety_factor_design"], SB["misalignment_mm"]
RANGE_F = FT["design_basis"]["range_load_factor"]      # sensing-range load, x sim peak
ABUSE_F = FT["design_basis"]["abuse_load_factor"]      # structural abuse load, x sim peak
RATED_N = ds.RATED_N
SIM = FT["sim_measured_loads"]["peak"]


def member_forces(ms, W):
    A = np.array([m["line"] for m in ms]).T
    return np.linalg.pinv(A) @ W


def size_neck(f_range, f_sim, rod_len, mat=ROD_MAT):
    """Smallest neck (d, l) that passes every check; returns dict or None.
    f_range: member force at the sensing-range load (sim peak x range factor); f_sim: at the sim peak.
    The abuse force is f_sim x ABUSE_F."""
    f_abuse = f_sim * ABUSE_F
    E, Sy, Se = mat["youngs_mpa"], mat["yield_mpa"], mat["endurance_mpa"]
    best = None
    for d in np.arange(2.0, 5.01, 0.1):
        for l in np.arange(2.0, 8.01, 0.5):
            A = math.pi * d * d / 4
            I = math.pi * d ** 4 / 64
            span = SB["shoulder_mm"] + 2 * l + SB["rod_body_mm"]
            L = max(rod_len, span)
            s_axial = f_range / A
            s_abuse = f_abuse / A
            kth = E * I / l
            pcr = 2 * kth / L
            sb = E * d * (MIS / L) / (2 * l)
            s_alt = (f_sim / A) / 2 + sb          # half the walking range plus assembly bending
            ok = (s_axial <= Sy / SF and s_abuse <= Sy and pcr >= 2 * f_abuse and
                  s_axial + sb <= Sy / SF and s_alt <= Se / 1.5)
            if ok:
                # prefer the shortest span, then the thinnest neck
                key = (span, d)
                if best is None or key < best[0]:
                    best = ((span, d), dict(d=round(d, 1), l=l, span=span, s_axial=s_axial, s_bend=sb,
                                            s_alt=s_alt, pcr=pcr, kax=E * A / l, kth=kth))
    return None if best is None else best[1]


def bolt_checks(F):
    As, proof = BOLT["stress_area_mm2"], BOLT["proof_mpa"]
    preload = BOLT["preload_fraction_of_proof"] * proof * As
    a = SB["fixed_end_contact_length_mm"]
    M = F * 2 * ds.HOLE                      # moment at the fixed-end bolt, N.mm
    sep_load = 6 * M / a                     # preload needed to keep the contact closed
    Le = SB["thread_engagement_mm"]
    tau = 0.577 * PAD_MAT["yield_mpa"]
    strip = 0.5 * math.pi * 6 * Le * tau
    return dict(preload=preload, sep_required=sep_load, strip=strip, bolt_stress=F / As + preload / As)


def pad_bending(F):
    M = F * 2 * ds.HOLE
    w, a = ds.CW, ds.PAD_LEN
    return 6 * M / (w * a * a)


def report(p, label, builder=None, circle=True):
    ds.set_envelope(88.85, ds.FORM["diameter_mm"] / 2.0 if circle else None)
    ms = (builder or ds.members)(p, "rods")
    A = np.array([m["line"] for m in ms]).T
    Fd = np.abs(np.linalg.pinv(A)) @ ds.W_DESIGN_MM
    Fs = np.abs(np.linalg.pinv(A)) @ (ds.W_DESIGN_MM / RANGE_F)
    f_design, f_sim = float(Fd.max()), float(Fs.max())
    rod_len = min(sum(2 * b.h[2] for b in m["boxes"] if b.name.startswith("rod")) for m in ms)
    print(f"\n=== {label}")
    print(f"  member force, worst of six: sensing-range load {f_design:.0f} N ({100 * f_design / RATED_N:.0f}% of the "
          f"{RATED_N / 9.81:.0f} kg rating), simulated peak {f_sim:.0f} N, abuse {f_sim * ABUSE_F:.0f} N "
          f"({100 * f_sim * ABUSE_F / RATED_N:.0f}% of rating: needs overload stops); rod span in this layout {rod_len:.1f} mm")
    for name, mat in MAT.items():
        n = size_neck(f_design, f_sim, rod_len, mat)
        if n is None:
            print(f"  [{name}] no neck up to 5 mm diameter passes")
            continue
        print(f"  [{name}] neck d {n['d']} mm x l {n['l']} mm -> rod span >= {n['span']:.1f} mm; axial {n['s_axial']:.0f} MPa, "
              f"misalignment bending {n['s_bend']:.0f} MPa, alternating {n['s_alt']:.0f} MPa, buckling {n['pcr'] / 1000:.1f} kN; "
              f"neck axial stiffness {n['kax'] / 1000:.0f} MN/m each")
    b = bolt_checks(f_design)
    print(f"  M6 {BOLT["grade"]} bolt: preload {b['preload'] / 1000:.1f} kN, stress {b['bolt_stress']:.0f} MPa (proof {BOLT['proof_mpa']}); "
          f"thread strip capacity {b['strip'] / 1000:.0f} kN against {f_design / 1000:.1f} kN")
    print(f"  fixed-end joint: preload needed to keep it closed {b['sep_required'] / 1000:.1f} kN against {b['preload'] / 1000:.1f} kN available "
          f"-> {'HOLDS' if b['sep_required'] <= b['preload'] else 'OPENS'} at the design wrench "
          f"(opens above about {b['preload'] / b['sep_required'] * f_design:.0f} N member force)")
    print(f"  pad bending {pad_bending(f_design):.0f} MPa (7075 allowable {PAD_MAT['yield_mpa'] / SF:.0f} MPa)")


STOP = FT["stop_basis"]


def member_stiffness(k_cell, neck):
    """Series stiffness of cell + two necks + rod body, N/mm. k_cell in N/mm (1 MN/m = 1000 N/mm)."""
    E = ROD_MAT["youngs_mpa"]
    k_neck = E * math.pi * neck["d"] ** 2 / 4 / neck["l"]
    k_body = E * math.pi * ds.ROD ** 2 / 4 / SB["rod_body_mm"]
    return 1.0 / (1.0 / k_cell + 2.0 / k_neck + 1.0 / k_body)


def stop_design(ms, neck, k_cell_n_per_mm):
    """Central pin-in-bore stop. Returns the cap displacement at which the worst member reaches
    the stop force, per wrench axis, so the clearances can be set. Forces/moments in N, N.mm."""
    A = np.array([m["line"] for m in ms]).T          # 6x6
    km = member_stiffness(k_cell_n_per_mm, neck)
    K = km * A @ A.T
    out = {}
    for j, name in enumerate(("Fx", "Fy", "Fz", "Mx", "My", "Mz")):
        e = np.zeros(6)
        e[j] = 1.0
        d = np.linalg.solve(K, e)                    # displacement per unit wrench
        f = km * A.T @ d                             # member forces per unit wrench
        W = STOP["stop_member_force_n"] / np.abs(f).max()
        out[name] = (W, d * W)                       # wrench at the stop, displacement at the stop
    return km, out


def pad_block_bolts(F, u_z, h_c):
    """Two M6 flat-head bolts hold a wedge block to its flat plate. F: member force; the line of
    action makes u_z with the plate normal and sits h_c above the plate. Tension per bolt."""
    Fn, Ft = F * abs(u_z), F * math.sqrt(1 - u_z ** 2)
    s = SB["fixed_end_contact_length_mm"] - 6.0                 # bolt spacing
    return Fn / 2 + Ft * h_c / s, Ft


def plate_point_stiffness(t, radius):
    """Centre point stiffness of a simply supported circular plate, N/mm."""
    E, nu = ROD_MAT["youngs_mpa"], 0.33
    D = E * t ** 3 / (12 * (1 - nu ** 2))
    return 16 * math.pi * D / radius ** 2


def report_stops_and_structure(v, label):
    ms = ds.members(v, "rods")
    A = np.array([m["line"] for m in ms]).T
    Fs = float((np.abs(np.linalg.pinv(A)) @ (ds.W_DESIGN_MM / RANGE_F)).max())
    neck = size_neck(float((np.abs(np.linalg.pinv(A)) @ ds.W_DESIGN_MM).max()), Fs,
                     min(2 * b.h[2] for m in ms for b in m["boxes"] if b.kind == "rod"))
    print(f"\n=== {label}: stops and structure")
    print(f"  neck used: d {neck['d']} mm x l {neck['l']} mm (7075)")
    print(f"  stop engages when any member reaches {STOP['stop_member_force_n']} N "
          f"({100 * STOP['stop_member_force_n'] / ds.RATED_N:.0f}% of the cell rating)")
    print("  stop clearances (a box inscribed in the safe region, scaled from the displacement the abuse load causes,")
    print("  so that no member exceeds the stop force under ANY combination); depends on the unmeasured cell stiffness:")
    W_ab = ds.W_DESIGN_MM / RANGE_F * ABUSE_F
    chosen = None
    for kc in (5.0, 10.0, 21.0):                                 # MN/m
        km = member_stiffness(kc * 1000.0, neck)
        A = np.array([m["line"] for m in ms]).T
        K = km * A @ A.T
        Kinv = np.linalg.inv(K)
        w = np.array([abs(Kinv[j, j]) * abs(W_ab[j]) for j in range(6)])      # displacement per axis at its abuse value
        sc = STOP["stop_member_force_n"] / (km * (np.abs(A).T @ w)).max()
        c = sc * w
        print(f"    k_cell {kc:>4.0f} MN/m (member {km / 1000:.1f}): x/y {c[0] * 1000:.0f}/{c[1] * 1000:.0f} um, z {c[2] * 1000:.0f} um, "
              f"tilt {c[3] * 1e3:.1f}/{c[4] * 1e3:.1f} mrad, twist {c[5] * 1e3:.1f} mrad "
              f"(scale {sc:.2f} of the abuse displacement)")
        if kc == 10.0:
            chosen = c
    land = 35.0
    c_r = min(chosen[0], chosen[1], min(chosen[3], chosen[4]) * land / 2)
    print(f"  suggested stop (k_cell = 10 MN/m; re-set from Test H2/H3): pin lands {land:.0f} mm apart (the pin spans the cap gap), "
          f"radial clearance {c_r * 1000:.0f} um (this also gives a tilt stop of {2 * c_r / land * 1e3:.1f} mrad), "
          f"axial clearance {chosen[2] * 1000:.0f} um, key slot {chosen[5] * 10 * 1000:.0f} um at a 10 mm key radius")
    # abuse loads on the stop (5x simulated peak, shear and moment)
    Fabuse, Mabuse = 5 * max(SIM["fx_n"], SIM["fy_n"]), 5 * max(SIM["mx_nm"], SIM["my_nm"]) * 1000
    land = STOP["pin_diameter_mm"]
    p_force = Mabuse / 20.0                                      # land spacing 20 mm
    print(f"  pin at 5x simulated peak: shear {Fabuse:.0f} N, tilt couple {p_force:.0f} N on a land "
          f"{land} x 6 mm -> bearing {p_force / (land * 6):.0f} MPa (steel)")
    # wedge blocks
    pad = next(b for b in ms[0]["boxes"] if b.name.startswith("pad1"))
    uz = float(ms[0]["u"][2])
    hc = float(ms[0]["h1"][2])
    Fab = Fs * ABUSE_F
    Tb, Ft = pad_block_bolts(Fab, uz, hc)
    pre = BOLT["preload_fraction_of_proof"] * BOLT["proof_mpa"] * BOLT["stress_area_mm2"]
    print(f"  wedge block: {pad.h[0] * 2:.1f} x {pad.h[1] * 2:.1f} mm footprint, {pad.h[2] * 2:.1f} mm long along the bolt axis; "
          f"two M6 flat-head bolts through the 6 mm plate, tension {Tb / 1000:.1f} kN each at the abuse load "
          f"against {pre / 1000:.1f} kN preload; shear {Ft:.0f} N against friction capacity {2 * pre * 0.15 / 1000:.1f} kN")
    kp = plate_point_stiffness((ds.HEIGHT - ds.GAP) / 2, 44.0)
    print(f"  6 mm plate local stiffness {kp / 1000:.0f} MN/m (simply supported, 44 mm radius); against the cell's 5-21 MN/m, "
          f"the plate is only {kp / 1000 / 21:.1f}-{kp / 1000 / 5:.1f}x stiffer than the cell")
    sg = 6 * Mabuse / (60 * 6.0 ** 2)
    print(f"  plate global bending at 5x simulated peak moment: {sg:.0f} MPa (7075 allowable {PAD_MAT['yield_mpa'] / SF:.0f} MPa)")


def pad_bolts(F):
    """Two M6 bolts fixing a pad to its cap, spaced along the cell's length: the moment F x 30 mm
    reacts as a couple, so each bolt sees F x 30 / spacing on top of F / 2."""
    spacing = SB["fixed_end_contact_length_mm"]
    per_bolt = F / 2 + F * 2 * ds.HOLE / spacing
    return per_bolt, BOLT["preload_fraction_of_proof"] * BOLT["proof_mpa"] * BOLT["stress_area_mm2"]


if __name__ == "__main__":
    # the layout the CAD model is built from (design/ft_sensor.yaml, build_candidate)
    bc = FT["build_candidate"]
    ds.configure(bc["cap_thickness_mm"], bc["min_free_rod_mm"], float(bc["rated_kg"]))
    RATED_N = ds.RATED_N
    ds.REPS = 1
    report(bc["layout"], f"build candidate: {bc['rated_kg']} kg cell, {bc['cap_thickness_mm']} mm caps, free rod >= {bc['min_free_rod_mm']} mm",
           builder=ds.members_inv, circle=False)
    ms = ds.members_inv(bc["layout"], "rods")
    A = np.array([m["line"] for m in ms]).T
    Fs = float((np.abs(np.linalg.pinv(A)) @ (ds.W_DESIGN_MM / RANGE_F)).max())
    print(f"  neck chosen for the CAD model: d {bc['rod']['neck_d_mm']} x l {bc['rod']['neck_l_mm']} mm "
          f"({bc['rod']['material']}); abuse member force {Fs * ABUSE_F:.0f} N")

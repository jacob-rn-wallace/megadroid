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


def report(p, label):
    ds.set_envelope(88.85, ds.FORM["diameter_mm"] / 2.0)
    ms = ds.members(p, "rods")
    A = np.array([m["line"] for m in ms]).T
    Fd = np.abs(np.linalg.pinv(A)) @ ds.W_DESIGN_MM
    Fs = np.abs(np.linalg.pinv(A)) @ (ds.W_DESIGN_MM / RANGE_F)
    f_design, f_sim = float(Fd.max()), float(Fs.max())
    rod_len = min(2 * b.h[2] for m in ms for b in m["boxes"] if b.kind == "rod")
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
    print(f"  M6 A2-70 bolt: preload {b['preload'] / 1000:.1f} kN, stress {b['bolt_stress']:.0f} MPa (proof {BOLT['proof_mpa']}); "
          f"thread strip capacity {b['strip'] / 1000:.0f} kN against {f_design / 1000:.1f} kN")
    print(f"  fixed-end joint: preload needed to keep it closed {b['sep_required'] / 1000:.1f} kN against {b['preload'] / 1000:.1f} kN available "
          f"-> {'HOLDS' if b['sep_required'] <= b['preload'] else 'OPENS'} at the design wrench "
          f"(opens above about {b['preload'] / b['sep_required'] * f_design:.0f} N member force)")
    print(f"  pad bending {pad_bending(f_design):.0f} MPa (7075 allowable {PAD_MAT['yield_mpa'] / SF:.0f} MPa)")


def pad_bolts(F):
    """Two M6 bolts fixing a pad to its cap, spaced along the cell's length: the moment F x 30 mm
    reacts as a couple, so each bolt sees F x 30 / spacing on top of F / 2."""
    spacing = SB["fixed_end_contact_length_mm"]
    per_bolt = F / 2 + F * 2 * ds.HOLE / spacing
    return per_bolt, BOLT["preload_fraction_of_proof"] * BOLT["proof_mpa"] * BOLT["stress_area_mm2"]


if __name__ == "__main__":
    # best rods layouts from design_synthesis.py with 6 mm caps and an 18 mm rod span
    runs = {
        "75 kg cell": (75.0, dict(r=38.2, delta=29.5, c0=-3.9, alpha=54.7, psi=249.5, lenmode=0, sc=1.0, zc=19.9)),
        "150 kg cell": (150.0, dict(r=32.6, delta=36.4, c0=-11.0, alpha=54.7, psi=224.8, lenmode=0, sc=1.0, zc=20.3)),
    }
    for k, (kg, v) in runs.items():
        ds.configure(6.0, 18.0, kg)
        RATED_N = ds.RATED_N
        report(v, f"{k}, 6 mm caps, rod span >= 18 mm")
        ms = ds.members(v, "rods")
        A = np.array([m["line"] for m in ms]).T
        Fs = float((np.abs(np.linalg.pinv(A)) @ (ds.W_DESIGN_MM / RANGE_F)).max())
        pb, pre = pad_bolts(Fs * ABUSE_F)
        print(f"  pad-to-cap bolts at the abuse load: {pb / 1000:.1f} kN each against {pre / 1000:.1f} kN preload "
              f"-> {'OK' if pb <= pre else 'EXCEEDS preload (joint can open)'}")

#!/usr/bin/env python3
"""
F/T sensor design synthesis -- diagonal six-cell Stewart-type layouts, two cases.

THE LAYOUT (the builder's concept): six bending-beam cells, each with its bolt axis tilted
from vertical, in three mirrored pairs (one pair per 120 degrees) between two end caps. The
cell's sensitive axis is its bolt axis u. Each cell sits on tilted pads (wedge blocks) that
reach down to the bottom cap or up to the top cap along u.

TWO CASES, because it is not yet known whether a cell bolted at both ends still measures
(docs/P4_JOINT_TEST_RIG.md Test H2/H3):

  direct  Both ends of every cell are bolted to the caps (the builder's concept, no rods).
          The cell's stiff directions carry load in parallel with its sensing direction.
          Scored with a three-spring cell model (axial, width, bending stiffness) and the
          full 6-DOF platform stiffness, over a range of bending stiffnesses. The result is
          a 6x6 calibration matrix S (cell readings per unit wrench) whose inverse gives the
          resolution. Axial/bending stiffnesses are ESTIMATES (cell stiffness is unmeasured).

  rods    One end of each cell is bolted to a pad; the other end takes a rod along the same
          line, with a flexure pivot at each end of the rod, reaching the far cap. The cell
          is then the only compliant path in its sensing direction (an ideal strut), scored
          from the lines of action alone.

SEARCHED: radius, pair half-angle, cluster rotation, tilt from vertical, azimuth of the tilt,
cell length direction, cell centre height, which end is which. Every part is an oriented
box; all must lie inside the hexagon (wall margin) and clear each other. The cluster
symmetry is exact (3-fold + mirror), so only pairs involving a member of the first cluster
are checked.

SCORE: the worst of the margins against the targets in design/ft_sensor.yaml (cell load at
the design wrench, and resolution of Mx/My, Mz, Fz, shear); below 1.0 meets every target.

NOT DONE: stress, bolt sizes, flexure neck sizing, the cap bodies themselves (the pads
stand for the cap material below and above them), or CAD. Boxes are bounding volumes.

    python3 hardware/ft_sensor/design_synthesis.py --case rods
    python3 hardware/ft_sensor/design_synthesis.py --case direct --across-flats 94.8
"""

import argparse
import itertools
import math
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
FT = yaml.safe_load((ROOT / "design" / "ft_sensor.yaml").read_text())
FORM = yaml.safe_load((ROOT / "design" / "sensors.yaml").read_text())["end_of_limb_sensing"]["ft_sensor"]["form_factor"]
BASIS = FT["design_basis"]
CELL = FT["load_cell"]["dimensions_mm"]
CL, CW, CT = CELL["length"], CELL["width"], CELL["thickness"]
HOLE = FT["load_cell"]["mounting"]["hole_spacing_mm"] / 2.0
RATED_N = FT["load_cell"]["rating"]["rated_load_kg"] * 9.81
NOISE_N = BASIS["noise_cell_equivalent_n"]
HEIGHT = FORM["height_mm"]
GAP = HEIGHT - 2 * BASIS["cap_thickness_mm"]
MARGIN = BASIS["hex_wall_margin_mm"]
CLEAR = BASIS["min_clearance_mm"]
ROD = BASIS["rod_section_mm"]
TARGET = BASIS["targets"]
ORIGIN = np.array([0.0, 0.0, GAP / 2.0])
PAD_LEN = 12.7            # pad footprint along the cell length (around the bolt)
ROD_MIN = 12.0            # shortest usable rod: two pivots plus thread engagement
EMBED = 6.0               # how far pads/bosses may sit inside the cap material

SIM = FT["sim_measured_loads"]["peak"]
SENSOR_BELOW_ANKLE = HEIGHT / 2000.0 + 0.010
W_DESIGN = BASIS["design_load_factor"] * np.array([
    SIM["fx_n"], SIM["fy_n"], SIM["fz_n"],
    SIM["mx_nm"] + SENSOR_BELOW_ANKLE * SIM["fy_n"],
    SIM["my_nm"] + SENSOR_BELOW_ANKLE * SIM["fx_n"], SIM["mz_nm"]])
W_DESIGN_MM = W_DESIGN * np.array([1, 1, 1, 1000, 1000, 1000])     # moments to N.mm

# Cell stiffness sets (MN/m): axial, width, bending. ESTIMATES, see module docstring.
STIFFNESS = {"solid block": (263.0, 50.0, 21.0), "thinned web": (263.0, 30.0, 5.0)}

APOTHEM = 44.425
CIRCLE_R = None          # when set, the envelope is a cylinder of this radius instead of the hexagon
NORMALS = [np.array([math.cos(math.radians(30 + 60 * k)), math.sin(math.radians(30 + 60 * k))]) for k in range(6)]


def set_envelope(across_flats, circle_radius=None):
    global APOTHEM, CIRCLE_R
    APOTHEM = across_flats / 2.0
    CIRCLE_R = circle_radius


class Box:
    def __init__(self, name, centre, axes, half, kind):
        self.name, self.kind = name, kind
        self.c = np.asarray(centre, float)
        self.R = np.column_stack([np.asarray(a, float) / np.linalg.norm(a) for a in axes])
        self.h = np.asarray(half, float)

    def corners(self):
        return np.array([self.c + self.R @ (self.h * np.array(s)) for s in itertools.product((-1, 1), repeat=3)])


def separated(a, b, margin):
    ha, hb = a.h + margin / 2, b.h + margin / 2
    t = b.c - a.c
    axes = [a.R[:, i] for i in range(3)] + [b.R[:, i] for i in range(3)]
    axes += [np.cross(a.R[:, i], b.R[:, j]) for i in range(3) for j in range(3)]
    for ax in axes:
        n = np.linalg.norm(ax)
        if n < 1e-9:
            continue
        ax = ax / n
        if abs(t @ ax) > sum(ha[i] * abs(a.R[:, i] @ ax) for i in range(3)) + sum(hb[i] * abs(b.R[:, i] @ ax) for i in range(3)):
            return True
    return False


def inside(box, zlo=0.0, zhi=None):
    zhi = GAP if zhi is None else zhi
    pts = box.corners()
    if pts[:, 2].min() < zlo - 1e-6 or pts[:, 2].max() > zhi + 1e-6:
        return False
    if CIRCLE_R is not None:
        return np.hypot(pts[:, 0], pts[:, 1]).max() <= CIRCLE_R - MARGIN + 1e-6
    return all((pts[:, :2] @ n).max() <= APOTHEM - MARGIN + 1e-6 for n in NORMALS)


def unit2(deg):
    a = math.radians(deg)
    return np.array([math.cos(a), math.sin(a), 0.0]), np.array([-math.sin(a), math.cos(a), 0.0])


Z = np.array([0.0, 0.0, 1.0])


def members(p, case):
    """Six members: dict(id, boxes, mating pairs, cell centre, axes, hole points, line)."""
    out = []
    for k in range(3):
        for m in (+1, -1):
            idx = 2 * k + (0 if m > 0 else 1)
            rho, tau = unit2(p["c0"] + 120 * k + m * p["delta"])
            xy = p["r"] * rho
            u = math.sin(math.radians(p["alpha"])) * (math.cos(math.radians(p["psi"])) * rho
                                                       + m * math.sin(math.radians(p["psi"])) * tau) \
                + math.cos(math.radians(p["alpha"])) * Z
            u = u / np.linalg.norm(u)
            if p["lenmode"] == 0:                       # length along the slope
                cvec = Z - (Z @ u) * u
            else:                                       # length horizontal
                cvec = np.cross(Z, u)
            cvec = cvec / np.linalg.norm(cvec) * p["sc"] * (m if p["lenmode"] == 1 else 1)
            w = np.cross(u, cvec)
            centre = np.array([xy[0], xy[1], p["zc"]])
            axes = [cvec, w, u]
            cell = Box(f"cell{idx}", centre, axes, [CL / 2, CW / 2, CT / 2], "cell")
            h1, h2 = centre - HOLE * cvec, centre + HOLE * cvec        # e1 -> bottom cap, e2 -> top cap / rod
            boxes, mating = [cell], set()

            def pad(name, hole, d, z_target):
                face = hole + d * CT / 2
                dz = d[2]
                if abs(dz) < 0.2:
                    return None
                L = (z_target - face[2]) / dz
                if L < 1.0:
                    return None
                b = Box(name, face + d * L / 2, [cvec, w, u], [PAD_LEN / 2, CW / 2, L / 2], "pad")
                return b
            pad1 = pad(f"pad1_{idx}", h1, -u, 0.0)
            if case == "direct":
                pad2 = pad(f"pad2_{idx}", h2, u, GAP)
                if pad1 is None or pad2 is None:
                    return None
                boxes += [pad1, pad2]
                mating |= {(cell.name, pad1.name), (cell.name, pad2.name)}
                line = None
            else:
                if pad1 is None:
                    return None
                face2 = h2 + u * CT / 2
                if u[2] < 0.2:
                    return None
                Lr = (GAP - face2[2]) / u[2]
                if Lr < ROD_MIN:
                    return None
                end = face2 + u * Lr
                rod = Box(f"rod{idx}", face2 + u * Lr / 2, [cvec, w, u], [ROD / 2, ROD / 2, Lr / 2], "rod")
                boss = Box(f"boss{idx}", end - u * 4.0, [cvec, w, u], [6.0, 6.0, 4.0], "pad")
                boxes += [pad1, rod, boss]
                mating |= {(cell.name, pad1.name), (cell.name, rod.name), (rod.name, boss.name)}
                line = np.concatenate([u, np.cross((h2 - ORIGIN), u)])
            out.append(dict(id=idx, boxes=boxes, mating=mating, centre=centre, t=u, l=cvec, w=w,
                            h1=h1, h2=h2, line=line, u=u))
    return out


def feasible(ms):
    for mem in ms[:2]:                                        # symmetry: cluster 0 stands for all
        for b in mem["boxes"]:
            if b.kind in ("pad", "rod"):
                if not inside(b, -EMBED, GAP + EMBED):
                    return False
            elif not inside(b):
                return False
    mating = set().union(*[m["mating"] for m in ms])
    allboxes = [b for m in ms for b in m["boxes"]]
    mine = [b for m in ms[:2] for b in m["boxes"]]
    for a in mine:
        for b in allboxes:
            if a is b or (a.name, b.name) in mating or (b.name, a.name) in mating:
                continue
            if a.name.endswith(b.name[-1]) and a.name[-1] == b.name[-1]:
                continue                                       # same member: mating or intentionally close
            if not separated(a, b, CLEAR):
                return False
    return True


def overlap_depth(a, b, margin):
    """0 if separated; else the smallest penetration over the separating axes (mm)."""
    ha, hb = a.h + margin / 2, b.h + margin / 2
    t = b.c - a.c
    axes = [a.R[:, i] for i in range(3)] + [b.R[:, i] for i in range(3)]
    axes += [np.cross(a.R[:, i], b.R[:, j]) for i in range(3) for j in range(3)]
    best = float("inf")
    for ax in axes:
        n = np.linalg.norm(ax)
        if n < 1e-9:
            continue
        ax = ax / n
        pen = sum(ha[i] * abs(a.R[:, i] @ ax) for i in range(3)) + sum(hb[i] * abs(b.R[:, i] @ ax) for i in range(3)) - abs(t @ ax)
        if pen <= 0:
            return 0.0
        best = min(best, pen)
    return best


def excess(box, zlo, zhi):
    """How far outside the envelope (mm); 0 if inside."""
    pts = box.corners()
    e = max(0.0, zlo - pts[:, 2].min(), pts[:, 2].max() - zhi)
    if CIRCLE_R is not None:
        return e + max(0.0, np.hypot(pts[:, 0], pts[:, 1]).max() - (CIRCLE_R - MARGIN))
    return e + max(0.0, max((pts[:, :2] @ n).max() for n in NORMALS) - (APOTHEM - MARGIN))


def violation(ms):
    v = 0.0
    for mem in ms[:2]:
        for b in mem["boxes"]:
            lo, hi = (-EMBED, GAP + EMBED) if b.kind in ("pad", "rod") else (0.0, GAP)
            v += excess(b, lo, hi)
    mating = set().union(*[m["mating"] for m in ms])
    allboxes = [b for m in ms for b in m["boxes"]]
    for a in [b for m in ms[:2] for b in m["boxes"]]:
        for b in allboxes:
            if a is b or (a.name, b.name) in mating or (b.name, a.name) in mating or a.name[-1] == b.name[-1]:
                continue
            v += overlap_depth(a, b, CLEAR)
    return v


def skew(c):
    return np.array([[0, -c[2], c[1]], [c[2], 0, -c[0]], [-c[1], c[0], 0]])


def score_rods(ms):
    A = np.array([m["line"] for m in ms]).T                       # N, N.mm
    if np.linalg.matrix_rank(A, tol=1e-9) < 6:
        return None
    pinv = np.linalg.pinv(A)
    frac = float((np.abs(pinv) @ W_DESIGN_MM).max() / RATED_N)
    res = np.sqrt((A ** 2).sum(axis=1)) * NOISE_N
    res[3:] /= 1000.0
    return finish(frac, res, {"cond": float(np.linalg.cond(np.diag([1, 1, 1, .02, .02, .02]) @ A))})


def score_direct(ms, stiff):
    ka, kw, kt = stiff
    K = np.zeros((6, 6)); T = []
    for m in ms:
        c = m["centre"] - ORIGIN
        G = np.hstack([np.eye(3), -skew(c)])
        k = ka * np.outer(m["l"], m["l"]) + kw * np.outer(m["w"], m["w"]) + kt * np.outer(m["t"], m["t"])
        K += G.T @ k @ G
        T.append(G.T @ m["t"])
    T = np.array(T)
    try:
        S = kt * T @ np.linalg.inv(K)                          # readings (N-equivalent) per unit wrench
        Sinv = np.linalg.inv(S)
    except np.linalg.LinAlgError:
        return None
    frac = float(np.abs(S @ W_DESIGN_MM).max() / RATED_N)
    res = np.sqrt((Sinv ** 2).sum(axis=1)) * NOISE_N
    res[3:] /= 1000.0
    d = np.linalg.inv(K) @ W_DESIGN_MM
    ax = max(abs(ka * float(m["l"] @ (d[:3] + np.cross(d[3:], m["centre"] - ORIGIN)))) for m in ms)
    # f: share of a pure Fz that shows up in the readings, against an ideal strut set
    Aid = np.array([np.concatenate([m["t"], np.cross(m["centre"] - ORIGIN, m["t"])]) for m in ms]).T
    return finish(frac, res, {"axial_n": ax, "f_signal": float(np.abs(S[:, 2]).sum() / np.abs(np.linalg.inv(Aid)[:, 2]).sum())})


def finish(frac, res, extra):
    m = {"load": frac / TARGET["worst_cell_fraction_of_rating"],
         "mxy": max(res[3], res[4]) / TARGET["resolution_mx_my_nm"],
         "mz": res[5] / TARGET["resolution_mz_nm"],
         "fz": res[2] / TARGET["resolution_fz_n"],
         "shear": max(res[0], res[1]) / TARGET["resolution_shear_n"]}
    return dict(score=max(m.values()), margins=m, frac=frac, res=res, **extra)


def sample(rng):
    return dict(r=rng.uniform(22, 43), delta=rng.uniform(8, 52), c0=rng.uniform(0, 120),
                alpha=rng.uniform(25, 70), psi=rng.uniform(-180, 180), lenmode=int(rng.integers(2)),
                sc=float(rng.choice([-1, 1])), zc=rng.uniform(GAP * 0.2, GAP * 0.8))


def perturb(p, rng, s):
    q = dict(p)
    for k, w in (("r", 2.5), ("delta", 4), ("c0", 6), ("alpha", 4), ("psi", 15), ("zc", 2.5)):
        q[k] = p[k] + rng.normal(0, w * s)
    return q


def main():
    global MARGIN, CLEAR
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", choices=("direct", "rods"), required=True)
    ap.add_argument("--samples", type=int, default=12, help="random starts per (length mode, sign) combination")
    ap.add_argument("--refine", type=int, default=400, help="hill-climb steps per start")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--across-flats", type=float, default=FORM["across_flats_mm"])
    ap.add_argument("--margin", type=float, default=MARGIN,
                    help="parts keep this far inside the envelope (default: design_basis in ft_sensor.yaml)")
    ap.add_argument("--clearance", type=float, default=CLEAR,
                    help="minimum gap between non-mating parts (default: design_basis in ft_sensor.yaml)")
    ap.add_argument("--circle", action="store_true",
                    help="use the circumscribing cylinder (sensors.yaml diameter_mm) as the envelope, not the hexagon")
    args = ap.parse_args()
    MARGIN, CLEAR = args.margin, args.clearance
    set_envelope(args.across_flats, FORM["diameter_mm"] / 2.0 if args.circle else None)
    rng = np.random.default_rng(args.seed)
    stiff_for_search = STIFFNESS["thinned web"]

    def evaluate(p):
        ms = members(p, args.case)
        if ms is None or not feasible(ms):
            return None
        e = score_rods(ms) if args.case == "rods" else score_direct(ms, stiff_for_search)
        return None if e is None else (e["score"], p, e, ms)

    env = f"cylinder {FORM['diameter_mm']} mm" if args.circle else f"hexagon {args.across_flats} mm across flats"
    print(f"case {args.case}: {env}, gap {GAP:.2f} mm, margin {MARGIN}, clearance {CLEAR}")
    print("design wrench (sim peak x %.1f): " % BASIS["design_load_factor"]
          + ", ".join(f"{n} {v:.1f}" for n, v in zip(("Fx", "Fy", "Fz", "Mx", "My", "Mz"), W_DESIGN)))
    BIG = 1e6

    def cost(p):
        ms = members(p, args.case)
        if ms is None:
            return BIG, None
        v = violation(ms)
        if v > 0:
            return 1000.0 + 100.0 * v, None
        e = score_rods(ms) if args.case == "rods" else score_direct(ms, stiff_for_search)
        return (BIG if e is None else e["score"]), (e, ms)

    best = None
    nfeas = 0
    for lenmode, sc in itertools.product((0, 1), (-1.0, 1.0)):
        for _ in range(args.samples):
            p = sample(rng)
            p["lenmode"], p["sc"] = lenmode, sc
            c, info = cost(p)
            step = 1.0
            for it in range(args.refine):
                q = perturb(p, rng, step)
                cq, iq = cost(q)
                if cq < c:
                    p, c, info = q, cq, iq
                if it % 100 == 99:
                    step *= 0.7
            if info is not None:
                nfeas += 1
                if best is None or c < best[0]:
                    best = (c, p, info[0], info[1])
    print(f"{nfeas} of {4 * args.samples} hill-climbs reached a feasible placement")
    if best is None:
        print("no feasible placement found")
        return
    score, p, e, ms = best
    m = e["margins"]
    print(f"\nBEST  score {score:.2f}   load {m['load']:.2f}  Mxy {m['mxy']:.2f}  Mz {m['mz']:.2f}  Fz {m['fz']:.2f}  shear {m['shear']:.2f}")
    print("  parameters: " + ", ".join(f"{k} {v:.1f}" for k, v in p.items()))
    print(f"  resolution p-p: Fx {e['res'][0]:.2f}  Fy {e['res'][1]:.2f}  Fz {e['res'][2]:.2f} N;  "
          f"Mx {e['res'][3]:.3f}  My {e['res'][4]:.3f}  Mz {e['res'][5]:.3f} N.m")
    if args.case == "rods":
        print(f"  worst cell at the design wrench: {100 * e['frac']:.0f}% of rating; cond {e['cond']:.1f}")
        print("  rod lengths (mm): " + ", ".join(f"{(b.h[2] * 2):.1f}" for mm in ms for b in mm['boxes'] if b.kind == 'rod'))
    else:
        for name, st in STIFFNESS.items():
            r2 = score_direct(ms, st)
            print(f"  [{name}] signal fraction f {r2['f_signal']:.3f}; worst resolution margin {r2['score']:.2f} "
                  f"(Mxy {r2['res'][3]:.3f} N.m, Fz {r2['res'][2]:.2f} N); axial force in a cell {r2['axial_n']:.0f} N")
    u0 = ms[0]["u"]
    print(f"  member 0: bolt axis tilt {math.degrees(math.acos(abs(u0[2]))):.1f} deg from vertical, "
          f"hole z {ms[0]['h1'][2]:.1f} / {ms[0]['h2'][2]:.1f}; pads reach z=0 and z={GAP:.1f}")


if __name__ == "__main__":
    main()

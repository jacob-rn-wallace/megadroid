#!/usr/bin/env python3
"""
Foot width x F/T-sensor mass sensitivity sweep.

WHY THIS EXISTS
---------------
The F/T sensor concept in design/sensors.yaml is a hexagonal prism (88.85 mm
across flats, 57.544 mm tall) that must sit on the foot plate. The simulated
foot is a 150 x 80 mm box hard-coded in generate_mjcf.py -- narrower than the
sensor -- and the sensor itself is a zero-mass pass-through body, so the swing
leg carries none of its mass. Both are known placeholders. This sweep asks how
much the P3 results move if the foot is as wide as the sensor needs and the
sensor carries a plausible mass, before anyone commits hardware to either.

WHAT IT CHANGES (in memory only; the committed MJCF is never touched)
---------------------------------------------------------------------
- Foot contact box half-width (y), at fixed length. The foot body mass stays
  at the mass.yaml value; only the contact box widens.
- A point mass on each `*_ft_sensor` body at the sensor's vertical centre
  (ankle-centred in x), with a solid-cylinder inertia over the hexagon's
  footprint. Adds to total robot mass.

WHAT IT DOES NOT DO
-------------------
- Does not say what the real foot or sensor weigh. Sensor mass values are
  estimates (see docs/P4_JOINT_TEST_RIG.md): 0 is the committed model; the
  others bracket a machined-aluminium two-cap build.
- Contact solref applies per contact point, so widening the foot changes
  rocking stiffness but not vertical stiffness per corner.
- Measures the CURRENT controller tuning, which was fitted at the 80 mm,
  massless-sensor model. A failure means "this tuning does not survive that".

    python3 tools/sweep_foot.py            # ~10 min on 4 cores
    python3 tools/sweep_foot.py --quick
    python3 tools/sweep_foot.py --json out.json
"""

import argparse
import copy
import json
import multiprocessing as mp
import sys
import tempfile
import xml.etree.ElementTree as ET
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

import sweep_estimates as se  # noqa: E402

# Foot half-widths in metres: committed 80 mm, a 100 mm plate, a 115 mm plate.
# 88.85 mm across flats is the sensor's own minimum; the rest is mounting margin.
HALF_WIDTHS_M = (0.040, 0.050, 0.0575)
SENSOR_MASSES_KG = (0.0, 0.3, 0.5)

# Hexagonal prism, per the CAD: 88.85 mm across flats, 57.544 mm tall.
SENSOR_ACROSS_FLATS_M = 0.08885
SENSOR_HEIGHT_M = 0.057544
# Sensor centre, measured down from the ankle: the gimbal sits above it, so
# the centre is half the sensor height below the ankle plus a small clearance.
SENSOR_CENTRE_Z_M = -(SENSOR_HEIGHT_M / 2.0 + 0.010)


def build_model(design, half_width, sensor_mass):
    import mujoco
    gm = sys.modules["generate_mjcf"]
    d = copy.deepcopy(design)
    root = gm.create_mjcf(d["joints"], d["geometry"], d["kinematics"],
                          d["mass"], d["actuation"], d["sensors"])

    for geom in root.iter("geom"):
        if geom.get("name", "").endswith("_foot_geom"):
            sx, _, sz = geom.get("size").split()
            geom.set("size", f"{sx} {half_width:.4f} {sz}")

    if sensor_mass > 0.0:
        # Solid cylinder over the hexagon's footprint (circumradius would
        # overstate it; use the across-flats radius).
        r = SENSOR_ACROSS_FLATS_M / 2.0
        h = SENSOR_HEIGHT_M
        ixx = sensor_mass * (3 * r * r + h * h) / 12.0
        izz = 0.5 * sensor_mass * r * r
        for body in root.iter("body"):
            if body.get("name", "").endswith("_ft_sensor"):
                ET.SubElement(body, "inertial", mass=str(sensor_mass),
                              pos=f"0 0 {SENSOR_CENTRE_Z_M:.5f}",
                              diaginertia=f"{ixx:.7f} {ixx:.7f} {izz:.7f}")

    xml = gm.prettify(root)
    with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as f:
        f.write(xml)
        path = f.name
    try:
        return mujoco.MjModel.from_xml_path(path)
    finally:
        Path(path).unlink(missing_ok=True)


def _foot_gap_mm(model, half_width):
    """Edge-to-edge gap between the two feet in the nominal standing pose.
    The feet sit at the hip spacing (design/geometry.yaml twin_rail), so a wide
    enough foot makes them collide with each other: <= 0 means the sweep point
    is measuring that collision, not the foot width."""
    import mujoco
    import sim_zmp_balance as sb
    root_z, _, _ = sb.solve_nominal_geometry(model)
    data = mujoco.MjData(model)
    sb.set_initial_state(model, data, root_z)
    mujoco.mj_forward(model, data)
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{s}_foot")
           for s in ("left", "right")]
    return float((data.xpos[ids[0], 1] - data.xpos[ids[1], 1] - 2 * half_width) * 1000.0)


def evaluate(args):
    half_width, sensor_mass, n_timings = args
    gm, design = se._load()
    model = build_model(design, half_width, sensor_mass)
    out = {"foot_width_mm": round(half_width * 2000, 1), "sensor_mass_kg": sensor_mass,
           "robot_mass_kg": float(model.body_mass.sum()),
           "foot_gap_mm": _foot_gap_mm(model, half_width)}
    out.update(se.measure_model(model, n_timings))
    return out


def report(results, n_timings):
    results = sorted(results, key=lambda r: (r["sensor_mass_kg"], r["foot_width_mm"]))
    print()
    print(f"{'width':>6} {'gap':>5} {'sensor':>7} {'robot':>6} | {'stand':>6} {'lipm10':>7} "
          f"{'rec8':>7} {'rec16':>7} | {'5N':>6} {'10N':>6}")
    print(f"{'mm':>6} {'mm':>5} {'kg':>7} {'kg':>6} |")
    print("-" * 80)
    last = None
    for r in results:
        if last is not None and r["sensor_mass_kg"] != last:
            print()
        last = r["sensor_mass_kg"]
        print(f"{r['foot_width_mm']:>6.0f} {r['foot_gap_mm']:>5.0f} {r['sensor_mass_kg']:>7.2f} {r['robot_mass_kg']:>6.2f} | "
              f"{se._cell(r['stand'], 'tilt'):>6} {se._cell(r['lipm10'], 'tilt'):>7} "
              f"{se._cell(r['recede8'], 'tilt'):>7} {se._cell(r['recede16'], 'tilt'):>7} | "
              f"{se._cell(r['push5N'], 'push'):>6} {se._cell(r['push10N'], 'push'):>6}")
    print(f"\n  tilt in degrees; * = at or above {se.SURVIVE_TILT_DEG:g} (a fall). "
          f"Push cells = timings survived of {n_timings}, lateral, n={se.PUSH_WALK_STEPS}.")
    print("  80 mm / 0.00 kg is the committed model. gap <= 0: the feet touch each other, "
          "so that row\n  measures a foot-on-foot collision, not the foot width.")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--quick", action="store_true", help="4 push timings instead of 12")
    ap.add_argument("--json", help="write raw results here")
    ap.add_argument("--widths-mm", help="comma list of foot widths, e.g. 90,95")
    ap.add_argument("--masses-kg", help="comma list of sensor masses per foot, e.g. 0,0.4")
    args = ap.parse_args()
    widths = (tuple(float(x) / 2000.0 for x in args.widths_mm.split(","))
              if args.widths_mm else HALF_WIDTHS_M)
    masses = (tuple(float(x) for x in args.masses_kg.split(","))
              if args.masses_kg else SENSOR_MASSES_KG)

    n_timings = 4 if args.quick else 12
    jobs = [(w, m, n_timings) for m in masses for w in widths]
    print(f"{len(jobs)} points, {n_timings} push timings each, {args.workers} workers",
          flush=True)
    with ProcessPoolExecutor(max_workers=args.workers,
                             mp_context=mp.get_context("spawn")) as ex:
        results = list(ex.map(evaluate, jobs))
    report(results, n_timings)
    if args.json:
        Path(args.json).write_text(json.dumps(results, indent=1))
        print(f"\nraw results -> {args.json}")


if __name__ == "__main__":
    main()

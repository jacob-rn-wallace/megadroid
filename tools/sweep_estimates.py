#!/usr/bin/env python3
"""
Sensitivity sweep over the design files' ranged placeholders.

WHY THIS EXISTS
---------------
Several authoritative values are unmeasured estimates (motor stall torque,
drivetrain efficiency, sole compliance -- see PLACEHOLDER in design/*.yaml).
Waiting for the P4 test rig to replace them blocks the project, but a single
midpoint estimate is not enough here: results in this project are knife-edge
sensitive (sole timeconst alone moved 10 N push survival from 0/12 to 9/12 and
back to 3/12), so a result at one point says little about the rest of the
plausible range.

This tool records the *range* instead. Each ranged placeholder carries
`low`/`high` bounds and a basis in `estimate_ranges:` in its design file. The
tool runs the P3 battery across low / midpoint / high (plus the current
nominal) and reports which conclusions are ROBUST to the uncertainty and which
FLIP inside it. Only the latter justify physical measurement.

WHAT IT DOES NOT DO
-------------------
- It does not say where in the range the truth lies. A parameter that flips
  stays unresolved until someone measures it; the sweep says whether to bother.
- It measures robustness of the CURRENT controller tuning, which was fitted at
  the nominal values. A failure away from nominal means "this tuning does not
  survive that", not "no controller could".
- Stall torque and efficiency enter the simulation only through their product
  (peak joint torque = stall * total_reduction * efficiency), so they are swept
  jointly through that derived value. no_load_speed_rpm is not enforced in
  simulation at all (design/actuation.yaml modeling_limitations) and is
  reported, not swept.
- The committed MJCF is never modified: every model is built in memory from
  overridden design data.

Run under a Python with mujoco (see tools/CLAUDE.md for the venv):
    python3 tools/sweep_estimates.py
    python3 tools/sweep_estimates.py --quick          # fewer push timings
    python3 tools/sweep_estimates.py --json out.json
"""

import argparse
import copy
import json
import sys
import tempfile
from concurrent.futures import ProcessPoolExecutor
from itertools import product
from pathlib import Path

import numpy as np

TOOLS_DIR = Path(__file__).resolve().parent
REPO_ROOT = TOOLS_DIR.parent
DESIGN_DIR = REPO_ROOT / "design"
sys.path.insert(0, str(TOOLS_DIR))

# A walk or push counts as survived below this peak tilt. Looser than the
# gate's 20 degrees so a large-but-recovered excursion is not scored as a fall;
# genuine falls in this project land at 60-95 degrees.
SURVIVE_TILT_DEG = 30.0
PUSH_FORCES_N = (5.0, 10.0)
# Mid-walk window (seconds). Starts after the start-up transient: a push at ~1.6 s
# lands in the first steps and falls even at the committed baseline, which would
# make every row look bad for a reason unrelated to the swept parameters.
PUSH_WINDOW_S = (2.0, 3.1)
WALK_STEPS = (8, 16)
PUSH_WALK_STEPS = 12


def _load():
    import generate_mjcf as gm
    return gm, {k: gm.load_yaml(DESIGN_DIR / f"{k}.yaml")
                for k in ("joints", "geometry", "kinematics", "mass", "actuation", "sensors")}


def _mid(r):
    return 0.5 * (r["low"] + r["high"])


def build_points(design):
    """Return (torque_points, sole_points): lists of (label, value)."""
    act, geo = design["actuation"], design["geometry"]
    ar, gr = act["estimate_ranges"], geo["estimate_ranges"]
    red = act["drivetrain"]["total_reduction"]

    st, ef = ar["motor.stall_torque_nm"], ar["drivetrain.efficiency"]
    torque = [
        ("current", act["output"]["peak_joint_torque_nm"]),
        ("low", st["low"] * red * ef["low"]),
        ("mid", _mid(st) * red * _mid(ef)),
        ("high", st["high"] * red * ef["high"]),
    ]
    sr = gr["foot_stack.sole.contact_timeconst_s"]
    sole = [
        ("current", geo["foot_stack"]["sole"]["contact_timeconst_s"]),
        ("low", sr["low"]),
        ("mid", _mid(sr)),
        ("high", sr["high"]),
    ]
    return torque, sole


def _model_for(design, peak_torque, timeconst):
    import mujoco
    gm = sys.modules["generate_mjcf"]
    d = copy.deepcopy(design)
    d["actuation"]["output"]["peak_joint_torque_nm"] = float(peak_torque)
    d["geometry"]["foot_stack"]["sole"]["contact_timeconst_s"] = float(timeconst)
    root = gm.create_mjcf(d["joints"], d["geometry"], d["kinematics"],
                          d["mass"], d["actuation"], d["sensors"])
    xml = gm.prettify(root)
    # The model references no external assets, so a temp file is sufficient.
    with tempfile.NamedTemporaryFile("w", suffix=".xml", delete=False) as f:
        f.write(xml)
        path = f.name
    try:
        return mujoco.MjModel.from_xml_path(path)
    finally:
        Path(path).unlink(missing_ok=True)


def _safe(fn, default=None):
    try:
        return fn()
    except Exception as exc:  # extreme parameters can make IK/sim raise
        return {"error": f"{type(exc).__name__}: {exc}"} if default is None else default


def evaluate(args):
    """Worker: build one model and measure everything. Top-level so it pickles."""
    tlabel, torque, slabel, tc, n_timings = args
    import mujoco  # noqa: F401  (imported in the worker, not the parent)
    import sim_walk_lipm as lw
    import sim_walk_recede as sr
    import sim_zmp_balance as sb

    gm, design = _load()
    model = _model_for(design, torque, tc)
    out = {"torque_label": tlabel, "peak_torque_nm": torque,
           "sole_label": slabel, "timeconst_s": tc}

    def standing():
        root_z, tx, ty = sb.solve_nominal_geometry(model)
        r = sb.run(model, sb.DEFAULT_KP_RAD_PER_M, sb.DEFAULT_ZMP_FILTER_ALPHA,
                   tx, ty, root_z, 3.0)
        return {"tilt": float(max(r["tilt"])), "fell": bool(r["fell"])}
    out["stand"] = _safe(standing)

    def lipm10():
        r = lw.run_walk(model, n_steps=10, verbose=False)
        return {"tilt": float(r["max_tilt_deg"]), "diverged": bool(r["diverged"])}
    out["lipm10"] = _safe(lipm10)

    for n in WALK_STEPS:
        def walk(n=n):
            r = sr.run_walk_recede(model, n_steps=n, verbose=False)
            return {"tilt": float(r["max_tilt_deg"]), "diverged": bool(r["diverged"])}
        out[f"recede{n}"] = _safe(walk)

    timings = np.linspace(PUSH_WINDOW_S[0], PUSH_WINDOW_S[1], n_timings)
    for f in PUSH_FORCES_N:
        survived = 0
        errors = 0
        for t in timings:
            try:
                r = sr.run_walk_recede(model, n_steps=PUSH_WALK_STEPS, verbose=False,
                                       push_at=float(t), push_force=(0.0, f),
                                       push_duration=0.1)
                survived += int(not r["diverged"] and r["max_tilt_deg"] < SURVIVE_TILT_DEG)
            except Exception:
                errors += 1
        out[f"push{int(f)}N"] = {"survived": survived, "of": n_timings, "errors": errors}
    return out


def _cell(v, kind):
    if v is None or "error" in v:
        return "ERR"
    if kind == "tilt":
        return f"{v['tilt']:6.2f}" + ("*" if v["tilt"] >= SURVIVE_TILT_DEG else " ")
    if kind == "push":
        return f"{v['survived']:>2}/{v['of']:<2}"
    return "?"


def report(results, n_timings):
    rows = sorted(results, key=lambda r: (r["torque_label"], r["sole_label"]))
    order = {"current": 0, "low": 1, "mid": 2, "high": 3}
    rows.sort(key=lambda r: (order[r["torque_label"]], order[r["sole_label"]]))

    print()
    print(f"{'torque':>8} {'N.m':>6} | {'sole':>7} {'tc':>6} | {'stand':>7} {'lipm10':>7} "
          f"{'rec8':>7} {'rec16':>7} | {'5N':>6} {'10N':>6}")
    print("-" * 82)
    for r in rows:
        print(f"{r['torque_label']:>8} {r['peak_torque_nm']:>6.1f} | "
              f"{r['sole_label']:>7} {r['timeconst_s']:>6.3f} | "
              f"{_cell(r['stand'], 'tilt'):>7} {_cell(r['lipm10'], 'tilt'):>7} "
              f"{_cell(r['recede8'], 'tilt'):>7} {_cell(r['recede16'], 'tilt'):>7} | "
              f"{_cell(r['push5N'], 'push'):>6} {_cell(r['push10N'], 'push'):>6}")
    print(f"\n  tilt in degrees; * = at or above {SURVIVE_TILT_DEG:.0f} (a fall). "
          f"Push cells = timings survived of {n_timings}, lateral, n={PUSH_WALK_STEPS}.")

    # Verdict per metric: does the conclusion hold across the whole grid?
    print("\nRobustness across the whole grid (all torque x sole points):")
    metrics = [("stand", "tilt"), ("lipm10", "tilt"), ("recede8", "tilt"),
               ("recede16", "tilt"), ("push5N", "push"), ("push10N", "push")]
    for key, kind in metrics:
        vals = [r[key] for r in rows if r[key] is not None and "error" not in r[key]]
        if kind == "tilt":
            ok = sum(v["tilt"] < SURVIVE_TILT_DEG for v in vals)
            verdict = "ROBUST" if ok == len(vals) else ("FAILS EVERYWHERE" if ok == 0 else "FLIPS")
            print(f"  {key:>9}: {ok:>2}/{len(vals)} points stay up   -> {verdict}")
        else:
            lo = min(v["survived"] for v in vals)
            hi = max(v["survived"] for v in vals)
            half = n_timings / 2
            verdict = ("ROBUST (high)" if lo >= 0.75 * n_timings else
                       "FAILS EVERYWHERE" if hi <= 0.25 * n_timings else "FLIPS")
            print(f"  {key:>9}: survival ranges {lo}-{hi} of {n_timings}   -> {verdict}")

    # Marginals: effect of each axis alone, other axis at its current value.
    def pick(tl, sl):
        return next((r for r in rows if r["torque_label"] == tl and r["sole_label"] == sl), None)
    print("\nOne axis at a time (other axis held at the current nominal):")
    print("  torque axis (sole = current):")
    for tl in ("current", "low", "mid", "high"):
        r = pick(tl, "current")
        print(f"    {tl:>8} {r['peak_torque_nm']:>6.1f} N.m   rec16 {_cell(r['recede16'],'tilt')}  "
              f"push5N {_cell(r['push5N'],'push')}  push10N {_cell(r['push10N'],'push')}")
    print("  sole axis (torque = current):")
    for sl in ("current", "low", "mid", "high"):
        r = pick("current", sl)
        print(f"    {sl:>8} tc={r['timeconst_s']:.3f}   rec16 {_cell(r['recede16'],'tilt')}  "
              f"push5N {_cell(r['push5N'],'push')}  push10N {_cell(r['push10N'],'push')}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--quick", action="store_true", help="6 push timings instead of 12")
    ap.add_argument("--json", help="also write raw results to this path")
    a = ap.parse_args()

    gm, design = _load()
    torque, sole = build_points(design)
    n_timings = 6 if a.quick else 12

    print("Ranged placeholders (from design/*.yaml estimate_ranges):")
    ar = design["actuation"]["estimate_ranges"]
    for k, v in ar.items():
        print(f"  {k:<34} low {v['low']:<8} high {v['high']:<8} mid {_mid(v):g}")
    for k, v in design["geometry"]["estimate_ranges"].items():
        print(f"  {k:<34} low {v['low']:<8} high {v['high']:<8} mid {_mid(v):g}")
    print("\nSwept values:")
    print("  peak joint torque (stall x reduction x efficiency):",
          ", ".join(f"{l}={v:.1f}" for l, v in torque), "N.m")
    print("  sole timeconst:", ", ".join(f"{l}={v:.3f}" for l, v in sole), "s")
    print(f"  {len(torque) * len(sole)} points, {n_timings} push timings each, "
          f"{a.workers} workers (no_load_speed_rpm not enforced in sim; not swept)\n")

    jobs = [(tl, tv, sl, sv, n_timings) for (tl, tv), (sl, sv) in product(torque, sole)]
    import multiprocessing as mp
    with ProcessPoolExecutor(max_workers=a.workers, mp_context=mp.get_context("spawn")) as ex:
        results = list(ex.map(evaluate, jobs))

    report(results, n_timings)
    if a.json:
        Path(a.json).write_text(json.dumps(results, indent=2))
        print(f"\nraw results -> {a.json}")


if __name__ == "__main__":
    main()

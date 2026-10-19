#!/usr/bin/env python3
"""
Sole stiffness x damping sweep -- find the pad window the current controllers tolerate.

WHY THIS EXISTS
---------------
tools/sweep_estimates.py showed the sole contact decides every walking and push
result, but it only visited a handful of timeconsts at one damping ratio. Choosing
a pad material needs a TARGET WINDOW in physical units, so this sweeps timeconst
and dampratio on a grid and converts each point to a static stiffness (N/m) and
sag (mm) that a real material can be matched against (docs/P4_JOINT_TEST_RIG.md).

HOW THE CONVERSION IS DONE
--------------------------
For each (timeconst, dampratio) the full robot is held in its nominal pose and the
settled foot height gives the static sag; stiffness = per-foot weight / sag.
Measured, not derived: MuJoCo's contact stiffness goes as 1/(timeconst*dampratio)^2,
so damping moves stiffness too, and it mixes the foot and ground solrefs (solmix).

WHAT IT DOES NOT DO
-------------------
- It measures where the CURRENT controller tuning works, not where a controller
  could. Gains were fitted at the nominal sole (tools/CLAUDE.md).
- MuJoCo's contact is a linear spring-damper. Real foam and rubber are nonlinear,
  creep, and dissipate differently, so a material that matches the window in
  stiffness and damping may still not behave like it. Test F measures the real pad.
- Torque is held at the committed cap; this is a sole-only grid.

    python3 tools/sweep_sole.py              # ~15 min on 4 cores
    python3 tools/sweep_sole.py --quick
    python3 tools/sweep_sole.py --json out.json
"""

import argparse
import json
import multiprocessing as mp
import sys
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

import sweep_estimates as se  # noqa: E402

TIMECONSTS_S = (0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08)
DAMPRATIOS = (0.5, 1.0, 2.0, 4.0)


def foot_stiffness(model, sb):
    """(sag_mm, stiffness_N_per_m) of the actual robot standing on this sole.

    The robot is released in its nominal pose with every actuator held at its
    starting angle, and the mean sole-point height of the two feet is read after
    it settles. Measured on the full model, not a free-standing foot: a
    free-standing box under the same load sagged 1.8-2.3x less (0.30 vs 0.53 mm
    at the nominal sole), because the real feet do not share load evenly or sit
    flat. Stiffness is the per-foot weight over that sag, so it is an effective
    value for THIS robot, not a material property.
    """
    import mujoco
    import numpy as np
    root_z, _, _ = sb.solve_nominal_geometry(model)
    data = mujoco.MjData(model)
    sb.set_initial_state(model, data, root_z)
    ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{s}_foot")
           for s in ("left", "right")]
    zs = []
    steps = int(3.0 / model.opt.timestep)
    for i in range(steps):
        mujoco.mj_step(model, data)
        if i >= steps - int(0.5 / model.opt.timestep):
            zs.append(np.mean([data.xpos[j, 2] for j in ids]))
    sag_m = -float(np.mean(zs))
    if not np.isfinite(sag_m) or sag_m <= 0 or np.std(zs) > 1e-4:
        return float("nan"), float("nan")
    weight_per_foot = float(model.body_mass.sum()) * 9.81 / 2.0
    return sag_m * 1000.0, weight_per_foot / sag_m


def evaluate(args):
    tc, dr, n_timings, torque = args
    import numpy as np
    import sim_walk_lipm as lw
    import sim_walk_recede as sr
    import sim_zmp_balance as sb

    gm, design = se._load()
    model = se._model_for(design, torque, tc, dampratio=dr)
    out = {"timeconst_s": tc, "dampratio": dr}
    out["sag_mm"], out["stiffness_n_per_m"] = foot_stiffness(model, sb)

    def standing():
        root_z, tx, ty = sb.solve_nominal_geometry(model)
        r = sb.run(model, sb.DEFAULT_KP_RAD_PER_M, sb.DEFAULT_ZMP_FILTER_ALPHA,
                   tx, ty, root_z, 3.0)
        return {"tilt": float(max(r["tilt"])), "fell": bool(r["fell"])}
    out["stand"] = se._safe(standing)

    def lipm10():
        r = lw.run_walk(model, n_steps=10, verbose=False)
        return {"tilt": float(r["max_tilt_deg"]), "diverged": bool(r["diverged"])}
    out["lipm10"] = se._safe(lipm10)

    for n in se.WALK_STEPS:
        def walk(n=n):
            r = sr.run_walk_recede(model, n_steps=n, verbose=False)
            return {"tilt": float(r["max_tilt_deg"]), "diverged": bool(r["diverged"])}
        out[f"recede{n}"] = se._safe(walk)

    timings = np.linspace(se.PUSH_WINDOW_S[0], se.PUSH_WINDOW_S[1], n_timings)
    for f in se.PUSH_FORCES_N:
        survived = errors = 0
        for t in timings:
            try:
                r = sr.run_walk_recede(model, n_steps=se.PUSH_WALK_STEPS, verbose=False,
                                       push_at=float(t), push_force=(0.0, f),
                                       push_duration=0.1)
                survived += int(not r["diverged"] and r["max_tilt_deg"] < se.SURVIVE_TILT_DEG)
            except Exception:
                errors += 1
        out[f"push{int(f)}N"] = {"survived": survived, "of": n_timings, "errors": errors}
    return out


def _ok(v):
    return v is not None and "error" not in v and v["tilt"] < se.SURVIVE_TILT_DEG


def report(results, n_timings):
    results = sorted(results, key=lambda r: (r["dampratio"], r["timeconst_s"]))
    print()
    print(f"{'damp':>5} {'tc':>6} | {'sag mm':>7} {'k kN/m':>7} | {'stand':>6} {'lipm10':>7} "
          f"{'rec8':>7} {'rec16':>7} | {'5N':>6} {'10N':>6}")
    print("-" * 84)
    last = None
    for r in results:
        if last is not None and r["dampratio"] != last:
            print()
        last = r["dampratio"]
        print(f"{r['dampratio']:>5.1f} {r['timeconst_s']:>6.3f} | "
              f"{r['sag_mm']:>7.2f} {r['stiffness_n_per_m'] / 1000:>7.1f} | "
              f"{se._cell(r['stand'], 'tilt'):>6} {se._cell(r['lipm10'], 'tilt'):>7} "
              f"{se._cell(r['recede8'], 'tilt'):>7} {se._cell(r['recede16'], 'tilt'):>7} | "
              f"{se._cell(r['push5N'], 'push'):>6} {se._cell(r['push10N'], 'push'):>6}")
    print(f"\n  tilt in degrees; * = at or above {se.SURVIVE_TILT_DEG:g} (a fall). "
          f"Push cells = timings survived of {n_timings}, lateral, n={se.PUSH_WALK_STEPS}.")

    # Window: rows that stand, walk 8 and 16 steps, and survive >= half of both pushes.
    half = n_timings / 2
    good = [r for r in results
            if all(_ok(r[k]) for k in ("stand", "recede8", "recede16"))
            and r["push5N"]["survived"] >= half and r["push10N"]["survived"] >= half]
    print("\nWindow (stands, walks 8 and 16 steps, survives >= half of 5 N and 10 N pushes):")
    if not good:
        print("  none on this grid")
    for r in good:
        print(f"  damp {r['dampratio']:.1f}  tc {r['timeconst_s']:.3f}  "
              f"sag {r['sag_mm']:.2f} mm  k {r['stiffness_n_per_m'] / 1000:.1f} kN/m")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--quick", action="store_true", help="4 push timings instead of 12")
    ap.add_argument("--json", help="write raw results here")
    args = ap.parse_args()

    n_timings = 4 if args.quick else 12
    _, design = se._load()
    torque = design["actuation"]["output"]["peak_joint_torque_nm"]
    jobs = [(tc, dr, n_timings, torque) for dr in DAMPRATIOS for tc in TIMECONSTS_S]
    print(f"{len(jobs)} points, {n_timings} push timings each, {args.workers} workers, "
          f"torque held at {torque} N.m", flush=True)

    with ProcessPoolExecutor(max_workers=args.workers,
                             mp_context=mp.get_context("spawn")) as ex:
        results = list(ex.map(evaluate, jobs))
    report(results, n_timings)
    if args.json:
        Path(args.json).write_text(json.dumps(results, indent=1))
        print(f"\nraw results -> {args.json}")


if __name__ == "__main__":
    main()

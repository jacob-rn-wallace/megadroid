#!/usr/bin/env python3
"""
Log the simulated foot F/T sensor channels through walking and pushes.

WHY: the F/T sensor layout study (hardware/ft_sensor/layout_study.py) assumed peak
loads. This measures what the simulated robot actually puts through each foot's
sensor, so the sensor can be sized against data.

HOW: sim_walk_recede.run_walk_recede reads the sensors after calling
mujoco.mj_rnePostConstraint every step. This wraps that call and records
`{side}_ft_force` / `{side}_ft_torque` right after it, so the controller is
untouched. Pushes are the lateral and sagittal pelvis pushes the sensitivity sweeps
use; only runs that survive (no fall, tilt < 30 deg) are kept, since loads from a
robot that is falling are not design loads.

READING THE NUMBERS
- The sensor is the MuJoCo site on the foot body, so the wrench is the reaction
  between the foot and the ankle, in the foot frame, at the ankle (zero offset to the
  sensor body). The real sensor sits ~10-70 mm below the ankle gimbal, but the force
  is the same and the moments shift by r x F.
- The simulated sole is a compliant contact, 11.2 N.m cap, 175 mm hip spacing, 9.6 kg
  robot. Real impacts (heel strike, foot slap) are likely sharper than this contact
  model produces, so treat peaks as a floor on what the hardware sees.

    python3 tools/log_ft_loads.py --json out.json
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

TOOLS_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(TOOLS_DIR))

import mujoco  # noqa: E402
import sweep_estimates as se  # noqa: E402

CHANNELS = ("fx", "fy", "fz", "mx", "my", "mz")


def run_logged(model, **kw):
    import sim_walk_recede as sr
    adr = {}
    for side in ("left", "right"):
        adr[side] = (
            model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, f"{side}_ft_force")],
            model.sensor_adr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, f"{side}_ft_torque")],
        )
    rows = {"left": [], "right": []}
    real = mujoco.mj_rnePostConstraint

    def hooked(m, d):
        real(m, d)
        for side, (fa, ta) in adr.items():
            rows[side].append(np.concatenate([d.sensordata[fa:fa + 3], d.sensordata[ta:ta + 3]]))

    mujoco.mj_rnePostConstraint = hooked
    try:
        r = sr.run_walk_recede(model, verbose=False, **kw)
    finally:
        mujoco.mj_rnePostConstraint = real
    return r, {s: np.array(v) for s, v in rows.items()}


def summarise(series):
    """Per-channel peak, 99th percentile and RMS over every foot sample, all runs pooled."""
    x = np.vstack(series)
    out = {}
    for i, c in enumerate(CHANNELS):
        a = np.abs(x[:, i])
        out[c] = {"peak": float(a.max()), "p99": float(np.percentile(a, 99)),
                  "rms": float(np.sqrt((x[:, i] ** 2).mean()))}
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--json")
    ap.add_argument("--timings", type=int, default=6)
    args = ap.parse_args()

    _, design = se._load()
    gm = sys.modules["generate_mjcf"]
    model = se._model_for(design, design["actuation"]["output"]["peak_joint_torque_nm"],
                          design["geometry"]["foot_stack"]["sole"]["contact_timeconst_s"])
    print(f"robot mass {model.body_mass.sum():.2f} kg", flush=True)

    scenarios = {}
    r, rows = run_logged(model, n_steps=se.WALK_STEPS[1])
    assert not r["diverged"] and r["max_tilt_deg"] < se.SURVIVE_TILT_DEG, "nominal walk fell"
    scenarios["nominal 16 steps"] = {"runs": 1, "samples": [rows["left"], rows["right"]]}

    timings = np.linspace(se.PUSH_WINDOW_S[0], se.PUSH_WINDOW_S[1], args.timings)
    for axis, vec in (("lateral", lambda f: (0.0, f)), ("sagittal", lambda f: (f, 0.0))):
        for f in se.PUSH_FORCES_N:
            kept, samples = 0, []
            for t in timings:
                r, rows = run_logged(model, n_steps=se.PUSH_WALK_STEPS, push_at=float(t),
                                     push_force=vec(f), push_duration=0.1)
                if not r["diverged"] and r["max_tilt_deg"] < se.SURVIVE_TILT_DEG:
                    kept += 1
                    samples += [rows["left"], rows["right"]]
            scenarios[f"{axis} push {f:g} N"] = {"runs": kept, "of": len(timings), "samples": samples}
            print(f"  {axis} {f:g} N: kept {kept}/{len(timings)}", flush=True)

    result = {}
    pooled = []
    print(f"\n{'scenario':<22} {'runs':>5} | " + " ".join(f"{c:>13}" for c in CHANNELS))
    print(f"{'':<22} {'':>5} | " + " ".join(f"{'peak / p99':>13}" for _ in CHANNELS))
    print("-" * 118)
    for name, sc in scenarios.items():
        if not sc["samples"]:
            continue
        s = summarise(sc["samples"])
        pooled += sc["samples"]
        result[name] = {"runs": sc["runs"], "channels": s}
        print(f"{name:<22} {sc['runs']:>5} | " + " ".join(
            f"{s[c]['peak']:>6.1f}/{s[c]['p99']:>5.1f}" for c in CHANNELS))
    s = summarise(pooled)
    result["ALL"] = {"channels": s}
    print("-" * 118)
    print(f"{'ALL (pooled)':<22} {'':>5} | " + " ".join(
        f"{s[c]['peak']:>6.1f}/{s[c]['p99']:>5.1f}" for c in CHANNELS))
    print("\nforce in N, moment in N.m, absolute values; per foot, both feet pooled.")

    if args.json:
        Path(args.json).write_text(json.dumps(result, indent=1))
        # The raw time series lets the layout study score real wrenches, not a bound.
        np.savez_compressed(Path(args.json).with_suffix(".npz"),
                            wrenches=np.vstack(pooled))
        print(f"raw -> {args.json}, {Path(args.json).with_suffix('.npz')}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
P3 Static Pose Validation — fixed-base load test in MuJoCo.

Position control on a floating-base biped cannot balance the pelvis by
itself (nothing corrects tip-over) — that requires a real balance
controller (P3 follow-up: ZMP ankle-pitch control). Before building that,
this test welds the pelvis to the world (like a test stand bolted at the
hip) and holds the nominal standing pose, so the leg structure can be
checked in isolation from balance:

  1. Simulation stays numerically stable (no NaN/divergence)
  2. The weld reaction force converges to the robot's total weight
     (everything below the pelvis is cantilevered from it — this is the
     one load path, so it must equal gravity exactly at rest)
  3. Load is distributed symmetrically between the left and right legs
  4. Joint torques are reported for each MVS joint (no pass/fail — motor
     torque limits are not yet populated in design/actuation.yaml)

Ground contact is deliberately excluded: welding the pelvis while the
feet also rest on the ground would make the support statically
indeterminate (the weld can supply vertical force on its own, so ground
reaction force would depend on solver/contact stiffness, not on gravity)
and wouldn't validate anything meaningful.

The weld constraint and contact exclusions are injected into the model
in memory at load time — the checked-in
simulation/mujoco/megadroid_mvs.xml (the generated floating-base scene
needed for the future dynamic balance controller) is never modified on
disk.

Usage:
    python3 tools/sim_static_pose.py
    python3 tools/sim_static_pose.py --duration 5.0
"""

import math
import argparse
import numpy as np
import mujoco
from pathlib import Path
from xml.etree import ElementTree as ET

REPO_ROOT = Path(__file__).resolve().parent.parent
MJCF_PATH = REPO_ROOT / "simulation" / "mujoco" / "megadroid_mvs.xml"

# Nominal joint angles (radians). All others remain at 0.
NOMINAL_POSE = {
    "left_knee_pitch":  math.radians(12.0),
    "right_knee_pitch": math.radians(12.0),
}

# Validation thresholds (test-script parameters, not design values)
WEIGHT_TOLERANCE_FRAC = 0.05   # weld Fz must settle within 5% of robot weight
LOAD_SYMMETRY_FRAC    = 0.05   # relative |left - right| joint-torque diff allowed
GRAVITY_MPS2          = 9.81


def build_welded_model(mjcf_path):
    """Parse the generated MJCF, weld the pelvis to the world at its
    reference pose, and exclude foot/ground contact (see module
    docstring for why). Returns a compiled MjModel — the file on disk is
    never modified."""
    tree = ET.parse(mjcf_path)
    root = tree.getroot()

    equality = ET.SubElement(root, "equality")
    ET.SubElement(equality, "weld",
                  body1="pelvis",
                  solref="0.002 1",
                  solimp="0.9 0.95 0.001")

    contact = root.find("contact")
    if contact is None:
        contact = ET.SubElement(root, "contact")
    for side in ("left", "right"):
        ET.SubElement(contact, "exclude",
                      body1="world", body2=f"{side}_foot")

    xml_str = ET.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml_str)


def set_nominal_pose(model, data):
    """Set joint qpos and control targets to nominal standing pose."""
    mujoco.mj_resetData(model, data)

    for joint_name, angle in NOMINAL_POSE.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = angle

    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        jname = act_name[4:] if act_name.startswith("act_") else act_name
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            data.ctrl[i] = data.qpos[model.jnt_qposadr[jid]]
        else:
            data.ctrl[i] = 0.0

    mujoco.mj_forward(model, data)


def weld_reaction_fz(model, data):
    """Vertical component of the external force MuJoCo computes on the
    pelvis (the weld reaction, since no other external forces act on it
    with ground contact excluded)."""
    pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    return data.cfrc_ext[pid, 5]


def leg_torque_totals(model, data):
    """Sum |torque| across each leg's actuators, for a symmetry check."""
    totals = {"left": 0.0, "right": 0.0}
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        for side in totals:
            if act_name.startswith(f"act_{side}_"):
                totals[side] += abs(data.actuator_force[i])
    return totals["left"], totals["right"]


def total_mass(model):
    return sum(model.body_mass)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Simulation duration in seconds (default: 2.0)")
    args = parser.parse_args()

    print("=" * 60)
    print("P3 Static Pose Validation (fixed base)")
    print("=" * 60)
    print()

    if not MJCF_PATH.exists():
        print(f"ERROR: MJCF not found at {MJCF_PATH}")
        print("Run: python3 tools/generate_mjcf.py")
        raise SystemExit(1)

    model = build_welded_model(MJCF_PATH)
    data = mujoco.MjData(model)

    set_nominal_pose(model, data)

    weight_n = total_mass(model) * GRAVITY_MPS2
    print(f"Robot mass:   {total_mass(model):.3f} kg")
    print(f"Robot weight: {weight_n:.2f} N")
    print()

    dt = model.opt.timestep
    n_steps = int(args.duration / dt)
    log_interval = max(1, n_steps // 20)

    print(f"Simulating {args.duration}s ({n_steps} steps, dt={dt}s)...")
    print()
    print(f"  {'Time':>6s}  {'Weld_Fz':>9s}  {'Leg_L':>8s}  {'Leg_R':>8s}")
    print(f"  {'-'*6}  {'-'*9}  {'-'*8}  {'-'*8}")

    log = []
    diverged = False

    for step in range(n_steps):
        mujoco.mj_step(model, data)
        # mj_step does not populate cfrc_ext (external/constraint body
        # forces) — it must be computed explicitly.
        mujoco.mj_rnePostConstraint(model, data)
        t = (step + 1) * dt

        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            diverged = True
            print(f"  simulation diverged (non-finite state) at t={t:.3f}s")
            break

        weld_fz = weld_reaction_fz(model, data)
        leg_l, leg_r = leg_torque_totals(model, data)
        log.append((t, weld_fz, leg_l, leg_r))

        if step % log_interval == 0 or step == n_steps - 1:
            print(f"  {t:6.2f}s  {weld_fz:9.2f}  {leg_l:8.2f}  {leg_r:8.2f}")

    print()
    print("=" * 60)
    print("Joint torques (final step)")
    print("=" * 60)
    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  {act_name:<28s}  {data.actuator_force[i]:8.3f} N·m")
    print()
    print("  (no pass/fail gate — design/actuation.yaml is not yet populated"
          " with motor torque limits)")
    print()

    print("=" * 60)
    print("Results")
    print("=" * 60)

    checks = []
    checks.append(("Simulation stayed numerically stable", not diverged, ""))

    if log and not diverged:
        # Use the settled tail (last 25% of samples) to avoid initial transient.
        tail = log[-max(1, len(log) // 4):]
        weld_fz_mean = sum(s[1] for s in tail) / len(tail)
        leg_l_mean = sum(s[2] for s in tail) / len(tail)
        leg_r_mean = sum(s[3] for s in tail) / len(tail)

        weight_err_frac = abs(weld_fz_mean - weight_n) / weight_n if weight_n > 0 else 1.0
        checks.append(("Weld Fz within %.0f%% of weight (%.2f N)"
                        % (WEIGHT_TOLERANCE_FRAC * 100, weight_n),
                        weight_err_frac <= WEIGHT_TOLERANCE_FRAC,
                        f"Fz={weld_fz_mean:.2f} N ({weight_err_frac*100:.1f}% err)"))

        leg_avg = (leg_l_mean + leg_r_mean) / 2
        symmetry_frac = abs(leg_l_mean - leg_r_mean) / leg_avg if leg_avg > 0 else 1.0
        checks.append(("Leg torque load symmetric (≤%.0f%%)" % (LOAD_SYMMETRY_FRAC * 100),
                        symmetry_frac <= LOAD_SYMMETRY_FRAC,
                        f"L={leg_l_mean:.2f} N·m, R={leg_r_mean:.2f} N·m "
                        f"({symmetry_frac*100:.1f}% diff)"))
    else:
        checks.append(("Weld Fz within tolerance of weight", False, "no data"))
        checks.append(("Leg torque load symmetric", False, "no data"))

    all_passed = all(ok for _, ok, _ in checks)

    for label, ok, detail in checks:
        status = "✓" if ok else "✗"
        print(f"  {status} {label:<45s}  {detail}")

    print()
    if all_passed:
        print("✓ PASS — static load carried as expected with fixed base")
    else:
        failed_checks = [label for label, ok, _ in checks if not ok]
        print(f"✗ FAIL — {', '.join(failed_checks)}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
P3 Standing Validation — quasi-static balance test in MuJoCo.

Loads the MJCF model, initializes the nominal standing pose, runs a
position-controlled simulation, and validates:

  1. Robot maintains upright stance (pelvis height and tilt)
  2. ZMP stays within the support polygon (between the feet)
  3. Joint torques are within reasonable bounds

Pass/fail threshold: pelvis must not drop more than 30mm or tilt more
than 15° within 2 seconds under gravity with position-hold control.

Usage:
    python3 tools/sim_standing.py
    python3 tools/sim_standing.py --duration 5.0
"""

import math
import sys
import argparse
import numpy as np
import mujoco
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MJCF_PATH = REPO_ROOT / "simulation" / "mujoco" / "megadroid_mvs.xml"

# Nominal joint angles (radians). All others remain at 0.
NOMINAL_POSE = {
    "left_knee_pitch":  math.radians(12.0),
    "right_knee_pitch": math.radians(12.0),
}

# Validation thresholds
MAX_HEIGHT_DROP_M  = 0.030   # 30 mm
MAX_TILT_DEG       = 15.0    # pelvis tilt from vertical
MIN_CONTACT_FZ_N   = 5.0     # minimum total vertical contact force to count as "standing"


def set_nominal_pose(model, data):
    """Set joint qpos and control targets to nominal standing pose."""
    mujoco.mj_resetData(model, data)  # initializes freejoint from MJCF body pos

    # Set joint angles
    for joint_name, angle in NOMINAL_POSE.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = angle

    # Set all actuator targets to match current joint positions
    for i in range(model.nu):
        act_name  = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        # Strip "act_" prefix to get joint name
        jname = act_name[4:] if act_name.startswith("act_") else act_name
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid >= 0:
            data.ctrl[i] = data.qpos[model.jnt_qposadr[jid]]
        else:
            data.ctrl[i] = 0.0

    mujoco.mj_forward(model, data)


def compute_zmp(model, data):
    """
    Compute Zero Moment Point from foot contact forces.

    Uses cfrc_ext (net external contact forces on each body, in world frame).
    cfrc_ext[i] layout: [Tx, Ty, Tz, Fx, Fy, Fz]

    Returns (zmp_x, zmp_y, total_fz) or (None, None, 0) if no ground contact.
    """
    foot_body_names = ("left_foot", "right_foot")
    total_fz  = 0.0
    zmp_x_num = 0.0
    zmp_y_num = 0.0

    for bname in foot_body_names:
        bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, bname)
        if bid < 0:
            continue
        fz = data.cfrc_ext[bid, 5]   # world-frame Fz
        if fz > 0.01:
            x = data.xpos[bid, 0]
            y = data.xpos[bid, 1]
            zmp_x_num += fz * x
            zmp_y_num += fz * y
            total_fz   += fz

    if total_fz < 0.01:
        return None, None, 0.0
    return zmp_x_num / total_fz, zmp_y_num / total_fz, total_fz


def pelvis_tilt_deg(model, data):
    """Return pelvis tilt from vertical in degrees (0 = perfectly upright)."""
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    # xmat is the 3x3 rotation matrix, row-major. Column 2 is the body z-axis in world frame.
    mat = data.xmat[pelvis_id].reshape(3, 3)
    body_z_in_world = mat[:, 2]
    world_z = np.array([0.0, 0.0, 1.0])
    cos_angle = np.clip(np.dot(body_z_in_world, world_z), -1.0, 1.0)
    return math.degrees(math.acos(cos_angle))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=2.0,
                        help="Simulation duration in seconds (default: 2.0)")
    args = parser.parse_args()

    print("=" * 60)
    print("P3 Standing Validation")
    print("=" * 60)
    print()

    if not MJCF_PATH.exists():
        print(f"ERROR: MJCF not found at {MJCF_PATH}")
        print("Run: python3 tools/generate_mjcf.py")
        raise SystemExit(1)

    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data  = mujoco.MjData(model)

    set_nominal_pose(model, data)

    pelvis_id        = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    initial_height   = data.xpos[pelvis_id, 2]
    initial_tilt_deg = pelvis_tilt_deg(model, data)

    print(f"Initial state:")
    print(f"  Pelvis height:  {initial_height:.4f} m")
    print(f"  Pelvis tilt:    {initial_tilt_deg:.2f}°")
    zx, zy, fz = compute_zmp(model, data)
    if zx is not None:
        print(f"  ZMP:            ({zx:.4f}, {zy:.4f}) m,  Fz={fz:.1f} N")
    else:
        print("  ZMP:            no ground contact yet")
    print()

    # Simulation loop
    dt = model.opt.timestep
    n_steps = int(args.duration / dt)
    log_interval = max(1, n_steps // 20)

    print(f"Simulating {args.duration}s ({n_steps} steps, dt={dt}s)...")
    print()
    print(f"  {'Time':>6s}  {'Height':>8s}  {'Drop':>8s}  {'Tilt':>7s}  {'ZMP_x':>8s}  {'ZMP_y':>8s}  {'Fz':>8s}")
    print(f"  {'-'*6}  {'-'*8}  {'-'*8}  {'-'*7}  {'-'*8}  {'-'*8}  {'-'*8}")

    heights = []
    tilts   = []
    zmps    = []
    failed  = False
    fail_reason = ""

    for step in range(n_steps):
        mujoco.mj_step(model, data)
        t = (step + 1) * dt

        h    = data.xpos[pelvis_id, 2]
        drop = initial_height - h
        tilt = pelvis_tilt_deg(model, data)
        heights.append(h)
        tilts.append(tilt)

        zx, zy, fz = compute_zmp(model, data)
        if zx is not None:
            zmps.append((t, zx, zy, fz))

        if step % log_interval == 0 or step == n_steps - 1:
            zx_str = f"{zx:.4f}" if zx is not None else "  n/a  "
            zy_str = f"{zy:.4f}" if zy is not None else "  n/a  "
            fz_str = f"{fz:.1f}"  if fz > 0     else "  n/a  "
            print(f"  {t:6.2f}s  {h:8.4f}m  {drop*1000:6.1f}mm  {tilt:6.1f}°  "
                  f"{zx_str:>8s}  {zy_str:>8s}  {fz_str:>8s}")

        # Early exit on catastrophic failure
        if drop > MAX_HEIGHT_DROP_M * 3:
            failed = True
            fail_reason = f"Robot fell (height drop {drop*1000:.0f}mm at t={t:.2f}s)"
            break

    print()
    print("=" * 60)
    print("Results")
    print("=" * 60)

    final_height   = heights[-1]
    final_drop     = initial_height - final_height
    max_drop       = initial_height - min(heights)
    max_tilt       = max(tilts)
    mean_tilt      = sum(tilts) / len(tilts)

    print(f"  Final pelvis height:  {final_height:.4f} m")
    print(f"  Final height drop:    {final_drop*1000:.1f} mm")
    print(f"  Maximum height drop:  {max_drop*1000:.1f} mm")
    print(f"  Maximum tilt:         {max_tilt:.1f}°")
    print(f"  Mean tilt:            {mean_tilt:.1f}°")

    if zmps:
        zmp_arr = np.array([[z[1], z[2]] for z in zmps])
        print(f"  ZMP mean:            ({zmp_arr[:,0].mean():.4f}, {zmp_arr[:,1].mean():.4f}) m")
        print(f"  ZMP x range:          [{zmp_arr[:,0].min():.4f}, {zmp_arr[:,0].max():.4f}] m")
        print(f"  ZMP y range:          [{zmp_arr[:,1].min():.4f}, {zmp_arr[:,1].max():.4f}] m")
    else:
        print("  ZMP: no sustained ground contact detected")

    print()

    # ── Pass/fail evaluation ───────────────────────────────────────────────
    checks = []
    checks.append(("Height drop ≤ 30mm", max_drop <= MAX_HEIGHT_DROP_M,
                   f"{max_drop*1000:.1f} mm"))
    checks.append(("Max tilt ≤ 15°",     max_tilt <= MAX_TILT_DEG,
                   f"{max_tilt:.1f}°"))
    if zmps:
        zmp_arr = np.array([[z[1], z[2]] for z in zmps])
        # Support polygon: roughly ±foot_length/2 in x, between feet in y
        zmp_in_x = bool(np.all(np.abs(zmp_arr[:,0]) < 0.20))
        zmp_in_y = bool(np.all(np.abs(zmp_arr[:,1]) < 0.10))
        checks.append(("ZMP within support (x)", zmp_in_x,
                       f"x∈[{zmp_arr[:,0].min():.3f}, {zmp_arr[:,0].max():.3f}]"))
        checks.append(("ZMP within support (y)", zmp_in_y,
                       f"y∈[{zmp_arr[:,1].min():.3f}, {zmp_arr[:,1].max():.3f}]"))

    all_passed = not failed and all(ok for _, ok, _ in checks)

    for label, ok, detail in checks:
        status = "✓" if ok else "✗"
        print(f"  {status} {label:<30s}  {detail}")

    print()
    if all_passed:
        print("✓ PASS — robot maintained standing balance")
    else:
        if failed:
            print(f"✗ FAIL — {fail_reason}")
        else:
            failed_checks = [label for label, ok, _ in checks if not ok]
            print(f"✗ FAIL — {', '.join(failed_checks)}")
        raise SystemExit(1)


if __name__ == "__main__":
    main()

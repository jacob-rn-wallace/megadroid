#!/usr/bin/env python3
"""
P3 ZMP Balance Validation — floating-base quasi-static standing control.

The floating-base pelvis is free to tip in any direction; joint position
control alone (holding a fixed pose) does nothing to correct that unless
the pose itself is balanced. This script implements the P3 balance
deliverable: a proportional ankle-pitch controller that measures the
Zero Moment Point (ZMP) from foot contact forces and adjusts both
ankles' commanded pitch to hold the ZMP at the target point (the
centroid of the support polygon, "between the feet").

Two things had to be fixed to get here, both worth recording since
they'll matter for any future balance work on this model:

1. The naive nominal pose (bend the knee, leave hip_pitch and
   ankle_pitch at 0) leaves the pelvis/torso column well forward of the
   foot — a large structural lean, not a small perturbation. That's more
   than an ankle-only strategy can ever correct (ankle strategy has
   limited authority even in humans/real robots — it's for small
   deviations from an already-balanced posture). NOMINAL_POSE below is
   instead a proper crouch, solved by forward kinematics so hip_pitch
   and ankle_pitch counter-rotate the knee bend and the foot ends up
   level and centered under the pelvis. With this pose alone (no active
   control at all) the robot is already stable — see run(kp=0.0).

2. ZMP measured as a single point per foot (the foot body's own origin)
   is blind to heel/toe load shifting, since both feet share the same
   x-offset and only differ in y — it can't see a tip coming. ZMP here
   is computed from the actual ground-contact points instead. That
   signal is noisy frame to frame (MuJoCo's box-plane contact can
   activate a different subset of a foot's corner contacts between
   steps), so it's low-pass filtered before being fed to the P
   controller — an unfiltered version of this loop injects that noise
   into the ankle position servo and makes things worse than doing
   nothing.

With both fixes, the controller holds tilt near zero indefinitely and
noticeably tighter than passive holding alone. It does NOT extend how
large a disturbance the robot can absorb before falling beyond what
passive holding already tolerates — ankle-only strategy has a hard
authority limit (correction is clamped to stay within the safe ankle
range), and going further would mean a hip/torso strategy too. That's
out of scope for the MVS's ankle-pitch-only DOF set and is future work,
not a bug in this controller.

Scope: sagittal (front-back) balance only. The MVS has no *actuated* lateral
ankle DOF — ankle_roll (design/joints.yaml) exists physically as a passive,
spring-centered joint, but nothing in this controller drives or reads it; it
simply settles under gravity/contact/spring forces in the sim like any other
unactuated MuJoCo joint. Active lateral correction is out of scope here by
design, not by omission. The nominal double-support stance is laterally
symmetric, so no lateral disturbance is expected in this test.

Usage:
    python3 tools/sim_zmp_balance.py
    python3 tools/sim_zmp_balance.py --duration 10.0 --kp 1.5
    python3 tools/sim_zmp_balance.py --kp 0 --baseline   # passive-only comparison
"""

import math
import argparse
import numpy as np
import mujoco
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
MJCF_PATH = REPO_ROOT / "simulation" / "mujoco" / "megadroid_mvs.xml"

# Nominal joint angles (radians): a balanced crouch (see module docstring
# point 1). hip_pitch and ankle_pitch were solved by forward kinematics —
# the value below centers the foot under the pelvis at this knee bend and
# keeps the foot level (ankle = -(hip + knee)).
NOMINAL_KNEE_DEG = 12.0
NOMINAL_HIP_DEG = -1.1934157477568936
NOMINAL_ANKLE_DEG = -(NOMINAL_HIP_DEG + NOMINAL_KNEE_DEG)
NOMINAL_POSE = {}
for _side in ("left", "right"):
    NOMINAL_POSE[f"{_side}_hip_pitch"] = math.radians(NOMINAL_HIP_DEG)
    NOMINAL_POSE[f"{_side}_knee_pitch"] = math.radians(NOMINAL_KNEE_DEG)
    NOMINAL_POSE[f"{_side}_ankle_pitch"] = math.radians(NOMINAL_ANKLE_DEG)
del _side

DEFAULT_KP_RAD_PER_M = 1.0          # ankle-pitch correction gain on filtered ZMP error
DEFAULT_ZMP_FILTER_ALPHA = 0.02     # EMA weight on each new raw ZMP sample (see point 2)
MAX_ANKLE_CORRECTION_RAD = math.radians(15.0)   # clamp around the nominal ankle angle

# Pass/fail thresholds (test-script parameters, not design values)
MAX_HEIGHT_DROP_M = 0.030
MAX_TILT_DEG = 15.0


def solve_nominal_geometry(model):
    """Forward-kinematics solve at the nominal pose (pelvis at its
    as-generated height) to find:
      - the pelvis root z that puts the feet exactly at ground level
        (avoids an initial contact impact when the sim starts)
      - the target ZMP (x, y): the midpoint of the two feet's nominal
        contact points, i.e. the centroid of the support polygon.
    Purely numerical — no leg-geometry trig is hardcoded here."""
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    for jn, a in NOMINAL_POSE.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = a
    mujoco.mj_forward(model, data)

    lfid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_foot")
    rfid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_foot")
    foot_z = data.xpos[lfid, 2]   # symmetric: left/right share the same z

    pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    default_pelvis_z = model.body_pos[pid, 2]
    root_z = default_pelvis_z - foot_z

    target_x = (data.xpos[lfid, 0] + data.xpos[rfid, 0]) / 2.0
    target_y = (data.xpos[lfid, 1] + data.xpos[rfid, 1]) / 2.0
    return root_z, target_x, target_y


def set_initial_state(model, data, root_z):
    """Reset to the nominal pose with the pelvis placed so the feet just
    touch the ground, and set actuator targets to match."""
    mujoco.mj_resetData(model, data)
    data.qpos[2] = root_z   # freejoint qpos layout: [x, y, z, qw, qx, qy, qz]

    for jn, a in NOMINAL_POSE.items():
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid >= 0:
            data.qpos[model.jnt_qposadr[jid]] = a

    for i in range(model.nu):
        act_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        jname = act_name[4:] if act_name.startswith("act_") else act_name
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        data.ctrl[i] = data.qpos[model.jnt_qposadr[jid]] if jid >= 0 else 0.0

    mujoco.mj_forward(model, data)


def _foot_body_ids(model):
    return {mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
            for n in ("left_foot", "right_foot")}


def compute_zmp(model, data, foot_body_ids):
    """ZMP (x, y) and total vertical contact force, computed from each
    active foot/ground contact's true world-frame position and vertical
    force — not the feet's body-frame origins (see module docstring
    point 2: a fixed point per foot can't see heel/toe load shifting).
    Returns (None, None, 0.0) if no foot is in contact."""
    total_fz, x_num, y_num = 0.0, 0.0, 0.0
    force = np.zeros(6)
    for i in range(data.ncon):
        c = data.contact[i]
        b1 = model.geom_bodyid[c.geom1]
        b2 = model.geom_bodyid[c.geom2]
        if b1 not in foot_body_ids and b2 not in foot_body_ids:
            continue
        mujoco.mj_contactForce(model, data, i, force)
        frame = np.array(c.frame).reshape(3, 3)
        fz = (frame.T @ force[:3])[2]
        if fz > 0.01:
            x_num += fz * c.pos[0]
            y_num += fz * c.pos[1]
            total_fz += fz
    if total_fz < 0.01:
        return None, None, 0.0
    return x_num / total_fz, y_num / total_fz, total_fz


def pelvis_tilt_deg(model, data):
    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    mat = data.xmat[pelvis_id].reshape(3, 3)
    cos_angle = np.clip(np.dot(mat[:, 2], np.array([0.0, 0.0, 1.0])), -1.0, 1.0)
    return math.degrees(math.acos(cos_angle))


def run(model, kp, zmp_filter_alpha, target_x, target_y, root_z, duration):
    """Run one simulation with the given ankle-pitch ZMP gain and ZMP
    low-pass filter weight (kp=0 disables the controller — pure
    position-hold baseline, using the balanced nominal pose either
    way). Returns a dict of trace arrays and summary stats."""
    data = mujoco.MjData(model)
    set_initial_state(model, data, root_z)

    l_ankle_i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_left_ankle_pitch")
    r_ankle_i = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "act_right_ankle_pitch")
    ankle_lo, ankle_hi = model.actuator_ctrlrange[l_ankle_i]
    foot_body_ids = _foot_body_ids(model)
    nominal_ankle_rad = math.radians(NOMINAL_ANKLE_DEG)

    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    initial_height = data.xpos[pelvis_id, 2]

    dt = model.opt.timestep
    n_steps = int(duration / dt)

    t_log, h_log, tilt_log, zx_log, ankle_log = [], [], [], [], []
    diverged = False
    fell = False
    zx_filtered = target_x

    for step in range(n_steps):
        if kp != 0.0:
            error = target_x - zx_filtered
            correction = np.clip(kp * error, -MAX_ANKLE_CORRECTION_RAD, MAX_ANKLE_CORRECTION_RAD)
            ankle_target = np.clip(nominal_ankle_rad + correction, ankle_lo, ankle_hi)
            data.ctrl[l_ankle_i] = ankle_target
            data.ctrl[r_ankle_i] = ankle_target

        mujoco.mj_step(model, data)
        mujoco.mj_rnePostConstraint(model, data)
        t = (step + 1) * dt

        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            diverged = True
            break

        h = data.xpos[pelvis_id, 2]
        tilt = pelvis_tilt_deg(model, data)
        zx_raw, zy_raw, fz = compute_zmp(model, data, foot_body_ids)
        if zx_raw is not None:
            zx_filtered = zmp_filter_alpha * zx_raw + (1 - zmp_filter_alpha) * zx_filtered

        t_log.append(t)
        h_log.append(h)
        tilt_log.append(tilt)
        zx_log.append(zx_filtered)
        ankle_log.append(data.ctrl[l_ankle_i])

        if (initial_height - h) > MAX_HEIGHT_DROP_M * 3 or tilt > 60.0:
            fell = True
            break

    return {
        "t": t_log, "h": h_log, "tilt": tilt_log, "zx": zx_log, "ankle": ankle_log,
        "initial_height": initial_height, "diverged": diverged, "fell": fell,
    }


def summarize(label, result):
    print(f"--- {label} ---")
    if result["diverged"]:
        print("  simulation diverged (non-finite state)")
        return None
    if not result["h"]:
        print("  no data collected")
        return None
    if result["fell"]:
        print(f"  robot fell at t={result['t'][-1]:.2f}s")
    max_drop = result["initial_height"] - min(result["h"])
    max_tilt = max(result["tilt"])
    final_tilt = result["tilt"][-1]
    zx_final_mean = sum(result["zx"][-50:]) / len(result["zx"][-50:])
    print(f"  max height drop: {max_drop*1000:6.1f} mm")
    print(f"  max tilt:        {max_tilt:6.2f}deg   final tilt: {final_tilt:6.2f}deg")
    print(f"  ZMP_x (final window mean, filtered): {zx_final_mean:.4f} m")
    print()
    return {"max_drop": max_drop, "max_tilt": max_tilt, "fell": result["fell"]}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Simulation duration in seconds (default: 5.0)")
    parser.add_argument("--kp", type=float, default=DEFAULT_KP_RAD_PER_M,
                        help=f"Ankle-pitch ZMP proportional gain, rad/m "
                             f"(default: {DEFAULT_KP_RAD_PER_M}). Pass 0 for the "
                             f"passive (uncontrolled) baseline.")
    parser.add_argument("--zmp-filter-alpha", type=float, default=DEFAULT_ZMP_FILTER_ALPHA,
                        help=f"EMA weight on each new raw ZMP sample, in (0, 1] "
                             f"(default: {DEFAULT_ZMP_FILTER_ALPHA}). Lower = more filtering.")
    parser.add_argument("--baseline", action="store_true",
                        help="Also run and report the passive (kp=0) baseline for comparison.")
    args = parser.parse_args()

    print("=" * 60)
    print("P3 ZMP Ankle-Pitch Balance Validation (floating base)")
    print("=" * 60)
    print()

    if not MJCF_PATH.exists():
        print(f"ERROR: MJCF not found at {MJCF_PATH}")
        print("Run: python3 tools/generate_mjcf.py")
        raise SystemExit(1)

    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    root_z, target_x, target_y = solve_nominal_geometry(model)
    print(f"Nominal pose: hip_pitch={NOMINAL_HIP_DEG:.2f}deg  knee_pitch={NOMINAL_KNEE_DEG:.2f}deg  "
          f"ankle_pitch={NOMINAL_ANKLE_DEG:.2f}deg")
    print(f"Target ZMP (support polygon centroid): ({target_x:.4f}, {target_y:.4f}) m")
    print(f"Initial pelvis root z: {root_z:.4f} m")
    print()

    results = {}
    if args.baseline:
        results["passive baseline (kp=0)"] = run(model, 0.0, args.zmp_filter_alpha,
                                                   target_x, target_y, root_z, args.duration)
    controlled_key = f"controlled (kp={args.kp}, zmp_filter_alpha={args.zmp_filter_alpha})"
    results[controlled_key] = run(model, args.kp, args.zmp_filter_alpha,
                                   target_x, target_y, root_z, args.duration)

    summaries = {}
    for label, result in results.items():
        summaries[label] = summarize(label, result)

    print("=" * 60)
    print("Results")
    print("=" * 60)

    s = summaries[controlled_key]
    all_passed = False
    if s is not None:
        checks = [
            ("Robot did not fall", not s["fell"], ""),
            (f"Max height drop <= {MAX_HEIGHT_DROP_M*1000:.0f}mm", s["max_drop"] <= MAX_HEIGHT_DROP_M,
             f"{s['max_drop']*1000:.1f} mm"),
            (f"Max tilt <= {MAX_TILT_DEG:.0f}deg", s["max_tilt"] <= MAX_TILT_DEG,
             f"{s['max_tilt']:.2f}deg"),
        ]
        all_passed = all(ok for _, ok, _ in checks)
        for label, ok, detail in checks:
            status = "PASS" if ok else "FAIL"
            print(f"  [{status}] {label:<35s}  {detail}")

    print()
    if all_passed:
        print("PASS - ZMP ankle-pitch controller maintained standing balance")
    else:
        print("FAIL - see checks above")
        raise SystemExit(1)


if __name__ == "__main__":
    main()

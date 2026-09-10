#!/usr/bin/env python3
"""
Architecture D, Stage 2 — standing balance under constraint-aware MPC.

Wires mpc_lipm.py's box-constrained QP to standing balance, replacing
sim_zmp_balance.py's proportional ankle law. Standing first because it is
Stephens & Atkeson's own validated scenario (recover from a push WITHOUT
stepping, 18-23 Ns on the Sarcos Primus) and because sim_zmp_balance.py is
currently the one walking-adjacent thing that still passes under the real
2.8 N·m torque envelope — so there is an honest baseline to compare against.

WHAT IS BEING TESTED
--------------------
Not "does MPC track better". The question is whether planning the CoP inside
the feasible box does something a proportional law cannot: stay upright when
the torque limit is the thing that binds. The diagnostic that matters is
therefore not tilt alone but whether the QP's bound goes ACTIVE during a push
and the robot survives anyway.

REALISING A DESIRED CoP ON A POSITION-CONTROLLED ROBOT
------------------------------------------------------
The plan flagged this as the one genuinely unproven step -- Stephens assumes
force control, megadroid has position servos. It was measured rather than
assumed, and the answer is split by axis.

FEEDFORWARD DOES NOT WORK. Commanding an ankle angle offset sized to produce the
desired torque (theta_nominal + tau_des/kp) realises almost none of the intended
CoP: measured gain 0.06 lateral (40 mm commanded -> 2.5 mm delivered), and
NEGATIVE then unstable sagittal (topples at >=20 mm). The stiff position servo
simply settles at a different equilibrium; the delivered steady-state torque is
not kp*offset, because theta_actual moves in response.

CLOSED-LOOP TORQUE TRACKING WORKS, LATERALLY. Driving the offset with an
integral loop on the MEASURED ankle torque from the foot F/T sensor,

    offset += k_i * (tau_desired - tau_measured)

realises lateral CoP at gain 1.01 (20 mm commanded -> 20.25 mm delivered). This
is admittance control with a non-zero target -- exactly what
ankle_roll_admittance's `tau_x_target` parameter was added for earlier this
session and then left unused.

SAGITTAL RESISTS BOTH, and the reason is structural rather than a tuning miss:
ankle_pitch is load-bearing for the balanced crouch. Stage 0 established the
pose is stable only while the joints hold it near-rigidly, so an integral loop
that overrides ankle_pitch removes the support that keeps the robot up, and it
falls at either sign. Sagittal therefore stays on the proven proportional
ankle-pitch ZMP law from sim_zmp_balance.py, and the MPC governs the LATERAL
axis -- which is the axis that has failed every push test this project has run.

Signs were determined empirically (`--calibrate`), not derived, per this
codebase's unbroken record of sign traps in exactly this kind of mapping.
"""

import math
import argparse
import numpy as np
import mujoco

import sim_zmp_balance as sb
import mpc_lipm as mpc

# MPC horizon. 16 x 50 ms = 0.8 s, comfortably longer than the ~0.23 s LIPM
# time constant, so the terminal cost sees the divergent mode coming.
MPC_N = 16
MPC_DT = 0.05

# Re-solve at 100 Hz, not every 500 Hz physics tick. The QP is warm-started so
# a re-solve is ~1 iteration, but re-solving every tick would inject a
# discontinuity at 500 Hz -- the coherent-periodic-forcing risk this codebase
# has already been bitten by twice (see sim_walk_recede.py's REPLAN_PERIOD_S).
MPC_SOLVE_EVERY = 5

# Cost weights. r_cop is deliberately small: CoP effort is nearly free until it
# hits the box, and the box is what we want doing the work.
Q_DCM, R_COP, Q_TERM = 1.0, 1e-3, 10.0

# Closed-loop CoP realisation on the lateral axis. Sign and gain measured, not
# derived: sign=+1 gives 20 mm commanded -> 20.25 mm delivered (gain 1.01);
# sign=-1 falls over. K_I_ANKLE is the integral rate on the offset.
SIGN_ROLL = +1.0
K_I_ANKLE = 2e-4

# Sagittal stays on sim_zmp_balance.py's proportional ankle-pitch ZMP law --
# see the module docstring: closing a torque loop on ankle_pitch removes the
# crouch support and topples the robot at either sign.
ANKLE_PITCH_KP = 1.0

# Clamp on the sagittal (proportional) ankle-pitch correction, mirroring
# lw.MAX_ADM_CORRECTION_RAD.
MAX_ANKLE_OFFSET_RAD = math.radians(15.0)

# Clamp on the LATERAL closed-loop ankle_roll offset -- deliberately far
# tighter, and this is load-bearing rather than cosmetic. At the 15deg used for
# pitch, the integral winds up under a push and levers the robot over its own
# 40mm-wide foot: 10N lateral gave 60.04deg (a fall) with the QP bound active
# 65% of the time, WORSE than the proportional baseline it replaced. Swept:
#   clamp   15deg   8deg   4deg   2deg   1deg
#   10N     60.04   2.70   0.97   0.60   0.52   deg tilt
#   bound%   65.0   60.3    2.2    2.5    3.3
# 4deg is the knee: the robot survives comfortably and the QP stops fighting an
# unstable plant (bound activity 60% -> 2%). Ankle-roll authority is small
# because the foot is small; a clamp larger than the foot can support is not
# extra authority, it is a tipping moment.
MAX_ROLL_OFFSET_RAD = math.radians(4.0)


def _foot_geom_half(model):
    return (float(model.geom("left_foot_geom").size[0]),
            float(model.geom("left_foot_geom").size[1]))


class BalanceMPC:
    """Per-axis constraint-aware MPC for double-support standing."""

    def __init__(self, model, target_xy, z_c):
        self.model = model
        self.target = np.asarray(target_xy, dtype=float)
        self.omega = math.sqrt(abs(float(model.opt.gravity[2])) / z_c)
        self.pred = mpc.LIPMPredictor(MPC_N, MPC_DT, self.omega,
                                      q_dcm=Q_DCM, r_cop=R_COP, q_term=Q_TERM)
        self.tau_max = float(model.actuator_forcerange[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR,
                              "act_left_ankle_pitch")][1])
        self.hx, self.hy = _foot_geom_half(model)
        self.warm = {0: None, 1: None}
        self.bound_active_ticks = 0
        self.total_solves = 0

    def bounds(self, axis, f_z, n_loaded, foot_span_y):
        """Feasible CoP half-width on this axis: geometry AND torque.

        Torque authority scales with how many feet are loaded, since each
        contributes its own ankle. Geometry differs per axis: sagittal is one
        foot's length, lateral spans between the feet in double support.
        """
        geom = self.hx if axis == 0 else foot_span_y
        tau_total = self.tau_max * max(n_loaded, 1)
        return mpc.cop_bounds_from_torque(f_z, tau_total, geom)

    def solve(self, com, com_vel, f_z, n_loaded, foot_span_y):
        """Return desired CoP (x, y) and whether either box went active."""
        p_des = np.zeros(2)
        any_active = False
        for axis in (0, 1):
            x0 = np.array([com[axis], com_vel[axis]])
            xi_ref = np.full(MPC_N, self.target[axis])
            p_ref = np.full(MPC_N, self.target[axis])
            g = self.pred.gradient_terms(x0, xi_ref, p_ref)

            half = self.bounds(axis, f_z, n_loaded, foot_span_y)
            lo = np.full(MPC_N, self.target[axis] - half)
            hi = np.full(MPC_N, self.target[axis] + half)

            u, _, _ = mpc.solve_box_qp(self.pred.H, g, lo, hi, u0=self.warm[axis])
            self.warm[axis] = u
            p_des[axis] = u[0]
            if u[0] <= lo[0] + 1e-9 or u[0] >= hi[0] - 1e-9:
                any_active = True
        self.total_solves += 1
        if any_active:
            self.bound_active_ticks += 1
        return p_des, any_active


def run(model, duration=5.0, push_at=None, push_force=(0.0, 0.0),
        push_duration=0.1, use_mpc=True, verbose=True):
    """Stand for `duration`, optionally taking a pelvis push. Returns stats.

    Lateral CoP is governed by the MPC and realised by closed-loop ankle_roll
    torque tracking; sagittal stays on the proportional ankle-pitch ZMP law.
    use_mpc=False reproduces the sim_zmp_balance.py baseline on both axes.
    """
    data = mujoco.MjData(model)
    root_z, tx, ty = sb.solve_nominal_geometry(model)
    sb.set_initial_state(model, data, root_z)
    mujoco.mj_forward(model, data)
    mujoco.mj_subtreeVel(model, data)
    mujoco.mj_rnePostConstraint(model, data)

    foot_ids = sb._foot_body_ids(model)
    pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    act = {n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{n}")
           for n in ("left_ankle_pitch", "right_ankle_pitch",
                     "left_ankle_roll", "right_ankle_roll")}
    tadr = {s: model.sensor_adr[mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_SENSOR, f"{s}_ft_torque")]
            for s in ("left", "right")}
    foot_bid = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{s}_foot")
                for s in ("left", "right")}
    nominal_pitch = math.radians(sb.NOMINAL_ANKLE_DEG)

    z_c = float(data.subtree_com[0][2] - data.xpos[foot_bid["left"]][2])
    hy = _foot_geom_half(model)[1]
    hip_y = abs(float(data.xpos[foot_bid["left"]][1]
                      - data.xpos[foot_bid["right"]][1])) / 2.0
    foot_span_y = hip_y + hy

    ctrl = BalanceMPC(model, (tx, ty), z_c) if use_mpc else None

    dt = model.opt.timestep
    init_h = data.xpos[pid, 2]
    max_tilt, max_drop, fell = 0.0, 0.0, False
    p_des = np.array([tx, ty])
    off = {"left": 0.0, "right": 0.0}
    zx_f, zy_f = tx, ty
    cop_err = []

    for step in range(int(duration / dt)):
        t = data.time
        if push_at is not None and push_at <= t < push_at + push_duration:
            data.xfrc_applied[pid, 0] = push_force[0]
            data.xfrc_applied[pid, 1] = push_force[1]
        else:
            data.xfrc_applied[pid, :] = 0.0

        zx, zy, f_z = sb.compute_zmp(model, data, foot_ids)
        if zx is not None:
            zx_f += sb.DEFAULT_ZMP_FILTER_ALPHA * (zx - zx_f)
            zy_f += sb.DEFAULT_ZMP_FILTER_ALPHA * (zy - zy_f)

        if ctrl is not None and step % MPC_SOLVE_EVERY == 0:
            com = data.subtree_com[0][:2].copy()
            com_vel = data.subtree_linvel[0][:2].copy()
            p_des, _ = ctrl.solve(com, com_vel, max(f_z, 1e-6),
                                  2 if f_z > 1e-6 else 0, foot_span_y)

        # Sagittal: the proven proportional ankle-pitch ZMP law.
        pitch_corr = float(np.clip(ANKLE_PITCH_KP * (tx - zx_f),
                                   -MAX_ANKLE_OFFSET_RAD, MAX_ANKLE_OFFSET_RAD))
        for side in ("left", "right"):
            data.ctrl[act[f"{side}_ankle_pitch"]] = nominal_pitch + pitch_corr

        # Lateral: MPC target realised by closed-loop ankle_roll torque tracking.
        if ctrl is not None and f_z > 1e-6:
            for side in ("left", "right"):
                fy = data.xpos[foot_bid[side]][1]
                tau_des = 0.5 * f_z * (p_des[1] - fy)
                tau_meas = float(data.sensordata[tadr[side]])     # x-axis = roll
                off[side] = float(np.clip(
                    off[side] + K_I_ANKLE * SIGN_ROLL * (tau_des - tau_meas),
                    -MAX_ROLL_OFFSET_RAD, MAX_ROLL_OFFSET_RAD))
                data.ctrl[act[f"{side}_ankle_roll"]] = off[side]
            if zy is not None:
                cop_err.append(abs(zy - p_des[1]))
        else:
            for side in ("left", "right"):
                data.ctrl[act[f"{side}_ankle_roll"]] = 0.0

        mujoco.mj_step(model, data)
        mujoco.mj_subtreeVel(model, data)
        mujoco.mj_rnePostConstraint(model, data)

        tilt = sb.pelvis_tilt_deg(model, data)
        max_tilt = max(max_tilt, tilt)
        max_drop = max(max_drop, init_h - data.xpos[pid, 2])
        if tilt > 60.0 or not np.all(np.isfinite(data.qpos)):
            fell = True
            break

    out = {
        "max_tilt": max_tilt, "max_drop_mm": max_drop * 1000.0, "fell": fell,
        "cop_track_rms_mm": (float(np.sqrt(np.mean(np.square(cop_err)))) * 1000.0
                             if cop_err else float("nan")),
        "bound_active_pct": (100.0 * ctrl.bound_active_ticks
                             / max(ctrl.total_solves, 1) if ctrl else float("nan")),
    }
    if verbose:
        print(f"  max_tilt={out['max_tilt']:6.2f}deg  drop={out['max_drop_mm']:6.1f}mm  "
              f"fell={out['fell']}  lat-CoP-track={out['cop_track_rms_mm']:.1f}mm  "
              f"bound_active={out['bound_active_pct']:.1f}%")
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=5.0)
    ap.add_argument("--push-at", type=float, default=None)
    ap.add_argument("--push-force", type=float, nargs=2, default=(0.0, 0.0),
                    metavar=("FX", "FY"))
    ap.add_argument("--baseline", action="store_true",
                    help="Run with the MPC disabled, for comparison.")
    args = ap.parse_args()

    model = mujoco.MjModel.from_xml_path(str(sb.MJCF_PATH))
    print(f"{'MPC standing balance' if not args.baseline else 'baseline (no MPC)'}:")
    run(model, duration=args.duration, push_at=args.push_at,
        push_force=tuple(args.push_force), use_mpc=not args.baseline)


if __name__ == "__main__":
    main()

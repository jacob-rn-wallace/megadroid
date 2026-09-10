#!/usr/bin/env python3
"""
P3 state estimator — CoM / CoM-velocity / DCM from MEASURABLE signals only.

WHY THIS EXISTS
---------------
Every walking controller in this repo (sim_walk_lipm.py, sim_walk_recede.py,
sim_walk_gait.py) drives off `data.subtree_com[0]` and `data.subtree_linvel[0]`
— MuJoCo's world-frame centre-of-mass position and velocity. Those are
privileged simulator state. The real robot, per design/sensors.yaml, has
joint-mounted absolute encoders, a per-foot 6-DOF F/T sensor, and (as of
2026-09-10) a 6-axis pelvis IMU. None of those measure absolute CoM velocity,
and before the IMU was added nothing measured pelvis orientation either.

So the DCM signal that every P3 disturbance-rejection mechanism was tuned
against was not reproducible on hardware. This module estimates that state
from what the robot actually has, and — critically — reports how wrong the
estimate is, so the size of that gap is a measured number rather than an
assumption.

THE RULE THIS MODULE FOLLOWS
----------------------------
It may read, from the live `MjData`:
  - hinge joint qpos                      (joint-mounted absolute encoders)
  - sensordata for {side}_ft_force/torque (per-foot 6-DOF F/T)
  - sensordata for imu_gyro / imu_accel   (6-axis pelvis IMU)

It may NOT read: subtree_com, subtree_linvel, cvel, the freejoint's qpos/qvel
(the robot cannot measure its own world pose), or the framepos/framequat
sensors. Those exist in the model for scoring an estimator, never as input.

A subtlety worth stating because it looks like a violation and isn't: this
module DOES call mj_forward and read subtree_com — but on its OWN private
MjData, populated from measured encoder angles and its own estimated
orientation. That is using MuJoCo as a forward-kinematics library the way the
real robot would use its own URDF, not reading the simulation's truth.

METHOD
------
1. Orientation: complementary filter. Integrate the gyro for short-term rate,
   correct roll/pitch drift against the accelerometer's gravity direction.
   Yaw has no absolute reference (design/sensors.yaml deliberately omits the
   magnetometer) so it drifts; the drift is reported rather than hidden.
2. Stance: the foot carrying more vertical load, from the F/T force channel.
3. Position: leg odometry. Assume the stance foot is stationary, run FK up
   from it with measured joint angles and estimated orientation to place the
   pelvis, then take the whole-body CoM from the kinematic model. The anchor
   is handed over when stance switches, so error accumulates exactly as it
   would on hardware.
4. Velocity: filtered finite difference of estimated CoM. This is the hard
   part, and the quantity the whole DCM method depends on.
"""

import math
import numpy as np
import mujoco

HINGE_JOINTS = (
    "left_hip_roll", "right_hip_roll",
    "left_hip_pitch", "right_hip_pitch",
    "left_knee_pitch", "right_knee_pitch",
    "left_ankle_pitch", "right_ankle_pitch",
    "left_ankle_roll", "right_ankle_roll",
    "torso_pitch", "torso_roll", "torso_yaw",
)

# Complementary-filter blend toward the accelerometer's gravity direction, per
# tick. The accelerometer also sees real linear acceleration during walking,
# which is NOT gravity, so trusting it too fast injects gait acceleration into
# the orientation estimate -- and orientation error is amplified by the ~0.5m
# FK lever arm from stance foot to CoM.
#
# SWEPT 2026-09-10 (8-step receding-horizon gait, balance-relevant CoM error):
#   alpha   0.0     0.001   0.005   0.02    0.05
#   CoM-rel 0.48    4.58    17.40   48.19   75.60  mm
# This is monotonic: LESS accelerometer correction is strictly better here,
# and disabling it entirely is best by a wide margin.
#
# 0.001 is used anyway, NOT 0.0, and the reason matters. MuJoCo's gyro is
# noise-free and bias-free, so pure integration is flawless in simulation and
# catastrophic on hardware -- a real MEMS gyro's bias drift accumulates without
# bound unless something observes gravity. Tuning this constant against an
# ideal sensor would repeat, in the estimator, exactly the mistake the torque
# envelope just corrected in the actuators: letting an idealised model flatter
# the result. 0.001 gives a ~2s filter time constant, which is a defensible
# textbook complementary-filter value independent of what the sim rewards.
#
# THIS CONSTANT CANNOT BE HONESTLY TUNED until the MJCF models gyro/
# accelerometer noise and bias. Until then, treat any estimator error figure
# from this file as a LOWER BOUND on the real thing.
ACCEL_TRUST_ALPHA = 0.001

# Fraction of body weight on a foot before it counts as stance.
STANCE_LOAD_FRACTION = 0.25

# EMA on the finite-differenced CoM velocity. Differentiation amplifies noise;
# this is the same lesson sim_zmp_balance.py and sim_walk_lipm.py both learned
# about raw measurements, applied to a derived one.
#
# SWEPT 2026-09-10 (same run, DCM error, which is what a controller consumes):
#   alpha   0.005   0.01    0.02    0.05    0.1     0.3
#   DCM RMS 86.61   105.71  132.25  178.06  216.86  271.15  mm
# Monotonic again -- heavier filtering is better on this metric. Note the
# metric does not penalise LAG, and lag inside a feedback loop is destabilising,
# so this should be re-checked against closed-loop behaviour before any
# controller actually consumes this signal. 0.005 is a ~0.4s time constant.
VEL_FILTER_ALPHA = 0.005


def _quat_mul(a, b):
    out = np.zeros(4)
    mujoco.mju_mulQuat(out, a, b)
    return out


def _rot_vec(q, v):
    out = np.zeros(3)
    mujoco.mju_rotVecQuat(out, v, q)
    return out


def _quat_conj(q):
    out = np.zeros(4)
    mujoco.mju_negQuat(out, q)
    return out


class StateEstimator:
    """Estimates CoM, CoM velocity and DCM from encoders + F/T + IMU only."""

    def __init__(self, model, tc, initial_stance_foot_world=None):
        self.model = model
        self.tc = tc                      # LIPM time constant, for the DCM
        self.dt = model.opt.timestep
        self.data_fk = mujoco.MjData(model)   # private, FK only

        self.jnt_qposadr = {}
        for jn in HINGE_JOINTS:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            if jid >= 0:
                self.jnt_qposadr[jn] = model.jnt_qposadr[jid]

        self.ft_force_adr = {
            s: model.sensor_adr[mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_SENSOR, f"{s}_ft_force")]
            for s in ("left", "right")
        }
        self.gyro_adr = model.sensor_adr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_gyro")]
        self.accel_adr = model.sensor_adr[mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SENSOR, "imu_accel")]

        self.foot_bid = {
            s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{s}_foot")
            for s in ("left", "right")
        }

        self.total_mass = float(model.body_subtreemass[
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")])
        self.weight_n = self.total_mass * abs(float(model.opt.gravity[2]))

        # Estimated pelvis orientation, world<-body. Initialised upright: a
        # real robot would level itself at startup with the accelerometer.
        self.quat = np.array([1.0, 0.0, 0.0, 0.0])

        # Leg-odometry anchor: estimated world position of the stance foot.
        # Needs ONE external initialisation, exactly like a real robot zeroing
        # its odometry frame at startup. Everything after is dead reckoning.
        self.stance = "right"
        self.anchor = (np.zeros(3) if initial_stance_foot_world is None
                       else np.asarray(initial_stance_foot_world, dtype=float))

        self.com = None
        self.com_vel = np.zeros(3)
        self._com_prev = None
        self._initialised = False

    # ---- measurement accessors (the ONLY things read from live data) ----

    def _encoders(self, data):
        return {jn: float(data.qpos[adr]) for jn, adr in self.jnt_qposadr.items()}

    def _gyro(self, data):
        return np.array(data.sensordata[self.gyro_adr:self.gyro_adr + 3])

    def _accel(self, data):
        return np.array(data.sensordata[self.accel_adr:self.accel_adr + 3])

    def _foot_load(self, data, side):
        adr = self.ft_force_adr[side]
        return abs(float(data.sensordata[adr + 2]))

    # ---- estimation steps ----

    def _update_orientation(self, gyro, accel):
        """Complementary filter: gyro integration + accelerometer levelling."""
        w = np.linalg.norm(gyro)
        if w > 1e-9:
            dq = np.zeros(4)
            mujoco.mju_axisAngle2Quat(dq, gyro / w, w * self.dt)
            self.quat = _quat_mul(self.quat, dq)

        a = np.linalg.norm(accel)
        if a > 1e-6:
            # MuJoCo's accelerometer reads +z in body frame when upright
            # (verified directly on this model: [0, 0, +9.80] standing), so
            # measured "up" is +normalize(accel), not -.
            up_meas = accel / a
            up_pred = _rot_vec(_quat_conj(self.quat), np.array([0.0, 0.0, 1.0]))
            # SIGN, derived and then confirmed by the failure it caused:
            # applying `corr` as a body-frame increment (q <- q (x) corr) makes
            # the new body-frame world-up equal R_corr^T * up_pred. Setting that
            # equal to up_meas means R_corr must rotate up_meas -> up_pred, so
            # the axis is cross(up_meas, up_pred). The opposite order is a
            # stable-but-wrong feedback loop: it drives the estimate to the
            # ANTIPODE and parks there (observed as a ~180deg flip about Y and
            # ~170deg of apparent yaw drift before this was fixed).
            err = np.cross(up_meas, up_pred)
            e = np.linalg.norm(err)
            if e > 1e-9:
                corr = np.zeros(4)
                mujoco.mju_axisAngle2Quat(
                    corr, err / e, ACCEL_TRUST_ALPHA * math.asin(min(e, 1.0)))
                self.quat = _quat_mul(self.quat, corr)

        mujoco.mju_normalize4(self.quat)

    def _fk(self, encoders):
        """Forward kinematics on our OWN data, pelvis at the origin.

        Returns (com_rel, foot_rel) with the pelvis at the origin and the
        estimated orientation applied. Because translating the free root
        translates every body rigidly, callers can place the robot by adding a
        single offset rather than re-running FK.
        """
        d = self.data_fk
        mujoco.mj_resetData(self.model, d)
        d.qpos[0:3] = 0.0
        d.qpos[3:7] = self.quat
        for jn, adr in self.jnt_qposadr.items():
            d.qpos[adr] = encoders[jn]
        mujoco.mj_forward(self.model, d)
        return (d.subtree_com[0].copy(),
                {s: d.xpos[bid].copy() for s, bid in self.foot_bid.items()})

    def update(self, data):
        """Advance the estimate one tick. Returns a dict of estimates."""
        gyro, accel = self._gyro(data), self._accel(data)
        self._update_orientation(gyro, accel)

        encoders = self._encoders(data)
        com_rel, foot_rel = self._fk(encoders)

        loads = {s: self._foot_load(data, s) for s in ("left", "right")}
        threshold = STANCE_LOAD_FRACTION * self.weight_n
        loaded = [s for s in ("left", "right") if loads[s] > threshold]

        # Hand the anchor over on a stance switch, keeping the pelvis estimate
        # continuous across the handover: re-express the anchor as the NEW
        # stance foot's current estimated position. Any error in the old
        # estimate is inherited, which is the real behaviour of leg odometry.
        if loaded and self.stance not in loaded:
            new_stance = max(loaded, key=lambda s: loads[s])
            pelvis_now = self.anchor - foot_rel[self.stance]
            self.anchor = pelvis_now + foot_rel[new_stance]
            self.stance = new_stance

        pelvis_pos = self.anchor - foot_rel[self.stance]
        com = com_rel + pelvis_pos

        if not self._initialised:
            self.com, self._com_prev = com, com
            self._initialised = True
        else:
            raw_v = (com - self._com_prev) / self.dt
            self.com_vel = ((1 - VEL_FILTER_ALPHA) * self.com_vel
                            + VEL_FILTER_ALPHA * raw_v)
            self._com_prev = com
            self.com = com

        dcm = self.com[:2] + self.tc * self.com_vel[:2]
        return {"com": self.com.copy(), "com_vel": self.com_vel.copy(),
                "dcm": dcm.copy(), "quat": self.quat.copy(),
                "stance": self.stance}


def _yaw_of(q):
    r = np.zeros(9)
    mujoco.mju_quat2Mat(r, q)
    return math.atan2(r[3], r[0])


def validate(gait="recede", n_steps=8, verbose=True):
    """Score the estimator against MuJoCo ground truth over a real gait run.

    Ground truth is read ONLY here, for scoring -- never inside the estimator.
    """
    import sim_walk_lipm as lw
    import sim_walk_recede as swr

    model = mujoco.MjModel.from_xml_path(str(swr.MJCF_PATH))
    pelvis_z, hip_deg, knee_deg, ankle_deg = lw.solve_walk_pose(model)
    z_c, _, _, _ = lw.solve_walk_com_height(model, pelvis_z, hip_deg, knee_deg, ankle_deg)
    tc = math.sqrt(z_c / lw.G)

    est = {"obj": None}
    rec = {"com": [], "vel": [], "dcm": [], "yaw": [],
           "com_rel": [], "dcm_rel": []}

    orig_step = mujoco.mj_step

    def traced(m, d, nstep=1):
        orig_step(m, d, nstep)
        mujoco.mj_subtreeVel(m, d)
        mujoco.mj_rnePostConstraint(m, d)
        if est["obj"] is None:
            fid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_foot")
            est["obj"] = StateEstimator(m, tc, d.xpos[fid].copy())
        e = est["obj"].update(d)

        true_com = d.subtree_com[0].copy()
        true_vel = d.subtree_linvel[0].copy()
        true_dcm = true_com[:2] + tc * true_vel[:2]
        pid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        true_q = d.xquat[pid].copy()

        rec["com"].append(e["com"] - true_com)
        rec["vel"].append(e["com_vel"] - true_vel)
        rec["dcm"].append(e["dcm"] - true_dcm)
        rec["yaw"].append(_yaw_of(e["quat"]) - _yaw_of(true_q))

        # Balance-relevant error: CoM RELATIVE TO THE STANCE FOOT. This is what
        # a balance controller actually consumes -- where the CoM sits over the
        # support polygon -- and it is free of leg-odometry drift, since both
        # the estimate and the truth are differenced against the same foot.
        # Absolute world pose drifts without exteroception on any robot; that
        # matters for navigation, not for staying upright, so scoring only the
        # absolute error would overstate the problem.
        est_obj = est["obj"]
        sfoot_bid = est_obj.foot_bid[e["stance"]]
        true_rel = true_com - d.xpos[sfoot_bid]
        est_rel = e["com"] - est_obj.anchor      # anchor IS the estimated foot
        rec["com_rel"].append(est_rel - true_rel)
        true_dcm_rel = true_rel[:2] + tc * true_vel[:2]
        est_dcm_rel = est_rel[:2] + tc * e["com_vel"][:2]
        rec["dcm_rel"].append(est_dcm_rel - true_dcm_rel)

    mujoco.mj_step = traced
    try:
        if gait == "recede":
            r = swr.run_walk_recede(model, n_steps=n_steps, verbose=False)
        else:
            r = lw.run_walk(model, n_steps=n_steps, verbose=False)
    finally:
        mujoco.mj_step = orig_step

    com = np.array(rec["com"]); vel = np.array(rec["vel"])
    dcm = np.array(rec["dcm"]); yaw = np.degrees(np.array(rec["yaw"]))
    com_rel = np.array(rec["com_rel"]); dcm_rel = np.array(rec["dcm_rel"])

    def rms(a):
        return float(np.sqrt((np.linalg.norm(a, axis=1) ** 2).mean()))

    out = {
        "gait": gait, "n_steps": n_steps, "ticks": len(com),
        "max_tilt_deg": r["max_tilt_deg"],
        "com_rms_mm": rms(com) * 1000.0,
        "com_final_mm": float(np.linalg.norm(com[-1])) * 1000.0,
        "vel_rms_mm_s": rms(vel) * 1000.0,
        "dcm_rms_mm": rms(dcm) * 1000.0,
        "dcm_final_mm": float(np.linalg.norm(dcm[-1])) * 1000.0,
        "yaw_drift_deg": float(yaw[-1]),
        "com_rel_rms_mm": rms(com_rel) * 1000.0,
        "dcm_rel_rms_mm": rms(dcm_rel) * 1000.0,
    }

    if verbose:
        print(f"=== estimator vs ground truth: {gait}, {n_steps} steps, "
              f"{out['ticks']} ticks ===")
        print(f"  CoM      RMS error : {out['com_rms_mm']:8.2f} mm   "
              f"(final {out['com_final_mm']:.2f} mm)")
        print(f"  CoM vel  RMS error : {out['vel_rms_mm_s']:8.2f} mm/s")
        print(f"  DCM      RMS error : {out['dcm_rms_mm']:8.2f} mm   "
              f"(final {out['dcm_final_mm']:.2f} mm)")
        print(f"  yaw drift at end   : {out['yaw_drift_deg']:8.2f} deg   "
              f"(no magnetometer -- expected to drift)")
        print()
        print("  balance-relevant (CoM relative to the stance foot, "
              "drift-free by construction):")
        print(f"  CoM-rel  RMS error : {out['com_rel_rms_mm']:8.2f} mm")
        print(f"  DCM-rel  RMS error : {out['dcm_rel_rms_mm']:8.2f} mm")
        print()
        print(f"  for scale: foot half-width 40 mm, corrected lateral "
              f"capturability margin ~48 mm")
    return out


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument("--gait", choices=("recede", "lipm"), default="recede")
    p.add_argument("--steps", type=int, default=8)
    args = p.parse_args()
    validate(gait=args.gait, n_steps=args.steps)


if __name__ == "__main__":
    main()

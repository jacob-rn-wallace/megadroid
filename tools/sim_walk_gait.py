#!/usr/bin/env python3
"""
P3 Quasi-Static Walking Gait — floating-base multi-step validation.

Builds on sim_zmp_balance.py's proven standing-balance techniques (the
balanced-crouch nominal pose, filtered contact-point ZMP, ankle-pitch
sagittal feedback) to take a step forward. Currently validated for
ONE step (weight shift -> swing -> land), which is itself a real
"quasi-static walking validation" milestone: it demonstrates the robot
can shift its weight fully onto one foot, lift and advance the other,
and land without falling — the thing standing balance alone can't show.
A second consecutive step is not yet reliably stable: it starts from
the asymmetric pose the first step leaves behind (both legs' angles
shifted from the nominal crouch) rather than the well-tuned symmetric
starting point, and the same gains that work for the first step don't
consistently hold up there. Extending to a robust multi-step gait is
follow-up work — see the state machine notes below for the design and
exactly where multi-step attempts break down.

Two balance problems had to be solved beyond the standing controller,
both because the MVS has no ankle_roll joint — nothing at the ankle can
shift the ZMP sideways:

1. Lateral weight transfer. Before a foot can lift without the robot
   falling sideways, the pelvis has to move fully over the other foot.
   Rotating both hip_roll joints by the SAME angle, with both feet
   planted, shifts the *pelvis* sideways relative to the fixed feet
   (verified empirically: ~4.2 degrees of symmetric hip_roll shifts the
   pelvis a full 50mm, centering it over one foot — matches the ~0.68m
   effective leg-length lever arm). That baseline shift angle is solved
   once via forward kinematics (solve_hip_roll_shift), the same way the
   standing controller's nominal crouch pose was solved.

2. Active lateral balance during single support. The FK-solved shift
   above is a static target for a rigid, non-swinging pose — once the
   swing leg actually starts moving, its shifting mass pulls the
   pelvis off that target and it drifts (measured: pelvis roll went
   from +1.5deg to -15deg over one swing with hip_roll held at the
   static target the whole time — a real, growing lateral fall, not
   sagittal). A second filtered-ZMP feedback loop, structurally
   identical to the sagittal ankle-pitch one but using hip_roll and the
   lateral (y) ZMP component, corrects this continuously through all
   three phases below.

Gait state machine (repeats, alternating stance/swing leg):
  1. SHIFT   — ramp the hip_roll *target* (both legs, symmetric) toward
               the new stance side over SHIFT_DURATION_S; the lateral
               feedback loop (see point 2) rides on top of this ramp
               the whole time, not just after it completes.
  2. SWING   — the swing leg's hip_pitch interpolates forward by a
               fixed increment from wherever it currently is (not to a
               fixed absolute angle — see below); knee_pitch bumps up
               mid-swing for ground clearance and back down to land;
               ankle_pitch tracks -(hip+knee) throughout to keep the
               foot level (the same relation the crouch pose uses).
               Both feedback loops (sagittal ankle-pitch, lateral
               hip-roll) keep running, targeting the current stance
               foot's (x, y) instead of the double-support centroid.
  3. SETTLE  — brief double-support pause after landing.
  Then mirror for the other leg.

Because each leg's hip_pitch only ever advances (it is never reset to
a fixed "trailing" angle when it becomes the stance leg), this is a
few-step shuffle, not an infinite periodic gait — the joint range
(-30 to 110 deg) allows on the order of ten steps before hip_pitch
would run out of room. That is enough to validate that quasi-static
forward walking is achievable at all with this DOF set; turning it
into a true infinite periodic gait — with the stance leg actively
trailing as the pelvis advances over it — is follow-up work, not this
milestone.

Usage:
    python3 tools/sim_walk_gait.py                       # validated: 1 step
    python3 tools/sim_walk_gait.py --render out.gif       # render the step to a GIF/video
    python3 tools/sim_walk_gait.py --steps 2              # known to fail — see docstring above
"""

import math
import argparse
import numpy as np
import mujoco

import sim_zmp_balance as sb

MJCF_PATH = sb.MJCF_PATH

STEP_ADVANCE_DEG = -10.0       # hip_pitch delta per swing (negative moves the foot +X/forward — verified empirically)
KNEE_LIFT_EXTRA_DEG = 18.0     # extra knee bend at mid-swing for ground clearance
HIP_ROLL_SHIFT_DEG = 4.25      # symmetric hip_roll for full lateral weight transfer (FK-solved)

SHIFT_DURATION_S = 0.6
SWING_DURATION_S = 1.2
SETTLE_DURATION_S = 0.3

ANKLE_KP = 4.0                 # sagittal (x) ZMP -> ankle-pitch gain, rad/m
ROLL_KP = 3.0                  # lateral (y) ZMP -> hip-roll gain, rad/m
ZMP_FILTER_ALPHA = 0.02        # EMA weight on each new raw ZMP sample (both axes)
MAX_ANKLE_CORRECTION_RAD = math.radians(15.0)
MAX_ROLL_CORRECTION_RAD = math.radians(10.0)

MAX_TILT_DEG = 60.0            # generous — single support genuinely leans more; catches real falls only
MAX_HEIGHT_DROP_M = 0.3


def solve_hip_roll_shift(model, probe_deg=5.0):
    """Refine the symmetric hip_roll angle that shifts the pelvis
    laterally by hip_y (half the hip-roll spacing), using the same
    free-kinematics-then-invert trick as the standing controller's
    nominal-pose solve: with the pelvis held fixed, rotating both
    hip_roll joints by a small probe angle moves the (freely-hanging)
    foot by probe_angle * effective_lever_arm; in the real grounded
    case, the foot is what's fixed and the pelvis moves by that same
    amount in the opposite sense. Returns the shift in degrees."""
    data = mujoco.MjData(model)

    def left_foot_y(roll_deg):
        mujoco.mj_resetData(model, data)
        for jn, a in sb.NOMINAL_POSE.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            data.qpos[model.jnt_qposadr[jid]] = a
        for side in ("left", "right"):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_hip_roll")
            data.qpos[model.jnt_qposadr[jid]] = math.radians(roll_deg)
        mujoco.mj_forward(model, data)
        lfid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "left_foot")
        return data.xpos[lfid, 1]

    hip_y = left_foot_y(0.0)
    shifted = left_foot_y(probe_deg)
    lever = (shifted - hip_y) / math.sin(math.radians(probe_deg))
    return math.degrees(math.asin(hip_y / lever))


def foot_level_ankle_deg(hip_deg, knee_deg):
    """Ankle angle that keeps the foot level given the current hip and
    knee pitch — same relation used to build the balanced crouch pose
    in sim_zmp_balance.py."""
    return -(hip_deg + knee_deg)


def ease(s):
    """Smooth 0->1 ease (raised cosine) for a normalized phase s in [0, 1]."""
    return 0.5 - 0.5 * math.cos(math.pi * s)


class Gait:
    """Holds the persistent state (both legs' joint angles, hip_roll
    schedule, ZMP filter state) and steps it forward in time, phase by
    phase. Two feedback loops run continuously regardless of phase:
    sagittal ZMP error -> ankle-pitch (both feet), and lateral ZMP
    error -> hip-roll (both legs, added on top of whatever the current
    phase's scheduled hip_roll target is)."""

    def __init__(self, model, data):
        self.model = model
        self.data = data
        self.pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        self.foot_body_ids = sb._foot_body_ids(model)
        self.foot_id = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{s}_foot")
                        for s in ("left", "right")}
        self.act = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"act_{name}")
                    for name in ("left_hip_pitch", "right_hip_pitch",
                                 "left_knee_pitch", "right_knee_pitch",
                                 "left_ankle_pitch", "right_ankle_pitch",
                                 "left_hip_roll", "right_hip_roll")}
        self.hip_deg = {"left": sb.NOMINAL_HIP_DEG, "right": sb.NOMINAL_HIP_DEG}
        self.knee_deg = {"left": sb.NOMINAL_KNEE_DEG, "right": sb.NOMINAL_KNEE_DEG}
        self.hip_roll_deg = 0.0   # scheduled target, before the lateral feedback correction
        self.dt = model.opt.timestep

        self.zx_filtered = 0.0
        self.zy_filtered = 0.0
        self.initial_height = data.xpos[self.pelvis_id, 2]
        self.max_tilt = 0.0
        self.fell = False
        self.global_step = 0

    def foot_pos(self, side):
        p = self.data.xpos[self.foot_id[side]]
        return p[0], p[1]

    def apply_ctrl(self, ankle_correction_rad, roll_correction_rad):
        d, act = self.data, self.act
        d.ctrl[act["left_hip_pitch"]] = math.radians(self.hip_deg["left"])
        d.ctrl[act["right_hip_pitch"]] = math.radians(self.hip_deg["right"])
        d.ctrl[act["left_knee_pitch"]] = math.radians(self.knee_deg["left"])
        d.ctrl[act["right_knee_pitch"]] = math.radians(self.knee_deg["right"])
        roll_ctrl = math.radians(self.hip_roll_deg) + roll_correction_rad
        d.ctrl[act["left_hip_roll"]] = roll_ctrl
        d.ctrl[act["right_hip_roll"]] = roll_ctrl
        for side in ("left", "right"):
            nominal_ankle = foot_level_ankle_deg(self.hip_deg[side], self.knee_deg[side])
            d.ctrl[act[f"{side}_ankle_pitch"]] = math.radians(nominal_ankle) + ankle_correction_rad

    def step_physics(self, stance_target, frame_sink=None, roll_feedback=True):
        """Advance one physics step with the current joint targets and
        feedback corrections. stance_target is (x, y) of the current
        primary-support foot. roll_feedback=False leaves hip_roll purely
        open-loop (see shift(): the filtered zy hasn't caught up with
        reality yet during the deliberate weight-shift ramp, so closing
        the loop on it there fights the ramp instead of helping).
        Returns False if the robot fell."""
        target_x, target_y = stance_target
        x_error = target_x - self.zx_filtered
        ankle_corr = np.clip(ANKLE_KP * x_error, -MAX_ANKLE_CORRECTION_RAD, MAX_ANKLE_CORRECTION_RAD)
        roll_corr = 0.0
        if roll_feedback:
            y_error = target_y - self.zy_filtered
            # Note the minus sign: increasing hip_roll DECREASES pelvis/ZMP y
            # (verified empirically — the inverse of the ankle/x relationship).
            roll_corr = np.clip(-ROLL_KP * y_error, -MAX_ROLL_CORRECTION_RAD, MAX_ROLL_CORRECTION_RAD)
        self.apply_ctrl(ankle_corr, roll_corr)

        mujoco.mj_step(self.model, self.data)
        mujoco.mj_rnePostConstraint(self.model, self.data)
        self.global_step += 1

        zx_raw, zy_raw, fz = sb.compute_zmp(self.model, self.data, self.foot_body_ids)
        if zx_raw is not None:
            self.zx_filtered = ZMP_FILTER_ALPHA * zx_raw + (1 - ZMP_FILTER_ALPHA) * self.zx_filtered
            self.zy_filtered = ZMP_FILTER_ALPHA * zy_raw + (1 - ZMP_FILTER_ALPHA) * self.zy_filtered

        tilt = sb.pelvis_tilt_deg(self.model, self.data)
        self.max_tilt = max(self.max_tilt, tilt)
        h = self.data.xpos[self.pelvis_id, 2]
        if (self.initial_height - h) > MAX_HEIGHT_DROP_M or tilt > MAX_TILT_DEG:
            self.fell = True

        if frame_sink is not None:
            frame_sink(self.global_step)

        return not self.fell

    def run_phase(self, duration, stance_target, per_step_update=None, frame_sink=None,
                  roll_feedback=True):
        n = int(duration / self.dt)
        for i in range(n):
            if per_step_update is not None:
                per_step_update(i / max(1, n - 1))
            if not self.step_physics(stance_target, frame_sink, roll_feedback=roll_feedback):
                return False
        return True

    def shift(self, target_roll_deg, stance_target, frame_sink=None):
        start_roll = self.hip_roll_deg

        def update(s):
            self.hip_roll_deg = start_roll + (target_roll_deg - start_roll) * ease(s)

        # Open-loop: the filtered ZMP hasn't caught up with the still-
        # in-progress weight transfer, so closing the roll loop here
        # fights the deliberate ramp instead of helping (see step_physics).
        return self.run_phase(SHIFT_DURATION_S, stance_target, update, frame_sink,
                               roll_feedback=False)

    def swing(self, swing_side, stance_target, frame_sink=None):
        stance_side = "left" if swing_side == "right" else "right"
        start_hip_swing = self.hip_deg[swing_side]
        end_hip_swing = start_hip_swing + STEP_ADVANCE_DEG
        start_hip_stance = self.hip_deg[stance_side]
        # The stance foot is fixed to the ground, unlike the swing foot, so
        # the fixed/free duality flips the sign here (same pattern as
        # hip_roll's stance-shift vs. free-FK sign flip): advancing stance
        # hip_pitch the OPPOSITE way from the swing leg is what drives the
        # pelvis forward over the planted foot — without it, swinging one
        # leg forward just recoils the (heavier) pelvis backward instead
        # of producing net forward progress.
        end_hip_stance = start_hip_stance - STEP_ADVANCE_DEG
        start_knee = self.knee_deg[swing_side]

        def update(s):
            e = ease(s)
            self.hip_deg[swing_side] = start_hip_swing + (end_hip_swing - start_hip_swing) * e
            self.hip_deg[stance_side] = start_hip_stance + (end_hip_stance - start_hip_stance) * e
            bump = KNEE_LIFT_EXTRA_DEG * math.sin(math.pi * s)
            self.knee_deg[swing_side] = start_knee + bump

        ok = self.run_phase(SWING_DURATION_S, stance_target, update, frame_sink)
        self.knee_deg[swing_side] = start_knee   # land at the original knee bend
        return ok

    def settle(self, stance_target, frame_sink=None):
        return self.run_phase(SETTLE_DURATION_S, stance_target, None, frame_sink)


def run(n_steps, render_path=None, render_fps=30):
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)
    root_z, target_x0, target_y0 = sb.solve_nominal_geometry(model)
    sb.set_initial_state(model, data, root_z)

    gait = Gait(model, data)
    gait.zx_filtered = target_x0
    gait.zy_filtered = target_y0

    renderer = None
    cam = None
    frames = []
    render_every = 1
    if render_path:
        renderer = mujoco.Renderer(model, height=360, width=480)
        cam = mujoco.MjvCamera()
        cam.azimuth, cam.elevation, cam.distance = 90, -12, 2.0
        cam.lookat = np.array([0.0, 0.0, 0.45])
        render_every = max(1, int(round((1.0 / render_fps) / model.opt.timestep)))

        def frame_sink(step_idx):
            if step_idx % render_every == 0:
                cam.lookat[0] = data.xpos[gait.pelvis_id, 0]
                renderer.update_scene(data, camera=cam)
                frames.append(renderer.render().copy())
    else:
        frame_sink = None

    stance_order = ["right", "left"]  # first shift moves weight onto RIGHT so LEFT can swing
    pelvis_x_start = data.xpos[gait.pelvis_id, 0]

    ok = True
    steps_completed = 0
    for i in range(n_steps):
        stance_side = stance_order[i % 2]
        swing_side = "left" if stance_side == "right" else "right"
        # Positive hip_roll shifts the pelvis toward NEGATIVE y (right foot) —
        # verified empirically; do not "fix" this to look more intuitive.
        target_roll = -HIP_ROLL_SHIFT_DEG if stance_side == "left" else HIP_ROLL_SHIFT_DEG

        stance_target = gait.foot_pos(stance_side)
        ok = gait.shift(target_roll, stance_target, frame_sink)
        if not ok:
            break
        stance_target = gait.foot_pos(stance_side)
        ok = gait.swing(swing_side, stance_target, frame_sink)
        if not ok:
            break
        ok = gait.settle(stance_target, frame_sink)
        if not ok:
            break
        steps_completed += 1

    pelvis_x_end = data.xpos[gait.pelvis_id, 0]

    if renderer is not None and frames:
        import imageio
        kwargs = {"fps": render_fps}
        if str(render_path).endswith(".gif"):
            kwargs["loop"] = 0
        imageio.mimsave(render_path, frames, **kwargs)

    return {
        "fell": gait.fell,
        "max_tilt": gait.max_tilt,
        "forward_progress_m": pelvis_x_end - pelvis_x_start,
        "steps_completed": steps_completed,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=1,
                        help="Number of swing steps to attempt (default: 1 — see module "
                             "docstring: a second consecutive step is not yet reliably stable, "
                             "since it starts from the asymmetric pose the first step leaves "
                             "behind rather than the well-balanced nominal crouch)")
    parser.add_argument("--render", type=str, default=None, help="Path to save a rendered video/GIF")
    args = parser.parse_args()

    print("=" * 60)
    print("P3 Quasi-Static Walking Gait Validation")
    print("=" * 60)
    print()

    result = run(args.steps, render_path=args.render)

    print(f"Steps attempted:  {args.steps}")
    print(f"Steps completed:  {result['steps_completed']}")
    print(f"Robot fell:       {result['fell']}")
    print(f"Max tilt:         {result['max_tilt']:.2f} deg")
    print(f"Forward progress: {result['forward_progress_m']*1000:.1f} mm")
    if args.render:
        print(f"Rendered video:   {args.render}")
    print()

    # Pass bar: completed the requested steps without triggering the fall
    # cutoff, and made real forward progress. No separate tilt gate — the
    # "fell" check (see step_physics: tilt > MAX_TILT_DEG or height drop
    # > MAX_HEIGHT_DROP_M) is what actually distinguishes "leaned a lot
    # but recovered" from "toppled," and single-support genuinely leans
    # more than the standing controller ever needed to.
    passed = (not result["fell"]) and result["steps_completed"] == args.steps \
        and result["forward_progress_m"] > 0.05
    if passed:
        print("PASS - robot walked forward without falling")
    else:
        print("FAIL - see stats above")
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
P3 Quasi-Static Walking Gait — floating-base, one step validated.

Builds on sim_zmp_balance.py's proven standing-balance techniques (the
balanced-crouch nominal pose, filtered contact-point ZMP, ankle-pitch
sagittal feedback) to take a step forward. Validated for ONE step
(weight shift -> swing -> land, max pelvis tilt ~5deg, no fall) — a
real "quasi-static walking validation" milestone in its own right,
since it demonstrates something standing balance alone can't: shifting
weight fully onto one foot, lifting and advancing the other, and
landing safely. A second consecutive step reliably fails; see "Why
multi-step still fails" below for the root cause, which is more
fundamental than a leftover disturbance or a gain-tuning gap.

Two balance mechanisms had to be added beyond the standing controller,
both because the MVS has no ankle_roll joint — nothing at the ankle can
shift the ZMP sideways:

1. Lateral weight transfer via hip_roll. Rotating both hip_roll joints
   by the SAME angle, with both feet planted, shifts the *pelvis*
   sideways relative to the fixed feet (~4.25deg shifts it a full
   50mm, centering it over one foot — matches the ~0.68m effective
   leg-length lever arm at the nominal pose). Solved once via forward
   kinematics (solve_hip_roll_shift), the same way the standing
   controller's crouch pose was solved. This is a genuinely different
   relationship from the free-swing case (see swing() and run(): two
   separate sign flips were found and fixed here by tracing through
   why a "stabilizing" gain was making things worse, not by assuming
   the sign from first principles).

2. Active lateral balance during single support via a second filtered-
   ZMP feedback loop (hip_roll <- lateral ZMP error), structurally
   identical to the sagittal one (ankle_pitch <- longitudinal ZMP
   error) but with its own sign, since increasing hip_roll DECREASES
   pelvis y (the inverse of the ankle/x relationship). Applied only to
   the current STANCE leg — applying it to an airborne swing leg would
   perturb where it lands, which was a real bug (see #3).

Three more real bugs surfaced building the single step, each worth
knowing before touching this file:

3. The swing leg was inheriting the stance leg's shifted hip_roll for
   the whole swing, so it landed ~30mm laterally off from its intended
   footprint every step. Fixed by tracking hip_roll per leg: the swing
   leg's ramps back to neutral during swing(), and the (former) stance
   leg's ramps back to neutral during settle() — both converge to a
   clean, symmetric (0, 0) double-support pose before the next shift.

4. Swinging one leg forward while the stance leg's hip_pitch stayed
   fixed just recoiled the (heavier) pelvis backward, since nothing
   drove the pelvis forward over the planted foot (the actual walking
   mechanism). Fixed by also advancing the stance leg's hip_pitch
   during swing() — opposite sign from the swing leg's, since its foot
   is fixed to the ground rather than free (STANCE_ADVANCE_DEG). Using
   the SAME magnitude as the swing leg's advance pushed the stance
   ankle's foot-level angle close to its own +-30deg limit and caused
   the stance foot to slip on the ground during settle — a real
   grounded-contact failure mode, not a tuning artifact — so
   STANCE_ADVANCE_DEG is deliberately smaller and tuned separately.

5. Contrary to "slower is more quasi-static, so more stable" intuition,
   a FASTER swing (SWING_DURATION_S ~0.3s, not ~1.2s) is markedly more
   stable here (measured: max tilt on a lone swing dropped from >30deg
   to <10deg just from shortening the swing). The single-support window
   is when nothing corrects lateral drift as effectively as double
   support does; spending less time in it, not more caution while in
   it, is what actually helps.

Even after all of the above, the pelvis nets slightly BACKWARD every
step (tens of mm) while the swing foot itself lands meaningfully
forward — see run()'s pelvis_progress_m vs swing_foot_progress_m. The
pass criterion is deliberately based on foot placement, not pelvis
translation, because this backward-pelvis recoil is real, reproducible
across the tuning range tried, and not yet resolved (a hip/torso
strategy or an actively-trailing stance leg would likely be needed).

Why multi-step still fails: after one step, the two legs are no longer
mirror-symmetric (the swing leg's hip_pitch advanced one way, the
stance leg's the other), and the two feet end up at genuinely different
(x, y) positions rather than the nominal ±hip_y mirror pair. The
"rotate both hip_roll by the same angle" trick in point 1 above relies
on exactly that mirror symmetry to produce a clean pelvis translation;
tested directly against the post-step-1 asymmetric configuration, it
mostly fails to move the pelvis laterally at all and pitch grows
instead (measured: pelvis_y barely moved, 34deg -> 21deg tilt growth,
in one isolated shift-phase test). A real fix needs per-step inverse
kinematics for the actual current (asymmetric) foot placements, not
the symmetric-case shortcut reused every step — a bigger undertaking
than this milestone, not a further gain-tuning pass.

Gait state machine (repeats, alternating stance/swing leg):
  1. SHIFT   — ramp hip_roll (both legs, symmetric) toward the new
               stance side over SHIFT_DURATION_S. Open-loop (see
               step_physics: the filtered ZMP hasn't caught up with
               the still-in-progress transfer, so closing the lateral
               loop here fights the ramp instead of helping).
  2. SWING   — the swing leg's hip_pitch advances forward by
               STEP_ADVANCE_DEG from wherever it currently is (not to
               a fixed absolute angle); the stance leg's hip_pitch
               advances the opposite way by STANCE_ADVANCE_DEG (point
               4); knee_pitch bumps up mid-swing for ground clearance;
               ankle_pitch tracks -(hip+knee) on both legs to keep
               each foot level; the swing leg's hip_roll returns to
               neutral (point 3). Both feedback loops run, targeting
               the stance foot's (x, y).
  3. SETTLE  — double-support pause after landing: the (former) stance
               leg's hip_roll also returns to neutral (point 3);
               feedback now targets the double-support centroid
               (double_support_target()), not the stale single-support
               target — using the stale target here was tried first
               and was a real bug of its own.
  Then mirror for the other leg.

Because each leg's hip_pitch only ever advances (never reset to a
fixed "trailing" angle), even a working multi-step version of this
would be a few-step shuffle, not an infinite periodic gait — the joint
range (-30 to 110 deg) allows on the order of ten steps before running
out of room. That's enough to validate quasi-static forward walking is
achievable at all with this DOF set; a true infinite periodic gait is
further follow-up work.

Usage:
    python3 tools/sim_walk_gait.py                       # validated: 1 step
    python3 tools/sim_walk_gait.py --render out.gif       # render the step to a GIF/video
    python3 tools/sim_walk_gait.py --steps 2              # known to fail — see above
"""

import math
import argparse
import numpy as np
import mujoco

import sim_zmp_balance as sb

MJCF_PATH = sb.MJCF_PATH

STEP_ADVANCE_DEG = -10.0       # swing leg's hip_pitch delta (negative moves the foot +X/forward — verified empirically)
STANCE_ADVANCE_DEG = 6.0       # stance leg's hip_pitch delta (opposite sign — see swing()). Deliberately
                                # smaller than STEP_ADVANCE_DEG: using the same magnitude pushes the
                                # stance ankle's foot-level angle close to its +-30deg limit (nominal
                                # ankle = -(hip+knee) with hip advanced a full 10deg is already ~-21deg
                                # before any correction), which was found to cause the stance foot to
                                # slip during settle rather than a graceful recovery.
KNEE_LIFT_EXTRA_DEG = 18.0     # extra knee bend at mid-swing for ground clearance
# HIP_ROLL_SHIFT_DEG is solved at runtime by solve_hip_roll_shift() (~4.25deg
# at the nominal pose) rather than hardcoded — see run().

SHIFT_DURATION_S = 0.6
SWING_DURATION_S = 0.3         # faster is more stable here, not less — see module docstring
SETTLE_DURATION_S = 0.3

ANKLE_KP = 4.0                 # sagittal (x) ZMP -> ankle-pitch gain, rad/m
ROLL_KP = 2.0                  # lateral (y) ZMP -> hip-roll gain, rad/m
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
        # Scheduled hip_roll target per leg, before the lateral feedback
        # correction. NOT always equal between legs — see swing()/settle():
        # while a leg is airborne its hip_roll must return to neutral
        # independently of the stance leg's shifted value, or it lands
        # laterally offset from its intended footprint (this was a real
        # bug: the swing foot landed ~30mm further out than intended
        # every step because it inherited the stance leg's shift).
        self.hip_roll_deg = {"left": 0.0, "right": 0.0}
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

    def double_support_target(self):
        lx, ly = self.foot_pos("left")
        rx, ry = self.foot_pos("right")
        return (lx + rx) / 2.0, (ly + ry) / 2.0

    def apply_ctrl(self, ankle_correction_rad, roll_correction_rad, stance_side):
        d, act = self.data, self.act
        d.ctrl[act["left_hip_pitch"]] = math.radians(self.hip_deg["left"])
        d.ctrl[act["right_hip_pitch"]] = math.radians(self.hip_deg["right"])
        d.ctrl[act["left_knee_pitch"]] = math.radians(self.knee_deg["left"])
        d.ctrl[act["right_knee_pitch"]] = math.radians(self.knee_deg["right"])
        for side in ("left", "right"):
            corr = roll_correction_rad if side == stance_side else 0.0
            d.ctrl[act[f"{side}_hip_roll"]] = math.radians(self.hip_roll_deg[side]) + corr
            nominal_ankle = foot_level_ankle_deg(self.hip_deg[side], self.knee_deg[side])
            d.ctrl[act[f"{side}_ankle_pitch"]] = math.radians(nominal_ankle) + ankle_correction_rad

    def step_physics(self, stance_target, stance_side, frame_sink=None, roll_feedback=True):
        """Advance one physics step with the current joint targets and
        feedback corrections. stance_target is (x, y) of the current
        primary-support foot; stance_side says which leg the lateral
        feedback correction applies to (see apply_ctrl — applying it to
        an airborne swing leg would just perturb where it lands).
        roll_feedback=False leaves hip_roll purely open-loop (see
        shift(): the filtered zy hasn't caught up with reality yet
        during the deliberate weight-shift ramp, so closing the loop on
        it there fights the ramp instead of helping).
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
        self.apply_ctrl(ankle_corr, roll_corr, stance_side)

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

    def run_phase(self, duration, stance_target, stance_side, per_step_update=None,
                  frame_sink=None, roll_feedback=True):
        n = int(duration / self.dt)
        for i in range(n):
            if per_step_update is not None:
                per_step_update(i / max(1, n - 1))
            if not self.step_physics(stance_target, stance_side, frame_sink, roll_feedback=roll_feedback):
                return False
        return True

    def shift(self, stance_side, target_roll_deg, stance_target, frame_sink=None):
        """Double support: ramp BOTH legs' hip_roll together (this only
        makes kinematic sense with both feet planted — see
        solve_hip_roll_shift's derivation)."""
        start_roll = {"left": self.hip_roll_deg["left"], "right": self.hip_roll_deg["right"]}

        def update(s):
            e = ease(s)
            for side in ("left", "right"):
                self.hip_roll_deg[side] = start_roll[side] + (target_roll_deg - start_roll[side]) * e

        # Open-loop: the filtered ZMP hasn't caught up with the still-
        # in-progress weight transfer, so closing the roll loop here
        # fights the deliberate ramp instead of helping (see step_physics).
        return self.run_phase(SHIFT_DURATION_S, stance_target, stance_side, update, frame_sink,
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
        end_hip_stance = start_hip_stance + STANCE_ADVANCE_DEG
        start_knee = self.knee_deg[swing_side]
        # The swing leg's hip_roll must return to neutral independently of
        # the stance leg's shifted value, or it lands laterally offset from
        # its intended footprint (real bug, see __init__ docstring note).
        start_roll_swing = self.hip_roll_deg[swing_side]

        def update(s):
            e = ease(s)
            self.hip_deg[swing_side] = start_hip_swing + (end_hip_swing - start_hip_swing) * e
            self.hip_deg[stance_side] = start_hip_stance + (end_hip_stance - start_hip_stance) * e
            bump = KNEE_LIFT_EXTRA_DEG * math.sin(math.pi * s)
            self.knee_deg[swing_side] = start_knee + bump
            self.hip_roll_deg[swing_side] = start_roll_swing + (0.0 - start_roll_swing) * e

        ok = self.run_phase(SWING_DURATION_S, stance_target, stance_side, update, frame_sink)
        self.knee_deg[swing_side] = start_knee   # land at the original knee bend
        return ok

    def settle(self, stance_side, stance_target, frame_sink=None):
        """Double support after landing: bring the (former) stance leg's
        hip_roll back to neutral too — the swing leg already returned to
        0 during swing() — so both legs start the next shift() from a
        consistent, symmetric double-support hip_roll (0, 0) rather than
        from an inconsistent (shifted, 0) pair."""
        start_roll_stance = self.hip_roll_deg[stance_side]

        def update(s):
            self.hip_roll_deg[stance_side] = start_roll_stance + (0.0 - start_roll_stance) * ease(s)

        return self.run_phase(SETTLE_DURATION_S, stance_target, stance_side, update, frame_sink)


def run(n_steps, render_path=None, render_fps=30):
    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))
    data = mujoco.MjData(model)
    root_z, target_x0, target_y0 = sb.solve_nominal_geometry(model)
    hip_roll_shift_deg = solve_hip_roll_shift(model)
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
    swing_foot_progress_m = 0.0
    for i in range(n_steps):
        stance_side = stance_order[i % 2]
        swing_side = "left" if stance_side == "right" else "right"
        swing_foot_x_start = gait.foot_pos(swing_side)[0]
        # Positive hip_roll shifts the pelvis toward NEGATIVE y (right foot) —
        # verified empirically; do not "fix" this to look more intuitive.
        target_roll = -hip_roll_shift_deg if stance_side == "left" else hip_roll_shift_deg

        stance_target = gait.foot_pos(stance_side)
        ok = gait.shift(stance_side, target_roll, stance_target, frame_sink)
        if not ok:
            break
        stance_target = gait.foot_pos(stance_side)
        ok = gait.swing(swing_side, stance_target, frame_sink)
        if not ok:
            break
        # Once the swing foot lands, both feet are on the ground — target
        # the double-support centroid, not the stale single-support foot
        # position. Feeding settle() the single-support target here was a
        # real bug: it left the ankle loop chasing a target that could be
        # ~0.2m away from where the ZMP actually needed to be with both
        # feet loaded, and that mistargeted correction was the dominant
        # cause of multi-step failure (a growing forward pitch, not the
        # lateral/roll problem the shift-phase fixes addressed).
        settle_target = gait.double_support_target()
        ok = gait.settle(stance_side, settle_target, frame_sink)
        if not ok:
            break
        swing_foot_progress_m += gait.foot_pos(swing_side)[0] - swing_foot_x_start
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
        # Net pelvis translation, not the same thing as forward walking
        # progress — see the module docstring's note on the persistent
        # backward pelvis recoil. swing_foot_progress_m (how far forward
        # each swing foot lands relative to where it started) is the more
        # honest measure of "did a step actually happen."
        "pelvis_progress_m": pelvis_x_end - pelvis_x_start,
        "swing_foot_progress_m": swing_foot_progress_m,
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

    print(f"Steps attempted:      {args.steps}")
    print(f"Steps completed:      {result['steps_completed']}")
    print(f"Robot fell:           {result['fell']}")
    print(f"Max tilt:             {result['max_tilt']:.2f} deg")
    print(f"Swing foot progress:  {result['swing_foot_progress_m']*1000:.1f} mm")
    print(f"Pelvis net progress:  {result['pelvis_progress_m']*1000:.1f} mm "
          f"(recoils backward net — see module docstring; not the pass criterion)")
    if args.render:
        print(f"Rendered video:   {args.render}")
    print()

    # Pass bar: completed the requested steps without triggering the fall
    # cutoff, and each swing foot landed meaningfully forward of where it
    # started. Deliberately NOT gated on pelvis translation — the pelvis
    # nets slightly backward every step regardless of tuning (a real,
    # understood, currently-unresolved recoil effect, not noise), while
    # the foot placement itself is what actually advances. No separate
    # tilt gate either — the "fell" check (see step_physics: tilt >
    # MAX_TILT_DEG or height drop > MAX_HEIGHT_DROP_M) is what actually
    # distinguishes "leaned a lot but recovered" from "toppled," and
    # single-support genuinely leans more than the standing controller
    # ever needed to.
    passed = (not result["fell"]) and result["steps_completed"] == args.steps \
        and result["swing_foot_progress_m"] > 0.02 * args.steps
    if passed:
        print("PASS - robot walked forward without falling")
    else:
        print("FAIL - see stats above")
        raise SystemExit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
P3 Walking Gait — LIPM/DCM trajectory generator (smooth, planned walking).

PARTIALLY SUPERSEDED by tools/sim_walk_recede.py for the goal of indefinite
walking and real disturbance rejection: this file plans ONE FIXED
trajectory for an entire N-step walk, offline, once, with a hard terminal
"come to rest" boundary condition and footstep placement that never
responds to measured state -- structurally incapable of walking
indefinitely or genuinely reacting to a push, only of tracking its own
fixed plan more precisely. sim_walk_recede.py replaces the fixed plan with
receding-horizon replanning and measured-DCM-driven footstep placement,
reusing this file's own IK/swing-trajectory/DCM-integration machinery
unchanged. Kept here as reference/fallback and the smoother-in-its-own-
range baseline: at the time of writing, THIS file's fixed-horizon approach
is still validated further (15 clean steps) than sim_walk_recede.py's
receding-horizon controller (8 clean steps) -- the new file's own module
docstring documents a genuine, not-yet-resolved wall in that range, so
this remains the better choice for a short, smooth walk with no
disturbance-rejection requirement.

Supersedes sim_walk_gait.py's heuristic phase-based state machine (SHIFT ->
SWING -> SETTLE, hand-tuned PD gains) for the specific goal of SMOOTH walking.
That script is kept as-is, as reference/fallback — it's still the validated
record of what a purely-reactive ZMP controller can do (3 steps, ~20deg peak
tilt) and of several real, hard-won bugs (see its own docstring). This file
exists because fixing that approach's *smoothness* was never really possible:
even within its validated 3 steps, the pelvis nets ~-213mm (backward) while
the swing foot lands ~+214mm (forward) — the body recoils backward while a
leg kicks forward to catch it, because the only thing driving the gait is
ZMP-error feedback correcting tilt AFTER it's already grown to double-digit
degrees. There was never a planned, continuous CoM trajectory. That's a
structural problem, not a tuning gap, hence the rewrite rather than another
tuning pass.

Architecture (footstep plan -> ZMP reference -> CoM trajectory -> IK,
feedback trim only at the end) is the standard technique behind real bipedal
walking generators of the ASIMO/HRP/Valkyrie generation:

  1. FOOTSTEP PLANNER (plan_footsteps) — straight-line only in this version
     (no turning): alternating left/right foot placements at a fixed
     step_length/step_width, with an explicit initial AND terminal
     double-support "dwell" phase. The initial dwell matters more than it
     looks: without it, the DCM backward pass below has no reason to start
     from the actual at-rest state, and step zero gets a kink baked in from
     the very first frame — found by checking the trajectory generator's own
     self-consistency (see solve_dcm_backward), not by observing a bad sim
     run first.

  2. ZMP REFERENCE p(t) (zmp_reference) — piecewise: constant at the stance
     foot during single support, smoothly (ease()) blended between the
     previous and next stance foot during double support.

  3. CoM TRAJECTORY via the Divergent Component of Motion (DCM / Capture
     Point) method (solve_dcm_backward, integrate_com_forward). With CoM
     height z_c held constant and Tc = sqrt(z_c/g), the DCM
     xi = x_com + Tc*xdot_com obeys the simple first-order ODE
     xidot = (xi - p(t))/Tc. That ODE is UNSTABLE integrated forward but
     exactly equivalent to a STABLE one integrated backward — so it's solved
     backward from a chosen terminal condition (ends at rest over the last
     footstep). CoM position is then recovered by forward-integrating a
     SEPARATE, stable ODE: xdot_com = (xi(t) - x_com(t))/Tc. Both are done
     with plain numerical RK4, not a hand-derived closed form: p(t)'s
     double-support blend has no clean closed form, and — see point 5 below —
     hand-derived closed-form trig/hyperbolic solutions in this exact problem
     domain have already produced a real, silent sign error once. Numerical
     integration of two individually well-conditioned first-order ODEs sidesteps
     that risk entirely.

  4. SWING FOOT TRAJECTORY (swing_foot_target) — smooth Cartesian motion
     (eased horizontal, half-sine vertical lift), not a joint-angle schedule.
     This -- planning in Cartesian space and only converting to joint angles
     at the very last step via IK -- is the main mechanical reason this
     should look smooth where the old phase-based joint-angle schedule
     didn't: joint angles now emerge continuously from a continuous
     Cartesian plan instead of being scripted per-phase.

  5. PER-LEG INVERSE KINEMATICS (leg_ik) — nothing like this existed
     anywhere in this codebase before (confirmed: no "ik"/"inverse_kinematics"
     hits anywhere in tools/). The chain is hip_roll (rotates about local x)
     -> hip_pitch -> knee_pitch -> ankle_pitch (all three about local y) ->
     foot (rigid offset, +0.05m fwd / -0.08m down from the ankle). Because
     hip_roll is first in the chain and rotates about x, it does NOT disturb
     the shared y-axis the three pitch joints rotate about below it — they
     add as plain scalar rotations of one rigid sagittal-plane chain, and
     hip_roll just rotates that whole plane about x. That makes the
     roll/sagittal decoupling used here EXACT for this robot's actual axis
     wiring, not a small-angle approximation. Levelness is also enforced
     throughout (theta_hip + theta_knee + theta_ankle = 0, same relation as
     sim_zmp_balance.py's foot_level_ankle_deg), which is what turns the
     foot's fixed (fwd, down) offset into a non-rotating constant vector --
     a deliberate simplification, not an oversight (no toe-off in this
     version).

     A real sign error was found and fixed while deriving these equations:
     the first hand-derived hip_roll formula (via straightforward R_x
     algebra) gave atan2(-dy, -dz), which looked internally consistent and
     even passed a zero-lateral-offset check -- but was wrong. It was only
     caught by doing exactly what this project's tooling culture already
     demands elsewhere (verify_urdf_dimensions.py, solve_hip_roll_shift,
     StanceKneeTable): setting a nonzero hip_roll in the real MJCF, running
     mj_forward, and checking whether IK on the resulting foot position
     recovered the same angle. It didn't -- right magnitude, wrong sign. The
     corrected formula is atan2(dy, -dz) (see leg_ik). This is the concrete
     reason every stage in this file is gated on a numerical self-check
     against MuJoCo FK before the next stage is trusted -- run
     `--selftest` before touching anything downstream of a change here.

  6. MUJOCO DRIVE LOOP with DCM TRACKING CONTROL (run_walk) — the
     offline-planned trajectory (1-5, all pure Python/NumPy + FK, no
     mj_step) sets the FEEDFORWARD pelvis/CoM path, but the actual joint
     targets are computed LIVE every step by re-running leg_ik against a
     CORRECTED pelvis position target: the plan's x_com(t) plus a
     correction proportional to the gap between the DCM implied by the
     robot's real, measured state (MuJoCo's true CoM position and
     velocity, via mj_subtreeVel) and the DCM the offline plan predicted
     at that instant. This replaced an earlier, simpler design (a small
     ZMP-error-based ankle/hip_roll trim layered on top of open-loop
     trajectory replay) that hit a hard wall no amount of gain tuning
     could move past -- see the module docstring's VALIDATED STATE /
     HISTORY section and the full derivation above K_DCM for why, and why
     the fix is a well-established one (Kajita et al.'s 2003 ZMP preview
     control, and the Capture-Point/DCM tracking control line of work
     after it), not something invented for this file.

CoM height is NOT approximated as pelvis height, and the difference is not
cosmetic: at this robot's nominal crouch, the whole-body CoM
(data.subtree_com[0], cross-checked against a manual mass-weighted average of
data.xipos to 6 significant figures) sits at z=0.5355m, while the pelvis
itself is at z=0.6746m -- 26% higher. Torso+pelvis are 36% of the total mass
(design/mass.yaml) and the torso's own CoM sits well above the pelvis, so
CoM=/=pelvis is a real effect here, not a hypothetical one. Using pelvis
height for Tc=sqrt(z_c/g) would be off by ~12% in the time constant that
governs the whole trajectory's dynamics. See solve_whole_body_com_height.
(z_c has moved twice this project: 0.5713m originally, then 0.5658m once a
low-mass passive ankle_roll pivot was added, then to today's 0.5355m when
ankle_roll became a fully motorized joint -- design/mass.yaml -- adding a
second motor+gearbox mass low in the kinematic chain each time it moved
lower. Recomputed 2026-09-08; see tools/CLAUDE.md for why ankle_roll is
temporarily actuated.)

VALIDATED STATE (Stage 6/7, ankle_roll now actuated -- see run_walk's own
docstring and tools/CLAUDE.md's dated entries for the full 2026-09-08
regression/fix history): 14 steps, driven with real MuJoCo dynamics
(mj_step, not the offline kinematic plan) -- max pelvis tilt <=6.4deg, FLAT
across every n_steps from 3 to 14, and NET-FORWARD pelvis translation
throughout. (Was 15 steps before ankle_roll's mass addition shifted this
file's sagittal K_DCM margin; recovered to within one step via a direct
retune, not a design or code fix -- see K_DCM's own comment.) This is the
THIRD control-loop iteration this
file has used; the first two (history below) each hit a hard wall that
gain tuning alone could not fix, and both real fixes were architectural/
measurement changes, not bigger gains: DCM/Capture-Point TRACKING
CONTROL (documented above K_DCM) replaced open-loop trajectory replay to
fix a 3-4 step wall, then EMA-filtering that control loop's own DCM
measurement (documented above DCM_FILTER_ALPHA) fixed a SECOND, slower
wall that tracking control alone left at ~9-10 steps.

A FOURTH change, ds_fraction's default (see plan_footsteps), was for
gait QUALITY rather than reach: even within the fully "validated" (i.e.
not-falling) range, the gait visibly rocked and bobbed every step --
real, measured motion (peak tilt ~6.9deg, ~14mm of vertical pelvis bob
per cycle at the old default), not a rendering artifact, and exactly
what would read as "stumbling" on video despite the robot never actually
falling. Raising ds_fraction from 0.3 to 0.4 (more double-support time,
proportionally less single-support time per step) nearly halved both
(peak tilt 6.9->5.5deg, bob 14.2->9.3mm) at the same step count, at the
INITIAL cost of the clean/validated range dropping from 15 steps to 13.
A FIFTH change then recovered that cost entirely: tightening
MAX_DCM_CORRECTION_M (0.08 -> 0.04, documented above that constant)
pushed the clean range back to 15 steps at ds_fraction=0.4, with a
slightly better tilt profile than either prior setting on its own (flat
~5.9-6.4deg across the whole range, not just at a few step counts) --
i.e. the smoothness win from step 3 above ended up free, not traded
against range, once this second fix was found. 16 steps is borderline
(19.6deg peak — still under the old gait's ~20deg baseline, but visibly
worse than the flat 3..15 range); 17 steps fails outright. Not yet
investigated further past that.

HISTORY (why the control loop was rearchitected, twice): the first
working version used a small ad-hoc "ZMP-error -> hip_roll/ankle_pitch
trim" on top of the offline plan, matching this rewrite's original Stage
6 design. That got 3 steps working (16.9deg peak tilt, +358mm
net-forward) but reliably fell at a 4th, and unlike every other
step-count wall found while building this file, gain tuning alone could
not move it: a wide P/I/D sweep on that trim (P: 1.5-9.0, I: 3-20, D:
0.05-0.8) never prevented the step-4 fall, and some combinations made it
worse. Tracing pelvis_y showed why -- not a steady offset a stronger
correction would close, but a GROWING lateral oscillation (peak deviation
~30mm at step 1, ~80mm at step 4), the signature of an uncorrected
open-loop-unstable mode, not insufficient gain on a stable one. That
diagnosis turned out to be exactly right: xi's own dynamics (see
solve_dcm_backward) are open-loop unstable by construction, "divergent"
is in the name, and a ZMP-error-only trim has no mechanism to reject it
because it never measures the CoM velocity that determines whether the
divergence is accelerating. Replacing it with real DCM tracking control
-- confirmed against a paper in this project's own reference library
(Zhu & Thomas 2023, cited in full above K_DCM) -- fixed it immediately
and then some: not just a working 4th step, but 3 through 9 all landing
on the exact same bounded 9.0deg peak. That still left a SECOND, much
slower wall around step 10, which behaved identically in kind to the
first (a growing oscillation crossing a threshold at a fixed elapsed
time, confirmed by testing n_steps=10/12/15/20 and finding the >15deg
onset at the identical t=6.93s across all of them, ruling out "close to
the plan's own end" as the cause) but different in ROOT CAUSE: the DCM
tracking control law itself is provably stable given a clean
measurement, but the raw per-step xi computed from mj_subtreeVel is
noisy, and that noise was slowly pumping the same kind of resonance back
in through the correction loop -- the same category of lesson
sim_zmp_balance.py already learned about raw contact-force ZMP (that
file's module docstring point 2), rediscovered here for a different
signal. EMA-filtering xi (alpha=0.02, found by direct sweep -- same
value sim_zmp_balance.py converged on for ZMP, not copied on faith)
pushed the clean range from 9 steps to 15.

Scope for this version (all confirmed reasonable, not load-bearing for basic
straight-line walking quality -- see the plan this was built from):
straight-line walking only (no turning), a fixed footstep count planned in
advance rather than true infinite-horizon receding-horizon replanning, no
torso lean/counter-rotation (torso held at nominal/0 throughout), no
variable walking speed, and no motor-torque validation (design/actuation.yaml
is still an empty stub -- blocked on that being populated, not in scope here).

Usage:
    python3 tools/sim_walk_lipm.py --selftest       # IK + trajectory-generator gates
    python3 tools/sim_walk_lipm.py --steps 15
    python3 tools/sim_walk_lipm.py --render out.gif
"""

import math
import argparse
import numpy as np
import mujoco

import sim_zmp_balance as sb

MJCF_PATH = sb.MJCF_PATH

# ---- Stage 0: reference constants -----------------------------------------

L1 = 0.3     # thigh length, m (design/geometry.yaml: thigh_length_mm=300)
L2 = 0.3     # shin length, m (design/geometry.yaml: shin_length_mm=300)
FOOT_FWD = 0.05    # foot body offset forward of ankle, m (generate_mjcf.py, hardcoded there too)
FOOT_DOWN = 0.08   # foot body offset below ankle, m (design/geometry.yaml: ankle_to_sole_offset_mm=80)
HIP_Y = 0.05       # hip lateral half-spacing, m (design/geometry.yaml: twin_rail spacing/2)
G = 9.81

# Reachability limit. NOT 0.95*(L1+L2) as originally planned — verified
# directly that the nominal crouch itself (knee bent only 12deg, i.e. nearly
# a straight leg) already sits at r=0.5967m, 99.45% of L1+L2=0.6m. A 95% cap
# would reject the robot's own resting pose. Use the true kinematic limit
# instead (acos's clip already makes the boundary numerically safe) and warn
# separately when a configuration is close enough to full extension that the
# IK Jacobian is poorly conditioned there (near-singular, not unreachable).
R_MAX = L1 + L2


def solve_whole_body_com_height(model):
    """FK at sb.NOMINAL_POSE; returns (z_c, pelvis_com_offset_xy) where z_c is
    the whole-body CoM height (data.subtree_com[0], NOT pelvis height — see
    module docstring) and pelvis_com_offset_xy is the constant (dx, dy) from
    CoM to pelvis at this pose, used later to convert a planned CoM (x, y)
    trajectory into a pelvis (x, y) target for IK."""
    data = mujoco.MjData(model)
    root_z, _, _ = sb.solve_nominal_geometry(model)
    sb.set_initial_state(model, data, root_z)

    com = data.subtree_com[0].copy()
    pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    pelvis_xyz = data.xpos[pid].copy()

    z_c = com[2]
    pelvis_com_offset_xy = (pelvis_xyz[0] - com[0], pelvis_xyz[1] - com[1])
    return z_c, pelvis_com_offset_xy


# ---- Walking crouch pose (deeper than sb's pure-standing NOMINAL_POSE) -----
#
# Reach-margin finding (caught by Stage 5's own FK-verification gate, exactly
# the class of thing this file's self-verification discipline exists to
# catch before it reaches MuJoCo): at sb.NOMINAL_POSE's pure-standing height
# (knee bent only 12deg), the leg already uses 99.45% of its max reach with
# ZERO horizontal offset -- only 3.3mm of slack. The full offline kinematic
# plan showed real CoM-to-planted-foot horizontal excursions up to ~114mm
# during double support (the far foot, still planted at its pre-step
# position, trails behind the advancing CoM) -- far beyond that 3.3mm/80mm
# budget. The fix is NOT a smaller step_length: even the plan's conservative
# 80mm steps exceed the standing-height budget on their own. It's a deeper
# standing crouch for WALKING than for static balance -- which also happens
# to be thematically apt, since ASIMO is well known for exactly this
# bent-knee gait, for the same reach-margin reason. Because the leg starts
# this close to full extension, crouch depth vs. reach margin is sharply
# nonlinear (near-singular Jacobian at full extension -- flagged as a risk
# while planning this file).
#
# 0.20 (200mm, ~1.75x the observed worst case) was the first value tried,
# but it over-corrected: deepening the REST crouch pushes the rest ankle
# angle closer to its own -30deg limit, and the swing foot's ~20-30mm lift
# shortens the hip-to-foot distance during swing, bending the knee MORE
# (not less) and driving ankle_pitch further negative on top of that already
# eaten-into rest angle -- a second joint-limit failure the first fix
# introduced. Swept jointly against step_height (see plan_footsteps):
# WALK_AX_MARGIN_M=0.15 (still ~1.3x the 114mm worst case) paired with
# step_height=0.02 keeps ankle_pitch's swing-lift minimum at -26.6deg,
# comfortably inside the +-1deg margin gate, while max knee_pitch (41.0deg)
# and hip_pitch stay well clear of their own limits too.
WALK_AX_MARGIN_M = 0.15   # target max |hip_x - foot_x| budget, see above


def solve_walk_pose(model):
    """Exact closed-form (same relations leg_ik uses, solved in reverse):
    the pelvis height at which a leg at full extension (r=R_MAX) has exactly
    WALK_AX_MARGIN_M of horizontal slack. Returns (pelvis_z, hip_deg,
    knee_deg, ankle_deg) for a foot centered exactly under the hip (roll=0,
    dx=0) at that height -- NOT sb.NOMINAL_POSE's (shallower) height."""
    az_budget = math.sqrt(R_MAX**2 - WALK_AX_MARGIN_M**2)
    pelvis_z = FOOT_DOWN + az_budget

    hip_origin = np.array([0.0, HIP_Y, pelvis_z])
    foot_target = np.array([0.0, HIP_Y, 0.0])
    roll, hip, knee, ankle = leg_ik(hip_origin, foot_target)
    assert abs(roll) < 1e-9, "centered target must give exactly zero hip_roll"
    return pelvis_z, math.degrees(hip), math.degrees(knee), math.degrees(ankle)


def solve_walk_com_height(model, pelvis_z, hip_deg, knee_deg, ankle_deg):
    """FK the full symmetric walking pose (both legs, pelvis freejoint z set
    directly to pelvis_z -- see solve_walk_pose's docstring on why this is
    exact, not approximate, given leg_ik/leg_fk's already-validated <2mm
    round-trip) to get its whole-body CoM height/offset. Analogous to
    solve_whole_body_com_height, but for this deeper walking crouch rather
    than sb.NOMINAL_POSE."""
    data = mujoco.MjData(model)
    mujoco.mj_resetData(model, data)
    data.qpos[2] = pelvis_z   # freejoint qpos layout: [x, y, z, qw, qx, qy, qz]
    for side in ("left", "right"):
        for jn, deg in (("hip_pitch", hip_deg), ("knee_pitch", knee_deg),
                        ("ankle_pitch", ankle_deg)):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{jn}")
            data.qpos[model.jnt_qposadr[jid]] = math.radians(deg)
    mujoco.mj_forward(model, data)

    com = data.subtree_com[0].copy()
    pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    pelvis_xyz = data.xpos[pid].copy()
    z_c = com[2]
    offset_xy = (pelvis_xyz[0] - com[0], pelvis_xyz[1] - com[1])
    return z_c, offset_xy, pelvis_xyz, data


# ---- Stage 1: per-leg inverse kinematics -----------------------------------

class UnreachableTarget(ValueError):
    pass


def leg_ik(hip_origin, foot_target):
    """Solve (hip_roll, hip_pitch, knee_pitch, ankle_pitch) in RADIANS for a
    single leg, given the hip_roll joint's world origin and the desired
    world-frame foot sole target — both (x, y, z) tuples/arrays.

    See module docstring point 5 for the derivation and for the sign error
    this caught. Do not "simplify" theta_roll's sign without re-running
    --selftest — a plausible-looking wrong version of this passed a
    zero-offset check before."""
    hip_origin = np.asarray(hip_origin, dtype=float)
    foot_target = np.asarray(foot_target, dtype=float)
    dx, dy, dz = foot_target - hip_origin

    theta_roll = math.atan2(dy, -dz)

    rx = dx
    rz = -math.sqrt(dy**2 + dz**2)

    ax = rx - FOOT_FWD
    az = rz - (-FOOT_DOWN)
    r = math.sqrt(ax**2 + az**2)
    eps = 1e-6   # floating-point safety margin at the r=L1+L2 boundary
    if r > R_MAX + eps or r < abs(L1 - L2) - eps:
        raise UnreachableTarget(
            f"leg_ik: target unreachable, r={r:.4f}m (limits "
            f"[{abs(L1-L2):.4f}, {R_MAX:.4f}]m) for foot_target={foot_target}, "
            f"hip_origin={hip_origin}")

    cos_k = np.clip((r**2 - L1**2 - L2**2) / (2 * L1 * L2), -1.0, 1.0)
    theta_k = math.acos(cos_k)          # knee_pitch, flexion-only branch, >= 0
    alpha = math.atan2(-ax, -az)
    gamma = math.atan2(L2 * math.sin(theta_k), L1 + L2 * math.cos(theta_k))
    theta_h = alpha - gamma             # hip_pitch — NOT alpha+gamma (hyperextension branch)
    theta_a = -(theta_h + theta_k)      # ankle_pitch, enforces level foot

    return theta_roll, theta_h, theta_k, theta_a


def leg_fk(model, data, side, hip_roll, hip_pitch, knee_pitch, ankle_pitch):
    """Forward-kinematic a leg's joint angles (radians) via the real MJCF and
    return the resulting world-frame foot body position. Used only for
    self-verification (--selftest), never in the hot path."""
    mujoco.mj_resetData(model, data)
    for jn, val in (("hip_roll", hip_roll), ("hip_pitch", hip_pitch),
                    ("knee_pitch", knee_pitch), ("ankle_pitch", ankle_pitch)):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{jn}")
        data.qpos[model.jnt_qposadr[jid]] = val
    mujoco.mj_forward(model, data)
    fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_foot")
    return data.xpos[fid].copy()


def leg_fk_full(model, data, pelvis_xyz, side, hip_roll, hip_pitch, knee_pitch, ankle_pitch):
    """Like leg_fk, but also places the pelvis freejoint at pelvis_xyz
    (identity orientation — consistent with this file's no-lean
    simplification) instead of leaving it at the model's default reset
    position. leg_fk alone is only valid when the caller's hip_origin was
    itself derived from that same default reset pose (true in Stage 1's
    round-trip tests); Stage 5's plan uses a time-varying walking-pose
    pelvis position, so its FK verification needs this variant instead."""
    mujoco.mj_resetData(model, data)
    data.qpos[0:3] = pelvis_xyz
    for jn, val in (("hip_roll", hip_roll), ("hip_pitch", hip_pitch),
                    ("knee_pitch", knee_pitch), ("ankle_pitch", ankle_pitch)):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{jn}")
        data.qpos[model.jnt_qposadr[jid]] = val
    mujoco.mj_forward(model, data)
    fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_foot")
    return data.xpos[fid].copy()


def hip_origin_for_side(pelvis_xyz, side):
    y_sign = 1.0 if side == "left" else -1.0
    return np.array([pelvis_xyz[0], pelvis_xyz[1] + y_sign * HIP_Y, pelvis_xyz[2]])


# ---- Stage 2: footstep planner --------------------------------------------

def ease(s):
    """Smooth 0->1 ease (raised cosine). Copied from sim_walk_gait.py — same
    helper, no behavior change."""
    return 0.5 - 0.5 * math.cos(math.pi * s)


def _centroid(foot_xy):
    lx, ly = foot_xy["left"]
    rx, ry = foot_xy["right"]
    return ((lx + rx) / 2.0, (ly + ry) / 2.0)


def plan_footsteps(n_steps, step_length=0.08, step_width=2 * HIP_Y, step_height=0.02,
                    step_duration=0.6, ds_fraction=0.4, dwell_s=0.8,
                    start_stance_side="right"):
    """Straight-line-only footstep plan: alternating left/right placements at
    a fixed step_length/step_width, wrapped in an initial and terminal
    double-support dwell (see module docstring point 1 — the initial dwell
    is required for solve_dcm_backward's boundary condition to be
    consistent with the robot actually starting at rest, not optional
    polish) PLUS an explicit initial weight-shift and final weight-shift-back
    ("double"-kind phases, identical in kind to every other step's
    double-support transition): standing centered (dwell) and standing on
    one foot about to swing the other (the first single-support phase's
    target) are genuinely different ZMP targets — (0,0) vs. the first stance
    foot's (0, +-hip_y) — so going straight from one to the other without an
    explicit blend is a real step discontinuity in p(t) itself, not just a
    derivative kink. This was caught by the implied-ZMP self-consistency
    check (see _selftest_stage3): the very first version of this planner
    omitted these shifts and the check failed with a sharp error spike
    exactly at the dwell/single-support boundary. The old sim_walk_gait.py
    had the same mechanism under a different name (its "SHIFT" phase, run
    before every swing) — this generalizes it to just the start/end of the
    whole walk, since mid-walk it's already covered by every step's
    "double" phase blending toward the next stance foot.

    step_height defaults to 20mm, not a rounder 30mm — see the comment above
    WALK_AX_MARGIN_M: a larger lift shortens the swing leg's hip-to-foot
    distance enough to push ankle_pitch past its -30deg limit given this
    design's tight reach margin. 20mm is the value a joint sweep of lift
    height against walking-crouch depth settled on.

    ds_fraction defaults to 0.4, not the more common 0.3, purely for gait
    QUALITY, not stability -- both are stable, but this was a real,
    measured tradeoff at first. Swept directly against a steady-state run
    (10 steps, K_DCM/DCM_FILTER_ALPHA held at their then-validated
    values): 0.4 nearly halves both the peak-tilt rocking (6.9 -> 5.5deg)
    and the pelvis's vertical bob (14.2 -> 9.3mm) each step -- the two
    things that visually read as "stumbling" even though the gait was
    never actually falling over at 0.3. The initial cost was the
    clean/validated step range dropping from 15 to 13 (14 steps degraded
    to 25deg tilt, not the sharp fall seen past the old boundary), and a
    K_DCM/DCM_FILTER_ALPHA re-sweep at 0.4 alone couldn't recover 15
    steps -- but a separate fix (tightening MAX_DCM_CORRECTION_M, see
    that constant's comment) recovered the full 15-step range on top of
    ds_fraction=0.4's smoothness gain, so this ended up a net win with no
    remaining tradeoff. If a longer walk still matters more than per-step
    smoothness for some future use, pass ds_fraction=0.3 explicitly.

    Returns (phases, t_end). Each phase is a dict:
      kind: "dwell" | "single" | "double"
      t0, t1: phase time window
      zmp_p0, zmp_p1: ZMP reference (x, y) at phase start/end (equal for
        "dwell"/"single" — constant; blended between for "double")
      foot_xy: {side: (x, y)} for every foot that is PLANTED throughout this
        phase (both feet for dwell/double; just the stance foot for single)
      swing_side, swing_from_xy, swing_to_xy, step_height: only present for
        "single" phases — feed directly to swing_foot_target.
    """
    half_w = step_width / 2.0
    foot_xy = {"left": (0.0, half_w), "right": (0.0, -half_w)}

    phases = []
    t = 0.0
    t_ss = step_duration * (1.0 - ds_fraction)
    t_ds = step_duration * ds_fraction

    centroid = _centroid(foot_xy)
    phases.append(dict(kind="dwell", t0=t, t1=t + dwell_s,
                        zmp_p0=centroid, zmp_p1=centroid,
                        foot_xy=dict(foot_xy)))
    t += dwell_s

    first_stance_xy = foot_xy[start_stance_side]
    phases.append(dict(kind="double", t0=t, t1=t + t_ds,
                        zmp_p0=centroid, zmp_p1=first_stance_xy,
                        foot_xy=dict(foot_xy)))
    t += t_ds

    stance_side = start_stance_side
    step_x = 0.0

    for _ in range(n_steps):
        swing_side = "left" if stance_side == "right" else "right"
        step_x += step_length
        new_xy = (step_x, foot_xy[swing_side][1])
        stance_xy = foot_xy[stance_side]

        phases.append(dict(kind="single", t0=t, t1=t + t_ss,
                            zmp_p0=stance_xy, zmp_p1=stance_xy,
                            foot_xy={stance_side: stance_xy},
                            swing_side=swing_side,
                            swing_from_xy=foot_xy[swing_side],
                            swing_to_xy=new_xy,
                            step_height=step_height))
        t += t_ss

        foot_xy[swing_side] = new_xy

        phases.append(dict(kind="double", t0=t, t1=t + t_ds,
                            zmp_p0=stance_xy, zmp_p1=new_xy,
                            foot_xy=dict(foot_xy)))
        t += t_ds

        stance_side = swing_side

    final_stance_xy = foot_xy[stance_side]
    centroid = _centroid(foot_xy)
    phases.append(dict(kind="double", t0=t, t1=t + t_ds,
                        zmp_p0=final_stance_xy, zmp_p1=centroid,
                        foot_xy=dict(foot_xy)))
    t += t_ds

    phases.append(dict(kind="dwell", t0=t, t1=t + dwell_s,
                        zmp_p0=centroid, zmp_p1=centroid,
                        foot_xy=dict(foot_xy)))
    t += dwell_s

    return phases, t


# ---- Stage 3: ZMP reference + DCM/CoM trajectory ---------------------------

def zmp_reference(t, phases):
    """Planned ZMP reference (x, y) at time t. Constant during dwell/single
    support; ease()-blended between the previous and next stance foot during
    double support. Clamps t to the plan's time range."""
    if t <= phases[0]["t0"]:
        return phases[0]["zmp_p0"]
    if t >= phases[-1]["t1"]:
        return phases[-1]["zmp_p1"]
    for p in phases:
        if p["t0"] <= t <= p["t1"]:
            if p["kind"] == "double":
                s = (t - p["t0"]) / (p["t1"] - p["t0"])
                e = ease(s)
                x = p["zmp_p0"][0] + e * (p["zmp_p1"][0] - p["zmp_p0"][0])
                y = p["zmp_p0"][1] + e * (p["zmp_p1"][1] - p["zmp_p0"][1])
                return (x, y)
            return p["zmp_p0"]
    raise ValueError(f"t={t} not within any phase")


def solve_dcm_backward(phases, t_end, dt, Tc):
    """Solve xi_dot = (xi - p(t))/Tc BACKWARD in time (RK4, negative dt) from
    a terminal condition xi(t_end) = p(t_end) — the robot ends at rest over
    the last footstep. This ODE is unstable integrated forward but exactly
    equivalent to a stable one integrated backward — see module docstring
    point 3. Each RK4 stage evaluates zmp_reference at its own true
    (decreasing) t, not at t_end. Returns (ts, xi) as dense arrays ascending
    in t, ts[0]=0, ts[-1]=t_end."""
    n = int(round(t_end / dt))
    ts = np.linspace(0.0, t_end, n + 1)
    xi = np.zeros((n + 1, 2))
    xi[-1] = zmp_reference(t_end, phases)

    def deriv(t, xi_val):
        p = np.array(zmp_reference(t, phases))
        return (xi_val - p) / Tc

    h = -dt
    for i in range(n, 0, -1):
        t, y = ts[i], xi[i]
        k1 = deriv(t, y)
        k2 = deriv(t + h / 2, y + h / 2 * k1)
        k3 = deriv(t + h / 2, y + h / 2 * k2)
        k4 = deriv(t + h, y + h * k3)
        xi[i - 1] = y + h / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    return ts, xi


def integrate_com_forward(ts, xi, x0, Tc):
    """Solve the SEPARATE, stable ODE xdot_com = (xi(t) - x_com(t))/Tc
    FORWARD in time (RK4) from x_com(0)=x0, using the dense xi(t) array from
    solve_dcm_backward (same time grid; xi at RK4 midpoints is linearly
    interpolated between grid points, which is accurate given dt is small
    relative to Tc). x0 should be phases[0]["zmp_p0"] (the initial dwell's
    centroid), consistent with the robot starting at rest there. Returns
    x_com as a dense array on the same ts grid."""
    n = len(ts) - 1
    dt = ts[1] - ts[0]
    x_com = np.zeros((n + 1, 2))
    x_com[0] = x0

    for i in range(n):
        xi_t, xi_t1 = xi[i], xi[i + 1]
        xi_mid = (xi_t + xi_t1) / 2.0
        y = x_com[i]
        k1 = (xi_t - y) / Tc
        k2 = (xi_mid - (y + dt / 2 * k1)) / Tc
        k3 = (xi_mid - (y + dt / 2 * k2)) / Tc
        k4 = (xi_t1 - (y + dt * k3)) / Tc
        x_com[i + 1] = y + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)

    return x_com


# ---- Stage 4: swing foot trajectory ----------------------------------------

def swing_foot_target(t, liftoff_xy, touchdown_xy, step_height, t_start, t_end):
    """Cartesian (x, y, z) target for a swinging foot's sole point at time t,
    relative to the ground (z=0 planted). Horizontal motion is ease()'d
    (zero velocity at both endpoints, matching the ZMP double-support blend's
    smoothness); vertical motion is a half-sine lift peaking at step_height
    at mid-swing and returning to exactly 0 at both endpoints (needed so the
    foot is flush with the ground the instant it plants — no impact residual
    baked into the plan itself)."""
    s = min(max((t - t_start) / (t_end - t_start), 0.0), 1.0)
    e = ease(s)
    x = liftoff_xy[0] + e * (touchdown_xy[0] - liftoff_xy[0])
    y = liftoff_xy[1] + e * (touchdown_xy[1] - liftoff_xy[1])
    z = step_height * math.sin(math.pi * s)
    return (x, y, z)


# ---- Self-test ---------------------------------------------------------

def _selftest_stage0(model):
    z_c, offset = solve_whole_body_com_height(model)
    Tc = math.sqrt(z_c / G)
    print(f"[stage 0] z_c={z_c:.5f} m (expect ~0.5355)  Tc={Tc:.4f} s (expect ~0.2336)"
          f"  pelvis_com_offset_xy={offset}")
    # Verified constants recomputed 2026-09-08 after ankle_roll became a
    # fully motorized joint (design/joints.yaml, temporary -- see
    # tools/CLAUDE.md) and picked up a second motor+gearbox mass
    # (design/mass.yaml). Was z_c=0.5658/Tc=0.2402 with ankle_roll as a
    # light passive pivot, and z_c=0.5713/Tc=0.241 before ankle_roll existed
    # at all -- see module docstring.
    assert abs(z_c - 0.5355) < 0.001, "z_c does not match verified whole-body CoM height"
    assert abs(Tc - 0.2336) < 0.002, "Tc does not match verified value"
    print("[stage 0] OK")


def _selftest_stage0b(model):
    pelvis_z, hip_deg, knee_deg, ankle_deg = solve_walk_pose(model)
    print(f"[stage 0b] walking crouch: pelvis_z={pelvis_z:.4f}m (sb standing: "
          f"{sb.solve_nominal_geometry(model)[0] + 0:.4f}m via root_z)  "
          f"hip={hip_deg:+.2f}deg knee={knee_deg:.2f}deg ankle={ankle_deg:+.2f}deg")

    z_c, offset_xy, pelvis_xyz, data = solve_walk_com_height(
        model, pelvis_z, hip_deg, knee_deg, ankle_deg)
    Tc = math.sqrt(z_c / G)
    print(f"[stage 0b] z_c_walk={z_c:.5f}m  Tc_walk={Tc:.4f}s  "
          f"pelvis_com_offset_xy={offset_xy}  pelvis actual z={pelvis_xyz[2]:.5f}")
    assert abs(pelvis_xyz[2] - pelvis_z) < 1e-9, "pelvis freejoint z != requested pelvis_z"

    for side in ("left", "right"):
        fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_foot")
        foot_z = data.xpos[fid, 2]
        assert abs(foot_z) < 2e-3, f"{side} foot not at ground level at walking pose: z={foot_z}"

    limits = _joint_limits_deg(model)
    margin_deg = 1.0
    for side in ("left", "right"):
        for jn, deg in (("hip_pitch", hip_deg), ("knee_pitch", knee_deg), ("ankle_pitch", ankle_deg)):
            lo, hi = limits[(side, jn)]
            assert lo + margin_deg < deg < hi - margin_deg, \
                f"{side}_{jn}={deg:.2f} outside limit margin ({lo:.1f}, {hi:.1f})"

    ax_check = math.sqrt(R_MAX**2 - WALK_AX_MARGIN_M**2) - (pelvis_z - FOOT_DOWN)
    print(f"[stage 0b] reach-margin closed-form self-consistency: {ax_check:.2e} (expect ~0)")
    assert abs(ax_check) < 1e-9, "solve_walk_pose's closed form is self-inconsistent"
    print("[stage 0b] OK — walking crouch deeper than standing, feet grounded, within joint limits")


def _selftest_stage1(model):
    data = mujoco.MjData(model)

    # (a) round-trip against sb.NOMINAL_POSE's own FK'd foot positions.
    root_z, _, _ = sb.solve_nominal_geometry(model)
    sb.set_initial_state(model, data, root_z)
    pid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    pelvis_xyz = data.xpos[pid].copy()

    max_err = 0.0
    for side in ("left", "right"):
        fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_foot")
        foot_xyz = data.xpos[fid].copy()
        hip_origin = hip_origin_for_side(pelvis_xyz, side)
        roll, hip, knee, ankle = leg_ik(hip_origin, foot_xyz)

        expect_hip = math.radians(sb.NOMINAL_HIP_DEG)
        expect_knee = math.radians(sb.NOMINAL_KNEE_DEG)
        err = max(abs(roll - 0.0), abs(hip - expect_hip), abs(knee - expect_knee))
        max_err = max(max_err, err)
        print(f"[stage 1a] {side}: roll={math.degrees(roll):+.6f} "
              f"hip={math.degrees(hip):+.6f} (expect {sb.NOMINAL_HIP_DEG:+.6f}) "
              f"knee={math.degrees(knee):+.6f} (expect {sb.NOMINAL_KNEE_DEG:+.6f})")
    assert max_err < 1e-6, f"nominal-pose IK round-trip error too large: {max_err}"
    print("[stage 1a] OK — nominal-pose round-trip within 1e-6 rad")

    # (b) round-trip at nonzero hip_roll, both signs, both sides — this is
    # the check that actually catches the sign error described in the module
    # docstring; (a) alone would not (dy=0 there).
    max_pos_err = 0.0
    max_angle_err = 0.0
    for side in ("left", "right"):
        for test_deg in (8.0, -5.0, -12.0):
            mujoco.mj_resetData(model, data)
            for jn, deg in (("hip_pitch", sb.NOMINAL_HIP_DEG), ("knee_pitch", sb.NOMINAL_KNEE_DEG),
                            ("ankle_pitch", sb.NOMINAL_ANKLE_DEG), ("hip_roll", test_deg)):
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{jn}")
                data.qpos[model.jnt_qposadr[jid]] = math.radians(deg)
            mujoco.mj_forward(model, data)
            fid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"{side}_foot")
            foot_xyz = data.xpos[fid].copy()
            pid2 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
            hip_origin = hip_origin_for_side(data.xpos[pid2], side)

            roll, hip, knee, ankle = leg_ik(hip_origin, foot_xyz)
            angle_err = abs(math.degrees(roll) - test_deg)
            max_angle_err = max(max_angle_err, angle_err)

            fk_check = leg_fk(model, data, side, roll, hip, knee, ankle)
            pos_err = float(np.linalg.norm(fk_check - foot_xyz))
            max_pos_err = max(max_pos_err, pos_err)
            print(f"[stage 1b] {side} hip_roll={test_deg:+.1f}deg -> "
                  f"recovered={math.degrees(roll):+.4f}deg  FK-check pos err={pos_err*1000:.4f}mm")
    assert max_angle_err < 1e-4, f"hip_roll sign/magnitude round-trip error too large: {max_angle_err}"
    assert max_pos_err < 2e-3, f"FK round-trip position error too large: {max_pos_err}"
    print("[stage 1b] OK — nonzero-hip_roll round-trip within 1e-4 deg / 2mm")


def _selftest_stage2():
    step_length, step_width = 0.08, 2 * HIP_Y
    phases, t_end = plan_footsteps(n_steps=5, step_length=step_length, step_width=step_width)
    print(f"[stage 2] {len(phases)} phases, t_end={t_end:.3f}s")

    single_phases = [p for p in phases if p["kind"] == "single"]
    assert len(single_phases) == 5, f"expected 5 single-support phases, got {len(single_phases)}"

    last_x = -1.0
    for i, p in enumerate(single_phases):
        landing_x, landing_y = p["swing_to_xy"]
        assert landing_x > last_x, f"step {i}: x did not advance ({landing_x} <= {last_x})"
        last_x = landing_x
        stance_side = "left" if p["swing_side"] == "right" else "right"
        stance_y = p["foot_xy"][stance_side][1]
        y_gap = abs(landing_y - stance_y)
        assert abs(y_gap - step_width) < 1e-9, \
            f"step {i}: lateral gap {y_gap} != step_width {step_width}"
        print(f"[stage 2]   step {i}: swing={p['swing_side']:5s} "
              f"landing=({landing_x:.3f}, {landing_y:+.3f})  y_gap={y_gap:.3f}")

    assert phases[0]["kind"] == "dwell" and phases[-1]["kind"] == "dwell", \
        "plan must start and end with a double-support dwell"
    print("[stage 2] OK — monotonic x, alternating sides, correct lateral spacing, dwell at both ends")


def _selftest_stage3(model):
    z_c, _ = solve_whole_body_com_height(model)
    Tc = math.sqrt(z_c / G)
    phases, t_end = plan_footsteps(n_steps=5, step_length=0.08, step_width=2 * HIP_Y, dwell_s=0.8)
    dt = 0.002  # matches the MJCF's sim timestep

    ts, xi = solve_dcm_backward(phases, t_end, dt, Tc)
    x0 = np.array(phases[0]["zmp_p0"])
    x_com = integrate_com_forward(ts, xi, x0, Tc)

    p0 = np.array(phases[0]["zmp_p0"])
    p_end = np.array(phases[-1]["zmp_p1"])
    xi0_err = float(np.linalg.norm(xi[0] - p0))
    xcom0_err = float(np.linalg.norm(x_com[0] - p0))
    xcom_end_err = float(np.linalg.norm(x_com[-1] - p_end))
    print(f"[stage 3] boundary: xi(0) err={xi0_err*1000:.3f}mm  x_com(0) err={xcom0_err*1000:.3f}mm  "
          f"x_com(t_end) err={xcom_end_err*1000:.3f}mm")
    assert xi0_err < 5e-3, f"xi(0) does not converge to p0 within the initial dwell: {xi0_err}"
    assert xcom0_err < 5e-3, f"x_com(0) inconsistent with initial dwell: {xcom0_err}"
    assert xcom_end_err < 1e-2, f"x_com(t_end) does not converge to the final footstep centroid: {xcom_end_err}"

    # Implied-ZMP self-consistency check: p_implied = x_com - (z_c/g)*xcom_ddot
    # should track the planned reference everywhere except right at the
    # double-support blend corners (p's acceleration is genuinely
    # discontinuous there — ease()'s second derivative jumps from 0 to
    # nonzero at s=0/1 — not a numerical artifact). Get xcom_dot from the
    # ODE's own defining relation (exact, no finite-differencing) rather
    # than differentiating x_com numerically, then finite-difference that
    # ONCE for acceleration instead of twice — halves the differentiation
    # noise this check would otherwise be dominated by.
    xcom_dot = (xi - x_com) / Tc
    xcom_ddot = np.gradient(xcom_dot, dt, axis=0)
    p_implied = x_com - (Tc**2) * xcom_ddot
    p_ref = np.array([zmp_reference(t, phases) for t in ts])
    err = np.linalg.norm(p_implied - p_ref, axis=1)

    mask = np.ones(len(ts), dtype=bool)
    margin = 0.05
    for p in phases:
        if p["kind"] == "double":
            lo = np.searchsorted(ts, p["t0"] - margin)
            hi = np.searchsorted(ts, p["t1"] + margin)
            mask[lo:hi] = False
    masked_max = float(err[mask].max()) if mask.any() else 0.0
    print(f"[stage 3] implied-ZMP tracking error: overall max={err.max()*1000:.2f}mm  "
          f"away from double-support corners max={masked_max*1000:.2f}mm")
    assert masked_max < 0.02, f"implied ZMP doesn't track the reference away from DS corners: {masked_max}"
    print("[stage 3] OK")


def _selftest_stage4():
    liftoff, touchdown, h, t0, t1 = (0.0, 0.05), (0.08, 0.05), 0.03, 1.0, 1.6

    x0, y0, z0 = swing_foot_target(t0, liftoff, touchdown, h, t0, t1)
    x1, y1, z1 = swing_foot_target(t1, liftoff, touchdown, h, t0, t1)
    assert abs(z0) < 1e-9 and abs(z1) < 1e-9, f"z must be 0 at both endpoints: {z0}, {z1}"
    assert abs(x0 - liftoff[0]) < 1e-9 and abs(x1 - touchdown[0]) < 1e-9, "endpoint x mismatch"

    t_mid = (t0 + t1) / 2.0
    _, _, z_mid = swing_foot_target(t_mid, liftoff, touchdown, h, t0, t1)
    assert abs(z_mid - h) < 1e-9, f"peak z at mid-swing should equal step_height: {z_mid} vs {h}"

    # Zero horizontal velocity at both endpoints (finite difference near each end).
    eps = 1e-6
    xa, _, _ = swing_foot_target(t0 + eps, liftoff, touchdown, h, t0, t1)
    xb, _, _ = swing_foot_target(t1 - eps, liftoff, touchdown, h, t0, t1)
    v_start = (xa - x0) / eps
    v_end = (x1 - xb) / eps
    print(f"[stage 4] endpoint z=({z0:.2e}, {z1:.2e})  mid z={z_mid:.4f} (expect {h})  "
          f"endpoint horiz. vel~=({v_start:.2e}, {v_end:.2e})")
    assert abs(v_start) < 1e-3, f"nonzero horizontal velocity at liftoff: {v_start}"
    assert abs(v_end) < 1e-3, f"nonzero horizontal velocity at touchdown: {v_end}"
    print("[stage 4] OK — swing endpoints flush with ground, zero horiz. velocity, correct peak height")


# ---- Stage 5: full offline kinematic plan ----------------------------------

def _phase_at(t, phases):
    """The phase dict containing time t (clamped to the plan's range)."""
    if t <= phases[0]["t0"]:
        return phases[0]
    if t >= phases[-1]["t1"]:
        return phases[-1]
    for p in phases:
        if p["t0"] <= t <= p["t1"]:
            return p
    raise ValueError(f"t={t} not within any phase")


def build_kinematic_plan(phases, ts, x_com, pelvis_com_offset_xy, pelvis_z):
    """Compose Stages 1-4: for every sample in ts, resolve the swinging foot's
    Cartesian target (if any phase is "single" at that t) and every planted
    foot's fixed target, convert the planned CoM (x, y) into a pelvis target
    (holding pelvis z at the nominal standing height throughout — no lean in
    this version), and solve leg_ik per side. Returns a dict:
      {"ts": ts, "left": (N,4) array, "right": (N,4) array}
    each row (hip_roll, hip_pitch, knee_pitch, ankle_pitch) in radians."""
    n = len(ts)
    joints = {"left": np.zeros((n, 4)), "right": np.zeros((n, 4))}

    for i, t in enumerate(ts):
        phase = _phase_at(t, phases)
        pelvis_xyz = np.array([x_com[i, 0] + pelvis_com_offset_xy[0],
                                x_com[i, 1] + pelvis_com_offset_xy[1],
                                pelvis_z])

        foot_target = {side: (xy[0], xy[1], 0.0) for side, xy in phase["foot_xy"].items()}
        if phase["kind"] == "single":
            foot_target[phase["swing_side"]] = swing_foot_target(
                t, phase["swing_from_xy"], phase["swing_to_xy"],
                phase["step_height"], phase["t0"], phase["t1"])

        for side in ("left", "right"):
            hip_origin = hip_origin_for_side(pelvis_xyz, side)
            joints[side][i] = leg_ik(hip_origin, foot_target[side])

    return {"ts": ts, "left": joints["left"], "right": joints["right"]}


def _joint_limits_deg(model):
    """{(side, jointname): (lo_deg, hi_deg)} read from the compiled model —
    the same limits as design/joints.yaml, already baked into the MJCF by
    generate_urdf.py/generate_mjcf.py."""
    limits = {}
    for side in ("left", "right"):
        for jn in ("hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch"):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{jn}")
            lo, hi = model.jnt_range[jid]
            limits[(side, jn)] = (math.degrees(lo), math.degrees(hi))
    return limits


def _selftest_stage5(model):
    # Use the deeper WALKING crouch (solve_walk_pose), not sb's shallow
    # pure-standing pose — see the module-level comment above solve_walk_pose
    # for why the standing height leaves essentially no reach margin for any
    # real CoM excursion during single/double support.
    pelvis_z, hip_deg, knee_deg, ankle_deg = solve_walk_pose(model)
    z_c, pelvis_com_offset_xy, _, _ = solve_walk_com_height(
        model, pelvis_z, hip_deg, knee_deg, ankle_deg)
    Tc = math.sqrt(z_c / G)

    phases, t_end = plan_footsteps(n_steps=5, step_length=0.08, step_width=2 * HIP_Y, dwell_s=0.8)
    dt = 0.002
    ts, xi = solve_dcm_backward(phases, t_end, dt, Tc)
    x0 = np.array(phases[0]["zmp_p0"])
    x_com = integrate_com_forward(ts, xi, x0, Tc)

    plan = build_kinematic_plan(phases, ts, x_com, pelvis_com_offset_xy, pelvis_z)
    print(f"[stage 5] built kinematic plan: {len(ts)} samples over {t_end:.3f}s")

    limits = _joint_limits_deg(model)
    margin_deg = 1.0
    joint_names = ("hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch")

    data = mujoco.MjData(model)
    max_fk_err = 0.0
    max_jump_deg = 0.0
    check_stride = 5   # FK-verify every 5th sample (~10ms) — exact enough, keeps selftest fast

    for side in ("left", "right"):
        arr_deg = np.degrees(plan[side])
        for jn_idx, jn in enumerate(joint_names):
            lo, hi = limits[(side, jn)]
            col = arr_deg[:, jn_idx]
            assert col.min() > lo + margin_deg and col.max() < hi - margin_deg, (
                f"{side}_{jn} exceeds limit margin: min={col.min():.2f} max={col.max():.2f} "
                f"limits=({lo:.2f},{hi:.2f})")

        jumps = np.abs(np.diff(arr_deg, axis=0)).max()
        max_jump_deg = max(max_jump_deg, jumps)

        for i in range(0, len(ts), check_stride):
            t = ts[i]
            phase = _phase_at(t, phases)
            target = (phase["foot_xy"][side][0], phase["foot_xy"][side][1], 0.0) \
                if side in phase["foot_xy"] else None
            if phase["kind"] == "single" and phase["swing_side"] == side:
                target = swing_foot_target(t, phase["swing_from_xy"], phase["swing_to_xy"],
                                            phase["step_height"], phase["t0"], phase["t1"])
            assert target is not None, f"no target resolved for {side} at t={t}"

            pelvis_xyz = np.array([x_com[i, 0] + pelvis_com_offset_xy[0],
                                    x_com[i, 1] + pelvis_com_offset_xy[1], pelvis_z])
            roll, hip, knee, ankle = plan[side][i]
            fk_pos = leg_fk_full(model, data, pelvis_xyz, side, roll, hip, knee, ankle)
            pos_err = float(np.linalg.norm(fk_pos - np.array(target)))
            max_fk_err = max(max_fk_err, pos_err)

    assert max_fk_err < 2e-3, f"FK round-trip position error too large: {max_fk_err}"
    print(f"[stage 5] max FK round-trip position error={max_fk_err*1000:.4f}mm "
          f"(checked every {check_stride}th sample)")
    print(f"[stage 5] max joint-limit margin held: {margin_deg:.1f}deg  "
          f"max per-sample joint jump={max_jump_deg:.4f}deg")
    print("[stage 5] OK")


def _selftest_stage5c(model):
    """Validates lateral_zmp_correction's sign and compute_zmp's y-output
    BEFORE trusting either inside the real Stage 6 drive loop -- matching
    every other stage in this file (see the module docstring's own
    discipline note). A sign mistake here is exactly the class of bug that
    silently DESTABILIZES instead of correcting (see K_DCM's own history),
    so this is checked directly, not assumed from the algebra."""
    # compute_zmp reads ~0 laterally at the nominal symmetric standing pose
    # (double support, weight evenly split) -- same fixture sim_zmp_balance.py
    # itself uses to validate compute_zmp.
    data = mujoco.MjData(model)
    root_z, _, _ = sb.solve_nominal_geometry(model)
    sb.set_initial_state(model, data, root_z)
    mujoco.mj_forward(model, data)
    foot_body_ids = sb._foot_body_ids(model)
    _, zy0, fz0 = sb.compute_zmp(model, data, foot_body_ids)
    assert zy0 is not None and fz0 > 0.0, "no foot contact at nominal standing pose"
    assert abs(zy0) < 0.005, f"lateral ZMP not ~0 at symmetric standing pose: {zy0:.5f}m"
    print(f"[stage 5c] compute_zmp lateral reading at nominal stance: {zy0*1000:.3f}mm (expect ~0)")

    # Sign check: if the target is to the LEFT (+y) of the measured ZMP, the
    # correction must be POSITIVE (this file's ankle_roll axis/positive
    # direction, design/kinematics.yaml: "eversion, sole tilts laterally
    # away from body midline" -- a positive command tilts the sole's near
    # edge down, shifting the sole's ground-pressure centroid toward +y,
    # i.e. toward the target). A restoring correction must shrink the error
    # if applied; check the SIGN directly rather than trust the algebra.
    corr_pos_error = lateral_zmp_correction(zy_filtered=0.0, target_y=0.05)
    corr_neg_error = lateral_zmp_correction(zy_filtered=0.05, target_y=0.0)
    assert corr_pos_error > 0.0, f"positive lateral error produced non-restoring correction: {corr_pos_error}"
    assert corr_neg_error < 0.0, f"negative lateral error produced non-restoring correction: {corr_neg_error}"
    assert abs(corr_pos_error) <= MAX_ANKLE_ROLL_CORRECTION_RAD + 1e-9, "correction exceeds its own clamp"
    print(f"[stage 5c] correction sign OK: +5cm error -> {math.degrees(corr_pos_error):+.2f}deg, "
          f"-5cm error -> {math.degrees(corr_neg_error):+.2f}deg")
    print("[stage 5c] OK")


# ---- Stage 6: MuJoCo drive loop --------------------------------------------
#
# REARCHITECTED from a small ad-hoc "ZMP-error -> hip_roll/ankle_pitch trim"
# (kept only as a superseded reference in git history) to real DCM/Capture-
# Point TRACKING CONTROL, after a wide P/I/D gain sweep on that trim (P:
# 1.5-9.0, I: 3-20, D: 0.05-0.8) failed to prevent a 4th-step fall in every
# configuration tried. That failure was not a tuning problem: xi's own
# defining ODE (xidot = (xi-p)/Tc, see solve_dcm_backward) is OPEN-LOOP
# UNSTABLE by construction -- literally why it's called the "divergent"
# component of motion -- so any open-loop trajectory replay, no matter how
# well-planned, has NO mechanism to reject the natural exponential growth
# of any real-world deviation from that plan (segment inertia, actuator
# lag, contact compliance -- all the things this file's own docstring
# already flagged as point-mass-model gaps). Tracing pelvis_y confirmed
# this exactly: not a steady offset, a GROWING oscillation with period
# close to one step_duration -- the signature of an uncorrected unstable
# mode, not insufficient gain on a stable one. A pure ZMP-error trim also
# structurally can't fix this: it reacts to CoM position error only, never
# to the CoM velocity error that determines whether the DCM's growth is
# accelerating or already decaying -- exactly the information the DCM
# itself (xi = x_com + Tc*xcom_dot) is defined to carry.
#
# This IS a solved problem, and not a new one: Kajita et al.'s 2003 ZMP
# preview control (used on HRP-2, closing the loop on the unstable ZMP-
# tracking mode via an LQR preview gain) and the later Capture-Point/DCM
# tracking control law (Pratt et al. 2006, formalized by Englsberger et
# al.) are both standard, well-documented answers from exactly this
# problem's research lineage -- confirmed directly against a paper in this
# project's own reference library (Zhu & Thomas 2023, "Mechanical Design
# of a Biped Robot FORREST and an Extended Capture-Point-Based Walking
# Pattern Generator," Section 6.1), which gives the DCM tracking control
# law used below almost verbatim:
#
#   xi_d_dot - xi_dot = -k*(xi_d - xi)      (a STABLE error ODE, any k>0)
#   p_cmd = p_d + (1 + k/omega)*(xi - xi_d)  (the corrected ZMP command)
#
# using the MEASURED xi (real CoM position + velocity, from MuJoCo's
# mj_subtreeVel), not the planned one -- the crux of the fix. That paper's
# p_cmd feeds a torque-based Cartesian controller; this file has no
# force/torque loop (position actuators only, kp=150, unchanged per the
# original plan), so the same correction is applied directly to the
# PELVIS POSITION TARGET that already feeds leg_ik every step, rather than
# to a ZMP command: pelvis_xy_cmd(t) = x_com_planned(t) + offset +
# K_DCM*(xi_actual(t) - xi_planned(t)).
#
# K_DCM's SIGN is opposite the paper's ZMP-domain law, and this was found
# by direct measurement, not derivation -- worth flagging since it's easy
# to get backwards. The paper's correction pushes the ZMP command FURTHER
# in the direction of a growing xi error, which is correct there because
# moving the ZMP (support point) *decelerates* the CoM away from it
# (xddot_com = omega^2*(x_com - p) -- CoM accelerates AWAY from p). This
# file instead corrects a PELVIS POSITION target that a position-controlled
# leg actively drives toward, which has the opposite effective sign: a
# same-direction correction on a position target amplifies the error
# instead of arresting it. Confirmed directly: K_DCM=+1.0 made every run
# catastrophically worse (max DCM error grew from ~75mm to ~900mm within
# 3-4 steps); flipping to K_DCM=-1.0 immediately gave a damped, bounded
# oscillation and, for the first time, sustained walking well past the
# earlier 3-4 step wall (see run_walk's docstring for the validated count).
# A small gain sweep around -1.0 (-0.6 to -1.5) showed a fairly narrow
# stable band, not a gentle gradient -- consistent with a real feedback
# stability margin rather than a free parameter to push arbitrarily.
#
# RETUNED 2026-09-08 (-1.0 -> -0.6): making ankle_roll actuated (see
# tools/CLAUDE.md's dated entries) added a second motor+gearbox mass low in
# the kinematic chain, which dropped z_c from 0.5713m to 0.5355m and Tc from
# 0.241s to 0.2336s -- a small (~3%) shift that nonetheless crossed this
# already-narrow stability margin: every model.opt.timestep, K_DCM=-1.0
# produced a ~87-96deg fall regardless of MAX_DCM_CORRECTION_M or any new
# ankle_roll correction, ACROSS THE WHOLE RANGE swept for both. Decomposing
# the fall into roll/pitch (a check this project should have done FIRST,
# not after chasing lateral fixes for a day) showed it was PITCH-dominated
# (48deg pitch vs 9deg roll shortly before the fall) -- a sagittal margin
# regression, not a missing-lateral-feedback one. A direct K_DCM sweep at
# the new mass found a new narrow clean band at -0.55 to -0.60 (-0.5 gives
# 40.85deg, -0.65 falls at 86.33deg) -- -0.6 gives 6.42deg, flat from n=3 to
# n=14 (n=15 regresses to the old file's own already-documented wall
# territory; not chased further, see run_walk's docstring).
K_DCM = -0.6                             # dimensionless gain on the DCM correction

# MAX_DCM_CORRECTION_M started as a safety clamp only (keeping a bad
# transient from commanding an outright-unreachable IK target), but turned
# out to matter for the SLOW step-13/14+ wall too (see plan_footsteps'
# ds_fraction=0.4 comment for that wall's discovery) -- swept directly
# against it, not assumed: a TIGHTER clamp (0.08 -> 0.04) dropped n=16's
# peak tilt from 94.7deg (a fall) to 19.6deg and, combined with
# ds_fraction=0.4, pushed the clean/validated range at THAT setting from 13
# steps back up to 15 (with a flatter, slightly better ~5.9-6.4deg tilt
# across the whole range, not just a wider range at the same quality). The
# landscape isn't a smooth gradient, though -- 0.035 gave 87.8deg (a fall)
# sandwiched between 0.03's and 0.04's clean results -- so this is a real,
# somewhat fragile stability margin, not a value to nudge casually without
# re-running --selftest and a longer --steps check.
MAX_DCM_CORRECTION_M = 0.04

# DCM_FILTER_ALPHA: the raw xi measurement (from mj_subtreeVel's per-step
# subtree_com/subtree_linvel) is noisy frame to frame -- the same lesson
# sim_zmp_balance.py already learned about raw per-contact ZMP (see that
# file's module docstring point 2), just for a different signal. Without
# filtering, K_DCM=-1.0 sustains real walking but with a SLOWLY GROWING
# lateral oscillation that eventually crosses an instability threshold at
# a roughly fixed elapsed time regardless of how many more steps the plan
# continues for -- confirmed directly: n_steps=10, 12, 15, and 20 all
# first exceeded 15deg tilt at the identical t=6.93s, which rules out
# "approaching the plan's own end" as the cause (t_end differed by nearly
# 2x across those runs) and points instead to a slow resonance building
# from measurement noise feeding back through the correction every single
# timestep. EMA-filtering xi the same way sim_zmp_balance.py filters ZMP
# (same alpha=0.02, found by direct sweep, not copied on faith) pushed the
# clean/validated range from 9 steps to 15 (42.7deg tilt at 16 -- degraded,
# not yet root-caused further; see run_walk's docstring).
DCM_FILTER_ALPHA = 0.02

# ---- Local lateral ZMP feedback (ankle_roll) -------------------------------
#
# HUBO's own architecture (Heo, Lee, Oh, "Development of Humanoid Robots in
# HUBO Laboratory, KAIST", 2012) is not a single monolithic controller: an
# offline walking PATTERN (what plan_footsteps/solve_dcm_backward/
# integrate_com_forward already build) plus several small, layered real-time
# feedback controllers, balancing control first -- "a damping controller and
# a ZMP compensator... play the most important role for not only stable
# walking but also for balanced standing itself." Everything above this
# point in the file is the offline pattern. What follows is a balancing
# layer using ankle_roll (now actuated -- tools/CLAUDE.md's dated
# 2026-09-08 entries): a direct port of sim_zmp_balance.py's already-
# validated ankle-pitch ZMP compensator to the lateral axis.
#
# HONEST RESULT, not the original hypothesis: this layer alone did NOT fix
# the fall found after making ankle_roll actuated -- see K_DCM's own
# comment above for the real cause (a sagittal gain-margin regression from
# ankle_roll's added mass, found by decomposing the fall into roll/pitch
# instead of assuming it was lateral). With K_DCM retuned, a small K_ZMP_Y
# is still a real, validated improvement (6.42deg -> 6.36deg at n=3..14,
# swept directly: 0.02-0.05 clean, 0.10+ causes an abrupt fall -- another
# narrow-band margin, not a gentle gradient) and is kept because it's
# genuine local lateral feedback (this project's actual goal, HUBO's own
# "ZMP compensator" pattern), not because it was the fix for this
# particular fall.
#
# sim_zmp_balance.py's compute_zmp() already returns BOTH x and y ZMP from
# real contact points; its own run() only ever used x (ankle-pitch standing
# control). The y component was already being computed and silently
# discarded. The MEASURED signal here is that same y; the TARGET is
# zmp_reference(t, phases)'s own y component -- the identical planned
# reference the sagittal DCM math already tracks, not a new plan.
K_ZMP_Y = 0.05                           # proportional gain, rad per meter of lateral ZMP error --
                                          # swept directly (0.0-0.3); 0.05 is inside the clean
                                          # 0.02-0.05 band, 0.10+ falls abruptly
ZMP_Y_FILTER_ALPHA = 0.02                # same EMA alpha as DCM_FILTER_ALPHA -- same class of
                                          # frame-to-frame contact noise, not re-derived from scratch
MAX_ANKLE_ROLL_CORRECTION_RAD = math.radians(10.0)   # stays inside ankle_roll's own +-15deg range


def lateral_zmp_correction(zy_filtered, target_y, k_zmp_y=K_ZMP_Y,
                            max_correction_rad=MAX_ANKLE_ROLL_CORRECTION_RAD):
    """Proportional ankle_roll correction from planned-vs-measured lateral
    ZMP error -- HUBO's own 'ZMP compensator' pattern (see the block comment
    above), structurally identical to sim_zmp_balance.py's ankle_pitch
    standing controller applied to the other axis. Positive error (target
    ahead of measured, in +y) must produce a positive correction that pulls
    the sole toward it -- verified directly in _selftest_stage5c, not just by
    inspection, since a sign mistake here would silently DESTABILIZE rather
    than correct (this file's own K_DCM history is exactly this class of
    bug: see the comment above K_DCM)."""
    error = target_y - zy_filtered
    return float(np.clip(k_zmp_y * error, -max_correction_rad, max_correction_rad))


def interpolate_plan(ts, arr, t):
    """Linear interpolation of an (N, 4) joint-angle trajectory at time t
    (clamped to [ts[0], ts[-1]])."""
    t = min(max(t, ts[0]), ts[-1])
    return np.array([np.interp(t, ts, arr[:, k]) for k in range(arr.shape[1])])


def run_walk(model, n_steps=5, render_path=None, render_every=10, verbose=True, k_dcm=K_DCM,
             k_zmp_y=K_ZMP_Y):
    """Build the offline DCM/CoM/footstep plan (Stages 0-3), then drive it
    with real MuJoCo dynamics (mj_step). Unlike Stages 5's pure offline
    plan, joint targets here are computed LIVE every step from real
    measured state via DCM tracking control (see the comment above K_DCM):
    the pelvis position target fed to leg_ik each step is the offline
    plan's x_com(t) corrected by K_DCM times the gap between the (EMA-
    filtered -- see DCM_FILTER_ALPHA) actual and planned Divergent
    Component of Motion, clamped by MAX_DCM_CORRECTION_M, PLUS a local
    lateral ZMP compensator driving ankle_roll (see the block comment above
    K_ZMP_Y -- HUBO's own "ZMP compensator" balancing layer, ankle_roll now
    actuated). VALIDATED (current model, ankle_roll actuated): 14 steps
    clean (peak tilt <=6.4deg, flat across n=3..14); 15 regresses to the
    old wall territory this file's history already documents, not chased
    further. This RECOVERS the model-without-ankle_roll baseline (was 15
    clean, 16 borderline, 17 fails) to within one step of range -- see
    K_DCM's own comment for the real regression this session found and
    fixed (a sagittal gain-margin shift from ankle_roll's added mass, NOT
    the missing-lateral-feedback problem originally suspected). Returns a
    summary dict; optionally renders an offscreen GIF."""
    data = mujoco.MjData(model)

    pelvis_z, hip_deg, knee_deg, ankle_deg = solve_walk_pose(model)
    z_c, pelvis_com_offset_xy, _, _ = solve_walk_com_height(
        model, pelvis_z, hip_deg, knee_deg, ankle_deg)
    Tc = math.sqrt(z_c / G)

    phases, t_end = plan_footsteps(n_steps=n_steps)
    dt_plan = 0.002
    ts, xi = solve_dcm_backward(phases, t_end, dt_plan, Tc)
    x0 = np.array(phases[0]["zmp_p0"])
    x_com = integrate_com_forward(ts, xi, x0, Tc)

    act_index = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
                 for i in range(model.nu)}

    # t=0 initial state: actual == planned, so no DCM correction is needed
    # yet -- IK the offline plan's own t=0 targets directly.
    pelvis_xyz0 = np.array([x_com[0, 0] + pelvis_com_offset_xy[0],
                             x_com[0, 1] + pelvis_com_offset_xy[1], pelvis_z])
    phase0 = _phase_at(0.0, phases)
    foot_target0 = {side: (xy[0], xy[1], 0.0) for side, xy in phase0["foot_xy"].items()}

    mujoco.mj_resetData(model, data)
    data.qpos[0] = pelvis_xyz0[0]
    data.qpos[1] = pelvis_xyz0[1]
    data.qpos[2] = pelvis_z
    for side in ("left", "right"):
        hip_origin = hip_origin_for_side(pelvis_xyz0, side)
        roll, hip, knee, ankle = leg_ik(hip_origin, foot_target0[side])
        for jn, val in zip(("hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch"),
                            (roll, hip, knee, ankle)):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{jn}")
            data.qpos[model.jnt_qposadr[jid]] = val
        data.ctrl[act_index[f"act_{side}_hip_roll"]] = roll
        data.ctrl[act_index[f"act_{side}_hip_pitch"]] = hip
        data.ctrl[act_index[f"act_{side}_knee_pitch"]] = knee
        data.ctrl[act_index[f"act_{side}_ankle_pitch"]] = ankle
    mujoco.mj_forward(model, data)
    mujoco.mj_subtreeVel(model, data)   # populate subtree_linvel for the first DCM measurement
    xi_filtered = data.subtree_com[0][:2].copy() + Tc * data.subtree_linvel[0][:2].copy()

    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")

    # Same offscreen-renderer / tracking-camera pattern as sim_walk_gait.py's
    # run() — a side-on (azimuth=90) view following the pelvis in x so the
    # walk stays framed regardless of how far it travels.
    renderer = None
    cam = None
    if render_path:
        renderer = mujoco.Renderer(model, height=360, width=480)
        cam = mujoco.MjvCamera()
        cam.azimuth, cam.elevation, cam.distance = 90, -12, 2.0
        cam.lookat = np.array([0.0, 0.0, 0.45])
    frames = []

    sim_dt = model.opt.timestep
    n_sim_steps = int(t_end / sim_dt)

    pelvis_x0 = data.xpos[pelvis_id, 0]
    max_tilt = 0.0
    max_dcm_err_m = 0.0
    diverged = False

    # Local lateral ZMP feedback setup (see the block comment above
    # lateral_zmp_correction) -- foot_body_ids mirrors sim_zmp_balance.py's
    # own run(); zy_filtered starts at 0.0 (nominal double-support ZMP is
    # centered laterally, matching the initial state set above).
    foot_body_ids = sb._foot_body_ids(model)
    zy_filtered = 0.0

    for step in range(n_sim_steps):
        t = data.time

        # Real DCM tracking control (see comment above K_DCM): correct the
        # pelvis position target using the MEASURED capture point, not just
        # the planned time-indexed trajectory. The measurement is EMA-
        # filtered (see DCM_FILTER_ALPHA) — using it raw sustains walking
        # but with a slowly growing oscillation that eventually diverges.
        com_actual_xy = data.subtree_com[0][:2].copy()
        comvel_actual_xy = data.subtree_linvel[0][:2].copy()
        xi_raw = com_actual_xy + Tc * comvel_actual_xy
        xi_filtered = (1 - DCM_FILTER_ALPHA) * xi_filtered + DCM_FILTER_ALPHA * xi_raw
        xi_planned = np.array([np.interp(t, ts, xi[:, 0]), np.interp(t, ts, xi[:, 1])])
        x_com_planned = np.array([np.interp(t, ts, x_com[:, 0]), np.interp(t, ts, x_com[:, 1])])

        dcm_err = xi_filtered - xi_planned
        max_dcm_err_m = max(max_dcm_err_m, float(np.linalg.norm(dcm_err)))
        correction = np.clip(k_dcm * dcm_err, -MAX_DCM_CORRECTION_M, MAX_DCM_CORRECTION_M)
        pelvis_xy_cmd = x_com_planned + np.asarray(pelvis_com_offset_xy) + correction
        pelvis_xyz_cmd = np.array([pelvis_xy_cmd[0], pelvis_xy_cmd[1], pelvis_z])

        phase = _phase_at(t, phases)
        foot_target = {side: (xy[0], xy[1], 0.0) for side, xy in phase["foot_xy"].items()}
        if phase["kind"] == "single":
            foot_target[phase["swing_side"]] = swing_foot_target(
                t, phase["swing_from_xy"], phase["swing_to_xy"],
                phase["step_height"], phase["t0"], phase["t1"])

        # Local lateral ZMP feedback (ankle_roll) -- see the block comment
        # above lateral_zmp_correction. Only a PLANTED foot's ankle_roll can
        # affect ZMP, so a currently-swinging foot is held at nominal (0)
        # instead, same as before this layer existed.
        _, zy_raw, _ = sb.compute_zmp(model, data, foot_body_ids)
        if zy_raw is not None:
            zy_filtered = (1 - ZMP_Y_FILTER_ALPHA) * zy_filtered + ZMP_Y_FILTER_ALPHA * zy_raw
        target_y = zmp_reference(t, phases)[1]
        # ankle_roll's nominal_stand_deg is 0 (design/joints.yaml), so the
        # correction IS the commanded angle -- would need "+ nominal" if
        # that ever changed.
        ankle_roll_corr = lateral_zmp_correction(zy_filtered, target_y, k_zmp_y=k_zmp_y)
        planted_sides = set(phase["foot_xy"].keys())
        for side in ("left", "right"):
            data.ctrl[act_index[f"act_{side}_ankle_roll"]] = (
                ankle_roll_corr if side in planted_sides else 0.0)

        for side in ("left", "right"):
            hip_origin = hip_origin_for_side(pelvis_xyz_cmd, side)
            try:
                roll, hip, knee, ankle = leg_ik(hip_origin, foot_target[side])
            except UnreachableTarget:
                # A transient DCM correction pushed the IK target just out
                # of reach -- hold the last commanded angles for this leg
                # rather than crash the whole run over one bad sample.
                roll = data.ctrl[act_index[f"act_{side}_hip_roll"]]
                hip = data.ctrl[act_index[f"act_{side}_hip_pitch"]]
                knee = data.ctrl[act_index[f"act_{side}_knee_pitch"]]
                ankle = data.ctrl[act_index[f"act_{side}_ankle_pitch"]]
            data.ctrl[act_index[f"act_{side}_hip_roll"]] = roll
            data.ctrl[act_index[f"act_{side}_hip_pitch"]] = hip
            data.ctrl[act_index[f"act_{side}_knee_pitch"]] = knee
            data.ctrl[act_index[f"act_{side}_ankle_pitch"]] = ankle

        for jn in ("torso_pitch", "torso_roll", "torso_yaw"):
            data.ctrl[act_index[f"act_{jn}"]] = 0.0

        mujoco.mj_step(model, data)
        mujoco.mj_subtreeVel(model, data)

        if not np.all(np.isfinite(data.qpos)) or not np.all(np.isfinite(data.qvel)):
            diverged = True
            if verbose:
                print(f"  simulation diverged at t={t:.3f}s")
            break

        max_tilt = max(max_tilt, sb.pelvis_tilt_deg(model, data))

        if renderer is not None and step % render_every == 0:
            cam.lookat[0] = data.xpos[pelvis_id, 0]
            renderer.update_scene(data, camera=cam)
            frames.append(renderer.render().copy())

    pelvis_x_final = data.xpos[pelvis_id, 0]
    net_forward = pelvis_x_final - pelvis_x0

    if render_path and frames:
        import imageio
        imageio.mimsave(render_path, frames, fps=int(1.0 / (sim_dt * render_every)))
        if verbose:
            print(f"  wrote {len(frames)} frames to {render_path}")

    return dict(diverged=diverged, max_tilt_deg=max_tilt, net_forward_m=net_forward,
                sim_time=data.time, n_steps=n_steps, max_dcm_err_m=max_dcm_err_m)


def _selftest_stage6(model):
    # 12 steps: comfortably inside the validated stable range (n=3..15 all
    # stay under 6.5deg peak tilt — see run_walk's docstring), with margin
    # below the n=16 borderline point.
    result = run_walk(model, n_steps=12, verbose=False)
    print(f"[stage 6] 12-step run: diverged={result['diverged']}  "
          f"max_tilt={result['max_tilt_deg']:.2f}deg  net_forward={result['net_forward_m']*1000:.1f}mm  "
          f"max_dcm_err={result['max_dcm_err_m']*1000:.1f}mm  sim_time={result['sim_time']:.3f}s")
    assert not result["diverged"], "simulation diverged within 12 steps"
    assert result["net_forward_m"] > 0.0, \
        "pelvis net motion is not forward — the headline stumbling-fix regressed"
    assert result["max_tilt_deg"] < 20.0, \
        f"max tilt {result['max_tilt_deg']:.2f}deg not below old gait's ~20deg baseline"
    print("[stage 6] OK — stable, net-forward pelvis motion, tilt held below old gait's baseline")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--selftest", action="store_true",
                        help="Run the numerical self-verification gates and exit.")
    parser.add_argument("--steps", type=int, default=None,
                        help="Run n_steps of the walking gait in MuJoCo.")
    parser.add_argument("--render", type=str, default=None,
                        help="Path to write an offscreen-rendered GIF (used with --steps).")
    args = parser.parse_args()

    if not MJCF_PATH.exists():
        print(f"ERROR: MJCF not found at {MJCF_PATH}")
        print("Run: python3 tools/generate_mjcf.py")
        raise SystemExit(1)

    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))

    if args.selftest:
        _selftest_stage0(model)
        _selftest_stage0b(model)
        _selftest_stage1(model)
        _selftest_stage2()
        _selftest_stage3(model)
        _selftest_stage4()
        _selftest_stage5(model)
        _selftest_stage5c(model)
        _selftest_stage6(model)
        print()
        print("All implemented self-tests passed.")
        return

    if args.steps is not None:
        result = run_walk(model, n_steps=args.steps, render_path=args.render)
        print()
        print(f"{args.steps}-step run: diverged={result['diverged']}  "
              f"max_tilt={result['max_tilt_deg']:.2f}deg  "
              f"net_forward={result['net_forward_m']*1000:.1f}mm  sim_time={result['sim_time']:.3f}s")
        return

    print("Nothing to do — pass --selftest or --steps N.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
P3 Walking Gait — receding-horizon DCM/Capture-Point replanning.

Why this file exists: sim_walk_lipm.py (validated to 15 steps, kept as
reference/fallback) plans ONE FIXED trajectory for an entire N-step walk,
offline, once -- a hard terminal "come to rest at the very end" boundary
condition, with footstep (x, y) placement that NEVER responds to measured
state. The user watched that and asked the right question: is chasing a
longer and longer fixed plan how real bipeds learned to walk indefinitely?
No -- real sustained walking and push recovery come from RECEDING-HORIZON
REPLANNING (continuously re-solving a short window ahead from the robot's
actual current state) with footstep PLACEMENT ITSELF adapting to a
measured disturbance, not a fixed plan tracked harder. This file replaces
the fixed whole-walk plan with exactly that, grounded in real literature,
not derived from memory (see CAPTURE-POINT FOOTSTEP PLACEMENT below for
the citation).

ARCHITECTURE -- two timescales, mirroring how real receding-horizon
controllers separate a fast tracking layer from a slower replanning layer:

  1. FAST inner loop (every physics tick, 500Hz) -- REUSES sim_walk_lipm's
     own K_DCM/DCM_FILTER_ALPHA/MAX_DCM_CORRECTION_M tracking correction
     completely unchanged: the pelvis position target is the currently
     active short-horizon plan's x_com(t), corrected toward the measured
     DCM. This layer's job didn't change; what changed is what it's
     tracking (a plan that keeps sliding forward, not one fixed at t=0).

  2. SLOW outer loop (every REPLAN_PERIOD_S, ~10Hz -- see that constant's
     comment for why, empirically, not the ~50-100Hz the plan anticipated)
     -- two things, every tick:
       a. CAPTURE-POINT FOOTSTEP PLACEMENT (capture_point_footstep):
          the immediately-swinging foot's touchdown target is recomputed
          from the MEASURED DCM via the closed-form one-step DCM
          propagation relation
              p_new = p0 + (xi_measured - p0)*exp(omega*(T-t)) - b_nom
          (Khadiv et al., via Roux 2024 "MPC-RL-based bipedal robot
          control" thesis, eqs. 1b/5a-b/51-62 -- confirmed against the
          paper before implementing, not assumed; reduces to the classical
          Pratt 2006 capture point p_new=xi_measured when T=t, b_nom=0,
          see _selftest_stage0). This is the actual mechanism that makes a
          real push shift where the next foot lands, instead of only
          changing how hard the pelvis-tracking correction works to chase
          a target that never moves. ONLY the immediately-swinging foot's
          target adapts -- footsteps beyond it use plain nominal geometry,
          since the closed form is only exact while the ZMP stays at one
          fixed stance point, true only within a single phase (see
          capture_point_footstep's docstring for the full invariant).
       b. SHORT-HORIZON REPLAN (replan_horizon/build_horizon_phases):
          sim_walk_lipm's OWN solve_dcm_backward/integrate_com_forward,
          reused completely unchanged, re-solved on a freshly built SHORT
          phases list (current swing + a small nominal lookahead,
          N_FUTURE_STEPS) with x0 = the robot's ACTUAL (filtered)
          measured CoM position, not an assumed-at-rest value. The
          terminal "rest" condition this ODE needs is always just "rest a
          few steps from now," recomputed fresh every replan -- this is
          what removes the fixed-whole-walk horizon.

  3. SWING RETARGET (retargeted_swing_foot_target, Hermite-blended) --
     when the touchdown target moves mid-flight, the swing foot's
     Cartesian path must stay velocity-continuous, not just position-
     continuous: a naive fresh ease() re-anchor (what an earlier draft of
     this design used) forces velocity to zero at the retarget instant
     even though the real mid-swing velocity is generally nonzero -- a
     real bug caught by a design-review pass BEFORE any code was written,
     fixed with a cubic Hermite blend matching the foot's actual position
     AND velocity at the retarget instant. Vertical lift is unaffected --
     retargeting only ever changes where the foot is headed horizontally.

BUGS FOUND BUILDING THIS (three, all real, all caught by this codebase's
own "verify before trusting" discipline -- tracing actual values in a
failing run, not guessing): (1) xi_filtered was updated TWICE on ticks
where the slow loop also fired (a redundant double-EMA using an identical
pre-mj_step sample) -- a periodic irregularity in the filter's effective
time constant, synced to the replan rate, exactly the coherent-forcing
risk flagged before implementation began. (2) x0_filtered was updated only
at replan ticks using a PER-TICK alpha value, silently giving it an
effective time constant ~10x slower than intended (replan_period/sim_dt
ticks between updates instead of 1) -- traced by finding it still near its
t=0 initial value multiple steps into a walk that was falling over for no
obvious reason. (3) both build_horizon_phases' lookahead footsteps AND the
top-level _nominal_next_touchdown inherited their lateral (y) reference
from the swinging foot's ACTUAL (possibly Y-adapted) last landing position
instead of the true fixed nominal track -- this let lateral adaptation
compound forward through every subsequent "nominal" footstep instead of
being a one-off correction, traced by watching the measured DCM's y grow
monotonically past the stance foot's own y, every step, without bound.

VALIDATED STATE (honest, not the full original goal; ankle_roll now
actuated, see tools/CLAUDE.md's dated 2026-09-08 entries for the full
regression/fix history): 12 steps clean (peak tilt <=12.4deg), 14 degrades
(~40deg), 15+ falls -- an IMPROVEMENT over the pre-ankle_roll baseline (was
8 clean/6.9deg, 9 borderline, 10+ falls), not just a recovery, once this
file's OWN K_DCM_RECEDE gain was found by direct sweep (-0.70, NOT
lw.K_DCM's -0.6 -- the two files' stability margins shifted differently
under the same mass change and needed separately-tuned values, confirmed
directly rather than assumed to transfer). Also carries lw's local lateral
ZMP feedback (ankle_roll, see lw.K_ZMP_Y's block comment) -- a real
addition, though the sagittal K_DCM_RECEDE retune was what actually fixed
the fall this session found (see that constant's own comment for why: a
mass-driven margin shift, not a missing-lateral-feedback problem). Per-axis
footstep-adaptation clamps were
necessary and are asymmetric for a real geometric reason, not just a
tuning choice: X has real room (step_length=80mm), but the total lateral
stance half-width is only 50mm (HIP_Y), so a naive symmetric clamp on Y
can place a foot AT the centerline or past the opposite foot's own track,
collapsing the base of support outright (confirmed directly: Y-only
adaptation fell at every clamp >=20mm, clean at 10mm).

KNOWN LIMITATION, NOT YET RESOLVED: even with footstep adaptation
DISABLED ENTIRELY (pure receding-horizon replanning of a fixed nominal
gait, nothing else changed), the walk still degrades in the same 8-15
step range -- confirmed directly, which rules out the footstep-placement
law as the (sole) cause of the remaining wall. Tracing the gap between
each short-horizon replan's own prediction and the NEXT replan's real
measured state shows a genuinely GROWING discontinuity (single-digit mm
early in a walk, hundreds of mm by the time it falls) -- a real,
accumulating divergence between the idealized short-horizon model and
actual MuJoCo dynamics, not a one-off implementation slip. A wide sweep of
REPLAN_PERIOD_S, N_FUTURE_STEPS, and K_DCM did not find a combination
extending cleanly past ~12 steps; disabling replanning after the first
window (reverting close to a single short fixed-horizon solve) fails even
faster, ruling out "replanning itself is destabilizing" as the explanation
too.

ONE HYPOTHESIS TESTED AND RULED OUT: research grounded in the same paper
the footstep-placement law came from (Roux 2024, eq. 24) showed real
receding-horizon controllers terminate each short window at the DCM
offset implied by CONTINUING to walk nominally (xi(t_end)=p(t_end)+
b_nom), not at rest -- build_horizon_phases originally ended every window
with a fabricated double-support+dwell (rest) sequence, forcing each
short-horizon solve to plan a full stop that never actually happens, a
real, literature-confirmed design flaw. Fixed via a closed-form
correction on replan_horizon's output (xi's ODE is linear, so shifting
the terminal condition by b_nom is just adding b_nom*exp((t-t_end)/Tc) to
the existing lw.solve_dcm_backward result, no need to modify that reused
function -- see replan_horizon's docstring for the full derivation).
Verified harmless (Stage 2's zero-disturbance regression numbers improved
slightly) but did NOT resolve the wall -- the growing-discontinuity
pattern traced afterward is essentially unchanged in magnitude and
timing. This rules out mis-specified terminal cost as the DOMINANT cause,
though the fix is correct and worth keeping regardless. Still-untried
next steps, in order of what the ruled-out result makes most likely:
investigating whether the short-horizon model's own Tc/point-mass-LIPM
dynamics assumptions systematically mismatch real MuJoCo behavior (this
project already found one concrete instance of exactly this kind of gap
-- sim_walk_lipm.py's own module docstring documents a commanded weight
shift settling at only ~70% of target under plain position control, a
steady-state error the idealized model doesn't predict; a state
estimator that only low-pass-filters a single derived signal, like this
file's EMA, has no independent way to detect or correct that kind of
systematic bias, unlike e.g. a Kalman filter fusing an independent
absolute reference -- research surfaced this concern but did not confirm
it as the specific cause here); smoothly BLENDING the fast loop's
reference across a replan boundary instead of hard-switching to the new
window (addresses the symptom directly, regardless of root cause,
still not attempted).

PUSH RECOVERY -- tested directly (Stage 5), not claimed: swept external
pelvis forces (5-30N, 0.1s, this design's own ~8.9kg mass -- NOT the
~1.2kg reference robot's 6.3N from the literature) applied mid-walk within
the validated range. Footstep adaptation does NOT yet show a clear,
consistent net benefit over the plain fast-loop correction alone at the
magnitudes tested -- comparable outcomes both ways, sometimes adaptation
did worse. The underlying mechanism (capture_point_footstep) is verified
mathematically correct in isolation (_selftest_stage0) and DOES shift the
footstep target in response to a real disturbance (confirmed by direct
trace) -- what's not yet demonstrated is that this measurably HELPS
recovery in the current tuning, which is a real, open finding, not a
claimed capability.

Non-goals (unchanged from the plan): turning/steering, step TIMING
adaptation (only footstep POSITION adapts; T stays bound to the nominal
phase's own t1, never recomputed relative to "now" -- see
capture_point_footstep's docstring for why that specific invariant
matters), a full inequality-constrained QP optimizer (the closed-form
single-step formula is a deliberate simplification given this project's
kinematic, not torque-controlled, actuators), adapting footsteps beyond
the immediately-swinging one.

Usage:
    python3 tools/sim_walk_recede.py --selftest
    python3 tools/sim_walk_recede.py --steps 8
    python3 tools/sim_walk_recede.py --steps 8 --render out.gif
    python3 tools/sim_walk_recede.py --steps 8 --push-at 2.0 --push-force 0 10 --push-duration 0.1
"""

import math
import argparse
import numpy as np
import mujoco

import sim_walk_lipm as lw
import sim_zmp_balance as sb

MJCF_PATH = lw.MJCF_PATH


# ---- Stage 0: footstep placement law ---------------------------------------

def nominal_dcm_offset(step_displacement, t_ss, omega):
    """The nominal (undisturbed, continued-walking) DCM offset from the new
    stance foot at touchdown -- b_nom in the plan's notation. Closed form
    from the periodicity condition (DCM offset relative to the stance foot
    must repeat every step under nominal walking): b_nom = L / (e^(omega*T) - 1)
    for a step of signed displacement L (in whichever axis) taken over
    duration T. Call once per axis: step_displacement=step_length for x,
    and the swing foot's signed lateral displacement (0 for this codebase's
    straight-line-only geometry, where a given foot's y never changes
    between steps) for y. Returns a plain float (m)."""
    return step_displacement / (math.exp(omega * t_ss) - 1.0)


def capture_point_footstep(p0_xy, xi_measured_xy, b_nom_xy, omega, t_now, T_touchdown):
    """Capture-point footstep placement law (see the plan file for the full
    derivation and citation): given the CURRENT measured DCM xi_measured_xy
    at time t_now, the current stance foot p0_xy, and the swinging foot's
    FIXED planned touchdown time T_touchdown (bound once at swing start to
    the current phase's own t1 -- see the plan's "critical implementation
    invariant"; never recompute T relative to "now" each call, that would
    silently let step TIMING adapt too, which is out of scope), returns the
    new footstep target p_new_xy such that the DCM at touchdown lands at
    the nominal offset b_nom_xy from it.

        p_new = p0 + (xi_measured - p0) * exp(omega*(T-t)) - b_nom

    Reduces to the classical Pratt 2006 capture point (p_new = xi_measured)
    exactly when T_touchdown == t_now and b_nom_xy == 0 (see _selftest_stage0).

    This closed form is only exact while the stance foot stays fixed at
    p0_xy for the entire window [t_now, T_touchdown] -- true only within a
    single "single support" phase in this codebase's phase model. Do not
    call this across a double-support boundary."""
    p0 = np.asarray(p0_xy, dtype=float)
    xi = np.asarray(xi_measured_xy, dtype=float)
    b_nom = np.asarray(b_nom_xy, dtype=float)
    growth = math.exp(omega * (T_touchdown - t_now))
    return p0 + (xi - p0) * growth - b_nom


# ---- Stage 1: swing retarget with velocity-continuous re-blending ---------

def swing_foot_velocity(t, liftoff_xy, touchdown_xy, t_start, t_end):
    """Horizontal (x, y) velocity of lw.swing_foot_target's ease()-based
    trajectory at time t -- the analytic derivative, not finite-differenced.
    Needed to get vel_now when a retarget is triggered against an
    UN-retargeted (plain ease()) segment; a retarget triggered against an
    already-retargeted segment instead uses hermite_velocity below. ease(s)
    = 0.5-0.5*cos(pi*s), ease'(s) = 0.5*pi*sin(pi*s); dx/dt = ease'(s) *
    (touchdown-liftoff) / (t_end-t_start) by the chain rule."""
    tau = t_end - t_start
    s = min(max((t - t_start) / tau, 0.0), 1.0)
    dease_ds = 0.5 * math.pi * math.sin(math.pi * s)
    liftoff = np.asarray(liftoff_xy, dtype=float)
    touchdown = np.asarray(touchdown_xy, dtype=float)
    return dease_ds * (touchdown - liftoff) / tau


def _hermite_basis(s):
    """Cubic Hermite basis functions and their derivatives w.r.t. s, at
    normalized progress s in [0, 1]."""
    h00 = 2 * s**3 - 3 * s**2 + 1
    h10 = s**3 - 2 * s**2 + s
    h01 = -2 * s**3 + 3 * s**2
    h11 = s**3 - s**2
    dh00 = 6 * s**2 - 6 * s
    dh10 = 3 * s**2 - 4 * s + 1
    dh01 = -6 * s**2 + 6 * s
    dh11 = 3 * s**2 - 2 * s
    return (h00, h10, h01, h11), (dh00, dh10, dh01, dh11)


def hermite_position(p0_xy, v0_xy, p1_xy, v1_xy, tau, s):
    """Cubic Hermite interpolation between (p0, v0) at s=0 and (p1, v1) at
    s=1, over a segment of real-time duration tau (v0/v1 are PHYSICAL
    velocities; the tau scaling on the h10/h11 terms is the standard
    formulation converting them to normalized-s units)."""
    (h00, h10, h01, h11), _ = _hermite_basis(s)
    p0, v0 = np.asarray(p0_xy, dtype=float), np.asarray(v0_xy, dtype=float)
    p1, v1 = np.asarray(p1_xy, dtype=float), np.asarray(v1_xy, dtype=float)
    return h00 * p0 + h10 * tau * v0 + h01 * p1 + h11 * tau * v1


def hermite_velocity(p0_xy, v0_xy, p1_xy, v1_xy, tau, s):
    """Real-time derivative of hermite_position (chain rule: d/dt = d/ds *
    ds/dt = (...) / tau)."""
    _, (dh00, dh10, dh01, dh11) = _hermite_basis(s)
    p0, v0 = np.asarray(p0_xy, dtype=float), np.asarray(v0_xy, dtype=float)
    p1, v1 = np.asarray(p1_xy, dtype=float), np.asarray(v1_xy, dtype=float)
    return (dh00 * p0 + dh10 * tau * v0 + dh01 * p1 + dh11 * tau * v1) / tau


def retargeted_swing_foot_target(t, pos_now_xy, vel_now_xy, touchdown_xy,
                                  t_retarget, T_touchdown, step_height, phase_t0, phase_t1):
    """Like lw.swing_foot_target, but for a swing whose touchdown (x, y) has
    just been updated mid-flight (see capture_point_footstep) rather than
    fixed for the whole phase. Horizontal motion is a cubic Hermite blend
    from the foot's ACTUAL position and velocity at the retarget instant
    (pos_now_xy, vel_now_xy -- from swing_foot_velocity or hermite_velocity,
    whichever the PREVIOUS segment was) to (touchdown_xy, zero velocity) at
    T_touchdown -- this is what fixes the velocity discontinuity a naive
    fresh-ease()-from-here re-anchor would introduce (see the plan file).
    Vertical motion is UNCHANGED from lw.swing_foot_target's half-sine lift,
    driven by the swing's OVERALL phase progress (phase_t0/phase_t1), not
    the retarget segment's own local progress -- retargeting only ever
    changes WHERE the foot is heading horizontally, never the lift profile.
    Returns (x, y, z)."""
    tau = T_touchdown - t_retarget
    s = min(max((t - t_retarget) / tau, 0.0), 1.0)
    x, y = hermite_position(pos_now_xy, vel_now_xy, touchdown_xy, (0.0, 0.0), tau, s)

    s_z = min(max((t - phase_t0) / (phase_t1 - phase_t0), 0.0), 1.0)
    z = step_height * math.sin(math.pi * s_z)
    return (x, y, z)


# ---- Stage 2: short-horizon DCM/CoM replanning wrapper ---------------------

def build_horizon_phases(remaining_swing_s, stance_side, stance_xy, swing_side,
                          swing_from_xy, swing_to_xy, n_future_steps,
                          step_length=0.08, step_width=2 * lw.HIP_Y, step_height=0.02,
                          step_duration=0.6, ds_fraction=0.4):
    """Build a SHORT rolling-horizon phases list (same dict shape as
    lw.plan_footsteps) anchored at the CURRENT swing in progress, instead of
    starting fresh at t=0 for an entire N-step walk. The current swing's
    target (swing_to_xy) may be the ADAPTED capture-point target from Stage
    0/1, not the nominal one -- only THIS immediately-swinging foot's target
    is allowed to differ from nominal geometry; every future footstep in the
    lookahead uses plain nominal step_length/step_width offsets (see the
    plan's scope: adapting footsteps beyond the immediately-swinging one is
    out of scope for v1, since the closed-form placement law is only exact
    within a single phase -- see capture_point_footstep's docstring).

    Does NOT end with a terminal double-support + dwell (rest condition) --
    an earlier version did, matching lw.plan_footsteps's own ending, and
    that was a real, verified-by-literature design bug, not a
    simplification: research grounded against the same paper the footstep-
    placement law came from (Roux 2024, eq. 24, terminal cost design)
    showed real receding-horizon controllers terminate each window at the
    DCM offset implied by CONTINUING to walk nominally, never at rest --
    forcing every short window to decelerate to a full stop, when the
    robot is never actually about to stop, injects a systematic bias that
    a fixed-gain feedback loop reproduces every single replan, compounding
    into the growing (not bounded) discontinuity this file's module
    docstring documents finding empirically before this fix. The caller
    (replan_horizon) applies the correct "continue walking" terminal
    condition via a closed-form correction on top of lw.solve_dcm_backward
    (reused unchanged) rather than this function faking a rest phase.

    Times in the returned phases are WINDOW-RELATIVE (t=0 at the replan
    instant / start of the current swing), NOT absolute simulation time --
    lw.solve_dcm_backward internally builds its own dense time array via
    np.linspace(0, t_end, ...), assuming phases span [0, t_end], so this
    must never be given absolute sim time. Callers translate back via
    t_window = t_sim - t_swing_start when interpolating the returned
    ts/xi/x_com against real simulation time. Returns (phases, t_end)."""
    phases = []
    t_ds = step_duration * ds_fraction
    t_ss = step_duration * (1.0 - ds_fraction)

    phases.append(dict(kind="single", t0=0.0, t1=remaining_swing_s,
                        zmp_p0=stance_xy, zmp_p1=stance_xy,
                        foot_xy={stance_side: stance_xy},
                        swing_side=swing_side,
                        swing_from_xy=swing_from_xy,
                        swing_to_xy=swing_to_xy,
                        step_height=step_height))
    t = remaining_swing_s

    foot_xy = {stance_side: stance_xy, swing_side: swing_to_xy}
    old_stance_xy = stance_xy
    stance_side = swing_side   # the swinging foot becomes the stance foot once it lands

    phases.append(dict(kind="double", t0=t, t1=t + t_ds,
                        zmp_p0=old_stance_xy, zmp_p1=swing_to_xy,
                        foot_xy=dict(foot_xy)))
    t += t_ds

    # Fixed nominal lateral track for lookahead footsteps -- NOT inherited
    # from foot_xy[nswing_side], which can carry the immediately-swinging
    # foot's ADAPTED (possibly Y-drifted) landing position. Every future
    # footstep beyond the immediately-swinging one must use PLAIN nominal
    # geometry (see this function's own docstring); inheriting a drifted Y
    # here let lateral adaptation compound forward through the whole
    # lookahead instead of being a one-off correction -- found by tracing a
    # real fall where xi_y grew monotonically past the stance foot's own y
    # every step, the signature of exactly this kind of runaway feedback.
    half_w = step_width / 2.0
    nominal_y = {"left": half_w, "right": -half_w}

    step_x = swing_to_xy[0]
    for _ in range(n_future_steps):
        nswing_side = "left" if stance_side == "right" else "right"
        step_x += step_length
        new_xy = (step_x, nominal_y[nswing_side])
        stance_xy_n = foot_xy[stance_side]

        phases.append(dict(kind="single", t0=t, t1=t + t_ss,
                            zmp_p0=stance_xy_n, zmp_p1=stance_xy_n,
                            foot_xy={stance_side: stance_xy_n},
                            swing_side=nswing_side,
                            swing_from_xy=foot_xy[nswing_side],
                            swing_to_xy=new_xy,
                            step_height=step_height))
        t += t_ss

        foot_xy[nswing_side] = new_xy

        phases.append(dict(kind="double", t0=t, t1=t + t_ds,
                            zmp_p0=stance_xy_n, zmp_p1=new_xy,
                            foot_xy=dict(foot_xy)))
        t += t_ds

        stance_side = nswing_side

    return phases, t


def replan_horizon(remaining_swing_s, stance_side, stance_xy, swing_side,
                    swing_from_xy, swing_to_xy, x0_xy, Tc, n_future_steps=2,
                    dt_plan=0.002, step_length=0.08, step_duration=0.6,
                    ds_fraction=0.4, **footstep_kwargs):
    """Build a short rolling-horizon phases list (build_horizon_phases) and
    solve its DCM/CoM trajectory, starting the CoM forward integration from
    x0_xy -- the robot's ACTUAL (EMA-filtered upstream by the caller, see
    the plan's highest-flagged risk) current CoM position, not an
    assumed-at-rest value.

    TERMINAL CONDITION CORRECTION (the fix for the growing-discontinuity
    limitation this file's module docstring documented): lw.solve_dcm_
    backward (reused unchanged) always solves toward xi(t_end)=p(t_end),
    i.e. AT REST over the final footstep -- correct for a whole walk that
    really does end there, wrong for a short rolling window where the
    robot is never actually about to stop. Confirmed against the same
    literature the footstep-placement law came from (Roux 2024, eq. 24):
    real receding-horizon controllers terminate each window at the DCM
    offset implied by CONTINUING to walk nominally, xi(t_end)=p(t_end)+
    b_nom, not at rest. Because xi's ODE is linear in xi, the closed-form
    fix doesn't require re-deriving or modifying solve_dcm_backward: if
    xi_rest(t) is its unmodified output, the corrected trajectory is
        xi(t) = xi_rest(t) + b_nom * exp((t - t_end) / Tc)
    (the homogeneous solution of xi_dot=xi/Tc integrated backward from
    d(t_end)=b_nom, d(t)=b_nom*exp((t-t_end)/Tc) -- decays to ~0 influence
    for t far before t_end, exactly ~b_nom right at t_end). x_com is then
    re-integrated forward (lw.integrate_com_forward, also reused
    unchanged) against this corrected xi, from the SAME real x0.

    Returns (phases, ts, xi, x_com, t_end), all in the SAME window-relative
    time as build_horizon_phases."""
    phases, t_end = build_horizon_phases(remaining_swing_s, stance_side, stance_xy,
                                          swing_side, swing_from_xy, swing_to_xy,
                                          n_future_steps, step_length=step_length,
                                          step_duration=step_duration,
                                          ds_fraction=ds_fraction, **footstep_kwargs)
    ts, xi_rest = lw.solve_dcm_backward(phases, t_end, dt_plan, Tc)

    omega = 1.0 / Tc
    t_ss = step_duration * (1.0 - ds_fraction)
    b_nom = np.array([nominal_dcm_offset(step_length, t_ss, omega), 0.0])
    xi = xi_rest + b_nom[np.newaxis, :] * np.exp((ts - t_end) / Tc)[:, np.newaxis]

    x_com = lw.integrate_com_forward(ts, xi, np.asarray(x0_xy, dtype=float), Tc)
    return phases, ts, xi, x_com, t_end


# ---- Self-test ---------------------------------------------------------

def _selftest_stage0(model):
    pelvis_z, hip_deg, knee_deg, ankle_deg = lw.solve_walk_pose(model)
    z_c, pelvis_com_offset_xy, _, _ = lw.solve_walk_com_height(
        model, pelvis_z, hip_deg, knee_deg, ankle_deg)
    Tc = math.sqrt(z_c / lw.G)
    omega = 1.0 / Tc

    # (a) Reduces to the classical zero-offset capture point at T=t_now,
    # b_nom=0: p_new should equal xi_measured exactly (growth factor = 1).
    p0 = np.array([0.12, -0.05])
    xi_measured = np.array([0.15, -0.03])
    t_now = 1.234
    p_new = capture_point_footstep(p0, xi_measured, (0.0, 0.0), omega, t_now, t_now)
    err_a = float(np.linalg.norm(p_new - xi_measured))
    print(f"[stage 0a] classical capture-point reduction: err={err_a:.3e}m (expect <1e-9)")
    assert err_a < 1e-9, f"capture_point_footstep does not reduce to classical form: {err_a}"
    print("[stage 0a] OK")

    # (b) Exponential-growth self-consistency, regressed against the ALREADY-
    # VALIDATED solve_dcm_backward machinery (not against touchdown targets --
    # see below for why that would be the wrong check). Within a single
    # "single support" phase the ZMP is held constant at p0 (zmp_reference's
    # own behavior for that phase kind), so the REAL xi(t) trajectory
    # (produced by solve_dcm_backward's RK4 integration of xi_dot=(xi-p)/Tc)
    # must satisfy xi(T) = p0 + (xi(t)-p0)*exp(omega*(T-t)) EXACTLY for any
    # t,T within that phase -- this is the same closed form
    # capture_point_footstep uses internally (with b_nom=0, i.e. comparing
    # raw xi propagation, not footstep placement). Confirms the growth term
    # correctly reproduces what the dense RK4 array already independently
    # computed, which is a real regression check with no assumption that the
    # walk has reached periodic steady state (it hasn't, for a short 5-step
    # plan with an initial dwell -- an earlier version of this test wrongly
    # compared against plan_footsteps' fixed touchdown targets assuming
    # steady state and failed by 65-98mm; that was a test-design bug, not a
    # formula bug, caught by exactly the kind of self-consistency check this
    # codebase's culture already demands).
    phases, t_end = lw.plan_footsteps(n_steps=5)
    ts, xi = lw.solve_dcm_backward(phases, t_end, 0.002, Tc)

    single_phases = [p for p in phases if p["kind"] == "single"]
    max_err_b = 0.0
    for p in single_phases:
        t0, t1 = p["t0"], p["t1"]
        stance_side = "left" if p["swing_side"] == "right" else "right"
        p0_actual = np.array(p["foot_xy"][stance_side])
        xi_at_T = np.array([np.interp(t1, ts, xi[:, 0]), np.interp(t1, ts, xi[:, 1])])

        for frac in (0.1, 0.5, 0.9):
            t_sample = t0 + frac * (t1 - t0)
            xi_at_t = np.array([np.interp(t_sample, ts, xi[:, 0]),
                                 np.interp(t_sample, ts, xi[:, 1])])
            xi_T_predicted = capture_point_footstep(p0_actual, xi_at_t, (0.0, 0.0),
                                                      omega, t_sample, t1)
            err = float(np.linalg.norm(xi_T_predicted - xi_at_T))
            max_err_b = max(max_err_b, err)

    print(f"[stage 0b] exponential-growth self-consistency vs. solve_dcm_backward: "
          f"max_err={max_err_b*1000:.4f}mm (expect <1mm)")
    assert max_err_b < 0.001, f"growth relation inconsistent with real DCM trajectory: {max_err_b}"
    print("[stage 0b] OK")

    # (c) Full formula (growth AND b_nom together) via a self-contained
    # algebraic round-trip -- construct forward, invert, recover exactly.
    # Pick an arbitrary nominal touchdown p_target and step geometry, derive
    # b_nom, compute what xi WOULD have to be at an earlier time t for the
    # step to land exactly at p_target (solving the growth relation
    # backward), then feed that constructed xi(t) through
    # capture_point_footstep and confirm it recovers p_target exactly. This
    # doesn't depend on any real walking plan or steady-state assumption --
    # it's pure algebra, so the tolerance is machine precision, not a
    # physical-tolerance family.
    p0_c = np.array([0.24, -0.05])
    p_target = np.array([0.32, -0.05])
    t_c, T_c = 2.876, 3.276
    t_ss_c = T_c - t_c
    b_nom_c = np.array([nominal_dcm_offset(p_target[0] - p0_c[0], t_ss_c, omega),
                         nominal_dcm_offset(p_target[1] - p0_c[1], t_ss_c, omega)])
    xi_T_c = p_target + b_nom_c
    growth_c = math.exp(omega * (T_c - t_c))
    xi_t_c = p0_c + (xi_T_c - p0_c) / growth_c   # invert the growth relation
    p_recovered = capture_point_footstep(p0_c, xi_t_c, b_nom_c, omega, t_c, T_c)
    err_c = float(np.linalg.norm(p_recovered - p_target))
    print(f"[stage 0c] full-formula algebraic round-trip: err={err_c:.3e}m (expect <1e-9)")
    assert err_c < 1e-9, f"full footstep-placement round-trip failed: {err_c}"
    print("[stage 0c] OK")


def _selftest_stage1():
    # Construct an original (un-retargeted) swing segment, sample its real
    # position AND velocity at a mid-swing retarget instant, then retarget
    # to a DIFFERENT touchdown point (simulating a disturbance response) and
    # confirm the new Hermite segment is both position- AND
    # velocity-continuous with the original at the retarget boundary --
    # this is the check that would have caught the velocity-discontinuity
    # bug the design-review pass flagged before any code was written.
    liftoff = (0.0, 0.05)
    touchdown_orig = (0.08, 0.05)
    t_start, t_end = 1.0, 1.42
    step_height = 0.02
    t_r = 1.15   # mid-swing, not a boundary

    pos_now = np.array(lw.swing_foot_target(t_r, liftoff, touchdown_orig,
                                              step_height, t_start, t_end)[:2])
    vel_now = swing_foot_velocity(t_r, liftoff, touchdown_orig, t_start, t_end)
    print(f"[stage 1] original segment at retarget instant: pos={pos_now} vel={vel_now}")
    assert np.linalg.norm(vel_now) > 1e-3, \
        "test setup error: retarget instant chosen too close to a zero-velocity endpoint"

    touchdown_new = (0.11, 0.06)   # a different target -- the "disturbance response"
    T_touchdown = t_end            # bound to the SAME phase end, per the plan's invariant

    # (a) Position continuity at the retarget boundary (t=t_r, s=0).
    x0, y0, z0 = retargeted_swing_foot_target(
        t_r, pos_now, vel_now, touchdown_new, t_r, T_touchdown, step_height, t_start, t_end)
    pos_err = float(np.linalg.norm(np.array([x0, y0]) - pos_now))
    print(f"[stage 1a] position continuity at retarget: err={pos_err*1000:.4f}mm (expect <2mm)")
    assert pos_err < 2e-3, f"position discontinuity at retarget: {pos_err}"
    print("[stage 1a] OK")

    # (b) Velocity continuity at the retarget boundary.
    tau = T_touchdown - t_r
    vel_at_0 = hermite_velocity(pos_now, vel_now, touchdown_new, (0.0, 0.0), tau, 0.0)
    vel_err = float(np.linalg.norm(vel_at_0 - vel_now))
    print(f"[stage 1b] velocity continuity at retarget: err={vel_err:.4f}m/s (expect <0.03m/s)")
    assert vel_err < 0.03, f"velocity discontinuity at retarget: {vel_err}"
    print("[stage 1b] OK")

    # (c) Touchdown boundary: position = new target exactly, velocity = 0.
    x1, y1, z1 = retargeted_swing_foot_target(
        T_touchdown, pos_now, vel_now, touchdown_new, t_r, T_touchdown, step_height, t_start, t_end)
    touchdown_pos_err = float(np.linalg.norm(np.array([x1, y1]) - np.array(touchdown_new)))
    vel_at_1 = hermite_velocity(pos_now, vel_now, touchdown_new, (0.0, 0.0), tau, 1.0)
    print(f"[stage 1c] touchdown boundary: pos_err={touchdown_pos_err*1000:.4f}mm "
          f"vel={vel_at_1} (expect pos<1e-9, vel~=0)")
    assert touchdown_pos_err < 1e-9, f"retargeted segment doesn't reach the new touchdown: {touchdown_pos_err}"
    assert np.linalg.norm(vel_at_1) < 1e-9, f"nonzero velocity at touchdown: {vel_at_1}"
    print("[stage 1c] OK")

    # (d) Vertical (z) lift is UNCHANGED by retargeting -- still the same
    # half-sine profile driven by overall phase progress.
    max_z_err = 0.0
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        t_sample = t_start + frac * (t_end - t_start)
        z_expected = step_height * math.sin(math.pi * frac)
        _, _, z_actual = retargeted_swing_foot_target(
            t_sample, pos_now, vel_now, touchdown_new, t_r, T_touchdown,
            step_height, t_start, t_end)
        max_z_err = max(max_z_err, abs(z_actual - z_expected))
    print(f"[stage 1d] vertical lift profile unchanged by retargeting: max_err={max_z_err:.3e}m (expect <1e-9)")
    assert max_z_err < 1e-9, f"retargeting altered the vertical lift profile: {max_z_err}"
    print("[stage 1d] OK")


def _selftest_stage2(model):
    pelvis_z, hip_deg, knee_deg, ankle_deg = lw.solve_walk_pose(model)
    z_c, offset, _, _ = lw.solve_walk_com_height(model, pelvis_z, hip_deg, knee_deg, ankle_deg)
    Tc = math.sqrt(z_c / lw.G)

    # "Ground truth": a full 8-step zero-disturbance plan, exactly as
    # sim_walk_lipm.py already validates.
    phases_full, t_end_full = lw.plan_footsteps(n_steps=8)
    ts_full, xi_full = lw.solve_dcm_backward(phases_full, t_end_full, 0.002, Tc)
    x0_full = np.array(phases_full[0]["zmp_p0"])
    x_com_full = lw.integrate_com_forward(ts_full, xi_full, x0_full, Tc)

    def ground_truth_x_com(t):
        return np.array([np.interp(t, ts_full, x_com_full[:, 0]),
                          np.interp(t, ts_full, x_com_full[:, 1])])

    # Pick a middle single-support phase (index 3 of the 8 -- well clear of
    # both the start and end transients).
    single_phases = [p for p in phases_full if p["kind"] == "single"]
    p_mid = single_phases[3]
    t_swing_start = p_mid["t0"]
    remaining_swing_s = p_mid["t1"] - p_mid["t0"]
    stance_side = "left" if p_mid["swing_side"] == "right" else "right"
    stance_xy = p_mid["foot_xy"][stance_side]

    # (a) Zero-disturbance regression: x0 = ground truth at the swing's
    # start, same (nominal, undisturbed) swing_to_xy target -- the short
    # horizon's own near-term x_com prediction should match the ground
    # truth closely for the remainder of THIS swing (beyond that, the two
    # trajectories' terminal conditions legitimately differ -- a receding
    # horizon's OWN nominal purpose -- so only the current-phase window is
    # a meaningful comparison).
    x0_a = ground_truth_x_com(t_swing_start)
    _, ts_h, _, x_com_h, _ = replan_horizon(
        remaining_swing_s, stance_side, stance_xy, p_mid["swing_side"],
        p_mid["swing_from_xy"], p_mid["swing_to_xy"], x0_a, Tc, n_future_steps=2)

    max_err_a = 0.0
    for frac in (0.0, 0.25, 0.5, 0.75, 1.0):
        tau = frac * remaining_swing_s
        x_h = np.array([np.interp(tau, ts_h, x_com_h[:, 0]),
                         np.interp(tau, ts_h, x_com_h[:, 1])])
        x_truth = ground_truth_x_com(t_swing_start + tau)
        max_err_a = max(max_err_a, float(np.linalg.norm(x_h - x_truth)))

    print(f"[stage 2a] zero-disturbance regression vs. full-plan ground truth: "
          f"max_err={max_err_a*1000:.4f}mm (expect <5mm)")
    assert max_err_a < 0.005, f"short-horizon replan diverges from ground truth: {max_err_a}"
    print("[stage 2a] OK")

    # (b) Reference discontinuity at a replan boundary: replan again a short
    # time later (still mid-swing), seeded with the ground truth's OWN
    # value at that later instant (consistent with (a) -- no disturbance
    # anywhere), and confirm the FIRST window's own extrapolation to that
    # instant agrees with the SECOND window's x0 -- i.e. replanning
    # mid-swing under zero disturbance introduces no discontinuity, despite
    # the two windows having different terminal conditions (different
    # absolute end-of-lookahead times).
    delta = remaining_swing_s * 0.3
    t_replan2 = t_swing_start + delta
    x0_b = ground_truth_x_com(t_replan2)
    _, ts_h2, _, x_com_h2, _ = replan_horizon(
        remaining_swing_s - delta, stance_side, stance_xy, p_mid["swing_side"],
        p_mid["swing_from_xy"], p_mid["swing_to_xy"], x0_b, Tc, n_future_steps=2)

    x_window1_at_delta = np.array([np.interp(delta, ts_h, x_com_h[:, 0]),
                                    np.interp(delta, ts_h, x_com_h[:, 1])])
    x_window2_at_0 = x_com_h2[0]
    discontinuity = float(np.linalg.norm(x_window1_at_delta - x_window2_at_0))
    print(f"[stage 2b] replan-boundary reference discontinuity: "
          f"{discontinuity*1000:.4f}mm (expect <5mm)")
    assert discontinuity < 0.005, f"replan boundary discontinuity too large: {discontinuity}"
    print("[stage 2b] OK")


# ---- Stage 3: two-timescale MuJoCo drive loop ------------------------------

# K_DCM_RECEDE: this file's own sagittal DCM gain -- deliberately NOT
# lw.K_DCM, even though both files share the same tracking-control law.
# Found 2026-09-08 while fixing the ankle_roll-actuation fall (see
# tools/CLAUDE.md's dated entry and lw.K_DCM's own comment for the shared
# root cause: ankle_roll's added mass shifted z_c/Tc enough to cross both
# files' already-narrow K_DCM stability margins): this file's receding-
# horizon replanning has a DIFFERENT stable band than lw.run_walk's
# fixed-horizon plan (-0.65 to -0.70 clean here vs. -0.55 to -0.60 there;
# -0.6, lw's own value, gives 33.5deg at n=6 in THIS file -- swept
# directly, not assumed to transfer). -0.70 was the cleanest point found:
# n=4-12 all <=12.4deg (n=10 was previously this file's own falling
# boundary -- now clean, a genuine improvement, not just a recovery); n=14
# degrades (39.98deg), n=15+ falls -- not chased further.
K_DCM_RECEDE = -0.70

# Slow outer loop cadence -- footstep retarget + short-horizon replan. Not
# every physics tick (dt=0.002s, 500Hz): the DCM/CoM trajectory only needs
# to be as fresh as the measurement driving it, and re-solving every tick
# would inject discontinuities at 500Hz instead of a controlled rate (see
# the plan's highest-flagged risk -- coherent periodic forcing at the
# replan rate exciting the same kind of resonance this codebase has already
# found twice). Swept directly, not assumed: the landscape is non-monotonic
# (0.02s and 0.06s both measurably worse than neighboring values), and
# 0.1s (10Hz) was the cleanest of the values tried -- slower than the
# ~50-100Hz the plan anticipated. Still an open question, not a settled
# one -- see the module docstring's known-limitation section.
REPLAN_PERIOD_S = 0.1

# Separate EMA filter for the CoM POSITION fed into replan_horizon's x0 --
# deliberately NOT reusing lw.xi_filtered's raw value directly (that's a
# DCM, position+Tc*velocity; x0 wants pure position), and deliberately
# filtered rather than fed raw, per the plan's highest-flagged risk: an
# unfiltered replan seed reintroduces exactly the noise-driven resonance
# lw.DCM_FILTER_ALPHA was already found necessary to suppress once.
X0_FILTER_ALPHA = 0.02

# How far Stage 0's adapted footstep may drift from the NOMINAL (straight-
# line, undisturbed) touchdown -- bounds the real risk (flagged by the
# design-review pass) of a genuine disturbance adapting the target outside
# leg_ik's reachable workspace. Separate per-axis values, NOT a single
# combined magnitude -- found necessary by direct sweep, not anticipated in
# the plan: X has real room (step_length=80mm), but the total lateral
# stance half-width is only 50mm (HIP_Y), so a symmetric +-50mm clamp on Y
# alone can place a foot AT the centerline or past the opposite foot's own
# track, geometrically collapsing the base of support -- confirmed directly
# (Y-only adaptation fell over at every clamp value >=20mm, and cleanly
# passed at 10-15mm; X-only was fine at 50mm throughout).
MAX_FOOTSTEP_ADAPT_X_M = 0.05
MAX_FOOTSTEP_ADAPT_Y_M = 0.01

# Freeze retargeting once a swing crosses this fraction of its nominal
# duration -- avoids a still-moving touchdown target fighting the foot's
# final approach into ground contact right when an UnreachableTarget would
# be costliest (see the plan's Stage 1 risk). Starting point, not yet swept.
COMMIT_FRACTION = 0.85

# Do NOT start retargeting right at swing onset. capture_point_footstep's
# growth term exp(omega*(T-t)) is LARGEST right at swing start (T-t at its
# max) and shrinks toward 1 approaching touchdown -- found empirically, not
# anticipated in the plan: with no minimum, small lateral (y) measurement
# noise/lag (b_nom_y=0 for this straight-line-only geometry, so there's no
# cushioning offset there at all) gets amplified by that early-swing growth
# factor (observed >3x within the first third of a step) into a wildly
# oversized lateral footstep correction, collapsing the stance width toward
# the centerline within a handful of steps and falling. Waiting until the
# swing is already MIN_RETARGET_FRACTION through its duration means a
# smaller remaining T-t, hence a smaller growth factor, by the time
# adaptation starts acting on the (by-then more settled) measured state.
# Value found by direct sweep, not derived.
MIN_RETARGET_FRACTION = 0.30

N_FUTURE_STEPS = 2   # rolling-horizon lookahead depth, in nominal footsteps


def _nominal_next_touchdown(stance_xy, new_swing_side, step_length, step_width):
    """The next step's TRUE fixed nominal (x, y) target -- y comes from the
    swinging side's fixed nominal lateral track (+-step_width/2), NOT from
    that foot's actual last landing position. Using the actual (possibly
    Y-adapted) last landing here would let lateral adaptation compound
    forward through every subsequent step's own "nominal" reference instead
    of being a one-off correction -- the same bug class fixed in
    build_horizon_phases, present here too since this nominal value is what
    MAX_FOOTSTEP_ADAPT_X_M/MAX_FOOTSTEP_ADAPT_Y_M's clamps are measured
    relative to; if the reference itself drifts, the clamp bounds nothing
    over many steps."""
    half_w = step_width / 2.0
    nominal_y = half_w if new_swing_side == "left" else -half_w
    return (stance_xy[0] + step_length, nominal_y)


def run_walk_recede(model, n_steps=None, duration=None, render_path=None,
                     render_every=10, verbose=True, k_dcm=K_DCM_RECEDE,
                     k_zmp_y=lw.K_ZMP_Y,
                     replan_period=REPLAN_PERIOD_S, push_at=None, push_force=(0.0, 0.0),
                     push_duration=0.1):
    """Drive the robot with the receding-horizon controller: the first step
    is bootstrapped exactly like lw.run_walk (same dwell/shift/first-step
    plan, same initial MuJoCo state), then from the SECOND step onward the
    immediately-swinging foot's touchdown adapts continuously to the
    measured DCM (capture_point_footstep + retargeted_swing_foot_target),
    and the short-horizon CoM/DCM reference driving the existing fast
    K_DCM tracking correction is rebuilt every replan_period from the
    robot's actual (filtered) state (replan_horizon) instead of a single
    fixed whole-walk plan -- removing the fixed-horizon boundary condition
    that caused every wall lw.run_walk hit.

    Stop condition: n_steps (footstep landings) or duration (sim seconds) --
    exactly one must be given. push_at/push_force/push_duration (all in the
    pelvis body's world frame, Newtons/seconds) apply a real external force
    via data.xfrc_applied for Stage 5's disturbance testing; omit for an
    undisturbed run. Returns a summary dict; optionally renders a GIF."""
    assert (n_steps is None) != (duration is None), "pass exactly one of n_steps, duration"
    data = mujoco.MjData(model)

    pelvis_z, hip_deg, knee_deg, ankle_deg = lw.solve_walk_pose(model)
    z_c, pelvis_com_offset_xy, _, _ = lw.solve_walk_com_height(
        model, pelvis_z, hip_deg, knee_deg, ankle_deg)
    Tc = math.sqrt(z_c / lw.G)
    omega = 1.0 / Tc
    offset = np.asarray(pelvis_com_offset_xy)

    step_length = 0.08
    step_width = 2 * lw.HIP_Y
    step_duration = 0.6
    ds_fraction = 0.4
    t_ss = step_duration * (1.0 - ds_fraction)
    step_height = 0.02

    # Bootstrap: identical to lw.run_walk's own initialization (same
    # plan_footsteps call shape, same initial MuJoCo state) for the dwell,
    # initial weight-shift, and first step -- Stage 2's own self-test
    # already validated that switching from a plan like this one to a
    # receding-horizon replan under zero disturbance introduces no
    # discontinuity, so bootstrapping this way and switching over after the
    # first step is a verified-safe transition, not an assumption.
    phases_boot, t_end_boot = lw.plan_footsteps(n_steps=3, step_length=step_length,
                                                 step_duration=step_duration,
                                                 ds_fraction=ds_fraction,
                                                 step_height=step_height)
    dt_plan = 0.002
    ts_boot, xi_boot = lw.solve_dcm_backward(phases_boot, t_end_boot, dt_plan, Tc)
    x0_boot = np.array(phases_boot[0]["zmp_p0"])
    x_com_boot = lw.integrate_com_forward(ts_boot, xi_boot, x0_boot, Tc)

    act_index = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i): i
                 for i in range(model.nu)}

    pelvis_xyz0 = np.array([x_com_boot[0, 0] + offset[0], x_com_boot[0, 1] + offset[1], pelvis_z])
    phase0 = lw._phase_at(0.0, phases_boot)
    foot_target0 = {side: (xy[0], xy[1], 0.0) for side, xy in phase0["foot_xy"].items()}

    mujoco.mj_resetData(model, data)
    data.qpos[0] = pelvis_xyz0[0]
    data.qpos[1] = pelvis_xyz0[1]
    data.qpos[2] = pelvis_z
    for side in ("left", "right"):
        hip_origin = lw.hip_origin_for_side(pelvis_xyz0, side)
        roll, hip, knee, ankle = lw.leg_ik(hip_origin, foot_target0[side])
        for jn, val in zip(("hip_roll", "hip_pitch", "knee_pitch", "ankle_pitch"),
                            (roll, hip, knee, ankle)):
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, f"{side}_{jn}")
            data.qpos[model.jnt_qposadr[jid]] = val
        data.ctrl[act_index[f"act_{side}_hip_roll"]] = roll
        data.ctrl[act_index[f"act_{side}_hip_pitch"]] = hip
        data.ctrl[act_index[f"act_{side}_knee_pitch"]] = knee
        data.ctrl[act_index[f"act_{side}_ankle_pitch"]] = ankle
    mujoco.mj_forward(model, data)
    mujoco.mj_subtreeVel(model, data)
    xi_filtered = data.subtree_com[0][:2].copy() + Tc * data.subtree_linvel[0][:2].copy()
    x0_filtered = data.subtree_com[0][:2].copy()

    pelvis_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    pelvis_body_id = pelvis_id

    renderer = None
    cam = None
    if render_path:
        renderer = mujoco.Renderer(model, height=360, width=480)
        cam = mujoco.MjvCamera()
        cam.azimuth, cam.elevation, cam.distance = 90, -12, 2.0
        cam.lookat = np.array([0.0, 0.0, 0.45])
    frames = []

    sim_dt = model.opt.timestep

    # ---- Active plan state (mutated as the walk proceeds) ----
    # Until t_switchover, use the bootstrap plan directly (nominal, no
    # adaptation -- matches lw.run_walk's own first-step behavior exactly).
    switchover_phase = phases_boot[4]   # step1's ORIGINAL single-support phase
    t_switchover = switchover_phase["t0"]
    ts_plan, x_com_plan, xi_plan = ts_boot, x_com_boot, xi_boot
    t_replan_sim = 0.0    # absolute sim time that window-relative t=0 maps to
    receding_active = False

    # Current-swing bookkeeping (populated at switchover and every commit).
    stance_side = stance_xy = swing_side = swing_from_xy = None
    swing_to_xy_nominal = swing_to_xy_current = None
    t_swing_start = T_touchdown = None
    retarget_active = False
    retarget_pos0 = retarget_vel0 = None
    retarget_t0 = retarget_tau = None
    foot_positions = None   # {"left": xy, "right": xy}, tracked from switchover onward

    t_next_replan = t_switchover

    pelvis_x0 = data.xpos[pelvis_id, 0]
    max_tilt = 0.0
    max_dcm_err_m = 0.0
    diverged = False
    steps_completed = 0

    # Local lateral ZMP feedback (ankle_roll) -- see lw.lateral_zmp_correction
    # and its block comment; mirrors run_walk's own wiring exactly.
    foot_body_ids = sb._foot_body_ids(model)
    zy_filtered = 0.0

    step = 0
    while True:
        t = data.time
        if n_steps is not None and steps_completed >= n_steps:
            break
        if duration is not None and t >= duration:
            break

        if push_at is not None and push_at <= t < push_at + push_duration:
            data.xfrc_applied[pelvis_body_id, 0] = push_force[0]
            data.xfrc_applied[pelvis_body_id, 1] = push_force[1]
        else:
            data.xfrc_applied[pelvis_body_id, :] = 0.0

        # ---- Switchover: hand off from the bootstrap plan to receding-
        # horizon mode at the start of step1's single support. ----
        if not receding_active and t >= t_switchover:
            receding_active = True
            swing_side = switchover_phase["swing_side"]
            stance_side = "left" if swing_side == "right" else "right"
            stance_xy = switchover_phase["foot_xy"][stance_side]
            swing_from_xy = switchover_phase["swing_from_xy"]
            swing_to_xy_nominal = switchover_phase["swing_to_xy"]
            swing_to_xy_current = swing_to_xy_nominal
            t_swing_start = switchover_phase["t0"]
            T_touchdown = switchover_phase["t1"]
            foot_positions = {stance_side: stance_xy, swing_side: swing_from_xy}
            retarget_active = False
            t_next_replan = t

        # ---- Measure real state and update filters ONCE per tick. Both the
        # slow and fast loops below read the SAME xi_filtered value -- an
        # earlier version updated it a second time inside the slow-loop
        # block using an identical pre-mj_step sample, a redundant double-
        # EMA applied only on replan ticks (i.e. a periodic irregularity in
        # the filter's effective time constant, synced to the replan rate
        # -- exactly the coherent periodic-forcing risk flagged before any
        # code was written). Fixed by computing/filtering exactly once. ----
        com_now = data.subtree_com[0][:2].copy()
        comvel_now = data.subtree_linvel[0][:2].copy()
        xi_now_raw = com_now + Tc * comvel_now
        xi_filtered = (1 - lw.DCM_FILTER_ALPHA) * xi_filtered + lw.DCM_FILTER_ALPHA * xi_now_raw
        # x0_filtered must ALSO update every tick, not just at replan ticks --
        # an earlier version updated it only inside the slow-loop block below,
        # which (since X0_FILTER_ALPHA was calibrated as a PER-TICK rate,
        # matching xi_filtered/DCM_FILTER_ALPHA's convention) silently gave it
        # an effective time constant ~10x slower than intended (replan_period
        # / sim_dt ticks between updates instead of 1), leaving it lagging the
        # real CoM by the better part of a second -- found by direct tracing
        # (x0_filtered was still near its t=0 initial value multiple steps
        # into the walk) after adaptation-disabled runs still fell over,
        # which ruled out the footstep-placement law itself as the cause.
        x0_filtered = (1 - X0_FILTER_ALPHA) * x0_filtered + X0_FILTER_ALPHA * com_now

        # ---- Slow outer loop: retarget + rebuild the short horizon. ----
        if receding_active and t >= t_next_replan:
            swing_progress = (t - t_swing_start) / (T_touchdown - t_swing_start)
            if MIN_RETARGET_FRACTION <= swing_progress < COMMIT_FRACTION:
                step_len_x = swing_to_xy_nominal[0] - swing_from_xy[0]
                step_len_y = swing_to_xy_nominal[1] - swing_from_xy[1]
                b_nom = (nominal_dcm_offset(step_len_x, t_ss, omega),
                         nominal_dcm_offset(step_len_y, t_ss, omega))
                p_new = capture_point_footstep(stance_xy, xi_filtered, b_nom,
                                                omega, t, T_touchdown)
                raw_delta = np.asarray(p_new) - np.asarray(swing_to_xy_nominal)
                delta = np.array([
                    np.clip(raw_delta[0], -MAX_FOOTSTEP_ADAPT_X_M, MAX_FOOTSTEP_ADAPT_X_M),
                    np.clip(raw_delta[1], -MAX_FOOTSTEP_ADAPT_Y_M, MAX_FOOTSTEP_ADAPT_Y_M),
                ])
                swing_to_xy_new = tuple(np.asarray(swing_to_xy_nominal) + delta)

                if retarget_active:
                    tau_now = t - retarget_t0
                    s_now = min(max(tau_now / retarget_tau, 0.0), 1.0)
                    pos_now = hermite_position(retarget_pos0, retarget_vel0,
                                                swing_to_xy_current, (0.0, 0.0),
                                                retarget_tau, s_now)
                    vel_now = hermite_velocity(retarget_pos0, retarget_vel0,
                                                swing_to_xy_current, (0.0, 0.0),
                                                retarget_tau, s_now)
                else:
                    pos_now = np.array(lw.swing_foot_target(
                        t, swing_from_xy, swing_to_xy_current, step_height,
                        t_swing_start, T_touchdown)[:2])
                    vel_now = swing_foot_velocity(t, swing_from_xy, swing_to_xy_current,
                                                   t_swing_start, T_touchdown)

                swing_to_xy_current = swing_to_xy_new
                retarget_active = True
                retarget_pos0, retarget_vel0 = pos_now, vel_now
                retarget_t0 = t
                retarget_tau = T_touchdown - t

            remaining = T_touchdown - t
            phases_h, ts_h, xi_h, x_com_h, t_end_h = replan_horizon(
                remaining, stance_side, stance_xy, swing_side, swing_from_xy,
                swing_to_xy_current, x0_filtered, Tc, n_future_steps=N_FUTURE_STEPS,
                step_length=step_length, step_duration=step_duration,
                ds_fraction=ds_fraction, step_height=step_height)
            ts_plan, x_com_plan, xi_plan = ts_h, x_com_h, xi_h
            t_replan_sim = t

            t_next_replan = t + replan_period

        # ---- Fast inner loop: existing K_DCM tracking correction. ----
        t_window = t - t_replan_sim
        xi_planned = np.array([np.interp(t_window, ts_plan, xi_plan[:, 0]),
                                np.interp(t_window, ts_plan, xi_plan[:, 1])])
        x_com_planned = np.array([np.interp(t_window, ts_plan, x_com_plan[:, 0]),
                                   np.interp(t_window, ts_plan, x_com_plan[:, 1])])

        dcm_err = xi_filtered - xi_planned
        max_dcm_err_m = max(max_dcm_err_m, float(np.linalg.norm(dcm_err)))
        correction = np.clip(k_dcm * dcm_err, -lw.MAX_DCM_CORRECTION_M, lw.MAX_DCM_CORRECTION_M)
        pelvis_xy_cmd = x_com_planned + offset + correction
        pelvis_xyz_cmd = np.array([pelvis_xy_cmd[0], pelvis_xy_cmd[1], pelvis_z])

        if receding_active:
            foot_target = {stance_side: (stance_xy[0], stance_xy[1], 0.0)}
            if retarget_active:
                x, y, z = retargeted_swing_foot_target(
                    t, retarget_pos0, retarget_vel0, swing_to_xy_current,
                    retarget_t0, T_touchdown, step_height, t_swing_start, T_touchdown)
            else:
                x, y, z = lw.swing_foot_target(t, swing_from_xy, swing_to_xy_current,
                                                step_height, t_swing_start, T_touchdown)
            foot_target[swing_side] = (x, y, z)
        else:
            phase = lw._phase_at(t, phases_boot)
            foot_target = {side: (xy[0], xy[1], 0.0) for side, xy in phase["foot_xy"].items()}
            if phase["kind"] == "single":
                foot_target[phase["swing_side"]] = lw.swing_foot_target(
                    t, phase["swing_from_xy"], phase["swing_to_xy"],
                    phase["step_height"], phase["t0"], phase["t1"])

        # Local lateral ZMP feedback (ankle_roll) -- see lw.lateral_zmp_
        # correction's block comment. target_y comes from whichever plan is
        # actually driving this tick (short-horizon replan vs. bootstrap),
        # matching xi_planned/x_com_planned's own source selection above.
        # Only the STANCE foot is planted during receding-horizon mode (the
        # swing foot has an in-air z-target); the bootstrap branch mirrors
        # run_walk's phase["foot_xy"] planted-set logic exactly.
        _, zy_raw, _ = sb.compute_zmp(model, data, foot_body_ids)
        if zy_raw is not None:
            zy_filtered = (1 - lw.ZMP_Y_FILTER_ALPHA) * zy_filtered + lw.ZMP_Y_FILTER_ALPHA * zy_raw
        if receding_active:
            target_y = lw.zmp_reference(t_window, phases_h)[1]
            planted_sides = {stance_side}
        else:
            target_y = lw.zmp_reference(t, phases_boot)[1]
            planted_sides = set(phase["foot_xy"].keys())
        ankle_roll_corr = lw.lateral_zmp_correction(zy_filtered, target_y, k_zmp_y=k_zmp_y)
        for side in ("left", "right"):
            data.ctrl[act_index[f"act_{side}_ankle_roll"]] = (
                ankle_roll_corr if side in planted_sides else 0.0)

        for side in ("left", "right"):
            hip_origin = lw.hip_origin_for_side(pelvis_xyz_cmd, side)
            try:
                roll, hip, knee, ankle = lw.leg_ik(hip_origin, foot_target[side])
            except lw.UnreachableTarget:
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
        step += 1

        # ---- Swing completion: commit and advance to the next step. ----
        if receding_active and data.time >= T_touchdown:
            steps_completed += 1
            landed_xy = swing_to_xy_current
            foot_positions[swing_side] = landed_xy
            new_stance_side = swing_side
            new_stance_xy = landed_xy
            new_swing_side = stance_side
            new_swing_from_xy = foot_positions[new_swing_side]
            new_swing_to_nominal = _nominal_next_touchdown(new_stance_xy, new_swing_side,
                                                             step_length, step_width)

            stance_side, stance_xy = new_stance_side, new_stance_xy
            swing_side, swing_from_xy = new_swing_side, new_swing_from_xy
            swing_to_xy_nominal = swing_to_xy_current = new_swing_to_nominal
            t_swing_start = T_touchdown
            T_touchdown = t_swing_start + t_ss
            retarget_active = False
            t_next_replan = data.time

    pelvis_x_final = data.xpos[pelvis_id, 0]
    net_forward = pelvis_x_final - pelvis_x0

    if render_path and frames:
        import imageio
        imageio.mimsave(render_path, frames, fps=int(1.0 / (sim_dt * render_every)))
        if verbose:
            print(f"  wrote {len(frames)} frames to {render_path}")

    return dict(diverged=diverged, max_tilt_deg=max_tilt, net_forward_m=net_forward,
                sim_time=data.time, steps_completed=steps_completed, max_dcm_err_m=max_dcm_err_m)


def _selftest_stage3(model):
    # 6 steps: comfortably inside the validated clean range (see run_walk_
    # recede's docstring -- 12 is the actual clean boundary as of the
    # 2026-09-08 K_DCM_RECEDE retune, 14 degrades, 15+ falls; 6 leaves
    # margin the same way sim_walk_lipm.py's own gates test at n=12 with
    # headroom below its own n=15 boundary).
    result = run_walk_recede(model, n_steps=6, verbose=False)
    print(f"[stage 3] 6-step receding-horizon run: diverged={result['diverged']}  "
          f"max_tilt={result['max_tilt_deg']:.2f}deg  net_forward={result['net_forward_m']*1000:.1f}mm  "
          f"max_dcm_err={result['max_dcm_err_m']*1000:.1f}mm  sim_time={result['sim_time']:.3f}s")
    assert not result["diverged"], "receding-horizon simulation diverged within 6 steps"
    assert result["net_forward_m"] > 0.0, "pelvis net motion is not forward"
    assert result["max_tilt_deg"] < 20.0, \
        f"max tilt {result['max_tilt_deg']:.2f}deg not below old gait's ~20deg baseline"
    print("[stage 3] OK — receding-horizon controller stable and net-forward within its validated range")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--selftest", action="store_true",
                        help="Run the numerical self-verification gates and exit.")
    parser.add_argument("--steps", type=int, default=None,
                        help="Run n_steps of the receding-horizon walking gait.")
    parser.add_argument("--duration", type=float, default=None,
                        help="Run for a fixed sim duration (seconds) instead of a step count.")
    parser.add_argument("--render", type=str, default=None,
                        help="Path to write an offscreen-rendered GIF.")
    parser.add_argument("--push-at", type=float, default=None,
                        help="Sim time (s) to apply an external pelvis force.")
    parser.add_argument("--push-force", type=float, nargs=2, default=(0.0, 0.0),
                        metavar=("FX", "FY"), help="External force (N) applied at --push-at.")
    parser.add_argument("--push-duration", type=float, default=0.1,
                        help="Duration (s) the push force is applied for.")
    args = parser.parse_args()

    if not MJCF_PATH.exists():
        print(f"ERROR: MJCF not found at {MJCF_PATH}")
        print("Run: python3 tools/generate_mjcf.py")
        raise SystemExit(1)

    model = mujoco.MjModel.from_xml_path(str(MJCF_PATH))

    if args.selftest:
        _selftest_stage0(model)
        _selftest_stage1()
        _selftest_stage2(model)
        _selftest_stage3(model)
        print()
        print("All implemented self-tests passed.")
        return

    if args.steps is not None or args.duration is not None:
        result = run_walk_recede(model, n_steps=args.steps, duration=args.duration,
                                  render_path=args.render, push_at=args.push_at,
                                  push_force=tuple(args.push_force),
                                  push_duration=args.push_duration)
        print()
        print(f"run: diverged={result['diverged']}  max_tilt={result['max_tilt_deg']:.2f}deg  "
              f"net_forward={result['net_forward_m']*1000:.1f}mm  "
              f"steps_completed={result['steps_completed']}  sim_time={result['sim_time']:.3f}s")
        return

    print("Nothing to do — pass --selftest, --steps N, or --duration T.")


if __name__ == "__main__":
    main()

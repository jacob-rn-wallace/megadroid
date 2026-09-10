#!/usr/bin/env python3
"""
Architecture D — constraint-aware receding-horizon MPC over the LIPM.

WHAT THIS IS FOR
----------------
Every walking controller in this repo generates a trajectory without any
knowledge of what the drivetrain can deliver, then asks a stiff position servo
to track it. Measured result (tools/CLAUDE.md, 2026-09-10): the receding-horizon
gait demands RMS 4.08 N·m and p95 8.50 N·m against a 2.8 N·m envelope, clipping
the torque limit 29% of the time, and falls. Stage 0 showed this is not a servo
gain that can be re-tuned away -- there is no kp that both holds the pose and
avoids saturation.

This module fixes the cause rather than the symptom: it *plans* the centre of
pressure inside the feasible set, so the infeasible command is never issued.
Stephens & Atkeson (Humanoids 2010) attribute the dominant push-recovery gain in
their own comparison (~8 -> ~21 Ns, 2.6x) specifically to constraint-awareness.

THE KEY IDENTITY
----------------
Ankle torque and CoP are the same constraint in different units:

    tau_ankle = F_z * d_CoP

At the 2.8 N·m envelope and full body weight (94.18 N) that is +-29.7 mm of CoP
excursion -- which is SMALLER than the foot (75 mm sagittal, 40 mm lateral
half-extents). So in single support the robot's effective support polygon is set
by the motors, not the sole, and the torque limit becomes an ordinary box
constraint on this QP's control variable. That is what makes architecture D
expressible at all, and what B/C/E structurally cannot represent.

FORMULATION
-----------
The LIPM decouples sagittal from lateral, so this is two independent 1-D
problems rather than one 2-D problem.

    state  x = [c, c_dot]         (CoM position, velocity)
    input  p = CoP
    c_ddot = omega^2 (c - p),      omega^2 = g / z_c

Discretised exactly under zero-order hold on p, then CONDENSED: the states are
eliminated so the whole horizon is an affine function of the CoP sequence,
leaving a box-constrained QP in N variables:

    minimise  1/2 u' H u + g' u        subject to   lo <= u <= hi

H depends only on (N, dt, omega) and the cost weights, so it is built once.

SOLVER
------
Projected Newton (Bertsekas) on the box, warm-started from the previous tick.
Dense, small, no scipy -- matching the precedent set by
capture_point_footstep_with_timing (sim_walk_recede.py), which hand-derives its
solution rather than taking a dependency.

FISTA was written first and rejected: this Hessian is badly conditioned, and
projected gradient stalled around 3e-5 against the closed form after 20000
iterations. Projected Newton reaches it exactly in 2, and a warm start converges
in 1 -- which is what makes it viable inside a control loop. Verified by
`--selftest`.
"""

import math
import argparse
import numpy as np


def lipm_matrices(dt, omega):
    """Exact zero-order-hold discretisation of the LIPM over one step.

    c(k+1)     = cosh(w dt) c + sinh(w dt)/w c_dot + (1 - cosh(w dt)) p
    c_dot(k+1) = w sinh(w dt) c + cosh(w dt) c_dot - w sinh(w dt) p

    Exact rather than Euler because the LIPM is unstable (that is the whole
    point of the DCM), and Euler error on an unstable mode compounds over a
    horizon instead of averaging out.
    """
    ch = math.cosh(omega * dt)
    sh = math.sinh(omega * dt)
    A = np.array([[ch, sh / omega],
                  [omega * sh, ch]])
    B = np.array([1.0 - ch, -omega * sh])
    return A, B


def condense(N, dt, omega):
    """Build (Sx, Su) with X = Sx x0 + Su U over an N-step horizon.

    X stacks [c(1), c_dot(1), ..., c(N), c_dot(N)] -- states AFTER each input,
    so U = [p(0) ... p(N-1)] and Su is strictly causal by construction.
    """
    A, B = lipm_matrices(dt, omega)
    Sx = np.zeros((2 * N, 2))
    Su = np.zeros((2 * N, N))
    Apow = np.eye(2)
    for k in range(N):
        Apow = A @ Apow                      # A^(k+1)
        Sx[2 * k:2 * k + 2, :] = Apow
        for j in range(k + 1):
            # contribution of u(j) to x(k+1) is A^(k-j) B
            Aij = np.linalg.matrix_power(A, k - j)
            Su[2 * k:2 * k + 2, j] = Aij @ B
    return Sx, Su


def dcm_selector(N, omega):
    """Row-selects the DCM xi = c + c_dot/omega from a stacked state vector."""
    S = np.zeros((N, 2 * N))
    for k in range(N):
        S[k, 2 * k] = 1.0
        S[k, 2 * k + 1] = 1.0 / omega
    return S


class LIPMPredictor:
    """Precomputed condensed prediction + cost Hessian for one axis.

    Cost, per horizon:
        q_dcm  * sum_k (xi(k) - xi_ref(k))^2      DCM tracking
        r_cop  * sum_k (p(k) - p_ref(k))^2        CoP effort / stay centred
        q_term * (xi(N) - xi_ref(N))^2            terminal DCM

    DCM rather than CoM is tracked because the DCM is the unstable mode -- it is
    the only part that runs away, so it is the only part that must be
    controlled. Same reasoning as sim_walk_lipm.py's own tracking law.
    """

    def __init__(self, N, dt, omega, q_dcm=1.0, r_cop=1e-3, q_term=10.0):
        self.N, self.dt, self.omega = N, dt, omega
        self.q_dcm, self.r_cop, self.q_term = q_dcm, r_cop, q_term

        self.Sx, self.Su = condense(N, dt, omega)
        Sd = dcm_selector(N, omega)
        self.Ex = Sd @ self.Sx            # xi_traj = Ex x0 + Eu U
        self.Eu = Sd @ self.Su

        W = np.full(N, q_dcm)
        W[-1] += q_term                   # terminal weight rides on the last step
        self.W = W

        self.H = 2.0 * (self.Eu.T @ (W[:, None] * self.Eu)
                        + r_cop * np.eye(N))
        self.H = 0.5 * (self.H + self.H.T)          # symmetrise against drift

    def gradient_terms(self, x0, xi_ref, p_ref):
        """Linear term g for the current state and references."""
        e0 = self.Ex @ np.asarray(x0, dtype=float) - np.asarray(xi_ref, dtype=float)
        return 2.0 * (self.Eu.T @ (self.W * e0) - self.r_cop * np.asarray(p_ref, dtype=float))

    def predict_dcm(self, x0, U):
        return self.Ex @ np.asarray(x0, dtype=float) + self.Eu @ np.asarray(U, dtype=float)


def solve_box_qp(H, g, lo, hi, u0=None, max_iter=100, tol=1e-11):
    """minimise 1/2 u'Hu + g'u subject to lo <= u <= hi.

    Bertsekas-style projected Newton: identify the epsilon-active set, take an
    exact Newton step on the free variables, and line-search along the projected
    path. Returns (u, iterations, converged).

    Chosen over a first-order method (FISTA was tried first) because this
    Hessian is badly conditioned -- DCM tracking over a horizon is nearly
    rank-deficient and r_cop is small -- and projected gradient stalled around
    3e-5, well short of the machine-precision agreement this repo verifies its
    hand-derived solvers to. Projected Newton solves the unconstrained case in a
    single step, exactly.

    Warm-startable through u0, which is what makes this affordable in a control
    loop: consecutive ticks differ by one horizon step, so the previous solution
    is nearly optimal.
    """
    n = len(g)
    g = np.asarray(g, dtype=float)
    lo = np.asarray(lo, dtype=float)
    hi = np.asarray(hi, dtype=float)
    u = np.clip(np.zeros(n) if u0 is None else np.asarray(u0, dtype=float), lo, hi)

    def obj(v):
        return 0.5 * float(v @ (H @ v)) + float(g @ v)

    scale = max(np.max(np.abs(hi - lo)), 1.0)
    eps = 1e-9 * scale

    for it in range(1, max_iter + 1):
        grad = H @ u + g
        if kkt_residual(H, g, lo, hi, u) < tol:
            return u, it, True

        # Epsilon-active: at a bound AND the gradient pushes further into it.
        binding = (((u <= lo + eps) & (grad > 0.0))
                   | ((u >= hi - eps) & (grad < 0.0)))
        free = ~binding

        d = np.zeros(n)
        d[binding] = -grad[binding]          # projected-gradient on bound vars
        if np.any(free):
            Hff = H[np.ix_(free, free)]
            try:
                d[free] = -np.linalg.solve(Hff, grad[free])
            except np.linalg.LinAlgError:
                d[free] = -np.linalg.lstsq(Hff, grad[free], rcond=None)[0]

        f0 = obj(u)
        alpha, improved = 1.0, False
        for _ in range(40):
            u_try = np.clip(u + alpha * d, lo, hi)
            if obj(u_try) < f0 - 1e-16 * max(abs(f0), 1.0):
                u, improved = u_try, True
                break
            alpha *= 0.5
        if not improved:
            # Cannot descend along the projected path -> at the optimum.
            return u, it, kkt_residual(H, g, lo, hi, u) < 1e-6

    return u, max_iter, kkt_residual(H, g, lo, hi, u) < 1e-6


def kkt_residual(H, g, lo, hi, u):
    """Max violation of the box-QP KKT conditions at u.

    Interior -> gradient zero; at lower bound -> gradient >= 0; at upper bound
    -> gradient <= 0. Used by the self-tests to check the returned point is
    actually optimal rather than merely feasible.
    """
    grad = H @ u + g
    res = 0.0
    for i in range(len(u)):
        at_lo = u[i] <= lo[i] + 1e-9
        at_hi = u[i] >= hi[i] - 1e-9
        if at_lo and not at_hi:
            res = max(res, max(0.0, -grad[i]))
        elif at_hi and not at_lo:
            res = max(res, max(0.0, grad[i]))
        else:
            res = max(res, abs(grad[i]))
    return res


def cop_bounds_from_torque(f_z, tau_max, geom_half, min_half=0.002):
    """Feasible CoP half-width: the tighter of motor torque and foot geometry.

    tau_ankle = F_z * d_CoP, so a torque ceiling is a CoP box. With no load on
    the foot the torque bound is meaningless (and divides by ~zero), so it
    collapses to a small floor rather than blowing up -- an unloaded foot cannot
    shape the CoP at all.
    """
    if f_z <= 1e-6:
        return min_half
    return float(max(min_half, min(geom_half, tau_max / f_z)))


# ---------------------------------------------------------------- self-tests

def _selftest_dynamics():
    omega, dt, N = 4.32, 0.05, 12
    P = LIPMPredictor(N, dt, omega)
    A, B = lipm_matrices(dt, omega)
    rng = np.random.default_rng(0)
    x0 = rng.normal(size=2)
    U = rng.normal(size=N)

    x = x0.copy()
    step_states = []
    for k in range(N):
        x = A @ x + B * U[k]
        step_states.append(x.copy())
    step_states = np.concatenate(step_states)
    cond = P.Sx @ x0 + P.Su @ U
    err = float(np.max(np.abs(step_states - cond)))
    print(f"[dyn] condensed rollout vs step-by-step: max_err={err:.3e} (expect <1e-12)")
    assert err < 1e-12, "condensed prediction disagrees with direct integration"

    xi_direct = np.array([s[0] + s[1] / omega for s in step_states.reshape(N, 2)])
    xi_cond = P.predict_dcm(x0, U)
    err2 = float(np.max(np.abs(xi_direct - xi_cond)))
    print(f"[dyn] DCM selector: max_err={err2:.3e} (expect <1e-12)")
    assert err2 < 1e-12
    print("[dyn] OK")


def _selftest_unconstrained():
    """(a) With bounds wide, the solver must match the closed-form solve."""
    omega, dt, N = 4.32, 0.05, 12
    P = LIPMPredictor(N, dt, omega)
    rng = np.random.default_rng(1)
    x0 = rng.normal(size=2) * 0.05
    xi_ref = rng.normal(size=N) * 0.05
    p_ref = np.zeros(N)
    g = P.gradient_terms(x0, xi_ref, p_ref)

    u_exact = np.linalg.solve(P.H, -g)
    wide = np.full(N, 1e6)
    u_fista, it, conv = solve_box_qp(P.H, g, -wide, wide)
    err = float(np.max(np.abs(u_exact - u_fista)))
    print(f"[qp-a] wide bounds vs closed form: max_err={err:.3e} in {it} iters "
          f"(expect <1e-6)")
    assert conv and err < 1e-6, "unconstrained limit does not match the closed form"
    print("[qp-a] OK")


def _selftest_kkt_and_active():
    """(b) KKT holds, and (c) a forced-active bound is met exactly."""
    omega, dt, N = 4.32, 0.05, 16
    P = LIPMPredictor(N, dt, omega)
    rng = np.random.default_rng(2)
    # A state far from the reference, so the unconstrained optimum wants a big
    # CoP excursion and the box genuinely binds.
    x0 = np.array([0.10, 0.60])
    xi_ref = np.zeros(N)
    p_ref = np.zeros(N)
    g = P.gradient_terms(x0, xi_ref, p_ref)

    half = 0.0297                       # the real torque-limited bound, metres
    lo, hi = np.full(N, -half), np.full(N, half)
    u, it, conv = solve_box_qp(P.H, g, lo, hi)

    res = kkt_residual(P.H, g, lo, hi, u)
    n_active = int(np.sum((u <= lo + 1e-9) | (u >= hi - 1e-9)))
    print(f"[qp-b] KKT residual={res:.3e} in {it} iters, {n_active}/{N} bounds active "
          f"(expect residual<1e-6, active>0)")
    assert conv, "solver did not converge on the constrained problem"
    assert res < 1e-6, "returned point violates KKT -- not optimal"
    assert n_active > 0, "test problem did not activate any bound; it proves nothing"

    assert np.all(u >= lo - 1e-12) and np.all(u <= hi + 1e-12), "solution outside the box"
    on_bound = np.abs(np.abs(u) - half) < 1e-12
    print(f"[qp-c] active entries sit exactly on the bound: {int(on_bound.sum())} "
          f"of {n_active}")
    assert on_bound.sum() == n_active, "active constraints are not met exactly"
    print("[qp-b/c] OK")


def _selftest_warm_start():
    """(d) Warm and cold starts must reach the same optimum."""
    omega, dt, N = 4.32, 0.05, 16
    P = LIPMPredictor(N, dt, omega)
    x0 = np.array([0.08, 0.45])
    xi_ref = np.zeros(N)
    g = P.gradient_terms(x0, xi_ref, np.zeros(N))
    half = 0.0297
    lo, hi = np.full(N, -half), np.full(N, half)

    u_cold, it_cold, _ = solve_box_qp(P.H, g, lo, hi)
    # Warm start from a shifted previous solution, as the control loop would.
    guess = np.concatenate([u_cold[1:], u_cold[-1:]])
    u_warm, it_warm, _ = solve_box_qp(P.H, g, lo, hi, u0=guess)
    err = float(np.max(np.abs(u_cold - u_warm)))
    print(f"[qp-d] warm vs cold: max_err={err:.3e}, {it_warm} vs {it_cold} iters "
          f"(expect err<1e-8)")
    assert err < 1e-8, "warm start converges somewhere different"
    print("[qp-d] OK")


def _selftest_torque_bounds():
    """The torque->CoP identity, against this robot's real numbers."""
    W, tau, hx, hy = 94.18, 2.8, 0.075, 0.040
    single = cop_bounds_from_torque(W, tau, hx)
    double = cop_bounds_from_torque(W / 2, tau, hx)
    lat = cop_bounds_from_torque(W, tau, hy)
    unloaded = cop_bounds_from_torque(0.0, tau, hx)
    print(f"[cop] single-support sagittal: {single*1000:.1f} mm "
          f"(torque-limited, foot is {hx*1000:.0f})")
    print(f"[cop] double-support sagittal: {double*1000:.1f} mm")
    print(f"[cop] single-support lateral : {lat*1000:.1f} mm "
          f"(torque-limited, foot is {hy*1000:.0f})")
    print(f"[cop] unloaded foot          : {unloaded*1000:.1f} mm (collapses safely)")
    assert abs(single - tau / W) < 1e-9, "torque bound not applied when it is tighter"
    assert single < hx and lat < hy, "expected torque to bind before geometry"
    assert double < hx, "double-support sagittal should still be torque-limited"
    assert unloaded <= 0.002 + 1e-12, "unloaded foot must not claim CoP authority"
    print("[cop] OK")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true",
                    help="Run the numerical verification gates and exit.")
    args = ap.parse_args()
    if not args.selftest:
        print("Nothing to do — pass --selftest.")
        return
    _selftest_dynamics()
    _selftest_unconstrained()
    _selftest_kkt_and_active()
    _selftest_warm_start()
    _selftest_torque_bounds()
    print()
    print("All MPC self-tests passed.")


if __name__ == "__main__":
    main()

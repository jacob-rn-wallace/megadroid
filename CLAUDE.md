# CLAUDE.md — Megadroid Claude Code Context

This file tells Claude Code what it needs to know to work on this repository
effectively. Read it before making any changes.

---

## What This Project Is

**Megadroid** is an open-hardware bipedal humanoid robot targeting hobbyist
ownership. The design philosophy is ASIMO/Hubo-class walking capability at
hobbyist desktop PC cost (~$800–1500), using 775 brushed DC motors at 24V,
high gear ratios, and ZMP-based balance control — validated by the Hubo robot
as an existence proof.

The project is developed by a single builder using a V-model-inspired dual-track
process (Product Track + Infrastructure Track). The current version is **v0.5.1**.
Completed stages: **P1** (Authoritative Design Definition) and **P2** (Kinematic
& Structural Validation). Active stage: **P3** (Simulation-First MVS Validation).

---

## P3 Active Work — Read Before Starting Simulation Tasks

**Simulation framework:** MuJoCo (installed: `pip install mujoco`).

**What exists and works:**
- `design/mass.yaml` — authoritative link mass estimates (8.90 kg total)
- `tools/generate_mjcf.py` — generates `simulation/mujoco/megadroid_mvs.xml`;
  also excludes self-collision pairs (pelvis↔thigh, and the non-adjacent
  pairs among the stacked torso_pitch/torso_roll/torso links) where
  collision-proxy geometry inevitably overlaps by construction
- `tools/sim_load_test.py` — model loads cleanly (15 bodies, 11 actuators) ✓
- `tools/sim_static_pose.py` — **P3 fixed-base milestone, passing.** Welds
  the pelvis to the world (like a test-stand bolt), holds the nominal
  standing pose (knees bent 12°), and validates the weld's vertical
  reaction force converges to the robot's exact computed weight (87.31 N,
  0.0% error) with joint torques symmetric left/right. Ground contact is
  intentionally excluded for this test — welding the pelvis while feet
  also touch ground would make support statically indeterminate. The weld
  is injected into the model in memory at load time; the checked-in MJCF
  (needed for the future floating-base ZMP controller) is never modified
  on disk. Run: `python3 tools/sim_static_pose.py`
- `tools/sim_zmp_balance.py` — **P3 dynamic-balance milestone, passing.**
  Floating-base (unmodified MJCF, no weld): a proportional ankle-pitch
  controller measures ZMP from actual foot ground-contact points and
  drives it toward the support-polygon centroid. Stable indefinitely
  (tested to 30s simulated) with <1° pelvis tilt. Two non-obvious fixes
  were required, both documented in the script's module docstring and
  worth knowing before touching this file:
    1. The naive nominal pose (bend only the knee) leaves the pelvis/torso
       column well forward of the foot — a large structural lean an
       ankle-only strategy can never correct. `NOMINAL_POSE` is instead a
       proper crouch (hip_pitch and ankle_pitch solved by FK to counter-
       rotate the knee bend) — with this pose alone, *no active control*,
       the robot is already stable. The controller's real job is the
       small residual/dynamic drift, which is what ankle strategy is
       actually good for.
    2. ZMP as one point per foot (the foot body's own origin) can't see
       heel/toe load shifting, since both feet share the same x-offset —
       it's blind to an oncoming tip. ZMP is computed from the actual
       contact points instead, then low-pass filtered (EMA, α=0.02)
       before being fed to the controller — the raw per-step signal is
       noisy (MuJoCo's box-plane contact activates different corner
       subsets frame to frame) and unfiltered feedback destabilizes an
       otherwise-stable pose.
  Known limitation: this ankle-only controller does not extend how large
  a disturbance the robot can absorb before falling beyond what the
  balanced passive pose already tolerates — that would need a hip/torso
  strategy too, out of scope for ankle-pitch alone. Run:
  `python3 tools/sim_zmp_balance.py` (add `--baseline` to compare against
  the uncontrolled/passive case).
- `tools/sim_walk_gait.py` — **P3 walking-gait milestone, partial: three
  steps validated, not yet a sustained gait.** Floating-base, no weld.
  Reuses sim_zmp_balance.py's balanced pose and sagittal ankle-pitch ZMP
  loop, plus a second control mechanism the standing controller didn't
  need — lateral (hip_roll) weight transfer, since there's no
  ankle_roll to shift ZMP sideways. `python3 tools/sim_walk_gait.py`
  passes: ~20° peak tilt, no fall, ~214mm total swing-foot forward
  progress over 3 steps (add `--render out.gif` for a visual — MuJoCo's
  offscreen renderer works in this environment; `imageio` + `ffmpeg`
  handle encoding). `--steps 4` reliably fails. Seven real bugs/dead-
  ends were found along the way (all documented in the script's module
  docstring, worth reading before touching gait code): two sign-
  convention bugs, the swing leg inheriting the stance leg's hip_roll
  shift, the stance leg's forward-drive needing a smaller magnitude
  than the swing leg's, a faster swing being more stable than a slower
  one, phase-dependent ankle gain (fixed 1→2 steps), and — the fix that
  took it from 2→3 steps — a *partial* (not full) CoM-balance
  correction on the stance leg's knee (`StanceKneeTable`,
  `PARTIAL_BALANCE_ALPHA`). One finding stands independent of further
  tuning: the pelvis nets slightly backward every step even though the
  foot itself lands forward (a real, reproduced recoil effect, not
  noise) — the pass criterion is deliberately based on foot placement,
  not pelvis translation.

  **Why a 4th step fails, and the two dead ends on the way to the
  actual fix:** the first theory blamed leg asymmetry breaking the
  hip_roll weight-shift trick — tested directly and wrong, the trick
  still works fine with very asymmetric legs. The real cause: a growing
  static CoM offset (foot no longer centered under the pelvis) as
  hip_pitch advances each step and only the level-foot ankle relation
  tracks it, not balance (~1cm of offset per degree of deviation — a
  single 10° swing already creates ~10cm, the same scale of problem the
  very first naive standing pose had). Solving this FULLY (recompute
  knee, not just ankle, so the foot is exactly balanced at every
  hip_pitch) does fix the static problem — verified working from
  hip_pitch -40° to +5° — but was found to make the FIRST TWO steps'
  dynamics measurably worse despite fixing what it targeted (likely: a
  more bent knee is a shorter, stiffer pendulum, and the existing gains
  no longer fit its faster dynamics). The actual fix was a *partial*
  correction (`PARTIAL_BALANCE_ALPHA=0.9`, retuned alongside the ankle
  and roll gains) — this improved the first two steps AND got a third
  working, not a trade of one for the other. A subtlety that cost real
  time getting here: comparing runs that let a known-to-fail extra step
  execute (and report its doomed tilt) against a run that stops at the
  validated count makes any change look like a regression — always
  compare at the same step count, and confirm a no-op setting exactly
  reproduces the committed baseline before trusting any diff from it.
- `simulation/mujoco/megadroid_mvs.xml` — full MuJoCo scene with dynamics,
  position actuators (kp=150), foot contact geometry, ground plane
- `tools/sim_walk_lipm.py` — **P3 smooth-walking rearchitecture, fifteen
  steps validated with real MuJoCo dynamics and DCM tracking control.**
  Supersedes `sim_walk_gait.py`'s reactive phase-based state machine for
  the goal of SMOOTH, ASIMO-like walking (kept as reference/fallback):
  that gait's validated 3 steps look like stumbling on video — pelvis
  nets ~213mm BACKWARD even as the swing foot lands forward, because tilt
  is only corrected reactively after it's already large, with no planned
  CoM trajectory. This file replaces that with a real model-based
  generator — footstep plan → piecewise ZMP reference → CoM trajectory
  via the DCM (Divergent Component of Motion / Capture Point) method,
  RK4-integrated → per-leg inverse kinematics (new to this codebase) →
  MuJoCo drive loop with real, EMA-filtered DCM TRACKING CONTROL — the
  standard technique behind ASIMO/HRP/Valkyrie-generation walking
  generators. `python3 tools/sim_walk_lipm.py --selftest` runs the full
  numerical self-verification chain (IK round-trip, footstep-plan
  invariants, implied-ZMP self-consistency, swing-trajectory endpoints,
  full-plan FK/joint-limit verification, a 12-step MuJoCo drive);
  `--steps 15` (or `--steps N --render out.gif`) runs the real thing.
  Validated: **≤6.4° peak tilt, flat across every step count from 3 to
  15, and sustained NET-FORWARD pelvis motion throughout**. `--steps 16`
  is borderline (19.6° peak — under the old gait's ~20° baseline, but
  visibly worse than the flat 3–15 range); `--steps 17` fails outright.
  This is the THIRD control-loop iteration this file has used, and both
  of the first two hit a hard wall that gain tuning alone could not fix
  — each real fix was an architectural or measurement change, not a
  bigger gain:
    1. The first version used a small ad-hoc ZMP-error → hip_roll/
       ankle_pitch trim on top of open-loop trajectory replay. It got 3
       steps working but hit a hard 4-step wall that a wide P/I/D gain
       sweep (P: 1.5–9.0, I: 3–20, D: 0.05–0.8) could not move — the
       telltale sign was a *growing* lateral oscillation (not a steady
       offset a stronger correction would close), the signature of an
       uncorrected open-loop-unstable mode. That diagnosis was correct:
       the DCM's own dynamics are open-loop unstable by construction
       ("divergent" is literally in the name), and a ZMP-error-only trim
       never measures the CoM velocity that determines whether the
       divergence is accelerating — exactly the missing half of the
       well-established fix (Kajita et al.'s 2003 ZMP preview control on
       HRP-2; Capture-Point/DCM tracking control after Pratt 2006 and
       Englsberger et al.), confirmed directly against a paper in this
       project's own reference library (Zhu & Thomas 2023, "Mechanical
       Design of a Biped Robot FORREST and an Extended Capture-Point-
       Based Walking Pattern Generator," Section 6.1) before implementing
       it. Full derivation, the control law, and a real sign-convention
       finding from getting it working (this file's position-controlled
       architecture needed the OPPOSITE correction sign from the paper's
       force/ZMP-domain law — confirmed by direct measurement, not
       derivation) are documented above `K_DCM`. This fix took the
       validated range from 3 to 9 steps, all landing on an identical
       9.0° peak tilt.
    2. That still left a SECOND, much slower wall around step 10 — the
       same kind of failure (a growing oscillation crossing threshold at
       a fixed elapsed time, confirmed by testing n_steps=10/12/15/20 and
       finding the >15° onset at an identical t=6.93s regardless of total
       plan length) but a different root cause: the DCM tracking law
       itself is stable given a clean measurement, but the raw per-step
       capture point computed from `mj_subtreeVel` is noisy, slowly
       pumping the same kind of resonance back in through the correction
       loop — the same category of lesson `sim_zmp_balance.py` already
       learned about raw contact-force ZMP, rediscovered here for a
       different signal. EMA-filtering that measurement (`alpha=0.02`,
       found by direct sweep — the same value `sim_zmp_balance.py`
       converged on for ZMP, not copied on faith) pushed the clean range
       from 9 steps to 15. Documented above `DCM_FILTER_ALPHA`.
    3. Even within that fully-validated range, the user watched a render
       and reported it looked visibly "stumbly" despite never falling —
       correct: the gait had a real, measured ~6.9° peak-tilt rocking and
       ~14mm vertical pelvis bob every single step, not a rendering
       artifact. Neither was a K_DCM/DCM_FILTER_ALPHA tuning gap (both
       were already near their local-optimum values); the fix was a gait
       *timing* change — raising `plan_footsteps`' `ds_fraction` default
       from 0.3 to 0.4 (more double-support time, proportionally less
       single-support time per step) — which nearly halved both (peak
       tilt 6.9→5.5°, bob 14.2→9.3mm), at an *initial* cost of the clean
       range dropping from 15 steps to 13 (a K_DCM/DCM_FILTER_ALPHA
       re-sweep at 0.4 alone could not recover it). Documented above
       `plan_footsteps`.
    4. That cost was then fully recovered by continuing to investigate
       the same step-13/14+ wall (same diagnostic signature as #2 — the
       >15° tilt onset for n_steps=16/18/20 all landed at an identical
       t≈9.6s regardless of total plan length): tightening
       `MAX_DCM_CORRECTION_M` (the correction's safety clamp, 0.08 →
       0.04) dropped a 16-step run's peak tilt from 94.7° (a fall) to
       19.6°, and pushed the clean range at `ds_fraction=0.4` back up to
       15 steps — with a slightly *better* tilt profile than either
       prior setting alone (a flat ~5.9–6.4° across the whole range, not
       just at a few step counts). Net effect of fixes #3+#4 together: a
       smoother gait with no step-count cost at all. The stability
       landscape isn't a smooth gradient, though — 0.035 gave a fall
       (87.8°) sandwiched between 0.03's and 0.04's clean results — so
       this is a real, somewhat fragile margin, documented above
       `MAX_DCM_CORRECTION_M`.
  Two more real findings surfaced getting the first 3 steps working at
  all (both still load-bearing, documented in comments above
  `WALK_AX_MARGIN_M` / `plan_footsteps`): (1) at `sim_zmp_balance.py`'s
  pure-standing crouch (knee bent only 12°), the leg already sits at
  99.45% of max reach with zero horizontal offset — real walking
  excursions (measured up to 114mm) need a *deeper* walking-specific
  crouch (knee ~27°, pelvis ~1.4cm lower); (2) the swing foot's lift
  height had to be reduced to 20mm (from an initial 30mm) — a higher lift
  shortens the swing leg's hip-to-foot distance enough to push
  `ankle_pitch` past its own -30° limit, given this design's tight reach
  margin.
- `tools/sim_walk_recede.py` — **P3 receding-horizon rearchitecture, eight
  steps validated — a real architectural change, not another tuning
  pass, prompted by the user asking whether chasing higher step counts in
  a *fixed*-horizon plan was actually how real bipeds learn to walk
  indefinitely (it isn't).** `sim_walk_lipm.py` plans one fixed trajectory
  for an entire N-step walk with a hard "rest at the very end" boundary
  condition and footstep placement that never responds to measured state
  — structurally incapable of walking indefinitely or genuinely reacting
  to a push. This file replaces that with real receding-horizon
  replanning (a short window continuously re-solved from actual measured
  state, reusing `sim_walk_lipm.py`'s own DCM/CoM integration machinery
  unchanged) and capture-point footstep placement — the immediately-
  swinging foot's touchdown adapts to the measured DCM via a closed-form
  formula confirmed directly against a paper in the project's own
  reference library (Khadiv et al., via Roux 2024, "MPC-RL-based bipedal
  robot control") before implementing it, not derived from memory.
  `python3 tools/sim_walk_recede.py --selftest` gates the footstep-
  placement formula, the swing-retarget Hermite blend, the short-horizon
  replan, and a 6-step drive; `--steps 8` runs the real thing; `--push-at
  T --push-force FX FY --push-duration D` applies a real external force
  to the pelvis for disturbance testing. **Validated: 8 steps clean
  (≤6.9° peak tilt), 9 borderline, 10+ falls** — honestly, a *more
  modest* range than `sim_walk_lipm.py`'s 15 steps, a real tradeoff, not
  a strict improvement. Three real bugs were found and fixed while
  building this (documented in full in the module docstring, worth
  reading before touching this file): a redundant double-EMA filter
  update on replan ticks; an EMA filter alpha calibrated as a per-tick
  rate but applied only at replan ticks, giving it an effective time
  constant ~10× slower than intended (found by noticing the filtered CoM
  estimate was still near its startup value several steps into a walk
  that was inexplicably falling); and lateral footstep adaptation
  compounding forward through every subsequent "nominal" footstep instead
  of being a one-off correction, because the nominal reference itself was
  wrongly inherited from a previous adapted landing rather than the true
  fixed geometric track. **Known limitation, not yet resolved:** even
  with footstep adaptation disabled entirely, pure receding-horizon
  replanning of an otherwise-fixed nominal gait still degrades in the
  same 8–15 step range — this rules out the footstep-placement law as
  the sole cause. Tracing the gap between each short-horizon replan's own
  prediction and the next replan's real measured state shows a genuinely
  *growing* discontinuity (single-digit mm early in a walk, hundreds of
  mm by the time it falls), and a wide sweep of the replan cadence,
  lookahead depth, and `K_DCM` didn't find a combination extending
  cleanly past ~12 steps. **One specific, literature-grounded hypothesis
  was tested and ruled out**: research (same paper the footstep-placement
  law came from) showed real receding-horizon controllers terminate each
  short window at the DCM offset for *continuing* the gait, not at rest —
  this file's rolling horizon originally ended every window with a
  fabricated "come to rest" dwell, a real, literature-confirmed design
  flaw. Fixed with a closed-form correction (documented in
  `replan_horizon`'s docstring) that measurably improved the Stage 2
  self-test numbers but did **not** resolve the wall — the growing-
  discontinuity pattern is essentially unchanged afterward, ruling out
  mis-specified terminal cost as the *dominant* cause (the fix is correct
  and worth keeping regardless). Leading remaining suspect: the short-
  horizon model's own constant-`Tc` point-mass assumption may
  systematically mismatch real MuJoCo dynamics in a way that compounds
  specifically over repeated replans — this project already found one
  concrete instance of exactly this class of gap (`sim_walk_lipm.py`
  documents a commanded weight shift settling at only ~70% of target
  under plain position control, an error the idealized model doesn't
  predict), and a plain EMA filter has no independent reference to detect
  that kind of bias, unlike e.g. a Kalman filter fusing kinematics with
  IMU data — a real state-estimation paper in the project's reference
  library uses exactly that, but this hasn't been confirmed as the actual
  cause here, only flagged as the most likely remaining lead. **Push
  recovery was tested, not just claimed, and the result is an honest open
  finding**: sweeping external pelvis forces (5–30N, 0.1s, scaled to this
  design's actual ~8.9kg mass) mid-walk, footstep adaptation does not yet
  show a clear, consistent advantage over the plain fast-loop correction
  alone — the mechanism is verified mathematically correct and does shift
  the footstep target in response to a real disturbance, but that doesn't
  yet translate into measurably better recovery at the magnitudes tested.

**Where work stopped:**
Five P3 simulation milestones are done: fixed-base static load
validation, the floating-base ZMP ankle-pitch standing controller, three
validated steps of quasi-static (stumbling) walking, fifteen validated
steps of smooth fixed-horizon LIPM/DCM-planned walking
(`sim_walk_lipm.py`), and eight validated steps of the new receding-
horizon controller with capture-point footstep placement
(`sim_walk_recede.py`). Neither indefinite walking nor a demonstrated
disturbance-rejection advantage is done yet — see that file's own bullet
above and its module docstring for the full, honest account: even with
footstep adaptation disabled, pure receding-horizon replanning alone
still degrades in the same 8-15 step range as `sim_walk_lipm.py`'s own
walls, via a *growing* (not constant) discontinuity between each short-
horizon replan's own prediction and the next replan's real measured
state — ruling out the footstep-placement law as the sole cause, and
ruling out both "replanning itself is destabilizing" (disabling it after
the first window fails even faster) and a wide `REPLAN_PERIOD_S`/
`N_FUTURE_STEPS`/`K_DCM` sweep as fixes. One specific, literature-grounded
hypothesis (mis-specified terminal cost — the rolling horizon was ending
each window "at rest" instead of at the DCM offset for continuing to
walk, per Roux 2024 eq. 24) WAS tried and fixed, and measurably improved
the Stage 2 self-test numbers, but did not resolve the wall — ruled out
as the dominant cause, not just untried. Remaining candidates (none
started, no decision made yet): smoothly blending the fast loop's
reference across a replan boundary instead of hard-switching to the new
window (addresses the symptom); checking whether the short-horizon
model's own constant-Tc point-mass assumption systematically mismatches
real MuJoCo dynamics in a way that specifically compounds over many
replans, possibly needing a more principled state estimator (e.g. a
Kalman filter fusing kinematics + IMU, as used in a paper already in the
project's reference library) rather than the current plain EMA, which
has no independent reference to detect that kind of systematic bias
(addresses a hypothesized root cause, unconfirmed). Push recovery itself
(once a longer validated range exists) also needs real tuning work: the
current footstep-adaptation clamps/gains don't yet show a measurable
recovery advantage over the plain tracking correction at the pelvis-force
magnitudes tested (5-30N).
Other next directions (none started, no decision made yet):
  - Extend either ZMP controller (or the receding-horizon one) to reject
    external disturbances more robustly, which will likely need a
    hip/torso strategy layered on top of the ankle strategy (see
    sim_zmp_balance.py's known limitation).
  - Move toward P4 physical prototyping — premature before sustained,
    disturbance-tolerant walking is validated, since walking is likely to
    stress-test mechanical dimensions and motor torque budgets that
    standing alone doesn't touch.

---

## Licensing

- **Software** (code, firmware, tools): Apache License 2.0 (`LICENSE`)
- **Hardware** (mechanical design, PCBs, schematics): CERN Open Hardware Licence
  Version 2 — Strongly Reciprocal (`LICENSE-HARDWARE.txt`)

---

## Authority Model — Read This First

### Single Source of Truth

```
design/*.yaml        ← ONLY place where numeric values and design decisions live
```

All other documents are derived views or static artifacts. Never introduce a
numeric design value anywhere except `design/*.yaml`.

### Document Hierarchy

| File | Role | Edit directly? |
|------|------|---------------|
| `design/*.yaml` | Authoritative source of truth | **Yes — always edit here** |
| `SPEC.md` | Derived system specification | **No — rehydrate only** |
| `MECH.md` | Derived mechanical description | **No — rehydrate only** |
| `README.md` | Derived repository overview | **No — rehydrate only** |
| `BOM.csv` | Derived cost/parts list | **No — rehydrate only** |
| `simulation/urdf/megadroid_mvs.urdf` | Generated URDF model | **No — regenerate only** |
| `simulation/mujoco/megadroid_mvs.xml` | Generated MuJoCo scene | **No — regenerate only** |
| `PHILOSOPHY.md` | Static — project philosophy | **No — static artifact** |
| `PROCESS.md` | Static — workflow definition | **No — static artifact** |
| `REHYDRATE.md` | Static — rehydration process description | **No — static artifact** |
| `docs/P2_VALIDATION.md` | Static — milestone report | **No — static artifact** |

**`SPEC.md` is the highest authority for the MVS configuration.** `MECH.md` and
`BOM.csv` must never contradict it.

If you find a discrepancy between a derived doc and `design/*.yaml`, **the YAML
wins**. Fix the YAML; rehydrate; commit in that order.

---

## Authoritative Design Files

```
design/
  joints.yaml       Joint definitions, limits, nominal poses, MVS flags
  geometry.yaml     Structural constants (link lengths, shaft diameters, etc.)
  kinematics.yaml   Axis directions, sign conventions, angle references
  mass.yaml         Link mass estimates for simulation (nominal; verified in P6)
  actuation.yaml    Motor and drivetrain parameters (stub — not yet populated)
  power.yaml        Power system parameters (stub — not yet populated)
  .meta.yaml        Schema/metadata for the design directory
```

### Key Design Constants (do not change without an explicit design revision)

- **MVS DOF:** 11 actuated joints
  - Per leg (×2): `hip_pitch`, `hip_roll`, `knee_pitch`, `ankle_pitch`
  - Torso: `torso_pitch`, `torso_roll`, `torso_yaw`
- **Actuator:** 775 brushed DC motors, 24V
- **Control mode:** ZMP-based
- **Standard joint output shaft:** 12mm steel
- **Twin-rail inner face spacing:** 100mm (structural — matches Hubo-style)
- **HTD belt:** 5M profile, 15mm width
- **Double-shear bearing support** required at all joints; single-shear prohibited
- **Angle units:** radians in computation; degrees in human-readable documents

**The design is frozen.** Do not propose changes to DOF count, link lengths,
coordinate conventions, base frame, control mode, motor type, or sensor strategy
unless the user explicitly initiates a design revision.

### Coordinate Conventions (from `kinematics.yaml`)

- **Handedness:** right-hand rule
- **Axes:** Z-up, X-forward, Y-left (standard robotics convention)
- **Base frame:** `pelvis_center` — geometric midpoint between left and right hip
  roll joint centers, aligned with global frame in nominal standing pose
- **Rotation sign:** positive rotation is counterclockwise when looking along the
  positive axis direction (right-hand rule)

---

## Common Tasks (Quick Reference)

| Task | Command |
|------|---------|
| Pre-commit check (rehydrate + validate + status) | `python3 tools/preflight.py` |
| Regenerate all derived docs | `python3 tools/rehydrate_all.py` |
| Run all validators | `python3 tools/validate_all.py` |
| Regenerate URDF | `python3 tools/generate_urdf.py` |
| Verify URDF dimensions | `python3 tools/verify_urdf_dimensions.py` |
| Regenerate MuJoCo scene | `python3 tools/generate_mjcf.py` |
| MuJoCo load test | `python3 tools/sim_load_test.py` |
| P3 static pose validation (fixed base) | `python3 tools/sim_static_pose.py` |
| P3 ZMP balance validation (floating base) | `python3 tools/sim_zmp_balance.py` |
| P3 walking gait validation (3 steps, stumbling) | `python3 tools/sim_walk_gait.py` |
| P3 smooth walking validation (15 steps, LIPM/DCM) | `python3 tools/sim_walk_lipm.py --steps 15` |
| P3 receding-horizon walking validation (8 steps) | `python3 tools/sim_walk_recede.py --steps 8` |
| Visualize robot structure | `python3 tools/visualize_urdf.py` |
| Analyze joint workspace | `python3 tools/analyze_workspace.py` |
| Print DOF summary | `python3 tools/generate_spec_dof.py` |

All tools can be run from any directory — they locate the repo root via
`Path(__file__).resolve()`.

---

## Mandatory Workflow

### Before committing any change

1. **Edit only `design/*.yaml`** for any design change.
2. **Run preflight** — rehydrates docs, runs all validators, and shows what changed:
   ```bash
   python3 tools/preflight.py
   ```
   Or run steps separately:
   - Rehydrate: `python3 tools/rehydrate_all.py`
   - Validate: `python3 tools/validate_all.py` (runs `validate_geometry.py`,
     `validate_no_geometry_literals.py`, `validate_dof_consistency.py`)
3. **If URDF-relevant geometry changed**, regenerate and verify the URDF:
   ```bash
   python3 tools/generate_urdf.py
   python3 tools/verify_urdf_dimensions.py
   ```
4. **Commit YAML changes first**, derived docs second — never in the same commit.

### Commit order (non-negotiable)

```
Commit 1: design/*.yaml changes            ← always first
Commit 2: SPEC.md MECH.md README.md        ← rehydrated derived docs
Commit 3: simulation/urdf/megadroid_mvs.urdf  ← if URDF was regenerated
```

### Never do

- Edit `SPEC.md`, `MECH.md`, or `README.md` directly
- Add a numeric design value to a derived doc
- Commit derived docs ahead of the YAML that produced them

---

## Toolchain

```
tools/
  preflight.py                  Rehydrate + validate + show git status (run before every commit)
  rehydrate_all.py              Run all rehydrators (primary entry point)
  rehydrate_spec.py             Regenerate SPEC.md
  rehydrate_mech.py             Regenerate MECH.md
  rehydrate_readme_structure.py Regenerate README.md structure
  validate_all.py               Run all validators (primary entry point)
  validate_dof_consistency.py   Check DOF counts are consistent across YAML
  validate_no_geometry_literals.py  Catch hardcoded numbers in derived docs
  validate_geometry.py          Geometry-specific consistency checks
  generate_urdf.py              Generate URDF from YAML (includes inertials via mass.yaml)
  verify_urdf_dimensions.py     Validate URDF dimensions against YAML
  generate_mjcf.py              Generate MuJoCo MJCF scene from YAML
  sim_load_test.py              MuJoCo load/sanity check (run after generate_mjcf.py)
  sim_static_pose.py            P3 fixed-base static load validation — passing
  sim_zmp_balance.py            P3 floating-base ZMP ankle-pitch balance controller — passing
  sim_walk_gait.py              P3 walking gait (reactive, stumbles) — 3 steps validated, 4th fails; superseded by sim_walk_lipm.py
  sim_walk_lipm.py               P3 smooth walking (LIPM/DCM + filtered DCM tracking control) — 15 steps validated, 16th borderline (not yet root-caused)
  sim_walk_recede.py             P3 receding-horizon walking + capture-point footstep placement — 8 steps validated; indefinite walking/push-recovery advantage not yet demonstrated (see module docstring)
  visualize_urdf.py             matplotlib-based 3D visualizer (macOS-compatible)
  analyze_workspace.py          Workspace sampling via forward kinematics
  generate_spec_dof.py          Generate DOF table markdown from joints.yaml
  check_joints.py               Joint-level sanity checks

templates/
  SPEC.md.j2                    Jinja2 template for SPEC.md
  MECH.md.j2                    Jinja2 template for MECH.md
```

Dependencies:

| Purpose | Packages |
|---------|----------|
| Rehydration and validation (required) | `pyyaml jinja2` |
| URDF verification | `numpy` |
| Visualization and workspace analysis | `matplotlib numpy` |
| MuJoCo simulation | `mujoco` |

```bash
pip install pyyaml jinja2 numpy matplotlib mujoco
```

---

## Repository Structure

```
megadroid/
  design/           Authoritative YAML (single source of truth)
  templates/        Jinja2 templates for derived docs
  tools/            Rehydration, validation, generation, visualization scripts
  simulation/
    urdf/           Generated URDF files (megadroid_mvs.urdf)
    mujoco/         Generated MuJoCo scenes (megadroid_mvs.xml)
  docs/             Static milestone reports (e.g., P2_VALIDATION.md)
  firmware/         Embedded motor/joint control — RP2350-CAN boards (not yet populated)
  software/         High-level control, gait planning, dev tools (not yet populated)
  SPEC.md           Derived system specification
  MECH.md           Derived mechanical description
  README.md         Derived repository overview
  BOM.csv           Derived bill of materials
  PHILOSOPHY.md     Static project philosophy
  PROCESS.md        Static workflow definition
  REHYDRATE.md      Static rehydration process description
  CLAUDE.md         This file
  LICENSE           Apache 2.0 (software)
  LICENSE-HARDWARE.txt  CERN-OHL-S v2 (hardware)
```

---

## CI

Two GitHub Actions workflows run on push/PR when design files, templates, tools,
or derived docs change:

- **`validate.yml`** — runs `validate_dof_consistency.py` and
  `validate_no_geometry_literals.py` when design files or derived docs change
- **`rehydration-check.yml`** — regenerates derived docs and diffs them against
  committed versions; fails if content diverges (timestamps are allowed to differ)

Both must pass before merging. If CI fails, fix `design/*.yaml` and rehydrate —
do not patch the derived docs directly.

---

## What "Authoritative" Means

A design decision is only authoritative when it is:
1. Written into `design/*.yaml`
2. Reflected in `SPEC.md` (via rehydration)
3. Committed to the repository

Conversational reasoning — including anything suggested in a chat session — is
**never** authoritative until it clears all three steps above.

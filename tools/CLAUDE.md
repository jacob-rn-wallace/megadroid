# tools/CLAUDE.md — P3 Simulation Work Context

This file is loaded when Claude Code works with files under `tools/`.
Root-level project context (design authority model, licensing, common
tasks, mandatory workflow) is in the repo-root `CLAUDE.md` — read that
first if you haven't already.

---

## Reference Material

`reference-material/` (gitignored, not in this repo — check `git status`
if unsure it's there) holds copied research/provenance PDFs curated from
the personal paper library at `~/Documents/Scientific papers/`. Nothing
in the build reads these; they exist to check design and control
decisions against primary sources instead of general knowledge, matching
this project's own "verify before trusting" discipline (see the
sim_walk_lipm.py/sim_walk_recede.py entries below for how this already
happens in practice — e.g. "confirmed against Zhu & Thomas 2023...
Section 6.1"). Cite the specific paper/section whenever a decision is
checked against one.

```
reference-material/
  humanoid-robotics/      ~70 papers on bipedal/humanoid robots, incl.
                           HRP-2/3/4, LOLA, TORO, iCub, WABIAN-2, ASIMO,
                           WALK-MAN, and other major reference platforms
    KAIST/                 HUBO lineage (KHR-2, KHR-3/HUBO, DRC-HUBO) —
                           this project's own stated existence proof
    Waseda/                 WABIAN lineage — human-like biped walking
  force-torque-sensors/   9 papers on 6-axis F/T sensor design,
                           calibration, and ZMP-measurement use in
                           humanoids — relevant now that
                           design/sensors.yaml specifies a per-foot F/T
                           sensor not yet modeled in simulation
  actuators/               MIT Cheetah proprioceptive actuator design
                           (the foot's own stated inspiration, SPEC.md
                           Sec 5) + a low-cost modular actuator paper
  MPC-RL-based bipedal robot control.pdf   Roux 2024 thesis — source of
                           the capture-point footstep-placement law used
                           in sim_walk_recede.py
```

**Priority reading**, triaged 2026-09-08 against the problems below (full
per-file rationale not kept — re-triage if this list goes stale):

- *Growing-discontinuity wall / state estimation* — `Humanoid Robot LOLA`
  (near-identical sensor suite to megadroid's, minus the IMU it lacks);
  `On the Hardware Design and Control Architecture of the Humanoid Robot
  Kangaroo` (attributes real landing failures to state-estimation/force-
  mapping issues); `Tsinghua Hephaestus 2019` (DCM+VSP with a CoM
  estimator fed by F/T + gyro + joint angles); `Discussion on the
  Stiffness of the Drive Chain in the Legs of Biped Robots` (plausible
  cause of sim_walk_lipm.py's own ~70%-of-commanded weight-shift finding).
- *HUBO-style local-feedback architecture* — `KAIST/Development of
  Humanoid Robots in HUBO Laboratory, KAIST` (primary source for the
  local-sensor control scheme behind the KHR-3/HUBO paper); `Compliance
  Control for Stabilizing the Humanoid on the Changing Slope` (admittance
  control for position-controlled actuators + foot F/T sensors — this
  project's own actuation architecture); `KAIST/System Design and Dynamic
  Walking of Humanoid Robot KHR-2` (direct KHR-3/HUBO precursor).
- *F/T sensor physical design* (spec'd in design/sensors.yaml, unbuilt) —
  `force-torque-sensors/A compact six-axis force:torque sensor using
  photocouplers for impact robustness` (KAIST, journal not arXiv version
  — 56mm×18mm, <$250, built for legged-robot feet); `force-torque-
  sensors/Multi-Axis Force:Torque Sensors for Measuring Zero-Moment Point
  in Humanoid Robots A Review`; `force-torque-sensors/Overload Protection
  Mechanism for 6-axis Force:Torque Sensor` (Waseda — matches the
  axial-only load path already in geometry.yaml's foot_stack).
- *Actuator philosophy counter-argument* — `actuators/Proprioceptive
  Actuator Design in the MIT Cheetah` argues low-gear-ratio + motor-
  current sensing eliminates the need for F/T sensors entirely — the
  opposite tradeoff from megadroid's committed high-gear-ratio + F/T +
  no-current-sensing design. Worth reading to stress-test that choice,
  not to copy it.
- *Stair climbing* (stated MVS goal, SPEC.md Sec 2.2, not yet started) —
  `Climbing of Stairs of an Autonomous Bipedal Robot`; `Walking of the
  iCub humanoid robot in different scenarios` (quantitative stair
  success-rate data); `Overview of the torque-controlled humanoid robot
  TORO` (also has real stair data, up to 5cm).
- *Closest philosophical/mechanical analog* — `MEVITA Open-Source Bipedal
  Robot Assembled from E-Commerce Components via Sheet Metal Welding`.

Curated, not comprehensive: the source library has ~150 other papers
(car design, computing history, unrelated hardware) not copied here as
irrelevant to a walking humanoid. If a research question doesn't fit
what's indexed above, the fuller library is still at
`~/Documents/Scientific papers/` (outside this repo) — check there
before concluding nothing exists on a topic.

---

**Simulation framework:** MuJoCo (installed: `pip install mujoco`).

**What exists and works:**
- `design/mass.yaml` — authoritative link mass estimates (9.60 kg total)
- `tools/generate_mjcf.py` — generates `simulation/mujoco/megadroid_mvs.xml`;
  also excludes self-collision pairs (pelvis↔thigh, and the non-adjacent
  pairs among the stacked torso_pitch/torso_roll/torso links) where
  collision-proxy geometry inevitably overlaps by construction
- `tools/sim_load_test.py` — model loads cleanly (17 bodies, 13 actuators — the
  usual 11 plus ankle_roll, now temporarily actuated on both legs; see the
  dated entry below `sim_walk_recede.py`) ✓
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
  cause here, only flagged as the most likely remaining lead. **A second,
  more specific untested hypothesis** (found reading Kajita et al.'s
  *Introduction to Humanoid Robotics* directly — the actual primary
  source behind the ZMP preview control already cited secondhand above,
  not derived from memory): the book's own preview controller (Sec. 4.4.3)
  includes an explicit integral-action term (`K_s * Σ(p_ref − p)`,
  accumulated ZMP tracking error) alongside state feedback and the
  future-reference feedforward, but `K_DCM`'s fast-loop correction here is
  purely proportional, with no analog. A small, systematic, *repeating*
  model-mismatch bias — exactly what a proportional-only correction
  cannot reject, since it only reacts to the instantaneous error, not the
  accumulated pattern — would produce precisely the observed symptom
  (single-digit mm early, growing to hundreds of mm), consistent with
  (though not proof of) the same short-horizon-model-mismatch suspect
  above. Not yet implemented or tested; a natural next experiment before
  reaching for a full state estimator. **Push recovery was tested, not
  just claimed, and the result is an honest open finding**: sweeping external pelvis forces (5–30N, 0.1s, scaled to this
  design's actual ~8.9kg mass) mid-walk, footstep adaptation does not yet
  show a clear, consistent advantage over the plain fast-loop correction
  alone — the mechanism is verified mathematically correct and does shift
  the footstep target in response to a real disturbance, but that doesn't
  yet translate into measurably better recovery at the magnitudes tested.

**2026-09-08 — ankle_roll made temporarily actuated; both walking gaits
currently REGRESSED, not fixed by this change alone.** Session context: the
foot/ankle redesign earlier this session added a passive, spring-centered
`ankle_roll` joint. Running the walking selftests against it (a check that
had never actually been run until this point) found both gaits now fall:
`sim_walk_lipm.py --selftest` Stage 6 hit 93.4° peak tilt (was ≤6.4°) and
`sim_walk_recede.py --selftest` Stage 3 hit 89.0° with net-backward drift.
Root cause: neither controller has ANY lateral (roll-axis) feedback — they
were validated assuming a laterally-rigid foot — so an uncontrolled passive
DOF near the ankle lets the robot wobble sideways with nothing to check it.
A stiffness sweep on the passive spring confirmed this: 15-400 Nm/rad (the
physically-plausible placeholder range) all fell the same way; only ~10⁴
Nm/rad (de facto rigid) recovered the old baseline, and 10⁶ was numerically
unstable.

The user proposed a concrete methodology: make ankle_roll actuated first
(get walking working fully powered), then swap specific joints to passive
springs one at a time. A quick validation of this — patching `actuated:
True` into `joints.yaml` in memory, at the model's *then-current* ankle_roll
mass (0.05kg, the small passive-pivot-hardware estimate) — showed 8.6° peak
tilt, appearing to confirm the idea outright with zero control-code changes.
**That result was wrong, and the error is worth recording so it isn't
repeated:** the quick patch only set `actuated`, but left the pre-existing
`passive_type: spring_centered` / `spring_stiffness_nm_per_rad: 15.0` fields
in the in-memory dict, so `generate_mjcf.py`'s `get_spring()` (keyed only on
`passive_type`, blind to `actuated`) still emitted a `<joint stiffness=...>`
alongside the new position actuator — an accidental actuator+spring
combination, not "just powered." The design change actually committed here
removes `passive_type`/`spring_stiffness_nm_per_rad` entirely (as it should
— a joint can't be both), and mass.yaml's `ankle_roll` was bumped from
0.05kg to 0.35kg to match `ankle`'s motor+gearbox convention, since it's now
a real actuated joint, not passive pivot hardware. Re-tested cleanly (real
committed YAML, actuator only, no leftover spring): **both gaits still
fall** — `sim_walk_lipm.py` 86.6° at the current 0.35kg mass, 91-96° across a
0.05-0.35kg mass sweep at the default kp=150 actuator gain; raising
ankle_roll's own actuator kp up to 10⁴ (mirroring the passive-spring
threshold that worked) did NOT recover stability either (86-92° across
150-10⁴, 157° and numerically unstable at 3×10⁴). A direct isolation test
(passive spring, stiffness=10⁴, everything else held fixed) confirmed the
new 0.35kg ankle_roll mass alone flips a previously-working configuration
from 5.9° (at the old 0.05kg mass) to 91.6° (at 0.35kg) — the system is
sitting close enough to a stability boundary that a small, physically
minor change (extra 0.6kg total, low in the kinematic chain) is enough to
cross it, in either the passive-spring or actuated-servo case.

**RESOLVED, same session — both walking selftests pass again, and a real
methodological lesson cost a lot of the time getting there.** The user
asked for real local lateral feedback control (HUBO-style), explicitly as
the *main* control model, not a side branch, and said a full control-system
rewrite was acceptable if the existing one was the blocker. Two primary
sources were read to ground this rather than guess: **"Development of
Humanoid Robots in HUBO Laboratory, KAIST"** (Heo, Lee, Oh, 2012) — HUBO's
own architecture is an offline walking pattern plus several small, layered
real-time feedback controllers (balancing control — "a damping controller
and a ZMP compensator" — first), not a monolith; and **"Compliance Control
for Stabilizing the Humanoid on the Changing Slope..."** (Li, Zhou,
Tsagarakis, Caldwell, 2016) — a concrete admittance-control law achieving
compliant balancing with POSITION-controlled actuators + F/T feedback only,
directly compatible with `SPEC.md` §8.1. Conclusion, confirmed with the
user: the existing DCM/ZMP sagittal planner already IS the "offline
pattern" layer; what's missing is the local "ZMP compensator" balancing
layer for the lateral axis. Implemented in `tools/sim_walk_lipm.py`: a new
`lateral_zmp_correction()` (`K_ZMP_Y`, `ZMP_Y_FILTER_ALPHA`,
`MAX_ANKLE_ROLL_CORRECTION_RAD`) reusing `sim_zmp_balance.py`'s
`compute_zmp()` — which already computed BOTH x and y ZMP from real
contact points, `y` just silently unused until now — and the SAME planned
`zmp_reference()` the sagittal math already tracks, applied only to
whichever foot/feet are actually planted each phase. A new `_selftest_
stage5c` validates its sign directly (verified empirically against the
real model, not just derived: a positive `ankle_roll` command does shift
measured ZMP-y positive) before Stage 6 trusts it. `sim_walk_recede.py`
got the identical wiring (`lw.lateral_zmp_correction`, no duplicated logic).

**This layer alone did NOT fix the fall.** After wiring it in, both gaits
still fell (~87-96°) across every `K_ZMP_Y`/`ZMP_Y_FILTER_ALPHA` combination
swept. The lesson: `pelvis_tilt_deg()` returns a combined tilt *magnitude*
(`acos` of the dot product with vertical) that doesn't separate roll from
pitch — the "missing lateral feedback" diagnosis had never actually been
checked against the real axis breakdown. Decomposing it (extracting roll
and pitch from `data.xmat` separately) showed the fall was **pitch-
dominated** (48° pitch vs. 9° roll shortly before going over) — this was a
**sagittal `K_DCM` margin regression**, not a lateral one. Root cause:
`ankle_roll`'s added mass (0.35kg/side motor+gearbox, replacing the earlier
0.05kg passive-pivot estimate) dropped `z_c` from 0.5713m to 0.5355m and
`Tc` from 0.241s to 0.2336s — only a ~3% shift, but `K_DCM`'s stability
band was already documented as narrow, not a gradient (see that constant's
own comment history), and this shift was enough to cross it. A direct
`K_DCM` sweep (not a blind one — one variable, fixed step count, per this
file's own established discipline) found a new clean band: **`K_DCM=-0.6`**
for `sim_walk_lipm.py` (was -1.0) gives 6.42° flat from n=3 to n=14 (n=15
regresses — one step short of the old 15-step range, not chased further),
and a *separately*-tuned **`K_DCM_RECEDE=-0.70`** for `sim_walk_recede.py`
(that file's own receding-horizon dynamics have a different margin than
`sim_walk_lipm.py`'s fixed-horizon plan — confirmed directly, `-0.6` alone
gives 33.5° there) gives a clean **12-step** range (12.4° peak) — an
*improvement* over its pre-session baseline (was 8 clean/6.9°, 9
borderline, 10+ fell), not just a recovery. With both retuned, a small
`K_ZMP_Y=0.05` (swept: 0.02-0.05 clean, 0.10+ falls abruptly — another
narrow band) shaves a further fraction of a degree off `sim_walk_lipm.py`'s
peak tilt (6.42°→6.36°) — a real, kept improvement, just not the fix for
*this* fall. Both `--selftest` suites are green again as of this entry.

**Takeaway for next time, stated plainly so it isn't relearned the hard
way:** when a gait falls, decompose the tilt into roll/pitch/yaw FIRST,
before assuming which control axis is responsible and building a fix for
it. An axis-blind combined-tilt number sent real effort down the wrong path
for a full investigation cycle here.

**Where work stopped:**
Five P3 simulation milestones are done: fixed-base static load
validation, the floating-base ZMP ankle-pitch standing controller, three
validated steps of quasi-static (stumbling) walking, fourteen validated
steps of smooth fixed-horizon LIPM/DCM-planned walking plus local lateral
ZMP feedback (`sim_walk_lipm.py`, as of the 2026-09-08 K_DCM retune above),
and twelve validated steps of the receding-horizon controller with
capture-point footstep placement (`sim_walk_recede.py`, likewise retuned).
Neither indefinite walking nor a demonstrated disturbance-rejection
advantage is done yet — see that file's own bullet above and its module
docstring for the full, honest account (numbers below predate the
2026-09-08 retune but the underlying dynamics finding is unaffected): even
with footstep adaptation disabled, pure receding-horizon replanning alone
still degrades in a similar step range to `sim_walk_lipm.py`'s own
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


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
  humanoid-robotics/      ~76 papers on bipedal/humanoid robots, incl.
                           HRP-2/3/4, LOLA, TORO, iCub, WABIAN-2, ASIMO,
                           WALK-MAN, and other major reference platforms.
                           Push-recovery/capturability set added
                           2026-09-09 (see the dated entry below for full
                           findings, already incorporated into this
                           file's own conclusions):
                             - Koolen, de Boer, Rebula, Goswami, Pratt,
                               "Capturability-based analysis and control
                               of legged locomotion, Part 1: Theory..."
                               (IJRR 2012) -- formal N-step capturability
                               theory, THE reference for "is this
                               disturbance recoverable at all"
                             - Pratt, Koolen, de Boer, Rebula, Cotton,
                               Carff, Johnson, Neuhaus, "...Part 2:
                               Application to M2V2..." (IJRR 2012) --
                               real controller + hardware validation
                             - Stephens, Atkeson, "Push Recovery by
                               Stepping for Humanoid Robots with Force
                               Controlled Joints" (Humanoids 2010)
                             - Stephens, "Integral Control of Humanoid
                               Balance" (IROS 2007)
                             - Runge, Shupert, Horak, Zajac, "Ankle and
                               hip postural strategies defined by joint
                               torques" (Gait & Posture 1999)
                             - Walking_Control_Algorithm_of_Biped_
                               Humanoid_Robot_on_Uneven_and_Inclined_
                               Floor.pdf (Kim, Park, Oh 2007) -- NOT yet
                               read, terrain rather than push recovery
                             - Kim, Oh, "Posture Control of a Humanoid
                               Robot with a Compliant Ankle Joint" (IJHR
                               2010, HUBO Lab/KAIST) -- the 6th originally-
                               requested paper, procured via ILL and added
                               2026-09-09, read in full. NOT about push
                               recovery (see the dated entry below) --
                               about preventing stance-foot liftoff caused
                               by the WALKING PATTERN'S OWN fast control
                               inputs, via mechanical sole compliance
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
- *HUBO-style local-feedback architecture* — READ (2026-09-08/09, see the
  dated entries below for what was built and found): `KAIST/Development
  of Humanoid Robots in HUBO Laboratory, KAIST`; `Compliance Control for
  Stabilizing the Humanoid on the Changing Slope` (admittance control for
  position-controlled actuators + foot F/T sensors — this project's own
  actuation architecture, now implemented in sim_walk_lipm.py); `Current
  and Future Perspective of Honda Humanoid Robot` (Hirai, IROS'97 — the
  P2/P3 lineage's Model ZMP Control, a torso-momentum recovery strategy;
  read, NOT yet implemented, now understood to be a secondary refinement
  rather than the primary fix — see the push-recovery/capturability
  entry below). `KAIST/System Design and Dynamic Walking of Humanoid
  Robot KHR-2` still unread (direct KHR-3/HUBO precursor).
- *Push recovery / capturability* — READ 2026-09-09, see the dated entry
  below for the full synthesis: the Koolen/Pratt capturability pair,
  Stephens & Atkeson's push-recovery-by-stepping, Stephens' integral
  control paper, and the Runge et al. ankle/hip biomechanics paper (all
  listed in the folder tree above). Directly resolved this project's own
  open push-robustness investigation — read this before proposing any
  further disturbance-rejection mechanism.
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

**2026-09-08 (continued) — F/T sensor modeled in simulation, admittance
control implemented; real nominal-walking improvement, no push-robustness
improvement.** After the lateral-ZMP-feedback work above, the user asked
directly for genuine disturbance rejection, not just good nominal tuning,
and tested this claim: pushing the (well-tuned) walk with real external
forces (`sim_walk_recede.py --push-at/--push-force`, 5-30N, 0.1s, mid-walk)
produced an erratic response (5N fell, 10N survived, 30N fell again) — the
signature of a system tuned around a trajectory, not one with real margin.

Pursuing HUBO-style local feedback further, and specifically checking
whether HUBO's own high-reduction actuators (harmonic drives — same
friction/backlash problem as this project's high-gear-ratio brushed DC
motors) rule out true torque control for it too (the user's direct
question), pointed at **admittance control**: the COMAN paper already
indexed above ("Compliance Control for Stabilizing the Humanoid on the
Changing Slope...", Li/Zhou/Tsagarakis/Caldwell 2016) achieves compliant
balancing using ONLY position-controlled actuators + F/T feedback — the
authentic mechanism `design/sensors.yaml`'s F/T sensor exists for, not an
approximation of it via contact-geometry ZMP inference. Implemented:

- `tools/generate_mjcf.py`: a zero-DOF, zero-mass `{s}_ft_sensor` body
  inserted between `ankle_roll` and `foot` (matching `design/geometry.yaml`
  foot_stack order, zero offset so no dimension changed — confirmed via
  `verify_urdf_dimensions.py`), with a site on `foot` (the child) and
  `<force>`/`<torque>` sensors reading it — MuJoCo's force/torque sensors
  measure the reaction between a site's body and its PARENT, so this reads
  the real ground-reaction load path. Verified physically sensible before
  trusting it in any control loop: ~45.6N vertical per foot at nominal
  double-support stance (expected ~47N for 9.6kg), symmetric L/R, and
  near-zero (1e-15) roll torque at a symmetric stance — exactly the
  fixture the new selftest stage checks.
- `tools/sim_walk_lipm.py`: new `ankle_roll_admittance()` (COMAN's
  Ks→∞ rigid-actuation simplification, since megadroid's joints are rigid
  position servos, not series-elastic), applied PER FOOT from that foot's
  own sensor — no phase/plan bookkeeping needed at all (a foot in the air
  reads ~0 torque, so its own correction is already ~0), a genuine
  simplification over the superseded ZMP compensator. `lateral_zmp_
  correction`/`K_ZMP_Y` kept in the file, unused, as historical record
  (not deleted — matches this file's own practice for prior iterations).
  `mujoco.mj_rnePostConstraint(model, data)` added after every `mj_step`
  (required to populate the force/torque sensors — `sim_zmp_balance.py`
  already did this, `run_walk()` did not).
- **A genuine sign trap, caught by testing, not derivation:** the "obvious"
  physical intuition (push back against the measured moment, like a
  restoring spring) tested WORSE on a real transient push (1.6°→2.4° peak
  tilt) than its opposite (1.6°→1.8°, improving further with a stiffer
  gain). Likely cause, not fully chased down: the sensor's reaction-force
  convention (force the foot exerts ON its parent) is the Newton's-third-
  law opposite of "the ground pushing the foot," flipping the naive sign
  once. `_selftest_stage5c` now encodes the actual empirical outcome (a
  real push-test comparison), not just an algebraic sign check, since a
  sign check alone would have passed the wrong formula too.
- **Gain tuning, one variable at a time, per this file's own discipline:**
  `K_ADM` swept 0-15000 on both files together. Non-monotonic, matching
  every other margin found this session: unsafe bands at 500-1500 (falls,
  37-95°) and 5000-8000 (falls again, 79-90°) sandwich a clean zone at
  2000-4000. `K_ADM=2500` (middle of that zone) works for BOTH files
  without a separate `_RECEDE` variant this time. Result: `sim_walk_lipm.py`
  unchanged at 14 clean steps (6.43°, was 6.36° with the superseded ZMP
  layer — negligible difference); `sim_walk_recede.py` jumped from 12
  clean steps (12.4°) to **25 clean steps (9.99°, flat n=6..25)**, 30+
  falls — a substantial, real, validated nominal-walking improvement.
- **HONEST RESULT ON THE ACTUAL GOAL:** re-running the exact same 5-30N
  push tests (both axes, mid-walk) with `K_ADM=2500` active shows NO
  meaningful improvement — still falls on nearly every tested case. A
  further check (same 15N lateral push, six different push times across
  a walk) fell at nearly every timing too, with AND without admittance —
  ruling out "just bad luck on push timing" as the explanation. **This
  magnitude of disturbance is beyond what ankle-roll-only correction can
  absorb for a robot this size, regardless of which mechanism drives that
  one joint** (passive spring, ZMP compensator, and now F/T admittance
  have all been tried and all show the same ceiling). The bottleneck is
  architectural, not this joint's control law: genuine push recovery at
  this scale likely needs bigger/faster corrective action than an ankle
  alone provides — real footstep placement/timing adaptation (the
  Roux 2024 QP-based sequencer already indexed above does BOTH position
  AND timing jointly; this codebase's existing capture-point footstep
  adaptation only ever did position, timing deliberately left "out of
  scope" per its own docstring) or a hip strategy — not another local
  ankle_roll mechanism. Neither started.

**2026-09-08 (continued again) — footstep position+timing adaptation
tried; fourth mechanism, same wall, decisive negative result.** User
directed pursuing footstep timing adaptation next, since `sim_walk_recede.
py`'s existing `capture_point_footstep` explicitly left it "out of scope"
per its own docstring, and it's the one piece of the formal method
(Khadiv et al., via Roux 2024's thesis, already cited in this file) not
yet tried.

Implemented `capture_point_footstep_with_timing` in `sim_walk_recede.py`:
a closed-form (no scipy/QP dependency, matching this codebase's existing
hand-derived-math style) solution to the paper's constrained multi-
objective QP (position `p_T`, timing via `Γ(T)=e^(ω0T)`, DCM offset `b_T`)
for the unconstrained case, reparametrized from the paper's absolute-time
`Γ(T)` to a remaining-time `g=e^(ω0(T-t_now))` that ties directly to
`capture_point_footstep`'s own already-validated `growth` variable.
Reduces to two Lagrange multipliers via a 2×2 `np.linalg.solve` — trivial,
verified to machine precision against the constraint equations directly
(`_selftest_stage0`'s new part (d)), and verified to reduce EXACTLY to
`capture_point_footstep`'s own output in the joint limit
`alpha2,alpha3->inf` (not `alpha2` alone — a real derivation subtlety:
`alpha3->inf` is what forces the optimized DCM offset to nominal, which
`capture_point_footstep` does implicitly by never treating it as free).

**A real debugging trail, not a clean first try:** wiring it in at the
paper's own Table 1 weights (`1e3, 1, 1e6`) immediately regressed nominal
walking (falls at n=8+, was clean to 25). Sweeping `alpha2` alone up to
`1e20` didn't fix it. Instrumenting the actual run (not just the isolated
selftest) found the real cause: even at `alpha2=1e20`, the raw timing
delta was exactly 0.0 every call (ruling out timing drift entirely) —
but `p_new` itself differed from `capture_point_footstep`'s output by up
to 1.8mm during the run, growing as the walk degraded. The reduction
proof requires BOTH `alpha2->inf` AND `alpha3->inf`; `alpha3=1e6` (the
paper's own value) wasn't "infinite enough" once the DCM tracking error
`a` grew during any real stumble, since the `a²/(2·alpha3)` term in the
solve stops being negligible. Root cause: `alpha3` needed to reach
`~1e8` just for basic long-horizon stability — three orders of magnitude
past the paper's own nominal value, this robot's scale/dynamics simply
don't share Bolt's. This is exactly the "test the real model, don't trust
a limit that looks right on paper" lesson from `ankle_roll_admittance`'s
own sign trap earlier this session, recurring in a new form.

With `alpha3` corrected to a genuinely safe value (`1e10`), **even the
most permissive `alpha2` that preserved SHORT-horizon (n=12) nominal
walking still regressed the LONG-horizon record** (18 clean steps vs. the
previous 25) — there is no middle ground here where timing adaptation is
both meaningfully active and long-horizon-safe. And at that setting, the
actual point of the exercise — re-running this session's full 5-30N push
battery (both axes, mid-walk) — showed **zero improvement**: falls on
nearly every case, statistically indistinguishable from every other
mechanism tried. Loosening the footstep/timing adaptation clamps 3-4x
(ruling out "the safety margins are too conservative to matter") changed
nothing either.

**Decision: shipped INERT, not deleted.** `ALPHA2_TIMING`/
`ALPHA3_DCM_OFFSET` defaulted to `1e20` — verified to exactly reduce to
the prior (25-step/9.99°) behavior, so nothing regresses from this
change landing. The solver itself is real, correct, and tested; it's
simply not been shown to earn its keep active. See `ALPHA2_TIMING`'s own
comment and `sim_walk_recede.py`'s module docstring for the full trace.

**Four mechanisms, four dead ends, one real conclusion:** a passive
spring, a contact-geometry ZMP compensator, F/T-sensor admittance
control, and now footstep position+timing adaptation have ALL been tried
this session for genuine disturbance rejection at the 5-30N pelvis-push
range, and all four hit the identical wall. This stops being "wrong
mechanism" and starts being informative: something more fundamental than
any single local correction is happening at these force magnitudes for
this robot's mass/scale (9.6kg, ~80mm nominal step, ~100mm stance width).
Two real remaining candidates, neither started: (1) a genuine MULTI-STEP
recovery sequence — every mechanism tried this session only ever adapts
the SINGLE upcoming footstep, never plans a sequence of steps to actually
arrest a large disturbance the way a real human recovery stumble does;
(2) the pre-existing (before this session) LIPM point-mass model-mismatch
hypothesis in this file's own "Known limitation" section below — never
confirmed, flagged again here since a systematic model/reality gap could
plausibly explain why every LOCAL correction mechanism looks equally
insufficient regardless of which joint or footstep parameter it adjusts.

**Where work stopped:**
Five P3 simulation milestones are done: fixed-base static load
validation, the floating-base ZMP ankle-pitch standing controller, three
validated steps of quasi-static (stumbling) walking, fourteen validated
steps of smooth fixed-horizon LIPM/DCM-planned walking plus F/T-sensor
admittance feedback (`sim_walk_lipm.py`, as of the 2026-09-08 entries
above), and twenty-five validated steps of the receding-horizon controller
with capture-point footstep placement (`sim_walk_recede.py`, likewise
retuned plus the same admittance layer). Neither indefinite walking nor a
demonstrated disturbance-rejection advantage is done yet — confirmed
directly this session (see the F/T-admittance entry above): real push
tests at 5-30N still fall on nearly every case despite both the nominal-
walking improvements above. See that file's own bullet above and its
module docstring for the full, honest account (numbers below predate the
2026-09-08 changes but the underlying dynamics finding is unaffected): even
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
Other next directions (see the 2026-09-09 push-recovery/capturability
entry below for the current, evidence-based read on this — the line that
used to be here about needing a hip/torso strategy first is now known to
have the emphasis backwards):
  - Move toward P4 physical prototyping — premature before sustained,
    disturbance-tolerant walking is validated, since walking is likely to
    stress-test mechanical dimensions and motor torque budgets that
    standing alone doesn't touch.

---

**2026-09-09 — Push-recovery literature synthesis: five mechanisms failed
this session, and reading the actual push-recovery/capturability
literature explains why, with the emphasis reversed from what seemed
intuitive.** Session context: after `ankle_roll` became actuated
(2026-09-08 entries above), five different local disturbance-rejection
mechanisms were tried against a 5-30N pelvis-push battery (mid-walk,
`sim_walk_recede.py --push-at/--push-force`) — a passive spring, a
contact-geometry ZMP compensator, F/T-sensor admittance control, footstep
position+timing adaptation, and lateral integral action on the DCM
tracking loop — and every one hit an identical wall (falls on nearly
every tested magnitude, non-monotonic in force, not a timing artifact).
Reading Honda/Hirai's foundational balance paper (IROS'97, already in the
library) suggested the missing piece was torso/hip angular momentum
(their "Model ZMP Control"), since the LIPM/DCM model this whole codebase
uses structurally cannot represent it. The user then had five more
papers procured (now in this folder, see the tree above) to check that
hypothesis against the actual push-recovery literature before building a
sixth mechanism.

**The literature does NOT support torso/hip momentum as the primary
fix.** Four independent, convergent findings:

1. **Formal capturability theory, checked against megadroid's own real
   numbers.** Koolen et al.'s N-step capturability framework (Part 1)
   gives a closed-form margin for ankle/CoP-only recovery (no stepping,
   no torso): `d_inf = l_max * e^(-omega0*dt_s)/(1-e^(-omega0*dt_s)) +
   r_max`. Plugging in megadroid's OWN validated values (omega0=4.28
   rad/s from z_c=0.5355m, r_max=0.075m foot half-length, and even the
   SMALLEST footstep-adaptation clamp tried this session, l_max=0.05m)
   gives `d_inf ~= 89mm`. Converting the tested push forces to an
   instantaneous capture-point shift (`Δv/omega0` where `Δv = F*dt/mass`)
   gives 12mm (5N) up to 73mm (30N) — EVERY tested push falls comfortably
   inside megadroid's own theoretical ankle/CoP-only recovery margin. The
   theory says these should all be recoverable without a step, let alone
   a torso strategy. See `Capturability based analysis and control of
   legged locomotion Part 1...pdf`, Sections 5-6 and Eq. 26b.
2. **Real hardware validation, no torso momentum used.** Koolen/Pratt
   Part 2's controller for M2V2 (a real 3D force-controlled biped)
   explicitly states it "did NOT exploit angular momentum of the upper
   body as a means of control" (Section 6.1) — and still recovered 21 Ns
   pushes on real hardware, 15 Ns while walking in simulation. Both
   exceed this session's own tested impulses (5-30N over 0.1s = 0.5-3.0
   Ns). Its two real mechanisms: a CoP control law that pushes the
   desired CoP AWAY from the current capture point (Eq. 2, leveraging
   that the capture point naturally diverges away from the CoP, not
   fighting that dynamic the way a "chase the error" proportional
   controller does) and capture-region-based footstep placement
   (Algorithm 1) for when the capture point exits the support polygon.
   Neither mechanism resembles any of the five tried this session.
3. **A second independent robot, same result.** Stephens & Atkeson's
   PR-MPC on the Sarcos Primus (hydraulic, force-controlled) recovered
   18-23 Ns pushes using NO STEP AT ALL — pure COM/CoP model-predictive
   control (`Push Recovery by Stepping...pdf`, Fig. 14's comparison
   table). Their own future-work section again flags the LIPM's missing
   angular momentum and the standard "flywheel" extension to add it —
   unused, because it wasn't needed for their validated results either.
4. **A direct, quantified ankle-vs-hip comparison.** Stephens' "Integral
   Control of Humanoid Balance" (`Integral Control of Humanoid
   Balance.pdf`, Fig. 6) compares four controllers' maximum recoverable
   push: naive LQR (~8 Ns) -> LQR with ankle-torque saturation (~16 Ns)
   -> constraint-aware receding-horizon LQR (~21 Ns) -> CoP regulator
   WITH hip/CM regulation added (~23 Ns). The dominant jump (8->21 Ns,
   2.6x) comes from properly constraint-aware ankle/CoP design; adding
   hip strategy on top buys only ~10% more (21->23 Ns) — closely matching
   Part 1's own finding that adding a reaction mass increases the
   capture-region AREA by 34% (a smaller, secondary effect layered on
   top of the 166% gain from ankle/CoP alone). Runge et al.'s human
   biomechanics data (`Ankle_and_hip_postural_strategies...pdf`)
   independently confirms the same pattern in humans: hip strategy is
   always observed ADDED to ankle strategy as disturbance grows, never
   alone.

**Conclusion:** the five mechanisms tried this session didn't fail
because they lacked torso/hip authority — they failed because none of
them implemented anything resembling the actual capture-point-based
strategy (CoP-repulsion + capture-region footstep placement) that the
literature and megadroid's own numbers say should already work at these
disturbance magnitudes. This reverses the emphasis of the Honda-inspired
"Model ZMP Control" plan this file's own history entry above proposed:
torso lean is a real, worth-having secondary refinement (confirmed
~10-16% additional margin by two independent sources), but implementing
a genuine capture-point/CoP-based controller — replacing the ad-hoc
proportional/admittance/integral corrections tried this session — is the
higher-leverage next step, not yet attempted in any form.

**Not yet done:** actually implementing this controller (adapting
Pratt/Koolen's CoP-repulsion law and capture-region footstep placement to
megadroid's position-only architecture, likely replacing or substantially
restructuring `ankle_roll_admittance` and the footstep-adaptation logic
in `sim_walk_recede.py`); reading the sixth procured paper
(`Walking_Control_Algorithm_of_Biped_Humanoid_Robot_on_Uneven_and_
Inclined_Floor.pdf`, terrain-focused, lower priority); reading Part 2's
remaining sections (walking-task footstep calculator details, Section
6-7 discussion). No decision made yet on when/whether to pursue any of
this — flagged for the user's call, matching this session's standing
practice throughout.

### 2026-09-09 — Capture-point-lite implementation: sagittal push recovery substantially fixed by loosening the footstep clamp; lateral remains unresolved, and it isn't the footstep clamp, K_ADM, or the timing clamp

Direct follow-through on the synthesis entry immediately above, scoped via
Plan Mode as "fix the two real gaps, not add a sixth mechanism": (1) the
footstep-adaptation clamps in `sim_walk_recede.py` were tuned this
session for nominal-walk smoothness, never re-checked against the push
battery, and were far tighter than the leg's real reach
(`sim_walk_lipm.py`'s own `WALK_AX_MARGIN_M=0.15`); (2) `ankle_roll_
admittance`'s gain (`K_ADM`) was tuned the same way, never swept against
a real disturbance. Explicit non-goals carried over unchanged: no torso/
hip strategy, no full polygon capture-region (Part 2 Algorithm 1) —
`capture_point_footstep_with_timing`'s existing closed-form target is the
right-sized fit for megadroid's flat-ground, straight-line gait.

**What was actually tried:**
- Footstep clamp: NOT loosened via a precomputed analytic reach budget
  (the plan's initial idea) — direct testing showed `WALK_AX_MARGIN_M`
  does not transfer as a clamp value here, and independent per-axis
  sweeps missed a real destructive X/Y interaction (X=0.07 alone and
  Y=0.03 alone were each individually clean; combined, they fell at
  n=12). A joint 2D grid search (not independent per-axis sweeps) found
  `MAX_FOOTSTEP_ADAPT_X_M=0.05` (unchanged) / `MAX_FOOTSTEP_ADAPT_Y_M=
  0.03` (loosened 3x from 0.01) as the best validated combination for
  the *current* control stack — see that constant's own code comment in
  `sim_walk_recede.py` for the full margin-non-transferability writeup.
- `K_ADM` (in `sim_walk_lipm.py`, shared by both walk files): swept
  500-4000 against both nominal walking and the full lateral push
  battery with the new clamps active. Confirmed to have essentially zero
  effect on lateral push-recovery outcomes (5N always ~7-8°, 10N+ always
  falls ~78-95° regardless of value) — ruled out as the lateral
  bottleneck. Left at its existing default (2500), which sits
  comfortably inside the ~1500-3000 nominal-walking safe range this
  sweep confirmed.
- `MAX_FOOTSTEP_ADAPT_T_S` (timing clamp, per the plan's item 1): swept
  0.10-0.30s (well above the ~0.36s single-support duration's own scale)
  against the lateral push battery. **Never once bound** — identical
  results (both nominal-walk tilt and all five push outcomes, to the
  first decimal) at every value tested. Left unchanged at 0.1s: there is
  no evidence it needs to move, and no cost to leaving it as-is.
- A further Y-clamp sweep past 0.03 (0.025, 0.035, 0.045, 0.05) was also
  run specifically against the lateral push battery, not just nominal
  walking: 0.03 remains the clear best on both axes; 0.025 is worse on
  both; ≥0.035 breaks nominal walking outright (91-98° at n=18) for no
  lateral improvement. Confirms 0.03 is a genuine local optimum, not an
  undershoot.

**Result — full 5/10/15/20/30N battery, mid-walk (`push_at=2.0s`),
final validated config (X=0.05, Y=0.03, T_S=0.1, K_ADM=2500 — all
defaults except Y):**

| axis | 5N | 10N | 15N | 20N | 30N |
|---|---|---|---|---|---|
| sagittal (X push) | 6.47° | 6.36° | 6.95° | 6.83° | **88.36° (falls)** |
| lateral (Y push) | 7.83° | **79.96° (falls)** | **93.18° (falls)** | **80.35° (falls)** | **79.41° (falls)** |

Sagittal: 4 of 5 forces now recover cleanly (was ~0-1 of 5 before this
session's footstep-clamp work), only 30N still falls. Lateral: only 5N
recovers; everything from 10N up still falls, unchanged from before this
implementation pass — the Y-clamp loosening and K_ADM re-sweep produced
real sagittal gains but no lateral gain at all.

A 15N six-timing-point sweep (0.8s-3.8s) confirms this isn't a phase-
alignment artifact in either direction: lateral falls at 5 of 6 timing
points (only 3.2s survives, 8.61°) — genuinely unrecovered, not a lucky/
unlucky sample. Sagittal also falls at the two earliest timing points
(0.8s, 1.4s: 77-81°) and only recovers from 2.0s onward — the "4 of 5"
sagittal result above is specific to mid-walk timing, not uniform across
the whole gait cycle; early-cycle sagittal robustness is a real,
separate gap not investigated this pass.

Full selftest suite (`sim_walk_recede.py --selftest`) passes clean
against this final config, stage 3 showing `max_tilt=6.36deg` at the
6-step receding-horizon check. `preflight.py` clean (only derived-doc
rehydration timestamps changed, no design/content diff).

**Conclusion, matching the plan's own explicit instruction not to add a
sixth ad-hoc mechanism if this doesn't clear the tested range:** this is
the honest result, not a stopping point chosen for convenience. Three
independent levers (footstep Y-clamp, K_ADM, timing clamp) were each
directly swept against the lateral push battery specifically and none of
them move lateral push recovery at all, which rules out "just loosen the
existing clamps further" as the fix. The likely remaining candidate,
not yet tested: megadroid's stance geometry is strongly asymmetric
between axes (`HIP_Y=0.05m` lateral half-spacing vs. ~80mm sagittal step
length — flagged earlier this session as the reason X/Y ever needed
separate clamps at all), so the capture-point TARGET calculation itself
may be structurally different in authority between axes, independent of
any clamp — e.g. `capture_point_footstep_with_timing`'s lateral solution
may be geometrically starved regardless of how far it's allowed to
travel. Not diagnosed this pass; flagged for the user's call on whether
to pursue it.

### 2026-09-09 — Stance-geometry asymmetry hypothesis investigated: confirmed as a real bug, but not the lateral bottleneck — the actual cause is growth-factor amplification saturating the footstep clamp on BOTH axes even with zero disturbance

Direct follow-through on the previous entry's flagged candidate: is
`capture_point_footstep_with_timing`'s lateral solution geometrically
starved because `HIP_Y=0.05m` lateral half-spacing is tiny relative to
sagittal step length?

**Confirmed, real bug — `b_nom_y` is structurally zero in two places:**
- `replan_horizon` (`sim_walk_recede.py:577`) hardcodes
  `b_nom = np.array([nominal_dcm_offset(step_length, t_ss, omega), 0.0])`
  — not a formula, a literal constant for the y-component.
- The footstep-placement law's own `b_nom_y` (lines 1194-95) computes
  `step_len_y = swing_to_xy_nominal[1] - swing_from_xy[1]`, which is ~0
  by construction on this straight-line gait (a given foot always
  returns to the same lateral track, so its own displacement is zero).

Direct numerical ground truth (measuring `xi_y` vs. the stance foot's y
at the end of each single-support phase, from `solve_dcm_backward`'s own
output, not the analytic approximation) shows the REAL nominal lateral
DCM offset from the stance foot at touchdown is **±56.8mm** — larger in
magnitude than the sagittal offset (52.8mm). Both `b_nom_y` call sites
are silently discarding this and using 0 instead.

**Tested two fix variants, neither meaningfully changed push-recovery
outcomes** (diagnostic edits, reverted — not committed):
- `replan_horizon`'s `b_nom_y`, patched to `±1.1364 * stance_y`
  (empirically-scaled, sign tried both ways): change within noise
  (7.83°→7.39-8.46° at 5N, 79.96°→78-80° at 10N+). Root cause found:
  `replan_horizon`'s terminal-condition correction is
  `xi_rest + b_nom*exp((t-t_end)/Tc)` — this decays to ~0 by the time
  you're back at "now" (t_end is several steps ahead, and
  `exp(-(several step durations)/Tc)` with `Tc≈0.23s` is astronomically
  small), so this call site's `b_nom_y` essentially never influences the
  near-term tracked trajectory regardless of its value. A real bug, but
  provably inert for disturbance recovery.
- The footstep-placement law's `step_len_y`, patched to
  `swing_to_xy_nominal[1] - stance_xy[1]` (stance-relative, matching the
  periodicity condition's actual intent): also no meaningful lateral
  improvement (10N+ still falls at 76-92°), and a mild sagittal cost at
  15-20N (26.08°/15.06° transient tilt vs. 6.95°/6.83° before — did not
  cause a fall, but worth noting).

**What actually explains the whole picture — found by instrumenting the
RAW (pre-clamp) footstep correction during NOMINAL, undisturbed
walking:** the clamp is saturating almost every single retarget tick,
on BOTH axes, with zero push applied. Direct trace (`n_steps=10`, no
push): Y clamp saturated 12/12 ticks, X clamp saturated 8/12 ticks, with
raw (unclamped) demands of 40-125mm against the 30-50mm clamps — 2-4x
over, during ordinary walking with no disturbance at all.

This matches a limitation already flagged in-code, written earlier this
session, in `MIN_RETARGET_FRACTION`'s own comment: `capture_point_
footstep`'s `growth = exp(omega*(T_touchdown-t_now))` term is largest
early in a swing and amplifies any DCM measurement noise/lag into an
oversized correction (that comment documented >3x amplification within
the first third of a step, specifically flagged for the y-axis at the
time). What's new this pass: direct instrumentation shows this isn't a
rare/edge-case effect confined to y — it's the PERMANENT, ALWAYS-ON
operating mode of the footstep-adaptation mechanism on both axes. The
clamps aren't a safety net for genuine disturbances; they're the actual
controller (the raw target saturates the clamp virtually every tick, so
the applied correction is "nominal ± clamp value, in whichever direction
the noise-amplified raw target points" almost all the time).

**Conclusion:** this reframes every finding from the previous
implementation entry. Y-clamp sweeps, the K_ADM sweep, the timing-clamp
sweep, and this pass's two `b_nom_y` fix attempts all failed to move
lateral push recovery for the SAME underlying reason — they're all
downstream of (or masked by) a mechanism that's already saturating from
growth-factor noise amplification, unrelated to any actual measured
disturbance. A real push's marginal contribution to `xi_y` doesn't
meaningfully change which direction an already-saturated clamp points,
because the baseline signal is already dominated by this amplification
effect. The stance-geometry asymmetry hypothesis and the `b_nom_y=0`
bugs are real and worth fixing on their own merits (they're a
structural inaccuracy in the reference trajectory), but they are NOT
the lateral push-recovery bottleneck.

**Not yet done — the real next step, in progress:** address the
growth-factor amplification itself, e.g. tightening
`MIN_RETARGET_FRACTION` further, damping/capping the `growth` term
directly inside `capture_point_footstep`/`capture_point_footstep_with_
timing` rather than only clamping its output, or reconsidering
`REPLAN_PERIOD_S`'s interaction with retarget timing. Whatever fix is
tried must be validated against BOTH the nominal-walk saturation-rate
diagnostic introduced this pass (raw-vs-clamped tick counts, not just
final tilt) and the full push battery, since "stops saturating during
nominal walking" and "recovers from a real push" are different claims
that need separate verification.

### 2026-09-09 — Growth-factor amplification traced to its root: raw_delta is dominated by a ~40-100mm b_nom_y magnitude error, not measurement noise — but the obvious fix (a bigger constant) causes a NEW regression (nominal walking now falls)

Direct follow-through, same day: decomposed `raw_delta_y` algebraically.
With `ALPHA2_TIMING`/`ALPHA3_DCM_OFFSET` both `1e20`,
`capture_point_footstep_with_timing` provably reduces to the simple
closed form `p_new = p0 + a*growth - b_nom` (a = xi_filtered - p0), so
`raw_delta = p_new - swing_to_xy_nominal = (p0 - swing_to_xy_nominal) +
a*growth - b_nom`. Traced this against real nominal-walk instrumentation
(`n_steps=10`, zero push): `(p0 - swing_to_xy_nominal)` is close to a
FULL STANCE WIDTH (~-78 to -100mm, since p0 and the nominal target are
opposite feet by construction) and `a*growth` is a further -25 to -54mm
— together explaining essentially all of the previously-reported 40-125mm
raw deltas. This was previously mis-attributed to "growth-factor noise
amplification" in the entry above; the growth term IS a real
multiplicative factor, but the dominant contributor is that `b_nom_y` is
supposed to cancel most of this and is currently ~0 (see the prior
"stance-geometry asymmetry" entry), not that the underlying signal is
noisy.

**Re-derived what `b_nom_y` should actually be, correctly this time.**
Earlier attempts used the wrong reference point: `b_nom` in this
codebase's own docstring is defined relative to the NEW stance foot
(`p_new`/`swing_to_xy_nominal`), but the direct numeric ground-truth
check two entries above measured `xi(touchdown) - p0` (the OLD/current
stance foot) — a different quantity. Correcting for this: `b_nom_y =
(measured xi(touchdown)-p0 offset) - (stance width) ≈ ∓43.2mm`
(±56.8mm measured offset minus the ~100mm stance-width term, sign
depending on which foot is swinging). This also has a clean theoretical
grounding: DCM dynamics under a fixed single-support ZMP give `xi(t) =
p0 + a(t)*growth(t)` with `a(t)*growth(t)` PROVABLY TIME-INVARIANT for a
genuinely nominal (undisturbed) trajectory (growth decays exactly as
fast as `a` grows) — so a well-posed `b_nom` should be a fixed constant
per swing, not dependent on which tick within the swing it's evaluated
at.

**Tested `b_nom_y = -0.8636 * swing_to_xy_nominal[1]` (the ±43.2mm
value) in the footstep-placement law, diagnostic only, reverted:**
- First ~5 retarget ticks: dramatic improvement, `raw_dy` dropped from
  the previous 40-125mm range to single digits (10.0, 2.1, 3.3, 3.0mm) —
  strong direct confirmation the magnitude/reference-point diagnosis is
  correct.
- Then diverges: by the 7th-9th tick, `raw_dy` grows explosively (77 →
  222 → 548 → 851 → 1363 → 1758mm) and nominal walking (zero push)
  itself FALLS (`max_tilt=94.04°`) — a regression worse than the
  baseline this was meant to fix.

**Why:** a single fixed constant is only exactly correct at the specific
swing-progress fraction it was empirically measured at (touchdown,
progress=1.0); real ticks fire across the whole `MIN_RETARGET_FRACTION`
–`COMMIT_FRACTION` window (progress 0.30-0.85), and real dynamics
(double-support ZMP motion, actual robot vs. idealized LIPM, DCM filter
lag) aren't a perfectly time-invariant `a(t)*growth(t)` in practice —
small per-tick residuals from using one fixed constant compound step
over step with no damping, and the gait's own feedback (each step's
footstep error feeds into the next step's stance position) amplifies
rather than corrects that drift over ~7-9 steps.

**Conclusion:** the root-cause diagnosis (b_nom_y magnitude/reference
error dominates raw_delta, not noise) is now well-evidenced — both by
the algebraic decomposition and by the dramatic short-term improvement
before the fixed-constant approach diverged. But a correct fix needs
`b_nom_y` to be RESPONSIVE to actual swing timing (not a single
precomputed constant) while still cleanly separating "genuine nominal
sway" from "genuine disturbance" — which is a real control-design
question (e.g. calibrating b_nom from the swing's own earliest
post-`MIN_RETARGET_FRACTION` measurement, or replacing the delta-vs-
nominal clamp with a reachability-based clamp on `p_new` directly, closer
to the original plan's first design idea before it was set aside for
empirical clamp retuning) — not a one-line constant swap. This is now a
genuine architecture question, not a diagnostic one; per this project's
own standing practice, that calls for scoping via Plan Mode before
further live edits rather than continuing to iterate constants directly
against the sim.

### 2026-09-09 — Per-swing calibrated b_nom_y implemented (Plan Mode, approved) and tested: fails faster and worse than the fixed constant, for a clear, fundamental reason — it removes ALL restoring force toward the true nominal stance geometry

Scoped and approved via Plan Mode
(`.claude/plans/scope-the-rearchitecture-out-jiggly-robin.md`): rather
than a fixed geometric constant for `b_nom_y` (previous entry, diverged
by step 7-9), calibrate it fresh from each swing's own first retarget
measurement, using the real DCM-dynamics invariant
(`b_nom_y_calibrated = stance_xy[1] + (xi_filtered[1]-stance_xy[1])*
growth - swing_to_xy_nominal[1]`, computed once at the first tick past
`MIN_RETARGET_FRACTION` each swing, held fixed for the rest of that
swing, reset to `None` at swing completion). Implemented in
`sim_walk_recede.py`, compiled clean, tested — **reverted, not
committed.**

**Result: worse than the fixed-constant failure, and faster.** Nominal
walking (zero push) was already fallen by n=10 (`max_tilt=89.71°`,
`net_forward=-297.8mm` — walking backward, stuck), identical at n=18 and
n=25 (confirms it fell early and stayed down, not a slow drift).

**Root cause, confirmed by direct trace:** `raw_delta_y` stayed at
~0.00mm on almost every single retarget tick, exactly as the formula
guarantees by construction (at the calibration tick, `p_new` is
tautologically forced to equal `swing_to_xy_nominal`). But `xi_y` (the
actual measured DCM) diverged catastrophically in the same trace: -21mm
→ -31mm → -65mm → **-304mm** → -342mm → -303mm → -277mm → -275mm, over
just 8 swings of nominal walking. The mechanism wasn't failing to
compute `raw_delta` correctly — it was computing it "correctly" by a
definition that has no content: calibrating `b_nom_y` fresh every swing
means each swing accepts wherever the robot currently is as the new
definition of nominal, so there is **no restoring force** pulling the
gait back toward the true geometric stance width (±HIP_Y) at all. It's
an undamped integrator/random walk, not a controller — the previous
entry's fixed-constant version was closer to correct specifically
BECAUSE it didn't recalibrate away real error (it just didn't track
legitimate model mismatch precisely enough, causing slower compounding
drift instead of this immediate loss of all corrective authority).

**Conclusion:** two structurally different, individually reasonable
approaches to fixing `b_nom_y` have now both failed, for complementary
reasons — a fixed constant undershoots real dynamics and compounds
slowly (falls by step 7-9); a fully recalibrated-per-swing estimate has
zero restoring force and fails almost immediately (falls by step ~8-10,
faster in wall-clock terms since it's swing-count not step-count, but
comparably early). This suggests the correct fix is something
IN BETWEEN — e.g. a slowly-adapting (multi-swing EMA, not full reset)
estimate that tracks genuine slow model mismatch while still being
anchored to the true `±HIP_Y` geometric target as its primary term, or
abandoning `b_nom_y` precision entirely in favor of the OTHER approach
flagged two entries above (reachability-based clamping of `p_new`
directly, decoupled from getting this reference term exactly right).
Not attempted this pass — flagged for the user's call on direction
before further live implementation, since the plan's specific approved
mechanism did not pan out and a third variant deserves a check-in rather
than more solo iteration.

### 2026-09-09 — b_nom_y magnitude sweep, conclusive: EVERY tested nonzero value is neutral-or-worse for nominal stability; the theoretically "correct" magnitude is the WORST one

One more bounded, cheap check before stepping back, staying within the
already-explored "fixed constant" design (not a new architecture): swept
`b_nom_y = k * swing_to_xy_nominal[1]` for `k` in
`{0.0, -0.2, -0.4, -0.6, -0.8, -0.8636}` (0.0 = current committed
baseline; -0.8636 = the exact empirically/theoretically-derived value
from two entries above) against nominal-walk stability at n=10/18/25,
diagnostic only, reverted:

| k | n=10 | n=18 | n=25 |
|---|---|---|---|
| 0.0 (baseline) | 6.36° | 8.21° | 79.72° |
| -0.2 | 6.36° | 13.22° | 77.86° |
| -0.4 | 6.36° | 11.56° | 88.95° |
| -0.6 | 25.32° | 87.46° (falls) | 87.46° |
| -0.8 | 77.57° (falls) | 77.57° | 77.57° |
| -0.8636 | 94.04° (falls) | 94.04° | 94.04° |

Monotonic: larger `|k|` fails earlier and harder, with zero exceptions
across the swept range. The current committed baseline (`k=0`, i.e. the
"broken" `b_nom_y≈0` this whole investigation set out to fix) is
tied-or-better than every nonzero value tested, including small ones
(-0.2, -0.4) that don't obviously reproduce the divergence failure modes
of the two previous entries.

**Conclusion — this closes out the `b_nom_y` investigation for now.**
Three structurally different fixes (naive stance-relative analytic
formula, empirically-measured fixed constant, per-swing calibration) and
now a full magnitude sweep of the fixed-constant family have all been
tested and found neutral-or-harmful. The theoretical case that
`b_nom_y≈0` is "wrong" (Sections above: real numeric ground truth shows
±56.8mm, the algebraic decomposition shows it should cancel a
near-full-stance-width term) is not in question — but the REST of this
control stack (`K_DCM`, the footstep clamps, `K_ADM`, the retarget
timing constants) was all tuned empirically WITH `b_nom_y≈0` already
baked in, and correcting `b_nom_y` in isolation, without re-tuning
everything that was implicitly compensating for it, makes things worse,
not better. A coordinated re-tune of the whole stack around a corrected
`b_nom_y` might work but is a substantially larger undertaking than
adjusting one term, and isn't attempted here.

**Where this leaves lateral push recovery:** unresolved, honestly, after
extensive and now fairly exhaustive investigation of this specific
lever. The other previously-flagged alternative — replacing delta-vs-
nominal clamping with reachability-based clamping of `p_new` directly
(closer to the original capture-point-controller plan's first design
idea, set aside earlier this session for empirical clamp retuning) — is
a genuinely different mechanism, not yet tried, and would sidestep the
`b_nom_y` precision question entirely rather than trying to fix it. That
or accepting this as the current architecture's real limit are both
reasonable next calls; flagged for the user's direction before further
implementation.

### 2026-09-09 — Reachability-based clamping tried (Plan Mode, approved), confirms the risk flagged in that same plan: fails immediately, for the exact predicted reason

User chose this direction over a coordinated re-tune or stopping.
Scoped and approved via Plan Mode: remove `MAX_FOOTSTEP_ADAPT_Y_M`'s
role as a delta-from-nominal clamp entirely, relying only on `leg_ik`'s
existing `UnreachableTarget` exception (already caught with a
hold-last-command fallback, `sim_walk_recede.py:1291-1303`) as the
safety net — sidestepping the `b_nom_y` precision question rather than
continuing to chase it. The plan's own Context section flagged the risk
up front: `UnreachableTarget` only fires past `R_MAX=0.6m` of required
leg extension, but the problematic `raw_delta_y` values measured all
session are 40-125mm — nowhere near that limit — so removing the clamp
was flagged as likely to let the same structurally-biased raw targets
through unchecked. Implemented (one-line change, Y-axis clip removed,
X unchanged), tested, **reverted, not committed.**

**Result: exactly the flagged risk, immediately.** Nominal walking
(zero push) fell by n=10 (`max_tilt=87.42°`, `net_forward=-570.3mm` —
walking backward), identical at n=18/25 (fell early, stayed down).
`UnreachableTarget` never engaged — the raw targets were well within
600mm of leg reach the whole time, exactly as predicted; the clamp's
real job was never about kinematic reachability, it was suppressing a
balance-relevant bias that has nothing to do with what the leg can
physically reach.

**Conclusion:** five structurally different mechanisms have now been
tried against the lateral push-recovery gap this session (four
`b_nom_y` fixes plus this reachability-based clamping attempt), all
tested to the same standard (nominal-walk stability checked first, real
before/after numbers, honest reporting), all neutral-or-harmful. The
remaining two options from the original three-way choice — a
coordinated re-tune of the whole stack (`K_DCM`, all footstep clamps,
`K_ADM` together, around a corrected `b_nom_y`) or accepting this as the
architecture's current honest limit — are both real, substantially
larger-scope decisions than anything tried so far. Flagged for the
user's call; not attempted further without one, per this session's
established practice of checking in before a new structurally different
attempt rather than continuing to iterate solo.

### 2026-09-09 — Coordinated whole-stack re-tune (Plan Mode, approved): fixes nominal-walk stability substantially, sagittal holds, but lateral push recovery STILL doesn't move at any of the 3 budgeted b_nom_y values

User chose this over stopping, after five prior mechanisms failed.
Scoped and approved via Plan Mode: instead of changing `b_nom_y` alone,
jointly re-sweep it together with `K_DCM_RECEDE`
(`sim_walk_recede.py:869`, the shared fast-loop DCM gain applied to
BOTH axes) and `MAX_FOOTSTEP_ADAPT_Y_M`, on the theory that both were
empirically tuned around the old `b_nom_y≈0` behavior and have no
reason to still be correct once `b_nom_y` changes. Diagnostic-only
(env-var runtime overrides), budget of 3 `b_nom_y` magnitudes per the
approved plan, each with its own small `K_DCM_RECEDE × Y_clamp` grid.
**All reverted, nothing committed as code.**

**b_nom_y k=-0.2** (the least-degraded nonzero value from the earlier
magnitude sweep) — Stage 2 grid found a genuinely strong nominal-walk
cell: `K_DCM_RECEDE=-0.77`, `Y_clamp=0.03` gives **6.32° flat through
n=30** (no growth at all), a dramatic improvement over the documented
baseline (n=18 clean at 8.21°, n=25 FALLS at 79.72°). Loosening the Y
clamp (0.06-0.15) was tested and uniformly catastrophic (86-100° at
every K_DCM_RECEDE value) — confirms the clamp's real job is
balance-relevant, not kinematic, consistent with the reachability-based
clamping entry above. Push battery at this configuration: sagittal held
(4/5 forces still survive, similar magnitudes to before, though 10N/20N
show larger transients — 17.58°/11.02° vs. the old ~6-7° — without
falling). **Lateral: unchanged.** 5N survives (6.49°, if anything
slightly better than before), but 10N/15N/20N/30N all still fall at
essentially the same magnitudes as the pre-re-tune baseline (80-92° vs.
80-93° before).

**b_nom_y k=-0.1** — best cell `K_DCM_RECEDE=-0.9`, `Y_clamp=0.03`:
nominal walking excellent (6.17° flat n18=n25). Push battery: same
pattern exactly — sagittal 4/5 survive, lateral only 5N survives
(7.25°), 10-30N all fall (79-94°).

**b_nom_y k=-0.3** — best cell `K_DCM_RECEDE=-0.8`, `Y_clamp=0.03`:
nominal walking fine (6.28-6.90°), but this one is a genuine
regression on push recovery: sagittal 5N now FALLS too (49.58°, was
~6.5° before), and lateral 5N also falls (79.74°, was the one push that
always survived before). Worse than either of the other two candidates
on every axis.

**Conclusion, budget exhausted per the approved plan:** across all 3
budgeted `b_nom_y` values, each independently re-tuned to its own best
`(K_DCM_RECEDE, Y_clamp)` combination, lateral push recovery NEVER
improves beyond "5N survives, 10N+ falls" — the exact same ceiling
found by every one of the five prior mechanisms this session. The
`k=-0.2` configuration is a genuine, real, non-regressive improvement to
NOMINAL walking specifically (clean through n=30 vs. falling at n=25 —
a substantial margin gain in its own right, sagittal push recovery
essentially preserved) — but it does not touch the lateral push-recovery
gap this whole investigation was aimed at. Six structurally different,
independently-tested mechanisms have now failed on that specific
question: four `b_nom_y` fixes, reachability-based clamping, and this
coordinated re-tune. `sim_walk_recede.py` is unmodified (all diagnostic
overrides reverted) — the nominal-walk-stability improvement
(`k=-0.2`/`K_DCM_RECEDE=-0.77`/`Y_clamp=0.03`) is NOT yet committed;
flagged for the user's call on whether to keep it as a standalone
nominal-walking-quality improvement despite not achieving its original
lateral-push-recovery goal.

### 2026-09-09 — Nominal-walking-quality improvement from the coordinated re-tune KEPT and committed (user's call)

User's call: keep the `k=-0.2`/`K_DCM_RECEDE=-0.77`/`Y_clamp=0.03`
configuration from the entry above, since it's a real, non-regressive
improvement even though it doesn't fix lateral push recovery. Committed
in `sim_walk_recede.py`: `b_nom = (nominal_dcm_offset(step_len_x, ...),
-0.2 * swing_to_xy_nominal[1])` (was `nominal_dcm_offset(step_len_y,
...)`, ~0 by construction) and `K_DCM_RECEDE = -0.77` (was `-0.70`);
`MAX_FOOTSTEP_ADAPT_Y_M` unchanged at `0.03`. Re-verified against the
committed file (not just runtime overrides) before committing: nominal
walking flat at `6.32°` through n=30 (was falling at n=25); sagittal
push battery unchanged from the diagnostic numbers (4/5 forces survive,
10N/20N show larger transients — 17.58°/11.02° — than the pre-re-tune
baseline's ~6-7° but don't fall); lateral push battery unchanged (only
5N survives, 10N+ still fall at 80-92°) — confirms this is a
walking-quality gain only, not a push-robustness fix. Full selftest
suite passes (stage 3: `max_tilt=6.32deg`, `net_forward=318.6mm`).
`preflight.py` clean (SPEC.md/MECH.md timestamp-only diffs discarded,
no design content change).

### 2026-09-09 — Read the 6th originally-requested paper (Kim & Oh, "Posture Control of a Humanoid Robot with a Compliant Ankle Joint," IJHR 2010): a different problem than push recovery, but a concrete hardware idea worth flagging

Procured via interlibrary loan and added to `reference-material/
humanoid-robotics/`, read in full (25 pages). This closes out the set of
6 papers originally requested when the push-recovery literature review
began.

**What it's actually about — NOT push recovery.** No external
disturbance/impulse testing anywhere in this paper. The problem it
solves: during ordinary walking, the stabilization controller's own
fast/aggressive control inputs (not any external push) can cause the
stance foot's sole to momentarily lose ground contact and tip — this
breaks the ZMP/inverted-pendulum assumption every existing controller in
their survey relies on, independent of any external disturbance. Their
fix: **add mechanical compliance between the sole and the ankle F/T
sensor** (a thin urethane layer, reducing torsional stiffness from 1616
to 1160 Nm/rad, a 72% reduction) specifically to reduce the sole's
tendency to lift off — with a derived bound on how compliant it can be
before the robot just falls over under its own weight
(`1.57mgl < K2 < K2*`, Eq. 5). This is a narrower, more mechanical
notion of "compliant ankle" than Honda's Model ZMP Control (torso
momentum) or this session's own `ankle_roll_admittance` (a software
admittance law on a still-mechanically-rigid joint) — it's literally
inserting a soft physical layer into the foot stack.

**Control architecture**: two decoupled SISO loops, justified by
`K_hip >> K_ankle` (hip stiffness ~14.5x the ankle's): a "body balancing
controller" (PD + low-pass, ankle reference → body inclination, applied
to the STANCE leg) and a "vibration-reduction controller" (lead
compensator, hip reference → swing-leg angular velocity, applied to the
SWING leg). Settling time dropped from 7-8s to 1-1.5s; swing-leg
oscillation power reduced 70-84%. Validated on walking-in-place, forward
walking (1.35 km/h), and an 8mm bump — body inclination stayed within
about ±0.5-1° in all three, ZMP stayed inside the support polygon
throughout.

**Relevance to megadroid — flagged, not acted on:** `design/joints.yaml`
already documents `ankle_roll` as temporarily fully actuated (INTERIM),
with the eventual plan to return it to passive spring-centered once a
validated spring or real local lateral feedback exists. This paper's
SOLE-compliance idea (a soft interposer between foot and F/T sensor) is
a DIFFERENT, smaller-footprint mechanical option than a spring-centered
ankle_roll joint — it targets foot-liftoff prevention specifically, not
elastic energy storage or lateral compliance. Genuinely relevant to a
future P3/hardware design conversation about the F/T sensor mounting
(`design/sensors.yaml`), but out of scope for the current push-recovery
software investigation and not implemented here. No connection found to
this session's lateral push-recovery gap (six mechanisms tried, all
documented above) — this paper doesn't test or claim anything about
recovering from external pushes at all.

### 2026-09-09 — Design decision: keep all 13 joints actuated for the time being, defer the ankle_roll-to-passive reversion plan

User's explicit design decision, prompted by a question about whether
lateral push recovery is realistically achievable given `ankle_roll`'s
actuation was documented as "INTERIM, not the intended end state." The
answer worked through: `hip_roll`, `ankle_roll`, and `torso_roll` are
all currently actuated, so the six failed lateral push-recovery
mechanisms this session were never blocked by missing hardware
authority — the footstep-placement law's `b_nom_y` reference-point bug
is. But the DESIGN's stated long-term intent (return `ankle_roll` to
passive spring-centered) would, if acted on, remove the dominant
push-recovery lever (CoP modulation within the foot, +166% capture-
region area per Koolen/Pratt Part 1 vs. only +34% more from a reaction
mass/torso strategy) — real hardware (M2V2, Sarcos Primus) both keep an
actuated ankle for exactly this reason. Given that, the user chose to
defer the passive-ankle_roll plan rather than commit to it, keeping all
joints actuated for now.

**What changed (design-intent documentation only — no DOF, actuation
value, or numeric design constant actually changed; every joint was
already `actuated: true`):**
- `design/joints.yaml`'s `ankle_roll` comment: removed "INTERIM, not the
  intended end state" framing; documents the decision and its
  capturability-theory/real-hardware rationale.
- `design/geometry.yaml`: `lower_leg_module.compatibility.future` →
  renamed `possible_future_direction` (was never read by any generator
  or validator — confirmed by grep before renaming), reframed from "planned
  reversion" to "deferred, not scheduled."
- `design/mass.yaml`: comment wording only (mass ESTIMATES were already
  correct — they already used the actuated-hardware figure, not the
  smaller passive-hardware one).
- `templates/SPEC.md.j2`, `templates/MECH.md.j2`: prose updated to match
  (rehydrated into `SPEC.md`/`MECH.md` — diffs are prose-only, no
  numeric/DOF content changed, confirmed before committing).
- Root `CLAUDE.md`'s "Key Design Constants" section: same reframing.

**Explicitly NOT changed:** DOF count (still 13), any joint's `actuated`
value (all were already `true`), any limit/mass/geometry number. This is
a design-INTENT change (what the plan is for `ankle_roll`'s future),
not a design-VALUE change. The underlying software problem (lateral
push recovery still doesn't work, `b_nom_y` bug documented in the
entries above) is unaffected by this decision — actuation was never the
blocker, and keeping it actuated for the time being doesn't fix the
control-software gap by itself. That remains open, flagged for whenever
the user wants to resume it.

`preflight.py` clean: rehydration + all validators pass.

### 2026-09-09 — Tested whether the small step_length (80mm, a visible "shuffle" in the demo GIF) contributes to the ~30-step falling boundary: the opposite is true — larger steps fail almost immediately, not later

User's question, prompted by watching the rendered GIF: is the small,
shuffling step size part of why nominal walking can't continue
indefinitely (falls at n=31-32)? Tested directly rather than reasoned
about — `step_length` is hardcoded (`0.08`) inside `run_walk_recede`,
not exposed as a parameter; added a temporary env-var override,
diagnostic only, reverted after.

**Result: larger steps make it dramatically WORSE, immediately, not
better.** Baseline (0.08m) reproduces exactly: clean through n=30
(6.32°), degrading at n=31 (18.53°), fallen by n=32. `step_length=0.10`
(just 20mm more) already falls within the first 10 steps
(`max_tilt=56.75°`, `net_forward=-216mm` — walking backward). 0.12m and
0.15m are worse still (92.03°, 85.78°, both walking backward
immediately). Monotonic in the wrong direction — no step_length larger
than the current 80mm was found to survive even 10 steps.

**Conclusion:** this rules out the natural hypothesis that more,
smaller steps accumulate error faster than fewer, larger ones would.
The opposite holds here: the whole control stack (`K_DCM_RECEDE=-0.77`,
the `b_nom_x`/`b_nom_y` magnitude scaling, `MAX_FOOTSTEP_ADAPT_X_M`/
`_Y_M`, `WALK_AX_MARGIN_M`'s reach-margin/crouch-depth pairing) was
tuned — every single session's worth of narrow, non-monotonic,
non-transferable safe zones already documented above — SPECIFICALLY
around `step_length=0.08m`. It isn't an arbitrary conservative choice
being left on the table; it's a load-bearing tuning assumption baked
into multiple interacting constants. The small "shuffle" appearance and
the eventual ~30-step fall are most likely both SYMPTOMS of the same
underlying narrowly-tuned, fragile control architecture (matching this
session's now well-established pattern), not a direct cause-and-effect
relationship where bigger steps would extend stability. A genuinely
larger, more natural step length remains possible in principle, but
would need its own from-scratch coordinated re-tune of the same
constants re-tuned earlier today for `b_nom_y`/`K_DCM_RECEDE` — not a
quick change, and not attempted here.

`tools/sim_walk_recede.py` is unmodified (diagnostic override reverted,
confirmed via `git diff`).

### 2026-09-09 — Real CoP-repulsion law implemented, tested, and reverted: the literature's actual ankle mechanism has genuinely zero measurable effect on the 10N lateral push, once implemented correctly

User raised a fair, important concern mid-session: most of the b_nom_y
work above was narrow iteration on one lever, not genuinely different
ideas, and "proven techniques" (M2V2, Sarcos Primus) hadn't obviously
reproduced here. Investigating that concern directly turned up a real
gap: `ankle_roll_admittance` is a torque-feedback admittance law
(`correction = (tau_measured - tau_target)/k_adm`, `tau_target`
hardcoded to `0.0` everywhere) — NOT the CoP-repulsion law those real
robots actually used (Koolen/Pratt Part 2, Eq. 2:
`r~_CoP,des = r_ic + k_ic*(r_ic - r_ic,des)`, a target computed directly
from capture-point error). Despite being cited as the target mechanism
multiple times this session, it had never actually been wired in.

**Implemented it for real, Plan Mode approved.** The capture-point error
was already computed every tick (`dcm_err = xi_filtered - xi_planned`,
`sim_walk_recede.py:1281`, previously only used for the pelvis
correction) and `ankle_roll_admittance` already accepted a `tau_x_target`
parameter, unused. Added: an `ft_force_adr` lookup (the F/T sensor's
vertical-force channel, `{side}_ft_force`, already modeled in the MJCF
but not previously read in this file), a `K_IC` gain, and
`tau_x_target = K_IC * dcm_err[1] * F_z_measured` fed into the existing
admittance call. Verified as a true no-op at `K_IC=0` (exact match to
committed baseline on nominal walk and both push batteries) before
testing anything nonzero.

**First pass (unclamped) looked promising, then turned out to be an
artifact.** A broad sign+magnitude sweep against the 10N lateral push
(the system's actual failure boundary) found real hits: `K_IC=5`,
`-30`, `-50` all recovered 10N cleanly (~8° vs. falling at ~80°
baseline) — the first mechanism all session to move the needle past 5N
at all. But a finer sweep around that region (`K_IC` in [-25,-45], step
~3) showed the response was CHAOTIC: adjacent values flipped between
near-perfect recovery and total failure with no smooth trend (`-35`:
6.96° nominal / recovers both pushes; `-38`: 100.76° nominal / fails
everything; `-42`: recovers nominal, fails both pushes). Traced the
cause: at these gains, `tau_x_target` swings into the hundreds-to-
thousands of Nm during a real push — far past what's needed to saturate
`ankle_roll_admittance`'s own `+-10deg` correction clamp — so the
mechanism was operating in bang-bang chatter, not smooth proportional
control. The apparent "wins" were which specific point in an
oscillating, saturating signal a given `K_IC` happened to land on, not
genuine CoP authority.

**Fixed properly: clamp the desired CoP shift itself to a physical
bound, not just the resulting correction.** `generate_mjcf.py`'s foot
geometry (`size="0.075 0.040 0.020"`, MuJoCo box half-extents) gives a
real physical limit: the CoP cannot shift more than 40mm laterally
within the foot before it's off the edge entirely. Added
`FOOT_HALF_WIDTH_M = 0.040`, clamped `desired_cop_shift_y` to
`+-FOOT_HALF_WIDTH_M` BEFORE multiplying by `F_z` to get the torque
target (rather than letting an unbounded gain blow up and get chopped
downstream by the correction's own clamp).

**Result with the physically-correct clamp in place: no measurable
improvement, at any gain or sign.** Swept `K_IC` from -1000 to +1000
(both signs, four orders of magnitude) against the 10N lateral push:
every single value sits at 77-95° (falls), statistically indistinguishable
from the `K_IC=0` baseline (80.04°) — no trend, no sign preference, no
magnitude threshold that helps. The earlier "wins" were confirmed to be
entirely saturation-chatter artifacts; the real, properly-bounded
mechanism has genuinely no measurable authority over this specific
push. All changes reverted; `sim_walk_recede.py` is unmodified.

**Why this makes sense, and isn't actually a dead end:** this is
consistent with Koolen/Pratt Part 1's own numbers, read early this
session — finite-foot CoP modulation grows the capture region by +166%
over point-foot alone, but that's a BOUNDED, finite gain, not unlimited
authority. 40mm of real foot half-width is a small, hard physical limit.
If a disturbance is large enough, ankle/CoP authority ALONE — even
correctly implemented and maximally exercised — may genuinely not be
sufficient; the theory's own framework requires COMBINING it with
correct capture-region footstep placement for larger disturbances. And
footstep placement is the OTHER piece that's still broken (the `b_nom_y`
bug, documented in the many entries above) — so this result doesn't
contradict the literature, it's consistent with needing BOTH mechanisms
working correctly together, and only one of the two has ever been
correctly implemented (and even that one, in isolation, isn't enough).

**Where this leaves things, honestly:** the real CoP-repulsion mechanism
has now been correctly implemented and ruled out as a standalone fix.
Combined with the `b_nom_y` investigation (5 variants, all failed) and
reachability-based clamping (failed), the lateral push-recovery gap
remains open after what is now a genuinely thorough, structurally
diverse investigation — not further narrow iteration on one lever. The
two real remaining paths are: fix `b_nom_y` AND wire this CoP mechanism
together (since capturability theory says both may be required
simultaneously, and neither was tested working together with a
correctly-functioning partner), or accept this as the architecture's
current honest limit.

### 2026-09-09 — Path (a) attempted (Plan Mode, approved): EMA b_nom_y + re-wired CoP-repulsion, jointly re-tuned -- real nominal-walking win, but an 8th mechanism fails to move the lateral push ceiling, plus a significant baseline-drift discovery along the way

User's call, choosing between "fix b_nom_y properly + wire CoP-repulsion
together" and "accept the current limit": scoped via Plan Mode
(`.claude/plans/validated-floating-origami.md`) to implement the one
genuinely untried piece flagged in the b_nom_y magnitude-sweep entry above
-- a slowly-adapting, anchor-bounded EMA, not a static constant or a full
per-swing reset -- and layer the previously-reverted CoP-repulsion mechanism
on top, testing both together for the first time.

**Stage 1 -- EMA b_nom_y, a real nominal-walking win.** Implemented in
`run_walk_recede`: a per-swing measurement (identical formula to the
reverted full-recalibration attempt) blended into a persistent EMA at a slow
rate, anchored to k=-0.2 (NOT the "theoretically pure" -0.8636 derived
earlier -- anchoring there was tried first and immediately reproduced that
value's own already-documented catastrophic failure, confirming the earlier
sweep's conclusion that the rest of the stack is tuned around -0.2, not the
theoretical value), and clamped to stay within a bounded drift of that
anchor. `BETA_B_NOM`/`MAX_B_NOM_DRIFT_K` found by a bounded grid sweep --
non-monotonic and knife-edge, same character as every other gain margin in
this file -- with (0.01, 0.08) the best cell found.

**Significant discovery while establishing a baseline for that sweep: this
file's own push-recovery characterization was already stale.** The
`b_nom_y=-0.2x`/`K_DCM_RECEDE=-0.77` commit (`628b560`) predates `dca7009`
(finalizing ankle_roll's actuator mass in
`design/geometry.yaml`/`joints.yaml`/`mass.yaml`) -- the same class of
z_c/Tc-shifting design change already documented once before crossing this
file's K_DCM margins. Direct measurement against the CURRENT MJCF (not
runtime overrides -- the actual committed code) showed the documented "flat
through n=30" claim no longer holds (falls by n=25, 82deg), and more
importantly the push-recovery numbers had drifted too: even 5N lateral --
every prior entry's one always-safe push -- now falls (88deg), and sagittal
"4/5 survive" is down to 1/5. Nobody had re-validated push recovery after
`dca7009` landed. This means the CoP-repulsion entry above (`c511a4d`,
"zero measurable effect") was run against a baseline that had silently
degraded underneath it, though as Stage 3 below shows, re-running it on a
properly re-validated baseline reaches the same conclusion anyway.

**Stage 0 (added mid-session, user's call after the drift discovery) --
coordinate-descent re-tune against the current MJCF.** `K_DCM_RECEDE=-0.77`
and `MAX_FOOTSTEP_ADAPT_Y_M=0.03` re-confirmed as still locally optimal
(re-swept directly, not assumed). `K_ADM` had drifted: the previous shared
default (`lw.K_ADM=2500`) now falls by n=40; `5000` holds flat through n=45
(6.31-6.43deg). Added as `K_ADM_RECEDE`, this file's own value -- NOT pushed
back into `lw.K_ADM`, since `lw.run_walk`'s own margin for this value was
not re-verified this pass. This re-tune, combined with the EMA fix, is a
real, substantial, verified nominal-walking improvement: flat through n=45
where the actual current baseline was falling by n=25.

**Stage 3 -- the actual path (a) question, on the now-validated baseline:
still no.** Swept `K_IC` (the re-wired CoP-repulsion gain, foot-half-width
clamp now sourced from the MJCF model at runtime rather than a duplicated
literal) from -50 to 50 against the full lateral push battery
(push_at=2.0s, n_steps=18). The 10N ceiling never moves -- consistently
78-81deg regardless of sign or magnitude, indistinguishable from `K_IC=0`.
`K_IC` does perturb the 5N cell (non-monotonically -- some values survive,
most don't) but never the actual failure boundary. This is the EIGHTH
structurally distinct mechanism to fail at this specific question (four
b_nom_y fixes, reachability clamping, the coordinated re-tune, CoP-repulsion
alone, and now corrected-b_nom_y + CoP-repulsion together) -- and this time
tested on a baseline that is actually trustworthy, closing the loop the
earlier CoP-repulsion entry couldn't.

**What's kept, what isn't.** Committed: the EMA b_nom_y fix and
`K_ADM_RECEDE=5000`, both real and independently verified nominal-walking
wins (flat through n=45, all self-tests pass) -- kept per the same standing
practice as `628b560`, on the user's explicit call. NOT committed: any
nonzero `K_IC` -- the CoP-repulsion wiring ships present but inert
(`K_IC=0.0` default, exact no-op), available for a future attempt but with
no validated reason to activate it.

**Where this leaves lateral push recovery:** unresolved, after what is now
eight structurally distinct, independently-tested mechanisms, all
neutral-or-harmful on this specific question despite two of them (the
2026-09-08/09 re-tunes) producing real, kept nominal-walking gains along the
way. The push-recovery response itself looks chaotic/knife-edge at every
gain checked this session (K_DCM_RECEDE, Y_clamp, K_ADM, K_IC alike) --
exactly one isolated cell survives per axis in most sweeps, with no smooth
margin anywhere nearby -- which is itself informative: this may not be a
missing mechanism so much as a system operating with no real disturbance-
rejection margin at all at these force magnitudes, for reasons deeper than
any single gain or reference term. Accepting the current architecture's
limit at this force range is now a well-evidenced, not a premature, call.

### 2026-09-10 — Investigating the d_inf-vs-empirical-zero-effect discrepancy: the axis-mismatch in the margin calculation is real but only half the story, and the "receding horizon erases the correction signal" hypothesis is cleanly ruled out

The 2026-09-09 literature synthesis's `d_inf~=89mm` margin calculation was
the basis for prioritizing CoP-repulsion over hip/torso strategy. That
mechanism has since been correctly implemented and shown zero measurable
effect, even at 5N (12mm required shift, theoretically trivial against an
89mm margin). This entry investigates why, as a bounded diagnostic (Plan
Mode approved, `.claude/plans/validated-floating-origami.md`), not a new
control mechanism.

**Finding 1 -- the d_inf calculation mixed sagittal and lateral parameters.**
Reconstructing it (`d_inf = l_max*e^(-omega0*t_ss)/(1-e^(-omega0*t_ss)) +
r_max`) reproduces ~89mm using `r_max=0.075` (the foot's SAGITTAL
half-LENGTH) and `l_max=0.05` (`MAX_FOOTSTEP_ADAPT_X_M`, the sagittal
clamp) -- both sagittal values, applied to what was presented as a general
margin. Recomputed with the correct LATERAL values (`r_max=0.040`, the foot
half-WIDTH; `l_max=0.03`, `MAX_FOOTSTEP_ADAPT_Y_M`): **d_inf_lateral ~=
48mm**, on the current model (z_c=0.5258m, omega0=4.32rad/s -- both shifted
slightly from the original calc's inputs, consistent with the same
z_c/Tc-drift pattern documented elsewhere this session). This is a real
correction to that earlier claim, roughly half the originally stated
margin.

**But 48mm still doesn't explain the empirical result.** 10N requires only
24.1mm of capture-point shift -- half of even the corrected 48mm margin,
should be comfortably recoverable -- yet it reliably falls at ~80deg in
every push-battery run this session. The axis-mismatch correction is real
and worth keeping as the accurate number, but it closes roughly half the
gap, not all of it.

**Finding 2 -- ruled out: the receding-horizon architecture does NOT
structurally erase the correction signal.** Hypothesis: `replan_horizon`
rebuilds its reference every `REPLAN_PERIOD_S=0.1s` from the robot's actual
(possibly disturbed) current state, so `dcm_err` -- the signal driving
every correction mechanism tried all session -- could shrink toward zero
within ~1 replan period of a push without the robot actually recovering its
intended path, explaining why every mechanism shows an identical wall.
Added `push_at`/`push_force`/`push_duration` to `sim_walk_lipm.py`'s
`run_walk` (mirroring `run_walk_recede`'s existing convention exactly --
this function had no push-testing support before) specifically to test
this: `run_walk`'s FIXED whole-walk horizon, computed once and never
rebuilt from live state, is architecturally immune to this failure mode if
it's real.

**Result: `run_walk` fails in the exact same pattern.** Full lateral
battery (5/10/15/20/30N, push_at=2.0s, n=12 -- this file's own clean range
also drifted from the documented n=3-14 down to n<=12, same z_c/Tc-shift
pattern as `run_walk_recede`): 5N/15N/20N/30N all fall (91.71-95.76deg);
only 10N survives (8.30deg) -- one isolated lucky cell, at a COMPLETELY
DIFFERENT force than `run_walk_recede`'s own one surviving cell (5N, under
its current re-tuned defaults). An architecture that never re-anchors its
reference to the disturbed state fails just as completely, and just as
chaotically, as one that does. Hypothesis 2 is cleanly ruled out.

**What's left, and what's newly suggested by this result.** Two candidate
explanations were investigated; neither fully explains the empirical
result, but the pattern in Finding 2's data is itself a clue: TWO
architecturally different controllers each survive exactly one isolated
force value, and those values don't match each other. That's much more
consistent with high sensitivity to the PRECISE PHASE of the gait cycle at
which a push lands (single- vs double-support, swing progress fraction)
than with anything about corrective authority, gain tuning, or reference-
tracking architecture -- if survival depended on those, the same force
should behave consistently across similar configurations, not flip
unpredictably. This has a precedent already in this file (2026-09-08: "same
15N lateral push, six different push times... fell at nearly every timing
too"), but that check never found a survival case, and was never re-run
against the current (re-tuned, drift-corrected) baselines with a FINE
timing sweep at a single fixed force to characterize the shape of that
sensitivity. Flagged for the user's call before pursuing it -- this would
be a genuinely different diagnostic than anything tried so far (sweeping
WHEN the push lands, not what mechanism resists it), not a continuation of
either hypothesis in this entry.

**Kept:** `push_at`/`push_force`/`push_duration` support in `sim_walk_lipm.
py`'s `run_walk` -- real, reusable diagnostic infrastructure, additive and
no-op when `push_at=None`, `--selftest` re-verified clean.

### 2026-09-10 — Push-timing sensitivity swept: confirms genuine chaos, not a discoverable timing rule -- closes out the push-recovery investigation

Direct follow-through on the entry above's own suggestion: since two
architecturally different gaits (`run_walk`, `run_walk_recede`) each
survived exactly one isolated force value in their push batteries, at
DIFFERENT values, the dominant variable might be the exact gait-cycle PHASE
a push lands in, not force magnitude or which mechanism resists it. Swept
`push_at` (not force) at a FIXED 10N lateral push, `run_walk_recede`, n=18,
current committed defaults.

**Coarse sweep (0.06s steps, 1.8-3.6s, ~3 step cycles):** narrow, sporadic
survival islands (2.52-2.58s: 6.8-6.9deg; an isolated single point at
3.06s: 7.89deg; 3.24-3.30s: 15.0/6.3deg) scattered among 26 falls
(75-95deg). Critically, **survival does NOT repeat at the step period**: a
point exactly one step (0.6s) after the 2.52s survivor (i.e. 3.12s) is
itself a fall (91.47deg) -- ruling out a simple "single-support bad,
double-support good" phase rule, which would predict periodic repetition.

**Fine zoom (0.01s steps, 2.95-3.15s) around the isolated 3.06s spike:**
fragments rather than resolving into a real window with defined edges --
3.03s and 3.04s survive (8.46/8.69deg), 3.05s falls catastrophically
(89.96deg), 3.06s survives again (7.89deg), 3.07s falls again (79.09deg).
A single 10ms shift in push timing flips the outcome between "recovers
cleanly" and "falls", more than once, non-monotonically, within a 40ms
window. This is not measurement noise (the sim is deterministic) and not a
narrow-but-real resonance (a real resonance would show smooth edges, not
flip twice within 40ms) -- it is genuine sensitive dependence on initial
conditions.

**Conclusion -- this closes out the push-recovery investigation.** Ten
structurally distinct investigative approaches have now been tried against
lateral push recovery at the 5-30N range: four `b_nom_y` fixes,
reachability-based clamping, a coordinated whole-stack re-tune,
CoP-repulsion alone, corrected-b_nom_y + CoP-repulsion together (path (a)),
the d_inf-margin/receding-horizon-architecture diagnosis, and now this
timing-sensitivity characterization. Every one converges on the same
picture from a different angle: this is not a missing mechanism, not a
mistuned gain, and not an architecture defect -- it is a system that is
genuinely chaotic (in the technical sense: deterministic, but with outcomes
arbitrarily sensitive to timing at the millisecond scale) at these
disturbance magnitudes, for reasons that go deeper than any control law
this session was in a position to change. No further local mechanism,
gain re-tune, or timing-avoidance rule can fix genuine chaotic sensitivity
-- by definition, arbitrarily small timing/state differences produce
arbitrarily different outcomes. Accepting this as the current
architecture's real limit at this force range is now the well-evidenced
conclusion, not a premature one.

### 2026-09-10 — sim_walk_gait.py's stale-vs-actuated ankle_roll mismatch fixed; the 3-step regression itself resists six independently-tested parameters, all consistent with a step-3-specific compounding instability

Spot-checking all five documented P3 milestones against the current MJCF
(prompted by the drift already found twice this session) found
`sim_walk_gait.py` -- one of the "done" milestones -- now falls at 60.01deg
after only 2/3 steps. Unlike the other two files' regressions, this one
wasn't ordinary gain drift: the file's own docstring says its lateral
strategy (hip_roll ZMP feedback) was designed BECAUSE ankle_roll was a
"passive, spring-centered" joint at the time. That's no longer true --
ankle_roll became fully actuated (2026-09-09 decision) -- but this file
never added a controller for it, so it sat rigidly at 0deg via `generate_
mjcf.py`'s stiff `kp=150` position servo the whole run, nothing like the
compliant joint the strategy assumes. User's call: fix the control law.

**Fix implemented and correctly wired.** Added `left_ankle_roll`/
`right_ankle_roll` to `Gait.act`, an `ft_torque_adr` lookup (same pattern
as `sim_walk_lipm.py`/`sim_walk_recede.py`), a `mj_forward`+`mj_
rnePostConstraint` seed call in `run()` (this file had neither before --
also fixes `Gait.__init__`'s own `self.initial_height` read, previously
taken pre-forward-kinematics), and a per-tick call to the already-validated
`lw.ankle_roll_admittance` (imported from `sim_walk_lipm.py`, not
reimplemented), gated by a new file-scoped `K_ADM_GAIT`.

**But the actual regression resists it, and five other parameters
besides.** Root-caused first: the fall is PURELY roll, not pitch (traced
axis-resolved -- pitch stays bounded -40 to +9deg throughout; roll grows
from ~0 to 60+deg, accelerating, with pitch flat near 0deg exactly when
roll takes off). Roll starts drifting mid-way through STEP 2 (not step 3),
partially oscillates in sign for the first ~1.5 steps (consistent with the
schedule's intended alternating-side symmetry), then stops reversing and
grows monotonically from ~step 2's shift phase onward, through step 3.

Six parameters swept against the `--steps 3` pass criterion, all
independently negative:
- `K_ADM_GAIT` (new ankle_roll admittance gain): 0-15000 initially showed
  ZERO variation -- traced to a units mistake (typical torque is only
  ~1.2-1.6Nm mean, so `tau/k_adm` at k_adm>=100 is a fraction of a degree,
  nowhere near enough to matter); re-swept 1-100 (spanning negligible to
  fully-saturated correction) -- still no improvement, and the saturated
  range (2-30) is actively WORSE (fails a full step earlier, at step 2).
- Sign-flipped admittance, same range: identical negative pattern --
  rules out a sign-trap (this codebase has real precedent for those, e.g.
  `ankle_roll_admittance`'s own doc comment, but not here).
- `ROLL_KP` (the pre-existing hip_roll ZMP gain, untouched since before
  the `dca7009` mass/geometry shift that affected every other gain this
  session): 0.5-5.0, no improvement; 2.0-3.0 actively worse.
- `solve_hip_roll_shift(model)`: re-verified geometrically exact at
  4.250deg, matching the historical value precisely (pure kinematics, not
  mass-dependent, so `dca7009` shouldn't have moved it, and didn't).
- `ANKLE_KP_SINGLE`/`ANKLE_KP_DOUBLE` (sagittal gains): 1-10 and 0.1-3
  respectively, no improvement (expected, given the failure is pure roll,
  but checked rather than assumed).
- `PARTIAL_BALANCE_ALPHA` (stance-knee balance blend, could plausibly
  shift lateral pendulum dynamics via effective leg length): 0.0-1.0, no
  improvement.

**A clarifying note on the data, not a new finding:** every failing run's
reported `max_tilt` clusters tightly at 60.0-60.3deg regardless of which
parameter or value was swept. This is NOT evidence all six parameters
produce equivalent dynamics -- the sim halts the instant tilt crosses
`MAX_TILT_DEG=60.0`, so that number is just "the value at the moment it
stopped," not a growth-rate or margin signal. The informative metric is
`steps_completed`: every one of the ~50 configs tested across six
parameters stayed at 2 (same failure point as baseline) or regressed to 1
(worse); none reached 3.

**Where this leaves things.** The ankle_roll actuation mismatch is real
and now correctly fixed in code (kept, not reverted -- real infrastructure
fixing a real staleness bug, same precedent as path (a)'s `K_IC` shipping
present-but-inert). The 3-step milestone itself remains broken: six
independently-tested parameters, all negative, consistent with a
compounding instability specific to how residual roll carries from step 2
into step 3 (the same "local gains can't fix a structural compounding
problem" theme as this session's much longer lateral-push-recovery
investigation above) rather than any single mistuned constant. `sim_walk_
gait.py` is already explicitly superseded by `sim_walk_lipm.py` for the
smooth-walking goal per its own docstring -- given the depth already
invested (six parameters, ~50 configs) without a working value, further
investigation here should be its own deliberately-scoped effort, not
open-ended continuation.

### 2026-09-10 — P3 milestone audit: 2 of 5 documented quick-reference commands are currently broken, and nothing in preflight/CI would have caught it

Prompted by finding the same silent drift three separate times in one
session (`sim_walk_recede.py`'s nominal-walk claims, `sim_walk_lipm.py`'s
`run_walk` clean range, `sim_walk_gait.py`'s 3-step milestone), ran every
P3 validation script exactly as the root `CLAUDE.md` quick-reference table
documents it, against the current committed code + MJCF:

| Command (as documented) | Result |
|---|---|
| `sim_static_pose.py` | PASS |
| `sim_zmp_balance.py` | PASS |
| `sim_walk_gait.py` | **FAIL** — falls at 60deg, 2/3 steps |
| `sim_walk_lipm.py --steps 15` | **FAIL** — falls at 90.27deg |
| `sim_walk_recede.py --steps 8` | PASS (6.31deg) |

**Two of the five documented commands fail**, and one of the five
milestones this file's own "Where work stopped" section lists as done
(three-step quasi-static walking) has quietly regressed. `sim_walk_lipm.
py`'s own docstring claims n=3-14 clean at <=6.4deg; measured now, n=12 is
the real boundary (n=14: 69.44deg, n=15: 90.27deg). The `--steps 15`
figure in the root quick-reference table is therefore also stale.

**Root cause is the same in every case, and it is a PROCESS gap, not a
control-law one.** `dca7009` (finalizing ankle_roll's actuator mass in
`design/geometry.yaml`/`joints.yaml`/`mass.yaml`) shifted z_c/Tc after
those gains were tuned — the same class of margin-crossing effect
`K_DCM_RECEDE`'s own comment already documents happening once before for
this exact joint. Nothing re-ran the sims afterward, so three files'
documented claims silently went stale and were then used as trusted
baselines by later investigations (including this session's own
CoP-repulsion work, which measured "zero effect" against a baseline that
had already degraded underneath it — see that entry above).

**Why nothing caught it:** `preflight.py` and both CI workflows
(`validate.yml`, `rehydration-check.yml`) run only the cheap validators
(`validate_dof_consistency.py`, `validate_no_geometry_literals.py`) and
the rehydration diff. No MuJoCo sim runs at all. A `design/*.yaml` edit
can therefore invalidate every walking-gait claim in the repo, pass all
gates, and merge clean. Given this has now happened at least once with
three-file blast radius, wiring a cheap sim-regression gate (even just
`sim_static_pose.py` + `sim_zmp_balance.py` + short walking runs at the
documented step counts) into preflight or CI is a real, identified gap —
NOT yet done, deliberately: fixing it requires first restoring a passing
baseline for the two broken commands, otherwise the gate lands red.
`sim_walk_recede.py` was re-tuned this session (`ca8925d`);
`sim_walk_lipm.py`'s `run_walk` and `sim_walk_gait.py` have not been.

**Standing accuracy note for anyone reading the docs:** treat any
performance number in a docstring or the root quick-reference table as
"true when written," not "true now," unless it postdates `dca7009`. The
numbers in this entry and the three above it are measured against the
current model.

### 2026-09-10 — Torque envelope modelled and IMU added: standing survives, all three walking gaits do not, and the state estimator turns out to be good enough

Two design revisions (`96ea7de`, `9b16f7b`) and their consequences. Both
came out of stepping back over the candidate control architectures, which
found that every P3 result to date rested on two things simulation was
quietly providing and hardware never could: unlimited joint torque, and a
CoM state the sensor suite could not measure.

**1. The torque envelope. `design/actuation.yaml` was an empty stub.**
It now records the drivetrain: 775 brushed DC at 24V, 20:1 planetary,
0.70 efficiency, giving **2.8 N·m per joint**, emitted into the MJCF as
`forcerange` on all 13 actuators (previously absent entirely). Belt stages
are recorded as 1:1 -- deliberately conservative, and a KNOWN
understatement for six joints, since BOM.csv lists secondary belt
reduction for hip_pitch/hip_roll/knee_pitch that no design file has ever
given a ratio for.

**Result -- standing passes, walking collapses:**

| Check | Before (unlimited) | With 2.8 N·m |
|---|---|---|
| `sim_static_pose.py` | PASS | PASS (needs 1.33 N·m/leg) |
| `sim_zmp_balance.py` | PASS | PASS (0.33deg tilt) |
| `sim_walk_recede.py --steps 8` | 6.31deg | **77.59deg, falls** |
| `sim_walk_lipm.py --steps 12` | ~6.4deg | **91.58deg, falls** |
| `sim_walk_gait.py --steps 3` | 152mm swing progress | **17mm, barely moves** |

Measured demand with the cap lifted (receding-horizon gait, 8 steps):
peak **37.95 N·m** at right_knee_pitch, but that peak is largely a stiff
position-servo transient and overstates the case. The honest figures are
**RMS 4.08 N·m (1.5x the envelope)**, **p95 8.50 N·m (3.0x)**, and
**28.1% of all samples over budget**. Eleven of thirteen joints exceed
2.8 N·m at peak.

Two observations worth carrying forward. First, the six joints BOM lists
secondary belt reduction for are exactly the highest-demand ones
(right_knee_pitch, both hip_pitches) -- a ~3:1 belt stage would put them
near 8.4 N·m, which matches their measured p95 almost exactly. Second,
that still would not close it: ankle_pitch demands p95 8.38 N·m with no
belt reduction listed at all. So the drivetrain gap is real and is NOT
uniformly distributed.

**2. The IMU and the estimator.** `design/sensors.yaml` gained a 6-axis
pelvis IMU (no magnetometer -- 13 brushed motors nearby; yaw comes from
leg odometry, with drift accepted and reported). `tools/state_estimator.py`
estimates CoM / CoM velocity / DCM from encoders + foot F/T + IMU ONLY,
and is scored against MuJoCo truth it never reads.

**A real sign trap, in this file's long tradition of them.** The
complementary filter's accelerometer correction was written as
`cross(up_pred, up_meas)`. That is a stable-but-wrong feedback loop: it
drives the orientation estimate to the ANTIPODE and parks there. Symptom
was a ~180deg flip about Y and ~170deg of apparent yaw drift; CoM error
was 1108mm. Deriving it properly (applying `corr` as a body-frame
increment makes the new body-frame world-up `R_corr^T * up_pred`, so
`R_corr` must rotate up_meas -> up_pred) gives `cross(up_meas, up_pred)`.
CoM error fell 1108mm -> 60mm. Checking the accelerometer's sign
empirically first was what narrowed it -- MuJoCo reads +z when upright,
so that half was already right.

**Result -- the estimator is good enough, which was not the expected
answer.** On a clean (unlimited-torque) 8-step run:

| Quantity | RMS error |
|---|---|
| CoM, absolute world frame | 7.69 mm |
| CoM velocity | 103.5 mm/s |
| DCM, absolute | 25.57 mm |
| yaw drift over 4.5s | 0.04 deg |
| **CoM relative to stance foot** | **4.58 mm** |
| **DCM relative to stance foot** | **25.81 mm** |

The relative figures are the ones that matter: a balance controller
consumes CoM *over the support polygon*, not absolute world pose, and
odometry drift cancels in that difference. 4.58 mm against a 40 mm foot
half-width and the corrected ~48 mm lateral capturability margin is
comfortable. DCM-relative at 25.81 mm is about half the margin --
significant, not disqualifying.

**But the estimator tuning is not yet trustworthy, for the same reason
the old torque results weren't.** Sweeping the accelerometer trust showed
LESS correction is monotonically better, with zero correction best by a
wide margin (0.48 mm CoM-rel). That is an artifact: MuJoCo's gyro is
noise-free and bias-free, so pure integration is flawless in simulation
and divergent on hardware. Tuning to it would repeat, in the estimator,
exactly the mistake the torque envelope just corrected in the actuators.
`ACCEL_TRUST_ALPHA` is therefore set to 0.001 (~2s time constant, a
defensible textbook value) rather than the 0.0 the sim rewards, and the
constant is marked as un-tunable until the MJCF models IMU noise and
bias. **Treat every estimator figure above as a LOWER BOUND.**

**Where this leaves the eight architectures.** The observability
objection is substantially answered -- with the IMU, the balance-relevant
state IS estimable to within the stability margin, so B/C/D/E/G are no
longer gated on sensing. They are now gated on actuation instead, and
harder: the gaits those architectures produce demand 1.5x sustained and
3x p95 more torque than the drivetrain can deliver. The binding
constraint has moved from "the robot cannot know where it is" to "the
robot cannot push hard enough", which is a more tractable problem with
two obvious levers (more reduction, or a controller that respects the
envelope -- note this is precisely the constraint-aware MPC that
Stephens' ladder credits with the largest single push-recovery gain).

Not done, deliberately: the estimator is NOT wired into any controller.
It was built to measure the gap, and it did. Wiring it in means the
controllers stop consuming ground truth, which will regress them on top
of the torque regression -- two variables at once, and the plan was
explicit about not doing that in one pass.

### 2026-09-10 — Architecture D, Stage 0: the torque gap is NOT a servo-gain artifact, and the reason is sharper than either hypothesis — there is no kp that both stands and walks

Stage 0 of the Architecture D build (constraint-aware LIPM MPC, chosen after
the full architecture review). Its purpose was to settle a confound left
open by the entry above: `kp=150` was chosen when torque was unlimited, and
against a 2.8 N·m cap it saturates at **1.07deg of tracking error**, so
"the drivetrain cannot power this gait" and "the servo gain is 5-10x too
stiff for this actuator" produce identical symptoms. A peak 9x the RMS is
the classic signature of the latter.

**Swept kp against the receding-horizon gait, 8 steps, real 2.8 N·m cap:**

| kp | saturates at | max tilt | % samples at limit | RMS N·m |
|---|---|---|---|---|
| 150 | 1.07deg | 77.59 | 29.1% | 1.78 |
| 100 | 1.60deg | 78.34 | 33.9% | 1.90 |
| 64 | 2.51deg | 91.36 | 26.6% | 1.65 |
| 32 | 5.01deg | 93.12 | 11.1% | 1.31 |
| 16 | 10.03deg | 92.65 | 6.1% | 1.05 |
| 8 | 20.05deg | 90.73 | **0.4%** | 0.79 |

Lowering kp does exactly what it should to saturation -- 29.1% -> 0.4%,
RMS 1.78 -> 0.79 N·m, i.e. the drivetrain ends up with plenty of headroom.
**And the robot still falls at every single value.** So the walking failure
is not torque saturation, and the confound is settled in the direction that
does NOT let the drivetrain off the hook.

**Why no kp works, which is the actually interesting part.** Checked
whether the robot can even STAND across kp (`sim_zmp_balance.py`, 5s):

| kp | standing, 2.8 N·m cap | standing, unlimited torque |
|---|---|---|
| 150 | 0.33deg (OK) | 0.33deg (OK) |
| 64 | 32.74deg (falls) | 36.05deg (falls) |
| 32 | 34.07deg (falls) | 36.19deg (falls) |

It falls at kp=64 *with torque unlimited*, so that failure is **pure
compliance, not torque**. Standing needs only ~1.33 N·m/leg, comfortably
inside the envelope. What breaks is the pose: `sim_zmp_balance.py`'s
balanced crouch is FK-solved and, per that file's own docstring, is stable
*with no active control at all* -- but only while the joints hold it
near-rigidly. Soften them and the robot sags out of the geometry that was
doing the balancing, and no amount of torque recovers it.

**Conclusion: the position-servo stiffness window is empty at this torque
envelope.** Stiffness is doing structural work here (holding a statically
balanced pose), not just trajectory tracking. High kp is required to hold
the pose and guarantees saturation during dynamic motion; low kp removes
the saturation and loses the pose. kp=150 is the only value that stands,
and at kp=150 walking clips the torque limit 29% of the time.

**What this means for the previous entry, stated plainly.** Its headline --
"the drivetrain cannot power this gait" -- survives, but the mechanism was
described too loosely. It is not simply that the motors are 1.5-3x too
weak in isolation. It is that a stiff position servo *tracking a
trajectory generated without knowledge of the torque limit* will demand
whatever that trajectory costs, and this one costs more than the drivetrain
has. The gain is not the bug and re-tuning it is not the fix.

**Which is precisely the argument for D**, and worth recording as the
reason to keep going rather than as a setback: an MPC that plans the CoP
inside the feasible box never commands the infeasible trajectory in the
first place, so the saturation has no opportunity to arise. It is also an
argument for F (HUBO-style local feedback), where compliance is handled
explicitly per-joint instead of being an emergent property of one global
stiffness constant. `kp` is therefore left at 150 -- not endorsed, but the
only value the current pose survives, and changing it belongs with a
control architecture that does not depend on near-rigid joints to stand up.

### 2026-09-10 — Architecture D, Stages 1-2: the QP is correct and the CoP cannot be delivered — a negative result, with the failing component identified precisely

Stage 1 built the constraint-aware MPC core (`tools/mpc_lipm.py`); Stage 2
wired it to standing balance (`tools/mpc_balance.py`) against
`sim_zmp_balance.py`'s proportional ankle law, which is the one
walking-adjacent thing still passing under the real torque envelope.

**Stage 1: the QP is sound.** Condensed LIPM MPC, per axis, box-constrained.
`--selftest` gates all pass exactly: condensed rollout vs step-by-step
integration 2e-15; wide bounds vs closed form 0.0 in 2 iterations; KKT
residual 0.0 with 16/16 bounds active; active entries exactly on the bound;
warm vs cold start 0.0, converging in 1 iteration vs 11. Probed in
isolation it behaves sensibly: 0 mm CoM error -> 0 mm CoP interior, 3.5 mm
-> 17.7 mm interior, 20 mm -> at the bound.

Solver is projected Newton, not FISTA as the plan specified. FISTA was
written first and stalled at 3e-5 against the closed form after 20000
iterations -- the Hessian is badly conditioned (DCM tracking over a horizon
is nearly rank-deficient). The plan's requirement was the self-test gates,
not the algorithm; the gates were kept and the solver changed.

**Stage 2, finding 1: feedforward CoP realisation does not work.** The plan
flagged "realise a desired CoP on a position-controlled robot" as the one
genuinely unproven step. Commanding `theta_nominal + tau_des/kp`, sized so
the servo's own stiffness delivers the desired ankle torque, measured:

| axis | commanded | delivered | gain |
|---|---|---|---|
| lateral | 40 mm | 2.5 mm | 0.06 |
| sagittal | 5 mm | -2.6 mm | -0.52 |
| sagittal | >=20 mm | falls | -- |

The stiff servo settles at a different equilibrium; delivered steady-state
torque is not `kp * offset`, because `theta_actual` moves in response.

(Two of my own errors on the way, both recorded because they shaped the
design. Anchoring the offset at MEASURED angle instead of nominal removes
the servo's absolute reference entirely and collapses the pose immediately
-- the same failure Stage 0 showed the crouch cannot survive. And an early
sign calibration reported sagittal sign=-1 as correct at +62 mm; that was
measured mid-fall, and with proper settling sagittal works at neither sign.)

**Finding 2: closed-loop torque tracking works, laterally.** Driving the
offset with an integral loop on the MEASURED ankle torque from the foot F/T
sensor, `offset += k_i * (tau_des - tau_meas)`, realises lateral CoP at
**gain 1.01** (20 mm commanded -> 20.25 mm delivered). This is admittance
control with a non-zero target -- exactly what `ankle_roll_admittance`'s
`tau_x_target` parameter was added for earlier this session and left unused.

**Sagittal resists both, structurally.** `ankle_pitch` is load-bearing for
the balanced crouch, so a torque loop overriding it removes the support
holding the robot up and it falls at either sign. The architecture was
therefore split by axis: MPC governs LATERAL (the axis that has failed every
push test this project has run), sagittal keeps the proportional law.

**Finding 3: a clamp inherited without thinking made it worse than useless.**
The first battery had the MPC falling at EVERY push including 5 N, while the
baseline barely registers 5 N at 0.30deg. Cause was reusing
`ankle_roll_admittance`'s +-15deg clamp for the closed-loop roll offset.
Fifteen degrees of ankle roll on a foot 40 mm wide is not authority, it is a
tipping moment: the integral winds up under a push and levers the robot over
its own foot. Swept: 15deg -> 60.04deg (falls, bound active 65%), 8deg ->
2.70deg, 4deg -> 0.97deg with bound activity collapsing to 2%. The 65% bound
activity was the tell -- the QP was pegged because it was fighting a plant
its own actuator command had destabilised.

**Result with that fixed -- still a regression against the baseline:**

| F_y | MPC tilt | bound active | baseline tilt |
|---|---|---|---|
| 5 N | 0.86 | 0.0% | 0.30 |
| 10 N | 0.97 | 2.2% | 0.51 |
| 15 N | 1.25 | 6.5% | 1.04 |
| **20 N** | **60.01 falls** | **62.8%** | **1.77** |
| 30 N | falls | 47.5% | falls |

The MPC is worse on every push it survives and falls at 20 N, which the
baseline handles comfortably. Neither moves the 30 N ceiling.

**And it is not a tuning miss.** Swept the roll clamp 4-12deg against
15/20/25/30 N: every value falls at 20 N and above, while larger clamps
monotonically degrade the 15 N case (1.25 -> 36.96deg). There is no window.

**Conclusion: the QP is not what failed -- the actuation path is.** Bound
activity goes 0.0 -> 2.2 -> 6.5% across 5/10/15 N and then jumps to 62.8% at
exactly the push that is lost. The QP correctly detects it has run out of
feasible CoP; what it cannot do is make the CoP it asks for actually appear.
The torque-derived bound (+-29.7 mm single support) turns out to be
optimistic: the REALISABLE CoP authority through a position servo and a
40 mm-wide foot is smaller still, so the MPC plans against a box it cannot
reach.

The baseline wins for an instructive reason. It never tries to place the CoP
at all -- it is a direct pose correction (`ankle_pitch <- kp*(target -
zmp_filtered)`) that the position servo executes natively. The MPC inserts
desired CoP -> desired torque -> integral loop -> angle, and authority is
lost at every layer. **On a position-controlled robot, commanding position
directly beats commanding position in service of a force objective.**

**Where this leaves D, honestly.** Its premise still holds -- planning inside
the feasible set is the right idea, and the torque/CoP identity that makes
the constraint expressible is real and verified. But D as specified assumes
force control (Stephens' Sarcos Primus was hydraulic and force-controlled),
and the admittance bridge that was supposed to substitute for it delivers
enough authority for 15 N and not 20 N. Options, none yet chosen: take D to
Stage 3 anyway on the theory that its value is in choosing footsteps and
timing during WALKING rather than in standing CoP micro-placement; or accept
that this result is a direct argument for architecture F, whose whole premise
is local per-joint compensators rather than a global model pushed through a
lossy actuation path. Flagged for the user's call rather than continued solo.


### 2026-09-10 — Drivetrain sized: 80:1 committed as an interim, and it recovers the primary walking milestone

The user's call, after the architecture review put actuation as the binding
constraint: fix the gearing rather than build another controller.

**Torque floor.** At the previous 20:1 (2.8 N·m/joint) the validated
receding-horizon gait clipped its limit on 28.1% of samples and fell
(77.59deg). Sweeping the ratio: 3x (60:1) is the first value that walks --
6.32deg, matching the unlimited-torque baseline -- and 4x (80:1) leaves
margin for the motor's torque-speed droop (0.4% of samples short, vs 1.8%
at 3x). Committed 80:1 = 20:1 gearbox x 4:1 belt, giving 11.2 N·m.

**The belt must include the ankles, or no ratio works.** BOM.csv listed
secondary belt reduction on six joints (hip_pitch, hip_roll, knee_pitch).
At that scope the gait NEVER walks, at any ratio tested to 6x -- raising
hip and knee simply promotes `ankle_pitch` (p95 demand 8.38 N·m) to the
binding joint. BOM now covers all 13. HUBO KHR-3 independently agrees: it
puts a 2:1 pulley-belt on BOTH ankle axes over a 100:1 harmonic drive.

**Speed ceiling, from the project's own CMU motion-capture study**
(`reference-material/motion-capture-cmu`, previously uncited here). Its
regression gives knee omega_95 = 11.0 rad/s per leg-length/s. At matched
normalised speed -- where stride time is invariant, so angular velocities
transfer across scale -- it predicts our gait's knee omega_95 as 2.44 rad/s
against a simulated 2.48. **Two percent agreement, which both validates our
gait as kinematically human-realistic and validates the study for
extrapolation.** It caps the knee at ~123:1 for HUBO-speed walking, ~74:1
for fast human walking. It also endorses the sizing statistic: the study
uses 95th percentile "robust to spikes", which is exactly why sizing
against our 37.95 N·m peak (a kp=150 servo transient) was wrong.

**HUBO KHR-3 leg, from the paper** (now in reference-material/humanoid-
robotics/KAIST, read directly): hip_roll 120:1 x gear 2.5:1 = 300:1,
hip_pitch 160:1 x belt 1.8:1 = 288:1, knee 120:1 x belt 1:1 with **two**
150W motors, ankle roll/pitch 100:1 x belt 2:1 = 200:1. Brushed 24V DC
motors -- the same actuator class as megadroid. The per-joint spread is
not arbitrary: ratios scale INVERSELY to each joint's speed demand, and
HUBO's knee (120:1) sits essentially on our computed ceiling (123:1).

**What could NOT be measured, and it matters.** Torque demand at HUBO
walking speed (1.25 km/h). The controller only walks at 0.36-0.48 km/h.
Three attempts failed: slowing the cadence detunes the controller into
falling (0.9s gave 84.99 N·m from a robot toppling, not a gentler gait);
inverse dynamics on the planned trajectory disagreed with the forward sim
by 2x (16.68 vs 8.50 N·m) and was abandoned when that validation failed;
longer steps fall above 0.10 m. The two speeds that do walk are not even
monotonic in torque, so there is no trend to fit. Scaling bounds the
requirement at 1.25 km/h to 143-357:1, which starts ABOVE the 123:1 knee
speed ceiling -- meaning **a single 775 per joint may not close at that
speed at any ratio.** HUBO's answer to the same collision was two motors
on the knee. Recorded in actuation.yaml's `scope_limitation`.

**P3 battery at 80:1 (was 2.8 N·m -> now 11.2):**

| check | at 2.8 N·m | at 11.2 N·m |
|---|---|---|
| `sim_static_pose.py` | PASS | PASS |
| `sim_zmp_balance.py` | PASS | PASS |
| `sim_walk_recede.py --steps 8` | 77.59deg fall | **8.76deg, WALKS** |
| `sim_walk_lipm.py --steps 12` | 91.58deg fall | 91.59deg fall |
| `sim_walk_gait.py --steps 3` | fell | fell |

**The primary receding-horizon milestone is recovered.** The other two are
not, and for reasons that are NOT torque. `sim_walk_lipm` demands only
7.23 N·m p95 -- comfortably inside 11.2 -- and clips on just **0.1% of
samples**, yet still falls. That is the same knife-edge sensitivity this
file documents elsewhere (10 ms of push-timing flips outcomes; adjacent
gain values swing between clean and falling): 0.1% clipping is enough.
`sim_walk_gait`'s failure has its own separate, already-documented root
cause (roll runaway, six parameters ruled out).

**Standing caveat:** 80:1 is scoped to the 0.48 km/h gait that exists, not
to ASIMO/HUBO-class 1.25 km/h, and the per-joint structure HUBO uses is
the eventual right answer rather than a uniform ratio. Both are recorded
in `design/actuation.yaml` rather than left implicit.

### 2026-09-10 — CoP-repulsion re-tested with a working actuation path: exhausted, and the original negative result is finally explained

The 80:1 drivetrain and the Stage 2 realisation findings made one previously
inconclusive mechanism worth re-opening. It is now closed properly.

**First, why it never had a chance -- the missing explanation.** Two prior
investigations recorded CoP-repulsion as having "zero measurable effect"
without explaining why. The reason is the actuation path. The law computes
`tau_x_target = desired_cop_shift_y * F_z`, clamped to the foot half-width.
A FULL-FOOT demand (40 mm at body weight) is 3.77 N·m -- and routed through
`ankle_roll_admittance`'s proportional law at `K_ADM_RECEDE=5000`, that
yields **0.043 degrees of ankle_roll**. The mechanism was mis-plumbed, not
merely ineffective: no gain, sign or baseline could have made it act,
because a maximum demand produced a twentieth of a degree of motion.

Architecture D Stage 2 measured the same thing independently and by a
different route: proportional realisation gain 0.06, integral 1.01.

**Correcting my own claim from earlier today.** I attributed the original
CoP verdict to a degraded baseline and torque-limited ankles. That was
wrong on both counts -- path (a) Stage 3 had already re-run the K_IC sweep
on the re-tuned baseline under UNLIMITED torque and still found nothing, so
neither the baseline nor the torque envelope was the explanation. The
0.043deg authority figure is.

**The proper re-test.** Re-routed CoP-repulsion through the integral
realisation Stage 2 measured at gain 1.01 (`offset += k_i * (tau_target -
tau_measured)`, clamped to the 4deg Stage 2 validated), at the new 80:1
envelope, against the lateral push battery. Experimental source patch, not
committed.

| K_IC | 5N | 10N | 15N | 20N | 30N |
|---|---|---|---|---|---|
| **0 (baseline)** | **7.76** | 77.21 | 79.12 | 98.17 | 76.57 |
| -1 | 89.79 | 90.95 | 78.67 | 77.09 | 77.22 |
| -5 | 78.18 | 77.91 | 77.58 | 76.96 | 78.03 |
| -60 | 77.81 | 78.31 | 79.16 | 78.45 | 78.66 |
| +5 | 90.51 | 78.55 | 78.44 | 78.18 | 78.68 |
| +60 | 91.13 | 82.05 | 78.65 | 78.03 | 78.80 |

**Every nonzero K_IC is WORSE than the baseline**, at both signs and every
magnitude tested -- it loses the 5 N push the baseline survives. So with the
realisation path finally working, the mechanism is not neutral, it is
actively harmful during walking.

**Why it helps standing and hurts walking.** In `mpc_balance.py`'s standing
test the same integral realisation worked (CoP tracking error 0.0 mm, bound
inactive) because `dcm_err` is small and quiet at rest. During walking
`dcm_err` is inherently large and oscillating -- it swings every step by
construction, since the DCM tracks a moving reference. An integral loop
chasing it injects a step-synchronous disturbance into ankle_roll rather
than correcting anything. This is the same shape as Stage 2's clamp
finding: authority that is useful when small becomes a disturbance when the
signal driving it is large.

**Conclusion: CoP-repulsion is exhausted for this system.** Three
independent tests now: proportional law on a degraded baseline (nothing),
proportional law on the re-tuned baseline under unlimited torque (nothing),
and integral realisation with measured gain 1.01 at the 80:1 envelope
(actively worse). The third is the decisive one, because it is the only one
where the actuator could actually deliver what the law asked for. `K_IC`
stays at 0.0 and the wiring stays present-but-inert; there is now a
measured reason for that rather than an unexplained null.

**What this does NOT close.** Architecture E's capture-region footstep
placement remains untried -- it is a stepping mechanism, not an ankle one,
so none of the above bears on it. It is now the only untried item from the
original eight that the 80:1 envelope plausibly unblocks.

### 2026-09-10 — Capture-region footstep placement (architecture E) tried: a false positive caught by the timing check, and the eighth approach closes

The last untried mechanism of the eight in the architecture review, and the
one the 80:1 envelope plausibly unblocked (its stated blocker was that a
recovery step is a high-torque manoeuvre the robot could not afford).

**What was built.** Koolen/Pratt Part 2 Algorithm 1 in spirit: predict the
capture point at touchdown, treat a foot-sized box around it as the 1-step
capture region, and move the NOMINAL footstep only as far as needed to land
inside it. This is structurally different from the committed law, which
always chases the capture point and then clamps the delta -- capture-region
placement leaves the foot alone whenever the nominal step is already viable.
Experimental source patch, deliberately not committed.

**Nominal walking: genuinely, mildly better.** 6.31deg vs the committed
law's 8.76deg at n=8, with no regression. Same clean step range though --
both fall at n=18.

**The apparent push-recovery win, and why it was not one.** At the standard
`push_at=2.0s` the smallest region tested, r=(40,15)mm, survived **20 N at
7.35deg where the committed law falls at 90.31deg**, and cleaned up 15 N
(6.31 vs 26.57). That would have been the first genuine push-recovery
improvement this project has produced.

**It does not survive the timing check.** Re-run across seven push timings
(1.6-2.8 s):

| 15 N | 1.6 | 1.8 | 2.0 | 2.2 | 2.4 | 2.6 | 2.8 | survived |
|---|---|---|---|---|---|---|---|---|
| committed law | 77.5 | 6.5 | 26.6 | 80.6 | 90.7 | 9.7 | 7.0 | **4/7** |
| capture region | 76.7 | 16.1 | 6.3 | 78.0 | 28.1 | 73.2 | 77.5 | 3/7 |

| 20 N | 1.6 | 1.8 | 2.0 | 2.2 | 2.4 | 2.6 | 2.8 | survived |
|---|---|---|---|---|---|---|---|---|
| committed law | 77.1 | 76.5 | 90.3 | 76.9 | 87.6 | **12.0** | 77.7 | 1/7 |
| capture region | 76.5 | 78.7 | **7.4** | 75.9 | 76.5 | 78.2 | 76.3 | 1/7 |

At 20 N both survive exactly ONE timing out of seven -- the committed law at
2.6 s, capture-region at 2.0 s. They are the same result with the lucky cell
in a different place. At 15 N capture-region is WORSE (3/7 vs 4/7).

**This is the timing-chaos entry above cashing out as a methodology rule.**
That entry established a 10 ms shift can flip this system between clean
recovery and a 90deg fall. The direct consequence: **a single-timing push
result is not evidence.** Every push number in this file measured at one
`push_at` should be read as one sample from a chaotic distribution, not as
a property of the mechanism. Had this been reported on the 2.0 s column
alone it would have gone in as a win.

**Conclusion: architecture E is exhausted for push recovery, and all eight
approaches from the review have now been tried.** What survives is a modest
nominal-walking gain (8.76 -> 6.31 deg at n=8, same step range). Left
uncommitted: it is an experimental patch with no self-test, and the gain is
small and verified at one step count. Available if wanted, not shipped on
that evidence.

**Also recorded, correcting the previous entry.** The 80:1 envelope supports
**8 clean steps, not 45**. The earlier entry reported `--steps 8` walking at
8.76deg, which is true, but only the documented step count was checked; n=18
falls (77.27deg) under the real torque limit where unlimited torque reached
n=45. The drivetrain fix recovered the milestone as documented, with a much
shorter clean range than the pre-envelope figure.

### 2026-09-10 — The robot was never following its own plan: 215% lateral over-swing found, fixed, and the clean walking range doubles

Chasing the inverse-dynamics discrepancy from the entry above led to a check
this project had never run in any form: compare the robot's ACTUAL
centre-of-mass trajectory against the one its own planner asked for.

**The two axes behave completely differently.**

| axis | actual sway | planned sway | ratio |
|---|---|---|---|
| sagittal | 272.2 mm | 284.3 mm | **96%** |
| lateral | 85.2 mm | 56.3 mm | **151%** (215% on `sim_walk_recede`) |

Sagittal tracking is fine. Lateral over-swings by 1.5-2x, and this is during
CLEAN walking, not while falling -- measured on a 4-step run peaking at
7.53deg. Testing for phase lag ruled that out: a 140 ms shift barely helps
(22.9 -> 20.8 mm RMS), so it is an amplitude failure, not a timing one.

For scale: 85 mm peak-to-peak is +-42.6 mm, which exceeds BOTH the 40 mm
foot half-width and the 37.8 mm corrected capturability margin. **The CoM
routinely swings past the edge of the support polygon as normal operation.**

**Root cause: `K_DCM_RECEDE` was one gain applied to both axes.** Correct
for sagittal, badly wrong for lateral. Nobody had compared planned against
actual sway, so a 2x lateral tracking failure sat unnoticed underneath every
mechanism built on top of it.

**Fix: a separate lateral gain, scaled UP not down.** Over-swing here means
loose tracking, so tightening it reduces the error -- the opposite of the
intuition that over-swing means too much gain. Swept 1.0-3.0, non-monotonic
as always: 215%, 121%, 181%, 249%, 121%. `K_DCM_Y_SCALE = 1.5` committed
(`f63ad0d`).

**Result -- the clean walking range doubles.** At the real 80:1 envelope:

| n_steps | before | after |
|---|---|---|
| 8 | 8.76 | **6.27** |
| 10 | 31.02 | **6.27** |
| 12 | 77.27 | **6.27** |
| 14 | falls | **6.27** |
| 16 | falls | **7.11** |
| 18 | falls | 24.63 (marginal) |
| 20 | falls | 78.29 |

Eight to sixteen clean steps, flat 6.27deg throughout, on deterministic
no-push runs across seven step counts -- not a lucky cell.

**But it buys NO push margin, so my unifying theory is half wrong.** The
hypothesis was that over-swing consumed the stability margin and therefore
explained BOTH the walking-range limit AND the eleven failed push-recovery
mechanisms. Push survival across 7 timings at n=8 (where both configs walk):

| F_y | before | after |
|---|---|---|
| 5 N | 6/7 | 7/7 |
| 10 N | 4/7 | 5/7 |
| 15 N | 4/7 | **2/7** |
| 20 N | 1/7 | 1/7 |
| 30 N | 0/7 | 0/7 |

Net neutral -- slightly better at low force, worse at 15 N. **Lateral
over-swing explains the walking-range limit and NOT the push-recovery
failures. They are two separate problems**, which is a narrowing even though
it is less satisfying than one root cause.

**A methodology error of mine, recorded because this file tracks those too.**
The first version of that push comparison ran at n=12, where the pre-fix
config does not walk nominally -- comparing push survival between a config
that walks and one that does not measures nothing coherent. It produced a
plausible-looking table (5/7 at 5 N for a config that falls unpushed) that
would have been written up as a real comparison had the inconsistency not
been noticed. Same class as the capture-region false positive: a number that
looks like a result without measuring what it claims. **Rule: only compare
push survival at step counts where every config under test walks unpushed.**

**Related correction to earlier push data in this file.** Measured properly
-- multi-timing, at a step count where it actually walks -- the pre-fix
baseline survives 6/7 at 5 N and 4/7 at 10 N. Earlier entries recorded much
worse push performance, but those were taken at single timings and at step
counts near or beyond the walking limit. Push numbers elsewhere in this file
predating this entry should be treated as pessimistic and methodologically
weak, not as clean measurements of the mechanism under test.

### 2026-09-10 — The chaos was substantially a modelling artifact: the design specifies a compliant sole, the simulation modelled a rigid box

The user asked why this has been so hard when Honda and Waseda fielded
reliably-walking bipeds in the mid-90s, and whether classical control was the
wrong choice. Investigating that question found something more useful than an
answer to it.

**The gap.** `BOM.csv` has always specified a "Flat Sole Plate Assembly
(compliant sole pad) -- rigid plate + compliant pad for vertical shock
absorption". `generate_mjcf.py` emitted the foot as a rigid box on a rigid
plane with MuJoCo's default contact. **The simulation was modelling a robot
that was never designed.**

`sim_zmp_balance.py`'s own docstring had already recorded the consequence
without connecting it: MuJoCo box-plane contact "activates different corner
subsets frame to frame". That was diagnosed as a noisy ZMP *measurement* and
filtered with an EMA. It is noise in the PHYSICS -- discontinuous ground
reaction force tick to tick -- and every controller in this project has been
fighting it since.

**Contact compliance dominates disturbance behaviour.** Sweeping the sole
timeconst (10N lateral push, 12 timings, n=12; nominal walking alongside):

| solref | n=8 | n=16 | n=20 | 5N | 10N | 15N | 20N |
|---|---|---|---|---|---|---|---|
| 0.02 rigid | 6.27 | 7.11 | 78.29 | 8/12 | **0/12** | 0/12 | 0/12 |
| 0.03 | 6.42 | 6.85 | **7.88** | 11/12 | 0/12 | 1/12 | 0/12 |
| 0.04 | 6.54 | 6.54 | 15.68 | 12/12 | 2/12 | 0/12 | 0/12 |
| 0.05 | 6.69 | 6.69 | 20.99 | 12/12 | **9/12** | 2/12 | 1/12 |
| 0.06 | 6.79 | 16.05 | 90.95 | 12/12 | **11/12** | 4/12 | 0/12 |
| 0.07 | 6.67 | 91.39 | 91.39 | 12/12 | 3/12 | 2/12 | 0/12 |

**Eleven separate control mechanisms failed to move 10N push recovery at
all. A contact-model correction moves it from 0/12 to 9/12.**

The character changes too, not just the count. A coarser sweep showed rigid
contact producing alternating survive/fall between adjacent 10ms samples,
while compliance produces contiguous blocks -- survival becomes a smooth
function of where in the gait cycle the push lands, which is how a real robot
behaves.

**Value chosen deliberately NOT by score.** 0.05 preserves the nominal
walking range while recovering most of the push benefit; 0.06 scores better
on pushes and worse on walking. Picking on score would repeat exactly the
model-flattering this project has spent effort correcting. Recorded as a
PLACEHOLDER in `design/geometry.yaml` -- it is a PHYSICAL property of the
pad, currently unmeasured, and the sweep shows outcomes are highly sensitive
to it. Must be measured in P6.

**What this means for the rest of this file, stated plainly.** Every push
result recorded before this entry was measured against a contact model that
flips outcomes on a 10ms perturbation. Those verdicts are not necessarily
wrong, but they are **not trustworthy** -- the capture-region false positive
was the visible instance of this, and there may be invisible ones in the
other direction. **Treat all pre-2026-09-10 push-recovery conclusions as
provisional.** That specifically includes the "exhausted" verdicts on
CoP-repulsion and capture-region placement, both of which were scored
against rigid contact.

**And it partly answers the user's question.** The classical techniques were
not the wrong choice. Honda and Waseda were tuning control against real
physics; this project has been tuning against a model that was wrong in at
least four separate ways discovered this session -- unlimited torque,
ground-truth state, stale gains, and rigid contact. Each took a measurement
to find. The difficulty has been substantially about model fidelity rather
than control theory.

**Battery after the change:** load test, static pose, ZMP balance and
`sim_walk_recede --selftest` all pass; receding-horizon nominal walking flat
at 6.69deg across n=8/12/16, marginal at n=20. `sim_walk_lipm` and
`sim_walk_gait` unchanged (separate documented causes).

### 2026-09-10 — Provisional verdicts re-tested against compliant contact: both CONFIRMED, and one was misleading in the other direction

The sole-compliance correction made every prior push result provisional (see
`docs/P3_MODEL_FIDELITY.md`). The two mechanisms whose "exhausted" verdicts
mattered most were re-run against the corrected contact model.

**Note the baseline is now a much harder opponent.** Against rigid contact the
committed controller survived 0/12 timings at 10 N. Against compliant contact
it survives 12/12 at 5 N and 9/12 at 10 N. A mechanism now has to beat a
baseline that already handles most moderate pushes, rather than one that fails
all of them.

**Re-run 1 -- CoP-repulsion: verdict CONFIRMED.** `K_IC` swept both signs,
11 timings, n=12:

| K_IC | 5N | 10N | 15N | 20N |
|---|---|---|---|---|
| 0 (baseline) | 12/12 | 9/12 | 2/12 | 1/12 |
| -1 | 12/12 | 10/12 | 2/12 | 1/12 |
| -5 | 12/12 | 10/12 | 2/12 | 0/12 |
| -20 | 12/12 | 10/12 | 3/12 | 0/12 |
| +1 / +5 / +20 | 12/12 | 8-9/12 | 1-2/12 | 1/12 |

Best case is +1 cell of 12 at 10 N and -1 at 20 N. Noise. `K_IC` stays inert
at 0.0 -- the same conclusion as before, but now reached against a contact
model that does not flip outcomes on a 10 ms perturbation, so it is a real
verdict rather than a coin-flip average.

**Re-run 2 -- capture-region footstep placement: verdict CONFIRMED, and more
strongly than the original.**

| config | n=8 | n=16 | 5N | 10N | 15N | 20N |
|---|---|---|---|---|---|---|
| baseline | 6.69 | 6.69 | 12/12 | 9/12 | 2/12 | 1/12 |
| r=(40,15) mm | 8.12 | **90.85 falls** | 10/12 | 8/12 | 1/12 | 0/12 |
| r=(75,40) mm | 6.02 | **91.61 falls** | 0/12 | 0/12 | 0/12 | 0/12 |

Both region sizes now break nominal walking at n=16 and are equal or worse on
every push magnitude.

**And this reverses a finding from the rigid model.** Under rigid contact,
capture-region's one redeeming feature was mildly BETTER nominal walking
(6.31 vs 8.76 deg at n=8), recorded at the time as "does not regress and is
slightly better". Under compliant contact it destroys the walking range
entirely. That earlier reading was an artifact -- a reminder that rigid
contact misled in both directions, not only toward false positives.

**Status of the provisional list.** Both re-tested verdicts stand. The
push-magnitude ceilings quoted in earlier entries remain provisional and were
NOT re-measured mechanism by mechanism; what is now known is that the
compliant baseline itself is far stronger than any pre-2026-09-10 entry
records (12/12 at 5 N, 9/12 at 10 N), so those ceilings understate the
current controller rather than overstating it.

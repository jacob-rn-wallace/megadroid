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


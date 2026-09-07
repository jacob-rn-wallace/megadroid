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
- `tools/sim_walk_gait.py` — **P3 walking-gait milestone, partial: one
  step validated (well), not yet a sustained gait.** Floating-base, no
  weld. Reuses sim_zmp_balance.py's balanced pose and sagittal
  ankle-pitch ZMP loop, plus a second control mechanism the standing
  controller didn't need — lateral (hip_roll) weight transfer, since
  there's no ankle_roll to shift ZMP sideways. `python3
  tools/sim_walk_gait.py` passes: ~5° peak tilt, no fall, swing foot
  lands ~34mm forward (add `--render out.gif` for a visual — MuJoCo's
  offscreen renderer works in this environment; `imageio` + `ffmpeg`
  handle encoding). `--steps 2` reliably fails. Five real bugs were
  found and fixed getting the single step this clean (all documented in
  the script's module docstring, worth reading before touching gait
  code) — two sign-convention bugs (which stance side needs positive
  vs. negative hip_roll; hip_pitch's sign for "forward," since positive
  hip_pitch rotates the thigh backward in this axis convention), the
  swing leg inheriting the stance leg's hip_roll shift and landing
  ~30mm off from its intended footprint, the stance leg's forward-drive
  needing a smaller, separately-tuned magnitude than the swing leg's or
  the stance foot slips on the ground, and — counter to intuition — a
  *faster* swing (~0.3s) being markedly more stable than a slower one.
  One finding stands independent of further tuning: the pelvis nets
  slightly backward every step even though the foot itself lands
  forward (a real, reproduced-across-the-tuning-range recoil effect,
  not noise) — the pass criterion is deliberately based on foot
  placement, not pelvis translation, because of this.

  **Why multi-step reliably fails (root cause, not just "needs more
  tuning"):** after one step the two legs are no longer mirror-
  symmetric — their hip_pitch angles advanced in opposite directions —
  so the two feet end up at genuinely different (x, y) positions rather
  than the nominal ±hip_y mirror pair. The "rotate both hip_roll by the
  same angle" weight-shift trick relies on exactly that mirror symmetry
  to produce a clean pelvis translation; tested directly against the
  post-step-1 asymmetric configuration, it mostly fails to move the
  pelvis laterally at all and pitch grows instead. Fixing this needs
  real per-step inverse kinematics for the actual (asymmetric) foot
  placements each step leaves behind, not the symmetric-case shortcut
  reused every step — a bigger undertaking than gain-tuning, and the
  actual next task if continuing this work (see below).
- `simulation/mujoco/megadroid_mvs.xml` — full MuJoCo scene with dynamics,
  position actuators (kp=150), foot contact geometry, ground plane

**Where work stopped:**
Three P3 simulation milestones are done: fixed-base static load
validation, the floating-base ZMP ankle-pitch standing controller, and
one well-validated step of quasi-static walking. Sustained multi-step
walking is NOT done. The next task, if continuing the gait work, is
per-step inverse kinematics: given the actual (asymmetric) current foot
placements after a step, solve for the hip_roll/hip_pitch/knee/ankle
needed for the next weight shift and swing, rather than reusing the
single nominal-pose-derived shift angle and fixed gains for every step
regardless of how asymmetric the legs have become. Periodic re-
centering (restoring a fresh symmetric-enough relative leg configuration
between steps) is a lighter-weight alternative worth trying first if it
proves easier than full per-step IK.
Other next directions (none started, no decision made yet):
  - Extend the ZMP controller to reject external disturbances (a push),
    which will likely need a hip/torso strategy layered on top of the
    ankle strategy (see sim_zmp_balance.py's known limitation).
  - Move toward P4 physical prototyping — premature before multi-step
    walking is validated, since walking is likely to stress-test
    mechanical dimensions and motor torque budgets that standing alone
    doesn't touch.

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
| P3 walking gait validation (1 step) | `python3 tools/sim_walk_gait.py` |
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
  sim_walk_gait.py              P3 walking gait — 1 step validated, multi-step not yet stable
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

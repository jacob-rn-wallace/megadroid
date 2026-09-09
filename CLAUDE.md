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

## P3 Active Work

P3 simulation/gait work is documented in `tools/CLAUDE.md` — read it before
touching any `tools/sim_*.py` or `tools/generate_mjcf.py` file.

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
  sensors.yaml      Sensing strategy: joint position sensing, motor-sensing
                     exclusions, end-of-limb 6-DOF F/T sensor spec
  actuation.yaml    Motor and drivetrain parameters (stub — not yet populated)
  power.yaml        Power system parameters (stub — not yet populated)
  .meta.yaml        Schema/metadata for the design directory
```

`design/sensors.yaml` is the only place the F/T-sensor spec and joint-encoder
policy live — check it before concluding this project has no force/torque
sensing story. (It used to live undisclosed inside `geometry.yaml`, and the
rendered docs didn't even source it from YAML; both were fixed together.)

### Key Design Constants (do not change without an explicit design revision)

- **MVS DOF:** 13 actuated joints, all 13 deliberately kept actuated for the
  time being (2026-09-09 decision — see `tools/CLAUDE.md`). `ankle_roll` in
  particular was first actuated because simulation found the walking
  controllers have no lateral feedback and a passive spring-centered
  ankle_roll left the gait unstable; a session of lateral push-recovery
  investigation since then confirms actuated ankle/hip/torso roll axes are
  the physically correct authority for this (capturability theory, real
  hardware), even though the control software doesn't yet exploit it well.
  A passive spring-centered ankle_roll remains a possible future
  simplification, not a committed plan.
  - Per leg (×2): `hip_pitch`, `hip_roll`, `knee_pitch`, `ankle_pitch`, `ankle_roll`
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

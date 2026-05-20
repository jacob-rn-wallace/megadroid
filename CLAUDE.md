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
| `PHILOSOPHY.md` | Static — project philosophy | **No — static artifact** |
| `PROCESS.md` | Static — workflow definition | **No — static artifact** |
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
  actuation.yaml    Motor and drivetrain parameters
  power.yaml        Power system parameters
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

---

## Mandatory Workflow

### Before committing any change

1. **Edit only `design/*.yaml`** for any design change.
2. **Run rehydration** to regenerate derived docs:
   ```bash
   python3 tools/rehydrate_all.py
   ```
   This regenerates `SPEC.md`, `MECH.md`, and `README.md` from YAML + templates.
3. **Run validation** to catch consistency errors:
   ```bash
   python3 tools/validate_dof_consistency.py
   python3 tools/validate_no_geometry_literals.py
   python3 tools/validate_geometry.py
   ```
4. **Commit YAML changes first**, derived docs second — never in the same commit.

### Commit order (non-negotiable)

```
Commit 1: design/*.yaml changes  ← always first
Commit 2: SPEC.md MECH.md README.md  ← rehydrated derived docs
```

### Never do

- Edit `SPEC.md`, `MECH.md`, or `README.md` directly
- Add a numeric design value to a derived doc
- Commit derived docs ahead of the YAML that produced them

---

## Toolchain

```
tools/
  rehydrate_all.py              Run all rehydrators (primary entry point)
  rehydrate_spec.py             Regenerate SPEC.md
  rehydrate_mech.py             Regenerate MECH.md
  rehydrate_readme_structure.py Regenerate README.md structure
  validate_all.py               Run all validators
  validate_dof_consistency.py   Check DOF counts are consistent across YAML
  validate_no_geometry_literals.py  Catch hardcoded numbers in derived docs
  validate_geometry.py          Geometry-specific consistency checks
  generate_urdf.py              Generate URDF from YAML
  verify_urdf_dimensions.py     Validate URDF against YAML
  visualize_urdf.py             matplotlib-based 3D visualizer (macOS-compatible)
  analyze_workspace.py          Workspace sampling via forward kinematics
  check_joints.py               Joint-level sanity checks

templates/
  SPEC.md.j2                    Jinja2 template for SPEC.md
  MECH.md.j2                    Jinja2 template for MECH.md
```

Dependencies: `pyyaml`, `jinja2`. Install with:
```bash
pip install pyyaml jinja2
```

---

## Repository Structure

```
megadroid/
  design/           Authoritative YAML (single source of truth)
  templates/        Jinja2 templates for derived docs
  tools/            Rehydration, validation, generation, visualization scripts
  simulation/
    urdf/           Generated URDF files (e.g., megadroid_mvs.urdf)
  docs/             Static milestone reports (e.g., P2_VALIDATION.md)
  firmware/         (placeholder — not yet populated)
  software/         (placeholder — not yet populated)
  SPEC.md           Derived system specification
  MECH.md           Derived mechanical description
  README.md         Derived repository overview
  BOM.csv           Derived bill of materials
  PHILOSOPHY.md     Static project philosophy
  PROCESS.md        Static workflow definition
  REHYDRATE.md      Static rehydration process description
  CLAUDE.md         This file
```

---

## CI

Two GitHub Actions workflows run on push/PR:

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
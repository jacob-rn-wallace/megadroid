<!--
name: Megadroid Development Process  
type: Process Specification  
description: Defines the formal yet lightweight development workflow used to evolve Megadroid from concept through Minimum Viable System (MVS) completion and beyond, ensuring traceability, consistency, and controlled iteration.  
authority: informative (governs workflow; does not override `SPEC.md`)
-->
# Megadroid — Development Process (PROCESS)

**Status:** Informative  
**Scope:** Project-wide development workflow for Megadroid, from concept through MVS completion and subsequent expansions  
**Last updated:** 2026-01-29

This document defines the formal yet lightweight development workflow used to evolve Megadroid from concept through Minimum Viable System (MVS) completion and beyond, ensuring traceability, consistency, and controlled iteration.

---

## 1. Purpose

This document defines the **official development process** for the Megadroid project.

Its goals are to:
- Provide a clear, repeatable workflow for design evolution
- Maintain traceability between requirements, design intent, implementation, and verification
- Prevent uncontrolled design drift
- Enable steady progress toward a buildable Minimum Viable System (MVS)

This process is intentionally **lightweight** and optimized for a **single-builder, cost-constrained, hardware-focused project**.

---

## 2. Development Model Overview

Megadroid is developed using a **dual-track development model**:

- A **Product Track**, which governs the design, validation, and realization of the robot itself.
- An **Infrastructure Track**, which governs documentation, tooling, automation, validation, and project governance.

The two tracks proceed **in parallel but independently**.

### 2.1 Product Track

The Product Track follows a **V-model-inspired lifecycle**, emphasizing:
- Explicit requirements
- Traceable design decisions
- Verification before physical construction
- Incremental validation (including simulation-first validation)

This track ultimately culminates in the completion of the **Minimum Viable System (MVS)** and later full-system extensions.

### 2.2 Infrastructure Track

The Infrastructure Track is **not V-model–constrained**.

Its purpose is to:
- Improve correctness, traceability, and maintainability
- Reduce human error through automation
- Improve project hygiene and reproducibility
- Support learning and skill development in software and systems engineering

Infrastructure work may occur **at any time**, including during product design, simulation, or physical build phases.

Infrastructure improvements are expected to stabilize over time but are never considered “complete.”

---

## 3. Process Philosophy

Megadroid follows an **artifact-driven, iterative V-model**.

Key principles:
- Design intent is captured **once**, in authoritative form
- Derived documents are **generated**, not hand-maintained
- Progress occurs in **stable plateaus**, not continuous churn
- Git tags represent **validated design states**, not arbitrary milestones

The process favors **clarity and enforcement over formality**.

---

## 4. Authoritative Artifacts

The following artifacts define the system and its constraints:

| Layer | Artifact | Authority |
|------|---------|----------|
| System intent | `SPEC.md` | **Authoritative** |
| Design parameters | `design/*.yaml` | **Authoritative (source of truth)** |
| Mechanical realization | `MECH.md` | Derived |
| Cost tracking | `BOM.csv` | Derived |
| Repository structure | `README.md` | Derived |

**Rule:**  
No implementation decision is authoritative unless it is expressed in `design/*.yaml` **where applicable** and reflected in `SPEC.md`.

---

## 5. The Iterative V-Model

### 5.1 Left Side — Definition

Design proceeds top-down:

1. **Requirements & constraints**
   - Captured in `SPEC.md`
2. **System architecture**
   - Encoded in machine-readable YAML (`design/*.yaml`)
3. **Mechanical realization**
   - Described in `MECH.md`
4. **Parts & cost**
   - Enumerated in `BOM.csv`

All downstream artifacts must conform to upstream constraints.

### 5.2 Bottom — Implementation Plateaus

Work proceeds in **discrete plateaus**, each producing a concrete outcome:

- Geometry freeze
- Mechanical layout freeze
- Electrical integration
- Physical assembly
- Locomotion demonstration

Each plateau ends in a **git tag**, not ongoing iteration.

### 5.3 Right Side — Verification

Verification is intentionally minimal, explicit, and constraint-focused:

| Level | Verification Mechanism |
|-----|------------------------|
| YAML | Python validators |
| SPEC / MECH | Consistency checks |
| BOM | Cost ceiling enforcement |
| Hardware | Simple physical tests |

Verification answers one question:
> “Does this implementation still satisfy the constraints above it?”

---

## 6. Rehydration and Automation

Megadroid uses a **rehydration workflow** to prevent divergence between documents.

- `design/*.yaml` files are the **single source of truth**
- Markdown documents are **derived views**
- Python scripts in `tools/` enforce:
  - DOF consistency
  - Geometry indirection
  - Authority boundaries
  - Cost limits

Manual edits to derived documents are permitted **only** where explicitly allowed.

---

## 7. Change Control Integration

This process works in tandem with **Section 15 (Change Control)** of `SPEC.md`.

- Changes that affect system behavior or constraints **must update `SPEC.md`**
- Implementation-only changes belong in `MECH.md` or `BOM.csv`
- All authoritative changes require:
  1. Update to YAML
  2. Rehydration
  3. Validation
  4. Commit with rationale

---

 ## 8. Versioning, Phases, and Roadmap

Megadroid uses **semantic versioning adapted for staged system development**, with additional structure to support student-scale progress and infrastructure evolution.

### 8.1 Version Numbering Scheme

Versions follow the form:

`vMAJOR.MINOR.PATCH`

Where:

- **MAJOR** represents a complete, validated system generation
- **MINOR** represents a coherent development stage
- **PATCH** represents refinements, fixes, or tooling improvements within a stage

Version numbers are **not used to indicate time or effort**, only **design and validation state**.

### 8.2 Infrastructure Track Versioning (`v0.x.y`)

Versions **below `v1.0`** are explicitly pre-product and may include incomplete designs.

The Infrastructure Track primarily occupies:

- `v0.1.x` → Initial repository and document structure
- `v0.2.x` → Authority model and change control formalization
- `v0.3.x` → Rehydration, templating, validation tooling, and metadata consistency

The current repository state corresponds to:

> **`v0.3.x` — Infrastructure Stabilization Phase**

Tags in this range represent **documentation and tooling plateaus**, not robot capability.

Infrastructure-only changes must not invalidate any previously tagged Product Track milestone.

### 8.3 Product Track Roadmap (Student-Scale Milestones)

The Product Track advances through **student-scale, achievable milestones**, each of which may span weeks rather than months.

These milestones are designed so that:
- Each stage produces a meaningful artifact
- Validation precedes physical construction
- Large conceptual jumps are avoided

#### **Stage P1 — Authoritative Design Definition**
**Target versions:** `v0.4.x`

- Freeze authoritative YAML schemas:
  - joints
  - geometry
  - kinematic conventions
- Ensure SPEC and MECH are fully derived and contradiction-free
- Establish joint limits, reference frames, and nominal poses
- No simulation or hardware yet

#### **Stage P2 — Kinematic & Structural Validation**
**Target versions:** `v0.5.x`

- Build kinematic models from authoritative data
- Validate joint ranges and singularity behavior
- Confirm anthropometrics and stance feasibility
- No dynamics, no control, no motors

#### **Stage P3 — Simulation-First MVS Validation**
**Target versions:** `v0.6.x`

- Implement robot in simulation (e.g., Gazebo, Isaac SIM, MuJoCo, or equivalent)
- Validate:
  - joint layout
  - balance feasibility
  - quasi-static walking concepts
- Simulation is the primary test environment
- No physical hardware assumed

#### **Stage P4 — Actuation & Control Design**
**Target versions:** `v0.7.x`

- Define actuator performance envelopes
- Validate gear ratios against simulated loads
- Implement position-only control in simulation
- Validate stair interaction virtually

#### **Stage P5 — Mechanical Detail & Build Readiness**
**Target versions:** `v0.8.x`

- Finalize:
  - shaft sizes
  - bearing selections
  - belt layouts
  - fastener standards
- Prepare fabrication-ready designs
- No assumption that hardware has been built yet

#### **Stage P6 — Physical MVS Construction**
**Target versions:** `v0.9.x`

- Begin physical build of the MVS
- Bring up electronics and motors
- Validate hardware matches design assumptions
- Expect iteration and fixes

### 8.4 `v1.0` Definition

**`v1.0` represents a completed, validated MVS**, equivalent in scope to what might previously have been labeled `v5.0`.

A `v1.0` tag requires:
- A physically constructed MVS
- Demonstrated standing and walking on flat ground
- Alignment between:
  - SPEC
  - MECH
  - BOM
  - simulation results
  - physical behavior

No infrastructure-only change may advance the MAJOR version.

---

## 9. Governance and Discipline

Megadroid prioritizes **process correctness over speed**.

Key principles:
- Decisions are only authoritative when written into `SPEC.md`
- Automation supports, but does not replace, human judgment
- Infrastructure improvements may occur opportunistically
- Product milestones must be earned through validation

This project explicitly values **learning, correctness, and traceability** over rapid completion.

---

## 10. Scope Beyond MVS

The same process applies beyond the MVS:
- Additional DOF
- Arms and manipulators
- Active ankles
- Force/impedance control

Each expansion is treated as a **new iteration** layered on top of the validated MVS baseline.

---

## 11. Authority

This document defines **how** Megadroid is developed.

It does **not** override technical constraints defined in `SPEC.md`.

If there is a conflict:
**`SPEC.md` always wins.**
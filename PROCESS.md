<!--
name: Megadroid Development Process  
type: Process Specification  
description: Defines the formal yet lightweight development workflow used to evolve Megadroid from concept through Minimum Viable System (MVS) completion and beyond, ensuring traceability, consistency, and controlled iteration.  
authority: informative (governs workflow; does not override `SPEC.md`)
-->
# Megadroid — Development Process (PROCESS)

**Status:** Informative  
**Scope:** Project-wide development workflow for Megadroid, from concept through MVS completion and subsequent expansions  
**Last updated:** 2026-01-24

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

## 2. Process Philosophy

Megadroid follows an **artifact-driven, iterative V-model**.

Key principles:
- Design intent is captured **once**, in authoritative form
- Derived documents are **generated**, not hand-maintained
- Progress occurs in **stable plateaus**, not continuous churn
- Git tags represent **validated design states**, not arbitrary milestones

The process favors **clarity and enforcement over formality**.

---

## 3. Authoritative Artifacts

The following artifacts define the system and its constraints:

| Layer | Artifact | Authority |
|------|---------|----------|
| System intent | `SPEC.md` | **Authoritative** |
| Design parameters | `design/*.yaml` | **Authoritative (source of truth)** |
| Mechanical realization | `MECH.md` | Derived |
| Cost tracking | `BOM.csv` | Derived |
| Repository structure | `README.md` | Derived |

**Rule:**  
No implementation decision is authoritative unless it is expressed in `design/*.yaml` and reflected in `SPEC.md`.

---

## 4. The Iterative V-Model

### 4.1 Left Side — Definition

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

---

### 4.2 Bottom — Implementation Plateaus

Work proceeds in **discrete plateaus**, each producing a concrete outcome:

- Geometry freeze
- Mechanical layout freeze
- Electrical integration
- Physical assembly
- Locomotion demonstration

Each plateau ends in a **git tag**, not ongoing iteration.

---

### 4.3 Right Side — Verification

Verification is intentionally minimal but explicit:

| Level | Verification Mechanism |
|-----|------------------------|
| YAML | Python validators |
| SPEC / MECH | Consistency checks |
| BOM | Cost ceiling enforcement |
| Hardware | Simple physical tests |

Verification answers one question:
> “Does this implementation still satisfy the constraints above it?”

---

## 5. Rehydration and Automation

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

## 6. Change Control Integration

This process works in tandem with **Section 15 (Change Control)** of `SPEC.md`.

- Changes that affect system behavior or constraints **must update `SPEC.md`**
- Implementation-only changes belong in `MECH.md` or `BOM.csv`
- All authoritative changes require:
  1. Update to YAML
  2. Rehydration
  3. Validation
  4. Commit with rationale

---

## 7. Versioning and Tags

Git tags represent **stable design plateaus**, not incremental progress.

A tag may be created **only when**:
- `SPEC.md`, `MECH.md`, and `BOM.csv` are internally consistent
- All validation scripts pass
- No known contradictions exist

### 7.1 Tag Semantics

| Tag | Meaning |
|----|--------|
| `v0.1-mvs` | MVS scope defined |
| `v0.2-authority` | Authority and traceability locked |
| `v0.3-rehydration` | Rehydration and enforcement tooling |
| `v0.4-geometry` | Geometry frozen |
| `v0.5-mech-freeze` | Mechanical design frozen |
| `v0.6-elec-freeze` | Electrical design frozen |
| `v0.7-assembly` | Physical assembly complete |
| `v0.8-walk` | Walking demonstrated |
| `v1.0-mvs` | MVS complete and validated |

---

## 8. Scope Beyond MVS

The same process applies beyond the MVS:
- Additional DOF
- Arms and manipulators
- Active ankles
- Force/impedance control

Each expansion is treated as a **new iteration** layered on top of the validated MVS baseline.

---

## 9. Authority

This document defines **how** Megadroid is developed.

It does **not** override technical constraints defined in `SPEC.md`.

If there is a conflict:
**`SPEC.md` always wins.**
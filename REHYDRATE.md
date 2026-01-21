# Megadroid Design Rehydration Process

This document defines how authoritative design data is propagated through the
Megadroid repository.

---

## 1. Authority Model

Megadroid uses a **single-source-of-truth** model.

### Authoritative Sources
- All concrete design parameters live in:
  - `design/*.yaml`

These files are the only location where numeric values, constraints, and
architectural decisions may be defined.

### Derived Artifacts
The following files are **derived views** of the authoritative data:
- `SPEC.md`
- `MECH.md`
- Generated tables, figures, and summaries

They must **never** introduce new design values.

---

## 2. Rehydration

“Rehydration” is the process of regenerating human-readable documentation
from authoritative design data.

Rehydration is performed by tools in:
- `tools/`

Examples:
- `tools/generate_spec_dof.py`
- future geometry, power, and materials generators

---

## 3. Automation Rules

- Scripts may generate text blocks or full documents
- Auto-generated sections must be clearly marked
- Manual edits inside auto-generated regions are forbidden
- If a value changes, it must change **only** in `design/*.yaml`

---

## 4. Version Control Policy

- YAML changes precede markdown changes
- Markdown commits must reference the generating tool
- Rehydration scripts are versioned alongside outputs

---

## 5. Intent

This process exists to:
- Prevent divergence between documents
- Enable machine validation
- Preserve long-term correctness as the project grows
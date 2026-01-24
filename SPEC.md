<!--
name: SPEC.md
type: specification
description: System-level specification defining design intent, architectural constraints, and locked assumptions for the Megadroid Minimum Viable System.
authority: authoritative
-->
---
# Megadroid — System Specification (SPEC)

**Status:** Authoritative  
**Scope:** Minimum Viable System (MVS)  
**Last updated:** 2026-01-24

> **Note:** This document is a derived view of authoritative design data
> defined in `design/*.yaml`. See `REHYDRATE.md` for the rehydration process.

---

## 1. Purpose

This document defines the **authoritative system-level specification** for the Megadroid humanoid robot project.

It captures **design intent, architectural constraints, and locked assumptions** for the Minimum Viable System (MVS). All implementation details must conform to this specification unless explicitly revised here.

---

## 2. Project Goals

### 2.1 Ultimate Goal
To develop a **fully featured bipedal humanoid robot** with actuated legs, arms, and torso that can be built and iterated on by non-institutional builders at a fraction of the cost of traditional research humanoids.

### 2.2 Minimum Viable System (MVS) Goals
The MVS is intended to:
- Walk upright on flat ground
- Ascend and descend residential stairs using conservative, quasi-static gaits
- Maintain whole-body balance using limited actuation and software constraints
- Serve as a mechanically and electrically conservative foundation for future expansion

---

## 3. System Overview

### 3.1 Actuated Degrees of Freedom (MVS)

The following actuated degrees of freedom are derived directly from the
authoritative joint model defined in `design/joints.yaml`.

<!-- BEGIN AUTO-GENERATED: DOF -->
**Legs (×2):**
- Hip Pitch (pitch)
- Hip Roll (roll)
- Knee Pitch (pitch)

**Torso:**
- Torso Pitch (pitch)
- Torso Roll (roll)
- Torso Yaw (yaw)

**Total actuated DOF:** **9**
<!-- END AUTO-GENERATED: DOF -->

### 3.2 Modular / Swappable Lower-Leg Assemblies (MVS vs Full System)

The robot supports modular lower-leg assemblies. The MVS uses a simplified calf/foot module that omits active ankle joints to reduce cost and complexity. A future module may add actuated ankles and alternate feet without requiring redesign of the pelvis, thigh, knee, or control stack.

### 3.3 Non-Actuated / Out-of-Scope (MVS)
- Ankles: not actuated in the MVS; implemented via a simplified calf/foot module (no ankle joints).
- Arms: not installed in the MVS.

---

## 4. Anthropometrics

- Thigh length (hip → knee): defined in `design/geometry.yaml`
- Shin length (knee → ankle): defined in `design/geometry.yaml`
- Ankle-to-sole offset: defined in `design/geometry.yaml`

Overall body proportions are humanoid and sized to comfortably navigate residential environments (e.g., standard doorways, stairs).

---

## 5. Feet and Ground Contact

- Foot type (MVS module): Deformable ball foot (MIT Cheetah Mini–inspired)
- Function:
  - Passive compliance
  - Shock absorption
  - Tolerance of foot placement errors

### 5.1 End-of-Limb Sensing
- Each foot includes a **6-DOF force/torque sensor**
- Sensor form factor: defined in `design/geometry.yaml`
- Sensor geometry is treated as a hard mechanical constraint
- F/T sensors are **excluded from MVS cost accounting**

---

## 6. Actuation

### 6.1 Motors
- All actuators use **775 brushed DC motors**
- Nominal motor voltage: **24 V**
- Motors mounted as proximally as possible to minimize distal inertia
- Total motor count (MVS): **9**

### 6.2 Gear Reduction
- Standardized gearbox classes reused across joints
- No joint uses a bespoke, one-off gearbox design
- Exact ratios are implementation details documented in `MECH.md`
- All belt-driven transmission stages use HTD 5M timing belts with standardized width
  as defined in `design/geometry.yaml`.
- Belt pitch and width are standardized across all joints to minimize part count,
  simplify sourcing, and improve serviceability.
---

## 7. Sensors

### 7.1 Joint Position Sensing
- **Joint-mounted absolute encoders only**
- Encoders measure **true joint output angle**
- No motor-mounted encoders are permitted

### 7.2 Motor Sensing
- No motor current sensing
- No motor-side velocity sensing

---

## 8. Control Architecture

### 8.1 Control Mode
- **Position-only control**
- No torque control
- No impedance control

### 8.2 Software Constraints
Stability and hardware protection are enforced through:
- Joint velocity limits
- Joint acceleration limits
- Gait timing constraints
- Brownout-aware behavior

---

## 9. Electrical Power System

### 9.1 Battery (Excluded from Cost Target)
- Model: **Toro 88675**
- Nominal voltage: **60 V**
- Capacity: **7.5 Ah**
- Energy: **405 Wh**

### 9.2 Power Distribution
- Single central DC/DC converter
- Conversion: **60 V → 24 V**
- Continuous current capability: ≥ **50 A**
- 24 V bus distributed to all motor drivers

### 9.3 Protection Philosophy
- Hardware current limiting at motor drivers
- Single main fuse on 60 V rail
- No per-motor DC/DC converters

---

## 10. Control Hardware Stack

### 10.1 High-Level Control
- **Raspberry Pi (Linux SBC)**
- Responsibilities:
  - Gait state machine
  - Trajectory generation
  - Supervisory control
  - Logging and debugging

### 10.2 Real-Time Control
- **3 × Waveshare RP2350-CAN boards**
  - Left leg controller
  - Right leg controller
  - Torso controller

### 10.3 Communications
- CAN bus
- Nominal bitrate: **500 kbps**

---

## 11. Cost Constraints

- **Target total system cost:** < **$1000 USD**
- Explicit exclusions:
  - Battery
  - End-of-limb force/torque sensors
- Cost tracking is maintained in `BOM.csv`

---

## 12. Explicit Exclusions (MVS)

The following are intentionally excluded from the MVS:
- Active ankle actuation (MVS uses a simplified calf/foot module without ankle joints)
- Arms or manipulators
- Torque or impedance control
- Motor current telemetry
- High-bandwidth force feedback loops
- Aesthetic-driven enclosure geometry

---

## 13. Design Philosophy

Megadroid prioritizes:
- Mechanical conservatism
- Electrical simplicity
- Software-enforced safety and stability
- Clear load paths and inspectable structures
- Incremental extensibility without redesign

---

## 15. Change Control

### 15.1 Purpose

This section defines the **formal process by which changes to the Megadroid system specification are proposed, reviewed, and adopted**.

The intent is to:
- Prevent uncontrolled design drift
- Maintain consistency across `SPEC.md`, `MECH.md`, and `BOM.csv`
- Ensure that all changes are intentional, documented, and traceable

This process is intentionally lightweight and optimized for a small, fast-moving project.

### 15.2 Change Authority

- **`SPEC.md` is the highest authority** for the Megadroid MVS.
- Any change that alters:
    - system capabilities,
    - degrees of freedom,
    - architectural assumptions,
    - control philosophy,
    - or cost constraints
      **must be reflected in `SPEC.md`.**

`MECH.md` and `BOM.csv` **must not contradict `SPEC.md`**.

### 15.3 Changes Requiring a SPEC Revision

A modification **requires a `SPEC.md` update** if it affects any of the following:
- Actuated or passive degrees of freedom
- Joint limits or kinematic conventions
- Control mode (e.g., position vs torque)
- Power architecture or voltage levels
- Sensor strategy (e.g., encoder placement, sensing modality)
- Modularity assumptions (e.g., swappable lower-leg modules)
- Explicit inclusions or exclusions in the MVS
- Cost target or cost exclusions

Pure implementation details that do **not** affect these constraints belong in `MECH.md` or `BOM.csv` only.

### 15.4 Change Workflow

All design changes follow this sequence:

1. Proposal
   - Identify the motivation for change (e.g., feasibility, cost, packaging, precedent)
   - Articulate the proposed modification clearly
2. Evaluation
   - Assess impact on:
       - mechanical design
       - control strategy
       - cost target
       - future extensibility
   - Compare against relevant prior art where applicable
3. Specification Update
	- Modify `SPEC.md` to reflect the new or revised constraint.
    - Changes must be explicit and unambiguous.
4. Propagation
	- Update `MECH.md` and/or `BOM.csv` as required to conform to the new SPEC.
    - No document may remain in conflict with `SPEC.md`.
5. Commit
	- Commit changes to the repository with a clear message describing:
        - what changed
        - why it changed
        - what assumptions were updated or removed
   
### 15.5 Locked vs Revisable Decisions

Design decisions fall into two categories:

**Locked decisions**
- Explicitly stated as "locked" in `SPEC.md` or `MECH.md`
- Not revised without strong justification
- Changes require a SPEC revision and a commit explaining the rationale

**Revisable decisions**
- Identified as implementation details
- May evolve without changing `SPEC.md`
- Must remain compliant with all locked constraints

When in doubt, treat a decision as **locked**.

### 15.6 Versioning and Tags

- Git tags represent **stable design plateaus**, not incremental progress.
- A tag should only be created when:
    - `SPEC.md`, `MECH.md`, and `BOM.csv` and internally consistent
    - No known contradictions exist between documents

Tags serve as historical reference points and rollback anchors.

### 15.7 Conversational and External Inputs

Design discussions, simulations, analyses, and external references (including LLM-assisted reasoning) are **non-authoritative inputs**.

A design decision becomes authoritative **only when it is written into `SPEC.md` and committed to the repository**.

### 15.8 Guiding Principle

> If a decision matters, it must be written down.
> If it is written down, it must be respected.

This change control process exists to ensure that Megadroid evolves deliberately, not accidentally.

---

## 16. Authority

This document is **authoritative**.

All mechanical, electrical, and software implementations must conform to this specification unless a revision is made here.

Detailed mechanical implementation is defined in `MECH.md`.

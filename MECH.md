<!--
name: MECH.md
type: mechanical-design
description: Mechanical implementation details for the Megadroid MVS, including joint layout, structural concepts, actuator placement, and load paths.
authority: authoritative
-->
# Megadroid — Mechanical Design (MECH)

**Status:** Authoritative (MVS)  
**Scope:** Mechanical implementation details for the Minimum Viable System  
**Last updated:** 2026-01-26

> **Note:** This document is a derived view of authoritative design data
> defined in `design/*.yaml`. See `REHYDRATE.md` for the rehydration process.

---

## 1. Purpose

This document defines the **authoritative mechanical design** of the Megadroid humanoid robot for the Minimum Viable System (MVS).

It specifies _how_ the system described in `SPEC.md` is physically realized, including joint layout, structural concepts, actuator placement, and load paths. Any mechanical implementation must conform to this document unless explicitly revised here.

---

## 2. Overall Mechanical Architecture

### 2.1 Structural Philosophy

Megadroid adopts a **mechanically conservative, inspection-friendly architecture** based on:
- Explicit load paths
- Double-shear joints wherever possible
- Proximal motor placement
- Reuse of structural and actuation patterns

The robot is organized around three primary structural modules:
- **Torso module** (houses power electronics and torso actuators)
- **Left leg module**
- **Right leg module**

Arms and active ankles are excluded from the MVS.

---

## 3. Leg Architecture (MVS)

### 3.1 Degrees of Freedom per Leg

Each leg implements **three actuated DOF**:
- Hip pitch
- Hip roll
- Knee pitch

The MVS uses a simplified lower-leg + foot module that omits actuated ankles. A future swappable module may add actuated ankles without redesigning the pelvis, thigh, or knee.

---

## 4. Leg Structural Construction

### 4.1 Twin-Rail / Box-Bulkhead Construction

Each leg segment (thigh and shank) is constructed as a **stiff boxed structure** using **twin longitudinal side rails** connected by **transverse bulkheads**, rather than a single tube or cosmetic shell.

- Rails are flat plates (e.g., aluminum or steel) arranged in parallel
- Bulkheads act as:
  - Structural diaphragms
  - Shaft and bearing mounting points
  - Load-transfer elements between rails
- Inner-face spacing between rails: defined in `design/geometry.yaml` (locked)

This architecture provides:
- High bending stiffness
- Good torsional rigidity when skinned
- Tolerance to fabrication variability
- Clear, inspectable load paths

Where beneficial, thin cover plates or printed skins may be added to form a closed box section and improve torsional stiffness.

---

### 4.2 Thigh Segment

- Length (hip → knee): defined in `design/geometry.yaml`
- Contains:
  - Hip pitch output shaft
  - Hip roll output shaft
  - Proximal portion of knee actuator (if applicable)

Motors are mounted as proximally as possible to reduce distal inertia.

---

### 4.3 Shank Segment

- Length (knee → ankle): defined in `design/geometry.yaml`
- Contains:
  - Knee pitch output shaft
  - Passive ankle structure
- Lower-leg module terminates in a foot interface for the F/T sensor + ball foot.
- MVS lower-leg module omits actuated ankle joints; ankle actuation is reserved for post-MVS modular upgrades.

---

## 5. Joint Design

### 5.1 General Joint Requirements

All joints must:
- Use **joint-mounted absolute encoders**
- Measure true output angle
- Be serviceable and inspectable
- Carry primary loads in double shear

Motor-mounted encoders are explicitly disallowed.

---

### 5.2 Hip Joint Assembly

The hip consists of **two orthogonal rotational axes**:

- **Hip roll** (lateral balance)
- **Hip pitch** (sagittal plane motion)

Key characteristics:
- Motors mounted proximally (pelvis or upper thigh)
- Gearboxes coupled directly to joint output shafts
- Bearings sized for combined radial and moment loads

Hip yaw is structurally present but **locked** in the MVS.

Hip joint output shafts use standardized shafting as defined in `design/geometry.yaml` (locked) to standardize bearings, hubs, and pulley bores.

---

### 5.3 Knee Joint Assembly

- Single-axis revolute joint (pitch)
- Actuator mounted proximally in the thigh
- Torque transmitted across the joint via a rigid shaft
- Bearings sized primarily for bending loads from stance phase

---

### 5.4 Lower-Leg Module and Foot Interface (MVS)

The MVS uses a simplified lower-leg module that does not include actuated ankles. The module provides a rigid, serviceable interface for mounting the end-of-limb F/T sensor and the deformable ball foot.

A future lower-leg module may add actuated ankle pitch/roll and alternate feet while preserving the same knee-to-lower-leg mechanical interface and joint encoder strategy.

---

## 6. End-of-Limb Force/Torque Sensor Integration

- One **6-DOF force/torque sensor per foot**
- Sensor form factor: defined in `design/geometry.yaml`
- Mounted between ankle structure and foot
- Structural design must:
  - Avoid imposing bending moments on the sensor body
  - Transfer loads axially through the sensor

F/T sensors are mechanically required but **excluded from cost accounting**.

---

## 7. Actuator Integration

### 7.1 Motors

- All actuators use **775 brushed DC motors**
- Motors mounted proximally where feasible
- Motors mechanically isolated from bending loads

---

### 7.2 Gearboxes

- Modular gearbox designs reused across joints
- Only a small number of gearbox ratios permitted
- No joint-specific one-off gearbox designs

Exact gearbox ratios and part selections are documented in `BOM.csv`.

---

### 7.3 Belt Reduction Standard (Locked)

All belt reduction stages in Megadroid use HTD 5M timing belts with standardized width as defined in `design/geometry.yaml`.

---

## 8. Torso Mechanical Design

### 8.1 Degrees of Freedom

The torso implements **three actuated DOF**:
- Pitch
- Roll
- Yaw

---

### 8.2 Structural Role

The torso structure:
- Serves as the primary load path between legs
- Houses power electronics and controllers
- Provides mounting for torso actuators

The torso must maintain stiffness sufficient to prevent coupling between leg motions through structural flex.

---

## 9. Materials and Fabrication

### 9.1 Preferred Materials

- Aluminum plate for primary structure
- Steel shafts and fasteners
- Polymer or printed parts only where loads are low

Pultruded carbon tubes are explicitly excluded from the MVS.

---

### 9.2 Fabrication Philosophy

Designs should favor:
- Flat plate machining or waterjet
- Off-the-shelf bearings and fasteners
- Minimal custom machining
- Tolerance-insensitive assemblies

---

## 10. Mechanical Constraints

- Must fit through standard residential doorways
- Must tolerate quasi-static stair climbing loads
- Must support conservative gait speeds

---

## 11. Authority

This document is **authoritative** for the mechanical design of the Megadroid MVS.

Any mechanical change that violates this document must be accompanied by an explicit revision here and corresponding updates to `SPEC.md`.

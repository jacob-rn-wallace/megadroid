<!--
name: README.md
type: documentation
description: Top-level project overview for Megadroid, describing the project’s goals, scope, repository structure, and how authoritative design documents relate to one another.
authority: derived (rehydrated from repository structure and design metadata)
-->
# Megadroid

**Megadroid** is an open-hardware humanoid robot project aimed at developing a **Hubo/ASIMO–class bipedal platform** that can be built, studied, and iterated on by advanced hobbyists and independent researchers.

The project emphasizes **cost-aware engineering, modular design, and buildability**, with the long-term goal of converging toward a **fully featured humanoid robot** with actuated legs, arms, and torso at a fraction of the cost of traditional research platforms.

This README intentionally avoids detailed architectural or implementation specifics. All concrete design decisions are documented in the authoritative specification and mechanical documents.

---

## Project Vision

The ultimate goal of Megadroid is to create a **complete, general-purpose humanoid robotics platform** that:

- Walks upright on two legs
- Manipulates the environment with articulated arms
- Maintains stable whole-body posture through an actuated torso
- Can be constructed and modified by individuals or small teams without institutional resources

Rather than pursuing state-of-the-art performance, Megadroid prioritizes **accessibility, clarity, and incremental progress**, enabling complex capability to emerge through careful engineering rather than specialized components.

---

## Design Philosophy

Megadroid is guided by the following high-level principles:

- **Cost minimization as a first-order constraint**
- **Modularity and reuse** of mechanical and electrical building blocks
- **Explicit, inspectable load paths** and mechanically conservative structures
- **Software-enforced constraints** instead of sensor-heavy or exotic hardware
- **Incremental extensibility**, allowing future capability to be layered on without redesigning the entire system

These principles are intended to remain stable even as the underlying implementation evolves.

---

## Repository Structure

This repository contains the canonical design documents for Megadroid:

<!-- BEGIN AUTO-GENERATED: REPO-STRUCTURE -->
```
megadroid/
├── BOM.csv              # Bill of materials and cost tracking for the Megadroid Minimum Viable System.
├── LICENSE              # Apache License 2.0 (software)
├── LICENSE-HARDWARE.txt # CERN-OHL-S v2 (hardware)
├── MECH.md              # Mechanical implementation details for the Megadroid MVS, including joint layout, structural concepts, actuator placement, and load paths.
├── REHYDRATE.md         # Defines the rehydration process used to generate derived Markdown documents (e.g., SPEC.md, MECH.md, README.md) from authoritative design data and repository metadata.
├── SPEC.md              # System-level specification defining design intent, architectural constraints, and locked assumptions for the Megadroid Minimum Viable System.
├── design/              # Authoritative design parameter definitions for Megadroid, including joints, geometry, and other machine-readable constraints. Files in this directory serve as the single source of truth for system configuration.
├── docs/                # Supporting documentation, background notes, references, and non-authoritative explanatory material related to the Megadroid project.
├── firmware/            # Embedded firmware for real-time motor, joint, and low-level hardware control, intended to run on RP2350-CAN boards or equivalent MCUs.
├── software/            # High-level control software, gait planning, supervisory logic, and development tools, typically running on a Linux SBC such as a Raspberry Pi.
├── templates/           # Jinja templates used to rehydrate derived documentation (e.g., SPEC.md, MECH.md) from authoritative YAML design data.
└── tools/               # Validation, consistency-checking, and rehydration scripts that enforce design constraints and keep derived documents synchronized with authoritative sources.
```
<!-- END AUTO-GENERATED: REPO-STRUCTURE -->

 **Authoritative design data lives in `design/*.yaml`.**

 `SPEC.md` and `MECH.md` are _derived views_ of that data, intended for human readability and review. Any design detail that matters must be represented in the authoritative YAML layer and propagated via the rehydration process.

---

## Licensing

Megadroid uses a dual-license model:

- **Software, firmware, and scripts** are licensed under the **Apache License, Version 2.0** (see `LICENSE`).
- **Hardware designs, mechanical documentation, and bills of materials** are licensed under the **CERN Open Hardware License v2 – Strongly Reciprocal (CERN-OHL-S)** (see `LICENSE-HARDWARE`).

Unless otherwise noted, files are licensed according to the category they fall under.

---

## Project Status

Megadroid is under active development. The architecture and implementation are expected to evolve as the project progresses.

For current design decisions, constraints, and locked assumptions, refer to **`SPEC.md`** and **`MECH.md`**.

---

## Contributing

Contributions are welcome in the form of design feedback, documentation improvements, tooling, and analysis.

Please open an issue or discussion before proposing major architectural changes so that design intent can be preserved.

---

## Disclaimer

Megadroid is an experimental robotics project. Builders and users are responsible for ensuring electrical, mechanical, and operational safety. The authors make no guarantees regarding performance, reliability, or suitability for any particular application.


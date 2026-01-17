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

```
megadroid/
├── LICENSE              # Apache License 2.0 (software)
├── LICENSE-HARDWARE     # CERN-OHL-S v2 (hardware)
├── README.md            # Project overview and vision
├── SPEC.md              # System-level specification (authoritative)
├── MECH.md              # Mechanical design documentation (authoritative)
├── BOM.xlsx             # Bill of materials and cost tracking
├── firmware/            # Low-level control software
├── software/            # High-level control, tools, and simulation
└── docs/                # Supporting documentation and notes
```

**SPEC.md and MECH.md are authoritative.** Any design detail that matters should live there, not in this README.

---

## Licensing

Megadroid uses a dual-license model:

- **Software, firmware, and scripts** are licensed under the **Apache License, Version 2.0** (see `LICENSE`).
- **Hardware designs, mechanical documentation, and bills of materials** are licensed under the **CERN Open Hardware License v2 – Strongly Reciprocal (CERN-OHL-S)** (see `LICENSE-HARDWARE`).

Unless otherwise noted, files are licensed according to the category they fall under.

---

## Project Status

Megadroid is under active development. The architecture and implementation are expected to evolve as the project progresses.

For current design decisions, constraints, and locked assumptions, refer to **SPEC.md** and **MECH.md**.

---

## Contributing

Contributions are welcome in the form of design feedback, documentation improvements, tooling, and analysis.

Please open an issue or discussion before proposing major architectural changes so that design intent can be preserved.

---

## Disclaimer

Megadroid is an experimental robotics project. Builders and users are responsible for ensuring electrical, mechanical, and operational safety. The authors make no guarantees regarding performance, reliability, or suitability for any particular application.


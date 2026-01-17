# megadroid

**Megadroid** is an open-hardware humanoid robot project aimed at developing a **Hubo/ASIMO–class bipedal platform** that can be built, studied, and iterated on by advanced hobbyists and independent researchers, with a rigorous emphasis on **cost minimization, mechanical simplicity, and incremental capability growth**.

The project prioritizes *buildability and clarity* over cutting-edge performance, with the long-term goal of converging toward a **fully featured humanoid** with actuated legs, arms, and torso at a fraction of the cost of traditional research platforms.

---

## Project Goals

### Ultimate Goal

To realize a **fully actuated bipedal humanoid robot**—including legs, arms, and torso—using a modular, repeatable architecture that minimizes part count, manufacturing complexity, and total system cost, while remaining accessible to non-institutional builders.

### Current Focus (Minimum Viable System — MVS)

The current phase of Megadroid targets a **Minimum Viable System (MVS)** capable of:

- Upright bipedal walking on flat ground
- Conservative, quasi-static stair ascent/descent
- Stable torso orientation via active torso DOFs

This MVS serves as a mechanically and electrically conservative foundation for future expansion.

---

## Key Design Principles

- **Cost-first engineering**: design decisions are driven by cost ceilings and availability of parts
- **Modularity**: reuse the same actuator, gearbox classes, and joint blocks wherever possible
- **Proximal actuation**: motors mounted close to the body to reduce distal inertia
- **Position-only control**: avoid torque control and motor current sensing in the MVS
- **Explicit load paths**: twin-rail, box-bulkhead limb construction with double-shear joints
- **Incremental extensibility**: future upgrades are anticipated but not required for the MVS

---

## Minimum Viable System (MVS) Overview

### Degrees of Freedom

- **Legs (×2)**: hip pitch, hip roll, knee pitch
- **Torso**: pitch, roll, yaw
- **Total actuated DOF**: 9

Ankles are passive/locked in the MVS and use deformable ball feet for compliance.

### Actuation

- All actuators are **775 brushed DC motors**
- Standardized gear reduction classes
- Joint-mounted absolute encoders only (no motor encoders)

### Control Stack

- **High-level control**: Raspberry Pi (Linux)
- **Real-time control**: 3× Waveshare RP2350-CAN boards
- **Communication**: CAN bus

### Power

- **Battery**: Toro 88675 (60 V, 7.5 Ah) — *excluded from cost target*
- Central DC/DC conversion (60 V → 24 V)

### Cost Target

- **< \$1000 USD total system cost**
- Battery and end-of-limb force/torque sensors explicitly excluded

---

## Repository Structure

```
megadroid/
├── README.md        # Project overview and context
├── SPEC.md          # System-level specification (authoritative)
├── MECH.md          # Mechanical design notes (authoritative)
├── BOM.xlsx         # Cost-tracked bill of materials
├── firmware/        # Low-level motor and joint control (Apache 2.0)
├── software/        # High-level control, tools, simulation (Apache 2.0)
└── docs/            # Supporting documentation and notes
```

**SPEC.md and MECH.md are authoritative**. Design discussions and changes should reference them explicitly.

---

## Licensing

Megadroid uses a **dual-license model**:

- **Software, firmware, and scripts** are licensed under the **Apache License, Version 2.0** (see `LICENSE`).
- **Hardware designs, mechanical documentation, and BOMs** are licensed under the **CERN Open Hardware License v2 – Strongly Reciprocal (CERN-OHL-S)** (see `LICENSE-HARDWARE`).

Unless otherwise noted, files are licensed according to the category they fall under.

---

## Project Status

- Architecture: **LOCKED (MVS)**
- Mechanical concept: **LOCKED**
- Electrical/control stack: **LOCKED**
- Arms, active ankles, force control: **Planned (post-MVS)**

Megadroid is under active development. Expect iteration, refinement, and occasional breaking changes as the design converges.

---

## Contributing

Contributions are welcome, particularly in:

- mechanical refinement and tolerance analysis
- low-cost manufacturing strategies
- gait planning under conservative control assumptions
- documentation and build guides

Please open an issue or discussion before proposing large architectural changes.

---

## Disclaimer

Megadroid is an experimental robotics project. Builders are responsible for ensuring electrical, mechanical, and operational safety. The authors make no guarantees regarding performance, reliability, or suitability for any particular application.


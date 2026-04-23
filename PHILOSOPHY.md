<!--
name: PHILOSOPHY.md
type: design-philosophy
description: Project vision, goals, design philosophy, and document authority hierarchy
authority: static (written once, updated deliberately — not auto-generated)
-->
# Megadroid — Design Philosophy

**Status:** Static  
**Scope:** Project-wide  
**Last updated:** 2026-04-23

This document captures the *why* behind Megadroid: what the project is trying
to exist as in the world, what principles constrain design decisions, and how
authority is structured across the project's documents. It is not generated
from YAML. It is written deliberately and updated only when the project's
foundational thinking changes.

---

## 1. The Goal

The ultimate goal of Megadroid is to build a **walking bipedal humanoid robot**
comparable in scale, proportion, and locomotion character to Honda's original
ASIMO (2000) or KAIST's KHR-3 Hubo (2005) — robots that stood approximately
120–125 cm tall, weighed 43–55 kg, and walked using deliberate, stable,
human-coded gait strategies — at a cost accessible to an individual hobbyist.

This is not a goal of matching the state of the art. It is a goal of making a
specific, well-understood class of robot exist at a price point where it never
has before.

---

## 2. Why This Goal

### 2.1 The Problem With Current Humanoid Robots

Research-grade humanoid robots cost tens of thousands to hundreds of thousands
of dollars. Even the most affordable current platforms — commercial products
targeting developers and educators — are priced at levels that put them in the
category of institutional purchases: things that research groups buy carefully,
treat as investments, and use conservatively.

This is not a new situation. It has been the case for essentially the entire
history of humanoid robotics. And it has a consequence: the people interacting
with these robots are, by selection, cautious. The robot is valuable. Breaking
it is expensive. Radical experiments are rarely run on things you cannot afford
to replace.

### 2.2 The Personal Computer Analogy

The history of personal computing offers a useful precedent. In the early 1970s,
computers existed — but they were institutional. Universities, corporations, and
government agencies owned them. The people who used them were, by necessity,
careful. The machines were too valuable to experiment recklessly with.

What changed with the Altair 8800 and the machines that followed was not
primarily capability. It was price. A computer that a hobbyist could afford to
own — and potentially break — enabled a qualitatively different kind of
engagement. The novel uses of the personal computer that we now take for granted
were not discovered by the people running mainframes. They were discovered by
people who owned machines they were not afraid to push to their limits.

Megadroid is premised on the belief that something similar is true for humanoid
robotics. The interesting things that hobbyists will do with a robot they own
and are not afraid to break will be different from what institutional researchers
do with expensive platforms. Enabling that requires hitting a specific price
point — not because lower cost is intrinsically good, but because cost is the
gating factor for that kind of ownership.

### 2.3 The Character of ASIMO-Era Robots

There is a second motivation, less strategic and more aesthetic, but no less
real: the robots being built today do not move the way ASIMO moved.

Contemporary humanoid robots — including many well-funded commercial efforts —
use quasi-direct-drive actuators with proprioceptive torque sensing and
learning-based or model-predictive control strategies. These produce impressive,
robust, dynamically stable motion. But the motion has a particular character:
it looks like continuous optimization. The robot is always slightly improvising.
It adapts fluidly, which is remarkable, but it does not move with the deliberate,
intentional quality that ASIMO had.

ASIMO's motion was the product of a different philosophy: high gear ratios,
stiff joints, precise position control, and ZMP-based gait strategies that
mathematically guaranteed stability within the support polygon at each instant.
The robot did exactly what it was told, and what it was told was carefully
designed. The result was motion that felt considered. Purposeful. Legible.

Megadroid is designed in this tradition — not because the ASIMO approach is
superior to modern methods in every dimension, but because it is the right
approach for this combination of goals: low cost, hobbyist buildability, and
that particular quality of movement.

---

## 3. Design Principles

These principles are ordered by priority. When they conflict, earlier principles
take precedence.

### 3.1 Hobbyist Accessibility

The robot must be buildable and ownable by an individual without institutional
resources. This is the foundational constraint from which all others derive.

### 3.2 Cost Proportional to a Capable Hobbyist Desktop PC

The target cost for a complete, walking Megadroid system is comparable to a
capable hobbyist desktop PC — approximately $800–$1,500 USD as of 2026, with
the battery and end-of-limb force/torque sensors excluded from cost accounting.

This is not a hard ceiling but a design discipline. The question to ask of any
component choice is: *could an individual hobbyist justify this purchase without
it being a significant financial decision?* If the answer is no, the choice
requires strong justification.

Costs that scale with quantity (i.e., costs that fall as more units are built)
are evaluated at single-unit hobbyist quantities, not at production scale.

### 3.3 Reproducibility From a Reliable Supply Chain

Every component must be sourceable by someone building the robot independently,
from suppliers with stable availability. One-off parts, custom-manufactured
components available only in large quantities, and parts dependent on a single
supplier relationship are avoided. The robot must be rebuildable.

### 3.4 Mechanical Conservatism

Structures are designed for explicit, inspectable load paths. Double-shear
bearing support is required at all primary joints. Complexity is introduced only
when necessary. When in doubt, the simpler design is preferred.

### 3.5 Software-Enforced Safety Over Hardware Complexity

Stability, safety, and hardware protection are implemented in software where
possible — joint velocity limits, acceleration limits, gait timing constraints,
brownout-aware behavior — rather than through additional sensors, exotic
actuators, or protective mechanical systems. This keeps the hardware simple and
the cost low while maintaining safe operation.

### 3.6 Incremental Extensibility Without Redesign

The MVS is explicitly designed as a foundation, not a complete system. Future
capability — active ankles, arms, additional sensing — must be addable without
requiring redesign of the existing structure. Mechanical interfaces are defined
and frozen with future modules in mind.

---

## 4. Actuator Philosophy

Megadroid uses **775-class brushed DC motors** with **high gear reductions**
(comparable to Hubo's 120:1–300:1 total ratios, achieved via combined gearbox
and belt stages) and **position-only control**.

This is a deliberate choice, not a cost compromise, for the following reasons:

**It is validated by existence proof.** The KHR-3 Hubo used brushed DC motors
at comparable gear ratios and walked. The actuator philosophy is not
speculative — it is the same class of solution that produced the robots
Megadroid is trying to emulate.

**It is compatible with ZMP-based gait control.** High gear ratios produce stiff,
predictable joints that do exactly what the controller commands. This is the
mechanical foundation that ZMP control depends on. Backdrivable, torque-sensing
actuators offer advantages for other control strategies, but they are not needed
for — and do not improve — the ZMP approach.

**It eliminates the need for per-joint torque sensing.** Ground reaction forces
during walking are fully characterized by the 6-DOF force/torque sensors at each
foot. Joint-level torque sensing provides redundant information in this context.
Since brushed DC motor controllers do not require the high-speed, precision
current sensing that BLDC FOC controllers do, the entire actuation system
(motor + driver + encoder) can be realized at a fraction of the cost of
equivalent BLDC solutions.

**It keeps the controller cost tractable.** A brushed DC motor requires only an
H-bridge driver, an encoder, and a PID loop running on already-present
microcontroller hardware. A BLDC motor requires a dedicated FOC controller —
in the best case doubling the cost per joint, in practice often more. At 11+
joints, this difference is significant at the target BOM.

The tradeoff is brush wear over time, which is accepted for this use case. A
hobbyist robot used in sessions rather than continuously will not encounter brush
life as a practical limitation.

---

## 5. Control Philosophy

Megadroid implements **ZMP-based (Zero Moment Point) gait control** with
**position-only joint commands**.

This approach was pioneered by Honda in the P-series and ASIMO, and implemented
independently in KHR-3 Hubo. It is fully documented in the open literature.
The mathematics is textbook. The compute required is trivially available on
modern single-board computers — ASIMO's original controller ran on a Pentium III;
a Raspberry Pi 5 substantially exceeds that.

ZMP control is not the frontier of bipedal locomotion research. That is
precisely why it is appropriate here. The goal is not to advance the state of
the art in control theory. The goal is to produce a walking robot using methods
that are understood, implementable by a single capable developer, and aligned
with the motion character that Megadroid is designed to produce.

---

## 6. The Minimum Viable System

The MVS is the first complete, walking instantiation of Megadroid. It is defined
by what is necessary to produce genuine bipedal walking — not by what is minimal
in an absolute sense.

**MVS includes:**
- Two legs, each with hip pitch, hip roll, knee pitch, and ankle pitch (11
  actuated DOF total across legs and torso)
- Three torso DOF (pitch, roll, yaw)
- 6-DOF force/torque sensors at each foot
- Full control and power stack

**Ankle pitch is included in the MVS** because it is the minimum addition that
enables ZMP-controlled walking with genuine single-support phase stability.
Without ankle pitch, the robot is limited to double-support shuffling — a
different and significantly less capable locomotion mode. A milestone that
requires immediate extension upon completion is not a valid milestone.

Ankle roll, arms, and head are post-MVS additions. They improve capability and
completeness but are not required for walking.

**The MVS will not walk with ASIMO-quality gait.** It will walk slowly,
deliberately, and without the fluency that a full system with arms and refined
software will eventually achieve. That is acceptable. The MVS exists to validate
the mechanical architecture, electronics stack, and control software foundation
— and to walk, in some genuine capacity, on flat ground.

---

## 7. Document Authority Hierarchy

The following hierarchy defines which documents take precedence when conflicts arise.

| Priority | Document | Authority | Notes |
|---|---|---|---|
| 1 | `PHILOSOPHY.md` | Static — this document | Defines *why*. Updated deliberately, never generated. |
| 2 | `SPEC.md` | Authoritative | Defines *what* the system is. Highest technical authority. |
| 3 | `design/*.yaml` | Authoritative (source of truth) | Machine-readable parameters. All numeric values live here. |
| 4 | `MECH.md` | Derived | Defines *how* the system is physically realized. Must not contradict SPEC. |
| 5 | `BOM.csv` | Derived | Cost tracking. Must not contradict SPEC. |
| 6 | `PROCESS.md` | Informative | Governs workflow. Does not override SPEC. |
| 7 | `REHYDRATE.md` | Informative | Documents the rehydration process. |
| 8 | `docs/P*_VALIDATION.md` | Static milestone records | Written once at stage completion. Not regenerated. |
| — | Conversational reasoning | **Never authoritative** | Analyses, discussions, and LLM-assisted reasoning become authoritative only when written into SPEC.md and committed. |

**The last row is the most important one.** A design decision exists only when
it is written down and committed. Everything else — however well-reasoned — is
a candidate, not a commitment.

---

## 8. What Megadroid Is Not Trying to Be

Explicitly, so that future decisions have a reference:

- **Not a research platform competing on dynamic performance.** There are many
  well-funded efforts building dynamically impressive humanoids. Megadroid is
  not trying to outperform them. It is trying to exist at a price point they
  do not.

- **Not a product.** Megadroid is open hardware. There is no production run,
  no warranty, no customer. The supply chain must be reliable enough that
  someone can build one independently, but unit cost at volume is not the goal.

- **Not a stepping stone to a different kind of robot.** The actuator philosophy,
  control approach, and mechanical architecture are chosen for this robot and
  this goal. They are not compromises pending a future redesign. If Megadroid
  eventually evolves to a different control philosophy, that is a new design
  decision made deliberately — not an implicit plan.

- **Not finished at v1.0.** The MVS is a foundation. Arms, refined gait,
  improved sensing, and eventually a head are all planned. But they are planned
  as additions to a validated base, not as requirements for v1.0.

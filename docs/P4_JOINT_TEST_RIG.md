<!--
name: P4_JOINT_TEST_RIG.md
type: test-plan
description: Single-joint bench rig to replace the drivetrain PLACEHOLDERs with measurements
-->
# Stage P4 — Single-Joint Test Rig

**Status:** Proposed — not yet built
**Date:** 2026-09-10

## Why

`docs/P3_MODEL_FIDELITY.md` established that simulation fidelity, not control
theory, has been the blocker. Six authoritative values remain guesses, and four
times in one session a guessed value produced a confident wrong conclusion. None
of them can be resolved by more simulation — they are physical facts.

This rig measures one actuated joint, built from the same parts as the robot, to
replace those guesses with numbers.

It is **not** a jump ahead to P6. `PROCESS.md` Stage P4 requires "define actuator
performance envelopes" and "validate gear ratios against simulated loads". A
V-model pairs each design stage with a verification activity; this rig is the
verification leg of P4.

---

## What it resolves

| # | Parameter | Current state | Measured by |
|---|---|---|---|
| 1 | `actuation.motor.stall_torque_nm` | PLACEHOLDER 0.20 | Test A |
| 2 | `actuation.motor.no_load_speed_rpm` | PLACEHOLDER 15000 | Test A |
| 3 | `actuation.drivetrain.efficiency` | PLACEHOLDER 0.70 | Test B |
| 4 | `actuation.drivetrain.belt_stage.ratio` | inferred 4:1, never engineered | Test A (verified directly) |
| 5 | `modeling_limitations.flat_torque_cap` | droop not modelled at all | Test A |
| 6 | `modeling_limitations.thermal` | no model | Test C |
| 7 | `modeling_limitations.compliance` | backlash/belt stretch not modelled | Test D |
| 8 | `geometry.foot_stack.sole.contact_timeconst_s` | PLACEHOLDER 0.05 | Test F (separate fixture) |

**Not resolved here**, listed so the gap stays visible:
- `mass.yaml` link masses — weigh parts as built during P6.
- `sensors.yaml` IMU noise/bias — separate static bench log (Test G), needs no rig.

---

## Fixture

One joint, using the robot's actual parts so the numbers transfer:

| Item | Source | Note |
|---|---|---|
| 775 brushed DC motor | BOM, $18 | The part that will fly |
| 20:1 planetary gearbox | BOM, $25 | " |
| 4:1 belt stage (HTD 5M, 15 mm) | BOM, $12 | Verifies the inferred ratio |
| Joint-mounted absolute encoder | BOM, $8 | Also validates the encoder policy itself |
| Motor driver | same class as flight controller | Drive characteristics matter |
| 24 V bench supply with current readout | — | Nominal voltage per `actuation.yaml` |

**Rig-only instrumentation** (not on the robot):

| Item | Purpose |
|---|---|
| Reaction torque sensor or lever arm + load cell | Ground truth output torque |
| Shunt/hall current sensor on the 24 V line | Electrical power in |
| Thermocouple on motor case | Thermal limit |
| Adjustable brake or mass-on-lever load | Sweep the operating point |

Note the deliberate asymmetry: `design/sensors.yaml` **excludes** motor current
sensing on the robot, but the rig needs it to compute efficiency. That is
correct — the rig characterises the drivetrain so the robot doesn't have to.

Mechanically, mount the output shaft in double shear per
`geometry.yaml.bearing_support` — the rig should not introduce a compliance the
robot won't have.

---

## Tests

### Test A — Torque–speed curve
Sweep load from free-run to stall at fixed 24 V. Log output torque, output
speed, supply current.

**Yields:** stall torque, no-load speed, and the **droop curve**. Simulation
currently applies a flat `forcerange`, which is optimistic at speed and accurate
only near stall. Also verifies total reduction directly by comparing motor-side
to joint-side revolutions — the 4:1 belt ratio is currently inferred, never
engineered.

*Would change the design if:* measured stall torque is materially below
0.20 N·m, or the droop is steep enough that the 6.4 rad/s knee speed needed for
HUBO-pace walking sits below the usable band. Either forces the ratio decision
open again.

### Test B — Efficiency
From Test A data: mechanical power out (τ·ω) over electrical power in (V·I),
across the operating range.

**Yields:** a real efficiency curve rather than the flat 0.70 placeholder.
Efficiency is not constant with load, so the single number in `actuation.yaml`
should likely become a curve or a worst-case figure.

### Test C — Thermal / continuous rating
Run a representative walking duty cycle (from simulated joint torque traces) and
log case temperature to steady state. Repeat at increasing duty until thermal
limit.

**Yields:** continuous torque rating, which is the number that actually sizes a
walking robot. Stall torque is instantaneous; `actuation.yaml` records having no
thermal model at all. If continuous rating is far below stall, the flat cap in
simulation is badly wrong in a way that matters.

### Test D — Backlash and compliance
Lock the output, apply known torque both directions, measure angular deflection
at the joint encoder. Separately, drive to position and reverse to measure lost
motion.

**Yields:** backlash angle and drivetrain stiffness. Neither is modelled.
`sensors.yaml`'s joint-mounted-encoder policy exists *because* these are expected
to be significant — this is the first check of that assumption.

### Test E — Position servo response
Closed-loop step and tracking response at the flight controller's gain, under
representative load.

**Yields:** validation of the `kp` question. Simulation found the stiffness
window *empty* at 20:1 — no gain both held the pose and avoided saturation. This
checks whether that survives contact with a real drivetrain, and whether backlash
from Test D changes it.

### Test F — Sole pad compliance *(separate fixture)*
Press or drop a known mass onto the sole assembly; log force and displacement to
settling.

**Yields:** contact stiffness and damping →
`geometry.foot_stack.sole.contact_timeconst_s`. This is the highest-leverage
single measurement available: sweeping it in simulation moved 10 N push survival
from 0/12 to 9/12 timings, and the current value is an unmeasured placeholder
chosen on physical plausibility rather than data.

### Test G — IMU noise and bias *(no rig needed)*
Log the actual IMU stationary for ~1 hour; compute Allan variance or simply bias
drift and noise density.

**Yields:** the noise model the MJCF lacks. `state_estimator.py` records that its
complementary-filter constant **cannot be honestly tuned** until this exists —
the sweep currently rewards a setting that would diverge on hardware.

---

## Order of value

If only some tests get done, this is the priority:

1. **Test F (sole)** — cheapest, and the parameter shown to dominate disturbance behaviour.
2. **Test A (torque–speed)** — sizes everything; resolves three placeholders at once.
3. **Test C (thermal)** — determines whether the envelope is real or instantaneous.
4. **Test G (IMU)** — unblocks honest estimator tuning; needs no fixture.
5. **Tests D, E** — refine the control model once the envelope is known.

## Prior art

Surveyed 2026-09-10 for existing open designs that could be adapted rather
than built from scratch. Nothing matches whole; the rig splits into two
halves and the better prior art is on the half that isn't a dynamometer.

**The filter.** Published motor dynos measure at the *motor shaft* — high
RPM, low torque — and their load absorbers (eddy brakes, propeller loads, a
second motor) are sized for that regime. Test A measures at the *joint
output*, after the full reduction recorded in `design/actuation.yaml`: low
speed, high torque, the inverse regime. Their instrumentation and logging
transfer; their loading mechanisms do not. At joint speeds this low, a
quasi-static sweep against a friction brake and a load cell is simpler than
absorbing the power electrically.

| Test | Closest existing design | What transfers |
|---|---|---|
| A, B, C | [Capo01 ODrive dynamometer](https://github.com/Capo01/odrive_based_electric_motor_dynamometer) (GPL-3.0) | Absorber on a pivot arm with a load cell reading reaction torque; current shunts on both sides so motor and controller losses separate. Brushless/ODrive electronics and its 3.5 N·m brake are both unusable here. |
| A | [Cambridge Univ. Drone Society motor test stand](http://cuds.soc.srcf.net/2021/07/25/designing-a-motor-test-stand-part-1/) | Build detail for the lever-arm-onto-load-cell fixture — the concrete version of the cheap Test A fallback noted under Cost. |
| A–C | [RAPID: An Inexpensive Open Source Dynamometer for Robotics Applications](https://ieeexplore.ieee.org/document/6584831/) ([RG](https://www.researchgate.net/publication/264566981_RAPID_An_Inexpensive_Open_Source_Dynamometer_for_Robotics_Applications)) | Purpose-built for brushed DC, robotics-oriented, automated PWM sweep. **Two caveats** below. |
| D | [Open-source test stand for backlash measurement in UART servo motors](https://www.sciencedirect.com/science/article/pii/S2468067226000271) | Lock-and-release fixture geometry, extensible with a load cell to give backlash as a function of applied torque — exactly Test D. Different actuator class, same rig. |
| F | [CNC Kitchen Open-Pull](https://github.com/CNCKitchen/Open-Pull), [UMTK](https://peer.asee.org/use-of-a-low-cost-open-source-universal-mechanical-testing-machine-in-an-introductory-materials-science-course.pdf), [low-cost UTM](https://hackaday.io/project/192166-low-cost-universal-tensile-testing-machine/details), [OSE testing machine](https://wiki.opensourceecology.org/wiki/Open_Source_Universal_Material_Science_Destructive_Testing_Machine) | Leadscrew + load cell frames that do compression, not just tension. Gives force-vs-displacement to settling — the stiffness half of the sole measurement. |
| — | [Design and Characterization of 3D Printed, Open-Source Actuators for Legged Locomotion](https://arxiv.org/pdf/2202.12395) | Methodology, not hardware: the same characterisation problem in the same application domain. |

**RAPID's two caveats.** First, the paper asserts drawings, schematics and
software are freely downloadable, but the files could not be located —
verify they are reachable before planning around it. Second, and more
important: its headline feature is modelling system inertia and friction to
*remove the need for a torque sensor*. Test B **is** the friction
measurement. A rig that assumes a friction model to infer torque cannot
measure the friction. Its sweep automation transfers; its measurement
principle is disqualified here.

**What this suggests for the build order.** It reinforces Test F first for a
second, independent reason: it is a compression test, not a dyno problem, and
the open hardware for it is markedly better. A UTM-style frame covers the
stiffness half; damping still needs the drop test, which is a mass, a guide
rail and a load cell — cheap enough not to need a donor design. Then build
Test A as a static reaction-arm fixture, which serves B, C, D and E unchanged.

## Cost

Roughly $65 in flight parts plus bench instrumentation. A reaction torque sensor
is the main expense; a lever arm and kitchen scale is a crude but workable
substitute for Test A if budget matters more than precision.

---

## Closing the loop

Each measurement replaces a specific field in `design/*.yaml`, then the models
regenerate and the full P3 battery re-runs. The mandatory commit order applies
(YAML → tooling → derived docs → generated models).

Expect results to change: this session found that correcting a single modelling
assumption moved push survival from 0/12 to 9/12, and that stale gains
invalidated three files' documented claims. **Re-running the battery after each
parameter update is the point, not a formality.**

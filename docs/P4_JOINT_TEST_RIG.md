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
| Lever arm + harvested bar load cell | Ground truth output torque — see *Load cells* below |
| Shunt/hall current sensor on the 24 V line | Electrical power in |
| Thermocouple on motor case | Thermal limit |
| Adjustable brake or mass-on-lever load | Sweep the operating point |

Note the deliberate asymmetry: `design/sensors.yaml` **excludes** motor current
sensing on the robot, but the rig needs it to compute efficiency. That is
correct — the rig characterises the drivetrain so the robot doesn't have to.

Mechanically, mount the output shaft in double shear per
`geometry.yaml.bearing_support` — the rig should not introduce a compliance the
robot won't have.

### Load cells

Torque measurement is **not** the cost driver this document originally assumed.
The build sources bar-type load cells by harvesting them from used Wii Fit
Balance Boards — four per board, one per corner, at a small fraction of the
price of equivalent new cells. The same supply is what makes the foot F/T
sensors effectively free (`design/sensors.yaml`,
`end_of_limb_sensing.ft_sensor.cost_accounting: excluded`), so the rig and the
robot draw on one stock of parts.

Two consequences for this rig, both of which are reasons to bench-characterise
a cell *before* designing anything around it:

- **Confirm the sensitive axis.** These are bending-beam cells: gauges read
  bending strain with one end fixed and load applied at the other. Loaded along
  the beam's long axis instead, output is small and off-axis moments dominate.
  For the Test A lever arm this is easy to get right — mount the cell in its
  native cantilever orientation and drive it perpendicular to the beam — but it
  must be deliberate, not assumed.
- **Establish the per-cell rating and linearity.** A board's total rating
  divided by four is a starting guess, not a specification, and harvested parts
  carry no datasheet. Test A stalls the joint against this cell, so its range
  has to cover peak output torque over the lever length actually used.

Characterising one cell costs nothing beyond a set of known masses, and it
doubles as the first step of the F/T sensor work. Added as **Test H** below.

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

### Test H — Harvested load-cell characterisation *(no rig needed)*
Hang known masses from a single cell in its native cantilever mounting. Record
output against load; repeat off-axis and after a thermal soak next to a running
motor.

**Yields:** sensitive axis confirmed, usable range, linearity, off-axis
sensitivity, and thermal drift — the four things a datasheet would give and a
harvested part does not. Prerequisite for Test A (which loads a cell to joint
stall torque) and for the foot F/T sensors, which use six of the same cells
each.

*Would change the design if:* per-cell range proves too low for peak joint
torque at a practical lever length, or off-axis sensitivity is high enough that
a six-cell wrench solution would be poorly conditioned.

**Note the signal-chain cost, which is where the real expense moved.** Twelve
channels across two feet need amplification and multi-channel 24-bit
acquisition. The commodity HX711 samples far too slowly for a walking control
loop, so a faster front end (or one converter per cell) is the part of the F/T
story that actually costs money and schedule. That also exposes a modelling
gap: the MJCF's force/torque sensors are read every control tick with no
bandwidth, latency, quantisation or noise model at all — the same class of
unmodelled-hardware defect `docs/P3_MODEL_FIDELITY.md` documents four instances
of. Whatever sample rate the real chain achieves should be measured here and
fed back into the simulation.

---

## Order of value

If only some tests get done, this is the priority:

1. **Test F (sole)** — cheapest, and the parameter shown to dominate disturbance behaviour.
2. **Test H (load cell)** — trivially cheap, and Tests A and F both depend on trusting a harvested cell.
3. **Test A (torque–speed)** — sizes everything; resolves three placeholders at once.
4. **Test C (thermal)** — determines whether the envelope is real or instantaneous.
5. **Test G (IMU)** — unblocks honest estimator tuning; needs no fixture.
6. **Tests D, E** — refine the control model once the envelope is known.

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
| A–C | [RAPID: An Inexpensive Open Source Dynamometer for Robotics Applications](https://ieeexplore.ieee.org/document/6584831/) (Morozovsky, Moroto & Bewley, UCSD, 2013 — paper in `reference-material/`) | Purpose-built for brushed DC, robotics-oriented, automated PWM sweep. Full CAD, BOM and software located — see below, along with **what it cannot do for us**. |
| D | [Open-source test stand for backlash measurement in UART servo motors](https://www.sciencedirect.com/science/article/pii/S2468067226000271) | Lock-and-release fixture geometry, extensible with a load cell to give backlash as a function of applied torque — exactly Test D. Different actuator class, same rig. |
| F | [CNC Kitchen Open-Pull](https://github.com/CNCKitchen/Open-Pull), [UMTK](https://peer.asee.org/use-of-a-low-cost-open-source-universal-mechanical-testing-machine-in-an-introductory-materials-science-course.pdf), [low-cost UTM](https://hackaday.io/project/192166-low-cost-universal-tensile-testing-machine/details), [OSE testing machine](https://wiki.opensourceecology.org/wiki/Open_Source_Universal_Material_Science_Destructive_Testing_Machine) | Leadscrew + load cell frames that do compression, not just tension. Gives force-vs-displacement to settling — the stiffness half of the sole measurement. |
| — | [Design and Characterization of 3D Printed, Open-Source Actuators for Legged Locomotion](https://arxiv.org/pdf/2202.12395) | Methodology, not hardware: the same characterisation problem in the same application domain. |

### RAPID: where the files are, and what they are good for

The paper points at `http://robotics.ucsd.edu/dyno`, which is dead — the
Wayback Machine holds exactly one capture of it, from 2024, already a 404.
The files survive as **Supplemental File 5** of the lead author's
dissertation, [*Design, Dynamics, and Control of Mobile Robotic Systems*
(Morozovsky, UCSD 2014)](https://escholarship.org/uc/item/15d2s309), open
access:

```
https://escholarship.org/content/qt15d2s309/supp/DC_Motor_Dynamometer_Files.zip
```

Retrieved and unpacked to `reference-material/humanoid-robotics/
RAPID-dynamometer-files/` (gitignored, like the rest of that tree):

| File | Contents |
|---|---|
| `Bill_of_Materials.pdf` | Full parts list |
| `UCSD_RAPID_CAD.zip` | 9 STL + 2 DXF — base plate, chuck/motor/encoder towers, inertial disc, clamping hub, shaft adapter, full assembly |
| `adapterPlates.zip` | 3 parametric SolidWorks parts + STLs (the only editable-source parts in the set) |
| `DynoMiniLabVIEW_2012.zip` | LabVIEW 2012 project, 17 VIs, FPGA bitfiles |

**Three limits, now read from the paper rather than inferred.**

1. **The load is an inertial disc, not a brake** (§II-A) — it spins the motor
   up and characterises the spin-down, fitting a gray-box model by least
   squares. There is no sustained loaded operating point, so **Test C is
   impossible on this rig**: a thermal rating needs the motor held under load
   to steady state, which a spin-down cannot do.
2. **No torque sensor, deliberately** — friction is *modelled* to avoid one.
   Test B exists to *measure* that friction. A rig that assumes a friction
   model to infer torque cannot measure the friction, so B is out too.
3. **Scale mismatch at the chuck.** Its three-jaw chuck spans 1.00–6.35 mm
   shafts, sized for motor-shaft work; megadroid's standard joint output
   shaft (`design/geometry.yaml`) is 12 mm. The chuck tower would need
   redesigning, and only the adapter plates ship as editable CAD — the towers
   are STL, so that means re-modelling from mesh or DXF.

**What is genuinely worth taking.** The BOM is a validated parts list for
brushed-DC characterisation (US Digital E6 2500 CPR encoder, Allegro ACS712
current sensor, Toshiba TB6612FNG driver, NI myDAQ). The `basePlate.dxf` and
`inertialDisk.dxf` are laser-cut-ready. And the **method** deserves separate
consideration from the hardware: gray-box parameter identification from
spin-down tests recovers motor constants without any torque sensor, which
would resolve items 1 and 2 of the table at the top of this document —
`stall_torque_nm` and `no_load_speed_rpm` — for the price of an encoder and
a current sensor. It handles geared motors explicitly (effective inertia
`J_E = J_gearbox + γ²·J_motor`). That is a real, cheap alternative route to
the motor parameters; it simply cannot also deliver B, C, or the output-side
droop curve, so it complements the reaction-arm fixture rather than
replacing it.

**What this suggests for the build order.** It reinforces Test F first for a
second, independent reason: it is a compression test, not a dyno problem, and
the open hardware for it is markedly better. A UTM-style frame covers the
stiffness half; damping still needs the drop test, which is a mass, a guide
rail and a load cell — cheap enough not to need a donor design. Then build
Test A as a static reaction-arm fixture, which serves B, C, D and E unchanged.

## Cost

Roughly $65 in flight parts plus bench instrumentation, and the instrumentation
is cheaper than first assumed: load cells come from used Balance Boards rather
than as a several-hundred-dollar reaction torque sensor. What remains is the
signal chain (amplification and fast multi-channel acquisition, per Test H), a
thermocouple, and a current sensor. A reaction torque sensor
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

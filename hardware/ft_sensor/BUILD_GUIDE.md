# Foot F/T sensor: build guide (model v0)

**Status: a verified model, not a proven design.** The solids are checked for interference, envelope, hole
break-out, tool access and assembly order (below). Nothing is FEA'd and nothing has been built. The cell's rating,
stiffness and mounting are unmeasured, so run Tests H, H2, H3 and R (`docs/P4_JOINT_TEST_RIG.md`,
`BENCH_TEST_STRUT.md`) before cutting the six-member unit.

Everything below comes from `cad_build.py`, which reads `design/ft_sensor.yaml` (`build_candidate`) and the layout
search. To regenerate after any change: `pip install build123d` (in a scratch venv) then
`python3 hardware/ft_sensor/cad_build.py --mesh viewer_mesh.json`.

## Files (`cad/megadroid_ft_v0/`)

| File | What it is |
|---|---|
| `sensor_assembly.step` | all 38 solids, coloured, in place. Frame: mm, z up, origin at the centre of the bottom cap's inner face |
| `cap_bottom.step`, `cap_top.step` | the two machined caps (**mirror images, two different parts**) |
| `rod.step` | the turned rod, one drawing, six off |
| `load_cell_modified.step` | the harvested cell with its two holes reamed |
| `fasteners.step` | one of each: cell bolt and washer, shim, free-end screw |
| `features.csv` | tilt, azimuth and hole entry points of all six wedges and bosses |
| `checks.txt` | the full output of the checks |

## The design in one paragraph

Six load cells on tilted wedge blocks, each in series with a turned flexure rod, between two 3 mm hex caps
(88.85 mm across flats, 57.544 mm overall). Three cells sit on each cap and alternate around the ring (inversion
symmetry), so the sensor keeps all six wrench axes. Every cell's bolt axis is tilted 54.5 deg from the cap normal.
Each rod runs from its cell on one cap to a boss on the other cap and carries axial load only, through two thin
necks. The wedges and bosses are machined into the caps.

## Parts

| Part | Qty | Material and source | Notes |
|---|---|---|---|
| Cap, bottom | 1 | 7075-T6 plate, hex 102.6 across corners x 88.85 across flats, 14.5 mm thick stock | mirror of the top cap |
| Cap, top | 1 | same | mirror of the bottom cap |
| Rod | 6 | 7075-T6 bar, 10 mm dia | turned, then 8 mm AF flats milled at both ends |
| Load cell | 6 | harvested, 42.25 x 13.25 x 8.5 mm | holes reamed: 6.1 mm fixed end, 7.0 mm free end |
| Cell bolt | 6 | M6x16 socket head cap screw, 12.9 | with 6.4 x 12 x 1.6 hardened washer |
| Free-end screw | 6 | M6x14 low-head socket screw (head 10 dia x 3.9) | **grade matters: standard DIN 7984 is 8.8; source 10.9 or 12.9 low-head if you can** |
| Shim | 6 | 0.25 mm stainless, 12 x 6.4 | plus an assortment 0.05 to 0.5 mm to zero each cell |
| Thread locker | | medium strength (e.g. 243) | boss-end studs and cell bolts |

## Machining notes

**Caps.** Hex outline corners on the x axis. The plate is 3.0 mm. Each cap carries three wedges and three bosses; the
overall height of the machined cap is 13.4 mm. Every wedge face and boss face is **square to its own axis, 54.5 deg
from the cap normal**, so this needs 5-axis or 3+2 work with a tilting table. `features.csv` gives, per member, the
tilt, the azimuth of the axis and the entry point of each tapped hole in the model frame.
- Wedge: 12.7 x 13.25 mm face, runs down to the cap plane. Tapped M6, **7.5 mm deep** (this is the deepest hole that
  keeps a 1 mm wall to the cap's outer face).
- Boss: 10 x 10 mm face, runs to the cap plane. Tapped M6, **7.0 mm deep**.
- Position of every hole entry: +-0.05 mm. Face flatness and squareness to the hole axis: 0.02 mm.
- The cell must sit on the wedge face with no rocking: the wedge face is the datum the whole joint rests on.
- Outer faces: plain. There is **no foot or ankle interface yet** (see open items).

**Rod** (see `rod.step`). 7075-T6, turned between centres, then flats.
- Overall: cell end block 6.5 mm, neck 4.5, body 2.8 (6 dia), neck 4.5, boss-end hex 3.0, M6x1 stud 6.5 mm.
- **Necks 2.4 mm dia x 4.5 mm long, 0.3 mm fillets at both ends.** Diameter +-0.02 mm. Necks coaxial with the body and
  with each end block to 0.02 mm. Polish the necks: they are fatigue-critical and surface finish matters.
- Hex blocks 8 mm across flats at both ends. The flats are outboard of the necks so that no tightening torque ever
  passes through a neck: **tighten only while holding a hex.**
- Cell end: blind M6 thread, 6.0 mm deep, from the end face.

**Load cell.** Ream the two M6 threaded holes to 6.1 mm (fixed end) and 7.0 mm (free end). The 0.5 mm of play at the
free end is what lets the two cap halves close without a precision fit. Measure the gauge area first and keep the
reamer clear of it. The cell's internals are not known to me, so this is a risk to the part.

## Assembly

Tools: 5 mm hex key (short), 8 mm AF **thin open-end spanner with the jaw ground to 12 mm dia** (a standard spanner's
jaw collides with the cap at the boss-end hex; the model finds 4 to 7 free approach directions with the thin one),
torque screwdriver.

1. **Each cap on the bench.** Screw the three rods into the cap's bosses (boss-end stud, thread locker, torque by
   spanner on the hex). Put the shims on the rod ends. Bolt the cap's three cells to its wedges. Either order of rods
   and cells is free of collisions.
2. **Close the halves along z.** The top half comes straight down; every part clears the bottom half to the final
   seat. Use a jig to hold the caps parallel and centred while you work.
3. **Zero each cell.** From the cell's underside, put the M6x14 free-end screw through the cell into the rod end.
   Read the cell with the screw finger tight. Choose shims so the unloaded reading is within 3 % of full scale, then
   tighten the screw **while holding the rod's cell-end hex with the thin spanner**.
4. **Calibrate** the 6x6 matrix on a rig (not designed).

The reason for this order: the six rods link the two caps along six different axes, and the finished structure is
rigid. A rod with a stud through each end cannot be inserted into the gap between fixed parts, which is why the cell
end is a threaded hole and the screw goes in last.

## Torques (starting values from closed-form checks; verify)

| Fastener | Value | Why |
|---|---|---|
| Boss-end stud | about 4 N.m | held by spanner on the hex |
| Free-end screw | about 3 N.m | holding the cell-end hex; the neck torsion limit is about 0.8 N.m, so never turn without holding |
| Cell bolt | about 9 N.m (about 7.5 kN) | limited by thread strip in 7075 at 5.9 mm engagement (16 kN). **With steel thread inserts: 14 N.m, full 12.9 preload** |

## What was checked on the solids (`checks.txt`)

- Interference between any two parts: none.
- Everything inside the 88.85 mm hexagon, inset 0.5 mm: yes.
- Every tapped hole stays inside the cap with at least 1 mm of wall to the outer face: yes (1.14 mm at the bosses).
- Smallest gap between parts that are not meant to touch: 4.9 mm (the cell's corner meets the wedge's knife edge at
  the cap plane, by design).
- Hex-key access to all 12 screws: clear.
- Spanner access to every rod hex with the thin spanner: 4 to 7 directions each.
- Closing along z, bench order, free-end screws: all free.

## What is NOT done

| Item | Consequence |
|---|---|
| **No FEA.** Caps, wedges and rods are hand-calculated only | stiffness, stress concentration at the necks and the 3 mm caps are unverified |
| Cell rating (75 kg), stiffness, whether a bolted cell shunts the load | Tests H, H2, H3; if the shunt fraction is under about 0.2 the rod design stays, if over 0.6 the rods may be unnecessary |
| **Fixed-end joint opens near 825 N**, below the 1045 N abuse load (3x sim peak, 142 % of cell rating) | no overload stops designed; a central pin stop is impractical at these stiffnesses |
| Thread engagement is about 1 x d at all three joint types | use steel thread inserts, or thicker caps |
| 3 mm caps: no foot or ankle interface, no mounting pattern | needs the ankle gimbal's geometry |
| Cables: 24 wires from six cells, amplifier and ADC (2 uV resolution) | no routing, no cable exits modelled |
| Calibration rig and procedure | |
| Rod necks: stress concentration (Kt) and fatigue life under real walking cycles | 22 % margin before Kt |
| Thin spanner | a custom 8 mm AF tool; I can model it |
| Cell holes reamed: effect on the gauge's response | unknown without the cell's internal geometry |

# Bench rigs for Tests H, H2, H3 and R: CAD

**Status: a modelled rig, not a proven one.** The parts fit (no overlaps, every joint seats, hole walls hold in the
wedge plates) but nothing is analysed or built. Plate and block sizes are my choices for a 75 kg load, not calculated;
catalogue parts (eyebolt, eye nut, linear bushing) are drawn at nominal size, so check the supplier's drawing.

Files are in `cad/bench_rigs/`; regenerate with `python3 hardware/ft_sensor/bench_rigs_build.py` (needs build123d, as
`cad_build.py` does). The cell, rod, screws, shim and rod length come from `design/ft_sensor.yaml` through
`cad_build.py`; everything else is bench tooling defined at the top of `bench_rigs_build.py`.

## One tower, five set-ups

All five tests hang from the same rigid tower: a 12 mm top plate and bottom plate on four Ø12 columns (160 mm clear
height), weights on an eyebolt that hangs below. Frame: mm, z up, x along the cell, origin on the underside of the top
plate. Swapping a few parts gives each test.

| STEP | Test | What is on the tower | Measure |
|---|---|---|---|
| `rig_H_cantilever.step` | H, on-axis | cell bolted at one hole to block A1 under the top plate; eyebolt in the other hole | output against load, linearity, thermal drift (`docs/P4_JOINT_TEST_RIG.md`) |
| `rig_H_offaxis.step` | H, off-axis | the same cell on the side face of the tall block A3, so gravity acts across the cell's width | cross-sensitivity |
| `rig_H2_guided.step` | H2 | cell between block A1 and a carriage that slides on two ground shafts, so the free end moves only along the bolt axis | guided sensitivity as a fraction of the H1 figure; guided stiffness |
| `rig_H3_vee.step` | H3 | two cells in an inverted V between an upper and a lower plate, bolt axes +-45 deg, holes 30 mm apart | f = reading per cell / (0.707 F) |
| `rig_R_strut.step` | R | one full sensor member: reamed cell, shim, flexure rod, carriage with the boss face, load on the rod axis | the six measurements in `BENCH_TEST_STRUT.md` |

In rig R the member is turned so that its rod axis is vertical: the cell's bolt axis and the rod then lie on the weight
line, and the 54.5 deg tilt of the real sensor is not reproduced (it only changes how the force splits between the
members, not what a single member does). It loads the member in tension only; the sensor sees both signs.

Dial indicator: Ø8 stem hole at x = 50 in the top plate, over a post on the carriage tab (rigs H2 and R). It reads the
carriage's travel against the top plate, so it includes the frame's own stretch.

## What to make and what to buy

Machined (6061 or 7075 plate, all 3-axis except the wedge plates; STEP in `cad/bench_rigs/parts/`):

| Part | Rigs | Note |
|---|---|---|
| top plate, bottom plate | all | Ø8 shaft sockets, M8 column studs, Ø8 dial stem, M6 clearance holes for the blocks and H3 plate |
| block A1 (30 x 30 x 15) | H, H2 | Ø6.4 through hole: the cell is screwed in from above through plate and block into the cell's own thread |
| block A2 (30 x 30 x 15) | R | blind M6 x 7.5 deep from below: the same fixed-end thread as the sensor's wedge |
| block A3 (30 x 30 x 50) | H off-axis | Ø6.4 hole through its width |
| carriage, two variants | H2, R | two Ø15 bores for the bushings; H2: Ø6.4 through; R: blind M6 7 deep (rod stud) and 12 deep (eyebolt) |
| H3 upper and lower plates | H3 | 15 mm plate, two integral 16 x 16 wedge pads parallel to the cell faces; 5-axis or 3+2 |
| rod | R | the sensor's rod, `rod.step` |

Bought: 4 columns Ø12 steel with M8 studs, two Ø8 h6 ground shafts 176 mm, two LM8LUU bushings, M6 SHCS 12.9 in 16,
20, 35 and 40 mm, an M6x65 all-thread, M6 eyebolts (DIN 580), one M6 eye nut (DIN 582), an M6x14 low-head SHCS and
shims for R, a dial indicator, and a weight hanger and mass set. The full list per rig is `parts_list.csv`.

## Checks (`checks.txt`)

No overlapping parts in any rig; every joint listed as seated is within 0.02 mm; the H3 clearance holes keep a 1 mm
wall through the wedges; the hanger passes through the bottom plate's Ø40 hole with the carriage at least 70 mm above it.

## Not modelled, and things to know

| Item | Consequence |
|---|---|
| Hanger and mass set | any pan or hooked-mass set; keep the load on the eyebolt's axis |
| Eyebolt and eye-nut rating | nominal M6; check the working load of the exact part. The cell is rated 75 kg: do not go past that without a bigger eyebolt |
| H off-axis: the eye is about 13 mm outboard of the cell face | the weight adds a twisting moment about the cell's long axis as well as the transverse load: note it, or use a hanger cross-pin |
| Frame compliance | the plates and columns are not analysed. Repeat H2 or R with a rigid spacer in place of the cell and subtract its (small) reading |
| Bushing friction in H2 and R | check that the zero returns after loading |
| R: sideways boss shims (test 4), ball-ended rod (test 3), side load (test 5), H3 sideways load | need extra parts or a pulley; not drawn |
| Cell wiring, amplifier and ADC | not drawn; see the foot-sensor note on the signal chain |

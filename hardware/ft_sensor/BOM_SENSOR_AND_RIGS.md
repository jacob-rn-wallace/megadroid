# Bill of materials: one F/T sensor and the five bench rigs

**Status: a stock plan, not a purchase order.** Quantities come from `cad/megadroid_ft_v0/` (the sensor) and
`cad/bench_rigs/parts_list.csv` (the rigs). Nothing is priced, and the stock sizes are my choice of standard sections
that cover the blanks. Check them against what your supplier actually sells. The full line list is
`BOM_SENSOR_AND_RIGS.csv`.

**Scope.** One sensor (one foot) plus the whole test program, which uses a single tower, so every rig part is bought
once. The last column of the CSV gives what each further sensor adds. The tests need cells of their own: see the cell
count below.

## How the stock is shared

- **One tower for five rigs.** Plates, columns, shafts, bushings, eyebolt and dial indicator are bought once.
  Tests H, H off-axis and H2 share block A1 and one cell. H3 uses two cells. The cell for R is one of the H3 pair,
  reamed after the native tests are done.
- **Two top/bottom plates from one length** of 12 x 125 flat bar.
- **One 60 x 50 bar** supplies the carriages, both H3 plates and all three blocks (cut list below).
- **Both caps from one 7075 blank**, laid side by side across a shared flat.
- **One 10 mm bar** gives all the rods, and the dial posts are cut from its offcut. I changed the posts from steel
  to 7075: they only carry the dial indicator's contact target.

## Raw stock

| Stock | Size to buy | Cut list | Parts' volume / stock volume |
|---|---|---|---|
| 6061 flat bar 12 x 125 | 305 mm | 2 x 150 mm: top plate, bottom plate | 89 % |
| 6061 flat bar 60 x 50 | 470 mm | carriage H2 87, carriage R 87, H3 lower plate 127, H3 upper plate 116, slice 33 for blocks | 32 % |
| 7075-T6 plate 15 mm | 105 x 182 mm | cap bottom and cap top, 102.6 x 88.85 each, across a shared flat | 18 % |
| 7075-T6 round 10 mm | 460 mm | 12 rods x 31 mm (27.8 + part-off), 2 dial posts (about 50 mm), 25 mm chuck grip | 22 % |
| Steel round 12 mm | 760 mm | 4 columns x 183 mm (160 + two 10 mm studs + part-off), 25 mm grip | 89 % |
| Ground shaft 8 mm h6 | 2 x 176 mm | 2 shafts | 100 % |
| Stainless shim, 0.25 mm | 14 x 160 mm | about 12 discs of 12 mm OD, 6.4 mm hole | |
| Stainless shim, 0.05 / 0.10 / 0.50 mm | 14 x 80 mm of each | zeroing shims | |

The slice of the 60 x 50 bar (33 mm long) splits down its width into two 30 x 50 x 30 pieces. One is block A3. The
other is cut across its 50 mm into A1 and A2 (15 mm each, 3 mm saw kerf between).

Total raw stock is about 7 kg. Aluminium utilisation is poor because the caps, carriages and H3 plates are mostly
chips, and you cannot buy less than the bounding blank. The caps are the worst case (18 %), a consequence of the
wedge geometry.

Rods: 12 are cut from the bar. That is 6 for the sensor, 3 for the neck-diameter series of Test R (2.2, 2.4, 2.6 mm)
and 3 spares. Nine is the minimum; the bar needs only 9 x 31 + the posts + grip = 360 mm.

## Cells (the count that matters most)

| Use | Cells |
|---|---|
| Sensor | 6 |
| Tests H, H off-axis, H2 and H3 | 2 (H3 needs two; one of them serves the single-cell tests) |
| Test R | 0 extra: reuse one of the H3 pair, reamed after the native tests |
| **Minimum** | **8** (two balance boards, no spare) |
| Recommended | 12 (three boards: covers a cell lost to the reaming) |

The earlier note that three boards make two sensors does not include the test cells. Two sensors plus the rigs need 14
cells, so four boards (16 cells, 2 spare).

Run the native-cell tests first. Reaming the holes (6.1 and 7.0 mm) is permanent, and the effect on the gauge is
unknown (see `BUILD_GUIDE.md`).

## Hardware and bought parts

| Item | Spec | Qty | Spares | Used in |
|---|---|---|---|---|
| Socket screw M6 x 16, 12.9 | cell bolt | 7 | 3 | sensor (6), R |
| Hardened washer 6.4 x 12 x 1.6 | under the cell bolt | 7 | 3 | sensor (6), R |
| Low-head socket screw M6 x 14 (DIN 6912), 10.9 or 12.9 | free-end screw | 7 | 3 | sensor (6), R |
| Socket screw M6 x 20, 12.9 | block and plate attachment | 4 | 2 | H, H off-axis, H2, R use 2; H3 uses 4 |
| Socket screw M6 x 35, 12.9 | cell screw | 1 | 1 | H, H2 |
| Socket screw M6 x 40, 12.9 | cell screws | 4 | 2 | H off-axis (1), H3 (4) |
| Stud M6 x 65, 10.9 or 12.9 | cut from all-thread | 1 | 1 | H2 |
| Set screw M4 x 6, cup point | shaft sockets, dial stem | 5 | 1 | tower |
| Eyebolt DIN 580 M6 | check the working load | 1 | 1 | H, H off-axis, H3, R in turn |
| Eye nut DIN 582 M6 | | 1 | 0 | H2 |
| Linear bushing LM8LUU | 8 / 15 / 45 | 2 | 0 | H2, R |
| Dial indicator | 0.001 mm, 8 mm stem | 1 | 0 | H2, R |
| Hanger and weights | to the cell rating | 1 set | | all |
| Thread locker | medium (243) | 1 | | sensor, R |

The 12.9 screws are bought with spares because a preloaded screw is not reused. The M4 set screws are implied by the
plate notes in `bench_rigs_build.py` but their tapped holes are not drawn.

## Tools

Reamers 6.1 and 7.0 mm; bottoming taps M6 and M8; a thin 8 mm AF spanner ground to a 12 mm jaw (made from a bought
thin wrench); a torque wrench or screwdriver for 2 to 10 N.m; 5 and 4 mm hex keys. Single-point threading for the
M6 x 1 rod studs on the lathe.

## Not in this BOM

Cell wiring, excitation, amplifier and ADC; the cable exits; the foot and ankle interface for the sensor
(`BUILD_GUIDE.md` lists it as open); and prices.

## Each further sensor adds

One 7075 cap blank (a spare is recommended), 240 mm of 10 mm bar for 7 rods, 6 cells, and 6 each of the M6 x 16
screw, washer and low-head screw, plus shims. No rig parts.

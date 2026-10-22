# Test R: single-strut bench test

Purpose: check the riskiest assumptions of the inversion-symmetric layout on ONE member before
machining six. Run Tests H, H2 and H3 first (`docs/P4_JOINT_TEST_RIG.md`); their result decides
whether the rod is needed at all.

Numbers below come from `SYNTHESIS_RESULTS.md` and the build candidate in `design/ft_sensor.yaml` (hand calculation,
7075-T6, 12.9 M6 bolts, 75 kg cell UNVERIFIED). Replace them with the rated values from Test H once known. The rig is
`cad/bench_rigs/rig_R_strut.step`; see `BENCH_RIGS.md`.

## What one member is

Cell (42.25 x 13.25 x 8.5 mm, holes reamed 6.1 / 7.0) bolted at one hole to a block, the other hole carrying the
sensor's turned rod through a 0.25 mm shim and the cell-end screw. The rod runs 21.55 mm (shim included) to a boss on
a guided carriage. In the rig the member is turned so its rod axis is vertical and the weights hang on that axis; the
54.5 deg tilt of the sensor is not reproduced.

## Parts to make

| Part | Process | Critical features |
|---|---|---|
| Rod, 7075-T6 | lathe | The sensor's rod (`rod.step`): two necks, d 2.4 x l 4.5 mm. Neck diameter +-0.02 mm, necks coaxial with the body to 0.02 mm, shoulders square. Make 6 (spares and a diameter series: 2.2, 2.4, 2.6 mm). |
| Block A2 | CNC | Blind M6 thread, 7.5 deep, square to the cell face. Hole position +-0.05 mm. |
| Carriage | CNC | Rod boss face (blind M6, 7 deep) square to the bushing bores. |
| Top plate, bottom plate | CNC | Flat and parallel to 0.02 mm; the shaft sockets must be coaxial with the bores (+-0.02 mm). |

## Measurements

1. **Axial stiffness of the strut.** Load the load plate along the rod axis in 50 N steps to 450 N (the sensing-range
   member load), then return. Record the cell output and the plate displacement (dial indicator, 1 um resolution).
   Expect the cell to dominate the compliance. Compare with the three-spring estimate in Test H2.
2. **Hysteresis and repeatability.** Ten cycles to 450 N. Target hysteresis under 0.1% of full scale. Zero return
   under 0.05%. A larger figure points at the stud joint or the bolt preload, not the neck.
3. **Parasitic stiffness of the necks.** Compare the cell reading for a given plate displacement against a version
   with a ball-ended or pinned rod. The difference is the load the necks carry in bending. It should be a few percent
   at most. If not, thin the necks.
4. **Misalignment sensitivity.** Shim the boss by 0.05, 0.10 and 0.20 mm sideways. Watch the cell offset and the neck
   with a strain gauge or a visible crack test after 10^4 cycles at 450 N. The sizing says 0.05 mm is acceptable and
   0.2 mm is not; this checks it.
5. **Side load.** Apply a load perpendicular to the rod at the load plate. The cell should read close to zero; its
   reading is the cross-talk of the pivot design.
6. **Joint opening.** Step the load past 450 N toward 900 N, looking for a step or drift in the reading. Predicted
   opening of the fixed-end joint: about 825 N. Stop at the first sign and record the load. Do not exceed the cell's
   150% safe overload.

## Pass criteria

| Check | Pass |
|---|---|
| Linearity of cell output against load | within 0.2% of full scale |
| Hysteresis | under 0.1% of full scale |
| Cross-talk to side load | under 3% |
| Joint opening load | above the 750 N stop reference |
| Neck after 10^4 cycles at range load | no visible crack, reading unchanged |

A failure in the joint opening check means the pad and bolt layout needs changing before building the sensor. A
failure in the hysteresis check points at the rod stud and washer, which are not yet designed.

## Notes

- Use the real cell mounting hardware (12.9 M6 screws at the stated preload, 60% of proof, 11.7 kN). Torque by
  the manufacturer's value for lubricated threads, and record it.
- Measure the bolt-face flatness of the cell: it is a stamped or milled part and its mounting surface may not be flat.
- The cell is a harvested part. Record which board it came from and its zero offset before loading.

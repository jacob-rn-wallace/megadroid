# Foot F/T sensor

A 6-axis force/torque sensor for the megadroid foot, built from harvested Wii
Fit Balance Board bar load cells (drop-in: MLC714N). Two of these per robot.

**Status:** a solid CAD model exists (`cad/megadroid_ft_v0/`), verified on the solids but not FEA'd, not built,
and with the cell's rating and stiffness unmeasured. Start with `BUILD_GUIDE.md`.

**Where things live**
- `design/sensors.yaml` (`end_of_limb_sensing.ft_sensor`) -- the interface the robot
  sees: envelope (hexagonal prism, 88.85 mm across flats, 57.544 mm tall), mounting,
  load path.
- `design/ft_sensor.yaml` -- cell dimensions, sources, and the study's assumptions.
- `layout_study.py` -- compares candidate layouts on paper (conditioning, per-cell load,
  noise-limited resolution).
- `design_synthesis.py`, `sizing.py` -- layout search and closed-form sizing. `SYNTHESIS_RESULTS.md` is the
  record, including what was found wrong along the way.
- `cad_build.py` -- builds the solid model and runs the checks. `BUILD_GUIDE.md` -- parts, machining, assembly.
- `BENCH_TEST_STRUT.md` -- the single-strut bench test (Test R); run it, and Tests H, H2, H3 in
  `docs/P4_JOINT_TEST_RIG.md`, before building all six members.

**Where it started.** `cad/stewart_concept_2026-10-20.step` is the original rough concept, a Fusion export by the
builder: six cells at 45 deg bolted at both ends to two plates, drawn to show the principle. It is kept as the
starting point and is **not the current design**. The layout search found no way to fit it in the envelope, and a
model of it suggested only a few percent of the load reaches the cells (the load-path question behind Tests H2 and
H3), which is why the current design puts a flexure rod in series with each cell. Where the other files say "the
builder's concept" or "your concept", this is the file.

**Intended to be extractable.** Nothing in this directory imports from the rest of
the repo except the two YAML files above. If it graduates to its own repository
(`git filter-repo --subdirectory-filter hardware/ft_sensor`, plus `design/ft_sensor.yaml`),
megadroid keeps only the interface block.

Licence: CERN-OHL-S for hardware, Apache-2.0 for software, as in the parent repository.

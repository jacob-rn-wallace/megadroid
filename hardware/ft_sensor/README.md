# Foot F/T sensor

A 6-axis force/torque sensor for the megadroid foot, built from harvested Wii
Fit Balance Board bar load cells (drop-in: MLC714N). Two of these per robot.

**Status:** layout study. No design chosen yet.

**Where things live**
- `design/sensors.yaml` (`end_of_limb_sensing.ft_sensor`) -- the interface the robot
  sees: envelope (hexagonal prism, 88.85 mm across flats, 57.544 mm tall), mounting,
  load path.
- `design/ft_sensor.yaml` -- cell dimensions, sources, and the study's assumptions.
- `layout_study.py` -- compares candidate layouts on paper (conditioning, per-cell load,
  noise-limited resolution).

**Intended to be extractable.** Nothing in this directory imports from the rest of
the repo except the two YAML files above. If it graduates to its own repository
(`git filter-repo --subdirectory-filter hardware/ft_sensor`, plus `design/ft_sensor.yaml`),
megadroid keeps only the interface block.

Licence: CERN-OHL-S for hardware, Apache-2.0 for software, as in the parent repository.

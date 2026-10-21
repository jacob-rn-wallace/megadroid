# Layout synthesis results (2026-10-21)

`design_synthesis.py`, both cases, on two envelopes. Margin and clearance 0.5 mm; design
wrench = 2.5 x the simulated peak (`design/ft_sensor.yaml`). A crude hill-climb over a
boxes-in-space model: "none found" is not proof of impossibility.

| | 88.85 mm hexagon | 107 mm cylinder |
|---|---|---|
| **Direct** (cells bolted to both caps, your concept) | no placement found (0 of 32 climbs) | only the concept's neighbourhood fits (7 of 32); misses resolution targets by 14x (solid-block cell) to 51x (thinned web) |
| **Rods** (cell + pivoted rod, ideal strut) | no placement found (0 of 72) | 6 of 72 climbs feasible; meets resolution; cells exceed rating at the design wrench |

## Direct, on the cylinder
tilt 43.5 deg, radius 41.3 mm, pair half-angle 30 deg, cell centre z 18.6 mm (of a 37.5 mm gap).
This is your concept. Signal fraction f = 0.081 (solid-block cell) or 0.021 (thinned web), from
an ESTIMATED three-spring stiffness model; Test H2/H3 measure it. Mx/My resolution 0.38-1.5 N.m
against a 0.05 N.m target; Fz 13-51 N against 1 N. About 460-500 N axial load in each cell at the
design wrench, carried by the cell's stiff direction (unmeasured; check the M6 screws and the cell
body for it).

## Rods, on the cylinder (best of 6 feasible)
| | seed 2 | seed 3 | first run |
|---|---|---|---|
| bolt-axis tilt from vertical | 33.0 deg | 33.6 deg | 33.6 deg |
| cell-centre radius | 31.6 mm | 32.1 mm | 30.8 mm |
| pair half-angle | 43.5 deg | 38.6 deg | 44.4 deg |
| cell centre height | 15.1 mm | 15.5 mm | 15.3 mm |
| rod length | 12.7 mm | 12.3 mm | 12.5 mm |
| Mx/My resolution, p-p | 0.040 N.m | 0.014 N.m | 0.039 N.m |
| Fz / shear resolution | 1.21 / 0.56 N | 1.20 / 0.57 N | 1.20 / 0.57 N |
| worst cell at design wrench | 277% of rating | 459% | 401% |
| condition number | 10.3 | 12.9 | 15.5 |

Every run lands on the same family: tilt about 33 deg, radius about 32 mm, cell length along the
slope, rods at the 12 mm minimum I allowed. Resolution is fine; the cells are the limit. At the
simulated peak (design wrench / 2.5) the worst cell is 110-180% of a 75 kg rating, past rated
and inside the 150% safe overload for the best. The MLC714N listings also offer 150 and 250 kg
variants (unverified): 150 kg would cure the overload and roughly double the resolution figures,
so Mx/My would sit near the 0.05 N.m target.

## What limits it
Rod length sits at the minimum in every rods result: the 37.5 mm gap between 10 mm caps is the
constraint. A 45 deg cell (42.25 mm long) alone spans about 30 mm of it. Thinner caps would lengthen
the rods; that is not modelled.

## Not done
Cap bodies, bolt and flexure sizing, stress, CAD. The hexagon result depends on the 88.85 mm
figure, which your own CAD caps exceed (94.8-99.6 mm); which number is the requirement is
unanswered.

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

---

# Update: sizing, thinner caps, and a layout that fits the spec hexagon (2026-10-21)

`sizing.py` (closed-form bolts, flexure necks, pads; hand calculation, not FEA) changed two
assumptions, and with them the answer.

1. **The rod must be longer than the 12 mm I allowed.** At 0.05 mm of assembly misalignment the neck
   that survives needs a rod span of about 15-18 mm (7075 neck d 2-4 mm x l 3.5-7 mm). Short rods are very
   sensitive to misalignment: at 0.2 mm no neck passes at any size.
2. **Load cases were muddled.** The stacked 2.5x design load and 2x abuse factor made a 75 kg cell look
   hopeless. Now: the cell must stay inside its rating at 1.5x the simulated peak (sensing range); the
   rods, bolts and caps must not yield at 3x (a fall), with overload stops to keep the cell safe.

With rods at an 18 mm span, the gap between the caps has to grow, so the caps must be thinner (6 mm,
gap 45.5 mm, instead of 10 mm). Then it works, and with the 75 kg cell.

## Candidate: 75 kg cell, 88.85 mm hexagon, 6 mm caps (rods case)
Three independent searches converge on it (score 0.86, below 1.0 = every target met):

| | |
|---|---|
| Cell-centre radius | 34.2 mm |
| Pair half-angle | 30.7 deg (pairs centred 120 deg apart, cluster rotation 35 deg) |
| Bolt-axis tilt from vertical | 57.1 deg (mostly tangential: azimuth -91.5 deg from the radial) |
| Cell centre height | 20.1 mm above the bottom cap's inner face (gap 45.5 mm) |
| Cell length | along the slope |
| Rod span | 18-19.5 mm (cell face to far cap) |
| Condition number | 2.5-2.6 |
| Worst cell at the sensing-range load | 74-79% of rating |
| Resolution p-p | Mx/My 0.022-0.024 N.m, Mz 0.042 N.m, Fz 0.77 N, shear 0.86 N |

Packing is extremely tight: cell-to-cell clearance 0.50 mm, cell-to-wall 0.72 mm. Any real tolerance
eats into that.

On the 107 mm cylinder the same thing works with more room (tilt 54.7 deg, radius 38.2 mm,
cond 2.6, worst cell 70%). The 150 kg cell gives no advantage (53% of rating, but twice the noise).

## Sizing results for the candidate (7075-T6, M6 A2-70)
- **Member force**, worst of six: 544 N at the sensing-range load (74% of rating); 363 N at the simulated
  peak; 1088 N at the 3x abuse load (148% of rating, so the cell needs overload stops).
- **Flexure rod** (turned, two thin necks): 7075 neck d 2.2 mm x l 3.5 mm gives a minimum rod span of
  13 mm (axial 143 MPa, misalignment bending 58 MPa, alternating 106 MPa, buckling capacity 2.4 kN, neck
  axial stiffness 78 MN/m each). 6061 needs d 2.8 x l 7.0 (span 20 mm). 17-4PH steel passes at d 2.0 x l 2.5
  but with 203 MPa misalignment bending, so it is tolerance-sensitive.
- **M6 bolts:** preload 5.4 kN (60% of proof), 297 MPa; thread strip capacity 25 kN against 0.5 kN.
- **Pad-to-cap bolts** at the abuse load: 2.1 kN each against 5.4 kN preload.
- **Pad bending:** 46 MPa against 335 MPa allowable.
- **Fixed-end joint:** holds at the sensing-range load (4.6 kN preload needed against 5.4 kN) but opens above about
  636 N of member force. That is below the 736 N rating and below the 1088 N abuse load.

## What is NOT settled
- **The cell's fixed-end joint** is the weakest link and rests on an UNVERIFIED contact length (21 mm). The
  stops must hold the cell below ~600 N, not at its 150% overload. Higher-grade bolts (12.9) would raise the
  limit. How the cell is meant to be mounted, and what it is rated for under that mount, is a Test H question.
- **Overload stops** are required (abuse load is 148% of rating) and are not designed.
- **Cap plate and pad machining.** 6 mm caps are not checked for plate bending under the pad loads. Thread
  engagement of 9 mm is not possible in a 6 mm plate (use through bolts and nuts, or inserts). Six tilted pads
  integral with a cap would be exactly the complex machining you want to avoid; bolted wedge blocks on flat
  plates are the alternative.
- **Rods are tolerance-critical.** The necks pass at 0.05 mm misalignment and not at 0.2 mm.
- **Cell stiffness, rating and noise** are unmeasured (Test H, H2, H3).

---

# Update: joint, stops and structure (2026-10-21)

## The fixed-end joint decides the margin
How long the clamp footprint under the cell's fixed end is, and how hard the bolt clamps it, set the load
at which that joint opens (M = F x 30 mm reacted by contact pressure over the pad).
| | contact length | bolt | joint opens above |
|---|---|---|---|
| first model | 21.1 mm (the cell's whole fixed half) | A2-70 | ~636 N |
| 12.9 bolts | 21.1 mm | 12.9 (11.7 kN preload) | ~1370 N, **but** a 21 mm pad collides with its neighbours and leaves the hexagon: no layout found |
| **adopted** | 12.7 mm | **12.9** | **~826 N** |

So the bolts are alloy steel 12.9 (not stainless A2-70), the pad is 12.7 mm, and the joint opens at 2.3x the
simulated peak. All unverified until Test H shows how the cell is meant to be mounted.

## Loads against what holds (candidate layout, worst of six members)
| x simulated peak | member force | % of 75 kg rating | joint (826 N) | cell |
|---|---|---|---|---|
| 1.0 | 363 N | 49% | holds | fine |
| 1.5 (sensing range) | 544 N | 74% | holds | fine |
| 2.0 | 725 N | 99% | holds | fine |
| 2.3 | 834 N | 113% | opens | within the 150% safe overload |
| 3.0 (abuse) | 1088 N | 148% | opens | within the 150% safe overload (1104 N) |
| 4.0 | 1451 N | 197% | opens | beyond safe overload |
| 5.0 | 1813 N | 246% | opens | beyond ultimate overload |

## Overload stops: a central pin-in-bore is not practical
A central pin and bore (lathe and drill only) was the plan. Sized from the platform stiffness (cell bending stiffness
5-21 MN/m, estimated), the clearances that keep every member under 750 N under any combination of loads are:
radial 4-14 um, axial 12-38 um, key-slot twist about 2 um at a 10 mm radius. That cannot be machined or set.
This is a result of the stiffness, not of the stop design: the sensor barely moves before the cells reach their limit.

What this means: no stop is needed for the first prototype for loads up to about 3x the simulated peak, because
the cell's own safe overload (150%) covers them. For a harder fall (4-5x), use member-level stops: a shim-set shoulder
under each cell's free end, with a gap equal to the stop force divided by the cell stiffness (about 35-150 um).
Whether to build them depends on Test H (cell stiffness, joint behaviour).

## Structure (candidate, 6 mm plates, wedge blocks)
- **Wedge blocks:** 12.7 x 13.2 mm footprint, about 10 mm long along the bolt axis. Twelve in all (six under the cells,
  six at the rod ends); two designs, each its mirror. Cut from stock at the tilt angle; tapped hole perpendicular to the
  sloped face.
- **Block to plate:** two M6 12.9 flat-head bolts through the plate. Tension 1.3 kN each at the abuse load against 11.7 kN preload;
  shear 913 N against friction capacity 3.5 kN.
- **Plate:** 6 mm 7075, flat, drilled and countersunk only. Local point stiffness about 38 MN/m (simply supported, 44 mm
  radius), only 1.8-7.5x stiffer than the cell: thicken locally, or use 8 mm plates if the gap allows (it costs rod length).
  Global bending at 5x the simulated peak moment: 139 MPa against a 335 MPa allowable.
- **Centre:** a 12 mm radius column is left clear (cables, mounting); the layout respects it.

## Still open
Cell rating, stiffness and mounting (Test H); rod machining tolerance (0.05 mm assumed); the rod stud and washer
details; the cap interface to the ankle gimbal and the foot plate; no FEA; and whether you have lathe access for the
turned rods (the alternative, off-the-shelf ball joints, has play).

---

# Update: a layout that looks the same either end up (2026-10-21)

The question: can the sensor be symmetric about a horizontal axis, three cells flipped so their rods run
the other way, so that it looks the same whichever end is up?

## Exact half-turn symmetry costs a measurement axis
A layout that a half turn about a horizontal axis maps onto itself always has rank 5: it cannot measure all six
force and torque components. Checked numerically on random layouts (200 each, three axes) and on the
three-fold + half-turn layout (rank 5 every time). The reason is representation counting: under a half turn the
six strut forces split into three symmetric and three antisymmetric combinations, but the six wrench components
split into two symmetric and four antisymmetric, so one antisymmetric component has no strut to read it.
No tuning of angles or positions can change that. The search agreed: the geometrically valid half-turn layouts
it found were all rank 5.

## Inversion symmetry keeps rank 6
Inversion through the centre (the point mirror: every cell has a partner on the opposite plate, on the far side of
the ring) keeps rank 6 and matches the "three cells flipped" picture: the cells alternate between plates around
the ring. Turned over, the sensor looks like its own mirror image, not an exact copy; that is the closest symmetry
that still measures all six axes. Both plates are the same flat part.

`design_synthesis.py --symmetry inv` finds it easily: 20-22 of 40 searches feasible (the mirrored-pair layout
managed 5-7), on both the hexagon and the cylinder, all meeting every target.

## Candidate: inversion layout, 88.85 mm hexagon, 75 kg cell, 6 mm caps
| | |
|---|---|
| Bolt-axis tilt from vertical | 54.7 deg (the cube-corner angle: every search landed on it) |
| Cell-centre radius | 28.7 mm |
| Cells | three on each plate, alternating around the ring |
| Cell heights | 20.0 mm above the bottom plate (bottom cells), 25.5 mm (top cells) |
| Rod span | 18.8 mm |
| Condition number | 1.6 (rank 6) |
| Worst cell at the sensing-range load | 62% of rating (458 N) |
| Resolution p-p | Mx/My 0.027 N.m, Mz 0.028 N.m, Fz 0.83 N, shear 0.83 N |
| Clearances | cell to wall 4.6 mm, tightest pair of parts 2.2 mm (0.5 mm in the mirrored-pair layout) |
| Nearest part to the axis | 17.1 mm (keep-out 12 mm) |

At 3x the simulated peak the worst member sees 916 N (124% of rating, inside the 150% safe overload). The fixed-end
joint opens above about 825 N, 2.7x the simulated peak. Rod, bolt and pad sizing results are unchanged in
kind (7075 neck d 2.1 x l 3.5 mm; M6 12.9 bolts).

Not yet re-run for this layout: the stop and wedge-block structure sizing. Not checked: how identical plates mate
with mirrored wedge-block lean (a wedge block is its own mirror image, so six cell blocks and six rod blocks
are each one part).

## Correction and detailing (2026-10-21): the CAD model, and what it changed

Building the layout as solid CAD (`cad_build.py`, `BUILD_GUIDE.md`) found problems that the box model in
`design_synthesis.py` could not see. **The inversion candidate published above (r 28.7, 6 mm caps, free rod 18.8 mm) is
superseded.**

| Found | Consequence |
|---|---|
| The far-end boss (about 8 mm deep along the rod at 54.7 deg) was counted as rod length | free rod was 10.8 mm, not 18.8; sizing needs at least 13 mm. Fixed in the search |
| The rod was modelled as a 6 mm bar; the real ends are 9.24 mm hex blocks | rods touched neighbouring wedges. The search now models hex ends, screw heads and key-access volumes |
| The two caps are **not** the same part flipped (symmetric difference 25 % of volume) | they are mirror images: two machined parts. Earlier text saying "identical plates" was wrong |
| A neck of 2.1 x 3.5 mm has no fatigue margin (107 against 106 MPa), and fatigue scales as d/l | neck is 2.4 x 4.5 mm (22 % margin) |
| A rod with studs through both ends cannot be assembled into a rigid six-rod loop | cell end is a threaded hole in the rod, with a screw from the underside; the halves close along z |
| 3 mm caps leave 1.1 to 1.2 mm of wall under the tilted tapped holes | holes are 7.5 mm (fixed end) and 7.0 mm (boss) deep; thread engagement about 1 x d |

Current layout (`design/ft_sensor.yaml`, `build_candidate`): 3 mm caps, free rod 21.5 mm (cell face to boss face),
bolt axis 54.5 deg from the cap normal, r 34.4 mm. Rank 6, condition number 2.1, worst cell 71 % of rating at the
sensing-range load, resolution Mx/My 0.020 N.m, Fz 0.84 N. A 4 mm cap layout also exists but needs a 38 mm rod and
loads the worst cell to 82 %.

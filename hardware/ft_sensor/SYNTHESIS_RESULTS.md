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

<!--
name: P3_MODEL_FIDELITY.md
type: validation-report
description: Stage P3 finding — simulation fidelity, not control theory, was the blocker
-->
# Stage P3 — Model Fidelity Findings

**Status:** In progress — significant finding, direction change recommended
**Date:** 2026-09-10

This document records the central finding of Stage P3 simulation work: **the
obstacle to validated walking was simulation fidelity, not control theory.**
Detailed chronology lives in `tools/CLAUDE.md`; this is the summary and its
consequences for the development plan.

---

## The finding

Over an extended investigation, eleven structurally different control
mechanisms were built and tested against lateral push recovery. All eleven
failed. The conclusion drawn at the time was that the gait architecture had
reached a real limit.

That conclusion was wrong. Four separate defects in the simulation model were
subsequently found, each invisible until specifically measured, and each
capable on its own of invalidating the results built on top of it.

| # | Defect | What it meant | How it was found |
|---|---|---|---|
| 1 | **No actuator torque limits.** `generate_mjcf.py` emitted position actuators with no `forcerange`. | Every result described a robot with unbounded joint torque. | Reviewing candidate control architectures against the design |
| 2 | **Controllers consumed privileged state.** All read `data.subtree_com` / `subtree_linvel` — MuJoCo's true CoM position and velocity. | The feedback signal every mechanism was tuned against is not measurable on the real robot. `design/sensors.yaml` had no IMU. | Auditing what the specified sensor suite can actually observe |
| 3 | **Gains went stale after a design change.** `dca7009` shifted z_c/Tc after the control constants were tuned; nothing re-ran the sims. | Three files' documented performance claims were false; two of five documented validation commands failed outright. | Running every documented milestone command against the current model |
| 4 | **Rigid foot contact.** `BOM.csv` specifies a compliant sole pad; the MJCF modelled a rigid box on a rigid plane. | Ground reaction force was discontinuous tick-to-tick, producing chaotic sensitivity — 10 ms of push timing flipping a clean recovery into a 90° fall. | Asking why results resembled coin flips when 1990s hardware walked reliably |

Defect 4 is the most consequential. Correcting it moved 10 N lateral push
survival from **0/12 to 9/12 timings** — after eleven control mechanisms had
failed to move it at all. The *character* of the response changed too: rigid
contact produced alternating survive/fall between adjacent 10 ms samples,
while compliant contact produces contiguous blocks, so survival became a
smooth function of gait phase rather than a coin toss.

---

## Consequence: earlier results are provisional

Every push-recovery result recorded before 2026-09-10 was measured against a
contact model that flips outcomes on a 10 ms perturbation. Those verdicts are
not necessarily wrong, but they are **not trustworthy**, and at least one
false positive was caught in the act (capture-region footstep placement looked
like a clear win at a single push timing and evaporated across seven).

Status after re-testing against compliant contact (2026-09-10):
- CoP-repulsion "exhausted" verdict — **CONFIRMED.** Best nonzero gain differs
  from baseline by at most one cell in twelve, in both directions. Ships inert.
- Capture-region footstep placement "exhausted" verdict — **CONFIRMED, more
  strongly.** Both region sizes now break nominal walking at n=16 and are equal
  or worse on every push magnitude. Its one redeeming feature under rigid
  contact (mildly better nominal walking) was itself an artifact and reverses.
- The push-magnitude ceilings quoted throughout `tools/CLAUDE.md` — **still
  provisional**, not re-measured mechanism by mechanism. What is known is that
  the compliant baseline is far stronger than any pre-2026-09-10 entry records
  (12/12 timings at 5 N, 9/12 at 10 N, versus 0/12 at 10 N rigid), so those
  ceilings understate the current controller rather than overstating it.

Rigid contact misled in **both** directions — toward false positives (the
capture-region single-timing "win") and false negatives (the baseline's true
push performance). Neither error type should be assumed absent elsewhere.

**Methodology rules adopted as a result:**
1. A single-timing push result is not evidence. Sweep timings and report survival counts.
2. Only compare push survival at step counts where every configuration under test walks *unpushed*.
3. Treat any performance figure in a docstring as "true when written" unless it postdates the last design change.

---

## What is genuinely validated

| Check | Status |
|---|---|
| `sim_static_pose.py` | PASS |
| `sim_zmp_balance.py` | PASS |
| `sim_walk_recede.py --selftest` | PASS |
| Receding-horizon nominal walking | Clean to n=16, marginal at n=20 |
| `sim_walk_lipm.py` | Fails — separate documented cause |
| `sim_walk_gait.py` | Fails — separate documented cause (roll runaway) |

Two genuine control gains were made and kept: a per-axis DCM gain
(`K_DCM_Y_SCALE`, the lateral axis was over-swinging to 215% of planned
amplitude) which doubled the clean walking range from 8 to 16 steps, and an
EMA-based `b_nom_y` estimate.

---

## Recommended direction: build the measurements, not the robot

Every remaining blocker is a **physical parameter that simulation cannot
resolve.** All are currently guesses:

| Parameter | Status | Consequence if wrong |
|---|---|---|
| 775 stall torque | PLACEHOLDER | sizes the entire drivetrain |
| Drivetrain efficiency (0.70) | PLACEHOLDER | ditto |
| Belt reduction ratio | inferred (4:1), never engineered | 20:1 vs 80:1 is a 4× swing in available torque |
| **Sole pad compliance** | PLACEHOLDER | **dominates disturbance behaviour (0/12 → 9/12)** |
| Link masses | nominal, "pending verification during P6" | already invalidated a full set of control tunings once |
| IMU noise and bias | not modelled at all | estimator tuning currently rewards settings that would diverge on hardware |

Four times in one session, a guessed parameter produced a confident and wrong
conclusion. No further simulation resolves any row in this table.

**Proposal: a single-joint test rig.** One 775 motor, one 20:1 gearbox, one
belt stage, an encoder and a load cell. That fixture measures the real
torque–speed curve, backlash, belt compliance, thermal behaviour under duty
cycle, and position-servo response — five of the six rows above. A sole-pad
drop test covers the sixth.

**Scoped in `docs/P4_JOINT_TEST_RIG.md`** — fixture bill of materials, seven
tests (A–G), which YAML field each one replaces, what result would reopen a
design decision, and a priority order if only some get done.

This is not a departure from the process. `PROCESS.md` Stage P4 requires
"define actuator performance envelopes" and "validate gear ratios against
simulated loads". A V-model pairs each design stage with a verification
activity; a bench rig verifying the actuator envelope **is** the validation
leg of P4, not a jump ahead to P6.

**Sequence recommended:**
1. Re-test the provisional verdicts against compliant contact (hours of compute).
2. Build the single-joint rig in parallel; replace placeholders with measurements.
3. Resume control work on a model that can predict — which it has never yet been able to do.

---

## Note on the original question

This investigation began from a reasonable doubt: classical ZMP/DCM control
was chosen deliberately over learning-based methods because Honda and Waseda
fielded reliably-walking bipeds with these techniques in the mid-1990s, so why
was it proving so difficult here?

The techniques were not the problem. Honda and Waseda tuned control against
real physics on hardware they could instrument. This project has been tuning
against a model wrong in at least four ways, each discovered only by
measurement. The difficulty has been model fidelity, and that is a solvable
and largely mechanical problem rather than a control-theoretic one.

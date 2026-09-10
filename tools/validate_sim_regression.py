#!/usr/bin/env python3
"""
Simulation regression gate for Megadroid.

Runs the P3 simulation checks that must stay green, so a design change
cannot silently invalidate them.

WHY THIS EXISTS
---------------
This closes a process gap that cost this project real work three times.
`preflight.py` and both CI workflows ran only the cheap validators and the
rehydration diff -- no MuJoCo at all -- so a `design/*.yaml` edit could
invalidate every walking-gait claim in the repo, pass all gates, and merge
clean. It did exactly that: `dca7009` shifted z_c/Tc after the control gains
were tuned, nothing re-ran the sims, and three files' documented performance
claims went stale. They were then used as trusted baselines by later
investigations, one of which measured a mechanism against a baseline that had
already degraded underneath it. A fourth instance surfaced later still --
`sim_walk_lipm.py`'s own selftest had been asserting a 12-step run stayed
under 20 degrees long after n=12 became a 90-degree fall, and nothing ran it.

See `tools/CLAUDE.md` (the P3 milestone audit entry) and
`docs/P3_MODEL_FIDELITY.md` for the full account.

WHAT IT GATES
-------------
Each check below owns its own pass criteria and signals failure through its
exit code; this script is a runner, not a second opinion. Total runtime is a
few seconds -- these are short deterministic runs, not the multi-config
sweeps that make simulation work slow.

`sim_walk_gait.py` is deliberately NOT gated. It fails at 2 of 3 steps for a
documented, independently re-confirmed reason (roll runaway, unaffected by
the contact-model correction), and it is explicitly superseded by
`sim_walk_lipm.py` for the smooth-walking goal. Gating it would land the gate
permanently red and train everyone to ignore it. If it is ever fixed, add it
here in the same commit.

INTERPRETER
-----------
These checks need `mujoco`, which the rehydrators and validators do not. The
script runs them under the first interpreter that can import it: the one
invoking this script, then $MEGADROID_SIM_PYTHON, then a conventional
sibling venv. If none can, it SKIPS rather than fails -- so `preflight.py`
stays usable on a system Python -- unless `--require` is passed, which CI
does, making a missing dependency a hard error there.

Usage:
    python3 tools/validate_sim_regression.py
    python3 tools/validate_sim_regression.py --require   # CI: never skip

Exit codes:
    0  All gated checks passed (or skipped without --require)
    1  A gated check failed, or --require was passed and mujoco is missing
"""

import argparse
import os
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

# (script + args, human label). Ordered cheapest-first so an obviously broken
# model fails in a fraction of a second rather than after the walking runs.
CHECKS = [
    (["tools/sim_load_test.py"], "model loads"),
    (["tools/sim_static_pose.py"], "static pose (fixed base)"),
    (["tools/sim_zmp_balance.py"], "ZMP balance (floating base)"),
    (["tools/sim_walk_lipm.py", "--selftest"], "LIPM/DCM walking selftest"),
    (["tools/sim_walk_recede.py", "--selftest"], "receding-horizon walking selftest"),
]


def _can_import_mujoco(python: str) -> bool:
    try:
        return subprocess.run(
            [python, "-c", "import mujoco"],
            capture_output=True, timeout=60,
        ).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False


def find_sim_python():
    """First interpreter that can import mujoco, or None."""
    candidates = [sys.executable]
    env_python = os.environ.get("MEGADROID_SIM_PYTHON")
    if env_python:
        candidates.append(env_python)
    # Conventional sibling venv, e.g. <repo>/../megadroid-venv/bin/python3
    candidates.append(str(REPO_ROOT.parent / "megadroid-venv" / "bin" / "python3"))

    for candidate in candidates:
        if candidate and Path(candidate).exists() and _can_import_mujoco(candidate):
            return candidate
    return None


def run_check(python: str, args: list, label: str) -> bool:
    result = subprocess.run(
        [python, str(REPO_ROOT / args[0]), *args[1:]],
        cwd=REPO_ROOT, capture_output=True, text=True,
    )
    if result.returncode != 0:
        print(f"  ✗ FAILED: {label}")
        for stream in (result.stdout, result.stderr):
            if stream:
                print(stream.rstrip())
        return False
    print(f"  ✓ {label}")
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--require", action="store_true",
                        help="treat a missing mujoco as a failure instead of a skip")
    args = parser.parse_args()

    python = find_sim_python()
    if python is None:
        message = (
            "mujoco is not importable by any candidate interpreter "
            "(this one, $MEGADROID_SIM_PYTHON, or a sibling megadroid-venv)"
        )
        if args.require:
            print(f"✗ {message}")
            print("  --require was passed, so this is a failure. Install with: pip install mujoco")
            sys.exit(1)
        print(f"  ⊘ SKIPPED — {message}")
        print("    Install mujoco, or set MEGADROID_SIM_PYTHON, to gate these checks.")
        return

    if python != sys.executable:
        print(f"  (running simulation checks under {python})")

    if not all(run_check(python, a, label) for a, label in CHECKS):
        print("\n✗ Simulation regression gate FAILED.")
        print("  A design or tooling change has invalidated a P3 milestone.")
        print("  Re-tune and re-measure before committing — do not update the")
        print("  documented numbers to match without understanding what moved.")
        sys.exit(1)

    print("\n✓ Simulation regression gate passed.")


if __name__ == "__main__":
    main()

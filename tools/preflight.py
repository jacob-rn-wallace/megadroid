#!/usr/bin/env python3
"""
Pre-commit preflight check for Megadroid.

Runs the full rehydrate → validate sequence and reports what changed.
Run this before every commit to derived docs or design YAML.

Usage:
    python3 tools/preflight.py

Exit codes:
    0  All checks passed; shows git status of changed files
    1  Rehydration or validation failed
"""

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

REHYDRATORS = [
    "tools/rehydrate_spec.py",
    "tools/rehydrate_mech.py",
    "tools/rehydrate_readme_structure.py",
]

VALIDATORS = [
    "tools/validate_geometry.py",
    "tools/validate_no_geometry_literals.py",
    "tools/validate_dof_consistency.py",
]


def run(script: str, label: str) -> bool:
    result = subprocess.run(
        ["python3", str(REPO_ROOT / script)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(f"  ✗ FAILED: {label}")
        if result.stdout:
            print(result.stdout.rstrip())
        if result.stderr:
            print(result.stderr.rstrip())
        return False
    print(f"  ✓ {label}")
    return True


def git_status() -> str:
    result = subprocess.run(
        ["git", "status", "--short"],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
    )
    return result.stdout.strip()


def main():
    print("=" * 60)
    print("Megadroid preflight check")
    print("=" * 60)

    print("\n[1/2] Rehydrating derived documents...")
    rehydrate_ok = all(run(s, s.split("/")[-1]) for s in REHYDRATORS)
    if not rehydrate_ok:
        print("\n✗ Rehydration failed — fix before committing.")
        sys.exit(1)

    print("\n[2/2] Running validators...")
    validate_ok = all(run(s, s.split("/")[-1]) for s in VALIDATORS)
    if not validate_ok:
        print("\n✗ Validation failed — fix design/*.yaml and rehydrate.")
        sys.exit(1)

    print("\n" + "=" * 60)
    status = git_status()
    if status:
        print("Modified files (ready to stage):")
        for line in status.splitlines():
            print(f"  {line}")
        print()
        print("Suggested commit sequence:")
        print("  # If design/*.yaml changed:")
        print("  git add design/*.yaml")
        print("  git commit -m 'Design: <description>'")
        print()
        print("  # Then for derived docs:")
        print("  git add SPEC.md MECH.md README.md")
        print("  git commit -m 'Docs: rehydrate from updated sources'")
        print()
        print("  # If URDF changed:")
        print("  git add simulation/urdf/megadroid_mvs.urdf")
        print("  git commit -m 'URDF: regenerate from updated geometry'")
    else:
        print("Working tree is clean — nothing to commit.")

    print("\n✓ Preflight passed.")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

import sys
import yaml
from pathlib import Path

EXPECTED_MVS_DOF = 9

LEG_LOCATIONS = {"hip", "knee", "ankle"}

def load_joints(path: Path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    if "joints" not in data:
        raise ValueError("joints.yaml missing top-level 'joints' key")
    return data["joints"]

def count_mvs_dof(joints: dict) -> int:
    total = 0

    for name, joint in joints.items():
        # Must be actuated at all
        if not joint.get("actuated", False):
            continue

        variants = joint.get("variants", {})
        if not variants.get("MVS", False):
            continue

        location = joint.get("location", "").lower()

        if location in LEG_LOCATIONS:
            total += 2  # left + right leg
        else:
            total += 1

    return total

def main():
    joints_path = Path("design/joints.yaml")
    if not joints_path.exists():
        print("ERROR: design/joints.yaml not found", file=sys.stderr)
        sys.exit(1)

    joints = load_joints(joints_path)
    dof = count_mvs_dof(joints)

    print(f"MVS actuated DOF (from joints.yaml): {dof}")

    if dof != EXPECTED_MVS_DOF:
        print(
            f"ERROR: Expected {EXPECTED_MVS_DOF} DOF, got {dof}",
            file=sys.stderr
        )
        sys.exit(1)

    print("DOF consistency check passed.")

if __name__ == "__main__":
    main()
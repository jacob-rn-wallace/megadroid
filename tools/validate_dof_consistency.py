#!/usr/bin/env python3

import sys
import yaml
from pathlib import Path

LEG_LOCATIONS = {"hip", "knee", "ankle"}

def load_joints(path: Path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    if "joints" not in data:
        raise ValueError("joints.yaml missing top-level 'joints' key")
    return data

def count_mvs_dof(joints: dict) -> int:
    total = 0
    for name, joint in joints.items():
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

REPO_ROOT = Path(__file__).resolve().parent.parent


def main():
    joints_path = REPO_ROOT / "design" / "joints.yaml"
    if not joints_path.exists():
        print("ERROR: design/joints.yaml not found", file=sys.stderr)
        sys.exit(1)

    data = load_joints(joints_path)

    # Read declared expected total from YAML meta block
    meta = data.get("meta", {})
    if "mvs_dof_total" not in meta:
        print("ERROR: design/joints.yaml missing meta.mvs_dof_total declaration", file=sys.stderr)
        sys.exit(1)

    expected = meta["mvs_dof_total"]
    joints = data["joints"]
    counted = count_mvs_dof(joints)

    print(f"MVS actuated DOF (from joints.yaml): {counted}")
    print(f"MVS actuated DOF declared (meta.mvs_dof_total): {expected}")

    if counted != expected:
        print(
            f"ERROR: Counted {counted} DOF but meta.mvs_dof_total declares {expected}. "
            f"Update meta.mvs_dof_total or fix joint definitions.",
            file=sys.stderr
        )
        sys.exit(1)

    print("DOF consistency check passed.")

if __name__ == "__main__":
    main()
#!/usr/bin/env python3

"""
Validate that the number of actuated degrees of freedom (DOF) in the
authoritative joints.yaml file matches the expected MVS DOF count.

Rules:
- A joint contributes DOF if:
    - joint.actuated == true
    - joint.variants.MVS == true
- Joint instances are counted as:
    - location == 'hip' or 'knee'  -> 2 (left + right)
    - location == 'torso'          -> 1
"""

import sys
import yaml

EXPECTED_MVS_DOF = 9
JOINTS_FILE = "design/joints.yaml"


def joint_instance_count(joint):
    """
    Return how many physical instances this joint represents.
    """
    location = joint.get("location", "")

    if location in ("hip", "knee"):
        return 2  # left + right
    elif location == "torso":
        return 1
    else:
        # ankles, arms, etc. (even if present)
        return 0


def count_mvs_actuated_dof(joints):
    total = 0

    for name, joint in joints.items():
        if not isinstance(joint, dict):
            continue

        if not joint.get("actuated", False):
            continue

        if not joint.get("variants", {}).get("MVS", False):
            continue

        instances = joint_instance_count(joint)
        total += instances

    return total


def main():
    try:
        with open(JOINTS_FILE, "r") as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        print(f"ERROR: Could not find {JOINTS_FILE}")
        sys.exit(1)
    except yaml.YAMLError as e:
        print(f"ERROR: Failed to parse {JOINTS_FILE}: {e}")
        sys.exit(1)

    if "joints" not in data:
        print("ERROR: joints.yaml does not contain a top-level 'joints' key")
        sys.exit(1)

    joints = data["joints"]
    mvs_dof = count_mvs_actuated_dof(joints)

    print(f"MVS actuated DOF (from joints.yaml): {mvs_dof}")

    if mvs_dof != EXPECTED_MVS_DOF:
        print(f"ERROR: Expected {EXPECTED_MVS_DOF} DOF, got {mvs_dof}")
        sys.exit(1)

    print("OK: DOF count matches SPEC")
    sys.exit(0)


if __name__ == "__main__":
    main()
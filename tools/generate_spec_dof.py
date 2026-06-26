#!/usr/bin/env python3
"""
Generate a Markdown DOF table from joints.yaml.

Utility for producing a human-readable DOF summary — useful when drafting
or reviewing SPEC content. Output is printed to stdout; pipe or redirect
as needed. This is not a rehydrator: it does not write to any file.

Usage:
    python3 tools/generate_spec_dof.py
    python3 tools/generate_spec_dof.py > /tmp/dof_check.md
"""

import yaml
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
JOINTS_FILE = REPO_ROOT / "design" / "joints.yaml"
VARIANT = "MVS"


def main():
    with open(str(JOINTS_FILE), "r") as f:
        data = yaml.safe_load(f)

    joints = data.get("joints", {})

    leg_joints = []
    torso_joints = []

    for name, j in joints.items():
        if not j.get("actuated"):
            continue
        if not j.get("variants", {}).get(VARIANT, False):
            continue

        location = j.get("location")

        if location in ("hip", "knee", "ankle"):
            leg_joints.append(name)
        elif location == "torso":
            torso_joints.append(name)

    # ------------------------------------------------------------------
    # Emit Markdown
    # ------------------------------------------------------------------
    print("### Actuated Degrees of Freedom (MVS)\n")

    print("**Legs (×2):**")
    for j in sorted(leg_joints):
        axis = joints[j].get("axis")
        print(f"- {j.replace('_', ' ').title()} ({axis})")

    print("\n**Torso:**")
    for j in sorted(torso_joints):
        axis = joints[j].get("axis")
        print(f"- {j.replace('_', ' ').title()} ({axis})")

    total_dof = len(leg_joints) * 2 + len(torso_joints)

    print(f"\n**Total actuated DOF:** **{total_dof}**")


if __name__ == "__main__":
    main()

import yaml
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
JOINTS_FILE = REPO_ROOT / "design" / "joints.yaml"


def _expected_dof():
    # Read the declared total from joints.yaml rather than hardcoding it here
    # (this was a stale literal, 11, long after the design reached 13).
    with open(JOINTS_FILE, "r") as f:
        return yaml.safe_load(f)["meta"]["mvs_dof_total"]


EXPECTED_MVS_DOF = _expected_dof()


def main():
    # ------------------------------------------------------------------
    # Load joints.yaml
    # ------------------------------------------------------------------
    try:
        with open(JOINTS_FILE, "r") as f:
            data = yaml.safe_load(f)
    except Exception as e:
        print(f"ERROR: Failed to load {JOINTS_FILE}: {e}")
        sys.exit(1)

    joints = data.get("joints", {})
    if not joints:
        print("ERROR: No joints defined in joints.yaml.")
        sys.exit(1)

    # ------------------------------------------------------------------
    # Classify actuated joints for MVS
    # ------------------------------------------------------------------
    leg_joints = [
        name for name, j in joints.items()
        if j.get("actuated")
        and j.get("variants", {}).get("MVS", False)
        and j.get("location") in ("hip", "knee", "ankle")
    ]

    torso_joints = [
        name for name, j in joints.items()
        if j.get("actuated")
        and j.get("variants", {}).get("MVS", False)
        and j.get("location") == "torso"
    ]

    # ------------------------------------------------------------------
    # Compute total DOF
    # ------------------------------------------------------------------
    total_dof = len(leg_joints) * 2 + len(torso_joints)

    # ------------------------------------------------------------------
    # Report
    # ------------------------------------------------------------------
    print(f"MVS actuated leg joints per leg: {len(leg_joints)}")
    print(f"MVS actuated torso joints: {len(torso_joints)}")
    print(f"MVS total actuated DOF: {total_dof}")

    # ------------------------------------------------------------------
    # Enforce constraint
    # ------------------------------------------------------------------
    if total_dof != EXPECTED_MVS_DOF:
        print(
            f"ERROR: Expected {EXPECTED_MVS_DOF} actuated DOF for MVS, "
            f"but found {total_dof}."
        )
        sys.exit(1)

    print("OK: MVS joint configuration is valid.")


if __name__ == "__main__":
    main()
import yaml
import sys

JOINTS_FILE = "design/joints.yaml"
EXPECTED_MVS_DOF = 9


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
import yaml

JOINTS_FILE = "design/joints.yaml"
VARIANT = "MVS"


def main():
    with open(JOINTS_FILE, "r") as f:
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

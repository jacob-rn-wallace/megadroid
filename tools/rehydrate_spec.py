#!/usr/bin/env python3

import yaml
from pathlib import Path
from datetime import date
from jinja2 import Environment, FileSystemLoader, StrictUndefined


ROOT = Path(__file__).resolve().parents[1]
DESIGN = ROOT / "design"
TEMPLATES = ROOT / "templates"
OUTPUT = ROOT / "SPEC.md"


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def main():
    # ---------------------------
    # Load authoritative data
    # ---------------------------
    joints_yaml = load_yaml(DESIGN / "joints.yaml")
    geometry_yaml = load_yaml(DESIGN / "geometry.yaml")

    # ---------------------------
    # Build joint lists (MVS only)
    # ---------------------------
    joints = joints_yaml["joints"]

    leg_joints = []
    torso_joints = []

    for name, joint in joints.items():
        if not joint.get("actuated", False):
            continue
        if not joint.get("variants", {}).get("MVS", False):
            continue

        entry = {
            "name": name.replace("_", " ").title(),
            "axis": joint["axis"]
        }

        if joint["location"] in ("hip", "knee"):
            leg_joints.append(entry)
        elif joint["location"] == "torso":
            torso_joints.append(entry)

    dof_total = 2 * len(leg_joints) + len(torso_joints)

    # ---------------------------
    # Dynamic metadata (Option B)
    # ---------------------------
    meta = {
        "last_updated": date.today().isoformat()
    }

    actuation = {
        "motor_model": "775 brushed DC motors",
        "motor_voltage": 24,
        "motor_count_mvs": dof_total
    }

    power = {
        "battery": {
            "model": "Toro 88675",
            "voltage": 60,
            "capacity_ah": 7.5,
            "energy_wh": 405
        },
        "bus": {
            "input_voltage": 60,
            "output_voltage": 24,
            "current_a": 50
        }
    }

    # ---------------------------
    # Preserve Change Control section verbatim
    # ---------------------------
    spec_text = OUTPUT.read_text()
    cc_start = spec_text.index("## 15. Change Control")
    change_control = spec_text[cc_start:].strip()

    # ---------------------------
    # Render template
    # ---------------------------
    env = Environment(
        loader=FileSystemLoader(TEMPLATES),
        undefined=StrictUndefined,
        trim_blocks=True,
        lstrip_blocks=True,
    )

    template = env.get_template("SPEC.md.j2")

    rendered = template.render(
        meta=meta,
        joints={
            "legs": leg_joints,
            "torso": torso_joints,
        },
        dof={"total": dof_total},
        actuation=actuation,
        power=power,
        change_control=change_control,
    )

    OUTPUT.write_text(rendered + "\n")

    print("SPEC.md successfully rehydrated.")


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
"""
rehydrate_mech.py

Rehydrate MECH.md from templates/MECH.md.j2 using authoritative design data
and repository metadata.

Policy:
- MECH.md is a derived document
- "Last rehydrated" reflects the date of rehydration
"""

from pathlib import Path
from datetime import date
import yaml
from jinja2 import Environment, FileSystemLoader, StrictUndefined


# ----------------------------
# Paths
# ----------------------------

REPO_ROOT = Path(__file__).resolve().parents[1]
TEMPLATES_DIR = REPO_ROOT / "templates"
DESIGN_DIR = REPO_ROOT / "design"
OUTPUT_FILE = REPO_ROOT / "MECH.md"


# ----------------------------
# Helpers
# ----------------------------

def load_yaml(path: Path):
    if not path.exists():
        return {}
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


# ----------------------------
# Main rehydration logic
# ----------------------------

def main():
    # Load authoritative design data
    joints = load_yaml(DESIGN_DIR / "joints.yaml")
    geometry = load_yaml(DESIGN_DIR / "geometry.yaml")

    # Construct metadata (publication authority)
    meta = {
        "last_rehydrated": date.today().isoformat()
    }

    # Jinja environment
    env = Environment(
        loader=FileSystemLoader(TEMPLATES_DIR),
        undefined=StrictUndefined,
        trim_blocks=True,
        lstrip_blocks=True,
    )

    template = env.get_template("MECH.md.j2")

    # Render document
    rendered = template.render(
        meta=meta,
        joints=joints,
        geometry=geometry,
    )

    # Write output
    OUTPUT_FILE.write_text(rendered, encoding="utf-8")

    print("MECH.md successfully rehydrated.")


if __name__ == "__main__":
    main()
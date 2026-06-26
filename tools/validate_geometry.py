#!/usr/bin/env python3

import sys
import yaml
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

REQUIRED_PATHS = [
    ("anthropometrics", "thigh_length_mm"),
    ("anthropometrics", "shin_length_mm"),
    ("anthropometrics", "ankle_to_sole_offset_mm"),
    ("leg_structure", "twin_rail", "inner_face_spacing_mm"),
    ("joints", "shafting", "standard_diameter_mm"),
    ("end_of_limb_sensing", "ft_sensor", "form_factor", "diameter_mm"),
    ("end_of_limb_sensing", "ft_sensor", "form_factor", "height_mm"),
]

def main():
    with open(REPO_ROOT / "design" / "geometry.yaml", "r") as f:
        data = yaml.safe_load(f)

    errors = []

    for path in REQUIRED_PATHS:
        cursor = data
        for key in path:
            if key not in cursor:
                errors.append("Missing geometry path: " + ".".join(path))
                break
            cursor = cursor[key]

    if errors:
        print("Geometry validation FAILED:")
        for e in errors:
            print("  -", e)
        sys.exit(1)

    print("Geometry validation PASSED.")

if __name__ == "__main__":
    main()

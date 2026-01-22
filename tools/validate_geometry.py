#!/usr/bin/env python3

import sys
import yaml

REQUIRED_PATHS = [
    ("legs", "segments", "thigh", "length_mm"),
    ("legs", "segments", "shank", "length_mm"),
    ("legs", "segments", "ankle_to_sole_offset_mm"),
    ("legs", "structure", "rail_spacing_inner_mm"),
    ("legs", "joints", "hip", "shaft_diameter_mm"),
    ("feet", "ft_sensor", "diameter_mm"),
    ("feet", "ft_sensor", "height_mm"),
]

def main():
    with open("design/geometry.yaml", "r") as f:
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
#!/usr/bin/env python3

import subprocess
import sys

validators = [
    "tools/validate_geometry.py",
    "tools/validate_no_geometry_literals.py",
    "tools/validate_dof_consistency.py",
]

for v in validators:
    print(f"Running {v}...")
    result = subprocess.run(["python3", v])
    if result.returncode != 0:
        print("Validation failed.")
        sys.exit(1)

print("All validations PASSED.")
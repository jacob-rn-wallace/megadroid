#!/usr/bin/env python3

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

validators = [
    "tools/validate_geometry.py",
    "tools/validate_no_geometry_literals.py",
    "tools/validate_dof_consistency.py",
]

for v in validators:
    print(f"Running {v}...")
    result = subprocess.run(["python3", str(REPO_ROOT / v)], cwd=REPO_ROOT)
    if result.returncode != 0:
        print("Validation failed.")
        sys.exit(1)

print("All validations PASSED.")
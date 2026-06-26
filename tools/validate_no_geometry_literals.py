#!/usr/bin/env python3

import sys
import re
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
FILES = [str(REPO_ROOT / "SPEC.md"), str(REPO_ROOT / "MECH.md")]
FORBIDDEN_PATTERN = re.compile(r"\b\d+(\.\d+)?\s+(mm|cm|m)\b", re.IGNORECASE)

def main():
    errors = []

    for fname in FILES:
        with open(fname, "r") as f:
            for i, line in enumerate(f, 1):
                if FORBIDDEN_PATTERN.search(line):
                    errors.append(f"{fname}:{i}: {line.strip()}")

    if errors:
        print("Numeric geometry literals found:")
        for e in errors:
            print("  -", e)
        sys.exit(1)

    print("No numeric geometry literals found in SPEC.md or MECH.md.")

if __name__ == "__main__":
    main()
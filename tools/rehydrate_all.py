#!/usr/bin/env python3
"""
Rehydrate all derived documents from authoritative sources.

Run this before committing any changes to design/*.yaml or templates/*.

This script ensures SPEC.md, MECH.md, and README.md are synchronized
with their authoritative sources (YAML data and templates).
"""

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent

REHYDRATORS = [
    "tools/rehydrate_spec.py",
    "tools/rehydrate_mech.py",
    "tools/rehydrate_readme_structure.py",
]

def main():
    print("=" * 60)
    print("Rehydrating all derived documents...")
    print("=" * 60)
    print()
    
    failed = []
    
    for script in REHYDRATORS:
        script_path = REPO_ROOT / script
        print(f"→ Running {script}...")
        
        result = subprocess.run(
            ["python3", str(script_path)],
            cwd=REPO_ROOT,
            capture_output=True,
            text=True
        )
        
        # Print the script's output
        if result.stdout:
            print(result.stdout.rstrip())
        
        if result.returncode != 0:
            print(f"✗ ERROR: {script} failed")
            if result.stderr:
                print(result.stderr.rstrip())
            failed.append(script)
        else:
            print(f"✓ {script} completed")
        
        print()
    
    print("=" * 60)
    
    if failed:
        print("✗ Rehydration FAILED")
        print()
        print("Failed scripts:")
        for script in failed:
            print(f"  - {script}")
        sys.exit(1)
    
    print("✓ All documents rehydrated successfully")
    print()
    print("Next steps:")
    print("  git add SPEC.md MECH.md README.md")
    print("  git commit -m 'Docs: rehydrate from updated sources'")

if __name__ == "__main__":
    main()

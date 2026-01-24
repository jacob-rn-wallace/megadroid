#!/usr/bin/env python3

import subprocess
from pathlib import Path
from jinja2 import Environment, FileSystemLoader

ROOT = Path(__file__).resolve().parent.parent

def get_git_last_updated():
    try:
        result = subprocess.check_output(
            ["git", "log", "-1", "--format=%cs"],
            cwd=ROOT,
            text=True
        ).strip()
        return result
    except Exception:
        return "UNKNOWN"

def main():
    env = Environment(
        loader=FileSystemLoader(str(ROOT / "templates")),
        autoescape=False
    )

    template = env.get_template("MECH.md.j2")

    meta = {
        "last_updated": get_git_last_updated()
    }

    rendered = template.render(meta=meta)

    output_path = ROOT / "MECH.md"
    output_path.write_text(rendered)

    print("MECH.md successfully rehydrated.")

if __name__ == "__main__":
    main()
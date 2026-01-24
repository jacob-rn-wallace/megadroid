#!/usr/bin/env python3

import re
import yaml
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
README_PATH = REPO_ROOT / "README.md"

BEGIN = "<!-- BEGIN AUTO-GENERATED: REPO-STRUCTURE -->"
END = "<!-- END AUTO-GENERATED: REPO-STRUCTURE -->"

LICENSE_MAP = {
    "LICENSE": "Apache License 2.0 (software)",
    "LICENSE.txt": "Apache License 2.0 (software)",
    "LICENSE-HARDWARE": "CERN-OHL-S v2 (hardware)",
    "LICENSE-HARDWARE.txt": "CERN-OHL-S v2 (hardware)",
}

# ---------- Metadata loaders ----------

def load_dir_meta(path: Path):
    meta = path / ".meta.yaml"
    if not meta.exists():
        return None
    with open(meta, "r") as f:
        return yaml.safe_load(f)


def load_file_meta(path: Path):
    """
    Only accept an explicit HTML comment metadata block.
    Ignore markdown blockquotes and any non key:value lines.
    """
    try:
        text = path.read_text()
    except Exception:
        return None

    m = re.match(r"\s*<!--(.*?)-->", text, re.DOTALL)
    if not m:
        return None

    meta = {}
    for line in m.group(1).splitlines():
        line = line.strip()
        if not line or line.startswith(">"):
            continue
        if ":" not in line:
            continue
        k, v = line.split(":", 1)
        meta[k.strip()] = v.strip()

    return meta if meta else None


# ---------- Tree assembly ----------

def collect_entries():
    entries = []

    for item in sorted(REPO_ROOT.iterdir(), key=lambda p: p.name):
        if item.name.startswith("."):
            continue
        if item.name == "README.md":
            continue

        if item.is_file():
            if item.name in LICENSE_MAP:
                entries.append({
                    "name": item.name,
                    "desc": LICENSE_MAP[item.name],
                })
            else:
                meta = load_file_meta(item)
                entries.append({
                    "name": item.name,
                    "desc": meta.get("description") if meta else None,
                })

        elif item.is_dir():
            meta = load_dir_meta(item)
            entries.append({
                "name": item.name + "/",
                "desc": meta.get("description") if meta else None,
            })

    return entries


# ---------- Rendering ----------

def render(entries):
    lines = ["megadroid/"]

    for i, e in enumerate(entries):
        branch = "└── " if i == len(entries) - 1 else "├── "
        name = f"{e['name']:<20}"
        if e["desc"]:
            lines.append(f"{branch}{name} # {e['desc']}")
        else:
            lines.append(f"{branch}{name}")

    return "\n".join(line.rstrip() for line in lines)


# ---------- README update ----------

def update_readme(tree):
    text = README_PATH.read_text()

    if BEGIN not in text or END not in text:
        raise RuntimeError("README.md missing repo-structure markers")

    before, rest = text.split(BEGIN, 1)
    _, after = rest.split(END, 1)

    block = (
        f"{BEGIN}\n"
        "```\n"
        f"{tree}\n"
        "```\n"
        f"{END}"
    )

    README_PATH.write_text(before + block + after)


def main():
    tree = render(collect_entries())
    update_readme(tree)
    print("README repository structure rehydrated cleanly.")


if __name__ == "__main__":
    main()
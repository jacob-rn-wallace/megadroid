#!/usr/bin/env python3

import yaml
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
README = ROOT / "README.md"

START = "<!-- BEGIN AUTO-GENERATED: REPO-STRUCTURE -->"
END   = "<!-- END AUTO-GENERATED: REPO-STRUCTURE -->"


def load_yaml(path: Path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def crawl():
    """
    Returns:
      files: list of (name, metadata)
      dirs:  list of (name, metadata)
    """
    files = []
    dirs = []

    # --- Top-level files via .meta/*.yaml ---
    meta_root = ROOT / ".meta"
    if meta_root.exists():
        for meta in sorted(meta_root.glob("*.yaml")):
            data = load_yaml(meta)
            files.append((data["name"], data))

    # --- Top-level directories via <dir>/.meta.yaml ---
    for item in sorted(ROOT.iterdir()):
        if not item.is_dir():
            continue
        if item.name.startswith("."):
            continue

        meta_file = item / ".meta.yaml"
        if meta_file.exists():
            data = load_yaml(meta_file)
            dirs.append((item.name, data))

    return files, dirs


def generate_block(files, dirs):
    """
    Generate a tree-style ASCII repository structure block.
    """
    lines = []
    lines.append("megadroid/")

    def normalize(desc: str) -> str:
        return " ".join(desc.split())

    entries = []

    # Files first
    for name, meta in files:
        entries.append((name, normalize(meta["description"])))

    # Then directories
    for name, meta in dirs:
        entries.append((name + "/", normalize(meta["description"])))

    for i, (name, desc) in enumerate(entries):
        prefix = "└──" if i == len(entries) - 1 else "├──"
        lines.append(f"{prefix} {name:<20} # {desc}")

    return "```" + "\n" + "\n".join(lines) + "\n```"


def main():
    files, dirs = crawl()
    block = generate_block(files, dirs)

    text = README.read_text()

    if START not in text or END not in text:
        raise RuntimeError("README.md missing auto-generated repo structure markers")

    before, rest = text.split(START)
    _, after = rest.split(END)

    README.write_text(
        before
        + START + "\n"
        + block + "\n"
        + END
        + after
    )

    print("README repository structure successfully rehydrated.")


if __name__ == "__main__":
    main()
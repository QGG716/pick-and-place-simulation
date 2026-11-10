"""Verify the published local/GPU asset inventory without loading model code."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path


def verify(root: Path, inventory: Path):
    root = root.resolve()
    data = json.loads(inventory.read_text(encoding="utf-8"))
    checked = 0
    errors = []
    for relative, expected in data["files"].items():
        path = (root / relative).resolve()
        if not path.is_relative_to(root):
            errors.append({"path": relative, "error": "PATH_OUTSIDE_REPOSITORY"})
            continue
        if not path.is_file():
            errors.append({"path": relative, "error": "MISSING"})
            continue
        if path.stat().st_size != expected["size"]:
            with path.open("rb") as stream:
                pointer = stream.read(128).startswith(b"version https://git-lfs.github.com/spec/v1\n")
            errors.append({"path": relative, "error": "LFS_POINTER_RUN_GIT_LFS_PULL" if pointer else "SIZE_MISMATCH"})
            continue
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(chunk)
        if digest.hexdigest() != expected["sha256"]:
            errors.append({"path": relative, "error": "SHA256_MISMATCH"})
        else:
            checked += 1
    return {"status": "FAIL" if errors else "PASS", "checked": checked,
            "expected": len(data["files"]), "errors": errors}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--inventory", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    report = verify(args.root, args.inventory or args.root / "assets/ASSET_INVENTORY.json")
    text = json.dumps(report, ensure_ascii=False, indent=2)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")
    print(text)
    raise SystemExit(1 if report["errors"] else 0)


if __name__ == "__main__":
    main()

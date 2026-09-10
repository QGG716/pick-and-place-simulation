"""Download only the fixed SAM/MoGe files and emit a content manifest."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path

from huggingface_hub import hf_hub_download, snapshot_download


SAM_ID = "facebook/sam-vit-base"
SAM_REVISION = "70c1a07f894ebb5b307fd9eaaee97b9dfc16068f"
MOGE_ID = "Ruicheng/moge-2-vits-normal"
MOGE_REVISION = "26b477f41595707c5db6770294c0d1721e8ed4ed"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--cache-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--endpoint", default=os.environ.get("HF_ENDPOINT"))
    args = parser.parse_args()
    args.cache_dir.mkdir(parents=True, exist_ok=True)
    if args.endpoint:
        os.environ["HF_ENDPOINT"] = args.endpoint

    sam_dir = Path(snapshot_download(
        repo_id=SAM_ID,
        revision=SAM_REVISION,
        cache_dir=args.cache_dir,
        allow_patterns=("config.json", "preprocessor_config.json", "model.safetensors"),
    )).resolve()
    moge_file = Path(hf_hub_download(
        repo_id=MOGE_ID,
        revision=MOGE_REVISION,
        filename="model.pt",
        cache_dir=args.cache_dir,
    )).resolve()
    files = [sam_dir / "config.json", sam_dir / "preprocessor_config.json", sam_dir / "model.safetensors", moge_file]
    if not all(path.is_file() for path in files):
        raise RuntimeError("a fixed model file is missing after download")
    manifest = {
        "manifest_version": 1,
        "sam": {"repository": SAM_ID, "revision": SAM_REVISION, "snapshot_path": str(sam_dir),
                "files": {path.name: sha256(path) for path in files[:3]}},
        "moge": {"repository": MOGE_ID, "revision": MOGE_REVISION, "model_path": str(moge_file),
                 "files": {"model.pt": sha256(moge_file)}},
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

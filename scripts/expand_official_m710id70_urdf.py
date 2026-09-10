#!/usr/bin/env python3
"""Deterministically expand the pinned official M-710iD/70 xacro.

The xacro dependency is deliberately optional and imported only by this build
tool.  The generated URDF is canonicalised so its bytes do not contain machine
paths, timestamps, indentation differences, or xacro generator comments.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import xml.etree.ElementTree as ET


DEFAULT_WRAPPER = Path(
    "assets/robots/fanuc_m710id_70/official/fanuc_m710_description/"
    "robot/m710id_70.standalone.urdf.xacro"
)
DEFAULT_OUTPUT = Path(
    "assets/robots/fanuc_m710id_70/official/fanuc_m710_description/"
    "urdf/m710id_70_official.urdf"
)


def expand(wrapper: Path, output: Path) -> None:
    try:
        import xacro
    except ImportError as exc:  # pragma: no cover - exercised by build environment
        raise SystemExit("xacro is required only to regenerate the static official URDF") from exc

    wrapper = wrapper.resolve()
    output = output.resolve()
    previous = Path.cwd()
    try:
        os.chdir(wrapper.parent)
        document = xacro.process_file(wrapper.name)
    finally:
        os.chdir(previous)

    raw = document.toxml()
    if "xacro:" in raw or "${" in raw or "$(" in raw:
        raise RuntimeError("xacro expansion left unresolved substitutions")
    canonical = ET.canonicalize(raw, strip_text=True, with_comments=False)
    payload = f'<?xml version="1.0" encoding="utf-8"?>\n{canonical}\n'.encode("utf-8")
    output.parent.mkdir(parents=True, exist_ok=True)
    temporary = output.with_suffix(output.suffix + ".tmp")
    temporary.write_bytes(payload)
    temporary.replace(output)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--wrapper", type=Path, default=DEFAULT_WRAPPER)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    expand(args.wrapper, args.output)


if __name__ == "__main__":
    main()

"""Select audited pure modules from an authorized local reference checkout.

Never import the legacy GUI, robot executor, serial writer or saved config.
"""
import argparse
import ast
import hashlib
import json
from pathlib import Path
import subprocess

PIN = "f0353d720374066528f258de6f4de24502408535"
ROOT = Path(__file__).resolve().parents[1]


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("reference", type=Path)
    a = p.parse_args()
    head = subprocess.check_output(["git", "-C", str(a.reference), "rev-parse", "HEAD"], text=True).strip()
    if head != PIN:
        raise ValueError("Reference changed; review before importing")
    def source_bytes(name):
        # Git blobs avoid platform checkout CRLF changing the recorded source identity.
        return subprocess.check_output(["git", "-C", str(a.reference), "show", f"{PIN}:arith_dex_pkg/{name}"])
    target = ROOT / "src/unloading_sim/workbench/reference"
    target.mkdir(parents=True, exist_ok=True)
    (target / "__init__.py").write_text('"""Audited pure observation helpers; see reference_sources.json."""\n', newline="\n")
    records = []
    for name in ("arith_grounding_dino.py", "arith_sam_v1.py", "camera_geometry.py"):
        data = source_bytes(name)
        (target / name).write_bytes(data)
        records.append({"source": "arith_dex_pkg/"+name, "sha256": hashlib.sha256(data).hexdigest(),
                        "selection": "unchanged file"})
    name = "vacuum.py"
    data = source_bytes(name)
    source = data.decode("utf-8-sig").replace("\r\n", "\n")
    lines = source.splitlines(keepends=True)
    selected = {"HEADER", "DEVICE", "BAUD", "MAX_PARAMETERS", "COMMANDS", "ALARMS",
                "POSITIVE_THRESHOLD", "NEGATIVE_THRESHOLD", "Gauge", "GAUGES", "HEARTBEATS",
                "PLAUSIBLE_KPA", "HEARTBEAT_FORMAT", "FrameParser", "TextReportParser",
                "_remove_frames", "parse_heartbeat"}
    chunks = ['"""Receive-only selections from reference vacuum.py at '+PIN+'.\n'
              '0x71=positive, 0x72=negative, as hardware-corrected upstream.\n'
              'No serial writer, threshold setter or alarm motion callback is imported.\n"""\n',
              'from __future__ import annotations\nimport math\nimport re\nimport struct\nfrom dataclasses import dataclass\n']
    for node in ast.parse(source).body:
        names = {getattr(node, "name", "")}
        if isinstance(node, ast.Assign):
            names |= {n.id for t in node.targets for n in ast.walk(t) if isinstance(n, ast.Name)}
        if names & selected:
            start = min([node.lineno] + [d.lineno for d in getattr(node, "decorator_list", [])])
            chunks.append("".join(lines[start-1:node.end_lineno])+"\n\n")
    (target / "vacuum_receive.py").write_text("".join(chunks).rstrip()+"\n", encoding="utf-8", newline="\n")
    records.append({"source": "arith_dex_pkg/vacuum.py", "sha256": hashlib.sha256(data).hexdigest(),
                    "selection": sorted(selected)})
    # Only the model-independent metric-input validator is reused, not a second scene/planner.
    data = source_bytes("rgbd_geometry.py")
    lines = data.decode("utf-8-sig").replace("\r\n", "\n").splitlines(keepends=True)
    chunks = ['"""Selected RGBD validation from the pinned reference."""\nfrom dataclasses import dataclass\nimport numpy as np\n']
    for node in ast.parse("".join(lines)).body:
        if getattr(node, "name", "") in {"RGBDInput", "validate_input"}:
            start = min([node.lineno] + [d.lineno for d in getattr(node, "decorator_list", [])])
            chunks.append("".join(lines[start-1:node.end_lineno])+"\n\n")
    (target / "rgbd_geometry.py").write_text("".join(chunks).rstrip()+"\n", encoding="utf-8", newline="\n")
    records.append({"source": "arith_dex_pkg/rgbd_geometry.py", "sha256": hashlib.sha256(data).hexdigest(),
                    "selection": ["RGBDInput", "validate_input"]})
    lock = {"repository": "zhangyong333/Unloading-Robot-Control-Platform", "commit": PIN,
            "source_files": records, "source_hash_basis": "PINNED_GIT_BLOB_BYTES", "credentials_included": False,
            "output_sha256": {f.name: hashlib.sha256(f.read_bytes()).hexdigest() for f in target.glob("*.py")}}
    (target / "reference_sources.json").write_text(json.dumps(lock, indent=2), encoding="utf-8")
    print(f"Imported {len(records)} audited observation modules from {head}")


if __name__ == "__main__":
    main()

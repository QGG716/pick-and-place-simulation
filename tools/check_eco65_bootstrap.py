"""Read-only BOOTSTRAP check; no STEP processing, planning or hardware calls."""
import hashlib
import importlib
import importlib.util
import json
from pathlib import Path
import platform
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml
from eco65_official_archive import inspect

ROOT = Path(__file__).resolve().parents[1]
OFFICIAL = ROOT / "assets/robots/realman_eco65/official"

def git(*args):
    return subprocess.check_output(["git", "-c", "safe.directory="+ROOT.as_posix(), *args], cwd=ROOT).decode("utf-8").strip()

def check():
    lock = yaml.safe_load((ROOT / "configs/integration/eco65_desktop_sources.lock.yaml").read_text(encoding="utf-8"))
    assert Path(git("rev-parse", "--show-toplevel")).resolve() == ROOT
    assert git("branch", "--show-current") == lock["target_branch"]
    assert git("remote", "get-url", "origin") == lock["repository"]
    assert git("rev-parse", "--is-shallow-repository") == "false"
    assert not (ROOT / ".git/objects/info/alternates").exists()
    for src in lock["sources"].values():
        assert git("rev-parse", src["local_ref"]) == src["sha"]
        assert git("cat-file", "-t", src["sha"]) == "commit"
    base = lock["sources"]["feasibility"]["sha"]
    assert not git("diff", base, "--", "src", "pyproject.toml", "uv.lock")
    assert lock["integration_status"] == "not_integrated"
    assert lock["hardware"]["robot_variant"] == "ECO65-B"
    assert Path(sys.prefix).resolve() == (ROOT / ".venv").resolve()
    modules = ["numpy", "yaml", "matplotlib", "unloading_sim.geometry", "unloading_sim.robot",
               "unloading_sim.scene", "unloading_sim.ik", "unloading_sim.grasp", "unloading_sim.planner"]
    for name in modules:
        importlib.import_module(name)
    drop = ROOT / "assets/tools/desktop_suction/cad/raw"
    assert drop.is_dir()
    for folder in ["cad/raw", "meshes", "collision"]:
        for name in ["a.step", "a.stp", "A.STEP", "A.STP", "中文 文件.StEp", "sub/装配.STp"]:
            rel = f"assets/tools/desktop_suction/{folder}/{name}"
            assert git("check-ignore", "--", rel), rel
        keep = f"assets/tools/desktop_suction/{folder}/.gitkeep"
        assert subprocess.run(["git", "-c", "safe.directory="+ROOT.as_posix(), "check-ignore", "-q", "--", keep], cwd=ROOT).returncode == 1
    manifest = json.loads((OFFICIAL / "SOURCE_MANIFEST.json").read_text(encoding="utf-8"))
    packages = {}
    for p in OFFICIAL.glob("description/**/package.xml"):
        packages.setdefault(ET.parse(p).getroot().findtext("name"), []).append(p.parent)
    missing, meshes, xml_count, files = [], 0, 0, 0
    for entry in manifest["entries"]:
        if entry["download_status"] != "downloaded":
            missing.append(entry["local_path"])
            continue
        p = OFFICIAL / entry["local_path"]
        data = p.read_bytes()
        assert len(data) == entry["size_bytes"], str(p)
        assert hashlib.sha256(data).hexdigest() == entry["sha256"], str(p)
        if entry.get("git_blob_sha"):
            assert hashlib.sha1(f"blob {len(data)}\0".encode()+data).hexdigest() == entry["git_blob_sha"]
        inspect(p)
        files += 1
        if p.suffix.lower() in (".urdf", ".xacro", ".xml", ".launch"):
            tree = ET.parse(p)
            xml_count += 1
            for mesh in tree.iter("mesh"):
                name = mesh.attrib["filename"]
                if name.startswith("package://"):
                    package, rel = name[len("package://"):].split("/", 1)
                else:
                    match = re.fullmatch(r"file://\$\(find ([^)]+)\)/(.*)", name)
                    assert match is not None, name
                    package, rel = match.groups()
                candidates = [base / rel for base in packages.get(package, [])]
                assert any(candidate.is_file() for candidate in candidates), (str(p), name)
                meshes += 1
        if p.suffix.lower() in (".png", ".jpg", ".jpeg"):
            from PIL import Image
            with Image.open(p) as image:
                image.verify()
    assert not missing, missing
    step_files = [str(p.relative_to(drop)) for p in drop.rglob("*") if p.is_file() and p.suffix.lower() in (".step", ".stp")]
    result = dict(status="bootstrap_checks_passed", root=str(ROOT), branch=git("branch", "--show-current"),
                  head=git("rev-parse", "HEAD"), python=sys.version, platform=platform.platform(),
                  modules_imported=modules, manifest_files_verified=files, xml_files_parsed=xml_count,
                  mesh_references_resolved=meshes, user_step_status="present_not_processed" if step_files else "awaiting_user_file",
                  user_step_files=step_files, cad_readers_in_project_venv={name: importlib.util.find_spec(name) is not None for name in ["OCP", "OCC", "cadquery", "FreeCAD"]},
                  caveat="File integrity only, no ECO65 kinematics/dynamics/integration acceptance.")
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return result

if __name__ == "__main__":
    check()

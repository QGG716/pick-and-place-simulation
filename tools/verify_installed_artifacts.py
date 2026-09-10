"""Verify built distributions from outside the repository source tree."""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import os
from pathlib import Path
import subprocess
import sys
import sysconfig
import zipfile


def _wheel_members(path: Path) -> set[str]:
    with zipfile.ZipFile(path) as archive:
        return set(archive.namelist())


def _metadata(path: Path) -> tuple[str, str]:
    with zipfile.ZipFile(path) as archive:
        metadata_name = next(name for name in archive.namelist() if name.endswith(".dist-info/METADATA"))
        fields = {}
        for line in archive.read(metadata_name).decode("utf-8").splitlines():
            if ": " in line:
                key, value = line.split(": ", 1)
                fields.setdefault(key, value)
        return fields["Name"], fields["Version"]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository", type=Path, required=True)
    parser.add_argument("--contracts-wheel", type=Path, required=True)
    parser.add_argument("--root-wheel", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    repository = args.repository.resolve()
    contracts_wheel = args.contracts_wheel.resolve()
    root_wheel = args.root_wheel.resolve()
    contracts_members = _wheel_members(contracts_wheel)
    root_members = _wheel_members(root_wheel)
    assert _metadata(contracts_wheel) == ("unloading-contracts", "1.1.0")
    assert _metadata(root_wheel) == ("unloading-layer1-sim", "0.5.0.dev0")
    assert any(name == "unloading_contracts/__init__.py" for name in contracts_members)
    assert any(name == "unloading_contracts/models.py" for name in contracts_members)
    assert not any(name.startswith("unloading_sim/") for name in contracts_members)
    assert any(name == "unloading_sim/__init__.py" for name in root_members)
    assert any(name == "unloading_perception/execution.py" for name in root_members)
    assert not any(name.startswith("unloading_contracts/") for name in root_members)

    import unloading_contracts
    import unloading_perception.execution
    import unloading_sim.geometry

    imported = {
        "unloading_contracts": Path(unloading_contracts.__file__).resolve(),
        "unloading_perception": Path(unloading_perception.execution.__file__).resolve(),
        "unloading_sim": Path(unloading_sim.geometry.__file__).resolve(),
    }
    for path in imported.values():
        assert repository != path and repository not in path.parents, path
    assert importlib.metadata.version("unloading-contracts") == "1.1.0"
    assert importlib.metadata.version("unloading-layer1-sim") == "0.5.0.dev0"
    owners = importlib.metadata.packages_distributions().get("unloading_contracts", [])
    assert owners == ["unloading-contracts"], owners
    assert not {"torch", "rclpy", "cuda"}.intersection(sys.modules)

    environment = os.environ.copy()
    environment.pop("PYTHONPATH", None)
    environment.pop("PYTHONHOME", None)
    commands = {}
    scripts_directory = Path(sysconfig.get_path("scripts")).resolve()
    for name in ("unloading-layer1-demo", "unloading-perception-demo"):
        executable = scripts_directory / name
        assert executable.is_file(), executable
        completed = subprocess.run(
            [str(executable), "--help"], cwd=Path.cwd(), env=environment,
            text=True, capture_output=True, timeout=30,
        )
        assert completed.returncode == 0, (name, completed.stderr)
        commands[name] = {"path": str(executable), "returncode": completed.returncode}

    result = {
        "schema_version": 1,
        "distributions": {
            "unloading-contracts": "1.1.0",
            "unloading-layer1-sim": "0.5.0.dev0",
        },
        "module_files": {name: str(path) for name, path in imported.items()},
        "entry_points": commands,
        "forbidden_modules_loaded": [],
        "source_tree_absent_from_imports": True,
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

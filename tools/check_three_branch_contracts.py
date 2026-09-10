"""Validate the frozen consumers in detached clones and isolated interpreters."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def run(command: list[str], *, cwd: Path, capture: bool = False) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        command,
        cwd=cwd,
        check=True,
        text=True,
        capture_output=capture,
    )


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def junit_summary(path: Path) -> dict[str, int | float]:
    root = ET.parse(path).getroot()
    suites = [root] if root.tag == "testsuite" else list(root.findall("testsuite"))
    return {
        "tests": sum(int(item.attrib.get("tests", 0)) for item in suites),
        "failures": sum(int(item.attrib.get("failures", 0)) for item in suites),
        "errors": sum(int(item.attrib.get("errors", 0)) for item in suites),
        "skipped": sum(int(item.attrib.get("skipped", 0)) for item in suites),
        "seconds": round(sum(float(item.attrib.get("time", 0.0)) for item in suites), 6),
    }


def venv_python(venv: Path) -> Path:
    return venv / ("Scripts/python.exe" if sys.platform == "win32" else "bin/python")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repository", type=Path, default=ROOT)
    parser.add_argument("--python", type=Path, default=Path(sys.executable))
    parser.add_argument("--work-root", type=Path, default=ROOT / ".test-tmp" / "contracts")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--check-only", action="store_true", help="only verify fixed SHAs and patch applicability")
    args = parser.parse_args()

    manifest = json.loads((ROOT / "integration" / "manifest.json").read_text(encoding="utf-8"))
    work_root = args.work_root.resolve()
    work_root.mkdir(parents=True, exist_ok=True)
    temporary = Path(tempfile.mkdtemp(prefix="run-", dir=work_root))
    report: dict[str, object] = {
        "schema_version": manifest["contracts_schema_version"],
        "source_repository": str(args.repository.resolve()),
        "consumers": {},
    }
    try:
        for name, consumer in manifest["consumers"].items():
            commit = consumer["commit"]
            patch = ROOT / consumer["patch"]
            run(["git", "cat-file", "-e", commit + "^{commit}"], cwd=args.repository)
            checkout = temporary / name
            run(["git", "clone", "--shared", "--no-checkout", str(args.repository.resolve()), str(checkout)], cwd=temporary)
            # Remote branch heads can be shallow/orphan commits.  Fetch the
            # exact object explicitly instead of relying on clone ref policy.
            run(["git", "fetch", "--depth=1", str(args.repository.resolve()), commit], cwd=checkout)
            run(["git", "checkout", "--detach", commit], cwd=checkout)
            run(["git", "apply", "--check", str(patch)], cwd=checkout)
            result: dict[str, object] = {
                "ref": consumer["ref"],
                "commit": commit,
                "head": run(["git", "rev-parse", "HEAD"], cwd=checkout, capture=True).stdout.strip(),
                "patch": str(patch.relative_to(ROOT)),
                "patch_sha256": sha256(patch),
                "patch_applies": True,
            }
            report["consumers"][name] = result
            if args.check_only:
                continue

            run(["git", "apply", str(patch)], cwd=checkout)
            venv = temporary / f"venv-{name}"
            run([str(args.python), "-m", "venv", str(venv)], cwd=temporary)
            python = venv_python(venv)
            run([str(python), "-m", "pip", "install", "pip==25.2", "setuptools==80.9.0", "wheel==0.45.1"], cwd=checkout)
            run([str(python), "-m", "pip", "install", str(ROOT / "packages" / "unloading_contracts")], cwd=checkout)
            run([str(python), "-m", "pip", "install", "-e", ".[dev]"], cwd=checkout)
            version = run(
                [str(python), "-c", "import unloading_contracts as c; print(c.SCHEMA_VERSION)"],
                cwd=checkout,
                capture=True,
            ).stdout.strip()
            if version != manifest["contracts_schema_version"]:
                raise RuntimeError(f"{name} loaded shared schema {version!r}")

            junit = temporary / f"{name}.xml"
            if name == "online_continuous":
                tests = sorted(str(path.relative_to(checkout)) for path in (checkout / "tests").glob("test_online*.py"))
                tests.append("tests/test_shared_contract_identity.py")
            else:
                tests = ["tests/test_shared_contract_adapter.py", "tests/test_geometry.py"]
            run([str(python), "-m", "pytest", "-q", *tests, f"--junitxml={junit}"], cwd=checkout)
            result["python"] = run([str(python), "--version"], cwd=checkout, capture=True).stdout.strip()
            result["contracts_schema_version"] = version
            result["tests"] = junit_summary(junit)

        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2, sort_keys=True))
        return 0
    finally:
        shutil.rmtree(temporary, ignore_errors=True)


if __name__ == "__main__":
    raise SystemExit(main())

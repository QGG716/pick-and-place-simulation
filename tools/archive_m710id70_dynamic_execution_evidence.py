"""Archive a portable summary of paired M-710 motion and execution preflight evidence.

This command is deliberately read-only with respect to planners and simulator
backends.  It verifies two existing JSON evidence documents and writes one
small ``run_summary.json``; it never imports or starts Isaac Sim.
"""

from __future__ import annotations

import argparse
import copy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path, PurePosixPath, PureWindowsPath
import platform
import re
import subprocess
import sys
from typing import Any, Mapping, Sequence


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from unloading_sim.layout_single_carton import (  # noqa: E402
    MOTION_IMPLEMENTATION_FILES,
    RESULT_SCHEMA as MOTION_RESULT_SCHEMA,
)
from unloading_sim.m710_execution import (  # noqa: E402
    EXECUTION_IMPLEMENTATION_FILES,
    PREFLIGHT_SCHEMA,
    verify_m710_execution_preflight,
)
from unloading_sim.workcell_layout import canonical_digest, sha256_file  # noqa: E402


SUMMARY_SCHEMA = "m710id70_dynamic_execution_run_summary_v1"
DEFAULT_DIRECTORY = ROOT / "docs/validation/evidence/m710id70_dynamic_execution_v1"
DEFAULT_MOTION = DEFAULT_DIRECTORY / "single_carton_motion_audit.json"
DEFAULT_PREFLIGHT = DEFAULT_DIRECTORY / "dynamic_execution_preflight.json"
DEFAULT_OUTPUT = DEFAULT_DIRECTORY / "run_summary.json"
ARCHIVE_IMPLEMENTATION_PATH = "tools/archive_m710id70_dynamic_execution_evidence.py"
SHA256_PATTERN = re.compile(r"[0-9a-f]{64}\Z")
GIT_OBJECT_ID_PATTERN = re.compile(r"(?:[0-9a-f]{40}|[0-9a-f]{64})\Z")
BACKEND_EXECUTION_STATUSES = frozenset({"NOT_RUN", "NOT_RUN_PER_USER_REQUEST", "PASS", "FAIL"})


def _unique_object(pairs: Sequence[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"duplicate JSON key {key!r}")
        result[key] = value
    return result


def _reject_nonfinite(token: str) -> None:
    raise ValueError(f"non-finite JSON number {token!r} is not permitted")


def _read_json_object(path: Path, label: str) -> tuple[dict[str, Any], str]:
    if not path.is_file():
        raise FileNotFoundError(f"{label} is missing: {path}")
    try:
        raw = path.read_bytes()
        value = json.loads(
            raw.decode("utf-8"),
            object_pairs_hook=_unique_object,
            parse_constant=_reject_nonfinite,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{label} is not valid UTF-8 JSON: {path}") from exc
    if not isinstance(value, dict):
        raise ValueError(f"{label} must contain one JSON object")
    return value, hashlib.sha256(raw).hexdigest()


def _mapping(value: object, name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ValueError(f"{name} must be a mapping")
    return value


def _text(value: object, name: str) -> str:
    result = str(value).strip() if value is not None else ""
    if not result:
        raise ValueError(f"{name} must be a non-empty string")
    return result


def _sha256(value: object, name: str) -> str:
    result = _text(value, name)
    if SHA256_PATTERN.fullmatch(result) is None:
        raise ValueError(f"{name} must be a lowercase SHA-256 digest")
    return result


def _git_object_id(value: object, name: str) -> str:
    result = _text(value, name)
    if GIT_OBJECT_ID_PATTERN.fullmatch(result) is None:
        raise ValueError(f"{name} must be a Git object ID")
    return result


def _boolean(value: object, name: str) -> bool:
    if not isinstance(value, bool):
        raise ValueError(f"{name} must be boolean")
    return value


def _repo_relative(path: Path, project_root: Path, name: str) -> str:
    root = project_root.resolve()
    resolved = path.resolve()
    try:
        relative = resolved.relative_to(root)
    except ValueError as exc:
        raise ValueError(f"{name} must be inside the repository") from exc
    if not relative.parts:
        raise ValueError(f"{name} cannot be the repository root")
    return relative.as_posix()


def _source_path(value: object, project_root: Path, name: str) -> tuple[str, Path]:
    declared = _text(value, name)
    if "\\" in declared:
        raise ValueError(f"{name} must use a repository-relative POSIX path")
    pure = PurePosixPath(declared)
    if pure.is_absolute() or not pure.parts or any(part in {"", ".", ".."} for part in pure.parts):
        raise ValueError(f"{name} must use a safe repository-relative POSIX path")
    normalized = pure.as_posix()
    resolved = (project_root / Path(*pure.parts)).resolve()
    if _repo_relative(resolved, project_root, name) != normalized:
        raise ValueError(f"{name} escapes or aliases its repository-relative path")
    if not resolved.is_file():
        raise FileNotFoundError(f"{name} source file is missing: {normalized}")
    return normalized, resolved


def _validated_source_hashes(
    value: object,
    expected_paths: Sequence[str],
    project_root: Path,
    name: str,
) -> dict[str, str]:
    source_hashes = _mapping(value, name)
    expected = set(expected_paths)
    if set(source_hashes) != expected or len(source_hashes) != len(expected):
        raise ValueError(
            f"{name} source set mismatch: missing={sorted(expected - set(source_hashes))} "
            f"extra={sorted(set(source_hashes) - expected)}"
        )
    result: dict[str, str] = {}
    for declared in sorted(source_hashes):
        relative, source = _source_path(declared, project_root, f"{name}.{declared}")
        recorded = _sha256(source_hashes[declared], f"{name}.{relative}")
        actual = sha256_file(source)
        if actual != recorded:
            raise ValueError(f"{name} source hash mismatch: {relative}")
        result[relative] = recorded
    return result


def _verify_motion_fingerprint(motion: Mapping[str, Any]) -> str:
    if motion.get("schema") != MOTION_RESULT_SCHEMA:
        raise ValueError("motion result has an unsupported schema")
    content = copy.deepcopy(dict(motion))
    recorded = _sha256(content.pop("evidence_fingerprint", None), "motion.evidence_fingerprint")
    if canonical_digest(content) != recorded:
        raise ValueError("motion result evidence fingerprint mismatch")
    return recorded


def _require_equal(actual: object, expected: object, name: str) -> None:
    if actual != expected:
        raise ValueError(f"motion/preflight identity mismatch: {name}")


def validate_evidence_pair(
    motion: Mapping[str, Any],
    preflight: Mapping[str, Any],
    *,
    project_root: str | Path = ROOT,
) -> dict[str, Any]:
    """Verify both content fingerprints, their shared identity, and source hashes."""

    root = Path(project_root).resolve()
    motion_fingerprint = _verify_motion_fingerprint(motion)
    preflight_verification = verify_m710_execution_preflight(preflight)
    preflight_fingerprint = _sha256(
        preflight_verification.get("preflight_fingerprint"), "preflight.preflight_fingerprint"
    )
    if preflight.get("schema") != PREFLIGHT_SCHEMA:
        raise ValueError("preflight has an unsupported schema")

    input_identity = _mapping(preflight.get("input_identity"), "preflight.input_identity")
    execution_asset_fingerprint = _sha256(
        preflight.get("execution_asset_fingerprint_sha256"),
        "preflight.execution_asset_fingerprint_sha256",
    )
    if canonical_digest(dict(input_identity)) != execution_asset_fingerprint:
        raise ValueError("preflight execution asset fingerprint mismatch")
    preflight_motion = _mapping(preflight.get("motion"), "preflight.motion")
    preflight_scene = _mapping(preflight.get("scene"), "preflight.scene")
    task_population = _mapping(motion.get("task_population"), "motion.task_population")

    _require_equal(input_identity.get("motion_evidence_fingerprint"), motion_fingerprint, "motion fingerprint")
    _require_equal(preflight_motion.get("evidence_fingerprint"), motion_fingerprint, "motion summary fingerprint")
    _require_equal(input_identity.get("layout_fingerprint"), motion.get("layout_fingerprint"), "layout fingerprint")
    _require_equal(preflight_scene.get("layout_fingerprint"), motion.get("layout_fingerprint"), "scene layout fingerprint")
    _require_equal(preflight_scene.get("layout_id"), motion.get("layout_id"), "layout ID")
    _require_equal(input_identity.get("scene_fingerprint"), motion.get("scene_fingerprint"), "scene fingerprint")
    _require_equal(preflight_scene.get("scene_fingerprint"), motion.get("scene_fingerprint"), "preflight scene fingerprint")
    _require_equal(preflight_motion.get("run_status"), motion.get("run_status"), "motion run status")
    _require_equal(
        preflight_motion.get("complete_trajectory_status"),
        motion.get("complete_trajectory_status"),
        "complete trajectory status",
    )
    _require_equal(
        preflight_motion.get("complete_trajectory_failure_reason"),
        motion.get("complete_trajectory_failure_reason"),
        "complete trajectory failure reason",
    )
    _require_equal(preflight_motion.get("statistics"), motion.get("statistics"), "motion statistics")
    _require_equal(
        preflight_motion.get("task_population"),
        task_population.get("carton_ids"),
        "task population",
    )
    dynamics = _mapping(preflight.get("dynamics"), "preflight.dynamics")
    _require_equal(dynamics.get("fingerprint"), input_identity.get("dynamics_fingerprint"), "dynamics fingerprint")

    motion_implementation = _mapping(motion.get("implementation_identity"), "motion.implementation_identity")
    if set(motion_implementation) != {"source_sha256", "runtime"}:
        raise ValueError("motion.implementation_identity must contain source_sha256 and runtime")
    motion_runtime = _mapping(motion_implementation["runtime"], "motion.implementation_identity.runtime")
    if set(motion_runtime) != {"python", "python_implementation", "platform", "numpy"}:
        raise ValueError("motion implementation runtime identity is incomplete")
    for key, value in motion_runtime.items():
        _text(value, f"motion.implementation_identity.runtime.{key}")
    motion_sources = _validated_source_hashes(
        motion_implementation["source_sha256"],
        MOTION_IMPLEMENTATION_FILES,
        root,
        "motion.implementation_identity.source_sha256",
    )
    execution_sources = _validated_source_hashes(
        input_identity.get("execution_implementation_source_sha256"),
        EXECUTION_IMPLEMENTATION_FILES,
        root,
        "preflight.input_identity.execution_implementation_source_sha256",
    )

    combined: dict[str, str] = {}
    for group in (motion_sources, execution_sources):
        for path, digest in group.items():
            if path in combined and combined[path] != digest:
                raise ValueError(f"implementation source identities disagree: {path}")
            combined[path] = digest
    archive_relative, archive_source = _source_path(
        ARCHIVE_IMPLEMENTATION_PATH, root, "archive implementation"
    )
    archive_sources = {archive_relative: sha256_file(archive_source)}
    aggregate_source_identity = canonical_digest(
        {
            "motion_source_sha256": motion_sources,
            "execution_source_sha256": execution_sources,
            "archive_source_sha256": archive_sources,
        }
    )
    aggregate_evidence_identity = canonical_digest(
        {
            "motion_evidence_fingerprint": motion_fingerprint,
            "preflight_fingerprint": preflight_fingerprint,
            "execution_asset_fingerprint_sha256": execution_asset_fingerprint,
        }
    )
    return {
        "motion_evidence_fingerprint": motion_fingerprint,
        "preflight_fingerprint": preflight_fingerprint,
        "execution_asset_fingerprint_sha256": execution_asset_fingerprint,
        "motion_source_sha256": motion_sources,
        "execution_source_sha256": execution_sources,
        "archive_source_sha256": archive_sources,
        "aggregate_source_identity_sha256": aggregate_source_identity,
        "aggregate_evidence_identity_sha256": aggregate_evidence_identity,
    }


def _run_git(project_root: Path, *args: str, binary: bool = False) -> str | bytes:
    completed = subprocess.run(
        ["git", *args],
        cwd=project_root,
        check=True,
        capture_output=True,
        text=not binary,
    )
    return completed.stdout if binary else completed.stdout.strip()


def _git_metadata(project_root: Path) -> dict[str, Any]:
    head = _git_object_id(_run_git(project_root, "rev-parse", "HEAD"), "git HEAD")
    branch = _text(_run_git(project_root, "rev-parse", "--abbrev-ref", "HEAD"), "git branch")
    paths: set[str] = set()
    commands = (
        ("diff", "--name-only", "-z", "--"),
        ("diff", "--cached", "--name-only", "-z", "--"),
        ("ls-files", "--others", "--exclude-standard", "-z"),
    )
    for command in commands:
        raw = _run_git(project_root, *command, binary=True)
        if not isinstance(raw, bytes):  # pragma: no cover - defensive typing guard
            raise TypeError("binary git output was not bytes")
        for encoded in raw.split(b"\0"):
            if not encoded:
                continue
            declared = encoded.decode("utf-8", errors="strict")
            relative, _ = _source_path(declared, project_root, "git changed path") if (
                project_root.joinpath(*PurePosixPath(declared).parts).is_file()
            ) else (_repo_relative(project_root / declared, project_root, "git changed path"), project_root / declared)
            paths.add(relative)
    return {
        "head": head,
        "branch": branch,
        "dirty": bool(paths),
        "changed_paths": sorted(paths),
    }


def _standard_commands(motion_path: str, preflight_path: str, output_path: str) -> list[str]:
    return [
        "python tools/run_m710id70_layout_single_carton.py "
        "--config configs/validation/m710id70_layout_v1_single_carton.yaml "
        f"--output {motion_path}",
        "python tools/prepare_m710id70_dynamic_execution.py "
        "--config configs/simulation/m710id70_dynamic_execution_v1.yaml "
        f"--motion-result {motion_path} --output {preflight_path}",
        "python tools/archive_m710id70_dynamic_execution_evidence.py "
        f"--motion-result {motion_path} --preflight {preflight_path} --output {output_path}",
    ]


def _portable_status(value: object, name: str) -> str:
    result = _text(value, name)
    if PureWindowsPath(result).is_absolute() or PurePosixPath(result).is_absolute():
        raise ValueError(f"{name} cannot contain an absolute path")
    if re.search(r"[A-Za-z]:[\\/]", result):
        raise ValueError(f"{name} cannot contain an absolute path")
    return result


def _assert_no_absolute_paths(value: object, name: str = "run_summary") -> None:
    if isinstance(value, Mapping):
        for key, item in value.items():
            _assert_no_absolute_paths(item, f"{name}.{key}")
    elif isinstance(value, (list, tuple)):
        for index, item in enumerate(value):
            _assert_no_absolute_paths(item, f"{name}[{index}]")
    elif isinstance(value, str):
        if PureWindowsPath(value).is_absolute() or PurePosixPath(value).is_absolute():
            raise ValueError(f"{name} contains an absolute path")
        if re.search(r"[A-Za-z]:[\\/]", value):
            raise ValueError(f"{name} contains an absolute path")


def build_run_summary(
    motion: Mapping[str, Any],
    preflight: Mapping[str, Any],
    *,
    motion_path: str,
    preflight_path: str,
    output_path: str,
    project_root: str | Path = ROOT,
    created_utc: datetime | None = None,
) -> dict[str, Any]:
    root = Path(project_root).resolve()
    pair = validate_evidence_pair(motion, preflight, project_root=root)
    timestamp = datetime.now(timezone.utc) if created_utc is None else created_utc
    if timestamp.tzinfo is None or timestamp.utcoffset() is None:
        raise ValueError("created_utc must be timezone-aware")
    timestamp = timestamp.astimezone(timezone.utc)

    backend = _portable_status(preflight.get("backend_execution_status"), "backend_execution_status")
    if backend not in BACKEND_EXECUTION_STATUSES:
        raise ValueError("unsupported backend_execution_status")
    isaac_performed = _boolean(preflight.get("isaac_validation_performed"), "isaac_validation_performed")
    if isaac_performed != (backend in {"PASS", "FAIL"}):
        raise ValueError("Isaac validation status is inconsistent with backend execution status")
    claims = _mapping(preflight.get("claims"), "preflight.claims")
    physical_execution = _portable_status(
        claims.get("physical_simulation_execution"), "claims.physical_simulation_execution"
    )
    if physical_execution != backend:
        raise ValueError("physical simulation claim does not match backend execution status")
    video_status = _portable_status(claims.get("video"), "claims.video")

    summary: dict[str, Any] = {
        "schema": SUMMARY_SCHEMA,
        "created_utc": timestamp.isoformat(timespec="seconds").replace("+00:00", "Z"),
        "environment": {
            "python_version": platform.python_version(),
            "python_implementation": platform.python_implementation(),
            "platform": platform.platform(),
        },
        "git": _git_metadata(root),
        "generation_commands": _standard_commands(motion_path, preflight_path, output_path),
        "evidence": {
            "motion": {
                "path": motion_path,
                "sha256": None,
                "evidence_fingerprint": pair["motion_evidence_fingerprint"],
            },
            "preflight": {
                "path": preflight_path,
                "sha256": None,
                "preflight_fingerprint": pair["preflight_fingerprint"],
            },
            "execution_asset_fingerprint_sha256": pair["execution_asset_fingerprint_sha256"],
            "aggregate_evidence_identity_sha256": pair["aggregate_evidence_identity_sha256"],
            "mutual_identity_status": "PASS",
        },
        "implementation_identity": {
            "motion_source_sha256": pair["motion_source_sha256"],
            "execution_source_sha256": pair["execution_source_sha256"],
            "archive_source_sha256": pair["archive_source_sha256"],
            "aggregate_source_identity_sha256": pair["aggregate_source_identity_sha256"],
        },
        "execution_status": {
            "preflight_status": _portable_status(preflight.get("status"), "preflight.status"),
            "simulation_execution_ready": _boolean(
                preflight.get("simulation_execution_ready"), "simulation_execution_ready"
            ),
            "execution_qualified": _boolean(preflight.get("execution_qualified"), "execution_qualified"),
            "machine_qualified": _boolean(preflight.get("machine_qualified"), "machine_qualified"),
            "isaac": {
                "backend_execution_status": backend,
                "validation_performed": isaac_performed,
            },
            "physical_simulation_execution": physical_execution,
            "video": video_status,
        },
    }
    _assert_no_absolute_paths(summary)
    return summary


def archive_dynamic_execution_evidence(
    motion_path: str | Path = DEFAULT_MOTION,
    preflight_path: str | Path = DEFAULT_PREFLIGHT,
    output_path: str | Path = DEFAULT_OUTPUT,
    *,
    project_root: str | Path = ROOT,
) -> dict[str, Any]:
    """Validate paired evidence and create a non-overwriting portable summary."""

    root = Path(project_root).resolve()
    motion_source = Path(motion_path).resolve()
    preflight_source = Path(preflight_path).resolve()
    destination = Path(output_path).resolve()
    motion_relative = _repo_relative(motion_source, root, "motion result")
    preflight_relative = _repo_relative(preflight_source, root, "preflight")
    output_relative = _repo_relative(destination, root, "output")
    if motion_source == preflight_source or destination in {motion_source, preflight_source}:
        raise ValueError("motion, preflight, and output paths must be distinct")
    if destination.exists():
        raise FileExistsError(f"refusing to overwrite evidence summary: {output_relative}")
    motion, motion_file_sha256 = _read_json_object(motion_source, "motion result")
    preflight, preflight_file_sha256 = _read_json_object(preflight_source, "preflight")
    summary = build_run_summary(
        motion,
        preflight,
        motion_path=motion_relative,
        preflight_path=preflight_relative,
        output_path=output_relative,
        project_root=root,
    )
    summary["evidence"]["motion"]["sha256"] = motion_file_sha256
    summary["evidence"]["preflight"]["sha256"] = preflight_file_sha256
    summary["summary_fingerprint"] = canonical_digest(summary)
    _assert_no_absolute_paths(summary)
    destination.parent.mkdir(parents=True, exist_ok=True)
    with destination.open("x", encoding="utf-8", newline="\n") as stream:
        json.dump(summary, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return summary


def verify_run_summary(summary: Mapping[str, Any]) -> dict[str, str]:
    if summary.get("schema") != SUMMARY_SCHEMA:
        raise ValueError("unsupported M-710 dynamic execution summary schema")
    content = copy.deepcopy(dict(summary))
    recorded = _sha256(content.pop("summary_fingerprint", None), "summary.summary_fingerprint")
    if canonical_digest(content) != recorded:
        raise ValueError("M-710 dynamic execution summary fingerprint mismatch")
    _assert_no_absolute_paths(summary)
    return {"status": "PASS", "summary_fingerprint": recorded}


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--motion-result", type=Path, default=DEFAULT_MOTION)
    parser.add_argument("--preflight", type=Path, default=DEFAULT_PREFLIGHT)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        summary = archive_dynamic_execution_evidence(
            args.motion_result,
            args.preflight,
            args.output,
        )
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "reason": str(exc)}, ensure_ascii=False))
        return 1
    print(
        json.dumps(
            {
                "status": "PASS",
                "output": _repo_relative(args.output, ROOT, "output"),
                "summary_fingerprint": summary["summary_fingerprint"],
                "mutual_identity_status": summary["evidence"]["mutual_identity_status"],
                "backend_execution_status": summary["execution_status"]["isaac"]["backend_execution_status"],
                "video": summary["execution_status"]["video"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = [
    "DEFAULT_MOTION",
    "DEFAULT_OUTPUT",
    "DEFAULT_PREFLIGHT",
    "SUMMARY_SCHEMA",
    "archive_dynamic_execution_evidence",
    "build_run_summary",
    "validate_evidence_pair",
    "verify_run_summary",
]

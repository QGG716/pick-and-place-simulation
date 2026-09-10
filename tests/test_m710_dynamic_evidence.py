from __future__ import annotations

import copy
from datetime import datetime, timezone
import json
from pathlib import Path, PurePosixPath, PureWindowsPath
import subprocess
import sys
from tempfile import TemporaryDirectory

import pytest
import tools.archive_m710id70_dynamic_execution_evidence as archive_module

from tools.archive_m710id70_dynamic_execution_evidence import (
    SUMMARY_SCHEMA,
    archive_dynamic_execution_evidence,
    validate_evidence_pair,
    verify_run_summary,
)
from unloading_sim.layout_single_carton import (
    run_layout_single_carton_audit,
)
from unloading_sim.m710_execution import (
    DEFAULT_CONFIG_PATH,
    build_m710_execution_preflight,
    load_m710_execution_config,
)
from unloading_sim.workcell_layout import canonical_digest, sha256_file


ROOT = Path(__file__).resolve().parents[1]
SOURCE_COPY_BASE_HEAD = "a372c7f117e61509229b2c15eef344cd7908d499"
SOURCE_COPY_BRANCH = "feat/v0.5-feasibility-core"


def _source_identity_arguments():
    # A full checkout follows its actual HEAD after future commits. The remote
    # upload has no complete Git database and explicitly declares its base.
    try:
        head = subprocess.run(
            ["git", "rev-parse", "--verify", "HEAD^{commit}"], cwd=ROOT,
            check=True, capture_output=True, text=True,
        ).stdout.strip()
        branch = subprocess.run(
            ["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=ROOT,
            check=True, capture_output=True, text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        head, branch = SOURCE_COPY_BASE_HEAD, SOURCE_COPY_BRANCH
    return {"source_base_head": head, "source_branch": branch}


def _motion_result() -> dict:
    execution = load_m710_execution_config(DEFAULT_CONFIG_PATH)
    return run_layout_single_carton_audit(execution.motion_policy_path, project_root=ROOT)


@pytest.fixture(scope="module")
def evidence_pair() -> tuple[dict, dict]:
    motion = _motion_result()
    assert motion["run_status"] == "BLOCKED"
    assert motion["complete_trajectory_failure_reason"] == "INITIAL_STATE_INVALID"
    preflight = build_m710_execution_preflight(
        DEFAULT_CONFIG_PATH,
        motion_result=motion,
        backend_execution_status="NOT_RUN_PER_USER_REQUEST",
    )
    return motion, preflight


def _contains_absolute_path(value: object) -> bool:
    if isinstance(value, dict):
        return any(_contains_absolute_path(item) for item in value.values())
    if isinstance(value, list):
        return any(_contains_absolute_path(item) for item in value)
    if isinstance(value, str):
        return PureWindowsPath(value).is_absolute() or PurePosixPath(value).is_absolute()
    return False


def test_archive_writes_portable_content_addressed_summary_with_git_and_runtime(evidence_pair):
    motion, preflight = evidence_pair
    with TemporaryDirectory(prefix=".tmp-m710-evidence-", dir=ROOT) as temporary:
        directory = Path(temporary)
        motion_path = directory / "motion.json"
        preflight_path = directory / "preflight.json"
        output_path = directory / "run_summary.json"
        motion_path.write_text(json.dumps(motion, ensure_ascii=False, indent=2), encoding="utf-8")
        preflight_path.write_text(json.dumps(preflight, ensure_ascii=False, indent=2), encoding="utf-8")

        summary = archive_dynamic_execution_evidence(motion_path, preflight_path, output_path, **_source_identity_arguments())
        persisted = json.loads(output_path.read_text(encoding="utf-8"))

        assert summary == persisted
        assert summary["schema"] == SUMMARY_SCHEMA
        assert datetime.fromisoformat(summary["created_utc"].replace("Z", "+00:00")).tzinfo == timezone.utc
        assert summary["environment"]["python_version"]
        assert summary["environment"]["platform"]
        assert len(summary["git"]["head"]) in {40, 64}
        assert isinstance(summary["git"]["dirty"], bool)
        assert summary["git"]["metadata_source"] in {"local_git", "declared_source_copy"}
        if summary["git"]["metadata_source"] == "declared_source_copy":
            assert summary["git"]["dirty"] is True
            identity = summary["implementation_identity"]
            assert summary["git"]["changed_paths"] == sorted(
                set(identity["motion_source_sha256"]) | set(identity["execution_source_sha256"]) | set(identity["archive_source_sha256"])
            )
        assert summary["git"]["changed_paths"] == sorted(summary["git"]["changed_paths"])
        assert all(not Path(item).is_absolute() for item in summary["git"]["changed_paths"])
        assert summary["evidence"]["motion"]["sha256"] == sha256_file(motion_path)
        assert summary["evidence"]["preflight"]["sha256"] == sha256_file(preflight_path)
        assert summary["evidence"]["mutual_identity_status"] == "PASS"
        assert len(summary["implementation_identity"]["aggregate_source_identity_sha256"]) == 64
        assert summary["execution_status"]["isaac"] == {
            "backend_execution_status": "NOT_RUN_PER_USER_REQUEST",
            "validation_performed": False,
        }
        assert summary["execution_status"]["video"] == "NOT_PRODUCED_WITHOUT_A_PHYSICAL_EXECUTION"
        assert not _contains_absolute_path(summary)
        assert str(ROOT) not in output_path.read_text(encoding="utf-8")
        assert verify_run_summary(summary)["status"] == "PASS"


def test_pair_validation_rejects_each_tamper_and_recomputed_cross_identity(evidence_pair):
    motion, preflight = evidence_pair

    tampered_motion = copy.deepcopy(motion)
    tampered_motion["run_status"] = "TAMPERED"
    with pytest.raises(ValueError, match="motion result evidence fingerprint mismatch"):
        validate_evidence_pair(tampered_motion, preflight)

    tampered_preflight = copy.deepcopy(preflight)
    tampered_preflight["status"] = "TAMPERED"
    with pytest.raises(ValueError, match="preflight fingerprint mismatch"):
        validate_evidence_pair(motion, tampered_preflight)

    different_motion = copy.deepcopy(motion)
    different_motion["scene_fingerprint"] = "1" * 64
    different_motion.pop("evidence_fingerprint")
    different_motion["evidence_fingerprint"] = canonical_digest(different_motion)
    with pytest.raises(ValueError, match="identity mismatch: motion fingerprint"):
        validate_evidence_pair(different_motion, preflight)


def test_pair_validation_rejects_stale_source_even_when_both_fingerprints_are_recomputed(evidence_pair):
    motion, preflight = copy.deepcopy(evidence_pair)
    path = "src/unloading_sim/layout_single_carton.py"
    motion["implementation_identity"]["source_sha256"][path] = "0" * 64
    motion.pop("evidence_fingerprint")
    motion["evidence_fingerprint"] = canonical_digest(motion)
    preflight["input_identity"]["motion_evidence_fingerprint"] = motion["evidence_fingerprint"]
    preflight["motion"] = copy.deepcopy(motion)
    assert preflight["replay_adapter_inputs"] is None
    preflight.pop("preflight_fingerprint")
    preflight["preflight_fingerprint"] = canonical_digest(preflight)

    with pytest.raises(ValueError, match="source hash mismatch"):
        validate_evidence_pair(motion, preflight)


def test_archive_cli_writes_once_without_starting_isaac_or_video(evidence_pair):
    motion, preflight = evidence_pair
    with TemporaryDirectory(prefix=".tmp-m710-evidence-cli-", dir=ROOT) as temporary:
        directory = Path(temporary)
        motion_path = directory / "motion.json"
        preflight_path = directory / "preflight.json"
        output_path = directory / "run_summary.json"
        motion_path.write_text(json.dumps(motion, ensure_ascii=False), encoding="utf-8")
        preflight_path.write_text(json.dumps(preflight, ensure_ascii=False), encoding="utf-8")
        command = [
            sys.executable,
            "tools/archive_m710id70_dynamic_execution_evidence.py",
            "--motion-result",
            str(motion_path),
            "--preflight",
            str(preflight_path),
            "--output",
            str(output_path),
            "--source-base-head", _source_identity_arguments()["source_base_head"],
            "--source-branch", _source_identity_arguments()["source_branch"],
        ]
        completed = subprocess.run(command, cwd=ROOT, check=True, capture_output=True, text=True)
        result = json.loads(completed.stdout)
        summary = json.loads(output_path.read_text(encoding="utf-8"))
        assert result["status"] == "PASS"
        assert result["output"].endswith("/run_summary.json")
        assert summary["execution_status"]["isaac"]["validation_performed"] is False
        assert not list(directory.glob("*.mp4"))

        repeated = subprocess.run(command, cwd=ROOT, check=False, capture_output=True, text=True)
        assert repeated.returncode == 1
        assert "refusing to overwrite" in json.loads(repeated.stdout)["reason"]


def test_source_copy_requires_both_fields_and_cannot_claim_clean(monkeypatch):
    def unavailable(*args, **kwargs):
        raise subprocess.CalledProcessError(128, "git")
    monkeypatch.setattr(archive_module, "_run_git", unavailable)
    with pytest.raises(ValueError, match="declare both"):
        archive_module._git_metadata(ROOT)
    for incomplete in (
        {"source_base_head": SOURCE_COPY_BASE_HEAD},
        {"source_branch": SOURCE_COPY_BRANCH},
    ):
        with pytest.raises(ValueError, match="requires both"):
            archive_module._git_metadata(ROOT, **incomplete)
    path = "tools/archive_m710id70_dynamic_execution_evidence.py"
    metadata = archive_module._git_metadata(
        ROOT, source_base_head=SOURCE_COPY_BASE_HEAD, source_branch=SOURCE_COPY_BRANCH,
        participating_source_paths=[path],
    )
    assert metadata["metadata_source"] == "declared_source_copy"
    assert metadata["head"] == SOURCE_COPY_BASE_HEAD
    assert metadata["branch"] == SOURCE_COPY_BRANCH
    assert metadata["dirty"] is True
    assert metadata["changed_paths"] == [path]


def test_declared_source_head_cannot_override_readable_git_head(monkeypatch):
    monkeypatch.setattr(archive_module, "_run_git", lambda *args, **kwargs: "b" * 40)
    with pytest.raises(ValueError, match="does not match actual Git HEAD"):
        archive_module._git_metadata(
            ROOT, source_base_head=SOURCE_COPY_BASE_HEAD, source_branch=SOURCE_COPY_BRANCH,
            participating_source_paths=["tools/archive_m710id70_dynamic_execution_evidence.py"],
        )


def test_complete_git_metadata_remains_authoritative(monkeypatch):
    monkeypatch.setattr(archive_module, "_run_git", lambda *args, **kwargs: SOURCE_COPY_BASE_HEAD)
    metadata = {"metadata_source": "local_git", "head": SOURCE_COPY_BASE_HEAD,
                "branch": SOURCE_COPY_BRANCH, "dirty": False, "changed_paths": []}
    monkeypatch.setattr(archive_module, "_local_git_metadata", lambda root: metadata)
    assert archive_module._git_metadata(
        ROOT, source_base_head=SOURCE_COPY_BASE_HEAD, source_branch=SOURCE_COPY_BRANCH,
    ) == metadata
    with pytest.raises(ValueError, match="does not match actual Git branch"):
        archive_module._git_metadata(ROOT, source_base_head=SOURCE_COPY_BASE_HEAD, source_branch="wrong-branch")


def test_incomplete_git_index_requires_explicit_source_copy_identity(monkeypatch):
    monkeypatch.setattr(archive_module, "_run_git", lambda *args, **kwargs: SOURCE_COPY_BASE_HEAD)
    def incomplete(*args, **kwargs):
        raise subprocess.CalledProcessError(128, "git diff")
    monkeypatch.setattr(archive_module, "_local_git_metadata", incomplete)
    with pytest.raises(ValueError, match="declare both"):
        archive_module._git_metadata(ROOT)
    metadata = archive_module._git_metadata(
        ROOT, source_base_head=SOURCE_COPY_BASE_HEAD, source_branch=SOURCE_COPY_BRANCH,
        participating_source_paths=["tools/archive_m710id70_dynamic_execution_evidence.py"],
    )
    assert metadata["metadata_source"] == "declared_source_copy"
    assert metadata["dirty"] is True

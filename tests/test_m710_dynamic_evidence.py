from __future__ import annotations

import copy
from datetime import datetime, timezone
import json
from pathlib import Path, PurePosixPath, PureWindowsPath
import subprocess
import sys
from tempfile import TemporaryDirectory

import pytest

from tools.archive_m710id70_dynamic_execution_evidence import (
    SUMMARY_SCHEMA,
    archive_dynamic_execution_evidence,
    validate_evidence_pair,
    verify_run_summary,
)
from unloading_sim.layout_single_carton import (
    RESULT_SCHEMA as MOTION_RESULT_SCHEMA,
    build_verified_motion_input,
    load_layout_motion_policy,
    motion_implementation_identity,
)
from unloading_sim.m710_execution import (
    DEFAULT_CONFIG_PATH,
    build_m710_execution_preflight,
    load_m710_execution_config,
)
from unloading_sim.m710_replay_contract import build_replay_input_binding
from unloading_sim.workcell_layout import canonical_digest, sha256_file


ROOT = Path(__file__).resolve().parents[1]


def _motion_result() -> dict:
    execution = load_m710_execution_config(DEFAULT_CONFIG_PATH)
    policy = load_layout_motion_policy(execution.motion_policy_path)
    scene = build_verified_motion_input(policy, ROOT)
    result = {
        "schema": MOTION_RESULT_SCHEMA,
        "run_status": "COMPLETED",
        "layout_id": scene.snapshot["layout_id"],
        "layout_fingerprint": scene.snapshot["layout_fingerprint"],
        "scene_fingerprint": scene.snapshot["scene_fingerprint"],
        "policy_fingerprint": policy.policy_fingerprint,
        "implementation_identity": motion_implementation_identity(ROOT),
        "task_population": {"carton_ids": list(scene.removable_cartons)},
        "statistics": {
            "task_count": 5,
            "task_success_count": 0,
            "candidate_pose_attempts": 48,
            "coverage_rejected": 34,
            "ik_calls": 14,
            "complete_trajectory_success_count": 0,
        },
        "complete_trajectory_status": "FAIL_CLOSED",
        "complete_trajectory_failure_reason": "EXECUTION_COLLISION_GEOMETRY_NOT_QUALIFIED",
    }
    result["evidence_fingerprint"] = canonical_digest(result)
    return result


@pytest.fixture(scope="module")
def evidence_pair() -> tuple[dict, dict]:
    motion = _motion_result()
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

        summary = archive_dynamic_execution_evidence(motion_path, preflight_path, output_path)
        persisted = json.loads(output_path.read_text(encoding="utf-8"))

        assert summary == persisted
        assert summary["schema"] == SUMMARY_SCHEMA
        assert datetime.fromisoformat(summary["created_utc"].replace("Z", "+00:00")).tzinfo == timezone.utc
        assert summary["environment"]["python_version"]
        assert summary["environment"]["platform"]
        assert len(summary["git"]["head"]) in {40, 64}
        assert isinstance(summary["git"]["dirty"], bool)
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
    preflight["motion"]["evidence_fingerprint"] = motion["evidence_fingerprint"]
    preflight["execution_asset_fingerprint_sha256"] = canonical_digest(preflight["input_identity"])
    adapter = preflight["replay_adapter_inputs"]
    adapter["plan_common"]["motion_evidence_fingerprint"] = motion["evidence_fingerprint"]
    adapter["plan_common"]["execution_asset_fingerprint_sha256"] = preflight[
        "execution_asset_fingerprint_sha256"
    ]
    adapter["input_binding"] = build_replay_input_binding(
        plan_common=adapter["plan_common"],
        configuration=adapter["configuration"],
        scene_primitives=preflight["scene"]["primitives"],
        trajectory_segment=adapter["trajectory_segment"],
        trajectory_segment_status=adapter["trajectory_segment_status"],
        input_identity=preflight["input_identity"],
        execution_asset_fingerprint_sha256=preflight[
            "execution_asset_fingerprint_sha256"
        ],
    )
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

from __future__ import annotations

import json
import hashlib
from types import SimpleNamespace

import pytest

from tools import continue_m710_contact_unloading as continuation


def test_request_is_complete_and_never_overwrites_previous_plan(tmp_path):
    path = tmp_path / "request.json"
    continuation.publish_request(path, {"bundle_path": "first"})
    with pytest.raises(FileExistsError):
        continuation.publish_request(path, {"bundle_path": "replacement"})
    assert json.loads(path.read_text()) == {"bundle_path": "first"}
    assert [item.name for item in tmp_path.iterdir()] == ["request.json"]


@pytest.mark.parametrize("returncode", [0, 1, 2])
def test_actual_state_drives_next_plan_and_failure_stops_without_deleting_boxes(monkeypatch, tmp_path, returncode):
    actual = tmp_path / "actual.json"
    actual.write_text(json.dumps({"cartons": [{"name": "remaining"}], "world_session_id": "same-world"}))
    ready = tmp_path / "segment_002_ready.json"
    request = tmp_path / "segment_002_request.json"
    ready.write_text(json.dumps({"status": "WORLD_RETAINED_AWAITING_OFFLINE_NEXT_PLAN",
        "completed_segments": 1, "world_session_id": "same-world", "request_path": str(request),
        "actual_state_sha256": hashlib.sha256(actual.read_bytes()).hexdigest(),
        "actual_state_path": str(actual)}))
    output = tmp_path / "next"
    def plan(command, **kwargs):
        assert command[command.index("--actual-state") + 1] == str(actual)
        assert kwargs["check"] is False
        if returncode == 0:
            (output / "replay_bundle.json").write_text("{}")
        return SimpleNamespace(returncode=returncode)
    monkeypatch.setattr(continuation.subprocess, "run", plan)
    assert continuation.main(["--ready-file", str(ready), "--output", str(output)]) == returncode
    if returncode == 1:
        assert not request.exists()
        failure = json.loads((output / "continuation_failure.json").read_text())
        assert failure["status"] == "OFFLINE_PLANNING_FAILED_WORLD_RETAINED"
        assert failure["request_published"] is False
        assert json.loads(actual.read_text())["cartons"] == [{"name": "remaining"}]
        return
    payload = json.loads(request.read_text())
    assert payload["world_session_id"] == "same-world"
    assert json.loads(actual.read_text())["cartons"] == [{"name": "remaining"}]
    if returncode:
        assert payload["stop"] is True and payload["reason"] == "ROW_BLOCKED"
    else:
        assert payload["bundle_path"] == str(output / "replay_bundle.json")


@pytest.mark.parametrize("changed", ["before", "during"])
def test_changed_actual_state_never_publishes_history_bundle(monkeypatch, tmp_path, changed):
    actual = tmp_path / "actual.json"
    actual.write_text(json.dumps({"world_session_id": "same", "q_rad": [0]*6}))
    request = tmp_path / "segment_002_request.json"
    ready = tmp_path / "ready.json"
    ready.write_text(json.dumps({"status": "WORLD_RETAINED_AWAITING_OFFLINE_NEXT_PLAN",
        "completed_segments": 1, "world_session_id": "same", "request_path": str(request),
        "actual_state_path": str(actual), "actual_state_sha256": hashlib.sha256(actual.read_bytes()).hexdigest()}))
    calls = []
    def plan(command, **kwargs):
        calls.append(command)
        assert "--history-source" in command
        assert all("reuse_m710_motion.py" not in part for part in command)
        actual.write_text(json.dumps({"world_session_id": "same", "q_rad": [.1]*6}))
        return SimpleNamespace(returncode=0)
    monkeypatch.setattr(continuation.subprocess, "run", plan)
    if changed == "before":
        actual.write_text("{}")
    with pytest.raises((ValueError, RuntimeError), match="identity disagree|state changed"):
        continuation.main(["--ready-file", str(ready), "--output", str(tmp_path / "out"),
                           "--history-source", str(tmp_path / "hints")])
    assert not request.exists()
    assert len(calls) == (0 if changed == "before" else 1)

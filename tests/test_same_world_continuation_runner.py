from __future__ import annotations

import json
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


@pytest.mark.parametrize("returncode", [0, 2])
def test_actual_state_drives_next_plan_and_failure_stops_without_deleting_boxes(monkeypatch, tmp_path, returncode):
    actual = tmp_path / "actual.json"
    actual.write_text(json.dumps({"cartons": [{"name": "remaining"}]}))
    ready = tmp_path / "segment_002_ready.json"
    request = tmp_path / "segment_002_request.json"
    ready.write_text(json.dumps({"status": "WORLD_RETAINED_AWAITING_OFFLINE_NEXT_PLAN",
        "completed_segments": 1, "world_session_id": "same-world", "request_path": str(request),
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
    payload = json.loads(request.read_text())
    assert payload["world_session_id"] == "same-world"
    assert json.loads(actual.read_text())["cartons"] == [{"name": "remaining"}]
    if returncode:
        assert payload["stop"] is True and payload["reason"] == "ROW_BLOCKED"
    else:
        assert payload["bundle_path"] == str(output / "replay_bundle.json")

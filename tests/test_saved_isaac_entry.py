"""The saved-task entry cannot change the reference or start repeated segments."""
from copy import deepcopy
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import pytest

spec = importlib.util.spec_from_file_location("saved_entry", Path(__file__).parents[1] / "tools/run_saved_curobo_isaac.py")
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


@pytest.mark.parametrize("damage", [None, "timestamps_seconds", "positions_rad", "joint_reference", "initial_actual_state_context"])
def test_reference_is_exact_not_just_old_pass(damage):
    saved = dict(timestamps_seconds=[0, 1], positions_rad=[[0], [1]],
                 metadata=dict(joint_names=["J1"], joint_reference={"q": [0, 1]},
                               target="carton", initial_actual_state_context={"attached": False}))
    current = deepcopy(saved)
    if damage:
        (current if damage in current else current["metadata"])[damage] = [99]
        with pytest.raises(ValueError):
            entry.require_same_reference(saved, current)
    else:
        entry.require_same_reference(saved, current)


def test_real_entry_single_world_normal_time_recording(tmp_path):
    args = SimpleNamespace(isaac_python=Path("isaac-python"), output=tmp_path,
                           reuse_usd_entrypoint=Path("robot.usda"), reuse_usd_run_evidence=Path("run.json"),
                           reuse_usd_source_contract=Path("contract.json"))
    cmd = entry.isaac_command(args, tmp_path, tmp_path / "bundle.json")
    assert Path(cmd[1]).name == "isaacsim_fanuc_replay.py"
    assert cmd[-5:] == ["--record-video", "--video-preview-speed", "1", "--maximum-segments", "1"]
    assert "--continuation-dir" not in cmd


def test_archived_counts_are_not_new_world_completions():
    from unloading_sim.qualification import reception_counts
    from unloading_sim.post_landing_transport import RECEPTION_SOURCE, OUTFED
    records = {name: dict(completion_source=RECEPTION_SOURCE, state=OUTFED) for name in ("old", "new")}
    result = reception_counts([], records, exclude_ids=["old"])
    assert result["ideal_reception"] == result["ideal_outfeed"] == 1
    assert result["actual_reception"] == 0
    assert set(records) == {"old", "new"}


def test_venv_interpreter_identity_must_not_follow_symlink(tmp_path):
    base = tmp_path / "base-python"
    base.write_text("base executable")
    venv = tmp_path / "venv-python"
    venv.symlink_to(base)
    assert entry.absolute_input_path("isaac_python", venv) == venv
    assert entry.absolute_input_path("motion", venv) == base


def test_import_failure_timestamp_does_not_claim_a_world(tmp_path):
    (tmp_path / "run_status.json").write_text('{"status":"started","started_unix_s":123}')
    assert entry.observed_world_id(tmp_path) is None
    (tmp_path / "initialized_actual_state.json").write_text('{"world_session_id":"measured-new-world"}')
    assert entry.observed_world_id(tmp_path) == "measured-new-world"

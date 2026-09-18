"""Exercise the production snapshot without importing the optional Isaac runtime."""
import ast
import json
from pathlib import Path
from types import SimpleNamespace as NS

import numpy as np
import pytest


@pytest.mark.parametrize("render_failure,advance", [(False, False), (True, False), (False, True)])
def test_capture_records_zero_time_or_secondary_failure(tmp_path, render_failure, advance):
    tree = ast.parse((Path(__file__).parents[1] / "scripts/isaacsim_fanuc_replay.py").read_text(encoding="utf-8"))
    node = next(n for n in ast.walk(tree) if isinstance(n, ast.FunctionDef)
                and n.name == "_capture_delivery_snapshot")
    world = NS(current_time=0.9)
    calls = []
    writes = []

    def render(**kwargs):
        calls.append(kwargs)
        if render_failure:
            raise RuntimeError("render unavailable")
        if advance:
            world.current_time += 1 / 240

    env = dict(np=np, json=json, world=world, run_started_unix_s=123,
               metadata={"target": "carton_a"}, args=NS(output=tmp_path, width=640, height=360),
               articulation=NS(get_dof_positions=lambda: NS(numpy=lambda: np.zeros((1, 6)))),
               _capture_carton_states=lambda: [{"name": "carton_a", "center_m": [0, 0, 1]}],
               rep=NS(orchestrator=NS(step=render)),
               rgb_annotator=NS(get_data=lambda: np.zeros((360, 640, 4), dtype=np.uint8)),
               cv2=NS(COLOR_RGB2BGR=1, cvtColor=lambda x, _: x,
                      imwrite=lambda path, pixels: writes.append(path) or True))
    exec(compile(ast.Module(body=[node], type_ignores=[]), "snapshot", "exec"), env)
    ok = env["_capture_delivery_snapshot"]("failure.png", "pregrasp", 0.0)
    evidence = json.loads((tmp_path / "failure.png.json").read_text())
    assert calls[0]["delta_time"] == 0.0
    assert ok == (not render_failure and not advance)
    assert bool(writes) == ok
    assert ("secondary_capture_error" in evidence) == (not ok)
    assert evidence["world_session_id"] == "123"
    assert evidence["target"] == "carton_a"
    if ok:
        assert evidence["physics_time_delta_s"] == 0
        assert evidence["carton_state_unchanged"]

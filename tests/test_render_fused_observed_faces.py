from __future__ import annotations

import json
from pathlib import Path

from tools.render_fused_observed_faces import render


def test_render_fused_observed_faces_writes_world_views(tmp_path: Path) -> None:
    source = tmp_path / "fused.json"
    output = tmp_path / "views.png"
    source.write_text(json.dumps({
        "coverage_status": "COMPLETE_MODULE_SET",
        "objects": [{
            "contributing_modules": ["module_0_upper", "module_1_lower"],
            "observed_faces": [{
                "corners_3d_m": [
                    [-1.0, -0.3, 0.2], [-0.6, -0.3, 0.2],
                    [-0.6, -0.3, 0.5], [-1.0, -0.3, 0.5],
                ],
            }],
        }],
    }), encoding="utf-8")

    result = render(source, output)

    assert result == {
        "object_count": 1,
        "observed_face_count": 1,
        "coverage_status": "COMPLETE_MODULE_SET",
        "output": str(output.resolve()),
    }
    assert output.stat().st_size > 1000

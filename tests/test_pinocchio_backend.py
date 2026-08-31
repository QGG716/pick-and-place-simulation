from pathlib import Path

import numpy as np
import pytest

from unloading_sim.pinocchio_backend import PinocchioHppFclBackend


def test_optional_pinocchio_backend_fails_with_actionable_message_when_unavailable(tmp_path):
    urdf = tmp_path / "robot.urdf"
    urdf.write_text("<robot name='empty'/>", encoding="utf-8")
    try:
        import pinocchio  # noqa: F401
        import coal  # noqa: F401
    except ImportError:
        with pytest.raises(RuntimeError, match="Pinocchio|required|Coal"):
            PinocchioHppFclBackend(urdf, tip_frame="tool")
    else:
        root = Path(__file__).parents[1]
        assets = root / "assets" / "robots" / "fanuc_m20id35"
        backend = PinocchioHppFclBackend(
            assets / "m20_35_18d.urdf",
            tip_frame="tool0",
            package_dirs=[assets],
            srdf_path=assets / "m20_35_18d.srdf",
        )
        assert backend.dof == 6
        assert backend.collision_pair_filter == "srdf"
        assert np.all(np.isfinite(backend.fk(np.zeros(6))))
        assert not backend.collision_result(np.zeros(6), []).in_collision

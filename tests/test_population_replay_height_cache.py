"""Regression for a real multi-layer replay cache alias, not planning tolerance."""
import importlib.util
from pathlib import Path
import sys


def test_adjacent_native_heights_do_not_share_replay_geometry():
    tools = Path(__file__).resolve().parents[1]/"tools"
    sys.path.insert(0, str(tools))
    try:
        spec = importlib.util.spec_from_file_location("population_replay_export", tools/"export_m710_top_layer_replay.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        a, b = 1.75, 1.7499999999999998
        assert format(a, ".12f") == format(b, ".12f")
        assert module.mechanism_key(a) != module.mechanism_key(b)
        assert float.fromhex(module.mechanism_key(a)) == a
        assert float.fromhex(module.mechanism_key(b)) == b
    finally:
        sys.path.remove(str(tools))

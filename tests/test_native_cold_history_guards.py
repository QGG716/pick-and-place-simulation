"""Strict native requests must fail before any historical solution is read."""
from types import SimpleNamespace

import pytest

from unloading_sim import history_adaptation


class _UnreadableHistory:
    def __getitem__(self, key):
        raise AssertionError("historical solution was read")


@pytest.mark.parametrize("flag", ["native_cold", "require_native_motion"])
@pytest.mark.parametrize("entry", [
    "solve_local", "adapt_branch", "free_loaded_hint",
    "loaded_prefix_geometry", "loaded_suffix", "checked_departure",
])
def test_strict_history_entries_reject_before_reading_solution(flag, entry):
    connector = SimpleNamespace(**{flag: True})
    hint = _UnreadableHistory()
    calls = {
        "solve_local": lambda: history_adaptation.solve_local(connector, None, None),
        "adapt_branch": lambda: history_adaptation.adapt_branch(
            connector, hint=hint, target=None, face=None,
            requested_virtual_contact=None, grasp_q=None, home_q=None,
            all_obstacles=None, receiver=None, support_names=None, suction=None, seed=0),
        "free_loaded_hint": lambda: history_adaptation.free_loaded_hint(
            connector, hint, None, None, None),
        "loaded_prefix_geometry": lambda: history_adaptation.loaded_prefix_geometry(
            connector, hint, None, None, None, None),
        "loaded_suffix": lambda: history_adaptation.loaded_suffix(
            connector, hint, None, None, None, None, None, None),
        "checked_departure": lambda: history_adaptation.checked_departure(
            connector, hint, None, None, None, None, None),
    }
    with pytest.raises(ValueError, match="NATIVE_COLD_HISTORY_ADAPTATION_FORBIDDEN"):
        calls[entry]()


def test_legacy_connector_keeps_explicit_history_permission():
    history_adaptation._require_history_allowed(SimpleNamespace())
    history_adaptation._require_history_allowed(
        SimpleNamespace(native_cold=False, require_native_motion=False))

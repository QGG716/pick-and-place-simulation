"""Request accounting only; synthetic data is not performance evidence."""
import importlib.util
from pathlib import Path
import pytest

spec = importlib.util.spec_from_file_location("retest_summary",
    Path(__file__).resolve().parents[1]/"tools/summarize_m710_top_row_retest.py")
entry = importlib.util.module_from_spec(spec)
spec.loader.exec_module(entry)


def test_exact_duplicates_and_distinct_ik_seeds():
    a = dict(request_id="1", batch_seed_index=0, request_round_trip_s=.3)
    b = dict(request_id="1", batch_seed_index=1, request_round_trip_s=0.)
    rows, duplicates = entry.unique_records([a, a.copy(), b], ("request_id", "batch_seed_index"))
    assert rows == [a, b]
    assert duplicates == 1
    assert sum(r["request_round_trip_s"] for r in rows) == .3


def test_conflicting_snapshots_cannot_be_replaced_by_last_or_max():
    a = dict(request_id="1", queries=3)
    b = dict(request_id="1", queries=5)
    with pytest.raises(ValueError, match="conflicting"):
        entry.unique_records([a, b], ("request_id",))

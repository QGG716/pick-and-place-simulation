from __future__ import annotations

from dataclasses import dataclass

from unloading_sim.online_journal import BoundedEventJournal


@dataclass(frozen=True)
class Event:
    sequence: int


def test_bounded_event_journal_preserves_sequence_and_reports_history_gap():
    journal = BoundedEventJournal[Event](3)
    for sequence in range(1, 7):
        journal.append(Event(sequence))

    assert [event.sequence for event in journal] == [4, 5, 6]
    assert journal.summary()["dropped_count"] == 3
    read = journal.events_since(1)
    assert read.history_gap
    assert read.overflowed
    assert read.oldest_available_sequence == 4
    assert read.latest_sequence == 6
    assert [event.sequence for event in read.events] == [4, 5, 6]


def test_bounded_event_journal_incremental_limit_and_sequence_validation():
    journal = BoundedEventJournal[Event](4)
    journal.append(Event(10))
    journal.append(Event(11))
    journal.append(Event(12))
    read = journal.events_since(10, limit=1)
    assert not read.history_gap
    assert [event.sequence for event in read.events] == [11]

    try:
        journal.append(Event(12))
    except ValueError as exc:
        assert "monotonically" in str(exc)
    else:
        raise AssertionError("duplicate journal sequence should be rejected")

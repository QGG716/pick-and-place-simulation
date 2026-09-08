"""Bounded provider-neutral event journals for long-running control planes."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Generic, Iterator, Protocol, Sequence, TypeVar, overload


class SequencedEvent(Protocol):
    sequence: int


EventT = TypeVar("EventT", bound=SequencedEvent)


@dataclass(frozen=True)
class EventJournalRead(Generic[EventT]):
    events: tuple[EventT, ...]
    oldest_available_sequence: int | None
    latest_sequence: int | None
    history_gap: bool
    overflowed: bool


class BoundedEventJournal(Sequence[EventT], Generic[EventT]):
    """A list-compatible bounded journal with explicit history-gap reporting."""

    def __init__(self, capacity: int) -> None:
        if not isinstance(capacity, int) or capacity < 1:
            raise ValueError("event journal capacity must be a positive integer")
        self.capacity = capacity
        self._events: deque[EventT] = deque()
        self.dropped_count = 0
        self.overflow_count = 0

    def append(self, event: EventT) -> None:
        if self._events and event.sequence <= self._events[-1].sequence:
            raise ValueError("event journal sequence must increase monotonically")
        if len(self._events) >= self.capacity:
            self._events.popleft()
            self.dropped_count += 1
            self.overflow_count += 1
        self._events.append(event)

    def events_since(
        self,
        sequence: int,
        limit: int | None = None,
    ) -> EventJournalRead[EventT]:
        if sequence < 0:
            raise ValueError("event sequence cursor must be non-negative")
        if limit is not None and (not isinstance(limit, int) or limit < 1):
            raise ValueError("event read limit must be a positive integer")
        oldest = None if not self._events else self._events[0].sequence
        latest = None if not self._events else self._events[-1].sequence
        gap = bool(oldest is not None and sequence < oldest - 1)
        selected = tuple(event for event in self._events if event.sequence > sequence)
        if limit is not None:
            selected = selected[:limit]
        return EventJournalRead(
            selected,
            oldest,
            latest,
            gap,
            self.dropped_count > 0,
        )

    def summary(self) -> dict[str, int | bool | None]:
        return {
            "capacity": self.capacity,
            "retained": len(self._events),
            "oldest_available_sequence": (
                None if not self._events else self._events[0].sequence
            ),
            "latest_sequence": None if not self._events else self._events[-1].sequence,
            "dropped_count": self.dropped_count,
            "overflow_count": self.overflow_count,
            "overflowed": self.dropped_count > 0,
        }

    def __len__(self) -> int:
        return len(self._events)

    def __iter__(self) -> Iterator[EventT]:
        return iter(self._events)

    @overload
    def __getitem__(self, index: int) -> EventT: ...

    @overload
    def __getitem__(self, index: slice) -> Sequence[EventT]: ...

    def __getitem__(self, index: int | slice) -> EventT | Sequence[EventT]:
        events = tuple(self._events)
        return events[index]


__all__ = ["BoundedEventJournal", "EventJournalRead", "SequencedEvent"]

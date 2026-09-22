"""Process-local, bounded execution context admission; no ROS dependencies."""
from __future__ import annotations

from dataclasses import dataclass
import math


CONTEXT_SCHEMA_VERSION = "1.2.0"


def positive_age_limit(value: float, name: str) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive")
    return value


def context_source_error(observed: float, *, now: float, clock_domain: str,
                         max_age_seconds: float) -> str | None:
    limit = positive_age_limit(max_age_seconds, "context_source_max_age_seconds")
    if clock_domain != "ros":
        return "EXECUTION_CONTEXT_CLOCK_DOMAIN_MISMATCH"
    if not math.isfinite(observed) or observed <= 0:
        return "EXECUTION_CONTEXT_SOURCE_TIME_INVALID"
    if not math.isfinite(now) or now <= 0:
        return "EXECUTION_CONTEXT_CLOCK_NOT_INITIALIZED"
    if observed > now:
        return "EXECUTION_CONTEXT_SOURCE_TIME_IN_FUTURE"
    if now - observed > limit:
        return "EXECUTION_CONTEXT_SOURCE_STALE"
    return None


@dataclass(frozen=True)
class _AcceptedContext:
    publisher: str
    sequence: int
    business: tuple[str, str]
    authority: tuple[str, str, int, str, str]
    observed: float


class ExecutionContextGuard:
    """Validate everything before committing ownership, sequence or retirement.

    Unlike SourceEpochGuard's rolling history, execution ownership must never
    forget a retired identity. Capacity exhaustion refuses further transitions;
    heartbeats from the current owner remain possible. No persistent replay or
    authentication guarantee is made.
    """

    def __init__(self, *, retired_capacity: int = 128) -> None:
        if type(retired_capacity) is not int or retired_capacity <= 0:
            raise ValueError("retired capacity must be a positive integer")
        self.retired_capacity = retired_capacity
        self.current: _AcceptedContext | None = None
        self._retired_publishers: set[str] = set()
        self._retired_business: set[tuple[str, str]] = set()

    def accept(self, message, *, observed: float, now: float,
               max_age_seconds: float) -> bool:
        """Return whether old grants/reservations must be revoked."""
        if message.schema_version != CONTEXT_SCHEMA_VERSION:
            raise ValueError("EXECUTION_CONTEXT_SCHEMA_MISMATCH")
        for field in ("session_id", "epoch", "publisher_epoch"):
            if not isinstance(getattr(message, field), str) or not getattr(message, field).strip():
                raise ValueError(f"EXECUTION_CONTEXT_INVALID_{field.upper()}")
        for field in ("allowed_plan_id", "predecessor_plan_id"):
            if not isinstance(getattr(message, field), str):
                raise ValueError(f"EXECUTION_CONTEXT_INVALID_{field.upper()}")
        for field in ("planning_generation", "publisher_sequence"):
            value = getattr(message, field)
            if type(value) is not int or not 0 <= value < 2**64:
                raise ValueError(f"EXECUTION_CONTEXT_INVALID_{field.upper()}")
        if type(message.publisher_restart) is not bool:
            raise ValueError("EXECUTION_CONTEXT_INVALID_PUBLISHER_RESTART")
        reason = context_source_error(observed, now=now, clock_domain=message.clock_domain,
                                      max_age_seconds=max_age_seconds)
        if reason:
            raise ValueError(reason)
        candidate = _AcceptedContext(
            message.publisher_epoch, message.publisher_sequence,
            (message.session_id, message.epoch),
            (message.session_id, message.epoch, message.planning_generation,
             message.allowed_plan_id, message.predecessor_plan_id), observed)
        previous = self.current
        if message.publisher_restart and candidate.sequence != 0:
            raise ValueError("EXECUTION_CONTEXT_RESTART_REQUIRES_SEQUENCE_ZERO")
        if candidate.publisher in self._retired_publishers:
            raise ValueError("EXECUTION_CONTEXT_RETIRED_PUBLISHER")
        if candidate.business in self._retired_business:
            raise ValueError("EXECUTION_CONTEXT_RETIRED_EXECUTION")
        publisher_changed = previous is not None and candidate.publisher != previous.publisher
        business_changed = previous is not None and candidate.business != previous.business
        if previous is not None:
            if publisher_changed:
                if not message.publisher_restart or candidate.sequence != 0:
                    raise ValueError("EXECUTION_CONTEXT_EXPLICIT_RESTART_REQUIRED")
                if len(self._retired_publishers) >= self.retired_capacity:
                    raise ValueError("EXECUTION_CONTEXT_PUBLISHER_HISTORY_FULL")
            elif message.publisher_restart or candidate.sequence <= previous.sequence:
                raise ValueError("EXECUTION_CONTEXT_PUBLISHER_SEQUENCE_NOT_INCREASING")
            if business_changed:
                if len(self._retired_business) >= self.retired_capacity:
                    raise ValueError("EXECUTION_CONTEXT_EXECUTION_HISTORY_FULL")
            elif candidate.authority[2] < previous.authority[2]:
                raise ValueError("EXECUTION_CONTEXT_GENERATION_ROLLBACK")
            if observed <= previous.observed:
                raise ValueError("EXECUTION_CONTEXT_SOURCE_TIME_NOT_INCREASING")
        # No state above this point is mutated, including on failed takeover.
        if publisher_changed:
            self._retired_publishers.add(previous.publisher)
        if business_changed:
            self._retired_business.add(previous.business)
        self.current = candidate
        return previous is not None and (publisher_changed or candidate.authority != previous.authority)

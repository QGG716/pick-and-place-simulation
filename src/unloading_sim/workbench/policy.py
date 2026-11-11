"""Hard stage-one boundary. Legacy execution configuration cannot enable control."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol
import json
from pathlib import Path

from . import MODE


class ReadOnlyViolation(PermissionError):
    pass


@dataclass(frozen=True)
class ObservationConfig:
    robot_host: str = ""
    robot_port: int = 8080
    serial_port: str = ""
    camera_serial: str = ""
    max_age_s: float = 1.0
    mode: str = MODE

    def __post_init__(self):
        if self.mode != MODE:
            raise ReadOnlyViolation("Only READ_ONLY_STAGE_1 is available")
        if not 1 <= self.robot_port <= 65535 or not 0 < self.max_age_s <= 60:
            raise ValueError("Invalid port or maximum sample age")


def load_config(path):
    """Data-only whitelist; never import or instantiate a legacy GUI/controller."""
    raw = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if not isinstance(raw, dict):
        raise ValueError("Configuration must be an object")
    fields = {k: raw[k] for k in ("robot_host", "robot_port", "serial_port",
                                "camera_serial", "max_age_s") if k in raw}
    ignored = sorted(set(raw)-set(fields))
    return ObservationConfig(**fields), ignored


class ExecutionPort(Protocol):
    """SI trajectory boundary with model/frame provenance.

    The default stage-one backend registers DisabledExecution; the explicit
    controlled entry registers ExecutionService. Do not pass a vendor handle
    through this interface or equate preview time with physical progress.
    """
    def submit(self, trajectory: dict) -> str: ...
    def feedback(self, task_id: str) -> dict: ...
    def cancel(self, task_id: str) -> None: ...


class DisabledExecution:
    def submit(self, trajectory):
        raise ReadOnlyViolation("Physical trajectory execution is unavailable in stage one")

    def feedback(self, task_id):
        raise ReadOnlyViolation("No physical execution task exists")

    def cancel(self, task_id):
        # Critically, this does not call a robot stop callback on alarms or shutdown.
        raise ReadOnlyViolation("No physical execution task exists")

"""Versioned JSON wire encoding for shared contracts (never pickle)."""

from __future__ import annotations

from dataclasses import fields, is_dataclass
from enum import Enum
import json
from typing import Any

from . import models


def to_wire(value: Any) -> Any:
    if isinstance(value, Enum):
        return {"__enum__": type(value).__name__, "value": value.value}
    if is_dataclass(value):
        return {
            "__type__": type(value).__name__,
            **{item.name: to_wire(getattr(value, item.name)) for item in fields(value) if item.init},
        }
    if isinstance(value, dict) or hasattr(value, "items"):
        return {str(key): to_wire(item) for key, item in value.items()}
    if isinstance(value, (tuple, list, frozenset)):
        return [to_wire(item) for item in value]
    return value


def from_wire(value: Any) -> Any:
    if isinstance(value, list):
        return tuple(from_wire(item) for item in value)
    if not isinstance(value, dict):
        return value
    if "__enum__" in value:
        enum_type = getattr(models, value["__enum__"])
        return enum_type(value["value"])
    if "__type__" in value:
        type_name = value["__type__"]
        contract_type = getattr(models, type_name)
        kwargs = {key: from_wire(item) for key, item in value.items() if key != "__type__"}
        return contract_type(**kwargs)
    return {str(key): from_wire(item) for key, item in value.items()}


def dumps(value: Any) -> str:
    return json.dumps({"schema_version": models.SCHEMA_VERSION, "payload": to_wire(value)}, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def loads(payload: str) -> Any:
    envelope = json.loads(payload)
    if envelope.get("schema_version") != models.SCHEMA_VERSION:
        raise ValueError("unsupported wire schema version")
    return from_wire(envelope["payload"])

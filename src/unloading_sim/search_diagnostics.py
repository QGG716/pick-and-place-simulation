"""Bounded, observational evidence from the production validation chain."""
from __future__ import annotations

from collections import Counter
import hashlib
import json
from pathlib import Path


def _json(value):
    if hasattr(value, "tolist"):
        return value.tolist()
    if isinstance(value, set):
        return sorted(value)
    raise TypeError(type(value).__name__)


class SearchDiagnostics:
    def __init__(self, path=None, *, identity=None, examples_per_group=3):
        self.path = None if path is None else Path(path)
        self.identity = dict(identity or {})
        self.examples_per_group = examples_per_group
        self.groups = {}
        self.contexts = {}
        self.events = Counter()
        self.candidate = {}
        self.validation_queries = Counter()
        self.last_validation = None
        self.query_count = 0
        self.outcomes = {}

    def observe(self, stage, origin, failed):
        self.query_count += 1
        key = json.dumps([origin, stage, "REJECTED" if failed else "ACCEPTED"])
        self.validation_queries[key] += 1
        changed = self.last_validation is None or self.last_validation["stage"] != stage
        self.last_validation = dict(stage=stage, origin=origin, failed=failed)
        if changed or self.query_count % 1024 == 0:
            self.flush()

    def bind_candidate(self, scheduler, **identity):
        self.candidate = {**scheduler, **identity}

    def event(self, item):
        self.events[str(item.get("event", "progress"))] += 1
        self.last_event = dict(item)
        failure = item.get("failure")
        reason = failure.get("reason") if isinstance(failure, dict) else failure
        if reason or item.get("termination") or item.get("event") in {"CANCELLED", "ERROR"}:
            key = json.dumps([item.get("event"), item.get("stage"), reason or item.get("termination")])
            group = self.outcomes.setdefault(key, {"count": 0, "examples": []})
            group["count"] += 1
            if len(group["examples"]) < self.examples_per_group:
                group["examples"].append({"candidate": dict(self.candidate), **dict(item)})
        self.flush()

    def reject(self, failure, q, *, origin, edge, context):
        pair = tuple(failure.get("pair", failure.get("collider_pair", ())))
        key = json.dumps([origin, failure.get("stage"), failure.get("reason"),
                          failure.get("classification"), pair], default=_json)
        group = self.groups.setdefault(key, {"count": 0, "examples": []})
        group["count"] += 1
        if len(group["examples"]) < self.examples_per_group:
            value = context()
            raw = json.dumps(value, sort_keys=True, default=_json)
            fingerprint = hashlib.sha256(raw.encode()).hexdigest()
            self.contexts[fingerprint] = json.loads(raw)
            group["examples"].append(json.loads(json.dumps({
                "candidate": self.candidate, "q_rad": q, "failure": failure,
                "origin": origin, "edge": edge, "context": fingerprint}, default=_json)))
            self.flush()

    def flush(self):
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(json.dumps({"schema": "production_search_diagnostics_v1",
            "identity": self.identity, "events": self.events,
            "validation_queries": self.validation_queries,
            "last_validation": self.last_validation,
            "outcomes": self.outcomes,
            "last_event": getattr(self, "last_event", None),
            "groups": self.groups, "contexts": self.contexts}, default=_json), encoding="utf-8")
        temporary.replace(self.path)

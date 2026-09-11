"""Differential checks for event-preserving contact callback cost reduction."""
from __future__ import annotations

import ast
from enum import IntEnum
from pathlib import Path
import random
import time
from types import SimpleNamespace

import numpy as np
import pytest

from unloading_sim.isaac_collision_policy import ActiveContactPairIndex, ContactPathCache, ContactReportProbe, read_effective_collision_offsets, verify_effective_collision_offsets, verify_authored_collision_offsets, colliders_by_physical_body, expand_owned_tool_wrist_pairs, PhysicalContactLedger, physical_support_contact_observed, robot_proximity_is_safety_relevant, premature_physical_conveyor_contacts, placement_support_window_start


class LegacyIndex:
    def __init__(self, headers):
        self.headers = headers

    def update(self, key, *, lost):
        if lost:
            self.headers.discard(key)
        else:
            self.headers.add(key)
        return sum(item[:2] == key[:2] for item in self.headers)


def test_active_contact_pair_index_matches_global_scan_after_every_event():
    rng = random.Random(7432)
    keys = [(f"/actor/{pair}", "/target", f"/shape/{pair}/{shape}", "/target/shape")
            for pair in range(17) for shape in range(23)]
    initial = {keys[0], keys[-1]}
    actual, expected = set(initial), set(initial)
    index, legacy = ActiveContactPairIndex(actual), LegacyIndex(expected)
    # Duplicate FOUND/PERSIST, LOST-before-FOUND and duplicate LOST are all exact.
    events = [(keys[3], True), (keys[0], False), (keys[0], False),
              (keys[0], True), (keys[0], True)]
    events += [(rng.choice(keys), rng.randrange(3) == 2) for _ in range(5000)]
    events += [(key, True) for key in keys]
    for key, lost in events:
        assert index.update(key, lost=lost) == legacy.update(key, lost=lost)
        assert actual == expected
    assert actual == set()


def test_contact_path_cache_resolves_every_distinct_id_once():
    calls = []
    cache = ContactPathCache(lambda value: calls.append(value) or f"/World/{value}")
    for value in [1, 2, 1, np.uint64(2), 3, 1]:
        assert cache.resolve(value) == f"/World/{int(value)}"
    assert calls == [1, 2, 3]


class Event(IntEnum):
    CONTACT_FOUND = 1
    CONTACT_PERSIST = 2
    CONTACT_LOST = 3


def test_actual_callback_preserves_every_aggregate_and_impulse_against_legacy():
    """Execute the real adapter callback without importing Isaac or starting App."""
    script = Path(__file__).resolve().parents[1] / "scripts/isaacsim_fanuc_replay.py"
    callback = next(node for node in ast.walk(ast.parse(script.read_text(encoding="utf-8")))
                    if isinstance(node, ast.FunctionDef) and node.name == "_on_contact_report")
    compiled = compile(ast.Module(body=[callback], type_ignores=[]), str(script), "exec")
    paths = {0: "/robot/J1", 1: "/robot/J2", 2: "/target", 3: "/floor", 4: "/other"}
    paths.update({number: f"/shape/{number}" for number in range(10, 50)})
    namespaces = []
    for factory in (ActiveContactPairIndex, LegacyIndex):
        active = set()
        namespace = {
            "time": time, "np": np, "ContactEventType": Event,
            "root_prim_path": "/robot", "target_carton_path": "/target",
            "released_payload_path": None, "contact_pairs": {},
            "active_contact_headers": active, "active_contacts": factory(active),
            "contact_path_cache": ContactPathCache(paths.__getitem__),
            "contact_probe": ContactReportProbe() if factory is ActiveContactPairIndex else None,
            "physical_contact_ledger": PhysicalContactLedger(.003),
            "contact_clock_s": [0.0], "contact_callback_wall_s": [0.0],
            "contact_trajectory_clock_s": [0.0],
            "contact_callback_header_count": [0],
            "_classify_runtime_contact": lambda *args, **kwargs: None,
        }
        exec(compiled, namespace)
        namespaces.append(namespace)
    rng = random.Random(9993)
    samples = [(rng.randrange(5), rng.randrange(5), rng.randrange(10, 50),
                rng.randrange(10, 50)) for _ in range(100)]
    events = [(samples[0], Event.CONTACT_LOST),
              (samples[1], Event.CONTACT_FOUND), (samples[1], Event.CONTACT_PERSIST),
              (samples[1], Event.CONTACT_LOST), (samples[1], Event.CONTACT_LOST)]
    events += [(rng.choice(samples), rng.choice(list(Event))) for _ in range(600)]
    for step, ((actor0, actor1, collider0, collider1), event_type) in enumerate(events):
        header = SimpleNamespace(actor0=actor0, actor1=actor1, collider0=collider0,
                                 collider1=collider1, type=event_type,
                                 contact_data_offset=0, num_contact_data=2)
        impulses = [SimpleNamespace(impulse=np.array([1., -2., step * .001])),
                    SimpleNamespace(impulse=np.array([2., 1., .5]))]
        for namespace in namespaces:
            namespace["contact_clock_s"][0] = step / 240
            namespace["contact_trajectory_clock_s"][0] = max(0, step - 5) / 240
            namespace["_on_contact_report"]([header], impulses)
        assert namespaces[0]["contact_pairs"] == namespaces[1]["contact_pairs"]
        assert namespaces[0]["active_contact_headers"] == namespaces[1]["active_contact_headers"]
    assert namespaces[0]["contact_callback_header_count"] == [len(events)]
    assert sum(record["event_count"] for record in namespaces[0]["contact_pairs"].values()) > 0


def test_probe_keeps_zero_point_headers_distinct_from_zero_impulse_contact_points():
    probe = ContactReportProbe()
    first = SimpleNamespace(actor0=1, actor1=2, type=Event.CONTACT_FOUND,
                            num_contact_data=0, contact_data_offset=0)
    second = SimpleNamespace(actor0=1, actor1=2, type=Event.CONTACT_PERSIST,
                             num_contact_data=2, contact_data_offset=0)
    points = [SimpleNamespace(separation=0.4, impulse=[0., 0., 0.]),
              SimpleNamespace(separation=-0.001, impulse=[3., 4., 0.])]
    probe.observe([first, second], points, lambda value: f"/actor/{value}")
    result = probe.as_dict()
    assert result["header_count"] == 2
    assert result["physics_or_event_filtering_changed"] is False
    assert result["event_point_count_distribution"] == {
        "CONTACT_FOUND:num_contact_data=0": 1, "CONTACT_PERSIST:num_contact_data=2": 1}
    row = result["actor_pairs"][0]
    assert row["zero_point_headers"] == 1 and row["point_count"] == 2
    assert row["minimum_separation_m"] == -0.001
    assert row["maximum_separation_m"] == 0.4
    assert row["nonzero_impulse_points"] == 1 and row["maximum_point_impulse_ns"] == 5.


def test_effective_offsets_are_actual_getter_values_not_sdk_default_sentinels():
    view = SimpleNamespace(get_contact_offsets=lambda: np.array([[0.02, 2.0]]),
                           get_rest_offsets=lambda: np.array([[0.0, 0.0]]))
    result = read_effective_collision_offsets(view)
    assert result["contact_offsets_m"] == [[0.02, 2.0]]
    assert result["rest_offsets_m"] == [[0.0, 0.0]]
    assert result["read_only"] is True


def test_probe_reads_the_existing_articulation_identifier():
    script = Path(__file__).resolve().parents[1] / "scripts/isaacsim_fanuc_replay.py"
    source = script.read_text(encoding="utf-8")
    assert "articulation._physics_articulation_view.get_link_transforms()" in source
    assert "robot._physics_articulation_view" not in source


def test_effective_offset_gate_is_exact_at_the_backend_float32_representation():
    values = np.full((1, 209), .01, dtype=np.float32).tolist()
    evidence = {"contact_offsets_m": values, "rest_offsets_m": np.zeros((1, 209)).tolist()}
    policy = dict(contact_offset_m=.01, rest_offset_m=0., expected_shape_count=209)
    assert verify_effective_collision_offsets(evidence, **policy)["status"] == "PASS"
    evidence["contact_offsets_m"][0][17] = 19.6200008392334
    with pytest.raises(ValueError, match="contact_offsets_m"):
        verify_effective_collision_offsets(evidence, **policy)
    evidence["contact_offsets_m"][0][17] = float(np.nextafter(np.float32(.01), np.float32(.02)))
    with pytest.raises(ValueError, match="contact_offsets_m"):
        verify_effective_collision_offsets(evidence, **policy)


def test_effective_offset_gate_rejects_missing_shape_and_nonzero_rest():
    evidence = {"contact_offsets_m": np.full((1, 40), .01, dtype=np.float32).tolist(),
                "rest_offsets_m": np.zeros((1, 40)).tolist()}
    with pytest.raises(ValueError):
        verify_effective_collision_offsets(evidence, contact_offset_m=.01, rest_offset_m=0., expected_shape_count=41)
    evidence["rest_offsets_m"][0][1] = .0001
    with pytest.raises(ValueError, match="rest_offsets_m"):
        verify_effective_collision_offsets(evidence, contact_offset_m=.01, rest_offset_m=0., expected_shape_count=40)


def test_authored_offsets_verify_static_colliders_and_newton_override():
    values = {"physxCollision:contactOffset": np.float32(.01).item(), "physxCollision:restOffset": 0.,
              "newton:contactGap": np.float32(.01).item(), "newton:contactMargin": 0.}
    prim = SimpleNamespace(GetAttribute=lambda name: SimpleNamespace(
        Get=lambda: values[name], HasAuthoredValueOpinion=lambda: True))
    stage = SimpleNamespace(GetPrimAtPath=lambda path: prim)
    records = [{"collider": "/static_deck", "contact_offset_m": .01, "rest_offset_m": 0., "newton_contact_gap_m": .01}]
    assert verify_authored_collision_offsets(stage, records)["collider_count"] == 1
    values["newton:contactGap"] = 19.62
    with pytest.raises(ValueError, match="authored collider offset mismatch"):
        verify_authored_collision_offsets(stage, records)


def test_nested_robot_links_assign_only_nearest_rigid_body_and_keep_both_wrist_filters():
    links = {"base_link": "/robot/base", "J5_link": "/robot/base/J5",
             "J6_link": "/robot/base/J5/J6"}
    meshes = [path + "/mesh" for path in links.values()]
    grouped = colliders_by_physical_body(links, meshes + ["/robot/base_lookalike/mesh", "/scene/floor"])
    assert grouped == {name: [path + "/mesh"] for name, path in links.items()}
    assert sum(map(len, grouped.values())) == 3
    tools = {"/robot/base/J5/J6/owned_tool": "wantai"}
    assert expand_owned_tool_wrist_pairs(grouped, tools, tool_owner="wantai") == [
        ("/robot/base/J5/J6/mesh", "/robot/base/J5/J6/owned_tool"),
        ("/robot/base/J5/mesh", "/robot/base/J5/J6/owned_tool"),
    ]


def test_physical_layer_rejects_20mm_support_but_keeps_robot_proximity_unsafe():
    ledger = PhysicalContactLedger(.003)
    key = ("/payload", "/receiver/TopCollision", "/payload", "/receiver/TopCollision")
    record = {}
    ledger.observe(
        key, [.020], lost=False, time_s=0., trajectory_time_s=0., record=record
    )
    assert not physical_support_contact_observed(ledger.active_headers, "/payload", {"/receiver"})
    assert record["physical_contact_observed"] is False
    assert premature_physical_conveyor_contacts([record], place_start_s=1., time_tolerance_s=.01) == []
    assert robot_proximity_is_safety_relevant({"actor0": "/robot/J6", "actor1": "/payload",
                                               "physical_contact_observed": False, "peak_impulse_ns": 0.}, "/robot")
    for separation in (0., -.001):
        ledger.observe(
            key,
            [separation],
            lost=False,
            time_s=.7,
            trajectory_time_s=.5,
            record=record,
        )
        assert physical_support_contact_observed(ledger.active_headers, "/payload", {"/receiver"})
    # Classification uses the trajectory clock, not a physical clock that may
    # include bounded waits.  Contact during PLACE is legal from its start.
    assert premature_physical_conveyor_contacts(
        [record], place_start_s=.5, time_tolerance_s=.01
    ) == []
    assert premature_physical_conveyor_contacts(
        [record], place_start_s=.52, time_tolerance_s=.01
    ) == [record]
    ledger.observe(key, [.02], lost=False, time_s=.6, record=record)
    assert not physical_support_contact_observed(ledger.active_headers, "/payload", {"/receiver"})
    assert record["physical_contact_observed"] is True  # historical evidence retained


def test_receiver_contact_window_starts_with_place_not_release_arrival():
    metadata = {
        "release_arrival_time_seconds": 105.09,
        "stage_windows": [
            {"stage": "transit", "start_time_s": 40.0, "end_time_s": 100.34},
            {"stage": "place", "start_time_s": 100.34, "end_time_s": 105.09},
        ],
    }
    assert placement_support_window_start(metadata, 105.19) == pytest.approx(100.34)
    assert premature_physical_conveyor_contacts(
        [{"physical_contact_observed": True}],
        place_start_s=100.34,
        time_tolerance_s=.01,
    )  # missing same-clock evidence fails closed


def test_physical_layer_lost_and_missing_data_remove_only_that_current_shape():
    ledger = PhysicalContactLedger(.003)
    first = ("/payload", "/receiver", "/payload/a", "/receiver/a")
    second = ("/payload", "/receiver", "/payload/b", "/receiver/b")
    record = {}
    ledger.observe(first, [0.], lost=False, time_s=0., record=record)
    ledger.observe(second, [.001], lost=False, time_s=0., record=record)
    ledger.observe(first, [], lost=True, time_s=.1, record=record)
    assert ledger.active_headers == {second}
    ledger.observe(second, [], lost=False, time_s=.2, record=record)
    assert not ledger.active_headers
    assert record["minimum_separation_m"] == 0. and record["maximum_separation_m"] == .001
    assert record["first_physical_contact_time_s"] == 0. and record["last_physical_contact_time_s"] == 0.

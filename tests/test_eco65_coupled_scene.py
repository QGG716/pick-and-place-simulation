"""Directed geometric tests for the 180 mm ECO65 coupled receiver."""
from copy import deepcopy
import numpy as np
import pytest

from unloading_sim.eco65.coupled_scene import (make_coupled_spec, coupled_conveyor_state,
    build_coupled_scene, with_coupled_state, receiving_height_candidates, _hull)
from unloading_sim.eco65.model import transform
from unloading_sim.moveit2_backend import digest


def source_scene():
    # Literal measured dimensions from run61; the fixture is independent of
    # the mechanism construction and contains D-baked source belt vertices.
    tb = [[-.18, -.20215986078540135, .17132526492684583],
          [.02, .24215986078540133, .18]]
    lb = [[-.37431972157080273, -.44031446711489125, .17132526492684583],
          [.07, -.2403144671148912, .18]]
    objects, items = [], []
    for owner, bounds in (("conveyor_transverse", tb), ("conveyor_longitudinal", lb)):
        vertices = [[x, y, z] for x in (bounds[0][0], bounds[1][0])
                    for y in (bounds[0][1], bounds[1][1]) for z in (bounds[0][2], bounds[1][2])]
        objects.append(dict(id=owner+"_belt", owner=owner, role="conveyor_belt",
                            pose=np.eye(4).tolist(), convex_mesh=_hull(vertices)))
        objects.append(dict(id=owner+"_old_support", owner=owner, role="conveyor_structure",
                            size=[.1, .1, .1], pose=transform([0, 0, .05]).tolist()))
        items.append(dict(id=owner, belt_bounds_D_m=bounds))
    result = dict(conveyors=dict(items=items), native_world_objects=objects,
                  obstacles=[dict(id="robot_mount_plate", size_m=[.14, .14, .24],
                                  position_m=[-.34, 0, .12]),
                             dict(id="junction_transition_plate", size_m=[.2, .038, .003],
                                  position_m=[-.08, -.22, .1785])],
                  mesh_obstacles=[dict(id=x) for x in ("conveyor_transverse", "conveyor_longitudinal")],
                  native_task={}, scene_fingerprint="source-fixture", boxes=[],
                  box=dict(id="carton_r00_l03_c01", pose=[.205, 0, .53], size_m=[.15, .1, .1]))
    return result


def vertices(row):
    if "size" in row:
        half = np.asarray(row["size"])/2
        local = np.array([[x, y, z] for x in (-half[0], half[0])
                          for y in (-half[1], half[1]) for z in (-half[2], half[2])])
    else:
        local = np.asarray(row["convex_mesh"]["vertices"])
    T = np.asarray(row["pose"])
    return local @ T[:3, :3].T+T[:3, 3]


@pytest.fixture
def spec():
    return make_coupled_spec(source_scene())


@pytest.mark.parametrize("height", [.18, .33, .43, .45])
def test_real_dimensions_rigid_belts_and_effective_seam(spec, height):
    state = coupled_conveyor_state(spec, height)
    m = state["mechanism"]
    upper, lower = np.asarray(m["upper_endpoint_D_m"]), np.asarray(m["lower_endpoint_D_m"])
    assert upper == pytest.approx([-.18, -.2, height], abs=1e-12)
    assert lower[2] == pytest.approx(.18)
    assert np.linalg.norm(upper-lower) == pytest.approx(.4443197215708027)
    assert m["seam"]["gap_m"] == 0
    assert m["seam"]["ymin_m"] == pytest.approx(-.3)
    assert state["receiving"]["bounds_D_m"][0][:2] == pytest.approx([-.18, -.3])
    assert state["receiving"]["bounds_D_m"][1][:2] == pytest.approx([.02, .1443197215708027])
    for stack in m["stacks"]:
        for bar in stack["bars"]:
            assert np.linalg.norm(np.diff(bar["endpoints_m"], axis=0)) == pytest.approx(bar["length_m"])
        for i in range(stack["stages"]):
            lanes = [bar["lane_offset_m"] for bar in stack["bars"] if f"_arm_{i}_" in bar["id"]]
            assert abs(lanes[0]-lanes[1]) == pytest.approx(.008)


def test_feet_and_rails_fixed_but_heads_and_carriages_move(spec):
    low, high = [coupled_conveyor_state(spec, h) for h in (.18, .43)]
    a, b = [{o["id"]: o for o in s["world_objects"]} for s in (low, high)]
    assert set(a) == set(b)
    for name in low["mechanism"]["fixed_structure_ids"]:
        assert a[name]["pose"] == b[name]["pose"], name
        assert a[name].get("size") == b[name].get("size")
    assert b["eco65_lower_rail_left_carriage"]["pose"][0][3] > a["eco65_lower_rail_left_carriage"]["pose"][0][3]
    assert b["eco65_lift_transverse_left_head_rail"]["pose"][2][3] > a["eco65_lift_transverse_left_head_rail"]["pose"][2][3]
    assert b["eco65_lift_longitudinal_near_swivel_head"]["pose"] != a["eco65_lift_longitudinal_near_swivel_head"]["pose"]
    low_contact = low["mechanism"]["stacks"][-1]["frame_contact_local_m"]
    high_contact = high["mechanism"]["stacks"][-1]["frame_contact_local_m"]
    assert abs(low_contact[0]-high_contact[0]) > .01
    for s in (low, high):
        for row in s["world_objects"]:
            assert vertices(row)[:, 2].min() >= -1e-10, row["id"]


@pytest.mark.parametrize('height', [.18, .33, .43, .45])
def test_rebuilding_coupled_scene_never_converts_local_belts_a_second_time(height):
    original = build_coupled_scene(source_scene())
    rebuilt = build_coupled_scene(with_coupled_state(original, height))
    rebuilt_again = build_coupled_scene(rebuilt)
    for scene in (original, rebuilt, rebuilt_again):
        state = coupled_conveyor_state(scene['coupled_conveyor_spec'], height)
        assert state['world_objects'] == coupled_conveyor_state(original['coupled_conveyor_spec'], height)['world_objects']
        belts = [r for r in state['world_objects'] if r['role'] == 'conveyor_belt']
        assert len(belts) == 2
        for row in belts:
            v = vertices(row)
            assert v[:, 1].min() == pytest.approx(-.3, abs=1e-9)
            frame = next(r for r in state['world_objects']
                         if r['owner'] == row['owner'] and r['role'] == 'conveyor_frame')
            assert vertices(frame)[:, 1].min() == pytest.approx(v[:, 1].min(), abs=1e-9)
        assert scene['mesh_obstacles'] == []


def test_rebuilding_previously_corrupted_local_templates_is_rejected():
    scene = build_coupled_scene(source_scene())
    spec = scene['coupled_conveyor_spec']
    for row in spec['belt_templates']:
        row['convex_mesh']['vertices'] = (np.array(row['convex_mesh']['vertices']) + [0., -.097840139, 0.]).tolist()
    spec['fingerprint'] = digest({k: v for k, v in spec.items() if k != 'fingerprint'})
    with pytest.raises(ValueError, match='ECO65_BELT_NOT_SURFACE_CENTRED_LOCAL'):
        build_coupled_scene(scene)


def test_frames_and_belt_solids_use_nonpenetrating_seam_plane(spec):
    slope, xmin = spec["seam_relief_slope"], spec["transverse_xmin_m"]
    for h in np.linspace(.18, .45, 10):
        state = coupled_conveyor_state(spec, h)
        for row in state["world_objects"]:
            if row["owner"] not in ("conveyor_transverse", "conveyor_longitudinal"):
                continue
            v = vertices(row)
            signed = v[:, 0]-xmin+slope*(v[:, 2]-h)
            if row["owner"] == "conveyor_transverse":
                assert signed.min() >= -1e-10
            else:
                assert signed.max() <= 1e-10


def test_old_supports_removed_and_dynamic_world_has_distinct_identity():
    source = source_scene()
    scene = build_coupled_scene(source)
    high = with_coupled_state(scene, .43)
    assert high["scene_fingerprint"] == scene["scene_fingerprint"]
    assert high["coupled_state"]["scene_fingerprint"] != scene["coupled_state"]["scene_fingerprint"]
    assert not high["mesh_obstacles"]
    assert {o["id"] for o in high["obstacles"]} == {"robot_mount_plate"}
    assert not any(o["id"].endswith("old_support") for o in high["native_world_objects"])
    assert source["mesh_obstacles"] and len(source["native_world_objects"]) == 4
    assert len(high["native_world_objects"]) == len({o["id"] for o in high["native_world_objects"]})
    assert with_coupled_state(high, .18)["native_world_objects"] == scene["native_world_objects"]


def test_receiving_candidates_are_frozen_from_initial_box_and_within_travel(spec):
    candidates = receiving_height_candidates(spec, source_scene()["box"])
    assert [x["height_m"] for x in candidates] == pytest.approx([.43, .33])
    assert all(x["in_hardware_range"] for x in candidates)
    # New longitudinal footprint clears the unchanged robot mounting plate.
    low = coupled_conveyor_state(spec, .18)
    long_parts = [o for o in low["world_objects"] if o["owner"] == "conveyor_longitudinal"]
    assert max(vertices(row)[:, 1].max() for row in long_parts) == pytest.approx(-.1)
    assert -.07 - max(vertices(row)[:, 1].max() for row in long_parts) == pytest.approx(.03)


def test_low_endpoint_bearing_remains_connected_at_each_height(spec):
    for h in (.18, .33, .43, .45):
        state = coupled_conveyor_state(spec, h)
        rows = {o["id"]: o for o in state["world_objects"]}
        lower = np.asarray(state["mechanism"]["lower_endpoint_D_m"])
        for side in ("left", "right"):
            base = "eco65_lower_rail_"+side
            carriage = vertices(rows[base+"_carriage"])
            post = vertices(rows[base+"_bearing_post"])
            housing = vertices(rows[base+"_bearing_housing"])
            assert carriage[:, 2].max() == pytest.approx(post[:, 2].min())
            assert post[:, 2].max() == pytest.approx(housing[:, 2].min())
            assert rows[base+"_bearing_housing"]["pose"][0][3] == pytest.approx(lower[0])
            assert rows[base+"_revolute_pin"]["pose"][2][3] == pytest.approx(.18)
        assert len(state["mechanism"]["lower_hinges"]) == 2


def test_out_of_range_and_tampered_spec_rejected(spec):
    with pytest.raises(ValueError, match="HEIGHT_OUT_OF_RANGE"):
        coupled_conveyor_state(spec, .451)
    bad = deepcopy(spec)
    bad["longitudinal_length_m"] = 2.8
    with pytest.raises(ValueError, match="IDENTITY"):
        coupled_conveyor_state(bad, .33)


def test_hull_compacts_interior_and_collinear_vertices_for_native_fcl():
    corners = [[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]
    mesh = _hull(corners + [[0, 0, 0], [0, 0, 1], [.5, 0, 1], [1, 0, 1]])
    assert len(mesh["vertices"]) == 8
    assert set(np.asarray(mesh["triangles"]).ravel()) == set(range(len(mesh["vertices"])))

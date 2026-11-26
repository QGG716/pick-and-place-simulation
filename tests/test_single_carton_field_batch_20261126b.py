"""Field batch 2026-11-26 evening (tasks 1a5d6d89 .. cc2546c8, ef4773e6,
37188dbb, f96dd018): fixed release point with the contact-derived tool-down
orientation, 0.10 m outward detach clearance, loaded-return veto back for
loaded runs, in-face cup-grid shifts of the single contact, JPEG upload.
Fake data only."""
from types import SimpleNamespace
import io
import numpy as np
import pytest
from unloading_sim.workbench.single_carton import release_pose as rp
from unloading_sim.workbench.single_carton.planner import SinglePlanner, detach_outward_clearance, DETACH_OUTWARD_CLEARANCE_M
from unloading_sim.workbench.single_carton.geometry import contact_search_settings
from unloading_sim.workbench.single_carton.remote_detector import encode_image, IMAGE_ENCODINGS
from unloading_sim.workbench.single_carton.remote_detector_server import decode_png

BASIS = 'SYNTHETIC'


def test_release_pose_fixed_point_validation():
    ok = dict(mode=rp.MODE, basis=BASIS, position_xy_m=[-.073, -.576], start_height_m=.4843, position_basis=BASIS)
    assert rp.policy(dict(release_pose=ok))['position_xy_m'] == [-.073, -.576]
    for bad in (dict(ok, start_height_m=None), dict(ok, position_xy_m=[1.]), {k: v for k, v in ok.items() if k != 'start_height_m'},
                dict(ok, position_basis='')):
        with pytest.raises(ValueError):
            rp.policy(dict(release_pose=bad))


def test_release_orientation_turns_the_contact_pose_down_without_a_turn_about_z():
    # contact: tool z into the face (-X), cup-array long side along +Y
    R = np.column_stack(([0, 1., 0], [0, 0, -1.], [-1., 0, 0]))
    G = rp.oriented_goal([-.073, -.576, .4843], R, .23)
    assert np.allclose(G[:3, 2], [0, 0, -1]) and np.allclose(G[:3, 0], [0, 1, 0])     # long side stays along Y
    assert np.allclose(G[:3, 3], [-.073, -.576, .2543])
    # a start rotated half a turn about the tool axis does not change it (field 16425baa)
    assert rp.cache_key([-.073, -.576], R, .4843) == rp.cache_key([-.073, -.576, 9.], R, .4843)


def test_detach_outward_clearance_default_and_bounds():
    assert DETACH_OUTWARD_CLEARANCE_M == .01 and detach_outward_clearance({}) == .01
    assert detach_outward_clearance(dict(detach_outward_clearance_m=.05)) == .05
    for bad in (0., .5, True, 'x'):
        with pytest.raises(ValueError):
            detach_outward_clearance(dict(detach_outward_clearance_m=bad))
    from unloading_sim.workbench.single_carton.geometry import corners
    actual = np.eye(4); P = np.eye(4); P[:3, 3] = [-.7, 0, .35]
    box = dict(size_m=[.2, .13, .11], pose=P.tolist(), corners_m=corners(P, [.2, .13, .11]).tolist())
    goals = dict(SinglePlanner.detach_goals(actual, box, dict(face=dict(normal=[1., 0, 0])), .10))
    assert abs(goals['OUTWARD'][0, 3] - .30) < 1e-12                                      # largest extent + 0.10 m


def test_loaded_return_record_only_needs_a_dry_run():
    record = dict(mode='RECORD_ONLY', basis=BASIS)
    dry = dict(mode='FULL_TRAJECTORY_NO_VACUUM', basis=BASIS)
    check = lambda site, stage='return': SinglePlanner.loaded_return_record_only(SimpleNamespace(site=site), stage)
    assert check(dict(loaded_return_collision=record, dry_run_no_vacuum=dry))
    assert not check(dict(loaded_return_collision=record))                                   # loaded: veto again
    assert not check(dict(loaded_return_collision=record, dry_run_no_vacuum=dict(mode='DISABLED', basis=BASIS)))
    assert not check(dict(loaded_return_collision=record, dry_run_no_vacuum=dry), 'detach')


def test_single_contact_in_face_shift_option_validation():
    base = dict(reference_face_roll_deg=90, j6_rotation_deg=180, lift_offsets_m=[0.], max_candidates=1, basis=BASIS,
                single_contact=dict(mode='FACE_CENTER_LONG_SIDE_ALONG_BASE_Y', basis=BASIS, long_side_toward='+Y'))
    assert contact_search_settings(base)
    shifted = dict(base, single_contact=dict(base['single_contact'], in_face_shift='CUP_GRID', in_face_shift_basis=BASIS))
    assert contact_search_settings(shifted)['single_contact']['in_face_shift'] == 'CUP_GRID'
    for bad in (dict(in_face_shift='CUP_GRID'), dict(in_face_shift='ANY', in_face_shift_basis=BASIS)):
        with pytest.raises(ValueError):
            contact_search_settings(dict(base, single_contact=dict(base['single_contact'], **bad)))


def test_jpeg_upload_is_smaller_and_the_server_decodes_it():
    rng = np.random.default_rng(1)
    rgb = (np.clip(rng.normal(128, 25, (480, 640, 3)), 0, 255)).astype(np.uint8)
    rgb[100:300, 200:400] = [180, 140, 90]
    png, _ = encode_image(rgb, 'PNG'); jpg, digest = encode_image(rgb, 'JPEG95')
    assert IMAGE_ENCODINGS == ('PNG', 'JPEG95') and len(jpg) < len(png)
    back = decode_png(jpg)
    assert back.shape == rgb.shape and back.dtype == np.uint8
    assert np.abs(back.astype(int) - rgb.astype(int)).mean() < 12
    assert encode_image(rgb, 'JPEG95')[1] == digest                                         # deterministic (server cache key)
    with pytest.raises(ValueError):
        encode_image(rgb, 'WEBP')

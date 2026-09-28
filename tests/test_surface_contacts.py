"""Synthetic measured planes only; these tests are not real SAM acceptance."""
from copy import deepcopy
from pathlib import Path
from types import SimpleNamespace
import importlib.util

import numpy as np
import pytest

from unloading_contracts import canonical_fingerprint
from unloading_sim.geometry import make_transform, rotation_matrix_from_rpy
from unloading_sim.grasp import SuctionGraspCandidate, suction_cup_layout_geometry
from unloading_perception.metric_faces import _binding, _support_runs
from unloading_perception.surface_contacts import (current_tool, face_candidates, prepare_support,
                                                  representative_surfaces, validate_tool)

ROOT=Path(__file__).resolve().parents[1]


def fixture(*, rotation=None, width=.85, height=.85, fault=None):
    depth=np.full((300,300),1.5,np.float32);mask=np.ones(depth.shape,bool)
    K=np.array([[400.,0,150],[0,400,150],[0,0,1]])
    p=np.array([[-width/2,-height/2,1.5],[width/2,-height/2,1.5],
                [width/2,height/2,1.5],[-width/2,height/2,1.5]])
    T=make_transform(np.eye(3) if rotation is None else rotation,[.2,-.1,.4])
    n=T[:3,:3]@np.array([0.,0.,-1.]);world=p@T[:3,:3].T+T[:3,3]
    uv=p@K.T;uv=uv[:,:2]/uv[:,2,None]
    region=mask.copy()
    if fault=='hole':region[100:200,100:200]=False
    if fault=='occlusion':mask[100:200,100:200]=False
    if fault=='invalid':depth[100:200,100:200]=np.nan
    if fault=='depth_error':depth[100:200,100:200]+=.01
    surface=dict(schema_version='observed_surface_v1',frame_id='world',volume_status='UNKNOWN',face_id='f',
        module_id='module',capture_id='capture',source_instance_id='instance',capture_time=1.,sensor_epoch='epoch',
        clock_domain='ros_sim_time',calibration_identity='calibration',T_W_C_at_capture=T.tolist(),
        corners_3d_m=world.tolist(),boundary_2d_px=uv.tolist(),plane_normal=n.tolist(),plane_offset_m=-float(world[0]@n),
        plane_residual_m=0.,point_support_count=1000,point_support_ratio=1.,evidence='registered_metric_depth_plane')
    record=dict(support_capture_binding=_binding(depth,mask,K),frozen_support_regions={'1':_support_runs(region)},
        camera_facing_faces=[dict(corners_3d_m=p.tolist(),support_label=1,final_support={'status':'PASS'})])
    tool=current_tool(ROOT);tool.update(grid_fractions=[0.],roll_degrees=[0.])
    return surface,record,depth,mask,K,tool


def run(values):
    return face_candidates(*values,artifact={'path':'SYNTHETIC_ONLY','sha256':'a'*64},object_id='object')


@pytest.mark.parametrize('angles',[(0,0,0),(0,np.pi/2,0),(np.pi,0,0),(.3,.5,.8)])
def test_pose_outside_and_tcp_flange_transforms(angles):
    args=fixture(rotation=rotation_matrix_from_rpy(*angles));item=run(args)[0];c=item.candidate
    assert isinstance(c,SuctionGraspCandidate) and item.evidence['support_status']=='GEOMETRIC_SUPPORT'
    assert len(c.sealed_cup_indices)==72 and c.sealed_cups_per_zone==(24,24,24)
    np.testing.assert_allclose(c.grasp_pose[:3,2],-c.outward_normal,atol=1e-12)
    np.testing.assert_allclose(c.pregrasp_pose[:3,3]-c.contact_point,c.outward_normal*.15,atol=1e-12)
    np.testing.assert_allclose(np.asarray(item.evidence['flange_pose'])@args[-1]['flange_from_task_tcp'],c.grasp_pose,atol=1e-12)
    assert all(v in ('UNKNOWN','NOT_EVALUATED','BLOCKED') for v in item.evidence['remaining'].values())
    from unloading_sim.grasp import select_fast_suction_candidates
    assert select_fast_suction_candidates([c],face_modes=('observed_surface',),candidates_per_face=1)==[c]


def test_partial_cup_support_keeps_original_minimum_and_zone_counts():
    args=fixture(width=.5,height=.285);args[-1]['roll_degrees']=[0.,90.]
    items=run(args)
    best=items[0]
    assert len(best.candidate.sealed_cup_indices)==60
    assert best.evidence['support_status']=='GEOMETRIC_SUPPORT'
    assert sum(best.candidate.sealed_cups_per_zone)==60 and best.evidence['required_cups']==60


def test_cup_center_inside_does_not_cover_crossing_lip():
    args=fixture(width=.53,height=.25)
    items=run(args)
    assert items[0].evidence['support_status']=='INSUFFICIENT_SUPPORTED_CUPS'
    assert any(v['reason']=='CUP_LIP_OUTSIDE_OBSERVED_PATCH' for v in items[0].evidence['cups'])


@pytest.mark.parametrize('fault',['hole','occlusion','invalid','depth_error'])
def test_support_is_not_filled_by_quad_or_hull(fault):
    item=run(fixture(fault=fault))[0]
    assert item.evidence['support_status']=='INSUFFICIENT_SUPPORTED_CUPS'
    assert any(v['reason'] in ('HOLE_OCCLUSION_INVALID_OR_ERODED_SUPPORT','LOCAL_DEPTH_PLANE_ERROR') for v in item.evidence['cups'])


@pytest.mark.parametrize('fault',['normal','frame','nan','degenerate','calibration','transform','radius','minimum','standoff'])
def test_invalid_geometry_or_tool_rejected(fault):
    args=list(fixture());s=args[0]
    if fault=='normal':s['plane_normal']=[0,0,1]
    elif fault=='frame':s['frame_id']='camera'
    elif fault=='nan':s['corners_3d_m'][0][0]=float('nan')
    elif fault=='degenerate':s['corners_3d_m'][1]=s['corners_3d_m'][0]
    elif fault=='calibration':args[4][0,0]=-1
    elif fault=='transform':s['T_W_C_at_capture'][0][0]=2
    elif fault=='radius':args[-1]['layout']['cup_radius_m']=-1
    elif fault=='minimum':args[-1]['layout']['minimum_sealed_cups']=0
    else:args[-1]['standoff_m']=-1
    with pytest.raises(ValueError):run(args)


@pytest.mark.parametrize('field',['depth','mask','K'])
def test_mutated_support_inputs_fail_binding(field):
    args=list(fixture())
    if field=='depth':args[2][0,0]+=.01
    elif field=='mask':args[3][0,0]=False
    else:args[4][0,0]+=1
    with pytest.raises(ValueError,match='BINDING'):run(args)


def test_crop_and_deterministic_order_and_identity():
    args=fixture();args[-1]['grid_fractions']=[-.42,0,.42];args[-1]['roll_degrees']=[0,90]
    before=canonical_fingerprint(args[0]);a=run(args);b=run(args)
    assert [i.to_dict() for i in a]==[i.to_dict() for i in b]
    assert canonical_fingerprint(args[0])==before and len({i.evidence['identity'] for i in a})==18
    # Crop frame near its principal point without changing metric sampling.
    s,r,d,m,K,t=fixture();d=d[:165];m=m[:165]
    r['support_capture_binding']=_binding(d,m,K);r['frozen_support_regions']={'1':_support_runs(m)}
    assert any(v['reason']=='CUP_IMAGE_CROPPED' for v in run((s,r,d,m,K,t))[0].evidence['cups'])


def test_duplicate_faces_use_fusion_representative_and_conflicts_block():
    s,*_=fixture();other={**s,'module_id':'other','face_id':'other'}
    cargo=SimpleNamespace(observed_surfaces=[s,other],association_status='ASSOCIATED_BY_WORLD_FACE_GEOMETRY',
        raw_result={'fusion_diagnostics':{'face_reduction':{'representatives':[s],'conflicts':[]}}})
    assert representative_surfaces(cargo)==([s],None)
    cargo.raw_result['fusion_diagnostics']['face_reduction']['conflicts']=[{'reason':'CONFLICT'}]
    assert representative_surfaces(cargo)[0]==[]
    cargo.observed_surfaces=[];cargo.raw_result={};cargo.association_status='UNASSOCIATED'
    assert representative_surfaces(cargo)==([],'NO_CERTIFIED_OBSERVED_FACE')


def test_artifact_bad_hash_fails_before_output(tmp_path):
    spec=importlib.util.spec_from_file_location('contact_cli',ROOT/'tools/derive_surface_contacts.py')
    cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
    source=tmp_path/'artifact.json';source.write_text('{}')
    plan=tmp_path/'plan.json';plan.write_text(__import__('json').dumps({'groups':[{'name':'A','artifact':{'path':str(source),'sha256':'0'*64}}]}))
    with pytest.raises(ValueError,match='HASH'):cli.main(['--plan',str(plan),'--output',str(tmp_path/'out')])
    assert not (tmp_path/'out').exists()


def test_real_capture_loader_accepts_contract_tuple_transform_and_rejects_changed_depth(tmp_path,monkeypatch):
    from test_workcell_perception_once import prepare
    from unloading_perception.algorithm_artifact import load_algorithm_artifact
    import json
    spec=importlib.util.spec_from_file_location('contact_cli_loader',ROOT/'tools/derive_surface_contacts.py')
    cli=importlib.util.module_from_spec(spec);spec.loader.exec_module(cli)
    capture,modules,_,main,argv=prepare(tmp_path,monkeypatch)
    assert main(argv)==0  # Explicit CPU model doubles, production capture/artifact encoding.
    output=capture/'perception-once'
    ref=json.loads((output/'summary.json').read_text())['algorithm_artifact']
    _,index=load_algorithm_artifact(ref['path'],ref['sha256'])
    depth,K,members,masks=cli.module_inputs(index,modules[0],output)
    assert len(members)==len(masks) and depth.dtype==np.float32
    (output/modules[0]/'metric_depth_m.npy').write_bytes(b'changed')
    with pytest.raises(ValueError,match='HASH'):cli.module_inputs(index,modules[0],output)


def test_local_support_is_separate_from_existing_scene_admission(tmp_path):
    from test_algorithm_artifact import artifact_fixture
    from unloading_perception.algorithm_artifact import load_algorithm_artifact
    from unloading_perception.scene import build_scene_update
    ref=artifact_fixture(tmp_path)
    observation,_=load_algorithm_artifact(ref['path'],ref['sha256'])
    fingerprint=canonical_fingerprint(observation)
    item=run(fixture())[0]  # Synthetic support test, never a real solver/model claim.
    assert item.evidence['support_status']=='GEOMETRIC_SUPPORT'
    assert not build_scene_update(observation).planning_admissible
    assert observation.unknown_regions and not any(c.candidate_eligible for c in observation.cargo)
    assert canonical_fingerprint(observation)==fingerprint

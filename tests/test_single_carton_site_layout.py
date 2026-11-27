"""Synthetic STEP conversion and registration; never field-layout evidence."""
from pathlib import Path
import sys
import json
import shutil
import numpy as np
import pytest
from unloading_sim.workbench.single_carton.site_layout import load_layout,register_layout,registration_limits


@pytest.fixture
def layout(tmp_path,monkeypatch):
    pytest.importorskip('OCP',reason='Use the existing pinned CAD cache; no install or upgrade')
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.TDataStd import TDataStd_Name
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.STEPCAFControl import STEPCAFControl_Writer
    from OCP.STEPControl import STEPControl_AsIs
    from OCP.gp import gp_Pnt
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TopLoc import TopLoc_Location
    root=Path(__file__).resolve().parents[1]
    monkeypatch.syspath_prepend(str(root/'tools'))
    import convert_site_layout,convert_static_cad
    monkeypatch.setattr(convert_site_layout,'ROOT',tmp_path);monkeypatch.setattr(convert_static_cad,'ROOT',tmp_path)
    raw=tmp_path/'assets/layout/raw';raw.mkdir(parents=True)
    doc=TDocStd_Document(TCollection_ExtendedString('BinXCAF'));st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    assembly=st.NewShape();TDataStd_Name.Set_s(assembly,TCollection_ExtendedString('synthetic_site_assembly'))
    for name,point,size in [('table_design',gp_Pnt(0,0,0),(200,300,100)),('robot_name_is_not_an_exclusion',gp_Pnt(400,0,0),(100,100,100))]:
        label=st.AddShape(BRepPrimAPI_MakeBox(point,*size).Shape(),False)
        TDataStd_Name.Set_s(label,TCollection_ExtendedString(name))
        instance=st.AddComponent(assembly,label,TopLoc_Location())
        TDataStd_Name.Set_s(instance,TCollection_ExtendedString(name+'_instance'))
    st.UpdateAssemblies()
    writer=STEPCAFControl_Writer();writer.SetNameMode(True)
    assert writer.Transfer(doc,STEPControl_AsIs)
    assert writer.Write(str(raw/'synthetic.step'))==IFSelect_RetDone
    report=convert_site_layout.convert()
    config=dict(site_layout=dict(manifest='assets/layout/derived/'+report['source_sha256'][:16]+'/site-layout.json',
        T_base_layout=np.eye(4).tolist(),transform_basis='SYNTHETIC_IDENTITY',excluded_part_ids=[],
        exclusion_basis='SYNTHETIC_KEEP_ALL_PARTS',association_distance_m=.05,association_basis='SYNTHETIC_50_MM_SEARCH_BAND'))
    return tmp_path,report,config,convert_site_layout


def test_synthetic_step_units_names_hierarchy_bounds_and_no_name_exclusion(layout):
    root,report,c,_=layout
    assert len(report['parts'])==2 and len(report['assembly_nodes'])==3
    assert report['assembly_nodes'][0]['assembly']
    assert report['assembly_nodes'][0]['name']=='synthetic_site_assembly'
    assert {p['name'] for p in report['parts']}=={'table_design','robot_name_is_not_an_exclusion'}
    assert report['source_units']=='millimetres' and report['normalization_scale_to_m']==.001
    loaded=load_layout(c,root);assert len(loaded['world'])==2
    assert all(o['category']=='site_layout_cad' and o['basis']=='CAD_DESIGN_GEOMETRY_NOT_FIELD_MEASUREMENT' for o in loaded['world'])
    np.testing.assert_allclose(loaded['world'][0]['size'],[.2,.3,.1],atol=1e-8)
    assert report['physical_validation']=='NOT_RUN'


def test_explicit_exclusion_and_transform_are_applied_with_basis(layout):
    root,report,c,_=layout
    c['site_layout']['excluded_part_ids']=[report['parts'][1]['id']]
    c['site_layout']['T_base_layout'][0][3]=.25
    loaded=load_layout(c,root);assert len(loaded['world'])==1
    assert loaded['world'][0]['pose'][0][3]==pytest.approx(.35,abs=1e-8)
    c['site_layout']['transform_basis']=''
    with pytest.raises(ValueError,match='BASIS_REQUIRED'):load_layout(c,root)


def test_registration_accepts_3mm_and_rejects_8mm_rms_or_no_surface(layout):
    root,_,c,_=layout;loaded=load_layout(c,root)
    xy=np.array([[x,y] for x in (.02,.05,.1,.15,.18) for y in (.03,.1,.2,.27)])
    good=np.c_[xy,np.full(len(xy),.103)];r=register_layout(loaded,good)
    assert r['valid'] and r['rms_m']==pytest.approx(.003,abs=1e-8)
    bad=good.copy();bad[:,2]=.108
    assert register_layout(loaded,bad)['status']=='LAYOUT_REGISTRATION_MISMATCH'
    assert register_layout(loaded,[[5,5,5]])['associated_points']==0


def test_registration_p95_gate_is_independent_of_rms(layout):
    root,_,c,_=layout;loaded=load_layout(c,root)
    p=np.tile([.1,.1,.1],(100,1));p[-6:,2]+=.012
    r=register_layout(loaded,p)
    assert r['rms_m']<.005 and r['p95_m']>.010 and not r['valid']


def test_multiple_step_revisions_need_explicit_source(layout):
    # Several revisions may sit in raw/ (user 2026-11-27 added layout 20261127.stp
    # next to the first one): conversion needs an explicit source, and a manifest
    # still loads only its own source while that file stays in raw/.
    root,_,c,converter=layout
    raw=root/'assets/layout/raw';shutil.copyfile(raw/'synthetic.step',raw/'second.stp')
    with pytest.raises(ValueError,match='MULTIPLE_SITE_LAYOUT_STEPS'):converter.convert()
    with pytest.raises(ValueError,match='SOURCE_NOT_IN_PRIVATE_RAW'):converter.convert(source=root/'outside.step')
    assert load_layout(c,root)['world']
    (raw/'synthetic.step').unlink()
    with pytest.raises((ValueError,FileNotFoundError)):load_layout(c,root)


def test_single_step_hash_change_is_rejected(layout):
    root,_,c,_=layout;p=root/'assets/layout/raw/synthetic.step'
    p.write_bytes(p.read_bytes()+b'\n')
    with pytest.raises(ValueError,match='SOURCE_HASH_CHANGED'):load_layout(c,root)


def test_unreviewed_step_units_are_rejected(layout):
    import re
    root,_,_,converter=layout;p=root/'assets/layout/raw/synthetic.step'
    text=p.read_text();text,count=re.subn(r'SI_UNIT\s*\(\s*\.MILLI\.\s*,\s*\.METRE\.\s*\)',
        'SI_UNIT(.CENTI.,.METRE.)',text)
    assert count;p.write_text(text)
    with pytest.raises(ValueError,match='UNREVIEWED_STEP_UNITS'):converter.convert()


def test_registered_layout_preserves_every_rgbd_obstacle(layout):
    from types import SimpleNamespace
    from unloading_sim.workbench.single_carton.site_layout import merge_registered_layout
    root,_,c,_=layout
    frame=SimpleNamespace(depth=np.full((16,16),.103),depth_scale_m=1.,rgb=np.zeros((16,16,3),'uint8'),observation_id='synthetic-layout',
        intrinsics=dict(width=16,height=16,fx=16,fy=16,ppx=-1,ppy=-1,model='SYNTHETIC_PINHOLE',coeffs=[0]*5))
    obs=SimpleNamespace(frame=frame,world_camera=np.eye(4),q_capture=[0]*6)
    obstacle=dict(id='rgbd_retained',category='observed',pose=np.eye(4).tolist(),size=[.01]*3)
    tracked=[];records=[]
    result=merge_registered_layout(dict(world=[obstacle]),obs,SimpleNamespace(masks=[]),c,
        SimpleNamespace(track=tracked.append),None,lambda name,value:records.append((name,value)),root)
    assert result['world'][0]==obstacle and len(result['world'])==3
    assert result['layout_explained_depth_removed']==0 and result['layout_registration']['valid']
    assert tracked and records[0][0]=='layout-registration.json'


def test_missing_step_never_uses_demo(tmp_path,monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'tools'))
    import convert_site_layout
    monkeypatch.setattr(convert_site_layout,'ROOT',tmp_path)
    with pytest.raises(FileNotFoundError,match='WAITING_FOR_REAL_STEP'):convert_site_layout.convert()


@pytest.mark.parametrize('key,value',[('max_rms_m',.0051),('max_p95_m',.0101)])
def test_registration_limits_cannot_be_relaxed(key,value):
    with pytest.raises(ValueError):registration_limits(dict(layout_registration={key:value}))


def test_curved_cad_analytic_bounds_enclose_true_sphere(tmp_path,monkeypatch):
    pytest.importorskip('OCP')
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeSphere
    from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
    from OCP.IFSelect import IFSelect_RetDone
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'tools'))
    import convert_site_layout,convert_static_cad
    monkeypatch.setattr(convert_site_layout,'ROOT',tmp_path);monkeypatch.setattr(convert_static_cad,'ROOT',tmp_path)
    raw=tmp_path/'assets/layout/raw';raw.mkdir(parents=True)
    writer=STEPControl_Writer();writer.Transfer(BRepPrimAPI_MakeSphere(50.).Shape(),STEPControl_AsIs)
    assert writer.Write(str(raw/'sphere.step'))==IFSelect_RetDone
    report=convert_site_layout.convert();bounds=np.asarray(report['parts'][0]['collision_bounds_m'])
    assert np.all(bounds[0]<=-.05) and np.all(bounds[1]>=.05)
    np.testing.assert_allclose(bounds,[[-.05]*3,[.05]*3],atol=1e-8)


def test_step_string_decodes_unicode_and_latin_escapes():
    import importlib,sys
    sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
    try:convert_static_cad=importlib.import_module('convert_static_cad')
    finally:sys.path.pop(0)
    # Literal names from the 2026-11-20 site STEP: 伺服电机, 箱子模型, 右90°.
    assert convert_static_cad.step_string('iSMK60D-040-DQBK-AA-000_\\X2\\4F3A670D7535673A\\X0\\')=='iSMK60D-040-DQBK-AA-000_伺服电机'
    assert convert_static_cad.step_string('\\X2\\7BB15B506A21578B\\X0\\')=='箱子模型'
    assert convert_static_cad.step_string('WLG12-LD_(\\X2\\53F3\\X0\\90\\X\\B0)')=='WLG12-LD_(右90°)'
    assert convert_static_cad.step_string('plain_name')=='plain_name'


def test_open_surface_body_beside_solid_is_kept_as_its_own_part(tmp_path,monkeypatch):
    pytest.importorskip('OCP')
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace,BRepBuilderAPI_Sewing
    from OCP.BRep import BRep_Builder
    from OCP.TopoDS import TopoDS_Compound
    from OCP.gp import gp_Pln,gp_Pnt,gp_Dir
    from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
    from OCP.IFSelect import IFSelect_RetDone
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'tools'))
    import convert_site_layout,convert_static_cad
    monkeypatch.setattr(convert_site_layout,'ROOT',tmp_path);monkeypatch.setattr(convert_static_cad,'ROOT',tmp_path)
    raw=tmp_path/'assets/layout/raw';raw.mkdir(parents=True)
    # Two perpendicular 100 mm square faces sewn into one open shell, 500 mm from a 100 mm solid cube.
    sewing=BRepBuilderAPI_Sewing(1e-3)
    sewing.Add(BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(500,0,0),gp_Dir(0,0,1)),0.,100.,0.,100.).Face())
    sewing.Add(BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(500,0,0),gp_Dir(0,-1,0)),0.,100.,0.,100.).Face())
    sewing.Perform();shell=sewing.SewedShape()
    builder=BRep_Builder();compound=TopoDS_Compound();builder.MakeCompound(compound)
    builder.Add(compound,BRepPrimAPI_MakeBox(gp_Pnt(0,0,0),100.,100.,100.).Shape());builder.Add(compound,shell)
    writer=STEPControl_Writer();writer.Transfer(compound,STEPControl_AsIs)
    assert writer.Write(str(raw/'mixed.step'))==IFSelect_RetDone
    report=convert_site_layout.convert()
    kinds=sorted(p['body_kind'] for p in report['parts'])
    assert kinds==['OPEN_SHELL','SOLID'],kinds
    shell_part=next(p for p in report['parts'] if p['body_kind']=='OPEN_SHELL')
    lo,hi=np.asarray(shell_part['collision_bounds_m'])
    assert lo[0]<=.5 and hi[0]>=.6 and hi[2]-lo[2]>=.1 and shell_part['untriangulated_faces']==0
    assert all(p['topology_valid'] for p in report['parts'])
    assert report['open_shell_naming']=='STEP_ORDER_FACE_COUNT_VERIFIED' and report['enclosed_surface_bodies']==[]


def test_tilted_part_gets_a_tight_analytic_oriented_box(tmp_path,monkeypatch):
    pytest.importorskip('OCP')
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.gp import gp_Trsf,gp_Ax1,gp_Pnt,gp_Dir
    from OCP.TopLoc import TopLoc_Location
    from OCP.STEPControl import STEPControl_Writer,STEPControl_AsIs
    from OCP.IFSelect import IFSelect_RetDone
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'tools'))
    import convert_site_layout,convert_static_cad
    monkeypatch.setattr(convert_site_layout,'ROOT',tmp_path);monkeypatch.setattr(convert_static_cad,'ROOT',tmp_path)
    raw=tmp_path/'assets/layout/raw';raw.mkdir(parents=True)
    # 200 x 30 x 1000 mm slab tilted 15 deg about X, like the inclined belt.
    t=gp_Trsf();t.SetRotation(gp_Ax1(gp_Pnt(0,0,0),gp_Dir(1,0,0)),np.radians(15))
    slab=BRepPrimAPI_MakeBox(gp_Pnt(0,0,0),200.,30.,1000.).Shape().Moved(TopLoc_Location(t))
    writer=STEPControl_Writer();writer.Transfer(slab,STEPControl_AsIs)
    assert writer.Write(str(raw/'slab.step'))==IFSelect_RetDone
    part=convert_site_layout.convert()['parts'][0]
    lo,hi=np.asarray(part['collision_bounds_m'])
    np.testing.assert_allclose(hi-lo,[.2,.2878,.9737],atol=1e-3)
    obb=part['collision_obb'];R=np.asarray(obb['rotation'])
    assert sorted(obb['size_m'])==pytest.approx([.03,.2,1.],abs=1e-6) and np.linalg.det(R)==pytest.approx(1.)
    # Every slab corner lies inside the oriented box.
    corners=np.array([[x,y,z] for x in (0,.2) for y in (0,.03) for z in (0,1.)])
    Rx=np.array([[1,0,0],[0,np.cos(np.radians(15)),-np.sin(np.radians(15))],[0,np.sin(np.radians(15)),np.cos(np.radians(15))]])
    local=(corners@Rx.T-np.asarray(obb['center_m']))@R
    assert np.all(np.abs(local)<=np.asarray(obb['size_m'])/2+1e-9)

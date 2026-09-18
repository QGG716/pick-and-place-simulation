"""Export the frozen compact workstation for static Warehouse appearance review.

No planning, trajectory replay, rigid-body execution, or hardware integration.
Private meshes are embedded only in the explicitly supplied ignored output folder.
"""
import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import trimesh
import yaml
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
from unloading_sim.eco65.model import ROOT, URDF, asset_check, load_json, robot_for, transform
from unloading_sim.eco65.observation_contracts import canonical_fingerprint


def material(stage, name, rgb, opacity=1., metallic=0., roughness=.5):
    mat = UsdShade.Material.Define(stage, '/Workstation/Materials/' + name)
    shader = UsdShade.Shader.Define(stage, str(mat.GetPath()) + '/Shader')
    shader.CreateIdAttr('UsdPreviewSurface')
    for key, value in [('roughness', roughness), ('metallic', metallic), ('opacity', opacity)]:
        shader.CreateInput(key, Sdf.ValueTypeNames.Float).Set(value)
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    shader.CreateInput('ior', Sdf.ValueTypeNames.Float).Set(1.1)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def export(snapshot_path, home_path, config_path, out):
    out = Path(out).resolve()
    out.relative_to(ROOT / 'outputs')
    out.mkdir(parents=True, exist_ok=True)
    path = out / 'workstation_D.usdc'
    if path.exists():
        raise FileExistsError('Use a fresh run directory; historical evidence is immutable')
    snapshot = load_json(snapshot_path)
    check = dict(snapshot); expected = check.pop('fingerprint')
    if canonical_fingerprint(check) != expected:
        raise ValueError('Snapshot fingerprint mismatch')
    asset_check()
    for relative, digest in snapshot['asset_sha256'].items():
        if hashlib.sha256((ROOT / relative).read_bytes()).hexdigest() != digest:
            raise ValueError('Snapshot asset changed: ' + relative)
    scene, tool = snapshot['scene'], snapshot['tool']
    c = yaml.safe_load(Path(config_path).read_text(encoding='utf-8'))
    assert len(scene['boxes']) == 4 and c['scope'] == 'STATIC_APPEARANCE_ONLY'
    home = load_json(home_path)['home_q']
    frames = robot_for(tool, scene).named_link_frames(home)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, 'Z'); UsdGeom.SetStageMetersPerUnit(stage, 1.)
    root = UsdGeom.Xform.Define(stage, '/Workstation').GetPrim(); stage.SetDefaultPrim(root)
    root.SetCustomDataByKey('physics_status', 'NOT_EVALUATED')

    def pose(prim, T):
        UsdGeom.Xformable(prim).AddTransformOp().Set(Gf.Matrix4d(np.asarray(T).T.tolist()))

    def mesh(path, m, color, T, collision=False, mat=None):
        obj = UsdGeom.Mesh.Define(stage, path)
        obj.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(m.vertices, np.float32)))
        obj.CreateFaceVertexCountsAttr([3] * len(m.faces))
        obj.CreateFaceVertexIndicesAttr(np.asarray(m.faces, np.int32).ravel().tolist())
        obj.CreateSubdivisionSchemeAttr('none')
        obj.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        pose(obj.GetPrim(), T)
        if collision:
            UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(obj.GetPrim()).CreateApproximationAttr('convexHull')
            obj.CreatePurposeAttr('guide')
        elif mat:
            UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
        return obj

    mats = {k: material(stage, k, *v) for k, v in {
        'robot': ([.79,.82,.86],1.,.35,.3), 'robot_dark': ([.12,.15,.18],1.,.3,.35),
        'adapter': ([.85,.55,.13],1.,.55,.3), 'table': ([.52,.42,.29],1.,0.,.5),
        'frame': ([.20,.24,.28],1.,.6,.3), 'glass': ([.75,.86,.91],.10,0.,.1),
    }.items()}
    for link in ET.parse(URDF).getroot().findall('link'):
        name = link.attrib['name']; geo = link.find('visual')
        file = URDF.parents[1] / geo.find('geometry/mesh').attrib['filename'].split('package://ECO65-B/')[1]
        m = trimesh.load(file, process=False)
        # The pinned ECO65 visual origins are identity; reject unhandled changes.
        origin = geo.find('origin')
        if origin is not None:
            assert np.allclose(np.fromstring(origin.get('xyz','0 0 0'),sep=' '),0)
            assert np.allclose(np.fromstring(origin.get('rpy','0 0 0'),sep=' '),0)
        mesh('/Workstation/Robot/'+name+'/Visual',m,[.8,.83,.86],frames[name],mat=mats['robot_dark' if name=='base_link' else 'robot'])
        mesh('/Workstation/Robot/'+name+'/Collision',m.convex_hull,[.2,.7,.8],frames[name],True)
    flange = frames['link_6']; Ttool = flange @ np.array(tool['T_flange_tool_cad'])
    for p in tool['visual_parts']:
        rgb = p['rgb'] or [.6,.6,.6]
        mat = material(stage,'tool_'+p['id'],rgb,metallic=.2,roughness=.4)
        mesh('/Workstation/Tool/visual_'+p['id'],trimesh.load(ROOT/p['visual'],process=False),rgb,Ttool,mat=mat)
    for p in tool['collisions']:
        mesh('/Workstation/Tool/collision_'+p['id'],trimesh.load(ROOT/p['mesh'],process=False),[.1,.8,.8],Ttool,True)
    for a in tool['adapter']:
        m = trimesh.creation.box(a['size']) if a['kind']=='box' else trimesh.creation.cylinder(radius=a['radius'],height=2*a['half_length'],sections=48)
        m.apply_translation(a['position'])
        mesh('/Workstation/Adapter/'+a['id'],m,[.85,.57,.15],flange,mat=mats['adapter'])
        mesh('/Workstation/Adapter/'+a['id']+'_collision',m,[.85,.57,.15],flange,True)

    def cube(name, size, position, mat, guide=False):
        obj = UsdGeom.Cube.Define(stage,'/Workstation/'+name); obj.CreateSizeAttr(1.)
        xf = UsdGeom.Xformable(obj); xf.AddTranslateOp().Set(Gf.Vec3d(*position)); xf.AddScaleOp().Set(Gf.Vec3d(*size))
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
        if guide: obj.CreatePurposeAttr('guide')
        else: UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
        return obj

    for o in scene['obstacles']:
        if o['id']=='trailer_floor' or o['id'] in {b['id'] for b in scene['boxes']}: continue
        mat = mats['glass'] if o['role'] in ('wall','roof') else material(stage,o['id'],o['rgba'][:3],metallic=.3,roughness=.4)
        cube(o['id'],o['size_m'],o['position_m'],mat)
    x0,x1,y0,y1 = c['table']['bounds_xy_D_m']; th = c['table']['thickness_m']; w = c['table']['leg_width_m']; inset = c['table']['leg_inset_m']
    cube('trailer_floor',[x1-x0,y1-y0,th],[(x0+x1)/2,(y0+y1)/2,-th/2],mats['table'])
    xs,ys = [x0+inset,x1-inset],[y0+inset,y1-inset]
    for i,x in enumerate(xs):
        for j,y in enumerate(ys): cube(f'Table/Leg_{i}{j}',[w,w,.8-th],[x,y,(-.8-th)/2],mats['frame'])
    for z in [-.09,c['table']['lower_support_z_D_m']]:
        for i,y in enumerate(ys): cube(f'Table/RailX_{i}_{abs(int(z*100))}',[xs[1]-xs[0],w,w],[(x0+x1)/2,y,z],mats['frame'])
        for i,x in enumerate(xs): cube(f'Table/RailY_{i}_{abs(int(z*100))}',[w,ys[1]-ys[0],w],[x,(y0+y1)/2,z],mats['frame'])
    for b in scene['boxes']:
        prim = UsdGeom.Xform.Define(stage,'/Workstation/Cartons/'+b['id']).GetPrim()
        pose(prim,transform(b['pose'])); prim.SetCustomDataByKey('stable_id',b['id'])
        # This exact layout envelope is the sole carton collision proxy.
        cube('Cartons/'+b['id']+'/Collision',b['size_m'],[0,0,0],mats['table'],True)
    stage.GetRootLayer().Save()
    manifest = dict(status='STATIC_GEOMETRY_EXPORTED',source_snapshot_fingerprint=expected,
        source_snapshot_sha256=hashlib.sha256(Path(snapshot_path).read_bytes()).hexdigest(),home_q_rad=home,
        table_config=c,cartons=scene['boxes'],base_position_D_m=scene['base_position_m'],
        source_config_sha256=snapshot['config_sha256'], physics='NOT_EVALUATED',planning='NOT_EXECUTED',
        workstation_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
    (out/'workstation_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
    print(path)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--snapshot',type=Path,required=True);p.add_argument('--home',type=Path,required=True)
    p.add_argument('--config',type=Path,default=ROOT/'configs/workcells/eco65_desktop_warehouse_v1.yaml')
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();export(a.snapshot,a.home,a.config,a.output)

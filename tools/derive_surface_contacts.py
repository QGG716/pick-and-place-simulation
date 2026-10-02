"""Explicit offline adapter for pinned artifacts. Never loads a model or planner."""
import argparse
from collections import Counter
from io import BytesIO
import json
from pathlib import Path
import sys
import time
from urllib.parse import urlparse, unquote

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src'),str(ROOT/'packages/unloading_contracts/src')]
import numpy as np
from unloading_contracts import canonical_fingerprint, loads
from unloading_perception.algorithm_artifact import load_algorithm_artifact, file_reference, _read_reference
from unloading_perception.finite_sequence import atomic_json
from unloading_perception.isaac_payload import camera_content_identity
from unloading_perception.surface_contacts import current_tool, face_candidates, representative_surfaces, BLOCKERS, ContactSupportInsufficient


def module_inputs(index, module, root):
    observation=loads(_read_reference(index['module_observations'][module],root).decode())
    binding=observation.coverage['module_binding']
    provenance=observation.coverage['input_provenance']
    refs=provenance['consumed_files']
    raw={k:_read_reference(refs[k],root) for k in
         ('metric_depth_m.npy','camera_info.json','capture_metadata.json','capture_binding.json')}
    info=json.loads(raw['camera_info.json']);metadata=json.loads(raw['capture_metadata.json'])
    if (camera_content_identity(info)!=binding['calibration_identity'] or not np.array_equal(info['T_W_C'],binding['T_W_C_at_capture'])
            or info['depth_semantics']!='optical_z_m' or info['registration_mode']!='SIMULATION_IDEAL_REGISTERED_DEPTH'
            or any(info['D']) or metadata['capture_id']!=binding['capture_id']):
        raise ValueError('CONTACT_CAPTURE_CALIBRATION_MISMATCH')
    depth=np.load(BytesIO(raw['metric_depth_m.npy']),allow_pickle=False)
    if depth.shape!=(info['height'],info['width']) or depth.dtype!=np.float32:
        raise ValueError('CONTACT_DEPTH_SHAPE_OR_UNIT_MISMATCH')
    members={c.source_instance_id:c for c in observation.cargo}
    if len(members)!=len(observation.cargo): raise ValueError('DUPLICATE_MODULE_INSTANCE')
    archives={};masks={}
    for c in observation.cargo:
        resource=c.mask_reference
        parsed=urlparse(resource.uri)
        if parsed.scheme!='file': raise ValueError('BOUND_LOCAL_MASK_REQUIRED')
        path=Path(unquote(parsed.path))
        if sys.platform=='win32' and str(path).startswith('\\') and ':' in str(path): path=Path(str(path)[1:])
        key=str(path)
        if key not in archives:
            value=_read_reference(dict(path=key,sha256=resource.sha256),root)
            with np.load(BytesIO(value),allow_pickle=False) as a:
                ids=[int(i) for i in a['mask_ids']]
                if len(ids)!=len(set(ids)) or len(ids)!=len(a['masks']): raise ValueError('MASK_ROSTER_MISMATCH')
                archives[key]={i:np.asarray(mask,bool) for i,mask in zip(ids,a['masks'])}
        identity=int(c.raw_result['instance_lineage']['mask_id'])
        masks[c.source_instance_id]=archives[key][identity]
    return depth,info['K'],members,masks


def visualize(path, surface, item, tool, title):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    from unloading_sim.grasp import suction_cup_layout_geometry
    candidate=item.candidate;points=np.asarray(surface['corners_3d_m']);R=candidate.grasp_pose[:3,:3]
    centers,zones,_=suction_cup_layout_geometry(tool['layout'])
    fig=plt.figure(figsize=(10,8));ax=fig.add_subplot(projection='3d')
    ax.add_collection3d(Poly3DCollection([points],alpha=.15,facecolor='steelblue',edgecolor='black'))
    theta=np.linspace(0,2*np.pi,65);rad=tool['layout']['cup_radius_m']
    allpoints=[points,candidate.pregrasp_pose[:3,3][None,:]]
    for i,offset in enumerate(centers):
        c=candidate.contact_point+R[:,:2]@offset
        ring=c+(rad*np.cos(theta))[:,None]*R[:,0]+(rad*np.sin(theta))[:,None]*R[:,1]
        ax.plot(*ring.T,color='green' if i in candidate.sealed_cup_indices else 'crimson',linewidth=.6)
        allpoints.append(ring)
    c=candidate.contact_point;ax.scatter(*c,color='black',s=25)
    for i,color in enumerate(('red','green','blue')):ax.quiver(*c,*(R[:,i]*.08),color=color)
    pre=candidate.pregrasp_pose[:3,3];ax.quiver(*pre,*(c-pre),color='purple',linewidth=2)
    ax.scatter(*pre,color='purple',marker='x')
    allpoints=np.concatenate(allpoints);mid=(allpoints.max(0)+allpoints.min(0))/2
    radius=max(np.ptp(allpoints,axis=0))*.6
    ax.set(xlim=(mid[0]-radius,mid[0]+radius),ylim=(mid[1]-radius,mid[1]+radius),zlim=(mid[2]-radius,mid[2]+radius),
           xlabel='world X (m)',ylabel='world Y (m)',zlabel='world Z (m)',title=title)
    ax.set_box_aspect((1,1,1));ax.view_init(25,145)
    fig.text(.05,.03,'Measured patch (not a box face boundary); green=geometric support, red=unsupported\n'
             'RGB=tool XYZ; purple=pre-approach -> contact. Collision / seal / IK NOT EVALUATED.',fontsize=9)
    fig.savefig(path,dpi=140);plt.close(fig)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--plan',type=Path,required=True,help='groups: name, artifact {path, sha256}')
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--project',type=Path,default=ROOT)
    p.add_argument('--visualize',action='store_true')
    args=p.parse_args(argv)
    tool=current_tool(args.project)
    plan=json.loads(args.plan.read_text(encoding='utf-8-sig'))
    groups=plan['groups']
    if not groups or len({g['name'] for g in groups})!=len(groups): raise ValueError('INVALID_GROUP_PLAN')
    loaded=[]
    for group in groups:
        if Path(group['name']).name!=group['name'] or group['name'] in ('.','..'): raise ValueError('INVALID_GROUP_NAME')
        observation,index=load_algorithm_artifact(group['artifact']['path'],group['artifact']['sha256'])
        if args.output.resolve().is_relative_to(Path(group['artifact']['path']).resolve().parent):
            raise ValueError('OUTPUT_MUST_BE_OUTSIDE_SOURCE_ARTIFACT')
        loaded.append((group,observation,index))
    args.output.mkdir(parents=True,exist_ok=False)
    frozen=dict(plan=plan,tool=tool,selection='ALL fused objects; existing representative faces only; independent groups',
        objects={g['name']:[c.source_instance_id for c in o.cargo] for g,o,_ in loaded},
        visualization_selection='first object with support, first object with no supported candidate; no re-selection/tuning')
    atomic_json(args.output/'frozen_plan.json',frozen)
    results=[];started=time.perf_counter();cpu=time.process_time()
    try:
        for group,observation,index in loaded:
            before=canonical_fingerprint(observation);root=Path(group['artifact']['path']).resolve().parent
            folder=args.output/group['name'];folder.mkdir();cache={};rows=[];visuals=set()
            for ordinal,cargo in enumerate(observation.cargo):
                surfaces,reason=representative_surfaces(cargo);items=[];errors=[]
                for surface in surfaces:
                    module=surface['module_id']
                    if module not in cache: cache[module]=module_inputs(index,module,root)
                    depth,K,members,masks=cache[module];member=members[surface['source_instance_id']]
                    if canonical_fingerprint(surface) not in {canonical_fingerprint(s) for s in member.observed_surfaces}:
                        raise ValueError('CONTACT_SOURCE_SURFACE_MISMATCH')
                    if 'record' not in member.raw_result:
                        errors.append(dict(face_id=surface['face_id'],reason='FROZEN_SUPPORT_RECORD_UNAVAILABLE'));continue
                    try:
                        items.extend(face_candidates(surface,member.raw_result['record'],depth,masks[member.source_instance_id],K,
                                                     tool,artifact=group['artifact'],object_id=cargo.source_instance_id))
                    except ContactSupportInsufficient as exc:
                        errors.append(dict(face_id=surface['face_id'],reason=str(exc),contact_search='NOT_ELIGIBLE'))
                items.sort(key=lambda item:-item.candidate.score)
                passing=[i for i in items if i.evidence['support_status']=='GEOMETRIC_SUPPORT']
                counts=Counter(cup['reason'] for i in items for cup in i.evidence['cups'] if cup['reason'])
                best=items[0] if items else None
                row=dict(object_id=cargo.source_instance_id,association_status=cargo.association_status,
                    raw_face_count=len(cargo.observed_surfaces),usable_faces=len(surfaces),candidate_count=len(items),
                    supported_candidates=len(passing),reason=reason or (None if passing else 'INSUFFICIENT_SUPPORTED_CUPS'),
                    cup_rejection_counts=dict(counts),face_errors=errors,remaining=BLOCKERS.copy(),
                    best=None if best is None else {k:v for k,v in best.to_dict().items() if k not in ('cups','support_audit')})
                target=folder/f'object-{ordinal:03d}.json'
                atomic_json(target,dict(summary=row,candidates=[v.to_dict() for v in items]))
                row['candidate_artifact']=file_reference(target);rows.append(row)
                kind='supported' if passing else 'rejected'
                if args.visualize and best and kind not in visuals:
                    s=next(s for s in surfaces if s['face_id']==best.evidence['source']['source']['face_id'])
                    visualize(folder/(kind+'.png'),s,best,tool,
                        f"{group['name']} / object {ordinal}: {kind}\n{len(best.candidate.sealed_cup_indices)} / {best.evidence['required_cups']} cups")
                    visuals.add(kind)
            assert canonical_fingerprint(observation)==before
            # Re-read pinned references after processing; original artifacts must remain unchanged.
            load_algorithm_artifact(group['artifact']['path'],group['artifact']['sha256'])
            results.append(dict(group=group,source_sequence=observation.source_sequence,capture_time=observation.capture_time,
                objects=rows,original_observation_fingerprint=before,original_candidate_eligible_unchanged=True,
                original_unknown_count=len(observation.unknown_regions),planning_admissible=False,
                raw_image_automatic=False,policy='HISTORICAL_REPLAY_DISPLAY_ONLY / ORACLE-PROMPTED / NO EXECUTION'))
        atomic_json(args.output/'summary.json',dict(status='COMPLETED_OFFLINE_ADAPTATION',groups=results,
            adapter_wall_seconds=time.perf_counter()-started,adapter_cpu_seconds=time.process_time()-cpu,
            timing_scope='artifact/support loading, local contacts, JSON and optional plots; excludes prior model/geometry runs'))
    except Exception as exc:
        atomic_json(args.output/'failure.json',dict(status='FAILED',error_type=type(exc).__name__,error=str(exc)))
        raise
    return 0


if __name__=='__main__':raise SystemExit(main())

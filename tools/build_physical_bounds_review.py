"""Post-solve review of six finite bounds on frozen notch predictions.

This evaluator receives actual rendered mesh truth. The production solver does
not import this module. It never writes an evaluation association into predictions.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

from build_notch_geometry_review import ROOT, build, read, write, box_errors, acceptance_config, review_capture
from unloading_perception.final_geometry import SIGNS


def legacy_axes(record):
    """Recover the original row-axis convention from its fitted observed planes."""
    faces = record['camera_facing_faces']
    axes = np.zeros((3, 3))
    for axis in (0, 1):
        found = next((f for f in faces if f['axis_index'] == axis), None)
        if found is None:
            return None
        axes[axis] = found['plane_normal']
    axes[2] = np.cross(axes[0], axes[1])
    return axes


def boundary_comparison(record, before, T, vertices):
    finite = record.get('boundary_constraints', {})
    axes = np.array(finite['axes_camera']) if finite else legacy_axes(before)
    if axes is None or 'completion_diagnostics' not in before:
        return None
    camera_vertices = (vertices-T[:3, 3]) @ T[:3, :3]
    reference = camera_vertices @ axes.T
    reference_bounds = np.array([reference.min(0), reference.max(0)])
    previous = before['completion_diagnostics']
    old_axes = legacy_axes(before)
    old_corners = (np.array(previous['bounds_low'])+(SIGNS+1)/2*(np.array(previous['bounds_high'])-previous['bounds_low'])) @ old_axes
    old_q = old_corners @ axes.T
    old_bounds = np.array([old_q.min(0), old_q.max(0)])
    old_initial = np.array([previous['bounds_low'], previous['bounds_high']])
    parameters = previous['silhouette_bound_parameters']
    for (axis, side), value in zip(parameters, before['metric_solver']['boundary_fit']['initial_free_bounds_m']):
        old_initial[int(side == 'high'), axis] = value
    rows = []
    world_axes = axes @ T[:3, :3].T
    for axis in range(3):
        world_axis = int(np.argmax(abs(world_axes[axis])))
        polarity = int(np.sign(world_axes[axis, world_axis]))
        for j, side in enumerate(('low', 'high')):
            bound = next((b for b in finite.get('bounds', []) if b['axis'] == axis and b['side'] == side), None)
            final = None if not bound else bound['value_m']
            rows.append(dict(axis=axis, side=side, world_direction=('-' if polarity < 0 else '+')+'XYZ'[world_axis],
                raw_valid_observed_range_m=None if not finite else [r[axis] for r in finite['raw_valid_observation_range_m']],
                stable_fit_sample_range_m=None if not finite else [r[axis] for r in finite['stable_fit_sample_range_m']],
                frozen_label_range_m=None if not finite else [r[axis] for r in finite['frozen_lifted_label_range_m']],
                quantile_initial_m=float(old_initial[j, axis]), before_optimized_m=float(old_bounds[j, axis]),
                final_m=final, reference_m=float(reference_bounds[j, axis]),
                diagnostic_outline_projection_m=None if not bound else bound.get('outline_projection_m'),
                diagnostic_outline_signed_error_m=(None if not bound or bound.get('outline_projection_m') is None
                    else float(bound['outline_projection_m']-reference_bounds[j, axis])),
                before_signed_error_m=float(old_bounds[j, axis]-reference_bounds[j, axis]),
                final_signed_error_m=None if final is None else float(final-reference_bounds[j, axis]),
                constraint_status=None if not bound else bound['status'],
                constraint_sources=[] if not bound else bound['sources'],
                uncertainty_interval_m=None if not bound else bound.get('interval_m'),
                containment=None if not bound else bound['containment_inequality']))
    dimensions = []
    for axis in range(3):
        lo, hi = rows[2*axis:2*axis+2]
        value = None if lo['final_m'] is None or hi['final_m'] is None else hi['final_m']-lo['final_m']
        dimensions.append(dict(axis=axis, world_direction=lo['world_direction'],
            before_m=float(old_bounds[1, axis]-old_bounds[0, axis]), final_m=value,
            reference_m=float(np.ptp(reference[:, axis])),
            final_signed_error_m=None if value is None else float(value-np.ptp(reference[:, axis]))))
    world_low, world_high = vertices.min(0), vertices.max(0)
    reference_corners = (world_low+(SIGNS+1)/2*(world_high-world_low)-T[:3, 3]) @ T[:3, :3]
    return dict(axis_convention='ROW axes, camera_xyz @ axes.T; signed error = prediction - mesh reference',
        axes_camera=axes.tolist(), axes_world=world_axes.tolist(), bounds=rows, dimensions=dimensions,
        before_candidate_corners=old_corners.tolist(), reference_corners_camera=reference_corners.tolist(),
        before_box=box_errors(old_corners, T, vertices), before_box_was_accepted=before['accepted'],
        current_box=box_errors(record['corners_3d'], T, vertices) if record['accepted'] else None,
        rejected_diagnostic_box=(box_errors(record['rejected_cuboid_candidate']['corners_3d'], T, vertices)
            if record.get('rejected_cuboid_candidate', {}).get('corners_3d') else None),
        reference='actual composed visible mesh world AABB for the verified unrotated notch objects; ideal cuboid model error retained')


def render_bound_images(folder, instance):
    """Native-pixel scientific overlays from the exact source RGB, no inference."""
    from PIL import Image, ImageDraw
    rgb=Image.open(folder/'sensor_rgb.png').convert('RGB')
    proposal=instance['proposals'][0]
    bbox=proposal.get('actual_prompt',{}).get('base_bbox') or proposal['bbox']
    crop=(max(0,int(bbox[0])-35),max(0,int(bbox[1])-35),
          min(rgb.width,int(bbox[2])+36),min(rgb.height,int(bbox[3])+36))
    directory=folder/f"mask-{instance['mask_id']}-edges";directory.mkdir()
    rgb.crop(crop).save(directory/'raw.png')
    before=rgb.copy();draw=ImageDraw.Draw(before)
    for face in instance['before_record'].get('completion_diagnostics',{}).get('candidate_faces',[]):
        for i,boundary in enumerate(face['boundary_evidence']):
            a,b=face['corners_2d'][i],face['corners_2d'][(i+1)%4]
            color='orange' if boundary['kind']=='PHYSICAL_EDGE_SUPPORTED' else '#ff3344'
            draw.line([tuple(a),tuple(b)],fill=color,width=2)
            draw.text(tuple((np.array(a)+b)/2),f"{face['support_label']}:{i}",fill='white')
    before.crop(crop).save(directory/'before.png')
    edge_index=[]
    evidence=instance['record'].get('boundary_constraints',{})
    for i,edge in enumerate(evidence.get('edges',[])):
        canvas=rgb.copy();d=ImageDraw.Draw(canvas)
        d.line([tuple(p) for p in edge['candidate_segment_pixels']],fill='#ff3344',width=2)
        for s in edge['samples']:
            if s.get('search_end_pixel'):d.line([tuple(s['candidate_pixel']),tuple(s['search_end_pixel'])],fill='#bb82ff',width=1)
            for key,color in [('inner_pixel','white'),('outer_pixel','orange'),('pixel','#00ff66')]:
                if key in s:
                    x,y=s[key];d.ellipse((x-2,y-2,x+2,y+2),fill=color)
        name=f'outline-{i}.png';canvas.crop(crop).save(directory/name)
        edge_index.append(dict(file=name,support_label=edge['support_label'],edge_index=edge['edge_index'],
                               status=edge['status'],reasons=[s['kind'] for s in edge['samples']]))
    for i,edge in enumerate(evidence.get('shared_edges',[])):
        canvas=rgb.copy();d=ImageDraw.Draw(canvas)
        d.line([tuple(p) for p in edge['corners_2d']],fill='#00ff66' if edge['supported'] else '#ff3344',width=2)
        for sample in edge['samples']:
            x,y=sample['pixel'];radius=edge['search_px'];d.ellipse((x-radius,y-radius,x+radius,y+radius),outline='#bb82ff')
            for j,surface in enumerate(sample['surfaces']):
                for x,y in surface.get('point_pixels',[]):d.ellipse((x-1,y-1,x+1,y+1),fill='white' if j==0 else 'orange')
        name=f'joint-{i}.png';canvas.crop(crop).save(directory/name)
        edge_index.append(dict(file=name,support_labels=edge['support_labels'],reason=edge['reason'],
                               crease_depth_check=edge['crease_depth_check']))
    write(directory/'index.json',dict(crop_xyxy=crop,source='sensor_rgb.png',native_pixels=True,edges=edge_index))
    return str(directory.relative_to(folder))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('frozen', 'before', 'after', 'visibility', 'output'):
        parser.add_argument('--'+name, type=Path, required=True)
    args = parser.parse_args()
    pages = build(args.frozen, args.after, args.visibility, args.output, before_geometry=args.before)
    visibility = read(args.visibility)
    capture = review_capture(read(args.frozen/'frozen_plan.json'),visibility)
    mesh = read(capture/'rendered_surface_truth.json')
    with np.load(capture/mesh['mesh_file']) as archive:
        vertices = {o['simulation_object_id']:archive[o['array_prefix']+'_vertices_world_m'] for o in mesh['objects']}
        triangles = {o['simulation_object_id']:archive[o['array_prefix']+'_triangles'] for o in mesh['objects']}
    from check_notch_visibility import mesh_surfaces
    mesh_audit={}
    for name,points in vertices.items():
        faces=mesh_surfaces(points,triangles[name],acceptance_config()['gt_normal_degrees'])
        mesh_audit[name]=dict(full_mesh_bounds_world_m=[points.min(0).tolist(),points.max(0).tolist()],
            flat_surface_bounds=faces,usage='POST_SOLVE_MODEL_DISCREPANCY_ONLY; never used to correct bounds')
    write(args.output/'mesh_model_audit.json',mesh_audit)
    rows, bound_rows = [], []
    for page in pages:
        folder = args.output/page['name']/page['module']
        data = read(folder/'geometry_review.json')
        camera = read(args.frozen/page['name']/page['module']/'camera_info.json')
        T = np.array(camera['T_W_C'])
        for instance in data['instances']:
            record, before = instance['record'], instance['before_record']
            identity = dict(group=page['name'], module=page['module'], mask_id=instance['mask_id'],
                            proposal_ids=instance['proposal_ids'])
            if page['name'] == 'stack_notch' and instance['matched_gt'] and before.get('cuboid_solver_attempted'):
                comparison = boundary_comparison(record, before, T, vertices[instance['matched_gt']])
                instance['boundary_comparison'] = comparison
                if comparison:
                    bound_rows.extend(dict(**identity, **b) for b in comparison['bounds'])
            if page['name']=='stack_notch' and page['module']=='module_1_lower' and instance['mask_id'] in (2,6):
                instance['edge_images']=render_bound_images(folder,instance)
            evaluation = instance['evaluation']
            planes = evaluation['planes'] if isinstance(evaluation['planes'], dict) else {}
            errors = evaluation['complete_box'] if isinstance(evaluation['complete_box'], dict) else {}
            row = dict(**identity, capture_id=data['capture_id'], reliable_planes=record['reliable_plane_count'],
                source_ambiguous=record['source_ambiguous'], before_accepted=before['accepted'], accepted=record['accepted'],
                accurate_recovery=bool(record['accepted'] and errors.get('external_accuracy_pass')),
                erroneous_acceptance=bool(record['accepted'] and errors.get('external_accuracy_pass') is False),
                external_accuracy='NOT_RUN_NO_MESH' if page['name'] == 'frame32' else errors.get('external_accuracy_pass'),
                missing_bound_constraints=record.get('boundary_constraints', {}).get('missing_bounds', []),
                extraction_omission=bool(planes.get('unmatched_gt_surfaces')),
                expected_visible_faces=planes.get('expected_reliable_visible_faces'),
                status=record['cuboid_status'])
            rows.append(row)
        data['review_scope'] = 'Fresh 50a9701 baseline versus fresh production finite-bound solve; DINO/SAM reused byte-for-byte'
        write(folder/'geometry_review.json', data)
        (folder/'geometry_review_data.js').write_text('window.GEOMETRY='+json.dumps(data, allow_nan=False).replace('</', '<\\/')+';', encoding='utf-8')
    write(args.output/'regression.json', rows)
    write(args.output/'six_boundaries.json', bound_rows)
    for name, values in (('regression', rows), ('six_boundaries', bound_rows)):
        with (args.output/(name+'.csv')).open('w', newline='', encoding='utf-8-sig') as f:
            writer = csv.DictWriter(f, fieldnames=list(values[0]))
            writer.writeheader()
            writer.writerows(values)
    summary = dict(all_predictions=len(rows), notch_predictions=sum(r['group']=='stack_notch' for r in rows),
        multipane_predictions=sum(r['reliable_planes']>1 for r in rows),
        accurate_recovery=sum(r['accurate_recovery'] for r in rows), erroneous_acceptance=sum(r['erroneous_acceptance'] for r in rows),
        constraint_insufficient=sum(r['reliable_planes']>1 and not r['accepted'] and not r['source_ambiguous'] for r in rows),
        extraction_omission=sum(r['extraction_omission'] for r in rows),
        source_ambiguity=sum(r['source_ambiguous'] for r in rows),
        flags_overlap=True, mesh_evaluation_frame32='NOT_RUN_NO_COMPOSED_MESH', thresholds=acceptance_config(),
        threshold_sha256=hashlib.sha256((ROOT/'configs/perception/metric_geometry_acceptance.json').read_bytes()).hexdigest(),
        detector_and_sam='NOT_RUN_REUSED_FROZEN_OUTPUTS', new_capture='NOT_RUN_REUSED_CAPTURE',
        later_stages={s:'NOT_RUN' for s in ('contacts','fusion','ros','planning','execution')})
    write(args.output/'summary.json', summary)
    links=''.join(f'<li><a href="{p["href"]}">{p["name"]} / {p["module"]} · 全部 {p["masks"]} 个实例</a></li>' for p in pages)
    (args.output/'index.html').write_text('''<!doctype html><html lang="zh"><meta charset="utf-8"><meta name="viewport" content="width=device-width"><title>有限边界与长方体评审</title><style>body{font:17px system-ui;margin:32px;background:#101822;color:#eef;max-width:1100px}a{color:#9df}.cards{display:flex;gap:20px}.card{display:block;padding:24px;background:#213247;flex:1;border-radius:10px}p{line-height:1.6}li{margin:12px 0}</style><h1>双面 / 三面：尺寸从哪里来</h1><p>原采集与 DINO/SAM 冻结。本轮重新运行生产几何，保留修改前后结果。GT 默认关闭；合成测试不代表真实模型精度。</p><div class="cards"><a class="card" href="stack_notch/module_1_lower/index.html?mask=2"><h2>双面箱子</h2><p>mask 2 / proposal 2</p><p>检查原错误接受、底端缺失约束与六边界有符号误差。</p></a><a class="card" href="stack_notch/module_1_lower/index.html?mask=6"><h2>三面箱子</h2><p>mask 6 / proposal 7</p><p>检查实际共同边、轮廓约束及独立网格精度。</p></a></div><h2>全部实例回归</h2><ul>'''+links+'''</ul><p><a href="regression.csv">回归表</a> · <a href="six_boundaries.csv">六边界表</a> · <a href="summary.json">结果与冻结验收阈值</a></p><p>采集 / DINO / SAM：本轮未重跑，使用已验证原始输入。吸附、接触、融合、ROS、规划、执行：NOT_RUN。</p></html>''', encoding='utf-8')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

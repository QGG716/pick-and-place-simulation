"""Independent fixed-threshold gates; GT is confined to this evaluation tool."""
import argparse
import hashlib
import json
from pathlib import Path
from diagnose_metric_calibration import evaluate


def main():
    p=argparse.ArgumentParser(); p.add_argument('--calibration',type=Path,required=True); p.add_argument('--small-matrix',type=Path,required=True)
    p.add_argument('--small-capture',type=Path,required=True); p.add_argument('--thresholds',type=Path,required=True); p.add_argument('--output',type=Path,required=True)
    args=p.parse_args(); limits=json.loads(args.thresholds.read_text()); failures=[]; checks=[]
    def good(face):
        return (face['plane_distance_m']<=limits['gt_plane_position_m'] and face['normal_error_degrees']<=limits['gt_normal_degrees']
                and face['boundary_outside_m']<=limits['gt_patch_boundary_outside_m'])
    calibration=json.loads((args.calibration/'summary.json').read_text())
    for module,row in calibration.items():
        for mode in ('ORACLE_MASK_DIAGNOSTIC','SAM'):
            result=row[mode]
            passed=(result['complete_accepted'] and len(result['faces'])>=2 and all(good(f) for f in result['faces'])
                    and result['center_error_m']<=limits['gt_complete_center_m']
                    and max(result['dimension_error_m'])<=limits['gt_complete_dimension_m']
                    and result['orientation_error_degrees']<=limits['gt_complete_orientation_degrees']
                    and result['corner_set_max_error_m']<=limits['gt_certified_corner_m'])
            checks.append({'case':f'CALIBRATION/{module}/{mode}','pass':passed})
            if not passed: failures.append(checks[-1])
    rows=json.loads((args.small_matrix/'summary.json').read_text()); ambiguous=[]
    if len(rows)!=4: failures.append({'reason':'INCOMPLETE_TWO_SCENE_TWO_MODULE_MATRIX'})
    for row in rows:
        for result in row['ORACLE_MASK_DIAGNOSTIC']:
            passed=bool(result['faces']) and all(good(f) for f in result['faces'])
            checks.append({'case':f"{row['scene']}/{row['module']}/ORACLE/{result['entity']}",'pass':passed})
            if not passed: failures.append(checks[-1])
        folder=args.small_capture/row['scene']/'modules'/row['module']
        camera=json.loads((folder/'camera_info.json').read_text())
        truth={g['simulation_object_id']:g for g in json.loads((folder/'gt_annotations.json').read_text())['objects']}
        records={r['mask_id']:r for r in json.loads((args.small_matrix/row['scene']/row['module']/'metric_final.json').read_text())['instances']}
        for pair in row['SAM_PAIRED']:
            if len(pair['entities'])==1:
                passed=bool(pair['new']['faces']) and all(good(f) for f in pair['new']['faces'])
            else:
                record=records[pair['mask_id']]; candidates=[]
                for face in record['camera_facing_faces']:
                    alternatives=[{'evaluation_entity':identity,**evaluate({'accepted':False,'camera_facing_faces':[face]},truth[identity],camera)['faces'][0]} for identity in pair['entities']]
                    candidates.append({'support_label':face['support_label'],'alternatives':alternatives,'has_geometric_support_in_source_entity_set':any(good(v) for v in alternatives)})
                passed=(bool(candidates) and all(v['has_geometric_support_in_source_entity_set'] for v in candidates)
                        and record['source_ambiguous'] and not record['accepted'])
                ambiguous.append({'scene':row['scene'],'module':row['module'],'source_entities':pair['entities'],
                                  'production_identity':'UNRESOLVED_PARENT_SAM_INSTANCE','faces':candidates})
            checks.append({'case':f"{row['scene']}/{row['module']}/SAM/{pair['mask_id']}",'pass':passed})
            if not passed: failures.append(checks[-1])
    report={'status':'PASS_OBSERVED_GEOMETRY_WITH_EXPLICIT_SOURCE_AMBIGUITY' if not failures else 'FAIL',
            'allow_single_frozen_stack_regression':not failures,'threshold_sha256':hashlib.sha256(args.thresholds.read_bytes()).hexdigest(),
            'thresholds':limits,'checks':checks,'failures':failures,'source_ambiguities':ambiguous,
            'small_gt_view_instance_denominator':sum(r['gt_visible_denominator'] for r in rows),
            'sam_output_instance_denominator':sum(len(r['SAM_PAIRED']) for r in rows),
            'sam_confirmed_unique_instances':sum(len(p['entities'])==1 for r in rows for p in r['SAM_PAIRED']),
            'sam_ambiguous_instances':len(ambiguous),
            'interpretation':'Passing observed geometry does not certify instance separation or hidden volume in the two-box scenes.'}
    args.output.write_text(json.dumps(report,indent=2)); print(json.dumps({k:v for k,v in report.items() if k not in ('checks','source_ambiguities','thresholds')}))
    if failures: raise SystemExit(1)


if __name__=='__main__': main()

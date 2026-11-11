"""Independent S50 task limits and reproducible model identity.

The real CAD chain stays in its explicitly defined simulation coordinates.
Controller ranges are converted only with reviewed, hash-bound sign/offset
evidence. Missing evidence produces a report, never a task URDF with preview
ranges. Generation alone does not qualify installation, collision or timing.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from .robot_model_binding import ModelNotReady, rigid_transform
from .s50_qualification import AXIS_LABELS, controller_mapping_audit, has_source, timing_audit

SCHEMA = 's50_task_model_v1'
MODEL_ID = 'huayan_s50_2000'
MODEL_USAGE = 'installed_task_geometry'
TASK_DIRECTORY = 'assets/robots/huayan_s50_2000/task'
TASK_DEFINITION = TASK_DIRECTORY + '/task_model.json'
INPUT_CONFIG = 'configs/robots/huayan_s50_2000.yaml'


def _root(root=None):
    return Path(root).resolve() if root is not None else Path(__file__).resolve().parents[2]


def _sha(content):
    return hashlib.sha256(content).hexdigest()


def _json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False)+'\n').encode('utf-8')


def _relative(path, parent):
    path, parent = Path(path).resolve(), Path(parent).resolve()
    try:
        return os.path.relpath(path, parent).replace('\\', '/')
    except ValueError:
        # Windows has no relative path from C: output to D: source assets.
        # Keep the real reference, and mark this in the generated documents.
        # Do not copy assets, guess a drive or suppress other path errors.
        if os.name == 'nt' and path.drive.casefold() != parent.drive.casefold():
            return path.as_posix()
        raise


def _path_transport_contract(task, config):
    absolute = []
    def visit(value, pointer):
        if isinstance(value, dict):
            for key,item in value.items(): visit(item, pointer+'/'+str(key))
        elif isinstance(value, list):
            for index,item in enumerate(value): visit(item, pointer+'/'+str(index))
        elif isinstance(value, str) and Path(value).is_absolute():
            absolute.append(dict(field=pointer, path=value))
    visit(task,'task');visit(config,'config')
    return dict(mode='LOCAL_ABSOLUTE_REFERENCES_NOT_PORTABLE' if absolute else 'RELATIVE_REFERENCES_PRESERVE_DIRECTORY_STRUCTURE',
        absolute_local_references=absolute, asset_files_copied=False,
        note='Cross-drive Windows references cannot be relative. Absolute paths are local-only; regenerate on the destination machine instead of treating this artifact as relocatable.' if absolute else
             'Relative references require the same project and mapping-source directory structure.')


def _config(path):
    return yaml.safe_load(Path(path).read_text(encoding='utf-8'))


def _relocate_references(value, old_parent, new_parent):
    if isinstance(value, dict):
        return {key:_relocate_references(item, old_parent, new_parent) for key,item in value.items()}
    if isinstance(value, list):
        return [_relocate_references(item, old_parent, new_parent) for item in value]
    if isinstance(value, str) and value.startswith(('../','./')):
        path, separator, fragment = value.partition('#')
        return _relative(old_parent/path, new_parent)+(separator+fragment if separator else '')
    return value


def _evidence_files(entry, config_path, label):
    """A review assertion must identify the exact local reference being reviewed."""
    evidence = entry.get('source')
    sources = evidence if isinstance(evidence, list) else [evidence]
    errors, records = [], []
    for index, item in enumerate(sources):
        key = f'{label}.source[{index}]'
        if (not isinstance(item, dict) or not item.get('path') or not item.get('sha256')
                or not has_source(item.get('locator'))):
            errors.append(key+'.HASHED_LOCAL_REFERENCE_AND_LOCATOR_REQUIRED')
            continue
        path = (config_path.parent/item['path']).resolve()
        if not path.is_file():
            errors.append(key+'.FILE_MISSING')
        elif _sha(path.read_bytes()) != item['sha256']:
            errors.append(key+'.SHA256_MISMATCH')
        else:
            records.append(dict(path=str(path), sha256=item['sha256'], locator=item['locator']))
    return errors, records


def mapping_requirements(data, config_path):
    """Keep manufacturer ranges separate; verify the evidence behind each map."""
    mapped = controller_mapping_audit(data)
    errors = list(mapped['missing'])
    references = {}
    if not isinstance(data.get('published_controller_position_limits_rad'), dict):
        errors.append('task.PUBLISHED_CONTROLLER_LIMITS_REQUIRED_SEPARATELY')
    if (data.get('controller_mapping') or {}).get('equation') != 'q_controller = sign * q_sim + offset_rad':
        errors.append('task.CONTROLLER_MAPPING_EQUATION_MISMATCH')
    for label in AXIS_LABELS:
        entry = ((data.get('controller_mapping') or {}).get('joints') or {}).get(label) or {}
        problems, records = _evidence_files(entry, config_path, 'controller_mapping.'+label)
        errors.extend(problems); references[label] = records
    # This is a parser field for geometry, with the published version ambiguity
    # retained independently. Timing qualification remains a separate audit.
    velocity = np.asarray(data.get('joint_velocity_limits_rad_s'), dtype=float)
    if velocity.shape != (6,) or not np.isfinite(velocity).all() or np.any(velocity <= 0):
        errors.append('task.SIX_FINITE_POSITIVE_PUBLISHED_VELOCITIES_REQUIRED')
    if not has_source(data.get('joint_velocity_source')):
        errors.append('task.PUBLISHED_VELOCITY_SOURCE_REQUIRED')
    return dict(ready=not errors, missing=sorted(set(errors)), mapping=mapped,
                evidence_files=references, timing=timing_audit(data))


def physical_frame_derivations(definition, root=None):
    """Return measured and intrinsic frames without inventing a mating adapter."""
    from .robot_replacement_layout import freeze_non_robot_scene, frozen_baseline_layout
    root = _root(root)
    frozen = freeze_non_robot_scene(frozen_baseline_layout(root))
    cad_mount = rigid_transform(definition['frames']['mount']['CAD_from_frame_zero'], 'CAD_from_mount')
    base = next(link for link in definition['links'] if link['name'] == definition['base_link'])
    cad_base = rigid_transform(base['CAD_from_link_zero'], 'CAD_from_base')
    return dict(
        source_cad_definition_sha256=definition['definition_sha256'],
        CAD_from_simulation_mount=cad_mount.tolist(),
        simulation_mount_from_model_base=(np.linalg.inv(cad_mount) @ cad_base).tolist(),
        interpretation='CAD_REFERENCE_COORDINATE_DERIVATION_NOT_INSTALLED_CLOCKING',
        physical_mount_world=frozen['parameters']['world_from_physical_mount'],
        measured_model_frames=deepcopy(definition['frames']),
        original_tool_intrinsic_frames=deepcopy(frozen['tool_intrinsic']['intrinsic_frames']),
        tool_connector_extension_m=frozen['tool_intrinsic']['connector_extension_m'],
        actual_tool_contact_target_world=frozen['initial_physical_contact_target'],
        world_from_task_model_base=None, flange_from_project_tool_mount=None,
        flange_from_tool_tcp=None,
        equations=dict(world_from_task_model_base='world_from_physical_mount @ verified_mounting_from_base',
            flange_from_tool_tcp='verified_flange_from_project_tool_mount @ T_tool_mount_reference_virtual_task_tcp'),
        missing_references=[
            dict(item='controller_mapping.J1..J6', required='For every axis: controller reading at the supplied CAD reference pose, plus a signed positive-jog reference showing parent and child orientation, or a manufacturer frame/zero/sign table tied to this S50 revision.',
                 existing_manual='S-series-mechanical.pdf PDF pages 25 and 33 give dimensions/range drawings, not a six-axis controller-angle reading for this CAD pose.'),
            dict(item='controller_mapping.J2', required='A labelled S50 side view or recorded reference pose giving J2 controller angle and positive direction relative to CAD +X axis. The published [-190,10] degrees must not be attached directly to CAD q2=0.'),
            dict(item='controller_mapping.J3', required='A labelled elbow reference giving J3 controller angle at the measured straight CAD arrangement and the positive relative rotation. Symmetric ±160 degrees alone does not determine zero or sign.'),
            dict(item='installation.mount_clocking', required='Identify the installation +X direction on the S50 base through a labelled cable exit, dowel or bolt-pattern datum, and match it to the frozen physical installation orientation; PDF page 55 gives hole sizes, not this correspondence.'),
            dict(item='tool_interface.physical_connection', required='Mating drawing or measured assembly identifying S50 flange plane, existing 200 mm extension/tool mounting plane, axial separation and bolt-pattern clocking; identify any actual adapter geometry. PDF page 56 flange dimensions alone do not establish this assembly.'),
        ], physical_adapter_added=False, tool_connected=False)


def _task_documents(data, definition, source_urdf, source_srdf, input_config, output_directory, root):
    """Build documents after mapping admission; does not write them."""
    mapped = mapping_requirements(data, input_config)
    if not mapped['ready']:
        raise ModelNotReady(MODEL_ID, mapped['missing'])
    kin = data.get('kinematics') or {}
    for field in ('base_link','flange_link','tip_link'):
        if kin.get(field) != definition[field]:
            raise ValueError('TASK_CONFIG_CAD_FRAME_MISMATCH:'+field)
    if (kin.get('active_joint_names') != definition['joint_names']
            or kin.get('joint_limit_keys') != list(AXIS_LABELS)
            or kin.get('planning_group','manipulator') != 'manipulator'):
        raise ValueError('TASK_CONFIG_CAD_JOINT_ORDER_MISMATCH')
    urdf = ET.parse(source_urdf).getroot(); urdf.set('name', MODEL_ID)
    asset_root = root/'assets/robots/huayan_s50_2000'
    for mesh in urdf.iter('mesh'):
        uri = mesh.get('filename','')
        if not uri.startswith('package://'):
            mesh_path = (Path(source_urdf).parent/uri).resolve()
            if not mesh_path.is_relative_to(asset_root):
                raise ValueError('TASK_SOURCE_MESH_OUTSIDE_S50_ASSETS')
            mesh.set('filename','package://huayan_s50_2000/'+mesh_path.relative_to(asset_root).as_posix())
    by_name = {joint.get('name'): joint for joint in urdf.findall('joint')}
    task = deepcopy(definition); task.pop('definition_sha256', None)
    task.update(schema=SCHEMA, model_id=MODEL_ID, model_usage=MODEL_USAGE,
        source_cad_definition_sha256=definition['definition_sha256'],
        source_cad_definition_path=_relative(root/'assets/robots/huayan_s50_2000/generated/cad_kinematic_model.json', output_directory),
        source_input_config=dict(path=_relative(input_config, output_directory), sha256=_sha(input_config.read_bytes())),
        controller_mapping=deepcopy(data['controller_mapping']),
        published_controller_position_limits_rad=deepcopy(data['published_controller_position_limits_rad']),
        joint_position_limits_rad=mapped['mapping']['converted_limits_rad'],
        joint_velocity_limits_rad_s=deepcopy(data['joint_velocity_limits_rad_s']),
        velocity_semantics='PUBLISHED_PARSER_CAPS_NOT_RESOLUTION_OF_SPEED_VERSION_CONFLICT',
        time_parameterization=mapped['timing'],
        dynamics_qualified=False, installed_task_qualified=False,
        qualification_scope='SAME_REAL_CAD_KINEMATICS_WITH_REFERENCE_MAPPED_POSITION_LIMITS',
        generation_source='build_s50_task_model', simulation_zero_unchanged=True)
    # The preview-only slider limits/effort annotations do not belong to this
    # independent model; unknown effort is omitted from source task URDF.
    task.pop('preview_only', None); task.pop('preview_constraints', None)
    task['simulation_convention'].update(controller_zero_verified=True, controller_signs_verified=True,
        controller_reference_pose_verified=True)
    task['claims'] = ['REAL_CAD_CHAIN_UNCHANGED', 'CONTROLLER_LIMITS_MAPPED_WITH_REVIEWED_REFERENCES',
                      'GENERATION_IS_NOT_INSTALLATION_COLLISION_OR_TASK_ACCEPTANCE']
    for index, (label, name) in enumerate(zip(AXIS_LABELS, definition['joint_names'])):
        limit = by_name[name].find('limit')
        lo, hi = task['joint_position_limits_rad'][label]
        limit.set('lower', repr(lo)); limit.set('upper', repr(hi))
        limit.set('velocity', repr(float(data['joint_velocity_limits_rad_s'][index])))
        limit.attrib.pop('effort', None)
        task['joints'][index]['position_limits_rad'] = [lo, hi]
        task['joints'][index]['position_limit_semantics'] = 'CONVERTED_FROM_HASHED_CONTROLLER_REFERENCE'
        task['joints'][index].pop('preview_range_rad', None)
        task['joints'][index]['range_semantics'] = 'REFERENCE_MAPPED_MECHANICAL_POSITION_LIMITS'
        task['joints'][index]['controller_mapping_verified'] = True
        task['joints'][index]['controller_zero_offset_rad'] = data['controller_mapping']['joints'][label]['offset_rad']
        task['joints'][index]['controller_positive_sign'] = data['controller_mapping']['joints'][label]['sign']
    # Empty exclusions are conservative when no pairs are reviewed. Formal
    # task admission still separately requires the reviewed-pair evidence.
    semantic = ET.parse(source_srdf).getroot(); semantic.set('name', MODEL_ID)
    for pair in list(semantic.findall('disable_collisions')):
        semantic.remove(pair)
    reviewed = (data.get('collision') or {}).get('reviewed_self_collision_pairs')
    if reviewed is not None:
        if not isinstance(reviewed, list):
            raise ValueError('TASK_REVIEWED_SELF_COLLISION_PAIRS_MUST_BE_LIST_OR_NULL')
        known = {link.get('name') for link in urdf.findall('link')}; seen = set()
        for entry in reviewed:
            pair = [entry.get('link1'), entry.get('link2')] if isinstance(entry, dict) else entry
            if (not isinstance(pair, (list, tuple)) or len(pair)!=2 or pair[0]==pair[1]
                    or any(name not in known for name in pair) or tuple(sorted(pair)) in seen):
                raise ValueError('TASK_INVALID_REVIEWED_SELF_COLLISION_PAIR')
            seen.add(tuple(sorted(pair)))
            ET.SubElement(semantic, 'disable_collisions', link1=pair[0], link2=pair[1], reason='Reviewed')
    urdf_bytes = ET.tostring(urdf, encoding='utf-8', xml_declaration=True)+b'\n'
    srdf_bytes = ET.tostring(semantic, encoding='utf-8', xml_declaration=True)+b'\n'
    task['generated_files'] = dict(urdf=dict(path='s50_task.urdf', sha256=_sha(urdf_bytes)),
        srdf=dict(path='s50_task.srdf', sha256=_sha(srdf_bytes)))
    for role,payload in (('urdf',urdf_bytes),('srdf',srdf_bytes)):
        task[role+'_path'] = _relative(output_directory/('s50_task.'+role),root)
        task[role+'_sha256'] = _sha(payload)
    task['config_path'] = _relative(output_directory/'model.yaml',root)
    task['mapping_reference_files'] = {label:[dict(path=_relative(record['path'], output_directory),
        sha256=record['sha256'], locator=record['locator']) for record in records]
        for label,records in mapped['evidence_files'].items()}
    config = _relocate_references(data, input_config.parent, output_directory)
    config.update(model_usage=MODEL_USAGE, generated_definition='task_model.json',
        generated_definition_schema=SCHEMA, joint_position_limits_rad=deepcopy(task['joint_position_limits_rad']))
    config['kinematics'].update(urdf_path='s50_task.urdf', srdf_path='s50_task.srdf',
        package_map={'huayan_s50_2000':_relative(root/'assets/robots/huayan_s50_2000', output_directory)})
    config['cad_preview_config'] = _relative(root/'configs/robots/huayan_s50_2000_cad_preview.yaml',output_directory)
    # Relocate evidence paths because the generated config is beside its model.
    for label, records in mapped['evidence_files'].items():
        config['controller_mapping']['joints'][label]['source'] = [dict(path=_relative(record['path'], output_directory),
            sha256=record['sha256'], locator=record['locator']) for record in records]
    task['path_transport'] = _path_transport_contract(task,config)
    config['path_transport'] = deepcopy(task['path_transport'])
    return {'s50_task.urdf':urdf_bytes, 's50_task.srdf':srdf_bytes,
        'task_model.json':_json_bytes(task), 'model.yaml':yaml.safe_dump(config, sort_keys=False, allow_unicode=True).encode('utf-8')}


def generate_task_model(config_path=None, *, root=None, output_directory=None):
    """Return NOT_READY without writing a task model if mapping is unknown."""
    from .s50_cad_model import ensure_preview_assets, load_definition
    root = _root(root); config_path = Path(config_path or root/INPUT_CONFIG).resolve()
    output_directory = Path(output_directory or root/TASK_DIRECTORY).resolve()
    data = _config(config_path)
    if data.get('model_id') != MODEL_ID:
        raise ValueError('TASK_GENERATOR_REQUIRES_REAL_S50_MODEL_ID')
    definition = load_definition(root)
    assets = ensure_preview_assets(root)
    mapping = mapping_requirements(data, config_path)
    report = dict(schema='s50_task_model_generation_v1', status='MODEL_NOT_READY',
        task_model_generated=False, task_definition_sha256=None,
        source_cad_definition_sha256=definition['definition_sha256'], source_assets=assets,
        input_config=str(config_path), input_config_sha256=_sha(config_path.read_bytes()),
        output_directory=str(output_directory), missing=mapping['missing'],
        controller_limit_mapping=mapping['mapping'], timing=mapping['timing'],
        physical_frame_derivations=physical_frame_derivations(definition, root),
        installed_task_qualified=False, native_worker_launched=False, dynamics_qualified=False)
    if not mapping['ready']:
        return report
    folder = root/'assets/robots/huayan_s50_2000/urdf'
    documents = _task_documents(data, definition, folder/'s50_cad_preview.urdf',
        folder/'s50_cad_preview.srdf', config_path, output_directory, root)
    # Existing different work is never overwritten by a regeneration command.
    for name, payload in documents.items():
        destination = output_directory/name
        if destination.exists() and destination.read_bytes()!=payload:
            raise ValueError('TASK_MODEL_OUTPUT_ALREADY_EXISTS_WITH_DIFFERENT_IDENTITY:'+str(destination))
    output_directory.mkdir(parents=True, exist_ok=True)
    for name, payload in documents.items():
        (output_directory/name).write_bytes(payload)
    verify_task_definition(output_directory/'model.yaml', _config(output_directory/'model.yaml'))
    report.update(status='TASK_MODEL_GENERATED_NOT_INSTALLED_TASK_QUALIFIED', task_model_generated=True,
        task_definition_sha256=_sha(documents['task_model.json']),
        path_transport=json.loads(documents['task_model.json'])['path_transport'],
        generated_config=str(output_directory/'model.yaml'),
        generated_files={name:dict(path=str(output_directory/name), sha256=_sha(payload)) for name,payload in documents.items()})
    return report


def verify_task_definition(config_path, data):
    """Verify before any core/Pin/native binding; return the actual definition."""
    config_path = Path(config_path).resolve()
    if data.get('generated_definition_schema') != SCHEMA:
        raise ValueError('TASK_DEFINITION_SCHEMA_REQUIRED')
    declared = data.get('generated_definition')
    if not declared:
        raise ModelNotReady(MODEL_ID, ['task.GENERATED_DEFINITION_PATH_MISSING'])
    path = (config_path.parent/declared).resolve()
    if not path.is_file():
        raise ModelNotReady(MODEL_ID, ['task.GENERATED_DEFINITION_FILE_MISSING'])
    task = json.loads(path.read_text(encoding='utf-8'))
    if (task.get('schema') != SCHEMA or task.get('model_id') != MODEL_ID or data.get('model_id') != MODEL_ID
            or task.get('model_usage') != MODEL_USAGE or data.get('model_usage') != MODEL_USAGE):
        raise ValueError('TASK_PREVIEW_OR_MODEL_IDENTITY_MISMATCH')
    source = task.get('source_input_config') or {}
    source_path = (path.parent/source.get('path','__missing__')).resolve()
    if not source_path.is_file() or _sha(source_path.read_bytes())!=source.get('sha256'):
        raise ValueError('TASK_SOURCE_INPUT_CONFIG_CHANGED')
    source_data = _config(source_path)
    requirements = mapping_requirements(source_data, source_path)
    if not requirements['ready']:
        raise ModelNotReady(MODEL_ID, requirements['missing'])
    mapped = requirements['mapping']['converted_limits_rad']
    if task.get('joint_position_limits_rad') != mapped or data.get('joint_position_limits_rad') != mapped:
        raise ValueError('TASK_CONTROLLER_TO_SIMULATION_LIMITS_CHANGED')
    if (task.get('controller_mapping') != source_data.get('controller_mapping')
            or task.get('published_controller_position_limits_rad') != source_data.get('published_controller_position_limits_rad')):
        raise ValueError('TASK_CONTROLLER_REFERENCE_IDENTITY_CHANGED')
    source_definition_path = (path.parent/task['source_cad_definition_path']).resolve()
    source_definition = json.loads(source_definition_path.read_text(encoding='utf-8'))
    if _sha(source_definition_path.read_bytes()) != task.get('source_cad_definition_sha256'):
        raise ValueError('TASK_SOURCE_CAD_DEFINITION_CHANGED')
    for field in ('frames', 'links', 'fixed_joints', 'base_link', 'flange_link', 'tip_link', 'joint_names'):
        if task.get(field) != source_definition.get(field):
            raise ValueError('TASK_KINEMATIC_DEFINITION_CHANGED:'+field)
    for index, (actual, original) in enumerate(zip(task.get('joints', []), source_definition.get('joints', []))):
        if any(actual.get(key)!=original.get(key) for key in ('name','parent_link','child_link','origin','axis')):
            raise ValueError('TASK_KINEMATIC_JOINT_CHANGED:'+str(index))
    if len(task.get('joints', []))!=6:
        raise ValueError('TASK_REQUIRES_SIX_REAL_JOINTS')
    root = source_definition_path.parents[4]
    for role in ('urdf','srdf'):
        original = root/source_definition[role+'_path']
        if _sha(original.read_bytes()) != source_definition[role+'_sha256']:
            raise ValueError('TASK_SOURCE_CAD_'+role.upper()+'_CHANGED')
    from .s50_cad_model import read_mesh_bytes
    checked = set()
    for link in task['links']:
        for mesh in [link['visual_mesh'], *link['collision_meshes']]:
            if mesh['path'] in checked:
                continue
            if _sha(read_mesh_bytes(root/mesh['path']))!=mesh['sha256']:
                raise ValueError('TASK_SOURCE_MESH_CHANGED:'+mesh['path'])
            checked.add(mesh['path'])
    kin = data.get('kinematics') or {}
    for role in ('urdf','srdf'):
        record = (task.get('generated_files') or {}).get(role) or {}
        target = (path.parent/record.get('path','__missing__')).resolve()
        if (not target.is_file() or target != (config_path.parent/kin.get(role+'_path','__missing__')).resolve()
                or _sha(target.read_bytes())!=record.get('sha256')):
            raise ValueError('TASK_GENERATED_'+role.upper()+'_IDENTITY_MISMATCH')
    # Rebuild expected source transport documents; a changed task JSON cannot
    # bless a different axis/mesh/limit just by changing its adjacent URDF hash.
    documents = _task_documents(source_data, dict(source_definition, definition_sha256=task['source_cad_definition_sha256']),
        root/'assets/robots/huayan_s50_2000/urdf/s50_cad_preview.urdf',
        root/'assets/robots/huayan_s50_2000/urdf/s50_cad_preview.srdf', source_path, path.parent, root)
    for role in ('urdf','srdf'):
        if documents['s50_task.'+role] != (path.parent/task['generated_files'][role]['path']).read_bytes():
            raise ValueError('TASK_MODEL_NOT_REPRODUCIBLE_FROM_CAD_AND_MAPPING:'+role)
    if task != json.loads(documents['task_model.json']):
        raise ValueError('TASK_DEFINITION_NOT_REPRODUCIBLE_FROM_CAD_AND_MAPPING')
    if data != yaml.safe_load(documents['model.yaml']):
        raise ValueError('TASK_CONFIGURATION_NOT_REPRODUCIBLE_FROM_SOURCE_INPUT')
    task['definition_sha256'] = _sha(path.read_bytes())
    return task


def task_identity_audit(config_path):
    data = _config(config_path)
    try:
        task = verify_task_definition(config_path, data)
        return dict(ready=True, status='TASK_DEFINITION_IDENTITY_VERIFIED',
            definition_sha256=task['definition_sha256'], source_cad_definition_sha256=task['source_cad_definition_sha256'],
            path=str((Path(config_path).parent/data['generated_definition']).resolve()), missing=[])
    except (OSError, ValueError, TypeError, KeyError, ModelNotReady) as exc:
        missing = exc.missing if isinstance(exc, ModelNotReady) else [type(exc).__name__+':'+str(exc)]
        return dict(ready=False, status='TASK_MODEL_NOT_READY', definition_sha256=None, missing=missing)

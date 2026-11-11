"""Explicit serial-model bindings shared by FK and native model construction.

This module validates definitions; it never invents a missing joint or flange.
Dynamic inertials and effort data are deliberately outside its readiness gate.
"""
from dataclasses import dataclass
import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml


class ModelNotReady(ValueError):
    def __init__(self, model, missing):
        self.model = str(model)
        self.missing = tuple(dict.fromkeys(str(x) for x in missing))
        self.errors = list(self.missing)
        super().__init__('MODEL_NOT_READY: '+self.model+': '+', '.join(self.missing))


def rigid_transform(value, label):
    matrix = np.asarray(value, dtype=float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all()
            or not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-10, rtol=0)
            or not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-9, rtol=0)
            or not np.isclose(np.linalg.det(matrix[:3, :3]), 1., atol=1e-9, rtol=0)):
        raise ValueError('INVALID_RIGID_TRANSFORM: '+label)
    return matrix.copy()


@dataclass(frozen=True)
class RobotModelBinding:
    config_path: Path
    data: dict
    model_id: str
    urdf_path: Path
    srdf_path: Path
    active_joint_names: tuple
    base_link: str
    flange_link: str
    tip_link: str
    mounting_from_base: np.ndarray
    package_map: dict
    joint_limits: np.ndarray
    joint_velocity_limits: np.ndarray
    chain_child_links: tuple
    source_asset_sha256: dict

    @classmethod
    def from_config(cls, path):
        path = Path(path).resolve()
        if not path.is_file():
            raise ModelNotReady(path.stem, ['config:FILE_MISSING'])
        data = yaml.safe_load(path.read_text(encoding='utf-8'))
        if data.get('generated_definition_schema') == 's50_task_model_v1':
            # Task and preview may share FK geometry while having different
            # real position limits. Bind the exact generated definition too.
            from .s50_task_model import verify_task_definition
            verify_task_definition(path, data)
        model = data.get('model_id', data.get('model', path.stem))
        kin = data.get('kinematics') or {}; missing = []
        required = ('urdf_path', 'srdf_path', 'active_joint_names', 'base_link', 'flange_link', 'tip_link')
        for name in required:
            if not kin.get(name): missing.append('kinematics.'+name)
        mounting = kin.get('mounting_from_base')
        if mounting is None and kin.get('mounting_reference') == 'base_link_origin_is_mounting_flange_plane':
            mounting = np.eye(4)
        if mounting is None: missing.append('kinematics.mounting_from_base')
        if missing: raise ModelNotReady(model, missing)
        urdf = (path.parent / kin['urdf_path']).resolve(); srdf = (path.parent / kin['srdf_path']).resolve()
        for name, file in [('urdf_path', urdf), ('srdf_path', srdf)]:
            if not file.is_file(): missing.append('kinematics.'+name+':FILE_MISSING')
        if missing: raise ModelNotReady(model, missing)
        if not isinstance(kin['active_joint_names'], (list, tuple)):
            raise ModelNotReady(model, ['ACTIVE_JOINT_NAMES_REQUIRE_ORDERED_LIST'])
        names = tuple(kin['active_joint_names'])
        if len(names) != 6 or len(set(names)) != 6 or not all(isinstance(n, str) and n for n in names):
            raise ModelNotReady(model, ['ACTIVE_JOINTS_REQUIRE_SIX_UNIQUE_NAMES'])
        if kin.get('planning_group', 'manipulator') != 'manipulator':
            raise ModelNotReady(model, ['NATIVE_ADAPTER_REQUIRES_MANIPULATOR_GROUP'])
        xml = ET.parse(urdf).getroot()
        link_names = [link.get('name') for link in xml.findall('link')]
        joint_names = [joint.get('name') for joint in xml.findall('joint')]
        if (any(not name for name in link_names+joint_names) or len(set(link_names)) != len(link_names)
                or len(set(joint_names)) != len(joint_names)):
            raise ModelNotReady(model, ['URDF_DUPLICATE_OR_EMPTY_LINK_JOINT_NAME'])
        children = {}; edges = {name: [] for name in link_names}
        for joint in xml.findall('joint'):
            parent, child = joint.find('parent'), joint.find('child')
            a = parent.get('link') if parent is not None else None
            b = child.get('link') if child is not None else None
            if a not in edges or b not in edges or b in children:
                raise ModelNotReady(model, ['URDF_INVALID_TREE_REFERENCES'])
            children[b] = a; edges[a].append(b)
            if joint.find('mimic') is not None:
                raise ModelNotReady(model, ['INDEPENDENT_SIX_AXIS_MODEL_FORBIDS_MIMIC_JOINTS'])
        roots = set(link_names)-set(children)
        if len(roots) != 1:
            raise ModelNotReady(model, ['URDF_REQUIRES_SINGLE_ROOT'])
        pending = list(roots); visited = set()
        while pending:
            name = pending.pop()
            if name in visited:
                raise ModelNotReady(model, ['URDF_JOINT_TREE_CYCLE'])
            visited.add(name); pending.extend(edges[name])
        if len(visited) != len(link_names):
            raise ModelNotReady(model, ['URDF_JOINT_TREE_DISCONNECTED_OR_CYCLIC'])
        from .robot import URDFRobot
        robot = URDFRobot.from_urdf(urdf, names, kin['base_link'], kin['tip_link'], tool_length=0.)
        moving = [j for j in robot.joints if j.joint_type != 'fixed']
        if tuple(j.name for j in moving) != names or any(j.joint_type != 'revolute' for j in moving):
            raise ModelNotReady(model, ['ACTIVE_JOINT_ORDER_MUST_MATCH_COMPLETE_SIX_REVOLUTE_CHAIN'])
        chain_links = {kin['base_link'], *(joint.child for joint in robot.joints)}
        if kin['flange_link'] not in chain_links:
            raise ModelNotReady(model, ['FLANGE_NOT_ON_ACTIVE_CHAIN'])
        flange_index = next((index for index, joint in enumerate(robot.joints)
            if joint.child == kin['flange_link']), -1)
        last_active_index = max(index for index, joint in enumerate(robot.joints) if joint.joint_type != 'fixed')
        if flange_index < last_active_index:
            raise ModelNotReady(model, ['PHYSICAL_FLANGE_MUST_FOLLOW_LAST_ACTIVE_JOINT'])
        limits = robot.joint_limits
        if limits.shape != (6, 2) or not np.isfinite(limits).all() or np.any(limits[:, 0] >= limits[:, 1]):
            raise ModelNotReady(model, ['URDF_JOINT_POSITION_LIMITS_INVALID'])
        joints = {j.get('name'): j for j in xml.findall('joint')}
        velocities = []; errors = []
        for joint in robot.joints:
            try:
                rigid_transform(joint.origin, 'joint_origin:'+joint.name)
            except ValueError:
                errors.append('URDF_JOINT_ORIGIN_INVALID:'+joint.name)
        for joint in moving:
            limit = joints[joint.name].find('limit')
            speed = float(limit.get('velocity', 'nan')) if limit is not None else float('nan')
            if not np.isfinite(speed) or speed <= 0: errors.append('URDF_VELOCITY_MISSING:'+joint.name)
            velocities.append(speed)
            if joint.axis.shape != (3,) or not np.isfinite(joint.axis).all() or np.linalg.norm(joint.axis) < 1e-12:
                errors.append('URDF_AXIS_INVALID:'+joint.name)
        configured = data.get('joint_position_limits_rad')
        if configured is not None:
            keys = kin.get('joint_limit_keys', names)
            if len(keys) != 6 or any(k not in configured for k in keys): errors.append('JOINT_LIMIT_KEYS_MISMATCH')
            elif not np.allclose([configured[k] for k in keys], limits, atol=1e-8, rtol=0):
                errors.append('CONFIG_URDF_POSITION_LIMIT_MISMATCH')
        configured_speed = data.get('joint_velocity_limits_rad_s')
        if configured_speed is not None:
            if np.asarray(configured_speed).shape != (6,) or not np.allclose(configured_speed, velocities, atol=1e-8, rtol=0):
                errors.append('CONFIG_URDF_VELOCITY_LIMIT_MISMATCH')
        packages = {key: (path.parent / value).resolve() for key, value in (kin.get('package_map') or {}).items()}
        # Resolve an enclosing ROS package from its own package.xml, never from
        # a robot-name heuristic or a different model's mesh directory.
        for parent in urdf.parents:
            if (parent/'package.xml').is_file():
                package_name = ET.parse(parent/'package.xml').findtext('name')
                if package_name: packages.setdefault(package_name, parent)
                break
        binding = cls(path, data, model, urdf, srdf, names, kin['base_link'], kin['flange_link'], kin['tip_link'],
            rigid_transform(mounting, 'mounting_from_base'), packages, limits.copy(), np.asarray(velocities),
            tuple(j.child for j in moving), {})
        for mesh in xml.iter('mesh'):
            try: binding.resolve_mesh(mesh.get('filename', ''))
            except (ValueError, FileNotFoundError) as exc: errors.append(str(exc))
            scale = np.asarray([float(x) for x in mesh.get('scale', '1 1 1').split()])
            if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0): errors.append('URDF_MESH_SCALE_INVALID')
        semantic = ET.parse(srdf).getroot(); chain = semantic.find("group[@name='manipulator']/chain")
        if chain is None or chain.get('base_link') != binding.base_link or chain.get('tip_link') != binding.tip_link:
            errors.append('SRDF_CHAIN_BINDING_MISMATCH')
        link_names = set(link_names)
        for pair in semantic.findall('disable_collisions'):
            if pair.get('link1') not in link_names or pair.get('link2') not in link_names: errors.append('SRDF_UNKNOWN_COLLISION_LINK')
        if errors: raise ModelNotReady(model, errors)
        object.__setattr__(binding, 'source_asset_sha256', binding._current_asset_hashes())
        return binding

    def resolve_mesh(self, uri):
        if uri.startswith('package://'):
            package, slash, relative = uri[len('package://'):].partition('/')
            if not slash or package not in self.package_map: raise ValueError('UNBOUND_MESH_PACKAGE:'+package)
            package_root = self.package_map[package].resolve(); path = (package_root/relative).resolve()
            if not path.is_relative_to(package_root): raise ValueError('MESH_OUTSIDE_BOUND_PACKAGE')
        elif uri.startswith('file://'): path = Path(uri[len('file://'):]).resolve()
        else: path = (self.urdf_path.parent/uri).resolve()
        if not path.is_file(): raise FileNotFoundError('MESH_FILE_MISSING:'+str(path))
        return path

    def world_from_base(self, world_from_mount):
        return rigid_transform(world_from_mount, 'world_from_mount') @ self.mounting_from_base

    def _current_asset_hashes(self):
        files = {'config': self.config_path, 'urdf': self.urdf_path, 'srdf': self.srdf_path}
        if self.data.get('generated_definition_schema') == 's50_task_model_v1':
            import json
            definition_path = (self.config_path.parent/self.data['generated_definition']).resolve()
            definition = json.loads(definition_path.read_text(encoding='utf-8'))
            files['task_definition'] = definition_path
            files['task_source_input_config'] = (definition_path.parent/definition['source_input_config']['path']).resolve()
            files['task_source_cad_definition'] = (definition_path.parent/definition['source_cad_definition_path']).resolve()
            for label, entries in definition['mapping_reference_files'].items():
                for index, entry in enumerate(entries):
                    files[f'task_reference:{label}:{index}'] = (definition_path.parent/entry['path']).resolve()
        for mesh in ET.parse(self.urdf_path).getroot().iter('mesh'):
            uri = mesh.get('filename', '')
            files['mesh:'+uri] = self.resolve_mesh(uri)
        return {key: hashlib.sha256(path.read_bytes()).hexdigest() for key, path in sorted(files.items())}

    def verify_source_unchanged(self):
        try:
            current = self._current_asset_hashes()
        except (OSError, ET.ParseError, ValueError) as exc:
            raise ModelNotReady(self.model_id, ['MODEL_SOURCE_REVALIDATION_FAILED:'+str(exc)]) from exc
        if current != self.source_asset_sha256:
            raise ModelNotReady(self.model_id, ['MODEL_SOURCE_CHANGED_SINCE_BINDING'])

    def identity(self):
        # Identity describes the model actually loaded, not whatever mutable
        # files happen to exist later. Native construction revalidates sources.
        result = dict(model_id=self.model_id, config_sha256=self.source_asset_sha256['config'],
            urdf_sha256=self.source_asset_sha256['urdf'], srdf_sha256=self.source_asset_sha256['srdf'],
            mesh_sha256={key.removeprefix('mesh:'): value for key, value in self.source_asset_sha256.items() if key.startswith('mesh:')},
            active_joint_names=list(self.active_joint_names), base_link=self.base_link,
            flange_link=self.flange_link, tip_link=self.tip_link, mounting_from_base=self.mounting_from_base.tolist(),
            planning_group='manipulator', inertials_required=False, effort_limits_required=False)
        if 'task_definition' in self.source_asset_sha256:
            result.update(model_definition_schema='s50_task_model_v1',
                model_definition_sha256=self.source_asset_sha256['task_definition'])
        return result

    def fk_probe_configurations(self):
        low, high = self.joint_limits.T
        return [np.clip(np.zeros(6), low, high), low+.35*(high-low), low+.65*(high-low)]

"""Independent MuJoCo review of twelve cartons and permanent moving equipment.

Every prepared conveyor convex and J1 accessory participates in the same
distance checks as the official robot and suction tool. There is no mj_step.
"""
from __future__ import annotations

import hashlib
import xml.etree.ElementTree as ET

import numpy as np

from .asset_paths import resolve_asset
from .model import ROOT, check_transform, transform
from .native_contact import validate_task_contract
from .native_scene import TWELVE_SCHEMA, validate_fixed_interfaces, verify_scene
from .task_world import UnloadingTaskWorld


class NativeSceneWorld(UnloadingTaskWorld):
    def __init__(self, tool, scene):
        import trimesh
        if scene.get('schema') != TWELVE_SCHEMA: raise ValueError('NATIVE_REVIEW_REQUIRES_TWELVE_SCENE')
        verify_scene(scene)
        self.task_contract = validate_task_contract(scene)
        self._native_world_ids = {row['id'] for row in scene['native_world_objects']}
        self._extra_ready = False
        self._extra_bodies = []
        super().__init__(tool, scene)
        root = ET.fromstring(self.xml); assets = root.find('asset'); body_root = root.find('worldbody')
        def add(ident, vertices, kind, parent, T):
            if ident in self.geoms: raise ValueError('NATIVE_REVIEW_GEOMETRY_ID_DUPLICATE')
            vertices = np.asarray(vertices, float)
            if vertices.ndim != 2 or vertices.shape[1] != 3 or not np.isfinite(vertices).all():
                raise ValueError('NATIVE_REVIEW_CONVEX_VERTICES_INVALID')
            index = len(self._extra_bodies); mesh_name = 'native_review_mesh_'+str(index)
            body_name = 'native_review_body_'+str(index)
            ET.SubElement(assets, 'mesh', name=mesh_name, vertex=' '.join(map(str, vertices.reshape(-1))))
            body = ET.SubElement(body_root, 'body', name=body_name, mocap='true')
            ET.SubElement(body, 'geom', name=ident, type='mesh', mesh=mesh_name,
                contype='0', conaffinity='0', group='3', rgba='.15 .65 .8 .25', mass='.01')
            self.entries.append(dict(name=ident, kind=kind, owner=ident, parent_frame=parent))
            self._extra_bodies.append(dict(body=body_name, parent=parent, T=check_transform(T)))
        for row in scene['native_world_objects']:
            T = check_transform(row['pose']); vertices = np.asarray(row['convex_mesh']['vertices'])
            add(row['id'], vertices, 'environment', 'D', T)
            points = vertices@T[:3, :3].T+T[:3, 3]
            lo, hi = points.min(0), points.max(0)
            # The bounds below only define the named top-contact condition.
            # Collision queries use the full compiled convex mesh above.
            self.object_info[row['id']] = dict(id=row['id'], size_m=(hi-lo).tolist(),
                position_m=((hi+lo)/2).tolist(), owner=row.get('owner', row['id']))
        for row in scene['moving_attachments']:
            path = resolve_asset(ROOT, row['mesh'])
            if hashlib.sha256(path.read_bytes()).hexdigest() != row['mesh_sha256']:
                raise ValueError('NATIVE_REVIEW_ATTACHMENT_MESH_CHANGED')
            mesh = trimesh.load(path, force='mesh', process=False)
            if not mesh.is_watertight or not mesh.is_convex:
                raise ValueError('NATIVE_REVIEW_ATTACHMENT_CONVEX_REQUIRED')
            add(row['id'], mesh.vertices, 'permanent_attachment', row['parent_link'], row['T_parent_local'])
        self.xml = ET.tostring(root, encoding='unicode')
        self.model = self.mj.MjModel.from_xml_string(self.xml); self.data = self.mj.MjData(self.model)
        for entry in self.entries:
            entry['gid'] = self.mj.mj_name2id(self.model, self.mj.mjtObj.mjOBJ_GEOM, entry['name'])
        self.geoms = {e['name']: e for e in self.entries}
        interfaces = set()
        for row in validate_fixed_interfaces(scene):
            if row['attachment_id'] not in self.geoms:
                raise ValueError('NATIVE_REVIEW_FIXED_INTERFACE_UNBOUND')
            interfaces.add(frozenset(['robot_'+row['parent_link'], row['attachment_id']]))
        self._fixed_attachment_pairs = interfaces
        # The parent validates every named mount pair with mj_geomDistance
        # against -contact_tolerance_m. Keep these pairs in the query list.
        self.mounting_pairs.update(interfaces)
        self.pairs = []
        entries = [e for e in self.entries if e['kind'] != 'visual']
        tool_kinds = {'adapter', 'conservative_rigid', 'terminal_contact_band'}
        for i, a in enumerate(entries):
            for b in entries[i+1:]:
                if a['kind'] == b['kind'] == 'environment': continue
                if a['kind'] in tool_kinds and b['kind'] in tool_kinds: continue
                if (a['kind'] == b['kind'] == 'robot'
                        and frozenset([a['owner'], b['owner']]) in self.adjacent): continue
                if (a['kind'] == b['kind'] == 'permanent_attachment'
                        and a['parent_frame'] == b['parent_frame']): continue
                self.pairs.append((a, b))
        self.pair_a = np.array([a['gid'] for a, b in self.pairs])
        self.pair_b = np.array([b['gid'] for a, b in self.pairs])
        self.radii = self.model.geom_rbound[self.pair_a]+self.model.geom_rbound[self.pair_b]
        self._extra_ready = True
        self.set_state(np.asarray(self.current_q), 'initial')

    def valid(self, q, phase='initial', detail=False):
        accepted = super().valid(q, phase, detail)
        if (self.last_failure and frozenset(self.last_failure.get('pair', []))
                in self._fixed_attachment_pairs):
            self.last_failure['allowance'] = 'source_bound_J1_mount_interface'
        return accepted

    def set_state(self, q, phase):
        result = super().set_state(q, phase)
        if self._extra_ready:
            frames = self.robot.named_link_frames(q); frames['D'] = np.eye(4)
            for row in self._extra_bodies:
                self.pose_body(row['body'], frames[row['parent']]@row['T'])
            self.mj.mj_forward(self.model, self.data)
        return result

    def _support_pair(self, a, b):
        for upper, lower in ((a, b), (b, a)):
            if upper['kind'] != 'payload': continue
            if upper['name'] != self.box_id:
                allowed = self.boxes[upper['name']]['supported_by']
            elif self.phase in ('initial', 'approach', 'contact', 'loaded_lift', 'front_separation'):
                allowed = self.task_contract['original_support_names']
            elif self.phase in ('place_contact', 'retreat'):
                allowed = self.task_contract['receiving']['support_ids']
            else:
                allowed = []
            if lower['name'] not in allowed: continue
            A = self._pose(upper['name'])
            # Mesh compilation may recenter and rotate its geom frame. These
            # half-extents are already D-world AABBs, so use their D center
            # with identity rotation, never the compiled geom pose.
            B = (transform(self.object_info[lower['name']]['position_m'])
                 if lower['name'] in self._native_world_ids else self._pose(lower['name']))
            ah = np.asarray(self.object_info[upper['name']]['size_m'])/2
            bh = np.asarray(self.object_info[lower['name']]['size_m'])/2
            bottom = A[2, 3]-np.abs(A[2, :3])@ah; top = B[2, 3]+np.abs(B[2, :3])@bh
            overlap = np.minimum(A[:2, 3]+np.abs(A[:2, :3])@ah, B[:2, 3]+bh[:2])-np.maximum(
                A[:2, 3]-np.abs(A[:2, :3])@ah, B[:2, 3]-bh[:2])
            if (np.allclose(B[:3, :3], np.eye(3), atol=1e-8, rtol=0)
                    and bottom >= top-self.scene['contact_tolerance_m'] and np.all(overlap > 0)):
                return True
        return False

    def support_report(self, pose):
        receiver = self.task_contract['receiving']; bounds = np.asarray(receiver['bounds_D_m'])
        half = np.asarray(self.boxes[self.box_id]['size_m'])/2
        corners = np.array([[x, y, -half[2]] for x in (-half[0], half[0]) for y in (-half[1], half[1])])
        corners = corners@pose[:3, :3].T+pose[:3, 3]
        error = float(np.max(np.abs(corners[:, 2]-bounds[1, 2])))
        inside = (np.all(corners[:, :2] >= bounds[0, :2]-1e-9)
                  and np.all(corners[:, :2] <= bounds[1, :2]+1e-9))
        valid = bool(self.receiving_region == receiver['region_id'] and inside
                     and error <= self.scene['contact_tolerance_m'])
        return dict(valid=valid, coverage=1. if inside else 0.,
            surface_ids=list(receiver['support_ids']), bottom_height_error_m=error,
            mode='Source belt receiving rectangle plus actual compiled convex distance checks')

"""Read-only evidence display. No association, TF, geometry completion or admission.

Namespaces encode complete identities rather than truncating a hash to int32.
Keep tombstones for this publisher's lifetime so a reconnecting display can also
remove records missed while disconnected. Each sample contains every current ADD.
"""
from collections import Counter, defaultdict
from copy import deepcopy
import json
import math
from urllib.parse import quote

from geometry_msgs.msg import Point
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

SOURCE_FIELDS = ('module_id', 'capture_id', 'source_instance_id', 'face_id')
HISTORY = (.5, .5, .5, .65)
CONFLICT = (1., .15, .3, .9)
BLOCKED = (1., .65, .1, .8)
COMPLETE = (.2, .85, .3, .5)
PATCH = (.1, .8, 1., .9)


def marker_qos():
    return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _json(text, expected, default):
    try:
        value = json.loads(text) if text else default
    except RecursionError as exc:
        raise ValueError('JSON nesting exceeds display limit') from exc
    if not isinstance(value, expected):
        raise ValueError('invalid JSON structure')
    return value


def _source(value):
    key = tuple(value[k] for k in SOURCE_FIELDS)
    if not all(isinstance(k, str) and k for k in key):
        raise ValueError('incomplete surface source identity')
    return key


def _finite(values, length):
    try:
        return len(values) == length and all(isinstance(v, (float, int)) and math.isfinite(v) for v in values)
    except OverflowError:
        return False


class MarkerScene:
    def __init__(self, owner='unloading_world_bridge'):
        self.prefix = quote(owner, safe='') + '/evidence/'
        self.owned = set()
        self.diagnostics = []

    def _marker(self, key, kind, snapshot, frame, color):
        marker = Marker()
        marker.ns = self.prefix + quote(json.dumps(key, ensure_ascii=True, separators=(',', ':')), safe='')
        marker.id = 0
        marker.type, marker.action = kind, Marker.ADD
        marker.header.frame_id = frame
        # Refreshing the display must never refresh acquisition time.
        marker.header.stamp = deepcopy(snapshot.source_capture_time)
        marker.pose.orientation.w = 1.
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = color
        return marker

    def _text(self, key, text, position, snapshot, frame, color):
        marker = self._marker(key, Marker.TEXT_VIEW_FACING, snapshot, frame, color)
        marker.text = text
        marker.pose.position = Point(x=float(position[0]), y=float(position[1]), z=float(position[2]))
        marker.scale.z = .09
        return marker

    def clear(self):
        """Only explicitly owned ns/id pairs; no DELETEALL or sim-time lifetime."""
        return MarkerArray(markers=[Marker(ns=ns, id=mid, action=Marker.DELETE)
                                    for ns, mid in sorted(self.owned)])

    def render(self, snapshot, frame):
        markers, diagnostics = [], []
        reasons = list(snapshot.blocking_reasons)
        replay = 'HISTORICAL_REPLAY_DISPLAY_ONLY' in reasons
        historical = any(any(word in reason for word in ('STALE', 'TIME', 'CLOCK')) for reason in reasons)
        entries = []
        for cargo in snapshot.obstacles:
            try:
                raw = _json(cargo.raw_result_json, dict, {})
                if cargo.object_id:
                    identity = ('object', cargo.object_id)
                elif cargo.track_id:
                    identity = ('track', cargo.track_id)
                else:
                    source = raw.get('fusion_id') or cargo.source_instance_id
                    if not isinstance(source, str) or not source:
                        raise ValueError('missing object/fusion/source identity')
                    identity = ('frame-local', snapshot.source_capture_time.sec,
                                snapshot.source_capture_time.nanosec, source)
                key = (snapshot.source_epoch, *identity)
                entries.append((key, cargo, raw))
            except (ValueError, TypeError, KeyError) as exc:
                diagnostics.append(f'{cargo.source_instance_id}: {exc}')
        counts = Counter(key for key, _, _ in entries)
        for key, cargo, raw in entries:
            if counts[key] != 1:
                diagnostics.append(f'{key}: ambiguous object identity; geometry omitted')
                continue
            try:
                markers.extend(self._cargo(key, cargo, raw, snapshot, frame, historical, diagnostics))
            except (ValueError, TypeError, KeyError, AttributeError) as exc:
                diagnostics.append(f'{key}: {exc}; geometry omitted')
        status = 'PLANNING_ADMISSIBLE' if snapshot.planning_admissible else 'NOT PLANNING ADMISSIBLE'
        if historical:
            status = 'HISTORY / TIME INVALID - retained observation\n' + status
        if replay:
            status = 'HISTORICAL REPLAY / READ ONLY; capture=0 is a replay placeholder, NOT sensor time\n' + status
        unknown = [f'{r.region_id}: {r.reason} (frame={r.frame_id})' for r in snapshot.unknown_regions]
        text = ('UI LEGEND / fixed anchor, NOT a spatial region\n'
                'green: complete | cyan: observed patch / unknown volume\n'
                'amber: blocked/ineligible | red: conflict | gray: history\n'
                f'{status}\n' + ', '.join(reasons) +
                f'\nsource_epoch={snapshot.source_epoch} capture={snapshot.source_capture_time.sec}.'
                f'{snapshot.source_capture_time.nanosec:09d}\n'
                f'Unknown regions (no 3D extent): {len(unknown)}\n' + '\n'.join(unknown))
        if diagnostics:
            text += '\nDISPLAY DIAGNOSTICS:\n' + '\n'.join(sorted(set(diagnostics)))
        color = HISTORY if historical or replay else BLOCKED if not snapshot.planning_admissible or diagnostics else COMPLETE
        markers.append(self._text(('ui',), text, (0., 0., 2.8), snapshot, frame, color))
        current = {(m.ns, m.id) for m in markers}
        self.owned.update(current)
        deletes = [Marker(ns=ns, id=mid, action=Marker.DELETE) for ns, mid in sorted(self.owned - current)]
        self.diagnostics = sorted(set(diagnostics))
        return MarkerArray(markers=deletes + markers)

    def _cargo(self, key, cargo, raw, snapshot, frame, historical, diagnostics):
        fusion = raw.get('fusion_diagnostics', {})
        if not isinstance(fusion, dict):
            raise ValueError('invalid fusion_diagnostics')
        reduction = fusion.get('face_reduction')
        if reduction is not None and not isinstance(reduction, dict):
            raise ValueError('invalid face_reduction')
        conflict_records = (reduction or {}).get('conflicts', [])
        conflict = ('CONFLICT' in cargo.association_status.upper()
                    or 'CONFLICT' in cargo.geometry_validity.upper() or bool(conflict_records))
        replay = 'HISTORICAL_REPLAY_DISPLAY_ONLY' in snapshot.blocking_reasons
        history = historical or replay or 'STALE' in cargo.association_status.upper()
        color = (HISTORY if history else CONFLICT if conflict else
                 BLOCKED if not snapshot.planning_admissible or not cargo.candidate_eligible else COMPLETE)
        state = ('HISTORY / TIME INVALID; ' if history else '') + ('CONFLICT; ' if conflict else '')
        if replay:
            state = 'HISTORICAL REPLAY / READ ONLY; ' + ('CONFLICT; ' if conflict else '')
        state += (f'candidate_eligible={cargo.candidate_eligible}; association={cargo.association_status}'
                  f'; geometry_validity={cargo.geometry_validity}')
        if conflict_records:
            # Includes raw endpoints even when only one representative survived.
            if not isinstance(conflict_records, list) or not all(isinstance(c, dict) for c in conflict_records):
                raise ValueError('invalid conflict diagnostics')
            state += '\nraw conflict provenance=' + json.dumps([
                {k: c.get(k) for k in ('first', 'second', 'reason')} for c in conflict_records], sort_keys=True)
        markers = []
        if cargo.has_pose and cargo.has_full_dimensions:
            p, q = cargo.pose.position, cargo.pose.orientation
            if (cargo.pose_frame_id != frame or not _finite((p.x, p.y, p.z), 3)
                    or not _finite((q.x, q.y, q.z, q.w), 4)
                    or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > 1e-6
                    or not _finite(cargo.full_dimensions_m, 3) or min(cargo.full_dimensions_m) <= 0):
                diagnostics.append(f'{key}: invalid complete geometry/frame; cube omitted')
            else:
                marker = self._marker((*key, 'cube'), Marker.CUBE, snapshot, frame, color)
                marker.pose = deepcopy(cargo.pose)
                marker.scale.x, marker.scale.y, marker.scale.z = cargo.full_dimensions_m
                markers.append(marker)
                markers.append(self._text((*key, 'cube-label'), 'COMPLETE GEOMETRY\n' + state,
                    (p.x, p.y, p.z + max(cargo.full_dimensions_m)), snapshot, frame, color))
        surfaces = _json(cargo.observed_surfaces_json, list, [])
        by_source = defaultdict(list)
        for surface in surfaces:
            try:
                by_source[_source(surface)].append(surface)
            except (ValueError, TypeError, KeyError) as exc:
                diagnostics.append(f'{key}: invalid surface identity: {exc}')
        mapped = reduction is not None and 'representatives' in reduction
        references = reduction['representatives'] if mapped else [dict(zip(SOURCE_FIELDS, s)) for s in sorted(by_source)]
        if not isinstance(references, list):
            raise ValueError('invalid representatives list')
        selected, seen = [], set()
        for reference in references:
            try:
                source = _source(reference)
                if source in seen:
                    raise ValueError('duplicate representative reference')
                seen.add(source)
                matches = by_source.get(source, [])
                if len(matches) != 1:
                    raise ValueError(f'missing/ambiguous representative source {source}')
                selected.append((source, matches[0]))
            except (ValueError, TypeError, KeyError) as exc:
                diagnostics.append(f'{key}: {exc}')
        count_label = f'representatives={len(references)}' if mapped else 'raw display (no representative mapping)'
        count_label += f'; raw source patches={len(surfaces)}'
        for source, surface in selected:
            try:
                points = surface['corners_3d_m']
                if (surface.get('schema_version') != 'observed_surface_v1'
                        or surface.get('frame_id') != frame or len(points) != 4
                        or not all(_finite(p, 3) for p in points)):
                    raise ValueError('invalid observed surface geometry/frame/schema')
                patch_color = color if color != COMPLETE else PATCH
                marker = self._marker((*key, 'patch', *source), Marker.LINE_STRIP, snapshot, frame, patch_color)
                marker.scale.x = .012  # metres; no thickness/volume inference
                marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for p in [*points, points[0]]]
                markers.append(marker)
                label = ('OBSERVED PATCH / full volume UNKNOWN\n' + state + '\n' + count_label +
                         '\nsource=' + '/'.join(source) + f"; surface.capture_time={surface.get('capture_time', 'missing')}")
                markers.append(self._text((*key, 'patch-label', *source), label,
                    (points[0][0], points[0][1], points[0][2] + .1), snapshot, frame, patch_color))
            except (ValueError, TypeError, KeyError, IndexError) as exc:
                diagnostics.append(f'{key} {source}: {exc}; patch omitted')
        if not markers:
            diagnostics.append(f'{key}: no displayable geometry; {state}; {count_label}')
        return markers

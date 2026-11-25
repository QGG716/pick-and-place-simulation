"""Candidate search strategy switches (user decisions 2026-11-25), PYTHON flow only.

Every switch is optional; absent keeps the previous behaviour exactly. None of
them changes a check, a threshold or a collision rule: they change which
candidate / IK branch is tried first, add IK seeds, and add one extra
collision check of the approach joint line during screening. Every accepted
plan still passes every planning check and the mandatory final audit.

1. ``candidate_family_order``: candidates derived from the same reference
   contact pose that differ only by the lift offset along the face (1 cm
   steps) are near duplicates. The first of each family (in the unchanged
   priority order) is tried before any second member, and so on. Nothing is
   removed.
3. ``screen_approach_path``: the Pilz PTP approach is a straight line in joint
   space from q_start to the approach configuration. Screening checks that
   line with the approach-stage scene and the same native path checker
   (same edge resolution, same audit method) before a branch is chosen; a
   branch whose line is clear is preferred, a candidate with a clear line is
   ranked before one without. A failed line never rejects a candidate.
4. ``cross_cycle_inheritance``: the approach/contact configurations of the
   latest PLAN_READY plans with the same frozen q_start are stored next to the
   task records. A new task uses them as extra IK seeds and (optionally) tries
   IK branches of the same configuration family first. Paths, scenes and
   observations are never reused; every check runs on the current task.
"""
import json
import math
import os
import time
from pathlib import Path

import numpy as np

FAMILY_MODES = ('KEEP_ORDER', 'INTERLEAVE_LIFT_SIBLINGS')
SCREEN_MODES = ('ENDPOINTS_ONLY', 'APPROACH_JOINT_LINE')
INHERITANCE_MODES = ('IK_SEEDS', 'IK_SEEDS_AND_FAMILY_FIRST')
INHERITANCE_DEFAULTS = dict(max_entries=3, family_tolerance_rad=.5, max_q_start_deviation_deg=1.,
                            store='cross-cycle-inheritance.json')
STORE_SCHEMA = 'eco65_cross_cycle_inheritance_v1'


def _policy(config, key, modes, default):
    value = config.get(key)
    if value is None:
        return None
    if not isinstance(value, dict) or value.get('mode') not in modes or not str(value.get('basis', '')).strip():
        raise ValueError(f'{key.upper()}_INVALID')
    return None if value['mode'] == default else dict(value)


def family_policy(config):
    return _policy(config, 'candidate_family_order', FAMILY_MODES, 'KEEP_ORDER')


def screen_policy(config):
    return _policy(config, 'screen_approach_path', SCREEN_MODES, 'ENDPOINTS_ONLY')


def inheritance_policy(config):
    value = config.get('cross_cycle_inheritance')
    if value is None:
        return None
    if not isinstance(value, dict) or value.get('mode') not in INHERITANCE_MODES or not str(value.get('basis', '')).strip():
        raise ValueError('CROSS_CYCLE_INHERITANCE_INVALID')
    out = dict(INHERITANCE_DEFAULTS, **value)
    if type(out['max_entries']) is not int or not 1 <= out['max_entries'] <= 10:
        raise ValueError('CROSS_CYCLE_INHERITANCE_MAX_ENTRIES_INVALID')
    for key, low, high in (('family_tolerance_rad', .05, 1.5), ('max_q_start_deviation_deg', 0., 5.)):
        if type(out[key]) not in (int, float) or not math.isfinite(out[key]) or not low <= out[key] <= high:
            raise ValueError(f'CROSS_CYCLE_INHERITANCE_{key.upper()}_INVALID')
    store = out['store']
    if not isinstance(store, str) or not store.endswith('.json') or '/' in store or '\\' in store or store.startswith('.'):
        raise ValueError('CROSS_CYCLE_INHERITANCE_STORE_INVALID')
    return out


def config_issues(config):
    """Configuration errors of the three switches (empty when valid)."""
    errors = []
    for key, check, text in (
            ('candidate_family_order', family_policy,
             '{mode: KEEP_ORDER（缺省行为）或 INTERLEAVE_LIFT_SIBLINGS（同一基准吸取位姿只差上移量的近重复候选交错排序，不删除）, basis: 依据}'),
            ('screen_approach_path', screen_policy,
             '{mode: ENDPOINTS_ONLY（缺省行为）或 APPROACH_JOINT_LINE（预检时检查 q_start 到接近构型的关节直线，通过的分支和候选优先）, basis: 依据}'),
            ('cross_cycle_inheritance', inheritance_policy,
             '{mode: IK_SEEDS 或 IK_SEEDS_AND_FAMILY_FIRST, basis: 依据, max_entries: 1-10（缺省 3）, '
             'family_tolerance_rad: 0.05-1.5（缺省 0.5）, max_q_start_deviation_deg: 0-5（缺省 1）, store: 文件名（缺省 cross-cycle-inheritance.json）}')):
        try:
            check(config)
        except ValueError:
            errors.append(f'{key}：{text}')
        if config.get(key) is not None and config.get('planning_module') == 'NATIVE':
            errors.append(f'{key}：仅 PYTHON 规划流程支持')
    return errors


def family_key(index, candidate):
    """Near-duplicate family: the reference contact pose a lifted candidate was derived from."""
    search = candidate.get('contact_search') or {}
    reference = search.get('reference_tcp_pose')
    if reference is None:
        return ('INDEX', index)
    return ('REFERENCE', tuple(np.round(np.asarray(reference, float).ravel(), 6)))


def family_label(index, candidate):
    """Short stable label of family_key for the evidence records."""
    import hashlib
    kind, value = family_key(index, candidate)
    return f'INDEX:{value}' if kind == 'INDEX' else 'REF:' + hashlib.sha256(repr(value).encode()).hexdigest()[:12]


def interleave_families(rows, group, family):
    """Stable reorder of an already prioritised list.

    ``group(row)`` is the coarse class kept in order (e.g. CLEAR before
    NOT_CONCLUSIVE); inside it, members are ordered by their rank inside their
    family (first members of every family, then second members, ...), ties by
    the original position. Returns (rows, [(family rank) per returned row]).
    """
    seen = {}
    keyed = []
    for position, row in enumerate(rows):
        f = family(row)
        rank = seen.get(f, 0)
        seen[f] = rank + 1
        keyed.append(((group(row), rank, position), row, rank))
    keyed.sort(key=lambda item: item[0])
    return [row for _, row, _ in keyed], [rank for _, _, rank in keyed]


def wrapped_distance(a, b):
    d = np.asarray(a, float) - np.asarray(b, float)
    return np.abs((d + np.pi) % (2 * np.pi) - np.pi)


def same_family(q, reference, tolerance):
    return bool(np.max(wrapped_distance(q, reference)) <= tolerance)


def store_path(folder, policy):
    return Path(folder).parent / policy['store']


def load(policy, folder, q_start, cycle_mode):
    """Entries usable for this task (most recent first) and the evidence of the selection."""
    path = store_path(folder, policy)
    evidence = dict(policy={k: policy[k] for k in sorted(policy)}, store=str(path), entries=[], rejected=[])
    if not path.is_file():
        evidence['status'] = 'NO_STORE'
        return [], evidence
    try:
        data = json.loads(path.read_text())
        if data.get('schema') != STORE_SCHEMA or not isinstance(data.get('entries'), list):
            raise ValueError('schema')
    except (OSError, ValueError) as error:
        evidence['status'] = 'STORE_UNREADABLE_IGNORED'
        evidence['error'] = str(error)[:200]
        return [], evidence
    limit = np.deg2rad(policy['max_q_start_deviation_deg'])
    usable = []
    for entry in reversed(data['entries']):
        try:
            q0 = np.asarray(entry['q_start'], float)
            qa = np.asarray(entry['approach_q'], float)
            qc = np.asarray(entry['contact_q'], float)
            ok_shape = q0.shape == qa.shape == qc.shape == (6,) and np.all(np.isfinite(np.r_[q0, qa, qc]))
        except (KeyError, TypeError, ValueError):
            ok_shape = False
        if not ok_shape:
            evidence['rejected'].append(dict(task_id=entry.get('task_id'), reason='MALFORMED'))
            continue
        if entry.get('cycle_mode') != cycle_mode:
            evidence['rejected'].append(dict(task_id=entry.get('task_id'), reason='CYCLE_STRUCTURE_DIFFERS'))
            continue
        deviation = float(np.max(np.abs(q0 - np.asarray(q_start, float))))
        if deviation > limit:
            evidence['rejected'].append(dict(task_id=entry.get('task_id'), reason='Q_START_DIFFERS',
                                             max_deviation_deg=float(np.rad2deg(deviation))))
            continue
        usable.append(entry)
        if len(usable) >= policy['max_entries']:
            break
    evidence['entries'] = [dict(task_id=e.get('task_id'), approach_q=e['approach_q'], contact_q=e['contact_q'],
                                candidate_index=e.get('candidate_index')) for e in usable]
    evidence['status'] = 'USED' if usable else 'NO_MATCHING_ENTRY'
    return usable, evidence


def record(policy, folder, entry, keep=20):
    """Append one PLAN_READY entry (atomic replace; the newest ``keep`` entries are retained)."""
    path = store_path(folder, policy)
    entries = []
    if path.is_file():
        try:
            data = json.loads(path.read_text())
            if data.get('schema') == STORE_SCHEMA and isinstance(data.get('entries'), list):
                entries = data['entries']
        except (OSError, ValueError):
            entries = []
    entries = (entries + [dict(entry, recorded_utc_ns=time.time_ns())])[-keep:]
    tmp = path.with_name(path.name + f'.tmp{os.getpid()}')
    tmp.write_text(json.dumps(dict(schema=STORE_SCHEMA, entries=entries), ensure_ascii=False, indent=1))
    os.replace(tmp, path)
    return path


def family_first(solutions, inherited, tolerance):
    """Stable partition: solutions in the configuration family of an inherited approach first."""
    if not inherited:
        return list(solutions), []
    first = [q for q in solutions if any(same_family(q, e['approach_q'], tolerance) for e in inherited)]
    rest = [q for q in solutions if not any(q is f for f in first)]
    return first + rest, first

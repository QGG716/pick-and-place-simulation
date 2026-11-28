"""Brain configuration: configs/brain/eco65_brain.json plus the ignored remote connection file.

Host details stay in the ignored `outputs/workbench/remote_vlm.local.json`
(same rule as the remote detector); no password is ever stored.
"""
import json
import math
from pathlib import Path

from . import AUTHORITY

ROOT = Path(__file__).resolve().parents[4]
DEFAULT_PATH = ROOT / 'configs/brain/eco65_brain.json'


def _hashes(path):
    from .vlm_client import load_model_hashes
    return load_model_hashes(path)


def load(path=DEFAULT_PATH):
    """Return (settings, issues). Issues disable the affected part only, never the workbench."""
    issues = []
    path = Path(path)
    try:
        s = json.loads(path.read_text())
    except (OSError, ValueError) as error:
        return None, [f'智能助手配置无法读取：{path}（{error}）']
    s['source'] = str(path)
    if s.get('authority') != AUTHORITY:
        issues.append(f'brain.authority 必须为 {AUTHORITY}（VLM 只做提示，用户 2026-11-28）')
    if not s.get('basis'):
        issues.append('brain.basis：记录用户决定来源')
    vlm = s.setdefault('vlm', {})
    remote_file = vlm.get('remote_config')
    remote = dict(vlm.get('remote') or {})
    if remote_file:
        rf = ROOT / remote_file
        if rf.is_file():
            remote.update(json.loads(rf.read_text()))
        else:
            issues.append(f'远程 VLM 连接文件不存在：{remote_file}')
    vlm['remote'] = remote
    model = vlm.get('model') or {}
    try:
        vlm['model_files'] = _hashes(ROOT / model['hash_file'])
    except (OSError, KeyError, ValueError) as error:
        vlm['model_files'] = None
        issues.append(f'VLM 模型哈希清单无效：{error}')
    a = s.setdefault('assessment', {})
    interval = a.get('interval_s', 15.)
    if not isinstance(interval, (int, float)) or not math.isfinite(interval) or not 3 <= interval <= 600:
        issues.append('assessment.interval_s：3–600 s')
    v = s.get('voice') or {}
    if v.get('enabled'):
        v['model_root'] = str(ROOT / v['model_root'])
        try:
            v['model_hashes'] = _hashes(ROOT / v['hash_file'])
        except (OSError, KeyError, ValueError) as error:
            v['enabled'] = False
            issues.append(f'语音模型哈希清单无效，语音关闭：{error}')
    s['record_root'] = str(ROOT / s.get('record_root', 'outputs/brain'))
    return s, issues


def vlm_issues(s):
    from .vlm_client import remote_config_issues
    if s is None:
        return ['未配置']
    out = remote_config_issues(s['vlm'].get('remote'))
    if not s['vlm'].get('model_files'):
        out.append('VLM 模型哈希清单')
    return out

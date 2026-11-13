"""Committed stand-ins for inputs that exist only on the field PC.

Tests must reproduce off the field PC, so they never read its ignored local
files. Field runs keep reading those files unchanged.
"""
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# User-approved (2026-11-13) byte copy of the field launcher's ignored
# outputs/workbench/installation.local.json: world = robot base, camera on
# link_1, CAD reference TCP from configs/local/eco65_desktop/tool.json. It holds
# no physical TCP or calibration measurement (physical_tcp_verified=false).
INSTALLATION = ROOT / 'tests/fixtures/eco65_installation_cad_reference.json'


def use_committed_installation(config):
    """Point a loaded single-carton (``config['execution']``) or controlled config at the fixture."""
    (config['execution'] if 'execution' in config else config)['installation'] = str(INSTALLATION)
    return config


def use_synthetic_remote_detector(config, tmp_path):
    """Complete but unreachable remote GPU settings, for configuration checks only.

    The field PC keeps the real host, user and key in an ignored local file.
    Tests neither need nor read it, and nothing here is ever connected to:
    ``.invalid`` never resolves and the key file is a placeholder.
    """
    key = Path(tmp_path) / 'synthetic_test_key'
    key.write_text('SYNTHETIC TEST PLACEHOLDER, NOT A KEY\n')
    os.chmod(key, 0o600)
    config['detector']['remote'] = dict(config['detector'].get('remote') or {}, host='gpu.invalid', port=22,
                                        user='synthetic-test', identity_file=str(key))
    return config

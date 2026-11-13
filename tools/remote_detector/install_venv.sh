#!/bin/bash
# Offline, hash-pinned install of the remote inference venv from verified wheels.
# Runs on the GPU server. Every package must be in wheels/ and match the
# SHA-256 recorded by fetch_wheels.py; pip never contacts an index.
set -euo pipefail
cd /root/autodl-tmp/eco65-gpu-inference
V=/root/eco65-gpu-infer-venv
grep '^VERIFIED ' fetch.log fetch2.log 2>/dev/null | sed 's/^[^:]*://' | sort -u > verified.txt || true
python3 - <<'EOF'
from pathlib import Path
import re
from pip._vendor.packaging.utils import parse_wheel_filename
verified = {}
for line in Path('verified.txt').read_text().split('\n'):
    if line.startswith('VERIFIED '):
        _, name, digest, _ = line.split(' ')
        verified[name] = digest
wanted = dict(l.split('==') for l in Path('closure.txt').read_text().split())
lines = []
for name, digest in sorted(verified.items()):
    project, version, *_ = parse_wheel_filename(name)
    key = re.sub(r'[-_.]+', '-', str(project)).lower()
    if key in wanted and str(version) == wanted[key]:
        lines.append(f'{key}=={wanted[key]} --hash=sha256:{digest}')
missing = set(wanted) - {l.split('==')[0] for l in lines}
if missing: raise SystemExit('MISSING_VERIFIED_WHEELS: '+' '.join(sorted(missing)))
Path('requirements.lock').write_text('\n'.join(lines)+'\n')
print('locked', len(lines))
EOF
rm -rf "$V"
python3 -m venv "$V"
"$V/bin/python" -m pip install -q --no-index --find-links wheels --require-hashes -r requirements.lock
"$V/bin/python" -m pip freeze > venv-freeze.txt
"$V/bin/python" -m pip check
"$V/bin/python" -c "import torch,transformers,numpy,PIL;print('REMOTE_VENV_OK',torch.__version__,torch.version.cuda,torch.cuda.is_available(),torch.cuda.get_device_name(0),transformers.__version__,numpy.__version__,PIL.__version__)"

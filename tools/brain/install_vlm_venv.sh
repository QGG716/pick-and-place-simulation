#!/bin/bash
# Runs on the GPU server: separate venv for the Qwen VLM so the detector venv
# (torch 2.5.1, transformers 4.44) stays untouched. Qwen3.5 needs
# transformers 5.x, which needs torch >= 2.6. pip only resolves the wheel list
# (--dry-run --report); aria2c fetches the wheels in parallel (single-stream
# pip downloads are throttled on these servers) and pip installs them offline
# with the index's SHA-256 checked (--require-hashes).
set -euo pipefail
V=/root/eco65-vlm-venv
D=/root/autodl-tmp/eco65-vlm/wheels
IDX=https://mirrors.cloud.tencent.com/pypi/simple
PKGS="torch==2.8.0 torchvision==0.23.0 transformers==5.20.2 accelerate pillow numpy flash-linear-attention==0.4.0 fla-core==0.4.0"
rm -rf "$V"; python3 -m venv "$V"
"$V/bin/python" -m pip install -q --timeout 120 --retries 10 -i $IDX "pip>=24"
mkdir -p "$D"; cd "$D"
"$V/bin/python" -m pip install -q --timeout 120 --retries 10 -i $IDX --dry-run --ignore-installed --report report.json $PKGS
"$V/bin/python" - <<'PY'
import json
r = json.load(open('report.json'))
with open('urls.txt', 'w') as u, open('lock.txt', 'w') as l:
    for item in r['install']:
        info = item['download_info']; url = info['url']; name = url.split('/')[-1].split('#')[0]
        digest = info['archive_info']['hashes']['sha256']
        u.write(f'{url}\n  out={name}\n')
        l.write(f"{name} --hash=sha256:{digest}\n")
print(len(r['install']), 'wheels')
PY
aria2c -q -x 8 -s 8 -j 6 -k 4M --file-allocation=none --allow-overwrite=false --auto-file-renaming=false --conditional-get=true -i urls.txt
sed "s|^|$D/|" lock.txt > lock.abs.txt
"$V/bin/python" -m pip install -q --no-index --no-deps --require-hashes -r lock.abs.txt
"$V/bin/python" -m pip freeze > /root/eco65-vlm-venv-freeze.txt
"$V/bin/python" -c "import torch,transformers,fla;from transformers.models.qwen3_5 import modeling_qwen3_5;print('VLM_VENV_OK',torch.__version__,torch.cuda.is_available(),transformers.__version__,'fla',fla.__version__)"

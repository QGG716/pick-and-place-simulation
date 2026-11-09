#!/usr/bin/env bash
set -euo pipefail
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 22.04 ]]; then
  echo 'This installer targets Ubuntu 22.04.' >&2
  exit 2
fi
cd "$(dirname "$0")/.."
if [[ $EUID == 0 ]]; then privilege=(); else privilege=(sudo); fi
"${privilege[@]}" apt-get update
"${privilege[@]}" apt-get install -y python3-venv python3-tk fonts-noto-cjk libgl1-mesa-dri libegl1 libopengl0
python3 -m venv .venv-workbench
.venv-workbench/bin/python -m pip install -e '.[dev,workbench,workbench-scene]'
echo 'Installed. Run .venv-workbench/bin/eco65-workbench. No device was connected.'

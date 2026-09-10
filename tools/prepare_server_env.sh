#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
GPU_VENV=${GPU_VENV:-/root/v05-gpu-venv}
MOGE_SOURCE=${MOGE_SOURCE:-/root/moge-src-v2}
UTILS3D_SOURCE=${UTILS3D_SOURCE:-}
ARTIFACT_ROOT=${ARTIFACT_ROOT:-/root/autodl-tmp/v05-acceptance}
HF_CACHE=${HF_CACHE:-/root/autodl-tmp/huggingface}
HF_ENDPOINT=${HF_ENDPOINT:-https://hf-mirror.com}
export PIP_INDEX_URL=${PIP_INDEX_URL:-https://pypi.org/simple}
INSTALL_ROS=0
if [[ ${1:-} == --install-ros ]]; then INSTALL_ROS=1; fi

grep -q '^VERSION_ID="22.04"' /etc/os-release
[[ $(uname -m) == x86_64 ]]
if (( INSTALL_ROS )); then
  apt-get update
  DEBIAN_FRONTEND=noninteractive apt-get install -y \
    ros-humble-ros-base ros-humble-control-msgs ros-humble-tf2-geometry-msgs \
    ros-humble-visualization-msgs ros-humble-ament-cmake \
    ros-humble-ament-cmake-pytest ros-humble-launch-testing-ament-cmake \
    python3-colcon-common-extensions python3-rosdep python3-pytest python3.10-venv
fi
test -f /opt/ros/humble/setup.bash
test -d "$MOGE_SOURCE/.git"
[[ $(git -C "$MOGE_SOURCE" rev-parse HEAD) == b942f00bdc2a2a23ebb474fbe034d487e6dcceec ]]

/usr/bin/python3.10 -m venv "$GPU_VENV"
"$GPU_VENV/bin/python" -m pip install pip==25.2 setuptools==80.9.0 wheel==0.45.1
"$GPU_VENV/bin/python" -m pip install --no-deps -r "$ROOT/configs/deployment/pytorch-cu128-requirements.txt"
"$GPU_VENV/bin/python" -m pip install --no-deps -r "$ROOT/configs/deployment/pytorch-cu128-runtime-requirements.txt"
"$GPU_VENV/bin/python" -m pip install -r "$ROOT/configs/deployment/gpu-vision-requirements.txt"
if [[ -n $UTILS3D_SOURCE ]]; then
  test -d "$UTILS3D_SOURCE/.git"
  [[ $(git -C "$UTILS3D_SOURCE" rev-parse HEAD) == 3fab839f0be9931dac7c8488eb0e1600c236e183 ]]
  "$GPU_VENV/bin/python" -m pip install --no-deps "$UTILS3D_SOURCE"
else
  "$GPU_VENV/bin/python" -m pip install --no-deps \
    'utils3d @ git+https://github.com/EasternJournalist/utils3d.git@3fab839f0be9931dac7c8488eb0e1600c236e183'
fi
# MoGe's application metadata pulls Gradio and a separate CLI pipeline that the
# headless model API does not use.  Expose the exact audited source instead of
# installing those unrelated dependencies into the worker environment.
SITE_PACKAGES=$("$GPU_VENV/bin/python" -c 'import site; print(site.getsitepackages()[0])')
printf '%s\n' "$MOGE_SOURCE" > "$SITE_PACKAGES/moge-v2-fixed-source.pth"
"$GPU_VENV/bin/python" -m pip check
"$GPU_VENV/bin/python" -c 'import moge, pathlib, sys; expected=pathlib.Path(sys.argv[1]).resolve(); actual=pathlib.Path(moge.__file__).resolve(); assert expected in actual.parents, (expected, actual); print(actual)' "$MOGE_SOURCE"

mkdir -p "$ARTIFACT_ROOT" "$HF_CACHE"
HF_ENDPOINT="$HF_ENDPOINT" "$GPU_VENV/bin/python" "$ROOT/tools/prepare_vision_models.py" \
  --cache-dir "$HF_CACHE" --output "$ARTIFACT_ROOT/model-manifest.json" --endpoint "$HF_ENDPOINT"
"$GPU_VENV/bin/python" - <<'PY'
import json, torch
assert torch.cuda.is_available()
x = torch.arange(4096, device="cuda", dtype=torch.float32)
y = (x.square().sum()).item()
torch.cuda.synchronize()
print(json.dumps({"torch": torch.__version__, "torch_cuda": torch.version.cuda,
                  "device": torch.cuda.get_device_name(0), "tensor_result": y}))
PY
"$GPU_VENV/bin/python" -m pip freeze > "$ARTIFACT_ROOT/gpu-pip-freeze.txt"

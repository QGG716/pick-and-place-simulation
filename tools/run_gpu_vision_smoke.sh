#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
GPU_VENV=${GPU_VENV:-/root/v05-gpu-venv}
VISION_ROOT=${VISION_ROOT:-/root/vision-upstream}
ARTIFACT_ROOT=${ARTIFACT_ROOT:-/root/autodl-tmp/v05-acceptance}
MODEL_MANIFEST=${MODEL_MANIFEST:-$ARTIFACT_ROOT/model-manifest.json}
RUN_ID=${RUN_ID:-smoke-$(date -u +%Y%m%dT%H%M%SZ)}
RUN="$ARTIFACT_ROOT/gpu/$RUN_ID"
mkdir -p "$RUN"
test -x "$GPU_VENV/bin/python"
test -f "$MODEL_MANIFEST"
timeout --signal=TERM 1900 "$GPU_VENV/bin/python" "$ROOT/tools/run_gpu_worker_smoke.py" \
  --gpu-python "$GPU_VENV/bin/python" --vision-root "$VISION_ROOT" \
  --model-manifest "$MODEL_MANIFEST" --output-root "$RUN/worker-runs" \
  --result "$RUN/result.json" --timeout 1800 2>&1 | tee "$RUN/smoke.log"

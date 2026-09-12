#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ISAAC_PYTHON="${ISAAC_PYTHON:-/root/autodl-tmp/envs/isaacsim-clean/bin/python}"
GPU_PYTHON="${GPU_PYTHON:-/root/v05-gpu-venv/bin/python}"
VISION_ROOT="${VISION_ROOT:-/root/vision-fixed}"
FEASIBILITY_ROOT="${FEASIBILITY_ROOT:-/root/autodl-tmp/m710-official-dynamics-20260910/repo-feasibility-core-648e177}"
MODEL_MANIFEST="${MODEL_MANIFEST:-/root/autodl-tmp/v05-acceptance/model-manifest.json}"
OUTPUT_ROOT="${ISAAC_PERCEPTION_OUTPUT:-${PROJECT_ROOT}/outputs/isaac_perception_validation}"
BUNDLE_DIR="${OUTPUT_ROOT}/scene_bundle"
USD_DIR="${OUTPUT_ROOT}/usd"

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
  echo "Isaac acceptance FAIL: missing Isaac Python at ${ISAAC_PYTHON}" >&2
  exit 2
fi
if [[ ! -x "${GPU_PYTHON}" || ! -d "${VISION_ROOT}" || ! -f "${MODEL_MANIFEST}" ]]; then
  echo "Isaac acceptance FAIL: resident vision worker environment is incomplete" >&2
  exit 2
fi
if [[ ! -d "${FEASIBILITY_ROOT}" ]]; then
  echo "Isaac acceptance FAIL: latest detached feasibility checkout is missing" >&2
  exit 2
fi
if [[ "${OMNI_KIT_ACCEPT_EULA:-}" != "YES" ]]; then
  echo "Isaac acceptance FAIL: set OMNI_KIT_ACCEPT_EULA=YES after accepting the NVIDIA EULA" >&2
  exit 2
fi

mkdir -p "${OUTPUT_ROOT}" "${BUNDLE_DIR}" "${USD_DIR}"
PERCEPTION_COMMIT="$(git -C "${PROJECT_ROOT}" rev-parse HEAD)"
export PYTHONPATH="${PROJECT_ROOT}/packages/unloading_contracts/src:${PROJECT_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"
export OMNI_KIT_ALLOW_ROOT="${OMNI_KIT_ALLOW_ROOT:-1}"
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp/runtime-root}"

"${ISAAC_PYTHON}" "${PROJECT_ROOT}/tools/build_isaac_perception_bundle.py" \
  --output "${BUNDLE_DIR}" --perception-commit "${PERCEPTION_COMMIT}" \
  --isaac-version "6.0.1.0" --run-id "isaac-perception-${PERCEPTION_COMMIT:0:12}"

"${ISAAC_PYTHON}" "${PROJECT_ROOT}/tools/analyze_j1_mast_preflight.py" \
  --bundle-directory "${BUNDLE_DIR}" --output "${OUTPUT_ROOT}"

"${ISAAC_PYTHON}" "${PROJECT_ROOT}/scripts/isaacsim_perception_capture.py" \
  --bundle-directory "${BUNDLE_DIR}" --project-root "${PROJECT_ROOT}" \
  --feasibility-root "${FEASIBILITY_ROOT}" --usd-directory "${USD_DIR}" \
  --output "${OUTPUT_ROOT}" --fps 20 --seconds 12

"${ISAAC_PYTHON}" - "${OUTPUT_ROOT}" <<'PY'
import json
from pathlib import Path
import sys
status = json.loads((Path(sys.argv[1]) / "run_status.json").read_text())
if status.get("status") != "PASS":
    raise SystemExit(f"Isaac capture failed closed: {status.get('reason', status.get('status'))}")
PY

"${ISAAC_PYTHON}" "${PROJECT_ROOT}/tools/finalize_isaac_perception_capture.py" \
  --bundle-directory "${BUNDLE_DIR}" --capture-directory "${OUTPUT_ROOT}" --project-root "${PROJECT_ROOT}"

"${ISAAC_PYTHON}" "${PROJECT_ROOT}/tools/run_isaac_mode_b1.py" \
  --capture-directory "${OUTPUT_ROOT}" --bundle-directory "${BUNDLE_DIR}" \
  --vision-root "${VISION_ROOT}" --gpu-python "${GPU_PYTHON}" \
  --model-manifest "${MODEL_MANIFEST}" --worker-timeout 1200 --fps 20 --seconds 12

"${ISAAC_PYTHON}" "${PROJECT_ROOT}/tools/run_isaac_rgbd_geometry.py" \
  --capture-directory "${OUTPUT_ROOT}" --bundle-directory "${BUNDLE_DIR}" \
  --vision-root "${VISION_ROOT}" --upstream-python "${GPU_PYTHON}" --timeout 1200

"${ISAAC_PYTHON}" - "${OUTPUT_ROOT}" <<'PY'
import json
from pathlib import Path
import sys
root = Path(sys.argv[1])
run = json.loads((root / "run_status.json").read_text())
domain = json.loads((root / "domain_summary.json").read_text())
if (run.get("status") != "PASS" or domain.get("status") not in {"PASS", "PASS_WITH_KNOWN_FEASIBILITY_GAP"}
        or domain.get("mode_c_moge") != "PASS" or domain.get("mode_b_staged_rgbd") != "PASS"):
    raise SystemExit("Isaac perception acceptance did not produce PASS summaries")
print(json.dumps({
    "status": "PASS",
    "mode_a": domain["mode_a"],
    "mode_b_staged_rgbd": domain["mode_b_staged_rgbd"],
    "mode_c_moge": domain["mode_c_moge"],
    "video": run["video"],
    "video_sha256": run["video_sha256"],
    "output": str(root.resolve()),
}, indent=2))
PY

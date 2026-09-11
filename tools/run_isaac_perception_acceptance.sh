#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
ISAAC_PYTHON="${ISAAC_PYTHON:-/root/autodl-tmp/envs/isaacsim-clean/bin/python}"
OUTPUT_ROOT="${ISAAC_PERCEPTION_OUTPUT:-${PROJECT_ROOT}/outputs/isaac_perception_validation}"
BUNDLE_DIR="${OUTPUT_ROOT}/scene_bundle"
USD_DIR="${OUTPUT_ROOT}/usd"

if [[ ! -x "${ISAAC_PYTHON}" ]]; then
  echo "Isaac acceptance FAIL: missing Isaac Python at ${ISAAC_PYTHON}" >&2
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

"${ISAAC_PYTHON}" "${PROJECT_ROOT}/scripts/isaacsim_perception_capture.py" \
  --bundle-directory "${BUNDLE_DIR}" --project-root "${PROJECT_ROOT}" \
  --usd-directory "${USD_DIR}" --output "${OUTPUT_ROOT}" --width 640 --height 480 --fps 20 --seconds 12

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

"${ISAAC_PYTHON}" - "${OUTPUT_ROOT}" <<'PY'
import json
from pathlib import Path
import sys
root = Path(sys.argv[1])
run = json.loads((root / "run_status.json").read_text())
domain = json.loads((root / "domain_summary.json").read_text())
if run.get("status") != "PASS" or domain.get("status") != "PASS":
    raise SystemExit("Isaac perception acceptance did not produce PASS summaries")
print(json.dumps({
    "status": "PASS",
    "mode_a": domain["mode_a"],
    "mode_b1": domain["mode_b1"],
    "video": run["video"],
    "video_sha256": run["video_sha256"],
    "output": str(root.resolve()),
}, indent=2))
PY

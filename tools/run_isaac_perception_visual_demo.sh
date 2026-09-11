#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="${PROJECT_ROOT:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
export PROJECT_ROOT
export ISAAC_PERCEPTION_OUTPUT="${ISAAC_PERCEPTION_OUTPUT:-${PROJECT_ROOT}/outputs/isaac_perception_visual_demo}"
"${PROJECT_ROOT}/tools/run_isaac_perception_acceptance.sh"

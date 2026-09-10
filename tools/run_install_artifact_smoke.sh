#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
PYTHON_BIN=${PYTHON_BIN:-/usr/bin/python3.10}
ARTIFACT_ROOT=${ARTIFACT_ROOT:-${TMPDIR:-/tmp}/unloading-acceptance}
RUN_ID=${RUN_ID:-$(date -u +%Y%m%dT%H%M%SZ)}
RUN="$ARTIFACT_ROOT/install-smoke/$RUN_ID"
BUILD_VENV="$RUN/build-venv"
INSTALL_VENV="$RUN/install-venv"
DIST="$RUN/dist"
OUTSIDE="$RUN/outside-source"
mkdir -p "$DIST" "$OUTSIDE"

"$PYTHON_BIN" -m venv "$BUILD_VENV"
"$BUILD_VENV/bin/python" -m pip install \
  pip==25.2 setuptools==79.0.1 wheel==0.45.1 build==1.2.2.post1
"$BUILD_VENV/bin/python" -m build --wheel --outdir "$DIST" \
  "$ROOT/packages/unloading_contracts"
"$BUILD_VENV/bin/python" -m build --wheel --outdir "$DIST" "$ROOT"

CONTRACTS_WHEEL=$(find "$DIST" -maxdepth 1 -type f -name 'unloading_contracts-1.1.0-*.whl' -print -quit)
ROOT_WHEEL=$(find "$DIST" -maxdepth 1 -type f -name 'unloading_layer1_sim-0.5.0.dev0-*.whl' -print -quit)
test -n "$CONTRACTS_WHEEL"
test -n "$ROOT_WHEEL"

"$PYTHON_BIN" -m venv "$INSTALL_VENV"
"$INSTALL_VENV/bin/python" -m pip install pip==25.2 setuptools==79.0.1 wheel==0.45.1
"$INSTALL_VENV/bin/python" -m pip install -r "$ROOT/configs/deployment/cpu-ci-requirements.txt"
# The root wheel declares the exact shared-contract dependency.  Install that
# locally built wheel first, then install the root wheel without consulting an
# external index for an unpublished internal distribution.
"$INSTALL_VENV/bin/python" -m pip install --no-deps "$CONTRACTS_WHEEL"
"$INSTALL_VENV/bin/python" -m pip install --no-deps "$ROOT_WHEEL"
"$INSTALL_VENV/bin/python" -m pip check
(
  cd "$OUTSIDE"
  unset PYTHONPATH PYTHONHOME
  "$INSTALL_VENV/bin/python" "$ROOT/tools/verify_installed_artifacts.py" \
    --repository "$ROOT" --contracts-wheel "$CONTRACTS_WHEEL" \
    --root-wheel "$ROOT_WHEEL" --output "$RUN/summary.json"
)
sha256sum "$CONTRACTS_WHEEL" "$ROOT_WHEEL" "$RUN/summary.json" > "$RUN/sha256.txt"

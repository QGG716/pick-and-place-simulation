#!/usr/bin/env bash
set -euo pipefail

ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ARTIFACT_ROOT=${ARTIFACT_ROOT:-/root/autodl-tmp/v05-acceptance}
export PIP_INDEX_URL=${PIP_INDEX_URL:-https://pypi.org/simple}
RUN="$ARTIFACT_ROOT/cpu"
mkdir -p "$RUN"
/usr/bin/python3.10 -m venv "$RUN/venv"
"$RUN/venv/bin/python" -m pip install pip==25.2 setuptools==80.9.0 wheel==0.45.1
"$RUN/venv/bin/python" -m pip install -e "$ROOT[dev]"
"$RUN/venv/bin/python" -m pytest -q "$ROOT/tests" \
  --basetemp "$RUN/pytest-tmp" --junitxml "$RUN/pytest.xml" 2>&1 | tee "$RUN/pytest.log"
"$RUN/venv/bin/python" "$ROOT/tools/check_three_branch_contracts.py" \
  --repository "$ROOT" --python /usr/bin/python3.10 \
  --work-root "$RUN/contracts-work" --output "$RUN/three-branch-contracts.json" \
  2>&1 | tee "$RUN/three-branch-contracts.log"

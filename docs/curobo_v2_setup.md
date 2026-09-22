# cuRobo V2 TRANSIT verification: reproduce this branch

This is an optional, fail-closed geometric candidate backend. It is not an accepted
execution backend yet: see `m710_curobo_v2_20260922.md` and its raw evidence.
The CPU package has no new mandatory CUDA/Isaac/ROS dependency.

## Fixed inputs and entry points

`fixtures/curobo_v2/manifest.json` fixes the full business TRANSIT of
`carton_l07_c04`, from extraction end to the existing pre-place state. The adjacent
historical motion is an endpoint/attachment source, not an inherited success.
The actual remaining state fixes all obstacle identities and poses. Current active
policy must match, except the unused historical hint-directory locator.

Run in the repository on the authorized Linux GPU server:

```bash
export PYTHONPATH="$PWD/src"
CPU=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
GPU=/root/autodl-tmp/curobo-v2-20260922/venv/bin/python
M=fixtures/curobo_v2/source_motion.json
S=fixtures/curobo_v2/actual_remaining_state.json
$CPU tools/run_curobo_v2_transit.py --motion "$M" --state "$S" \
  --output outputs/curobo_v2/business_gpu --gpu-python "$GPU"
$CPU tools/run_curobo_v2_transit.py --motion "$M" --state "$S" \
  --output outputs/curobo_v2/business_cpu --backend baseline --warm-runs 0
# Clearly artificial positive direct connection: 1 mrad independent J3 at pre-place.
# This is NOT another full business TRANSIT or a physical trial.
$CPU tools/run_curobo_v2_transit.py --motion "$M" --state "$S" --fixture direct_unit \
  --output outputs/curobo_v2/direct_gpu --gpu-python "$GPU"
$CPU tools/run_curobo_v2_transit.py --motion "$M" --state "$S" --fixture direct_unit \
  --output outputs/curobo_v2/direct_cpu --backend baseline
$CPU tools/probe_curobo_v2_collision.py --motion "$M" --state "$S" \
  --output outputs/curobo_v2/collision_probes.json
$CPU -m pytest -q tests/test_stage_backend.py tests/test_poc_pair_clearance.py tests/test_layout_trajectory.py
```

`--prepare-only` writes the neutral request, frozen bundle and input provenance.
The GPU worker checks source hashes, model order, full inertials/limits and three
FK states before permitting `plan_cspace`. `gpu_model.json` contains the exact
885-sphere collision configuration, link-by-link fitting sources and metrics.
The worker is a real `MotionPlanner` from the pinned source, not an example wrapper.
A YAML metrics file avoids the tag's in-place dictionary mutation across factories.
The tag's URDF inertial parsing defect is corrected in parsed tensors from the
hash-verified official XML, before building the solver's IK/TrajOpt/graph clones.
No NVIDIA source file was modified.

For the existing single-carton connector, after the existing scene/connector is
built, explicitly opt in:

```python
from unloading_sim.curobo_transit import install_curobo_transit
adapter = install_curobo_transit(scene, connector, gpu_python, output_directory)
try:
    outcome = connector.plan(...)  # normal complete workflow and final checks
finally:
    adapter.close()                # drain/reap the synchronous worker
```

Leaving the adapter uninstalled retains the original baseline. Installing it
replaces only `_finish_place_branch`'s extraction-to-preplace connection. Existing
receiving pose/IK branch selection remains authoritative. No CPU TRANSIT success
is required first; a GPU failure is never converted to CPU success. A changed
model, tool, attachment, scene or policy rebuilds the worker and records generation
and reason. Identical contexts reuse it. This first version rebuilds the entire
scene instead of doing in-place capacity updates; it never truncates obstacles.
Changing literal exported geometry can conservatively trigger a rebuild.

`unloading_stage_v1` is JSON with SI units, joint names J1..J6, world-from-local
4x4 transforms and explicit wxyz conversion. Ordinary lists cross the process
boundary. Native evaluated B-spline samples are trimmed by V2's
`get_interpolated_plan()`; they are not control points. Proposed geometric edges
are interpreted as joint-linear samples by the existing authority. Native dq/ddq
are evidence; they are not a validated controller command for that interpolation.
Successful geometry must still pass complete-task checks and the existing replay
exporter's final retiming/motion checks. The adapter never sets `delivered` or
`isaac_execution_completed` to true.

Cancellation/deadline is checked before/after synchronous solves and immediately
before acceptance. An in-flight CUDA call drains; its obsolete result is discarded.
`close()` waits for worker exit. There is no immediate kernel-interruption claim
or default business deadline. Hung native-call recovery remains a limitation.

## Pinned isolated installation

Validated source: `NVlabs/curobo` tag `v0.8.0`, full commit
`4ea77366ca48ee453e7df139e39fa6532af49f3b`. The official source archive used in this
run has SHA256 `a52a2e29c681f170d735df28eb7daf2dee401135041612f3d8849ebf31911849`.
Do not install floating `main`, v0.7.x or latest-documentation API substitutions.

The validated installation lives under `/root/autodl-tmp/curobo-v2-20260922`.
It uses a new Python 3.10 virtualenv and a read-only `.pth` pointing at the existing
`/root/v05-gpu-venv/lib/python3.10/site-packages` for Torch/CUDA dependencies.
All added packages were installed into the new virtualenv. The driver and Isaac
installation were not changed. This is host-specific reuse, not a portable lock.
`docs/evidence/curobo_v2_20260922/environment-freeze.txt` records actual versions.

For a fresh installation use a separate environment, obtain Torch 2.7.1+cu128
from the official PyTorch CUDA 12.8 wheel index, and reproduce the recorded Linux
Python 3.10 dependencies. Relevant pins are Warp 1.17.0, cuda-core 0.7.0,
cuda-bindings 12.9.8, cuda-pathfinder 1.8.2, yourdfpy 0.0.60,
numpy-quaternion 2024.0.13 and setuptools-scm 8.3.1. Use setuptools >=77 for the
source's license metadata. The core import/planning/sphere fit are verified;
optional Viser viewer extras were not installed/tested completely.

```bash
git clone --branch v0.8.0 --depth 1 https://github.com/NVlabs/curobo.git curobo-pinned
test "$(git -C curobo-pinned rev-parse HEAD)" = 4ea77366ca48ee453e7df139e39fa6532af49f3b
# Create provenance only AFTER verifying the checkout. No source edits.
python - <<'PY'
import json, subprocess
from pathlib import Path
p=Path('curobo-pinned')
commit=subprocess.check_output(['git','-C',str(p),'rev-parse','HEAD'],text=True).strip()
assert commit=='4ea77366ca48ee453e7df139e39fa6532af49f3b'
(p/'PINNED_SOURCE.json').write_text(json.dumps({'tag':'v0.8.0','commit':commit}))
PY
# Inside the isolated, dependency-prepared GPU venv:
SETUPTOOLS_SCM_PRETEND_VERSION=0.8.0 python -m pip install --no-deps --no-build-isolation -e ./curobo-pinned
python -c 'import curobo, torch; from curobo.motion_planner import MotionPlanner; print(curobo.__file__, torch.__version__, torch.cuda.get_device_name())'
```

Do not copy virtualenvs, CUDA caches or credentials into Git. Source meshes remain
in the existing official repository assets; evidence contains generated spheres
and source hashes instead of duplicate third-party mesh assets.

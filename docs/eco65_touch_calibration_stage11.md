# Known-tip calibration stage 1.1: acceptance and static geometry

Stage 1.1 preserves the estimator, camera CAD/installation, FOV/resampling,
known-tip assumption and frozen workstation. It reuses stage-one images without
starting Isaac or acquiring new images. No IK/search, trajectories, physics,
force control or hardware are invoked by the recheck entry.

The original normal and perturbed observations under
`outputs/eco65_native/touch_calibration_20261016_run08/` are read-only.
New output is under `outputs/eco65_native/touch_calibration_stage11_20261016_run01/`.
Old unbound result files are intentionally rejected and must be regenerated
from their original observations, not edited to add a trusted flag.

## Result and observation contract

The new result schema is `known_tip_fixed_extrinsics_v2`. Acceptance checks the
schema, parent `link_1`, child `camera_optical`, mapping direction, metres,
rotation representation, optical axes, finite proper SE(3), eligible solved
status, configuration identifiers, kinematic definition and sample groups.
A failed or previously validated result is not accepted as a fresh solver result.

Hash scopes are explicit and never compared interchangeably:

| Field | Content covered |
|---|---|
| `observation_binding.sha256` | The entire parsed observations.json, including image paths/hashes, q, K/distortion, probe/board/kinematics, groups, thresholds and diagnostic metadata |
| `definition_fingerprints` | Separately, the exact camera, probe, board and kinematics objects exported in observations |
| `identifiers.camera` | Opaque legacy identifier of the full nominal camera preparation configuration; checked against each photo and result, not against the reduced observation camera hash |
| `identifiers.probe/board` | Full exported definitions; recomputed and compared with same-scope identities |
| `scene_sources.binding_sha256` | Versioned source records, content hashes and association/readback evidence, excluding this hash field itself |
| `derived_scene_identity` | Versioned object containing scene-source binding plus full camera, probe, board and kinematic definitions |

JSON content hashing uses sorted keys, compact separators, ensure_ascii=True,
UTF-8 and rejection of NaN/Inf. Image bytes are independently checked against
their recorded SHA256 before detection. Whitespace-only changes to the JSON
do not change the content fingerprint; changes to values do.

`STAGE1_PROFILE` defines this experiment's coverage: six distinct training
touch datums, three validation datums, non-collinear local and measured point
sets, and three distinct J1 views per board placement in both splits. The J1
separation tolerance is 1e-6 rad with periodic-angle comparison. This is an
acceptance configuration, not a claim about every calibration's theoretical
minimum. Duplicate sample IDs, repeated point records or split mixing fail.

The evaluator recomputes training registration, actual image detection/PnP and
per-observation consistency using shared estimator functions. It also compares
the reproduced estimate with the submitted matrix. Training registration,
training reprojection, multi-observation consistency, held-out point error,
held-out multi-J1 consistency and simulation-truth error have separate evidence
and statuses. Missing evidence is rejected or marked NOT_EVALUATED. One view
cannot produce a claimed zero-spread multi-J1 PASS.

Both CLIs return zero only on their documented success status. Numerical
failure, identity/coverage problems and malformed input return nonzero while
saving a readable failure report in a fresh output directory. Existing outputs
are never overwritten. The solver has no truth/scene argument. Only the
independent evaluator reads truth, and neither moves a camera.

## Static policy and source binding

General and non-adjacent robot self-pair clearance comes from the existing
`eco65.native_model.POLICY`: 2.4 mm. It is independent of the unchanged 0.1 mm
tip-arrival and registration tolerances. Existing asset-bound flange contacts,
base installation and the specifically named probe-bracket/rigid-plate interface
retain the policy's 0.4 mm signed contact tolerance. Existing connector/link_1
and conformal-seat/link_1 interfaces retain their recorded original-triangle
assembly basis; their convex envelopes are not a new interface qualification.
These pairs and the same-rigid-assembly/URDF adjacency exclusions are recorded
in `pair_catalog.json`. There is no whole-board/probe/mast/tool-environment waiver.

The designated tip/datum pair requires a finite-datum radial check, tip arrival
and the support-plane test of all cone vertices. An invalid analytic condition
always rejects, even if the distance query is positive. Finite negative EPA
contact artifacts remain recorded for that designated pair. NaN, Inf and
backend exceptions fail closed, including for that pair; the analytical check
is not a general exception handler.

The query horizon is the maximum applicable clearance/contact/arrival threshold
plus the existing mesh reserve: currently 2.6 mm. A conservative sphere lower
bound above that horizon excludes a pair. All queried pairs, excluded pair-index
ranges, thresholds and possible `distmax` truncation are recorded. Raw minima
are explicitly minima of the queried subset, not exact whole-scene clearance.
The actual backend is MuJoCo 3.8.0; model flags and CCD settings are recorded.

Preparation now requires explicit `--composed-usd` and `--geometry-manifest`.
There is no historical-directory fallback. A manifest must match both the
snapshot content hash and its fingerprint/configuration identities. The actual
USD workstation reference must match the manifest's workstation hash. The
composed stage's units, Z axis, root transform and collision points/transforms
are read back. Source files and camera asset hashes are checked before use.
Recheck creates a derivative configuration bound to these sources and replaces
its old acceptance flags with the current results. Original files remain intact.

## Reproduction without rendering

Use the existing Linux environment, with a **new** R for each invocation:

```bash
set -e
export PYTHONPATH=src:tools OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
P=/root/autodl-tmp/envs/isaacsim-clean/bin/python
OLD=outputs/eco65_native/touch_calibration_20261016_run08
R=outputs/eco65_native/touch_calibration_stage11_new
mkdir "$R"
for CASE in capture02 perturb01; do
  "$P" tools/solve_touch_calibration.py --observations "$OLD/$CASE/observations" --output "$R/$CASE/solve"
  "$P" tools/evaluate_touch_calibration.py --observations "$OLD/$CASE/observations" \
    --result "$R/$CASE/solve/extrinsics.json" --truth "$OLD/$CASE/truth/truth.json" --output "$R/$CASE/validate"
  "$P" -c 'import json,sys; assert json.load(open(sys.argv[1]))["status"] == "PASS"' "$R/$CASE/validate/validation.json"
done
# The known geometry failure returns 1; stop here rather than advancing planning.
"$P" tools/recheck_eco65_touch_geometry.py --prepared "$OLD" \
  --snapshot outputs/eco65_native/j1_mast_20261016_connected_run01/build/snapshot.json \
  --composed-usd outputs/eco65_native/j1_mast_20261016_connected_run01/isaac_A/eco65_desktop_warehouse.usda \
  --geometry-manifest outputs/eco65_native/j1_mast_20261016_connected_run01/build/usd_A/workstation_manifest.json \
  --output "$R/geometry"
"$P" -c 'import json,sys; assert json.load(open(sys.argv[1]))["status"] == "PASS"' "$R/geometry/geometry_report.json"
```

Run the independent small test suite separately:
`"$P" -m unittest discover -s tests -p 'test_touch_calibration*.py' -v`.
The private run records expected nonzero diagnostic exits explicitly; they do
not authorize any later stage.

## Measured outcome and next-stage gate

The original eight tests plus fifteen new targeted regressions pass in the
existing Linux environment. They distinguish pure math from mocked acceptance,
distance-error and CLI control tests. A small real USD reference test exercises
source association. These tests do not substitute for the following real work:

* Actual original RGB re-detection and FK-based touch recomputation: both normal
  and perturbation batches pass all independent estimator checks. New and old
  estimated matrices and error values are identical (maximum difference zero).
  Normal: 1.440 mm / 0.1431°, held-out max 0.799 mm. Perturbed: 0.621 mm /
  0.0550°, held-out max 0.529 mm.
* Actual composed USD readback: 86 collision records; transform difference zero,
  maximum mesh-vertex difference 1.49e-8 m (USD float representation).
* Actual MuJoCo recheck: nine touch and six photo states all fail the same pair,
  tool `part_001` versus `probe_bracket`. Its queried distance is approximately
  0.999999 mm; previous threshold 0.1 mm, current threshold 2.4 mm. This is a
  clearance deficit of approximately 1.4 mm, not a reported penetration.

The estimator quality is accepted; the existing assembly/states are **not**
qualified endpoints for the next continuous single-point touch planning stage.
The named bracket/tool clearance must be resolved and rechecked under the same
policy in a separately authorized follow-up. This round does not redesign the
bracket, add an exception, lower clearance or search for replacement poses.
Even a future static pass would not validate the connecting path.

`NO_MOTION_EXECUTED` / `PHYSICAL_CONTACT_NOT_EVALUATED` / `Dynamics: NOT_EVALUATED`.


## 2026-10-17 follow-up

The explicit 30? installation, supported board, bracket repair and fresh calibration
are recorded in [eco65_tilted_camera_touch.md](eco65_tilted_camera_touch.md).
The measurements and failures above remain historical evidence for the older installation.

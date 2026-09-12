# Upstream observed-face V4 provenance and RGB-D adaptation

This note freezes the upstream reference used by the perception branch.  It
does not treat a preview image as an algorithm certificate.

## Frozen source and located artifacts

- Repository: `https://github.com/QGG716/cargo-detection-and-6D-estimation`
- Branch requested by the user: `dev`
- Resolved commit: `1d208f2ed380a207e6e46b4a62d2ac640edfe477`
- Source image recorded by the result: `images/厢内货物_8.png`, 1290 x 1188
- Final preview: `runs/experiments/15_batch_instance_aware/batch_2_8/cargo_8/07_final/final_observed_faces_v4_candidate.jpg`
- Final JSON: `runs/experiments/15_batch_instance_aware/batch_2_8/cargo_8/07_final/final_observed_faces_v4_candidate.json`
- Geometry preview/JSON: `06_geometry_3d/box_cuboids_observed_faces_v4_candidate.{jpg,json}` under the same cargo directory
- SAM instances: `04_sam_instances/cargo_masks_pre_held_v5.npz`
- Unanchored input: `06_geometry_3d/box_cuboids_unanchored_baseline.json`
- V4 implementation: `pipeline/geometry/refine_multiplane_cuboids.py`

SHA-256 checksums of the tracked evidence are:

| artifact | SHA-256 |
|---|---|
| final JPG | `40b3954a412605f4d2f1b2d36f89044ab5cb6f394d486ddeb9ba2f430792b1de` |
| final JSON | `9d801a1a0a5411754954bb3bef126cc78de885d92d137f4d512230f53d2f56e0` |
| geometry JPG | `204cc65b7d0a90c46c9580aba6f414b9342e9c5fc5b4e9d23b39cc76b95ffe85` |
| geometry JSON | `7df8b7892daef382a8dc03225144266083ba83c79f36978b1160302ff69e764b` |
| unanchored JSON | `039c85332b75fcc4d42fe8e87631d4869d2fec97268c1f166d248dffde4c1428` |
| SAM NPZ | `0bf04a86973443f5242849b3f92176e2c1bcd69465f16c475e49331932366651` |

The exact historical shell command is not tracked.  The JSON refers to
`06_geometry_3d/moge2_pointmap.npz`, but that large point map is absent from
the Git checkout.  Therefore an exact pixel-for-pixel rerun of the historical
Cargo_8 artifact cannot honestly be claimed from Git alone.  The model entry
point defaults to `Ruicheng/moge-2-vits-normal`, CUDA, 1800 tokens; the result
does not record whether these defaults were changed.  No evidence of manual
corner edits is recorded for V4, but generated annotations and deterministic
2D face proposals are upstream inputs.

The command below is reconstructed from the checked-in CLI, not recovered from
a run log:

```text
python pipeline/geometry/refine_multiplane_cuboids.py SOURCE MASKS POINTMAP UNANCHORED_BASELINE \
  --fallback FACE_ANCHORED_RESULT --suppress-hidden-fallback \
  --min-face-precision 0.72 --min-face-coverage 0.04 --max-pnp-rmse 6 \
  --json-output RESULT.json --output RESULT.jpg
```

## Stage comparison

| stage | upstream V4 | project before this adapter | now executed for registered RGB-D | RGB-D-specific rule |
|---|---|---|---|---|
| proposal + SAM | yes | yes | yes | each module has its own image/mask identity |
| multiple depth-plane candidates | yes | base recovery only | yes | metric registered depth replaces MoGe points |
| per-face precision/coverage gate | yes | produced but V4 not called | yes | same 0.72 / 0.04 gates initially retained |
| shared-edge/corner topology | yes | implicit in base cuboid | yes | one corner graph per instance |
| union-to-SAM ECC registration | yes | no | yes | may propose 2D correspondence; cannot override metric plane residual |
| fixed-intrinsics PnP | yes | no | yes | capture K is immutable |
| anisotropic size scale | 0.65--1.50 | no V4 | bounded worker step | metric-depth acceptance must additionally reject movement off observed planes |
| hidden-face suppression | yes | not consistently | yes | hidden/completed faces are not observations |
| observed-face contract | JSON face list | conflated with complete cuboid | explicit adapter provenance | published as `ObservedFaceSet` in the next geometry stage |
| complete cuboid | still present in record | always adapted | qualified separately | a single face without a sourced size prior is insufficient |

## What is reused and what is not

The main project invokes the actual pinned V4 script through a thin adapter;
the OpenCV/SciPy implementation is not copied.  The adapter temporarily maps
`registered_metric_depth_plane` to the historical selector literal
`monocular_depth_plane`, then restores truthful registered-depth evidence in
the output.  It also requires `--suppress-hidden-fallback`.

The monocular assumption that moderate 3D rescaling may be used to absorb 2D
silhouette mismatch cannot be accepted unchanged.  In registered RGB-D, final
acceptance must jointly check point-to-plane residual, observed boundary
reprojection, orthogonality/shared vertices, any sourced size prior, and free
space.  K and capture-time extrinsics are never optimization variables.

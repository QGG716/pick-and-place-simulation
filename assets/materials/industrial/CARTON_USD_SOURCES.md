# Complete carton assets

The sensor-only adapter references complete meshes, their original UVs and material graphs. Raw USD, MDL, textures and the scan archive stay in an external cache, outside Git. `configs/isaac/carton_assets.json` pins every dependency by SHA-256; `tools/fetch_carton_assets.py --config` reconstructs that cache.

| Asset | Source / terms | Use |
|---|---|---|
| `SM_CardBoxD_04.usd` | NVIDIA Isaac Assets **5.0**, `Isaac/Environments/Simple_Warehouse/Props`; NVIDIA proprietary content, **not** the Apache-2.0 license of the example Python code | 38 original box entities; original bevel mesh, UV, albedo, normal and ORM maps, MDL graph |
| `BOX_30X17X22.usdc` | [1057822006-svg, release v0.1](https://github.com/1057822006-svg/usd-cardboard-box-assets/releases/tag/v0.1), release commit `cb386b4cba6c6adf2d42b54be37a78f9d4f01589` | 2 original entities, `carton_l02_c02` and `carton_l07_c04`; photogrammetry cardboard, tape and label meshes |

Scan attribution: **1057822006-svg, Free USD Cardboard Box Assets for Robotics Simulation, v0.1**. The author's [README](https://github.com/1057822006-svg/usd-cardboard-box-assets#license) permits use and modification and requests attribution on redistribution; it describes CC BY 4.0 as the *recommended* license. We retain that wording rather than claiming a stronger license grant, and do not redistribute the raw model package. The downloadable archive SHA-256 is `b7931855ff2fa37c746182780bb7f126b64c156f1fe2bb431cfec1b632962001`.

NVIDIA references: [static warehouse assets](https://docs.isaacsim.omniverse.nvidia.com/latest/assets/asset_utilities/tutorial_static_assets.html), [official example using this standalone box](https://github.com/isaac-sim/IsaacSim/blob/main/source/standalone_examples/replicator/scene_based_sdg/scene_based_sdg.py), [Omniverse content/license terms](https://docs.omniverse.nvidia.com/dev-overview/latest/common/NVIDIA_Omniverse_License_Agreement.html), and [Isaac additional software/material terms](https://docs.isaacsim.omniverse.nvidia.com/latest/common/license-isaac-sim-additional.html). Model use and redistribution must follow the applicable NVIDIA content terms; this repository does not relicense those files.

The documented WarehousePile_A04 was found at the public bucket's actual `Assets/ArchVis/Industrial/Piles/WarehousePile_A4.usd` path. Its separate `Cardbox_A1/A2/A3/B1/B2/C2` references were inspected. Their squarer proportions are less suitable for the frozen 0.6 × 0.4 × 0.3 m boxes, so the independent official D04 asset is used. No pile, pallet or background was imported. Scan 01 is visibly damaged; scan 04's preview/shape is unsuitable for the regular stack. Neither is included. Poly Haven's optional API was unavailable during inspection; no Poly Haven model is used.

Adaptations are authored only below each original entity's `Visual` node:

- D04: Z-up, metadata metres; its mesh already contains a 0.01 scale. The composed envelope is approximately 0.38 × 0.25 × 0.14875 m. Fit once to the original nominal envelope, with maximum/minimum directional scale ratio 1.278. Preserve original points, bevels, normals, UV and materials.
- Scan 05: Z-up, metres. The scanned body axes differ from the source coordinate axes by a few degrees. Align the visual adapter using area-weighted main cardboard triangle normals (polar orthogonalization), then exchange its short axes with a local X rotation. The resulting fixed XYZ angles are recorded in the configuration. Fit the aligned visible envelope to the same nominal envelope; cardboard establishes the main body bounds and tape differs only locally. No global clearance or box-position change.
- Remove inherited physics/semantic APIs and block source physics attributes; invisible scan proxies stay invisible and have no active collider. Each entity retains the original static nominal conservative collision cube, hidden from RGB/depth.
- No new texture painting, added edge strips, emissive target, geometry jitter or material override on the entity parent. Normal-map shading is not labelled as measured depth displacement.

No upstream example code was copied; the project's adapter and downloader are authored here. Capture configuration, source hashes and rendered instance aggregation are delivered with the new images. Nominal corners/planes remain a cuboid approximation; exact detailed mesh surface corners/planes are not evaluated.

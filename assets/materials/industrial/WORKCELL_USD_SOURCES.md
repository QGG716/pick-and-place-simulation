# Static workcell visual assets (2026-09-16)

This adaptation belongs only to `feat/v0.5-perception-ros2`. It does not certify
feasibility, motion, conveyor transport or manufacturing CAD.

## Official conveyors

Source: NVIDIA Isaac Assets **5.0**, `Isaac/Props/Conveyors`:

- [ConveyorBelt_A06.usd](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Props/Conveyors/ConveyorBelt_A06.usd)
- [ConveyorBelt_A43.usd](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Props/Conveyors/ConveyorBelt_A43.usd)
- [NVIDIA conveyor documentation](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/digital_twin/warehouse_logistics/ext_isaacsim_asset_gen_conveyor.html)

The pinned file hashes and dependencies are in `configs/isaac/conveyor_assets.json`.
Original files and material dependencies stay in the server cache. No upstream
mesh, texture library or model weights are redistributed in Git. NVIDIA content
licensing applies; Isaac example-code licensing is not an asset license.

A06 is a BELT/STRAIGHT at level 1 in the installed official `track_types.json`.
Its source frame reaches about 2.311 m; the belt is approximately 2.000 × 0.900 m,
with a 0.0390 m thickness and original top Z about 1.78053 m. We reuse the actual
`/World/Belt/SM_ConveyorBelt_A06_Belt_02` mesh, UVs and material binding. The working
surface alone is resized in its horizontal directions, mounted at world Z=0.60,
and split into one 1.50 m transverse segment and two 1.40 m longitudinal segments.
Its thickness is preserved. Source tall frame/decals are explicitly excluded;
the project low frame and original full collision envelope remain. This is a
belt-subassembly adaptation, not a scaled complete conveyor machine.

A43 is **ROLLER/Y_MERGE**, with 199 meshes, a main endpoint at approximately
[4, 0, 0] and a second endpoint at [3.28299, -1.85157, 0], whose heading is -45°.
The main body envelope is approximately 4.0001 × 2.8307 × 1.1663 m. Even a width
adaptation from the roughly 0.90 m working track to 0.70 m cannot turn the 45°
branch into the required 90° L connection or fit its unaltered branch topology
into the 2.80 × 0.70 m longitudinal envelope. **A43 is inspected and rejected**;
it is not instantiated in the final workcell. There is no claimed 90° turntable
or moving transfer function. The junction belongs to the stationary longitudinal
belt, with the original rectangles touching at Y=-0.40.

## Trailer interior

**Official trailer replacement is not completed.** A bounded search of official
documentation, the NVIDIA ArchVis Industrial/Containers listing, and Isaac 5.0
Simple_Warehouse/Props found no verified trailer interior suitable for this
adaptation. This does not establish that no such asset exists anywhere.
The [official downloadable packs catalogue](https://docs.omniverse.nvidia.com/usd/latest/usd_content_samples/downloadable_packs.html)
describes its Containers & Shipping packs as boxes/bins/cases/totes and related
warehouse props; the name alone is not evidence of a usable trailer interior.

The final source is an explicitly **engineering-parametric interior** from
`configs/isaac/workcell_v2.yaml`: clear dimensions 9.60 × 2.30 × 2.70 m, local
section X=[-3.80, 4.00] m, 0.06 m outward thickness. Door world X is undefined.
There is no end wall or decorative doorframe at either local clipping edge.
Floor, both walls and roof share visible geometry with their static box collision
geometry. The roof remains present for every sensor and observer camera.

## Identity and provenance

The archive under `integration/isaac_scene_contract/m710id70_layout_v1` is an
immutable parent. `tools/build_isaac_perception_bundle.py` derives new snapshot
and contract files, binding effective config, asset locks, the current official
robot base and rig config before hashing. Capture consumes the derived contract
named by the bundle index. The robot import cache is still keyed to the unchanged
URDF, while scene geometry and sensor calibration have new identities.
`configs/trucks/2p3x2p7_m710id70.yaml` is the separate historical cross-section
study configuration; its X=1.00 test target is not the capture world's origin or
a door datum. It does not override this effective perception scene.

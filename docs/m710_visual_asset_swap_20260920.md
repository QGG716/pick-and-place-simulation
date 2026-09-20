# Recorded-state visual asset upgrade (2026-09-20)

This change reconstructs the five existing successful clips from three physical
worlds. It does not execute physics, repeat IK/RRT, or establish a same-world 5/5
result. `visual_asset_swap=true`, `new_physics_execution=false`, and
`same_world_continuous_trial=false` remain explicit in the output manifest.

## Sources and boundaries

Fetch verified the video baseline `1dd8e0485fca294e625ca23e398a1ddef6124eac`
and perception source `cc64bbd49e4e113f44ee0750b6981a24e452ee1b`.
Only the carton/conveyor manifests and their visual adapters were selected from
the perception source. No branch merge, ROS, planning, or controller code was
imported. Work and GPU outputs used separate directories; historical evidence
and videos remain intact.

* Cartons: NVIDIA Isaac Assets 5.0 `SM_CardBoxD_04.usd`, with the original
  `scan05` overrides for `carton_l02_c02` and `carton_l07_c04`. Original USD
  meshes, UVs, materials, dependency hashes, scan alignment and 1.35 aspect
  limit are retained. Normalization produces unit entity bounds: the recorded
  parent matrix applies actual size exactly once.
* Belts: NVIDIA Isaac Assets 5.0 `ConveyorBelt_A06.usd`, subtree
  `/World/Belt/SM_ConveyorBelt_A06_Belt_02`. One transverse and two longitudinal
  segments fit the existing envelopes. The approximately 39 mm belt thickness
  and project low supports are retained; the tall source frame and A43 are
  excluded.
* Chassis: the installed asset-root API resolved NVIDIA Isaac Assets 6.0;
  `Robots/Idealworks/iwhub/iw_hub_static.usd` was actually stat-checked and
  opened. Only `/iw_hub` is referenced (excluding its external ground).
  Uniform scale 1.4676568565 gives a 2.1 x 0.96738 x 0.33933 m appearance inside
  the existing 2.1 x 1.5 x 0.6 m physical envelope. A stationary mounting
  connector preserves the original robot interface. Wheels stay round and
  stationary; no mobile-base capability is claimed.

Exact URLs, SHA-256 hashes, licensing/attribution and dependency paths live in
`configs/isaac/`. NVIDIA and scan asset bytes are cached externally, not vendored.
UDIM templates are resolved against their authoring layer and every tile checked.

## Integration and motion

`m710_visual_assets.apply_visual_assets` is shared by the optional live scene
initialization hook (`--visual-asset-cache`) and the recorded-state renderer
(`--visual-asset-config` plus `--visual-asset-cache`).
`cache_m710_visual_assets.py` prepares/verifies the external caches using the
installed Isaac environment. The core package has no new heavyweight dependency.

Only new visual subtrees are stripped of source physics, control, sensors and
lights. Original physical Cube prims stay present. Their old rendered surfaces
use an alpha-cutout material, with weaker binding than descendant asset materials;
merely setting opacity to zero caused reflected proxy surfaces in RTX. Old
decorations are hidden without hiding the new assets or deleting collision.

With the physics timeline stopped, each rendered source timestamp explicitly
updates per-mesh UVs. Distance integrates recorded belt speed, start/stop times
and active-surface history. Reconstructed final phases must match the recorded
accumulators within 1 micrometre. Phase carries between clips of one world and
resets for a new world; offline planning delays do not contribute.

The adapted face UV gradient maps world displacement to UV displacement.
Transverse motion is -Y and longitudinal motion is -X. Per-segment primvars
avoid modifying shared shaders. Small asymmetric splice paint on the moving
skin makes travel visible at the original 5 Hz sampling. Frames, support,
collision surfaces and chassis remain stationary. Each stopped-time pose/UV
update explicitly resets DLSS history, then receives eight render-only settling
subframes; motion blur is disabled. Increasing settling alone did not clear the
stopped-time history. The reset removed the visible trails while preserving the
original camera, lighting and antialiasing mode.

## Validation and delivery

Five directed GPU-server tests cover unit-parent normalization, aspect rejection,
stop/resume, separate belt phases, world continuity, and rejection of a 16x belt
speed error. USD runtime checks compare all original prim attributes/schemas and
physical relationships before/after the swap, except allowed appearance fields.
They also validate mesh/material dependencies and exact world direction, stopped
UV hold and cross-belt independence. Existing q/FK and carton/link pose checks
remain active on every recorded frame; original state/result/event/remaining-state
files are hash-checked before and after rendering.

The final native 1280 x 720 movie contains 3113 source frames at 80 fps: source
5 fps divided by playback time gives 16x and exactly 38.9125 seconds. There is no
old-video pixel input, physical resimulation or extra bottom strip. The compact
bottom-left HUD retains target, phase, attachment, cup count and world identity.
The separate 1x belt check holds the robot still and labels its two source-time
windows; it is visual evidence only. Full media, original manifests, keyframes,
offline index and byte/hash verification are delivered outside Git under the
local `outputs/delivery/<timestamp>_assets_720p_16x/` directory.

Final output is in the local workspace at
`outputs/delivery/20260920_164253_assets_720p_16x/` (`index.html` is offline).
The 29,128,280-byte main video has SHA-256
`78113075267b62be94110aff47f29adfb2215f7f924c97726569d48193e79a6d`.
All 49 transferred files matched server bytes/SHA; local FFmpeg decoded all
3113 main frames, all 42 review frames and all 35 pictures. All 42 relative
index references resolve. FK/TCP maximum position error was 1.571687e-6 m;
carton/link USD position errors were zero. All 503 original prims passed
invariance checks and all 40 carton mappings passed. No missing-material pink
pixels were detected across the final movie.

See [compact evidence](validation/evidence/visual-assets-20260920/checks.json)
and the adjacent four screenshots for the actual results. Failed visual previews
and the first render with temporal trails remain isolated on the server; the
delivered version uses the verified explicit history reset. No physical trial
was executed by any of these previews or renders.

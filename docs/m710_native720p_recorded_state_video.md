# Native 720P recorded-state video

The user requested a newly rendered 720P, 16× video instead of editing or
upscaling the previous MP4. `scripts/render_m710_recorded_states.py` reads the
five archived `actual_frame_states.json` files. It never decodes the old MP4.
Each state file is checked against the original delivery manifest and world ID.

The renderer reuses the historical adapter's scene construction up to, but not
including, World creation. Its original bundle, implementation, asset and cached
USD integrity checks still run. It changes only the presentation output size to
1280×720, without rewriting the historical bundle or any of its fingerprints.
The original official FANUC geometry, Wantai tool, layout, materials, lighting
and camera are reused. USD pose changes are made in an in-memory session layer;
the original scene and recordings remain intact.

Robot links are reconstructed from measured joint angles and the official URDF.
The gripper body uses its recorded actual TCP pose and the exact original TCP
offset, including tool length. Every visible carton uses its recorded position,
orientation and dimensions. Cartons absent from a source world's state are
hidden. Per-clip commanded cup masks are restored from that clip's result.
This is measured-state visualization, not a new dynamic qualification trial.
The timeline stays stopped; render calls use zero elapsed physics time.
Conveyor boxes follow their saved poses; decorative belt/roller texture phases
are held static because those phases were not included in the archived frames.

The output is native 1280×720 with the compact parameter panel at the bottom
left, showing target, phase, source simulation time, attachment/cup state,
clip/world provenance and ideal reception/outfeed. At 80 fps, the 3113 recorded
samples play at 16× their original 5 fps cadence: 38.9125 seconds, without image
upscaling or interpolated motion samples. Historical success counts remain
separate for the three source worlds.

Validation checks scene transforms against every saved carton/link pose,
compares reconstructed TCP against measured TCP (tolerance 0.5 mm), verifies
native RGB dimensions before encoding, and decodes the full output to confirm
frame count, dimensions and rate. Five midpoint previews are inspected before
the full render. These are presentation checks, not additional physical trials.

The first preview attempt exposed a missing 0.25 m tool-length term in the
presentation TCP transform. The renderer stopped before writing any frames.
The term was restored to match the original telemetry definition, and the
subsequent five-frame preview passed with a maximum TCP position difference of
0.000001212 m. The physical records and their qualification were never changed.

The full render completed in 180.76 seconds: 3113 decoded frames, 1280×720,
80 fps, 38.9125 seconds. Maximum FK-to-recorded TCP translation difference was
0.000001572 m; all authored carton/link translation readbacks matched exactly.
The MP4 is 13,827,806 bytes. Local delivery is
`outputs/delivery/20260920_native720p_16x/`, including the player, poster and
per-file checksum manifest. Evidence is under `docs/evidence/m710_native720p/`.

# September 20 video presentation update

Future runtime recordings place the compact parameter panel at the bottom left.
It contains target, phase and simulation time, cup/attachment state, and the
existing reception/outfeed counts. The footer distinguishes physical reception
from ideal reception/outfeed. Detailed planning, height, conveyor and collision
diagnostics remain in execution evidence. This preference is also recorded in
`AGENTS.md`; initialization video labels use the bottom-left location as well.

The five successful clips retain their original three-world provenance. The new
`five_successful_pick_place_3worlds_16x_720p.mp4` is a presentation derivative of
the verified montage, using all 3113 source frames at 80 fps (16 × 5 fps), for
38.9125 seconds. The 640×416 montage is scaled proportionally to 720 pixels high
and centered in a 1280×720 frame with side bars. This is an upscale, not newly
captured 720P detail. The footer now says `16x speed` instead of `original speed`.
The historical upper-left HUD is burned into the source and remains unchanged;
the bottom-left policy applies to new recordings. No physical simulation was
rerun, and no intermediate frames or scene content were synthesized.

GPU-server checks compile the four changed Python files and render the actual
runtime HUD function at 640×360, 1280×720 and 1920×1080. Pixel bounds verify its
bottom-left position and that the upper half of the image remains untouched.
The derivative builder verifies the source hash, decodes every output frame and
checks dimensions, frame count and frame rate. The local delivery includes a
playback page, poster and SHA-256 manifest.

Local delivery: `outputs/delivery/20260920_five_successful_picks_16x_720p/`.
Validation and derivative manifest: `docs/evidence/m710_video_presentation/`.

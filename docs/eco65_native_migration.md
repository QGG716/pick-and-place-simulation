# ECO65-B native migration

Base: `3a84888f85fd490b538e5ce643aa135c2e8cabdb`.
Asset/scene source: `c07fb73b0caaca9965d8c5b3f09ede18dfb703d4`.
Branch: `feat/v0.6-eco65-moveit2-native-single-carton`.

Both remote heads were fetched and matched these pins on 2026-10-13.
The original ECO65 worktree contained only `.git` and 514 existing deletions;
it was preserved. Development uses the separate sibling worktree
`pick-and-place-simulation-v0.6-eco65-moveit2-native-single-carton`.
The sibling FANUC worktree's newer local HEAD was not used as the base.

The explicit migrated-file list is in
`configs/integration/eco65_native_sources.lock.json`. No merge/cherry-pick or
whole-directory replacement was used. FANUC implementation, configurations,
entry points and historical results remain. ECO65 `compact_task.py` and
`pipeline.py` motion generators were not imported. Source geometry helpers are
for scene construction and independent review, never native search callbacks.

Private asset ignore rules were installed before restoration. Official restore
uses the fixed manifest URLs/hashes and now writes a local restore receipt
without changing the source manifest. No vendor software or hardware is run.

Initial blockers: private tool CAD, derived meshes and `tool.json` were missing
from the original workspace. Recorded SSH endpoints refused the connection.
These are not successful asset, planning, geometry or replay results.
Dynamics and hardware remain `NOT_EVALUATED`.

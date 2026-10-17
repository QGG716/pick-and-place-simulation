# Diagnostic source provenance

Only trace-enabled requests use the project-local planner plugin. No system library is modified.
The growTree and solve bodies are adapted from OMPL 1.7.0, preserving the BSD license and algorithm statements.
The manager delegates to the installed MoveIt OMPLInterface, from MoveIt 2.5.10.

- https://github.com/ompl/ompl/blob/1.7.0/src/ompl/geometric/planners/rrt/src/RRTConnect.cpp
- https://github.com/moveit/moveit2/blob/2.5.10/moveit_planners/ompl/ompl_interface/src/ompl_planner_manager.cpp

Upstream RRTConnect.cpp SHA256: `1817e39ce3c20ae924a8c4d5476d2353083ef1c1cbfda58df9bfaeb83d604241`.
Accepted vertices/edges come from the inherited, unmodified getPlannerData(). Rejected extensions are recorded at the original checkMotion result. No extra collision queries or RNG calls are made. Snapshot occurs before context clear. Failed validity calls with no same-call custom witness remain UNKNOWN; cached/ordinary MoveIt failures are not guessed from stale last_failure.

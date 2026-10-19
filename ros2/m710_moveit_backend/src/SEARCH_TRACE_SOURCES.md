# Diagnostic source provenance

Trace-enabled sessions and ECO65 chain sessions with directed J1 use the project-local planner plugin. No system library is modified. With recording and direction both disabled, it delegates to the installed RRTConnect implementation. Direction alone does not collect a trace.
The growTree and solve bodies are adapted from OMPL 1.7.0, preserving the BSD license and algorithm statements.
The manager delegates to the installed MoveIt OMPLInterface, from MoveIt 2.5.10.

- https://github.com/ompl/ompl/blob/1.7.0/src/ompl/geometric/planners/rrt/src/RRTConnect.cpp
- https://github.com/moveit/moveit2/blob/2.5.10/moveit_planners/ompl/ompl_interface/src/ompl_planner_manager.cpp

Upstream RRTConnect.cpp SHA256: `1817e39ce3c20ae924a8c4d5476d2353083ef1c1cbfda58df9bfaeb83d604241`.
Accepted vertices/edges come from the inherited, unmodified getPlannerData(). Rejected extensions are recorded at the original checkMotion result. No extra collision queries or RNG calls are made. Snapshot occurs before context clear. Failed validity calls with no same-call custom witness remain UNKNOWN; cached/ordinary MoveIt failures are not guessed from stale last_failure.

The ECO65 adaptation checks raw J1 in execution order before each extension's existing motion check: parent to new for START, new to parent for GOAL. It does not alter samples or joint values. Recording caps affect stored events/samples only. Callback and PlannerData snapshot times are measured separately; they exclude worker serialization, RPC and viewer export. Wall-clock-limited solves can naturally differ when recording consumes time; deterministic directed tests compare full check sequences and returned paths without hitting the deadline.

Directed mode also propagates the existing solver termination condition into the CONNECT continuation loop. The cold02 run exceeded a 90 s solver allocation and lost its reply at the RPC watchdog; the upstream loop has no termination check inside CONNECT. This closes that loop's termination gap without replacing collision checks. It applies equally with recording off, on and capped; undirected upstream termination behavior and nonblocking additional-goal sampling are unchanged. A slow-validity, small-range directed test exercises the 20 ms solve limit. The real-request retest diag03 still hit its 240 s RPC watchdog and returned no trace: the real overrun remains unresolved, and this test does not establish its cause. The change does not preempt a single in-progress native collision query.

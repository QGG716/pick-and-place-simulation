"""Pure recorded-time integration; no physics, wall-clock or video-speed input."""
from __future__ import annotations


def distance_at(result, belt, time_s):
    speed = float(result['conveyor_speed_command_m_s'])
    history = result['conveyor_active_surface_history']
    assert speed >= 0 and history and history[0]['time_s'] == 0
    start = result['conveyor_started_time_s']
    stop = result['conveyor_stopped_time_s']
    if start is None:
        return 0.0
    end = min(float(time_s), float(stop) if stop is not None else float(time_s))
    total = 0.0
    for i, event in enumerate(history):
        left = max(float(event['time_s']), float(start))
        right = min(end, float(history[i+1]['time_s']) if i+1 < len(history) else end)
        if belt in event['active_surfaces'] and right > left:
            total += (right-left)*speed
    return total


def build_schedules(inputs):
    previous = {}
    schedules = []
    for data in inputs:
        result, world = data['result'], data['clip']['world_session_id']
        end = result['physics_steps']/result['physics_hz']
        final = result['conveyor_visual_motion']['independent_phase_accumulators_m']
        initial = previous.get(world, {name: 0.0 for name in final})
        errors = {name: initial[name]+distance_at(result,name,end)-value for name,value in final.items()}
        assert max(abs(e) for e in errors.values()) < 1e-6, errors
        schedules.append(dict(initial_phase_m=initial.copy(), result=result, duration_s=end,
            final_phase_m=final, reconstruction_error_m=errors,
            basis='recorded speed/start/stop/active-surface history; verified against recorded final accumulators'))
        previous[world] = final.copy()
    return schedules

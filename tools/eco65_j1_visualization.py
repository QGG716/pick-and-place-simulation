"""Finite J1-only visualization math; no IK, collision acceptance or controller."""
import math
import numpy as np
from scipy.optimize import brentq


def face_angle(robot, q_fixed, T_link_camera, target_D, limits):
    """Solve horizontal alignment including the optical centre's J1 orbit."""
    def evaluate(angle):
        q = np.array(q_fixed, float); q[0] = angle
        C = robot.named_link_frames(q)['link_1'] @ T_link_camera
        heading = C[:2, 2]; delta = np.asarray(target_D)[:2] - C[:2, 3]
        return float(heading[0]*delta[1]-heading[1]*delta[0]), float(heading @ delta)
    grid = np.linspace(*limits, 73)
    candidates = []
    for a, b in zip(grid[:-1], grid[1:]):
        if evaluate(a)[0]*evaluate(b)[0] > 0:
            continue
        angle = brentq(lambda x: evaluate(x)[0], a, b, xtol=1e-13)
        if evaluate(angle)[1] > 0 and all(abs(angle-x)>1e-8 for x in candidates):
            candidates.append(angle)
    if not candidates:
        raise ValueError('NO_FORWARD_FACING_J1_WITHIN_LIMITS')
    face = min(candidates, key=lambda x: abs(x-q_fixed[0]))
    away = next((face+d for d in [math.pi/2, -math.pi/2] if limits[0] <= face+d <= limits[1]), None)
    if away is None:
        raise ValueError('NO_90_DEGREE_AWAY_ENDPOINT_WITHIN_LIMITS')
    return face, away, dict(method='73-point 1D bracketing plus scalar root; official FK optical orbit',
                           candidates_rad=candidates, cross_residual_m=evaluate(face)[0],
                           forward_distance_m=evaluate(face)[1])


def schedule(face, away, *, fps=5, hold=2., turn=6., velocity=3.14):
    if min(fps, hold, turn, velocity) <= 0:
        raise ValueError('POSITIVE_REPLAY_PARAMETERS_REQUIRED')
    turn = math.ceil(max(turn, 1.875*abs(away-face)/velocity)*fps)/fps
    hold = math.ceil(hold*fps)/fps
    segments = [('face_1',hold,face,face), ('turn_1',turn,face,away),
                ('away_1',hold,away,away), ('return',turn,away,face),
                ('face_2',hold,face,face), ('turn_2',turn,face,away),
                ('away_2',hold,away,away)]
    rows=[]; stages=[]; start=0.
    for name,duration,a,b in segments:
        stages.append(dict(phase=name,start_s=start,end_s=start+duration,start_q1=a,end_q1=b))
        for j in range(round(duration*fps)):
            u=j/(duration*fps); blend=10*u**3-15*u**4+6*u**5
            rows.append(dict(frame=len(rows),replay_time_s=start+j/fps,phase=name,q1=a+(b-a)*blend))
        start += duration
    return rows, dict(fps=fps,duration_s=start,frames=len(rows),hold_s=hold,turn_s=turn,
                      peak_velocity_rad_s=1.875*abs(away-face)/turn,
                      velocity_limit_rad_s=velocity,segments=stages,
                      time_semantics='frame i represents replay time i/fps; physical timeline stopped')

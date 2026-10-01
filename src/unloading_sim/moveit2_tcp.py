"""Audit the executor's joint-edge geometry against a task TCP LIN contract.

Sampling uses the existing conservative clearance edge grid (including doubled
midpoint coverage), not a claim of continuous swept-volume certification.
The stopped C2 executor traverses the same straight joint edges monotonically.
"""
import numpy as np
from .geometry import rotation_vector_from_matrix, rotation_matrix_from_rotation_vector


def audit_linear_tcp(path, fk, origin, destination, *, position_tolerance,
                     orientation_tolerance, edge_resolution_rad):
    path=np.asarray(path,dtype=float);origin=np.asarray(origin);destination=np.asarray(destination)
    if path.ndim!=2 or len(path)<2 or not np.isfinite(path).all():
        return dict(passed=False,reason='INVALID_PATH')
    delta=destination[:3,3]-origin[:3,3];length=float(np.linalg.norm(delta))
    rotation=rotation_vector_from_matrix(destination[:3,:3]@origin[:3,:3].T)
    angle=float(np.linalg.norm(rotation));axis=rotation/angle if angle>1e-12 else np.zeros(3)
    maximum_position=maximum_orientation=maximum_progress_difference=0.
    previous=0.;samples=0;failure=None
    progress_tolerance=position_tolerance/length if length>1e-12 else (
        orientation_tolerance/angle if angle>1e-12 else 0.)
    def endpoint(q,expected):
        actual=fk(q)
        return dict(position_m=float(np.linalg.norm(actual[:3,3]-expected[:3,3])),
            orientation_rad=float(np.linalg.norm(rotation_vector_from_matrix(actual[:3,:3]@expected[:3,:3].T))))
    ends=[endpoint(path[0],origin),endpoint(path[-1],destination)]
    for edge,(a,b) in enumerate(zip(path[:-1],path[1:])):
        dq=b-a
        count=2*max(1,int(np.ceil(np.max(np.abs(dq))/edge_resolution_rad)),
            int(np.ceil(4*np.sum(np.abs(dq))/.0025)))
        for k in range(count+1):
            if edge and k==0: continue
            fraction=k/count;actual=fk(a+fraction*dq);samples+=1
            rv=rotation_vector_from_matrix(actual[:3,:3]@origin[:3,:3].T)
            rotation_progress=float(rv@axis/angle) if angle>1e-12 else 0.
            # Translation projection is the shared progress when nonzero;
            # pure rotation uses signed rotation about the requested shortest axis.
            progress=float((actual[:3,3]-origin[:3,3])@delta/(length*length)) if length>1e-12 else rotation_progress
            expected_position=origin[:3,3]+np.clip(progress,0,1)*delta
            expected_rotation=rotation_matrix_from_rotation_vector(np.clip(progress,0,1)*rotation)@origin[:3,:3]
            pe=float(np.linalg.norm(actual[:3,3]-expected_position))
            re=float(np.linalg.norm(rotation_vector_from_matrix(actual[:3,:3]@expected_rotation.T)))
            maximum_position=max(maximum_position,pe);maximum_orientation=max(maximum_orientation,re)
            if length>1e-12 and angle>1e-12:
                maximum_progress_difference=max(maximum_progress_difference,abs(progress-rotation_progress))
            invalid=(pe>position_tolerance or re>orientation_tolerance or
                progress < -progress_tolerance or progress>1+progress_tolerance or
                progress<previous-progress_tolerance)
            if invalid and failure is None:
                failure=dict(edge=edge,fraction=fraction,q_rad=(a+fraction*dq).tolist(),
                    progress=progress,previous_progress=previous,position_m=pe,orientation_rad=re)
            previous=max(previous,progress)
    passed=failure is None and all(x['position_m']<=position_tolerance and
        x['orientation_rad']<=orientation_tolerance for x in ends)
    return dict(passed=passed,reason=None if passed else 'LIN_TASK_TCP_CONSTRAINT',
        maximum_line_error_m=maximum_position,maximum_orientation_error_rad=maximum_orientation,
        maximum_translation_rotation_progress_difference=maximum_progress_difference,
        endpoint_errors=ends,first_failure=failure,samples=samples,
        progress_semantics='line_projection' if length>1e-12 else 'signed_shortest_rotation_axis',
        sampling='existing_conservative_joint_edge_grid_2x_max_joint_and_4m_sweep',
        continuous_sweep_proof=False,position_tolerance_m=position_tolerance,orientation_tolerance_rad=orientation_tolerance)

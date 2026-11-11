"""SI timing, exact polynomial extrema and session-local execution authority."""
import hashlib
import json
import secrets
import threading
from dataclasses import dataclass, asdict
import numpy as np
from scipy.interpolate import BPoly, PPoly


def digest(x):
    return hashlib.sha256(json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def expected_controller_payload(config):
    """Controller test setting is distinct from the physical tool mass model."""
    mass=float(config.get('controller_payload_kg',config.get('tool_mass_kg',1.5)))
    if not np.isfinite(mass) or not 0<mass<=float(config.get('robot_rated_payload_kg',5.)):
        raise ValueError('INVALID_CONTROLLER_PAYLOAD_SETTING')
    return mass


@dataclass(frozen=True)
class Limits:
    lower: tuple
    upper: tuple
    velocity: tuple
    acceleration: tuple

    def arrays(self):
        a = [np.asarray(x,float) for x in (self.lower,self.upper,self.velocity,self.acceleration)]
        if any(x.shape!=(6,) or not np.isfinite(x).all() for x in a) or np.any(a[0]>=a[1]) or np.any(a[2]<=0) or np.any(a[3]<=0):
            raise ValueError('INVALID_LIMITS')
        return a


def inverse_mapping(q, mapping):
    q=np.asarray(q,float)
    if q.shape!=(6,) or not np.isfinite(q).all(): raise ValueError('INVALID_JOINTS')
    source=np.empty(6)
    source[list(mapping.source_indices)]=(q-np.asarray(mapping.zero_offsets_rad))/np.asarray(mapping.signs)
    return np.rad2deg(source).tolist() if mapping.source_unit=='degree' else source.tolist()


def prepare(points, limits, time_scale=10., period=.005, reach_bound_m=10.):
    """Preserve original nonuniform knots; interpolate q,v,a by quintic Hermite.

    Time scale >=1 divides v by scale and a by scale squared. The send grid is
    fixed-period, with <=one period terminal hold, never compressed final spacing.
    """
    if not np.isfinite(time_scale) or not 1<=time_scale<=100 or not .005<=period<=.009: raise ValueError('INVALID_TIMING')
    t=np.array([p['t'] for p in points],float)
    q=np.array([p['q'] for p in points],float)
    v=np.array([p['v'] for p in points],float)
    a=np.array([p['a'] for p in points],float)
    if len(t)<2 or q.shape!=(len(t),6) or v.shape!=q.shape or a.shape!=q.shape or not all(np.isfinite(x).all() for x in (t,q,v,a)):
        raise ValueError('INVALID_TIMED_TRAJECTORY')
    if abs(t[0])>1e-9 or np.any(np.diff(t)<=0) or t[-1]*time_scale>600: raise ValueError('INVALID_TIME_AXIS')
    if np.max(abs(v[[0,-1]]))>1e-6: raise ValueError('NONZERO_ENDPOINT_VELOCITY')
    if np.max(abs(a[[0,-1]]))>1e-6: raise ValueError('NONZERO_ENDPOINT_ACCELERATION')
    t=t*time_scale;v=v/time_scale;a=a/time_scale**2
    poly=PPoly.from_bernstein_basis(BPoly.from_derivatives(t,[[q[i],v[i],a[i]] for i in range(len(t))]))
    lower,upper,vmax,amax=limits.arrays()
    speed_bounds=[]
    # Check all extrema, not just sample times (overshoot between knots matters).
    for j in range(6):
        p=PPoly(poly.c[:,:,j],poly.x)
        for order,lo,hi in [(0,lower[j],upper[j]),(1,-vmax[j],vmax[j]),(2,-amax[j],amax[j])]:
            f=p.derivative(order); roots=f.derivative().roots(extrapolate=False)
            xs=np.r_[t,roots[np.isfinite(roots)]];ys=f(xs)
            if np.any(ys<lo-1e-8) or np.any(ys>hi+1e-8): raise ValueError(f'INTERPOLATION_LIMIT:{j+1}:{order}')
            if order==1:speed_bounds.append(float(np.max(abs(ys))))
    ts=np.arange(int(np.ceil(t[-1]/period))+1)*period
    if len(ts)>120001: raise ValueError('TRAJECTORY_TOO_LARGE')
    clipped=np.minimum(ts,t[-1]);qs=poly(clipped);vs=poly(clipped,1);acc=poly(clipped,2)
    # Discrete commands and the terminal transition must also remain bounded.
    dv=np.diff(qs,axis=0)/period
    if np.any(abs(dv)>vmax+1e-8) or (len(dv)>1 and np.any(abs(np.diff(dv,axis=0)/period)>amax+1e-8)):
        raise ValueError('DISCRETE_COMMAND_LIMIT')
    if np.any(abs(np.diff(qs,axis=0))>.002): raise ValueError('COMMAND_DISCONTINUITY')
    # Analytic angular-speed bound and a model-derived conservative reach bound
    # limit the motion of ANY body pair between checks to 1 mm (<2.4 mm gap).
    # Include every sent sample; add samples only where the swept bound needs it.
    # Two root-reach bounds cover each joint-to-point lever; two bodies may move.
    if not np.isfinite(reach_bound_m) or reach_bound_m<=0:raise ValueError('INVALID_REACH_BOUND')
    subdivisions=max(1,int(np.ceil(4*reach_bound_m*sum(speed_bounds)*period/.001)))
    if subdivisions*len(ts)>500000:raise ValueError('COLLISION_PATH_RESOURCE_LIMIT')
    dense=np.arange(0,t[-1],period/subdivisions)
    dense=np.unique(np.r_[dense,t,clipped])
    return dict(schema='eco65_timed_send_v1',period_s=period,time_scale=time_scale,
                source_points=points,source_duration_s=float(t[-1]/time_scale),
                scaled_duration_s=float(t[-1]),send_duration_s=float(ts[-1]),
                points=[dict(t=float(tt),q=qq.tolist(),v=vv.tolist(),a=aa.tolist()) for tt,qq,vv,aa in zip(ts,qs,vs,acc)],
                collision_path=poly(dense).tolist(),limits=asdict(limits),
                collision_sampling=dict(reach_bound_m=reach_bound_m,analytic_speed_bounds_rad_s=speed_bounds,
                    maximum_pair_displacement_between_checks_m=.001,subdivisions_per_send=subdivisions),
                interpolation='quintic_q_v_a_nonuniform_knots',terminal_hold_s=float(ts[-1]-t[-1]))


class Authority:
    """Preview JSON and historical planning flags cannot mint a live permit."""
    def __init__(self): self.session=secrets.token_hex(24);self._plans={};self._lock=threading.RLock()
    def register(self, prepared, binding, evidence):
        if evidence.get('native_plan')!='PASS' or evidence.get('sent_path_collision')!='PASS': raise ValueError('VALIDATION_REQUIRED')
        package=dict(schema='eco65_controlled_empty_package_v1',session=self.session,
                     plan_id=secrets.token_hex(16),prepared=prepared,binding=binding,evidence=evidence)
        self._plans[package['plan_id']]=digest(package)
        return package
    def verify(self, package, binding):
        if package.get('schema')!='eco65_controlled_empty_package_v1' or package.get('session')!=self.session or package.get('binding')!=binding or self._plans.get(package.get('plan_id'))!=digest(package):
            raise ValueError('UNTRUSTED_STALE_OR_PREVIEW_TRAJECTORY')
    def consume(self, package): self._plans.pop(package['plan_id'],None)
    def claim(self,package,binding):
        with self._lock:
            self.verify(package,binding);self.consume(package)
    def invalidate(self):
        with self._lock:self._plans.clear();self.session=secrets.token_hex(24)

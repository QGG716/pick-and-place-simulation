"""SI timing, exact polynomial extrema and session-local execution authority."""
import hashlib
import json
import secrets
import threading
from dataclasses import dataclass, asdict
import numpy as np
from scipy.interpolate import BPoly, PPoly
from .. import timing


def digest(x):
    return hashlib.sha256(json.dumps(x,sort_keys=True,separators=(',',':'),allow_nan=False).encode()).hexdigest()


def expected_controller_payload(config):
    """Controller test setting is distinct from the physical tool mass model."""
    mass=float(config.get('controller_payload_kg',config.get('tool_mass_kg',1.5)))
    if not np.isfinite(mass) or not 0<mass<=float(config.get('robot_rated_payload_kg',5.)):
        raise ValueError('INVALID_CONTROLLER_PAYLOAD_SETTING')
    return mass


# loaded_return_collision=RECORD_ONLY (user decision 2026-11-26, single-carton
# entry): the loaded return's audited collision is recorded, not vetoed. Only
# that segment, with the rule and its basis in the evidence, is accepted
# without a PASS; every other segment still needs sent_path_collision=PASS.
LOADED_RETURN_RECORD_ONLY_RULE='LOADED_RETURN_COLLISION_RECORDED_NOT_VETOED_2026_11_26'
LOADED_RETURN_STAGES=('return','transfer')


def sent_path_accepted(evidence):
    status=evidence.get('sent_path_collision')
    if status=='PASS':return True
    recorded=evidence.get('collision_recorded') or {}
    return (status=='COLLISION_RECORDED_NOT_VETOED' and recorded.get('rule')==LOADED_RETURN_RECORD_ONLY_RULE
            and recorded.get('stage') in LOADED_RETURN_STAGES and bool(str(recorded.get('basis') or '').strip()))


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


PROBE_GRIDS=('SENT_GRID','GEOMETRIC')


def prepare(points, limits, time_scale=10., period=.005, reach_bound_m=10., probe_grid='SENT_GRID'):
    """Preserve original nonuniform knots; interpolate q,v,a by quintic Hermite.

    Time scale >=1 divides v by scale and a by scale squared. The send grid is
    fixed-period, with <=one period terminal hold, never compressed final spacing.

    ``probe_grid`` selects the collision probe path (the send grid is the same):
    SENT_GRID (default) contains every sent sample plus uniform subdivisions
    from the global speed bound. GEOMETRIC (single-carton PYTHON flow, user
    request 2026-11-22, AGENTS 2026-11-16 3b) subdivides each knot interval of
    the same quintic by its own analytic joint-speed bound, so that consecutive
    probe states still move any body pair by at most 1 mm; it does not depend
    on the time scale and does not add the 5 ms send samples themselves.
    """
    if probe_grid not in PROBE_GRIDS: raise ValueError('INVALID_PROBE_GRID')
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
    speed_bounds=[];speed_extrema=[]
    # Check all extrema, not just sample times (overshoot between knots matters).
    for j in range(6):
        p=PPoly(poly.c[:,:,j],poly.x)
        for order,lo,hi in [(0,lower[j],upper[j]),(1,-vmax[j],vmax[j]),(2,-amax[j],amax[j])]:
            f=p.derivative(order); roots=f.derivative().roots(extrapolate=False)
            xs=np.r_[t,roots[np.isfinite(roots)]];ys=f(xs)
            if np.any(ys<lo-1e-8) or np.any(ys>hi+1e-8): raise ValueError(f'INTERPOLATION_LIMIT:{j+1}:{order}')
            if order==1:speed_bounds.append(float(np.max(abs(ys))));speed_extrema.append((xs,abs(ys)))
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
    if probe_grid=='GEOMETRIC':
        dense,per_interval=geometric_probe_times(t,speed_extrema,reach_bound_m)
        sampling=dict(method='GEOMETRIC_PER_KNOT_INTERVAL_ANALYTIC_SPEED_BOUND',reach_bound_m=reach_bound_m,
                      analytic_speed_bounds_rad_s=speed_bounds,maximum_pair_displacement_between_checks_m=.001,
                      knot_intervals=len(per_interval),probe_states=int(len(dense)),
                      max_subdivisions_per_interval=int(max(per_interval)),
                      rule='SUM_J_MAX_ABS_QDOT_ON_INTERVAL_TIMES_STEP_TIMES_4_REACH_LE_1_MM')
    else:
        subdivisions=max(1,int(np.ceil(4*reach_bound_m*sum(speed_bounds)*period/.001)))
        if subdivisions*len(ts)>500000:raise ValueError('COLLISION_PATH_RESOURCE_LIMIT')
        dense=np.arange(0,t[-1],period/subdivisions)
        dense=np.unique(np.r_[dense,t,clipped])
        sampling=dict(reach_bound_m=reach_bound_m,analytic_speed_bounds_rad_s=speed_bounds,
                      maximum_pair_displacement_between_checks_m=.001,subdivisions_per_send=subdivisions)
    return dict(schema='eco65_timed_send_v1',period_s=period,time_scale=time_scale,
                source_points=points,source_duration_s=float(t[-1]/time_scale),
                scaled_duration_s=float(t[-1]),send_duration_s=float(ts[-1]),
                points=[dict(t=float(tt),q=qq.tolist(),v=vv.tolist(),a=aa.tolist()) for tt,qq,vv,aa in zip(ts,qs,vs,acc)],
                collision_path=poly(dense).tolist(),limits=asdict(limits),collision_sampling=sampling,
                interpolation='quintic_q_v_a_nonuniform_knots',terminal_hold_s=float(ts[-1]-t[-1]))


# analytic_send_slowdown (2026-11-23): relative margin above the 1e-8 limit
# tolerance by which a skipped factor must exceed a limit.
SLOWDOWN_SKIP_MARGIN=1e-6


def interpolation_peaks(points, time_scale):
    """Largest |velocity| and |acceleration| per joint of the quintic prepare() builds.

    The same knots, PPoly construction, derivative roots and evaluation points
    as prepare()'s extremum check. Uniform time scaling by f divides the
    velocity peaks by f and the acceleration peaks by f**2 (the scaled
    Hermite data define the scaled quintic). Returns (v_peak, a_peak, t_end).
    """
    t=np.array([p['t'] for p in points],float)
    q=np.array([p['q'] for p in points],float)
    v=np.array([p['v'] for p in points],float)
    a=np.array([p['a'] for p in points],float)
    tend=float(t[-1])
    t=t*time_scale;v=v/time_scale;a=a/time_scale**2
    poly=PPoly.from_bernstein_basis(BPoly.from_derivatives(t,[[q[i],v[i],a[i]] for i in range(len(t))]))
    peaks={1:np.zeros(6),2:np.zeros(6)}
    for j in range(6):
        p=PPoly(poly.c[:,:,j],poly.x)
        for order in (1,2):
            f=p.derivative(order); roots=f.derivative().roots(extrapolate=False)
            xs=np.r_[t,roots[np.isfinite(roots)]]
            peaks[order][j]=float(np.max(abs(f(xs))))
    return peaks[1],peaks[2],tend


def provably_exceeds(peaks, limits, time_scale, factor):
    """True only when prepare(time_scale*factor) reaches its extremum check and must fail it.

    Factors whose earlier checks (time scale range, 600 s axis) may decide are
    never reported, so skipping them cannot change which error a search ends with.
    """
    vpeak,apeak,tend=peaks
    scale=time_scale*factor
    if not np.isfinite(scale) or not 1<=scale<=100 or tend*scale>600:
        return False
    _,_,vmax,amax=limits.arrays()
    m=1+SLOWDOWN_SKIP_MARGIN
    return bool(np.any(vpeak/factor>(vmax+1e-8)*m) or np.any(apeak/factor**2>(amax+1e-8)*m))


def slowdown_search(points, limits, time_scale, factors, period=.005, reach_bound_m=10., probe_grid='SENT_GRID',
                    prepare_function=None, peaks=True):
    """The first factor in ``factors`` whose prepare() succeeds, as the sequential loop finds it.

    The sequential loop tries every factor in order, goes on after an
    INTERPOLATION_LIMIT or DISCRETE_COMMAND_LIMIT error and raises the last
    factor's error. Here a factor that provably fails the velocity or
    acceleration extremum check is not tried (the last factor always is), so
    the factor found, its prepared result and the final error are the same.
    Returns (prepared, factor, attempts).
    """
    prepare_function=prepare if prepare_function is None else prepare_function
    known=None
    if peaks:
        try:known=interpolation_peaks(points,time_scale)
        except Exception:known=None   # invalid input: prepare() itself reports it
    attempts=[]
    for k,factor in enumerate(factors):
        last=k==len(factors)-1
        if known is not None and not last and provably_exceeds(known,limits,time_scale,factor):
            attempts.append(dict(factor=factor,result='SKIPPED_PROVABLY_EXCEEDS_LIMITS'));continue
        try:
            return prepare_function(points,limits,time_scale*factor,period,reach_bound_m,probe_grid=probe_grid),factor,attempts
        except ValueError as error:
            attempts.append(dict(factor=factor,result=str(error)))
            if last or not str(error).startswith(('INTERPOLATION_LIMIT','DISCRETE_COMMAND_LIMIT')):raise


def geometric_probe_times(t,speed_extrema,reach_bound_m,step_m=.001,limit=500000):
    """Probe times on the knot intervals of one quintic, from local analytic speed bounds.

    For interval k, M_jk is the largest |dq_j/dt| on it (at its ends or at a
    root of d2q_j/dt2 inside it: ``speed_extrema`` holds every such abscissa
    with |dq_j/dt| there). Splitting the interval into n_k equal steps with
    4 * reach * sum_j(M_jk) * dt_k / n_k <= step_m bounds every body-pair
    displacement between consecutive probe states by step_m, the same lemma
    and constant as the SENT_GRID subdivision with a global bound. Positions
    are not involved, so the result is independent of uniform time scaling
    (dt_k * M_jk is invariant).
    """
    t=np.asarray(t,float);n=len(t)-1;bound=np.zeros(n)
    for xs,ys in speed_extrema:
        xs=np.asarray(xs,float);ys=np.asarray(ys,float);local=np.zeros(n)
        # Ends of each interval: knot k bounds intervals k-1 and k.
        knots=np.searchsorted(t,xs);exact=(knots<len(t))&(abs(t[np.minimum(knots,n)]-xs)<=1e-12*max(1.,abs(t[-1])))
        inside=np.clip(np.searchsorted(t,xs,side='right')-1,0,n-1)
        np.maximum.at(local,inside,ys)
        k=knots[exact];v=ys[exact]
        left=k-1;ok=left>=0;np.maximum.at(local,left[ok],v[ok])
        right=k;ok=right<n;np.maximum.at(local,right[ok],v[ok])
        bound+=local
    steps=np.maximum(1,np.ceil(4*reach_bound_m*bound*np.diff(t)/step_m)).astype(int)
    if int(steps.sum())+1>limit:raise ValueError('COLLISION_PATH_RESOURCE_LIMIT')
    times=[t[k]+(t[k+1]-t[k])*np.arange(steps[k])/steps[k] for k in range(n)]
    return np.unique(np.r_[np.concatenate(times),t[-1]]),steps.tolist()


class Authority:
    """Preview JSON and historical planning flags cannot mint a live permit."""
    def __init__(self,schema='eco65_controlled_empty_package_v1'):
        if schema not in {'eco65_controlled_empty_package_v1','eco65_single_carton_segment_v1'}:raise ValueError('UNSUPPORTED_EXECUTION_SCOPE')
        self.schema=schema;self.session=secrets.token_hex(24);self._plans={};self._lock=threading.RLock()
        self.digest_method='CANONICAL_JSON_SHA256';self.signing_memo=None
    def use_signing_memo(self):
        """digest_memo (2-second planning, 2026-11-23; tree digest only): register() and
        prewarm() share one memo per session, so subtrees already digested are not
        re-read at issue. verify()/claim() never use it and always re-read everything."""
        with self._lock:
            if self.digest_method!='CANONICAL_TREE_SHA256':raise ValueError('SIGNING_MEMO_REQUIRES_TREE_DIGEST')
            self.signing_memo={}
    def prewarm(self,values):
        """Digest these subtrees into the signing memo ahead of register (no effect without a memo)."""
        memo=self.signing_memo
        if memo is None:return
        from ..plan_digest import tree_digest
        for value in values:tree_digest(value,memo)
    def use_canonical_binary_digest(self):
        """Single-carton planning_module NATIVE (AGENTS 2026-11-16 3g): issue and claim both use
        SHA-256 over the canonical binary frame form (workbench/binary_frames.canonical_digest)."""
        with self._lock:
            if self._plans:raise ValueError('AUTHORITY_DIGEST_METHOD_FIXED_ONCE_PLANS_EXIST')
            self.digest_method='CANONICAL_BINARY_SHA256'
    def use_tree_digest(self):
        """Single-carton structured_plan_digest (2-second planning, 2026-11-22; AGENTS 2026-11-16 3g):
        issue and claim both use SHA-256 over the canonical tree form (workbench/plan_digest.tree_digest)."""
        with self._lock:
            if self._plans:raise ValueError('AUTHORITY_DIGEST_METHOD_FIXED_ONCE_PLANS_EXIST')
            self.digest_method='CANONICAL_TREE_SHA256'
    def _digest(self,package,signing=False):
        if self.digest_method=='CANONICAL_TREE_SHA256':
            from ..plan_digest import tree_digest
            return tree_digest(package,self.signing_memo if signing else None)
        if self.digest_method=='CANONICAL_BINARY_SHA256':
            from ..binary_frames import canonical_digest
            return canonical_digest(package)
        return digest(package)
    def register(self, prepared, binding, evidence):
        if evidence.get('native_plan')!='PASS' or not sent_path_accepted(evidence): raise ValueError('VALIDATION_REQUIRED')
        package=dict(schema=self.schema,session=self.session,
                     plan_id=secrets.token_hex(16),prepared=prepared,binding=binding,evidence=evidence)
        with timing.span('authority.register_digest'):self._plans[package['plan_id']]=self._digest(package,signing=True)
        return package
    def verify(self, package, binding):
        with timing.span('authority.verify_digest'):
            if package.get('schema')!=self.schema or package.get('session')!=self.session or package.get('binding')!=binding or self._plans.get(package.get('plan_id'))!=self._digest(package):
                raise ValueError('UNTRUSTED_STALE_OR_PREVIEW_TRAJECTORY')
    def consume(self, package): self._plans.pop(package['plan_id'],None)
    def claim(self,package,binding):
        with self._lock:
            self.verify(package,binding);self.consume(package)
    def invalidate(self):
        with self._lock:
            self._plans.clear();self.session=secrets.token_hex(24)
            if self.signing_memo is not None:self.signing_memo={}

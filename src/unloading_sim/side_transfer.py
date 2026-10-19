"""Named planar support transitions for the explicit right-inner template only."""
from copy import deepcopy
import numpy as np
from .collision_policy import PhysicsCheckedStackTracker
from .lifted_reception import bounds
from .pair_clearance import obb_surface_distance
from .moveit2_backend import box_message


class SideTransferTracker(PhysicsCheckedStackTracker):
    def __init__(self, target, neighbors, policy, margin_m, tolerance_m):
        super().__init__(target, neighbors, policy, margin_m, tolerance_m)
        self.origin=target
        bottom=bounds(target)[0][2]
        self.planar=sorted(b.name for b in self.neighbors.values()
            if abs(bounds(b)[1][2]-bottom)<=1e-9
            and np.allclose(b.rotation[:,2],[0,0,1],atol=1e-12,rtol=0))
        self.touched=set();self.released=set()
        self._advance(target)

    def _advance(self, box):
        for name,other in self.neighbors.items():
            gap=obb_surface_distance(box,other)
            if gap+1e-9 < self.policy.free_space_clearance_m:
                self.touched.add(name)
            elif name in self.touched:
                self.released.add(name)

    def clone(self):
        result=SideTransferTracker(self.origin,list(self.neighbors.values()),self.policy,self.margin_m,self.tolerance_m)
        result.last_box=self.last_box;result.fully_released=self.fully_released
        result.touched=set(self.touched);result.released=set(self.released)
        return result

    def state_failure(self, box, obstacles, support_names=()):
        for name,other in self.neighbors.items():
            gap=obb_surface_distance(box,other)
            signed=box.signed_distance_obb(other)
            if not np.isfinite([gap,signed]).all():
                raise ValueError('SIDE_TRANSFER_NONFINITE_DISTANCE')
            if gap+1e-9>=self.policy.free_space_clearance_m:
                continue
            if name in self.released:
                reason='SIDE_TRANSFER_RELEASED_PAIR_REENTRY'
            elif name not in self.planar:
                reason='SIDE_TRANSFER_NONPLANAR_CLEARANCE'
            elif signed < -self.tolerance_m or bounds(box)[0][2]<bounds(other)[1][2]-self.tolerance_m:
                reason='SIDE_TRANSFER_SUPPORT_PENETRATION'
            else:
                continue
            return dict(reason=reason,pair=[box.name,name],signed_distance_m=signed,surface_distance_m=gap)
        failure=super().state_failure(box,obstacles,support_names)
        if failure is None:self._advance(box)
        return failure

    def evidence(self):
        result=super().evidence()
        result['controlled_side_transfer']=dict(schema='right_inner_planar_history_v1',
            original_target=box_message(self.origin),planar_support_ids=self.planar,
            touched_ids=sorted(self.touched),released_ids=sorted(self.released))
        return deepcopy(result)

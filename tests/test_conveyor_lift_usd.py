"""Optional installed-USD tests: run directly, without launching Isaac Sim."""
from pathlib import Path
import sys
import tempfile
import unittest
from dataclasses import replace
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/"src"),str(ROOT/"scripts")]
from unloading_sim.workcell_layout import load_workcell_layout
from unloading_sim.conveyor_lift import coupled_conveyor_state
from unloading_sim.geometry import OBB
from coupled_conveyor_usd import create_stage, apply_state
try:
    from pxr import Usd, UsdGeom, UsdPhysics
except ImportError:
    Usd=None


@unittest.skipIf(Usd is None,"optional USD not installed")
class LiftUSD(unittest.TestCase):
    def setUp(self):
        layout=load_workcell_layout(ROOT/"configs/workcells/m710id70_unloading_layout_v1.yaml")
        self.low=coupled_conveyor_state(layout,.6)
        self.high=coupled_conveyor_state(layout,2.05)
        self.tmp=tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.path=Path(self.tmp.name)/"lift.usda"
        self.stage,_=create_stage(self.low,self.path)

    def test_collision_and_render_update_roundtrip(self):
        apply_state(self.stage,self.high)
        cache=UsdGeom.XformCache()
        for box in self.high.boxes:
            prim=self.stage.GetPrimAtPath("/CoupledConveyors/Bodies/"+box.name)
            np.testing.assert_allclose(np.asarray(cache.GetLocalToWorldTransform(prim)).T,box.world_from_local)
            self.assertTrue(self.stage.GetPrimAtPath(str(prim.GetPath())+"/Collider").HasAPI(UsdPhysics.CollisionAPI))
        self.stage.GetRootLayer().Save()
        reopened=Usd.Stage.Open(str(self.path))
        self.assertEqual(reopened.GetDefaultPrim().GetCustomDataByKey("scene_fingerprint"),self.high.scene_fingerprint)
        apply_state(reopened,self.low)

    def test_tampered_pose_rejected_before_other_updates(self):
        prim=self.stage.GetPrimAtPath("/CoupledConveyors/Bodies/conveyor_longitudinal")
        op=UsdGeom.Xformable(prim).GetOrderedXformOps()[0]
        matrix=op.Get();matrix.SetTranslateOnly((99,0,0));op.Set(matrix)
        with self.assertRaisesRegex(ValueError,"USD_POSE_DOES_NOT_MATCH"):
            apply_state(self.stage,self.high)
        self.assertEqual(self.stage.GetDefaultPrim().GetCustomDataByKey("scene_fingerprint"),self.low.scene_fingerprint)

    def test_unrelated_world_change_and_missing_ids_rejected(self):
        box=next(b for b in self.high.boxes if b.category=="carton")
        moved=OBB(box.center+[0,0,1],box.half_extents,box.rotation,box.name,box.category)
        changed=replace(self.high,boxes=tuple(moved if b.name==box.name else b for b in self.high.boxes))
        with self.assertRaisesRegex(ValueError,"UNRELATED_WORLD_OBJECT_CHANGED"):
            apply_state(self.stage,changed)
        with self.assertRaisesRegex(ValueError,"LIFT_OBJECT_IDENTITY_CHANGED"):
            apply_state(self.stage,replace(self.high,boxes=self.high.boxes[:-1]))

    def test_dynamic_claims_not_enabled(self):
        for prim in self.stage.Traverse():
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                self.assertTrue(UsdPhysics.RigidBodyAPI(prim).GetKinematicEnabledAttr().Get())
            if prim.IsA(UsdPhysics.Joint):
                self.assertFalse(UsdPhysics.Joint(prim).GetJointEnabledAttr().Get())
            self.assertFalse(prim.HasAPI(UsdPhysics.ArticulationRootAPI))

    def test_fixed_bearing_cannot_move(self):
        box=next(b for b in self.high.boxes if b.name=="lift_rear_bearing")
        moved=OBB(box.center+[0,0,.1],box.half_extents,box.rotation,box.name,box.category)
        changed=replace(self.high,boxes=tuple(moved if b.name==box.name else b for b in self.high.boxes))
        with self.assertRaisesRegex(ValueError,"UNRELATED_WORLD_OBJECT_CHANGED"):
            apply_state(self.stage,changed)


if __name__=="__main__":
    unittest.main()

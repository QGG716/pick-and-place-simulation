"""Optional USD/Isaac adapter for the coupled lift concept, without a simulator.

Kinematic colliders and disabled joint topology are deliberate: no mass, actuator
or hydraulic specification has been supplied. This is not a dynamic articulation.
"""
from pathlib import Path
import json
import numpy as np


def _matrix(pose):
    from pxr import Gf
    return Gf.Matrix4d(*np.asarray(pose).T.flatten().tolist())


def create_stage(state, output, *, asset_config=None, asset_cache=None):
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
    output = Path(output)
    if output.exists():
        raise FileExistsError(output)
    evidence = state.evidence()
    stage = Usd.Stage.CreateNew(str(output))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    root = UsdGeom.Xform.Define(stage,"/CoupledConveyors")
    stage.SetDefaultPrim(root.GetPrim())
    root.GetPrim().SetCustomDataByKey("qualification_status","NOT_EVALUATED")
    root.GetPrim().SetCustomDataByKey("physical_execution","NOT_RUN")
    root.GetPrim().SetCustomDataByKey("control","KINEMATIC_GEOMETRY_NO_ACTUATOR_MODEL")
    material = UsdShade.Material.Define(stage,"/CoupledConveyors/Materials/Frame")
    shader = UsdShade.Shader.Define(stage, str(material.GetPath())+"/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor",Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(.20,.28,.35))
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),"surface")
    asset_records=[]
    for item in evidence["world"]:
        path="/CoupledConveyors/Bodies/"+item["id"]
        body=UsdGeom.Xform.Define(stage,path)
        body.AddTransformOp().Set(_matrix(item["pose"]))
        moving=item["id"] in evidence["mechanism"]["moving_object_ids"]
        if moving:
            api=UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
            api.CreateKinematicEnabledAttr(True)
        cube=UsdGeom.Cube.Define(stage,path+"/Collider")
        cube.CreateSizeAttr(1.)
        cube.AddScaleOp().Set(Gf.Vec3f(*item["size"]))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        color=(.85,.49,.12) if item["id"].startswith("lift_") else (.20,.55,.65)
        cube.CreateDisplayColorAttr([Gf.Vec3f(*color)])
        if item["id"].startswith("conveyor_") and asset_cache is not None:
            from workcell_visuals import attach_a06_belts
            visual=UsdGeom.Xform.Define(stage,path+"/OfficialVisual")
            visual.AddScaleOp().Set(Gf.Vec3f(*item["size"]))
            record=attach_a06_belts(stage,str(visual.GetPath()),dict(name=item["id"],size_xyz_m=item["size"]),asset_config,asset_cache,material)
            asset_records.append(record)
            cube.CreatePurposeAttr(UsdGeom.Tokens.guide)  # collider remains enabled
    # Cross pins use the official USD joint schema, but are not force-solved.
    # Enabling these alone would omit the end sliders/swivel shoes; fail closed.
    for stack in evidence["mechanism"]["stacks"]:
        for hinge in stack["hinges"]:
            joint=UsdPhysics.RevoluteJoint.Define(stage,"/CoupledConveyors/Topology/"+hinge["id"])
            joint.CreateAxisAttr(UsdPhysics.Tokens.y)  # each bar's local Y is the hinge axis
            joint.CreateJointEnabledAttr(False)
            for k, body_id in enumerate(hinge["bodies"]):
                item=next(x for x in evidence["world"] if x["id"]==body_id)
                pose=np.asarray(item["pose"])
                local=pose[:3,:3].T@(np.asarray(hinge["position"])-pose[:3,3])
                getattr(joint,f"CreateBody{k}Rel")().SetTargets(["/CoupledConveyors/Bodies/"+body_id])
                getattr(joint,f"CreateLocalPos{k}Attr")().Set(Gf.Vec3f(*local))
            joint.GetPrim().SetCustomDataByKey("status","TOPOLOGY_ONLY_DYNAMIC_LOOP_NOT_CONFIGURED")
    root.GetPrim().SetCustomDataByKey("scene_fingerprint", state.scene_fingerprint)
    root.GetPrim().SetCustomDataByKey("schema", evidence["mechanism"]["schema"])
    root.GetPrim().CreateAttribute("lift:state",Sdf.ValueTypeNames.String).Set(json.dumps(evidence))
    stage.GetRootLayer().Save()
    return stage, asset_records


def apply_state(stage,state):
    """Validate the complete update before writing any prim; stopped-stage use.

    Caller must NOT use this to teleport a running dynamic articulation. The
    concept has kinematic bodies, and no robot planner may treat it as approved.
    """
    from pxr import UsdGeom
    root=stage.GetPrimAtPath("/CoupledConveyors")
    if not root or root.GetCustomDataByKey("control")!="KINEMATIC_GEOMETRY_NO_ACTUATOR_MODEL":
        raise ValueError("NOT_OWNED_KINEMATIC_LIFT_STAGE")
    previous=json.loads(root.GetAttribute("lift:state").Get())
    new=state.evidence()
    old={x["id"]:x for x in previous["world"]}
    if root.GetCustomDataByKey("scene_fingerprint")!=previous["scene_fingerprint"]:
        raise ValueError("STALE_LIFT_SCENE_IDENTITY")
    if set(old)!={x["id"] for x in new["world"]}:
        raise ValueError("LIFT_OBJECT_IDENTITY_CHANGED")
    if (previous["mechanism"]["moving_object_ids"]!=new["mechanism"]["moving_object_ids"] or
            previous["mechanism"]["rear_hinge_world"]!=new["mechanism"]["rear_hinge_world"]):
        raise ValueError("LIFT_CONTROL_BINDING_CHANGED")
    updates=[]
    for item in new["world"]:
        prior=old[item["id"]]
        if item["size"]!=prior["size"] or item["category"]!=prior["category"]:
            raise ValueError("LIFT_GEOMETRY_IDENTITY_CHANGED")
        if item["id"] not in previous["mechanism"]["moving_object_ids"] and item!=prior:
            raise ValueError("UNRELATED_WORLD_OBJECT_CHANGED")
        body=UsdGeom.Xformable(stage.GetPrimAtPath("/CoupledConveyors/Bodies/"+item["id"]))
        ops=body.GetOrderedXformOps()
        if len(ops)!=1 or not np.allclose(np.asarray(ops[0].Get()),np.asarray(prior["pose"]).T,atol=1e-12,rtol=0):
            raise ValueError("USD_POSE_DOES_NOT_MATCH_SCENE_IDENTITY")
        updates.append((ops[0],_matrix(item["pose"])))
    for op,value in updates:
        op.Set(value)
    root.SetCustomDataByKey("scene_fingerprint",state.scene_fingerprint)
    root.GetAttribute("lift:state").Set(json.dumps(new))

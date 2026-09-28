"""Independent CPU PhysX channel probe; no business assets or planner imports.

Run with the installed Isaac Python. Expectations come from Newton/Euler:
I_pivot*qdd = joint torque + m*g*r*cos(q) + applied body torque.
The +Y revolute axis, COM=(0.5,0,0), gravity=(0,0,-9.81) fix the sign.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import numpy as np
    import omni.usd
    from isaacsim.core.api import World
    from isaacsim.core.experimental.prims import Articulation, RigidPrim
    from pxr import Gf, UsdGeom, UsdPhysics, PhysxSchema

    dt = 1 / 240
    world = World(physics_dt=dt, rendering_dt=dt, stage_units_in_meters=1.0)
    world.get_physics_context().enable_gpu_dynamics(False)
    world.get_physics_context().set_broadphase_type("MBP")
    stage = omni.usd.get_context().get_stage()
    root = UsdGeom.Xform.Define(stage, "/Probe").GetPrim()
    base = UsdGeom.Xform.Define(stage, "/Probe/base").GetPrim()
    arm = UsdGeom.Xform.Define(stage, "/Probe/arm").GetPrim()
    for prim, mass, com in [(base, 1., (0., 0., 0.)), (arm, 2., (.5, 0., 0.))]:
        UsdPhysics.RigidBodyAPI.Apply(prim)
        m = UsdPhysics.MassAPI.Apply(prim)
        m.CreateMassAttr(mass)
        m.CreateCenterOfMassAttr(Gf.Vec3f(*com))
        m.CreateDiagonalInertiaAttr(Gf.Vec3f(.02, .2, .2))
    fixed = UsdPhysics.FixedJoint.Define(stage, "/Probe/fixed")
    fixed.CreateBody1Rel().SetTargets([base.GetPath()])
    UsdPhysics.ArticulationRootAPI.Apply(fixed.GetPrim())
    joint = UsdPhysics.RevoluteJoint.Define(stage, "/Probe/hinge")
    joint.CreateBody0Rel().SetTargets([base.GetPath()])
    joint.CreateBody1Rel().SetTargets([arm.GetPath()])
    joint.CreateAxisAttr("Y")
    PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim()).CreateJointFrictionAttr(0.)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "angular")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(1000.)
    drive.CreateDampingAttr(100.)
    drive.CreateMaxForceAttr(100.)
    articulation = Articulation("/Probe")
    body = RigidPrim("/Probe/arm")
    world.reset()
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
    import omni.physics.tensors
    from unloading_sim.joint_effort import (CONDITIONS, JointEffortMonitor,
        collect_isaac_effort_context, collect_isaac_effort_sample)
    context = collect_isaac_effort_context(articulation, omni.physics.tensors,
        world.get_physics_context(), [p for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)])
    monitor = JointEffortMonitor(list(articulation.dof_names), [100.], "probe authored finite drive limit", dt,
        context["source"], context["configured_limit_readback_nm"])
    assert all(v == 0. for prop in context["dof_friction_properties"] for row in prop for v in row)
    assert list(articulation.dof_names) == ["hinge"]
    assert len(articulation.link_names) == 2
    assert not joint.GetLowerLimitAttr().Get() > -float("inf")
    assert not joint.GetUpperLimitAttr().Get() < float("inf")
    observations = []
    gravity_hold_evidence = None
    rows = []
    previous_velocity = None
    for phase, external, effort in [("gravity_hold", 0., 0.), ("external_plus_3Nm", 3., 0.),
                                    ("explicit_plus_2Nm", 0., 2.), ("small_motion", 0., 0.)]:
        for index in range(480):
            target = .02 * math.sin(2 * math.pi * index * dt) if phase == "small_motion" else 0.
            articulation.set_dof_position_targets([[target]])
            articulation.set_dof_efforts([[effort]])
            body.apply_forces_and_torques_at_pos(torques=[[0., external, 0.]], local_frame=False)
            before = float(articulation.get_dof_projected_joint_forces().numpy()[0, 0])
            world.step(render=False, update_fabric=True)
            observations.append(collect_isaac_effort_sample(articulation, monitor, len(rows)+1,
                (len(rows)+1)*dt, dict.fromkeys(CONDITIONS, True)))
            if phase == "gravity_hold" and index == 479:
                gravity_hold_evidence = monitor.summary(480)
            q = float(articulation.get_dof_positions().numpy()[0, 0])
            qd = float(articulation.get_dof_velocities().numpy()[0, 0])
            qdd = None if previous_velocity is None else (qd - previous_velocity) / dt
            previous_velocity = qd
            force, torque = articulation.get_link_incoming_joint_force()
            rows.append(dict(phase=phase, physics_step=len(rows) + 1, simulation_time=(len(rows)+1)*dt,
                             observation_phase="post_physics_step", q=q, qd=qd, qdd=qdd,
                             target=target, external_body_torque_nm=external, explicit_target_nm=effort,
                             explicit_input_nm=float(articulation.get_dof_efforts().numpy()[0, 0]),
                             projected_nm=float(articulation.get_dof_projected_joint_forces().numpy()[0, 0]),
                             pre_step_projected_nm=before,
                             incoming_force=force.numpy()[0].tolist(), incoming_torque=torque.numpy()[0].tolist(),
                             expected_net_joint_nm=None if qdd is None else .7*qdd - 9.81*math.cos(q) - external))
    report = {"scope": "channel_probe_not_business_qualification", "backend": "CPU_PhysX",
              "joint_names": list(articulation.dof_names), "link_names": list(articulation.link_names),
              "max_effort_readback_nm": articulation.get_dof_max_efforts().numpy().tolist(),
              "model": {"mass_kg": 2., "com_m": [.5,0,0], "Iyy_com_kg_m2": .2,
                        "Iyy_pivot_kg_m2": .7, "gravity_m_s2": [0,0,-9.81], "axis": "+Y",
                        "contacts": False, "joint_limits": False, "joint_friction": 0.},
              "rows": rows, "collection_context": context,
              "collection_summary": monitor.summary(len(rows)),
              "gravity_hold_evidence": gravity_hold_evidence}
    with (args.output / "joint_effort_observations.jsonl").open("w") as stream:
        for observation in observations:
            stream.write(json.dumps(observation, allow_nan=False) + "\n")
    (args.output / "probe.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print("PROBE_COMPLETE", flush=True)
    app.close()


if __name__ == "__main__":
    main()

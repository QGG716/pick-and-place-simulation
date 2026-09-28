"""Synthetic XY mechanism fixture, never FANUC engineering evidence."""
from unloading_sim.planning_contract import fingerprint

def unit_scene():
    urdf = '''<robot name="unit_xy" xmlns:tesseract="https://tesseract-robotics.github.io" tesseract:make_convex="false">
      <link name="world"/><link name="base_link"/><link name="x"/>
      <link name="flange"><collision><geometry><box size="0.10 0.10 0.10"/></geometry></collision></link>
      <link name="tool0"/><link name="backend_tcp"/>
      <link name="obstacle"><collision><geometry><box size="0.2 0.6 0.2"/></geometry></collision></link>
      <joint name="base" type="fixed"><parent link="world"/><child link="base_link"/></joint>
      <joint name="Jx" type="prismatic"><parent link="base_link"/><child link="x"/><origin xyz="0 0 0.5"/><axis xyz="1 0 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
      <joint name="Jy" type="prismatic"><parent link="x"/><child link="flange"/><axis xyz="0 1 0"/><limit lower="-1" upper="1" effort="10" velocity="1"/></joint>
      <joint name="tool" type="fixed"><parent link="flange"/><child link="tool0"/></joint>
      <joint name="tcp" type="fixed"><parent link="tool0"/><child link="backend_tcp"/></joint>
      <joint name="obstacle_mount" type="fixed"><parent link="world"/><child link="obstacle"/><origin xyz="0 0 0.5"/></joint>
      </robot>'''
    scene = dict(urdf=urdf, srdf='<robot name="unit_xy"><group name="manipulator"><chain base_link="base_link" tip_link="tool0"/></group></robot>',
        joint_names=["Jx", "Jy"], joint_limits=[[-1, 1], [-1, 1]],
        joints=[dict(name="Jx", link="x", type="prismatic", axis=[1, 0, 0]),
                dict(name="Jy", link="flange", type="prismatic", axis=[0, 1, 0])],
        active_links=["flange"], collision_objects=["flange", "obstacle"], default_margin=.005, pair_margins=[],
        constraints=dict(joint_margin=0., maximum_jacobian_condition=100., radial_limit=None, edge_resolution_rad=.055, point_motion_bound_m=.00125, lever_arm_m=4.))
    scene["fingerprint"] = fingerprint(scene)
    return scene

# FANUC M-20iD/35 robot description

The meshes and kinematic values in this directory come from FANUC
Corporation's official `fanuc_description` repository:

- Repository: https://github.com/FANUC-CORPORATION/fanuc_description
- Package: `fanuc_m20_description`
- Model: `m20_35_18d` (M-20iD/35)
- Source macro: `urdf/m20_35_18d_urdf_macro.xacro`
- License: Apache-2.0 (see `LICENSE.txt`)

`m20_35_18d.urdf` is the macro expanded into a standalone URDF with local
relative mesh paths so the simulator does not require ROS or Xacro at runtime.

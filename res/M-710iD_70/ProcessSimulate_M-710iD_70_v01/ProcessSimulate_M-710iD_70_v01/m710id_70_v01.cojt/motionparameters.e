 config_family		cf_over_head_pos;
joint_config_family	j3	joint_cf_elbow_up;
joint_config_family	j5	joint_cf_pos;
default_turns		j1 0 -180,	j4 0 -180,	j6 0 -180;
elbow_jointj2_j3;
wrist_joint j5;

cart_max_lin_speed 4000;
cart_basic_lin_speed 4000;
speed_lim_jnt 4000;
default_control_parm 4000;

use_config use;
cart_turns use_as_possible;
single_joint_prof single_prof;

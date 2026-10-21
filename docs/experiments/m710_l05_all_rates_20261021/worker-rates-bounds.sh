#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/mount-install:${AMENT_PREFIX_PATH:-}
export LD_LIBRARY_PATH=/mount-install/lib:/mount-build:${LD_LIBRARY_PATH:-}
export M710_PLANNING_ONLY_ALL_JOINT_RATES_EXCEPTION=carton_l05_c03
export LD_PRELOAD=/l05-all-rates-20261021/libl05_joint_rates_bounds.so
exec /mount-build/m710_moveit_worker

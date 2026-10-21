from pathlib import Path
import subprocess,shlex
r=Path('/root/autodl-tmp/m710-mount-20261021/rootfs');p='/l05-all-rates-20261021';w='/work-l05-rates-bounds-655b495'
flags=(r/'mount-build/CMakeFiles/m710_moveit_worker.dir/flags.make').read_text()
inc=shlex.split(next(l.split('=',1)[1] for l in flags.splitlines() if l.startswith('CXX_INCLUDES =')))
base=['g++','-std=c++17','-O2',*inc,'-L/opt/ros/humble/lib']
def run(cmd):subprocess.run(['chroot',str(r),'/bin/bash','-c','source /opt/ros/humble/setup.bash; exec '+shlex.join(cmd)],check=True)
run(base+['-shared','-fPIC',w+'/tools/diagnostics/pilz_l05_joint_rates_only.cpp','-o',p+'/libl05_joint_rates_bounds.so','-lmoveit_robot_state','-lmoveit_robot_model','-lsrdfdom','-L/opt/ros/humble/lib/x86_64-linux-gnu','-lurdfdom_model','-ljoint_limits_common','-ldl'])
run(base+[w+'/tools/diagnostics/pilz_l05_joint_rates_only_test.cpp','-o',p+'/joint-rates-bounds-test','-ltrajectory_generation_common','-lmoveit_robot_state','-lmoveit_robot_model','-lsrdfdom','-L/opt/ros/humble/lib/x86_64-linux-gnu','-lurdfdom_model','-ljoint_limits_common'])
run(['/usr/bin/env','LD_PRELOAD='+p+'/libl05_joint_rates_bounds.so',p+'/joint-rates-bounds-test'])

run(base+[w+'/tools/diagnostics/pilz_l05_joint_rates_only_test.cpp','-o',p+'/joint-rates-bounds-local-test','-L'+p,'-Wl,-rpath,'+p,'-l:libl05_joint_rates_bounds.so','-lmoveit_robot_state','-lmoveit_robot_model','-lsrdfdom','-L/opt/ros/humble/lib/x86_64-linux-gnu','-lurdfdom_model','-ljoint_limits_common','-ldl'])
run(['/usr/bin/env','LD_PRELOAD='+p+'/libl05_joint_rates_bounds.so',p+'/joint-rates-bounds-local-test'])

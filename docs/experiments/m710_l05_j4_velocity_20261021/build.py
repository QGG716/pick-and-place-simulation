from pathlib import Path
import subprocess,shlex
r=Path('/root/autodl-tmp/m710-mount-20261021/rootfs');p='/l05-j4-velocity-20261021';w='/work-j4-velocity-b6abb7a'
flags=(r/'mount-build/CMakeFiles/m710_moveit_worker.dir/flags.make').read_text()
inc=shlex.split(next(l.split('=',1)[1] for l in flags.splitlines() if l.startswith('CXX_INCLUDES =')))
base=['g++','-std=c++17','-O2',*inc,'-L/opt/ros/humble/lib']
def run(cmd):subprocess.run(['chroot',str(r),'/bin/bash','-c','source /opt/ros/humble/setup.bash; exec '+shlex.join(cmd)],check=True)
run(base+['-shared','-fPIC',w+'/tools/diagnostics/pilz_l05_j4_velocity_only.cpp','-o',p+'/libl05_j4_velocity_only.so','-ljoint_limits_common','-ldl'])
run(base+[w+'/tools/diagnostics/pilz_l05_j4_velocity_only_test.cpp','-o',p+'/velocity-only-test','-ltrajectory_generation_common','-ljoint_limits_common'])
run(['/usr/bin/env','LD_PRELOAD='+p+'/libl05_j4_velocity_only.so',p+'/velocity-only-test'])

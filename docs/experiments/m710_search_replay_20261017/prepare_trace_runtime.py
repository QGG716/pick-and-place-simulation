from pathlib import Path
import subprocess,json,hashlib
R=Path('/root/autodl-tmp/m710-moveit2-20260922/rootfs');W=R/'work-single-contact-20261016';D=R/'tmp/search-trace-20261017'
P=D/'install';B=D/'build'
for path in ['lib','share/m710_moveit_backend','share/ament_index/resource_index/packages','share/ament_index/resource_index/moveit_core__pluginlib__plugin']:(P/path).mkdir(parents=True,exist_ok=True)
import shutil
shutil.copy2(B/'libm710_search_trace.so',P/'lib')
shutil.copy2(W/'ros2/m710_moveit_backend/search_trace_plugin.xml',P/'share/m710_moveit_backend')
(P/'share/ament_index/resource_index/packages/m710_moveit_backend').write_text('')
(P/'share/ament_index/resource_index/moveit_core__pluginlib__plugin/m710_moveit_backend').write_text('share/m710_moveit_backend/search_trace_plugin.xml')
(D/'worker.sh').write_text('''#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
export AMENT_PREFIX_PATH=/tmp/search-trace-20261017/install:$AMENT_PREFIX_PATH
export LD_LIBRARY_PATH=/tmp/search-trace-20261017/install/lib:$LD_LIBRARY_PATH
exec /tmp/search-trace-20261017/build/m710_moveit_worker
''');(D/'worker.sh').chmod(0o755)
paths=[
 'ros2/m710_moveit_backend/CMakeLists.txt','ros2/m710_moveit_backend/search_trace_plugin.xml',
 *['ros2/m710_moveit_backend/src/'+x for x in ['search_trace.h','search_trace.cpp','recorded_ompl_manager.cpp','SEARCH_TRACE_SOURCES.md','search_trace_test.cpp','worker.cpp']],
 'src/unloading_sim/single_contact.py','tools/plan_m710_approach_contact.py']
def git(*a):return subprocess.check_output(['git',*a],cwd=W,text=True).strip()
git('diff','--check');git('add','--sparse',*paths);git('diff','--cached','--check')
parent=git('rev-parse','HEAD');assert parent=='a76025c2b879ee20c4cfb5d38892c45f40e84091'
tree=git('write-tree','--missing-ok')
sha=subprocess.check_output(['git','commit-tree',tree,'-p',parent,'-m','feat: record real RRTConnect search trees without changing attempt policy'],cwd=W,text=True).strip()
git('update-ref','refs/heads/feat/v0.5-moveit2-native-single-carton',sha,parent)
(D/'baseline-freeze.json').write_text(json.dumps(dict(source_commit=sha,worker_sha256=hashlib.sha256((B/'m710_moveit_worker').read_bytes()).hexdigest(),trace_library_sha256=hashlib.sha256((B/'libm710_search_trace.so').read_bytes()).hexdigest(),strategy='BASELINE_SINGLE_ATTEMPT',trace_enabled=True),indent=2))
(D/'run-baseline.sh').write_text('''#!/bin/bash
set -eu
source /opt/ros/humble/setup.bash
cd /work-single-contact-20261016
export PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1 M710_MOVEIT_ASSET_ROOT=/work-single-contact-20261016
export ROS_HOME=/tmp/search-trace-20261017/ros-baseline-c04
mkdir -p "$ROS_HOME"
taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python tools/plan_m710_approach_contact.py --through-preplace --search-trace --target carton_l07_c04 --initial-state /tmp/preplace-ik-20261017/c04-input.json --output /tmp/search-trace-20261017/baseline-c04 --worker-command /tmp/search-trace-20261017/worker.sh --worker-binary /tmp/search-trace-20261017/build/m710_moveit_worker --source-commit '''+sha+''' --seed 71070 --task-budget-s 300 --stage-budget-s 90 --ipc-timeout-s 100
''')
print('BASELINE_FROZEN',sha)

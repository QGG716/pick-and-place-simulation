import os,sys
from pathlib import Path
base=Path('/root/autodl-tmp/v05-acceptance/resident-geometry-ros-20260928')
os.chdir(base/'code')
sys.path[:0]=[str(base/'code'/p) for p in ('','src','packages/unloading_contracts/src','tools','tests')]
# Reuse installed distro pytest without putting distro NumPy/SciPy ahead of the GPU venv.
sys.path.append('/usr/lib/python3/dist-packages')
import numpy,scipy,cv2,pytest
print('Numeric modules:',numpy.__file__,scipy.__file__,cv2.__file__,flush=True)
assert all('/root/v05-gpu-venv/' in m.__file__ for m in (numpy,scipy,cv2))
raise SystemExit(pytest.main(['-q','tests/test_workcell_geometry_process.py','tests/test_metric_thread_policy.py',
 '--basetemp='+str(base/'native-tests')]))

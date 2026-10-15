cd /root/autodl-tmp/m710-moveit2-20260922/rootfs/work-native-reuse2-20261015
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python -m pytest -q tests/test_planning_only.py tests/test_moveit2_native_cold.py

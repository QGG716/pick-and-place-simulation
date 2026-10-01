#!/bin/bash
cd /root/autodl-tmp/m710-moveit2-20260922/repo
export PYTHONPATH=src M710_MOVEIT_ASSET_ROOT=/work M710_MOVEIT_COMMAND=/root/autodl-tmp/m710-moveit2-20260922/worker.sh M710_MOVEIT_LOG=/root/autodl-tmp/m710-moveit2-20260922/evidence/worker-final.log
P=/root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python
$P tools/run_m710_moveit_integration.py --case loaded_fixed_transit --output ../evidence/native-loaded-tight-goal.json
$P tools/run_m710_moveit_integration.py --suite task --backend moveit2 --output ../evidence/native-task.json
$P tools/run_m710_moveit_integration.py --suite task --backend core --output ../evidence/core-task.json

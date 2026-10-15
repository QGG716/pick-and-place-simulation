#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /work-native-reuse2-20261015
python3 tools/benchmark_m710_native_checks.py \
 --binary /tmp/native-reuse-20261015/build/m710_process_benchmark \
 --model docs/experiments/m710_native_reuse_20261015/model.json \
 --input docs/experiments/m710_native_reuse_20261015/fixed-input.json \
 --output docs/experiments/m710_native_reuse_20261015/formal \
 --source-sha eaac48743d5c446524ed7d758659f608c94bf244 --pairs 3 --cpu 0

#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
test ! -e "$base/rootfs/c01-ext100-v2-20261023"
nohup bash "$base/c01-ext100-v2-deploy.sh" > "$base/c01-ext100-v2-deploy.log" 2>&1 < /dev/null &
echo "deployment_pid=$!"

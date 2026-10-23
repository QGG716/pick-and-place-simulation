#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$base" bash /c01-direct-20261023/formal-v2.sh /c01-direct-20261023/production-check --check-only > "$base/c01-direct-20261023/production-check-v2.log" 2>&1
set +e
chroot "$base" bash /c01-direct-20261023/formal-v2.sh > "$base/c01-direct-20261023/formal-v1.log" 2>&1
code=$?
printf '%s\n' "$code" > "$base/c01-direct-20261023/formal-v1.exit"
cat "$base/c01-direct-20261023/formal-v1.log"
exit "$code"

#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test ! -e "$rootfs/work-population-x500-66604f4"
test ! -e "$rootfs/population-x500-20261022"
tar xzf "$base/population-x500-source.tar.gz" -C "$rootfs"
mkdir "$rootfs/population-x500-20261022"
cp "$base/population-x500-formal.sh" "$rootfs/population-x500-20261022/formal-v1.sh"
chroot "$rootfs" /bin/bash -c 'cd /work-population-x500-66604f4; PYTHONPATH=src:tools PYTHONNOUSERSITE=1 /opt/planning-venv/bin/python -m pytest tests/test_mount_comparison.py -q' > "$rootfs/population-x500-20261022/focused-tests.log" 2>&1
test "$(sha256sum "$rootfs/mount-build/m710_moveit_worker" | cut -d' ' -f1)" = 0bab47de68ae112583afa76a285b74dccd16ff29518d8e11e6d1ac6bfb77756a
test "$(sha256sum "$rootfs/mount-install/lib/libm710_search_trace.so" | cut -d' ' -f1)" = 1e612f9e38df55bf55ecaee3a7efd0151235e53db553360a1a4d9b00fee95892
set +e
chroot "$rootfs" /bin/bash /population-x500-20261022/formal-v1.sh > "$rootfs/population-x500-20261022/formal-v1.log" 2>&1
code=$?
printf '%s\n' "$code" > "$rootfs/population-x500-20261022/formal-v1.exit"
exit "$code"

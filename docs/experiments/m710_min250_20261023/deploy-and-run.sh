#!/bin/bash
# Run on the authorized GPU after uploading the three prepared files below.
# This script has NOT been run: the prior SSH endpoint refused TCP connections.
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022
rootfs="$base/rootfs"
test -d "$rootfs/opt/planning-venv"
test ! -e "$rootfs/work-min250-a300d23"
test ! -e "$rootfs/min250-20261023"
test "$(sha256sum "$rootfs/mount-build/m710_moveit_worker" | cut -d' ' -f1)" = 0bab47de68ae112583afa76a285b74dccd16ff29518d8e11e6d1ac6bfb77756a
test "$(sha256sum "$rootfs/mount-install/lib/libm710_search_trace.so" | cut -d' ' -f1)" = 1e612f9e38df55bf55ecaee3a7efd0151235e53db553360a1a4d9b00fee95892
tar xzf "$base/min250-prepared-source.tar.gz" -C "$rootfs"
tar xzf "$base/min250-previous-inputs.tar.gz" -C "$rootfs"
cp "$base/min250-formal-v1.sh" "$rootfs/min250-20261023/formal-v1.sh"
set +e
chroot "$rootfs" /bin/bash /min250-20261023/formal-v1.sh > "$rootfs/min250-20261023/formal-v1.log" 2>&1
code=$?
printf '%s\n' "$code" > "$rootfs/min250-20261023/formal-v1.exit"
exit "$code"

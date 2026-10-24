#!/bin/bash
set -euo pipefail
rootfs=/root/autodl-tmp/m710-l05-x500-20261022/rootfs
chroot "$rootfs" /opt/planning-venv/bin/python /c04-extension200-v2-20261024/export-view.py /c04-extension200-v2-20261024/side/formal-v1 /c04-extension200-v2-20261024/side/analysis-v1/side-ik.html --source-root /work-c04-extension200-20b3ca7 --overlay /c04-extension200-v2-20261024/selected_routes.js
chroot "$rootfs" /opt/planning-venv/bin/python /c04-extension200-v2-20261024/export-view.py /c04-extension200-v3-20261024/top/formal-v1 /c04-extension200-v3-20261024/top/analysis-v1/top-ik.html --source-root /work-c04-extension200-c88b5eb --overlay /c04-extension200-v2-20261024/selected_routes.js
tar czf /root/autodl-tmp/m710-l05-x500-20261022/c04-extension200-ik-views.tar.gz -C "$rootfs" c04-extension200-v2-20261024/side/analysis-v1/side-ik.html c04-extension200-v2-20261024/side/analysis-v1/side-ik-checks.json c04-extension200-v3-20261024/top/analysis-v1/top-ik.html c04-extension200-v3-20261024/top/analysis-v1/top-ik-checks.json

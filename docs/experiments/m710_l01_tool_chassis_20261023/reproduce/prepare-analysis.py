from pathlib import Path
out=Path('artifacts/m710_l01_tool_chassis_20261023')
text=Path('artifacts/m710_bottom_two_top_20261023/analyze.py').read_text(encoding='utf-8')
text=text.replace('/work-bottom-top-4337f91','/work-l01-tool-e9dfef3').replace('/bottom-two-top-20261023','/l01-tool-chassis-v2-20261023')
text=text.replace('robot_mount_contact_exemption=True)','robot_mount_contact_exemption=True,l01_tool_chassis_exemption=True)')
text=text.replace('/lowprofile-build','/l01-tool-v2-build').replace('/lowprofile-install','/l01-tool-v2-install')
text=text.replace("'native-build-reuse.json'","'build.log','native-policy-tests.log','native-binary-sha256.txt'")
(out/'analyze.py').write_text(text,encoding='utf-8',newline='\n')
script=Path('artifacts/m710_bottom_two_top_20261023/analyze.sh').read_text(encoding='utf-8')
script=script.replace('/work-bottom-top-4337f91','/work-l01-tool-e9dfef3').replace('/bottom-two-top-20261023','/l01-tool-chassis-v2-20261023')
script=script.replace('bottom-top-analysis.tar.gz','l01-tool-analysis.tar.gz')
script=script.replace('native-build-reuse.json','build.log native-policy-tests.log native-binary-sha256.txt')
script=script.replace('INSIDE\ntar', '''/opt/planning-venv/bin/python - <<'PY'
import json
from pathlib import Path
from export_m710_top_layer_replay import export
if json.loads(Path('formal-v1/summary.json').read_text())['completed_count']:
    export(Path('replay-v1'),71070,Path('analysis-v1/accepted-cycles.html'))
PY
INSIDE
tar''')
(out/'analyze.sh').write_text(script,encoding='utf-8',newline='\n')
(out/'status.sh').write_text('''#!/bin/bash
set -euo pipefail
base=/root/autodl-tmp/m710-l05-x500-20261022/rootfs/l01-tool-chassis-v2-20261023
for f in build.log native-policy-tests.log focused-tests.log production-check.log formal-v1.log formal-v1.exit; do
 if test -f "$base/$f"; then printf '%s\\n' "$f"; tail -4 "$base/$f"; fi
done
''',encoding='utf-8',newline='\n')

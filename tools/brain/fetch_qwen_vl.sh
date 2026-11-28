#!/bin/bash
# Runs on the GPU server: fetch Qwen2.5-VL-7B-Instruct from ModelScope with
# parallel ranged aria2c and verify each file against the SHA-256 that the
# ModelScope file API lists. Prints FETCH_OK only when every file matches.
set -euo pipefail
MODEL=${MODEL:-Qwen/Qwen2.5-VL-7B-Instruct}
DEST=${DEST:-/root/autodl-tmp/eco65-vlm/models/Qwen2.5-VL-7B-Instruct}
mkdir -p "$DEST"; cd "$DEST"
curl -sf -m 30 "https://www.modelscope.cn/api/v1/models/$MODEL/repo/files?Revision=master&Recursive=true" -o .files.json
python3 - "$MODEL" <<'PY' > .list.txt
import json, sys
for f in json.load(open('.files.json'))['Data']['Files']:
    if f['Type'] == 'blob' and f['Path'] not in ('README.md', '.gitattributes'):
        print(f['Path'], f['Sha256'])
PY
while read -r path sha; do
  if [ -f "$path" ] && echo "$sha  $path" | sha256sum -c --status; then echo "HAVE $path"; continue; fi
  aria2c -q -x 16 -s 16 -k 8M --file-allocation=none --allow-overwrite=true --auto-file-renaming=false \
    -o "$path" "https://www.modelscope.cn/models/$MODEL/resolve/master/$path"
  echo "$sha  $path" | sha256sum -c --status || { echo "SHA_MISMATCH $path"; exit 1; }
  echo "VERIFIED $path $sha"
done < .list.txt
echo FETCH_OK

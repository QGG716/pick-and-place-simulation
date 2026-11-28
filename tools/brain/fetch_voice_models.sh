#!/bin/bash
# Field PC: fetch the local CPU voice models (sherpa-onnx official releases)
# into the ignored outputs/brain/models and print the SHA-256 of every file
# the voice loop loads (record them in configs/brain/voice_models.sha256).
set -euo pipefail
cd "$(dirname "$0")/../.."
D=outputs/brain/models; mkdir -p "$D"; cd "$D"
R=https://github.com/k2-fsa/sherpa-onnx/releases/download
fetch() { [ -s "$2" ] || curl -fL --retry 3 -C - -o "$2.part" "$1" && mv -f "$2.part" "$2" 2>/dev/null || true; [ -s "$2" ]; }
fetch $R/asr-models/silero_vad.onnx silero_vad.onnx
fetch $R/asr-models/sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17.tar.bz2 sense-voice.tar.bz2
fetch $R/tts-models/vits-melo-tts-zh_en.tar.bz2 melo-tts.tar.bz2
[ -d sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17 ] || tar xjf sense-voice.tar.bz2
[ -d vits-melo-tts-zh_en ] || tar xjf melo-tts.tar.bz2
rm -f sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17/model.onnx   # fp32 copy unused (int8 is loaded)
rm -f vits-melo-tts-zh_en/model.int8.onnx   # the release ships only a 133-byte LFS pointer here; model.onnx is loaded
find silero_vad.onnx sherpa-onnx-sense-voice-zh-en-ja-ko-yue-2024-07-17 vits-melo-tts-zh_en -type f \
  \( -name '*.onnx' -o -name '*.txt' -o -name '*.fst' \) | sort | xargs sha256sum
echo VOICE_FETCH_OK

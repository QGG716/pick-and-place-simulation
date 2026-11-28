"""Local CPU voice loop: microphone -> VAD -> ASR, and TTS -> speaker (sherpa-onnx).

User decision 2026-11-28: ASR and TTS run on the field PC CPU. Audio goes
through the system's ALSA tools (`arecord` / `aplay` subprocesses), so no
system audio library is installed or changed.

Stop words: a transcript containing one of `stop_words` is a voice stop. It
is reported before anything else (no VLM round trip) and triggers the same
software stop as the stop button. A stop never releases a held load and does
not replace the hardware emergency stop; ASR has a fraction of a second of
latency after the operator stops speaking and can mishear.

While the assistant speaks, the microphone keeps listening but only stop
words are acted on (barge-in); the assistant's own speech never contains a
stop word (`speakable`), so it cannot stop the robot by hearing itself.
"""
import hashlib
from pathlib import Path
import re
import subprocess
import threading
import time

import numpy as np

SAMPLE_RATE = 16000
WINDOW = 512  # silero VAD window, 32 ms at 16 kHz
DEFAULT_STOP_WORDS = ('停止', '停下', '停车', '急停', '快停', '别动', 'stop')
# How a stop word is said by the assistant itself (never a stop word).
SPOKEN_REPLACEMENTS = {'停止': '终止', '停下': '停住', '停车': '驻车', '急停': '紧急按钮', '快停': '尽快终止',
                       '别动': '不要动', 'stop': '英文终止'}

_PUNCT = re.compile(r'[\s\W_]+', re.UNICODE)


def normalize(text):
    return _PUNCT.sub('', (text or '').lower())


def is_stop_command(text, stop_words=DEFAULT_STOP_WORDS):
    t = normalize(text)
    return any(normalize(w) in t for w in stop_words if normalize(w))


def speakable(text, stop_words=DEFAULT_STOP_WORDS):
    out = text or ''
    for word in sorted(stop_words, key=len, reverse=True):
        replacement = SPOKEN_REPLACEMENTS.get(word, '终止')
        out = re.sub(re.escape(word), replacement, out, flags=re.IGNORECASE)
    return out


def verify_files(base, hashes):
    """hashes: {relative path: sha256}; raises on a missing or different file."""
    bad = []
    for name, digest in sorted(hashes.items()):
        path = Path(base) / name
        if not path.is_file():
            bad.append(f'{name}: MISSING')
            continue
        h = hashlib.sha256()
        with open(path, 'rb') as f:
            for block in iter(lambda: f.read(8 << 20), b''):
                h.update(block)
        if h.hexdigest() != digest:
            bad.append(f'{name}: SHA256_MISMATCH')
    if bad:
        raise ValueError('VOICE_MODEL_FILES_DIFFER: ' + '; '.join(bad))


class SherpaASR:
    def __init__(self, base, settings):
        import sherpa_onnx
        a = settings['asr']
        self.recognizer = sherpa_onnx.OfflineRecognizer.from_sense_voice(
            model=str(Path(base) / a['model']), tokens=str(Path(base) / a['tokens']),
            num_threads=int(a.get('num_threads', 2)), language=a.get('language', 'zh'), use_itn=True)

    def transcribe(self, samples):
        stream = self.recognizer.create_stream()
        stream.accept_waveform(SAMPLE_RATE, samples)
        self.recognizer.decode_stream(stream)
        return stream.result.text.strip()


class SherpaVAD:
    def __init__(self, base, settings):
        import sherpa_onnx
        v = settings['vad']
        config = sherpa_onnx.VadModelConfig(
            silero_vad=sherpa_onnx.SileroVadModelConfig(
                model=str(Path(base) / v['model']), threshold=float(v.get('threshold', .5)),
                min_silence_duration=float(v.get('min_silence_s', .4)), min_speech_duration=float(v.get('min_speech_s', .2)),
                window_size=WINDOW, max_speech_duration=float(v.get('max_speech_s', 15.))),
            sample_rate=SAMPLE_RATE, num_threads=1)
        self.vad = sherpa_onnx.VoiceActivityDetector(config, buffer_size_in_seconds=30)

    def segments(self, chunk):
        self.vad.accept_waveform(chunk)
        out = []
        while not self.vad.empty():
            out.append(np.array(self.vad.front.samples, dtype=np.float32))
            self.vad.pop()
        return out


class SherpaTTS:
    def __init__(self, base, settings):
        import sherpa_onnx
        t = settings['tts']
        b = Path(base)
        vits = sherpa_onnx.OfflineTtsVitsModelConfig(model=str(b / t['model']), lexicon=str(b / t['lexicon']),
                                                     tokens=str(b / t['tokens']), dict_dir=str(b / t['dict_dir']))
        rules = ','.join(str(b / r) for r in t.get('rule_fsts', []))
        self.tts = sherpa_onnx.OfflineTts(sherpa_onnx.OfflineTtsConfig(
            model=sherpa_onnx.OfflineTtsModelConfig(vits=vits, num_threads=int(t.get('num_threads', 2))),
            rule_fsts=rules, max_num_sentences=1))
        self.speed = float(t.get('speed', 1.0))
        self.speaker = int(t.get('speaker_id', 0))

    def synthesize(self, text):
        audio = self.tts.generate(text, sid=self.speaker, speed=self.speed)
        return np.asarray(audio.samples, dtype=np.float32), int(audio.sample_rate)


class AlsaMicrophone:
    """16 kHz mono S16 from `arecord`; yields float32 chunks of WINDOW samples."""

    def __init__(self, device=None):
        self.device = device
        self.process = None

    def chunks(self, stop_event):
        command = ['arecord', '-q', '-t', 'raw', '-f', 'S16_LE', '-r', str(SAMPLE_RATE), '-c', '1']
        if self.device:
            command += ['-D', self.device]
        self.process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        try:
            while not stop_event.is_set():
                data = self.process.stdout.read(WINDOW * 2)
                if len(data) < WINDOW * 2:
                    err = self.process.stderr.read().decode(errors='replace')[-300:] if self.process.poll() is not None else ''
                    raise RuntimeError('MICROPHONE_STREAM_ENDED' + (f': {err}' if err else ''))
                yield np.frombuffer(data, dtype='<i2').astype(np.float32) / 32768.
        finally:
            self.close()

    def close(self):
        process, self.process = self.process, None
        if process is not None and process.poll() is None:
            process.terminate()
            try:
                process.wait(2)
            except subprocess.TimeoutExpired:
                process.kill()


class AlsaSpeaker:
    """Plays float32 samples through `aplay`; interruptible."""

    def __init__(self, device=None):
        self.device = device
        self.process = None
        self.lock = threading.Lock()

    def play(self, samples, sample_rate):
        pcm = (np.clip(samples, -1, 1) * 32767).astype('<i2').tobytes()
        command = ['aplay', '-q', '-t', 'raw', '-f', 'S16_LE', '-r', str(sample_rate), '-c', '1']
        if self.device:
            command += ['-D', self.device]
        with self.lock:
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=subprocess.DEVNULL)
            process = self.process
        try:
            process.stdin.write(pcm)
            process.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        process.wait()

    def interrupt(self):
        with self.lock:
            process = self.process
        if process is not None and process.poll() is None:
            process.kill()


class VoiceLoop:
    """Listener thread (mic -> VAD -> ASR -> on_text) and a speaker thread (say -> TTS -> play).

    on_text(text, is_stop, info) runs on the listener thread; it must be quick
    (stop signals only). While speaking, non-stop transcripts are dropped.
    """

    def __init__(self, settings, on_text, on_status, *, asr=None, vad=None, tts=None, microphone=None, speaker=None):
        self.settings = settings
        self.stop_words = tuple(settings.get('stop_words') or DEFAULT_STOP_WORDS)
        self.on_text = on_text
        self.on_status = on_status
        base = settings['model_root']
        if asr is None or vad is None or tts is None:
            if settings.get('model_hashes'):
                verify_files(base, settings['model_hashes'])
        self.asr = asr or SherpaASR(base, settings)
        self.vad = vad or SherpaVAD(base, settings)
        self.tts = tts or SherpaTTS(base, settings)
        self.microphone = microphone or AlsaMicrophone(settings.get('input_device'))
        self.speaker = speaker or AlsaSpeaker(settings.get('output_device'))
        self.stopped = threading.Event()
        self.listening = threading.Event()
        self.speaking = threading.Event()
        self.queue = []
        self.generation = 0
        self.queue_ready = threading.Condition()
        self.threads = []

    def start(self):
        for target, name in ((self._listen, 'voice-listen'), (self._speak, 'voice-speak')):
            t = threading.Thread(target=target, daemon=True, name=name)
            t.start()
            self.threads.append(t)

    def set_listening(self, on):
        (self.listening.set if on else self.listening.clear)()

    def say(self, text, *, interrupt=False):
        text = speakable(text, self.stop_words).strip()
        if not text:
            return
        sentences = [x.strip() for x in re.split(r'(?<=[。！？；!?;])', text) if x.strip()]
        with self.queue_ready:
            if interrupt:
                self.generation += 1  # pending and pre-synthesized sentences are dropped
                self.queue.clear()
                self.speaker.interrupt()
            self.queue.extend((self.generation, x) for x in sentences)
            self.queue_ready.notify()

    def stop_speaking(self):
        """Drops queued and pre-synthesized sentences and cuts the one playing."""
        with self.queue_ready:
            self.generation += 1
            self.queue.clear()
        self.speaker.interrupt()

    def _listen(self):
        while not self.stopped.is_set():
            if not self.listening.wait(.2):
                continue
            try:
                self.on_status('麦克风监听中')
                for chunk in self.microphone.chunks(self.stopped):
                    if not self.listening.is_set():
                        break
                    for segment in self.vad.segments(chunk):
                        started = time.monotonic()
                        text = self.asr.transcribe(segment)
                        if not text:
                            continue
                        stop = is_stop_command(text, self.stop_words)
                        info = dict(audio_s=len(segment) / SAMPLE_RATE, asr_s=time.monotonic() - started,
                                    while_speaking=self.speaking.is_set())
                        if stop:
                            self.speaker.interrupt()
                        elif self.speaking.is_set():
                            continue  # barge-in only for stop words
                        self.on_text(text, stop, info)
                self.microphone.close()
                if not self.stopped.is_set():
                    self.on_status('语音监听已关闭')
            except Exception as error:
                self.listening.clear()
                if self.stopped.is_set():
                    return
                self.on_status(f'麦克风错误，语音监听已关闭：{error}')

    def _next_sentence(self, wait):
        with self.queue_ready:
            while wait and not self.queue and not self.stopped.is_set():
                self.queue_ready.wait(.5)
            if self.stopped.is_set() or not self.queue:
                return None
            return self.queue.pop(0)

    def _current(self, item):
        with self.queue_ready:
            return item is not None and item[0] == self.generation

    def _speak(self):
        """Synthesizes the next sentence while the current one plays (first audio sooner)."""
        ready = None  # (generation, samples, rate)
        while not self.stopped.is_set():
            try:
                if ready is None:
                    item = self._next_sentence(True)
                    if item is None:
                        continue
                    ready = (item[0], *self.tts.synthesize(item[1]))
                if not self._current(ready):
                    ready = None
                else:
                    self.speaking.set()
                    player = threading.Thread(target=self.speaker.play, args=ready[1:], daemon=True)
                    player.start()
                    item = self._next_sentence(False)
                    ready = (item[0], *self.tts.synthesize(item[1])) if item is not None else None
                    player.join()
            except Exception as error:
                ready = None
                self.on_status(f'语音合成/播放失败：{error}')
            if ready is None:
                time.sleep(.25)  # room echo tail
                self.speaking.clear()

    def close(self):
        self.stopped.set()
        self.listening.clear()
        self.speaker.interrupt()
        self.microphone.close()
        with self.queue_ready:
            self.queue_ready.notify_all()

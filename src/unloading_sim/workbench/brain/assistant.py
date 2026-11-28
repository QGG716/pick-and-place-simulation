"""OperatorBrain: one VLM owner thread for site assessments and operator chat, plus the voice loop.

Speech playback is a separate switch (user 2026-11-28 "加入语音播报的开关，能调用本机的扬声器播放大模型的回复"):
replies, changed verdicts and the voice-stop acknowledgement are spoken through
the local speaker only while it is on; listening does not require it.

Advisory only (user decision 2026-11-28): results are published through
`notify(kind, value)` for display and speech. The only action the brain can
cause is a voice stop, which calls `stop_callback(text)` - the same signal as
the stop button. It cannot start, resume, plan, move or release anything.

Every assessment and chat turn is appended to
`<record_root>/<YYYYMMDD>/brain-<pid>.jsonl` with the image SHA-256 (the JPEG
is kept next to it), the prompt SHA-256, the raw reply and timings.
"""
from datetime import datetime
import json
import os
from pathlib import Path
import threading
import time

from . import AUTHORITY
from .assessment import (assessment_messages, chat_messages, clean_reply, parse_assessment, prompt_sha256,
                         SiteAssessment)


class OperatorBrain:
    def __init__(self, settings, *, vlm_factory, frame_provider, context_provider, stop_callback, notify,
                 voice_factory=None, clock=time.monotonic):
        self.settings = settings
        self.vlm_factory = vlm_factory
        self.frame_provider = frame_provider      # () -> (rgb uint8 HxWx3, rgb_number) or (None, None)
        self.context_provider = context_provider  # () -> dict, thread-safe snapshot
        self.stop_callback = stop_callback
        self.notify = notify
        self.voice_factory = voice_factory
        self.clock = clock
        a = settings.get('assessment') or {}
        self.auto = bool(a.get('auto', True))
        self.interval = float(a.get('interval_s', 15.))
        self.announce = a.get('announce', 'ON_CHANGE')
        v = settings['vlm']
        self.assess_tokens = int(v.get('assess_max_new_tokens', 320))
        self.chat_tokens = int(v.get('chat_max_new_tokens', 160))
        self.jpeg_quality = int(v.get('jpeg_quality', 90))
        self.vlm = None
        self.voice = None
        self.speak_enabled = bool((settings.get('voice') or {}).get('speak_default', False))
        self.state = 'IDLE'
        self.last_assessment = None
        self.last_assess_at = None
        self.history = []
        self.pending_chat = []
        self.pending_assess = None
        self.cv = threading.Condition()
        self.closed = False
        self.thread = None
        self.record_lock = threading.Lock()
        day = datetime.now().strftime('%Y%m%d')
        self.record_dir = Path(settings['record_root']) / day
        self.record_path = self.record_dir / f'brain-{os.getpid()}.jsonl'

    # -- lifecycle --------------------------------------------------------------
    def start(self):
        self.thread = threading.Thread(target=self._run, daemon=True, name='brain-vlm')
        self.thread.start()

    def start_voice(self):
        """Builds the local voice loop (model load takes a few seconds); returns error text or ''."""
        if self.voice is not None:
            return ''
        if self.voice_factory is None:
            return '语音未配置'
        try:
            self.voice = self.voice_factory(self._on_voice_text, lambda s: self.notify('voice_status', s))
            self.voice.start()
            self._record(kind='voice_started')
            return ''
        except Exception as error:
            self.voice = None
            self._record(kind='voice_failed', error=str(error))
            return f'语音启动失败：{error}'

    def set_listening(self, on):
        if self.voice is None and on:
            error = self.start_voice()
            if error:
                self.notify('voice_status', error)
                return False
        if self.voice is not None:
            self.voice.set_listening(on)
            if not on:
                self.notify('voice_status', '语音监听已关闭')
        return on

    def set_speaking(self, on):
        """Voice playback switch; loads the local voice models on first use. Returns the new state."""
        if on and self.voice is None:
            error = self.start_voice()
            if error:
                self.notify('voice_status', error)
                return False
        self.speak_enabled = bool(on)
        if not on and self.voice is not None:
            self.voice.stop_speaking()
        self._record(kind='speech_playback', on=bool(on))
        return self.speak_enabled

    def close(self):
        with self.cv:
            self.closed = True
            self.cv.notify_all()
        if self.voice is not None:
            self.voice.close()
        if self.vlm is not None:
            try:
                self.vlm.close()
            except Exception:
                pass

    # -- requests ----------------------------------------------------------------
    def request_assessment(self, reason='operator'):
        with self.cv:
            if self.pending_assess is None:
                self.pending_assess = reason
                self.cv.notify_all()

    def ask(self, text, source='typed'):
        text = (text or '').strip()
        if not text:
            return
        with self.cv:
            self.pending_chat.append((text, source))
            self.cv.notify_all()

    def tick(self):
        """Called periodically from the UI thread: schedules the automatic assessment."""
        if self.auto and self.state == 'READY':
            now = self.clock()
            if self.last_assess_at is None or now - self.last_assess_at >= self.interval:
                self.request_assessment('periodic')

    def say(self, text, interrupt=False):
        if self.voice is not None and self.speak_enabled:
            self.voice.say(text, interrupt=interrupt)

    # -- voice ------------------------------------------------------------------------
    def _on_voice_text(self, text, is_stop, info):
        if is_stop:
            stamp = time.time()
            try:
                self.stop_callback(text)
                outcome = 'STOP_SIGNALLED'
            except Exception as error:
                outcome = f'STOP_CALLBACK_FAILED: {error}'
            self._record(kind='voice_stop', text=text, outcome=outcome, asr=info, wall_time=stamp)
            self.notify('voice_stop', dict(text=text, outcome=outcome))
            self.say('已发送软件终止命令。吸住的箱子不会释放。请核对机械臂是否已经静止，必要时按硬件急停。', interrupt=True)
            return
        self.notify('heard', text)
        self.ask(text, source='voice')

    # -- worker ------------------------------------------------------------------------
    def _set_state(self, state, detail=''):
        self.state = state
        self.notify('state', dict(state=state, detail=detail))

    def _connect(self):
        self._set_state('CONNECTING', '正在连接 GPU 服务器、校验并加载 Qwen VLM（首次约 1–2 分钟）')
        try:
            self.vlm = self.vlm_factory()
            info = self.vlm.connect() if hasattr(self.vlm, 'connect') else {}
            info = info or {}
            self._record(kind='vlm_connected', info={k: info.get(k) for k in
                                                     ('gpu', 'torch', 'transformers', 'model_type', 'connect_s', 'verify_s', 'load_s', 'warm_up_s', 'file_hash_states')})
            self._set_state('READY', f"Qwen VLM 已就绪 · {info.get('gpu') or ''} · 连接 {info.get('connect_s', 0):.0f} s")
            return True
        except Exception as error:
            self.vlm = None
            self._record(kind='vlm_connect_failed', error=str(error))
            self._set_state('ERROR', f'VLM 连接失败：{error}')
            return False

    def _run(self):
        if not self._connect():
            # Retry only when the operator asks again (an assessment or a question).
            pass
        while True:
            with self.cv:
                while not self.closed and not self.pending_chat and self.pending_assess is None:
                    self.cv.wait(.5)
                if self.closed:
                    return
                job = ('chat', self.pending_chat.pop(0)) if self.pending_chat else ('assess', self.pending_assess)
                if job[0] == 'assess':
                    self.pending_assess = None
            if self.vlm is None and not self._connect():
                if job[0] == 'chat':
                    self.notify('reply', dict(question=job[1][0], text='视觉大模型没有连接上，暂时无法回答。', error=True))
                    self.say('视觉大模型没有连接上，暂时无法回答。')
                continue
            try:
                if job[0] == 'chat':
                    self._chat(*job[1])
                else:
                    self._assess(job[1])
            except Exception as error:
                self._record(kind=f'{job[0]}_failed', error=str(error))
                if job[0] == 'chat':
                    self.notify('reply', dict(question=job[1][0], text=f'回答失败：{error}', error=True))
                else:
                    self.last_assess_at = self.clock()
                    a = SiteAssessment('ERROR', error=str(error), stamp=time.time())
                    self._publish_assessment(a)
                if self.vlm is not None and getattr(self.vlm, 'process', True) is None:
                    self.vlm = None
                    self._set_state('ERROR', f'VLM 连接中断：{error}（下一次请求时重连）')

    def _frame(self):
        from .vlm_client import encode_jpeg
        rgb, number = self.frame_provider()
        if rgb is None:
            return None, None, None
        data, digest = encode_jpeg(rgb, self.jpeg_quality)
        try:
            images = self.record_dir / 'images'
            images.mkdir(parents=True, exist_ok=True)
            path = images / f'{digest}.jpg'
            if not path.exists():
                path.write_bytes(data)
        except OSError:
            pass
        return data, digest, number

    def _assess(self, reason):
        data, digest, number = self._frame()
        self.last_assess_at = self.clock()
        if data is None:
            self._publish_assessment(SiteAssessment('ERROR', error='没有实时相机画面', stamp=time.time()))
            return
        context = self.context_provider()
        messages = assessment_messages(context)
        reply = self.vlm.generate(messages, [data], self.assess_tokens)
        timing = {k: reply.get(k) for k in ('round_trip_s', 'inference_s', 'input_tokens', 'output_tokens')}
        a = parse_assessment(reply['text'], image_sha256=digest, rgb_number=number, stamp=time.time(), timing=timing)
        self._record(kind='assessment', reason=reason, prompt_sha256=prompt_sha256(messages), context=context, **a.record())
        self._publish_assessment(a)

    def _publish_assessment(self, a):
        previous = self.last_assessment
        self.last_assessment = a
        self.notify('assessment', a)
        changed = previous is None or (previous.level, previous.can_start, previous.collapse_risk, previous.people_in_area) != \
            (a.level, a.can_start, a.collapse_risk, a.people_in_area)
        if self.announce == 'ALWAYS' or (self.announce == 'ON_CHANGE' and changed and (a.level != 'OK' or previous is not None)):
            self.say(a.spoken())

    def _chat(self, question, source):
        data, digest, number = self._frame()
        context = self.context_provider()
        if self.last_assessment is not None:
            context = dict(context, last_vlm_assessment=dict(headline=self.last_assessment.headline(),
                                                             summary=self.last_assessment.summary,
                                                             age_s=round(time.time() - self.last_assessment.stamp, 1)))
        messages = chat_messages(self.history, question, context, data is not None)
        reply = self.vlm.generate(messages, [data] if data is not None else [], self.chat_tokens)
        text = clean_reply(reply['text'])
        self.history += [('user', question), ('assistant', text)]
        self.history = self.history[-16:]
        self._record(kind='chat', source=source, question=question, reply=text, raw_text=reply['text'],
                     image_sha256=digest, rgb_number=number, prompt_sha256=prompt_sha256(messages), context=context,
                     timing={k: reply.get(k) for k in ('round_trip_s', 'inference_s', 'input_tokens', 'output_tokens')})
        self.notify('reply', dict(question=question, text=text, source=source, round_trip_s=reply.get('round_trip_s')))
        self.say(text)

    def _record(self, **event):
        event = dict(event, wall_time=event.get('wall_time', time.time()), authority=AUTHORITY)
        line = json.dumps(event, ensure_ascii=False, default=str)
        with self.record_lock:
            try:
                self.record_dir.mkdir(parents=True, exist_ok=True)
                with open(self.record_path, 'a', encoding='utf-8') as f:
                    f.write(line + '\n')
            except OSError as error:
                self.notify('voice_status', f'助手记录写入失败：{error}')

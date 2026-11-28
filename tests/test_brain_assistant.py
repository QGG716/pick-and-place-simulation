"""Operator assistant: assessment parsing, advisory authority, chat, voice stop and the voice loop (all fakes)."""
import json
from pathlib import Path
import threading
import time

import numpy as np
import pytest

from unloading_sim.workbench.brain import AUTHORITY, settings as brain_settings
from unloading_sim.workbench.brain.assessment import parse_assessment, assessment_messages, chat_messages, extract_json
from unloading_sim.workbench.brain.assistant import OperatorBrain
from unloading_sim.workbench.brain.voice import (DEFAULT_STOP_WORDS, SPOKEN_REPLACEMENTS, VoiceLoop, is_stop_command,
                                                  speakable)

ROOT = Path(__file__).resolve().parents[1]
GOOD = ('```json\n{"can_start": "YES", "collapse_risk": "LOW", "people_in_area": false, '
        '"hazards": [], "reasons": ["箱堆整齐"], "summary": "可以开始"}\n```')


# -- assessment --------------------------------------------------------------------
def test_valid_reply_is_parsed_and_advisory_only():
    a = parse_assessment(GOOD)
    assert a.status == 'OK' and a.level == 'OK' and a.authority == AUTHORITY == 'ADVISORY_ONLY_NO_GATE'
    assert a.reasons == ('箱堆整齐',) and a.summary == '可以开始'


@pytest.mark.parametrize('reply,level', [
    ('{"can_start": "NO", "collapse_risk": "LOW", "people_in_area": false}', 'ALERT'),
    ('{"can_start": "YES", "collapse_risk": "HIGH", "people_in_area": false}', 'ALERT'),
    ('{"can_start": "YES", "collapse_risk": "LOW", "people_in_area": true}', 'ALERT'),
    ('{"can_start": "YES", "collapse_risk": "MEDIUM", "people_in_area": null}', 'CAUTION'),
    ('{"can_start": "UNCERTAIN", "collapse_risk": "LOW", "people_in_area": false}', 'CAUTION'),
])
def test_levels(reply, level):
    assert parse_assessment(reply).level == level


@pytest.mark.parametrize('reply', ['看起来没问题，可以开始。', '{"can_start": "MAYBE", "collapse_risk": "LOW"}',
                                   '{"can_start": "YES", "collapse_risk": "LOW", "people_in_area": "no"}', '', '[1, 2]'])
def test_unparseable_reply_is_invalid_never_safe(reply):
    a = parse_assessment(reply)
    assert a.status == 'INVALID_OUTPUT' and a.level == 'CAUTION' and '失败' in a.spoken()


def test_json_extraction_skips_braces_in_prose():
    assert extract_json('先说明 {不是JSON} 然后 {"a": {"b": 1}} 结束') == {'a': {'b': 1}}


def test_messages_carry_one_image_and_bounded_history():
    m = assessment_messages(dict(job_state='IDLE'))
    assert sum(p['type'] == 'image' for msg in m for p in msg['content']) == 1
    history = [('user', f'q{i}') if i % 2 == 0 else ('assistant', f'a{i}') for i in range(30)]
    c = chat_messages(history, '现在能开始吗', {}, True)
    assert len(c) == 1 + 8 + 1 and sum(p['type'] == 'image' for msg in c for p in msg['content']) == 1


# -- voice words -----------------------------------------------------------------------
@pytest.mark.parametrize('text', ['停止', '快停下来！', '停车停车', 'STOP!', '别动', '急停'])
def test_stop_words(text):
    assert is_stop_command(text)


@pytest.mark.parametrize('text', ['机械臂现在什么状态', '箱堆稳不稳', '开始作业'])
def test_not_stop_words(text):
    assert not is_stop_command(text)


def test_assistant_speech_never_contains_a_stop_word():
    for word in DEFAULT_STOP_WORDS:
        assert not is_stop_command(speakable(f'请说{word}，或者按{word}'))
    for spoken in SPOKEN_REPLACEMENTS.values():
        assert not is_stop_command(spoken)


# -- voice loop with fakes ----------------------------------------------------------------
class FakeMic:
    def __init__(self, n):
        self.n = n

    def chunks(self, stop_event):
        for i in range(self.n):
            if stop_event.is_set():
                return
            yield np.full(512, i, np.float32)
            time.sleep(.005)
        while not stop_event.is_set():
            time.sleep(.01)
            yield np.zeros(512, np.float32)

    def close(self):
        pass


class FakeVAD:
    def __init__(self, at):
        self.at = at

    def segments(self, chunk):
        v = int(chunk[0])
        return [np.full(1600, v, np.float32)] if v in self.at else []


class FakeASR:
    def __init__(self, words):
        self.words = words

    def transcribe(self, seg):
        return self.words[int(seg[0])]


class FakeTTS:
    def synthesize(self, text):
        return np.zeros(10, np.float32), 16000


class FakeSpeaker:
    def __init__(self, delay=0.):
        self.played, self.interrupts, self.delay = [], 0, delay

    def play(self, samples, rate):
        self.played.append(len(samples))
        time.sleep(self.delay)

    def interrupt(self):
        self.interrupts += 1


def loop(words, speaker=None, speaking=False):
    heard = []
    v = VoiceLoop(dict(model_root='/nonexistent'), lambda t, s, i: heard.append((t, s)), lambda s: None,
                  asr=FakeASR(words), vad=FakeVAD(set(words)), tts=FakeTTS(), microphone=FakeMic(max(words) + 2),
                  speaker=speaker or FakeSpeaker())
    if speaking:
        v.speaking.set()
    return v, heard


def wait(cond, timeout=3.):
    end = time.time() + timeout
    while time.time() < end and not cond():
        time.sleep(.01)
    return cond()


def test_voice_loop_reports_questions_and_stops():
    v, heard = loop({1: '箱堆稳吗', 3: '停止'})
    v.start()
    v.set_listening(True)
    assert wait(lambda: len(heard) == 2)
    v.close()
    assert heard == [('箱堆稳吗', False), ('停止', True)] and v.speaker.interrupts >= 1


def test_while_speaking_only_stop_words_pass():
    v, heard = loop({1: '箱堆稳吗', 3: '快停下'}, speaking=True)
    v._speak = lambda: None  # keep the speaking flag set
    v.start()
    v.set_listening(True)
    assert wait(lambda: len(heard) == 1)
    time.sleep(.1)
    v.close()
    assert heard == [('快停下', True)]


def test_listening_off_hears_nothing():
    v, heard = loop({1: '停止'})
    v.start()
    time.sleep(.3)
    v.close()
    assert heard == []


def test_say_splits_sentences_and_interrupt_drops_pending():
    speaker = FakeSpeaker(delay=.05)
    v, _ = loop({1: 'x'}, speaker=speaker)
    v.say('第一句。第二句！第三句？')
    assert [t for _, t in v.queue] == ['第一句。', '第二句！', '第三句？']
    v.say('新的。', interrupt=True)
    assert [t for _, t in v.queue] == ['新的。'] and speaker.interrupts == 1
    v.start()
    assert wait(lambda: len(speaker.played) == 1 and not v.speaking.is_set())
    v.close()


# -- OperatorBrain with a fake VLM -------------------------------------------------------------
class FakeVLM:
    def __init__(self, replies):
        self.replies = list(replies)
        self.calls = []
        self.process = object()

    def connect(self):
        return dict(gpu='FAKE', connect_s=0.)

    def generate(self, messages, images, max_new_tokens):
        self.calls.append((messages, len(images)))
        return dict(text=self.replies.pop(0), round_trip_s=.1, inference_s=.05, input_tokens=5, output_tokens=5)

    def close(self):
        pass


def settings(tmp_path, **assessment):
    return dict(assessment=dict(dict(auto=False, interval_s=15), **assessment), vlm={}, record_root=str(tmp_path))


def brain(tmp_path, replies, frame=True, stops=None, **assessment):
    events = []
    vlm = FakeVLM(replies)
    b = OperatorBrain(settings(tmp_path, **assessment), vlm_factory=lambda: vlm,
                      frame_provider=lambda: (np.full((48, 64, 3), 100, np.uint8), 17) if frame else (None, None),
                      context_provider=lambda: dict(job_state='IDLE'),
                      stop_callback=(stops.append if stops is not None else lambda t: None),
                      notify=lambda k, v: events.append((k, v)))
    return b, vlm, events


def kinds(events, kind):
    return [v for k, v in events if k == kind]


def test_assessment_is_published_and_recorded(tmp_path):
    b, vlm, events = brain(tmp_path, [GOOD])
    b.start()
    assert wait(lambda: b.state == 'READY')
    b.request_assessment()
    assert wait(lambda: kinds(events, 'assessment'))
    b.close()
    a = kinds(events, 'assessment')[0]
    assert a.status == 'OK' and a.rgb_number == 17 and vlm.calls[0][1] == 1
    records = [json.loads(l) for l in b.record_path.read_text().splitlines()]
    rec = [r for r in records if r['kind'] == 'assessment'][0]
    assert rec['authority'] == AUTHORITY and rec['image_sha256'] == a.image_sha256 and rec['raw_text'] == GOOD
    assert (b.record_dir / 'images' / f'{a.image_sha256}.jpg').is_file()


def test_no_camera_frame_is_an_error_not_a_verdict(tmp_path):
    b, vlm, events = brain(tmp_path, [], frame=False)
    b.start()
    assert wait(lambda: b.state == 'READY')
    b.request_assessment()
    assert wait(lambda: kinds(events, 'assessment'))
    b.close()
    assert kinds(events, 'assessment')[0].status == 'ERROR' and vlm.calls == []


def test_chat_reply_uses_frame_and_last_assessment(tmp_path):
    b, vlm, events = brain(tmp_path, [GOOD, '**可以**开始。'])
    b.start()
    b.request_assessment()
    assert wait(lambda: kinds(events, 'assessment'))
    b.ask('现在能开始吗')
    assert wait(lambda: kinds(events, 'reply'))
    b.close()
    assert kinds(events, 'reply')[0]['text'] == '可以开始。'
    context = vlm.calls[1][0][-1]['content'][-1]['text']
    assert 'last_vlm_assessment' in context and vlm.calls[1][1] == 1


def test_voice_stop_signals_stop_without_any_vlm_call(tmp_path):
    stops = []
    b, vlm, events = brain(tmp_path, [], stops=stops)
    b._on_voice_text('快停下', True, dict(asr_s=.2))
    assert stops == ['快停下'] and vlm.calls == []
    assert kinds(events, 'voice_stop')[0]['outcome'] == 'STOP_SIGNALLED'
    rec = json.loads(b.record_path.read_text().splitlines()[-1])
    assert rec['kind'] == 'voice_stop' and rec['outcome'] == 'STOP_SIGNALLED'


def test_voice_question_goes_to_chat(tmp_path):
    b, vlm, events = brain(tmp_path, ['箱堆稳定。'])
    b.start()
    b._on_voice_text('箱堆稳吗', False, {})
    assert wait(lambda: kinds(events, 'reply'))
    b.close()
    assert kinds(events, 'heard') == ['箱堆稳吗'] and kinds(events, 'reply')[0]['source'] == 'voice'


def test_connect_failure_is_reported_and_retried_on_request(tmp_path):
    events = []
    attempts = []

    def factory():
        attempts.append(1)
        raise RuntimeError('ssh refused')
    b = OperatorBrain(settings(tmp_path), vlm_factory=factory, frame_provider=lambda: (None, None),
                      context_provider=dict, stop_callback=lambda t: None, notify=lambda k, v: events.append((k, v)))
    b.start()
    assert wait(lambda: b.state == 'ERROR')
    b.ask('你好')
    assert wait(lambda: kinds(events, 'reply'))
    b.close()
    assert len(attempts) == 2 and kinds(events, 'reply')[0]['error']


def test_periodic_assessment_follows_interval(tmp_path):
    now = [0.]
    b, vlm, events = brain(tmp_path, [GOOD, GOOD], auto=True)
    b.clock = lambda: now[0]
    b.state = 'READY'
    b.tick()
    assert b.pending_assess == 'periodic'
    b.pending_assess = None
    b.last_assess_at = 0.
    now[0] = 10.
    b.tick()
    assert b.pending_assess is None
    now[0] = 15.
    b.tick()
    assert b.pending_assess == 'periodic'


def test_committed_brain_config_is_advisory_and_loads():
    s, issues = brain_settings.load(ROOT / 'configs/brain/eco65_brain.json')
    assert s['authority'] == AUTHORITY and s['basis']
    assert not [i for i in issues if 'authority' in i or 'basis' in i or '哈希' in i]
    assert '停止' in s['voice']['stop_words']
    raw = (ROOT / 'configs/brain/eco65_brain.json').read_text() + (ROOT / 'configs/brain/remote_vlm.example.json').read_text()
    assert 'seetacloud' not in raw and '"password"' not in raw.lower()

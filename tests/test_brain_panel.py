"""Tk panel of the operator assistant with a fake brain (needs a display; run under xvfb-run)."""
import tkinter as tk

import pytest

from unloading_sim.workbench.brain.assessment import parse_assessment
from tkinter import font as tkfont

from unloading_sim.workbench.brain.panel import BrainPanel, LEVEL_COLOURS

SETTINGS = dict(enabled=True, assessment=dict(auto=True), voice=dict(enabled=False))


class FakeBrain:
    def __init__(self, notify):
        self.notify, self.started, self.asked, self.assess, self.auto, self.state = notify, False, [], 0, True, 'READY'

    def start(self):
        self.started = True

    def tick(self):
        pass

    def ask(self, text, source):
        self.asked.append((text, source))

    def request_assessment(self, reason):
        self.assess += 1

    def set_listening(self, on):
        return on

    def set_speaking(self, on):
        self.speaking = on
        return on

    def close(self):
        pass


@pytest.fixture
def root():
    try:
        r = tk.Tk()
    except tk.TclError:
        pytest.skip('no display')
    yield r
    r.destroy()


def make(root, start=True, settings=SETTINGS, issues=(), chat_parent=None):
    made, stops = [], []
    panel = BrainPanel(root, settings=settings, issues=list(issues), make_brain=lambda n: made.append(FakeBrain(n)) or made[-1],
                       on_voice_stop_ui=lambda: stops.append(1), start=start, chat_parent=chat_parent)
    panel.pack()
    return panel, made, stops


def pump(root, panel):
    root.update()
    panel._poll()
    root.update()


def test_assessment_colour_and_text(root):
    panel, made, _ = make(root)
    assert made[0].started
    made[0].notify('assessment', parse_assessment('{"can_start": "YES", "collapse_risk": "HIGH", "people_in_area": false, '
                                                  '"hazards": ["上层箱子外伸"], "summary": "有坍塌风险"}'))
    pump(root, panel)
    assert panel.verdict.cget('bg') == LEVEL_COLOURS['ALERT'][1]
    assert '坍塌风险高' in panel.verdict.cget('text') and '上层箱子外伸' in panel.detail.cget('text')


def test_voice_stop_updates_ui_and_typed_question_goes_to_brain(root):
    panel, made, stops = make(root)
    made[0].notify('voice_stop', dict(text='停止', outcome='STOP_SIGNALLED'))
    pump(root, panel)
    assert stops == [1] and '语音停止' in panel.log.get('1.0', 'end')
    panel.question.set('箱堆稳吗')
    panel._ask()
    assert made[0].asked == [('箱堆稳吗', 'typed')]


def test_preview_mode_and_disabled_config_never_create_a_brain(root):
    _, made, _ = make(root, start=False)
    assert made == []
    panel, made, _ = make(root, settings=None, issues=['未配置'])
    assert made == [] and '未启用' in panel.state_text.get()


def test_xd_bot_button_chat_in_its_own_tab_with_larger_font(root):
    other = tk.Frame(root)
    other.pack()
    panel, made, _ = make(root, chat_parent=other, settings=dict(SETTINGS, voice=dict(enabled=True)))
    assert panel.assess_button.cget('text') == 'XD-Bot'
    panel.assess_button.invoke()
    assert made[0].assess == 1
    assert panel.chat.master is other and panel.log.winfo_toplevel() is root
    base = tkfont.nametofont('TkDefaultFont', root=root).actual('size')
    size = tkfont.Font(root=root, font=panel.log.cget('font')).actual('size')
    assert abs(size) == abs(base) + 2


def test_speech_playback_toggle_reaches_the_brain(root):
    panel, made, _ = make(root, settings=dict(SETTINGS, voice=dict(enabled=True)))
    assert str(panel.speak_button.cget('state')) != 'disabled'
    panel.speak_button.invoke()
    assert made[0].speaking is True and '已打开' in panel.chat_state.get()
    panel.speak_button.invoke()
    assert made[0].speaking is False
    made[0].notify('voice_status', '语音合成/播放失败：no device')
    panel.speak.set(True)
    pump(root, panel)
    assert panel.speak.get() is False and made[0].speaking is False


def test_voice_controls_disabled_without_voice_config(root):
    panel, _, _ = make(root)
    assert str(panel.speak_button.cget('state')) == 'disabled' and str(panel.listen_button.cget('state')) == 'disabled'

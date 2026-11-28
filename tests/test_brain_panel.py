"""Tk panel of the operator assistant with a fake brain (needs a display; run under xvfb-run)."""
import tkinter as tk

import pytest

from unloading_sim.workbench.brain.assessment import parse_assessment
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


def make(root, start=True, settings=SETTINGS, issues=()):
    made, stops = [], []
    panel = BrainPanel(root, settings=settings, issues=list(issues), make_brain=lambda n: made.append(FakeBrain(n)) or made[-1],
                       on_voice_stop_ui=lambda: stops.append(1), start=start)
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

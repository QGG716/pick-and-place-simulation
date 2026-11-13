"""Review UI fixes: stop bar, button contrast, alarm banner, click target, 3-D preview,
refresh survival. Fake devices only; nothing opens hardware."""
from dataclasses import replace
import time
from tkinter import ttk
from types import SimpleNamespace
import numpy as np
import pytest
from PIL import ImageTk
from unloading_sim.workbench.contracts import DetectionResult
from unloading_sim.workbench.single_carton.geometry import corners
from test_single_carton_stable_ui import ui  # noqa: F401  (shared fake-device fixture)
from test_single_carton_observation import frame, observation


def luminance(root, color):
    r, g, b = (x/65535 for x in root.winfo_rgb(color))
    return .2126*r+.7152*g+.0722*b


def buttons(parent):
    found = []
    for child in parent.winfo_children():
        if isinstance(child, ttk.Button):
            found.append(child)
        found.extend(buttons(child))
    return found


def test_stop_is_one_red_top_bar_control_visible_on_every_tab_and_bound_to_escape(ui):
    w, errors = ui; root = w.root; calls = []
    w.manager.stop = lambda: calls.append('stop')
    assert w.stop_button.master is w.livebar and str(w.stop_button.cget('style')) == 'Stop.TButton'
    assert '不自动释放' in w.stop_button.cget('text') and 'Esc' in w.stop_button.cget('text')
    style = ttk.Style(root)
    red = root.winfo_rgb(style.lookup('Stop.TButton', 'background'))
    assert red[0] > 2*red[1] and red[0] > 2*red[2]
    # Exactly one stop control, so there is no second, different behaviour.
    assert [b for b in buttons(root) if '停止' in str(b.cget('text')) and '回放' not in str(b.cget('text'))
            and '检测' not in str(b.cget('text')) and '采集' not in str(b.cget('text'))] == [w.stop_button]
    for tab in w.tabs.tabs():
        w.tabs.select(tab); root.update()
        assert w.stop_button.winfo_ismapped()
    w.stop_button.invoke()
    assert calls == ['stop'] and '已请求软件停止' in w.details.get()
    assert root.bind_all('<Escape>')
    w.open_config(); root.update()
    editor = [x for x in root.winfo_children() if x.winfo_class() == 'Toplevel' and '现场配置' in x.title()][-1]
    def descendants(widget):
        for child in widget.winfo_children():
            yield child; yield from descendants(child)
    text = next(c for c in descendants(editor) if c.winfo_class() == 'Text')
    text.focus_force(); root.update()
    text.event_generate('<Escape>', when='now'); root.update()
    assert calls == ['stop', 'stop']
    editor.destroy()
    assert not errors


def test_disabled_buttons_are_darker_than_enabled_and_hover_stays_readable(ui):
    w, errors = ui; root = w.root; style = ttk.Style(root)
    enabled = luminance(root, style.lookup('TButton', 'background'))
    disabled = luminance(root, style.lookup('TButton', 'background', ['disabled']))
    active = luminance(root, style.lookup('TButton', 'background', ['active']))
    text = luminance(root, style.lookup('TButton', 'foreground'))
    disabled_text = luminance(root, style.lookup('TButton', 'foreground', ['disabled']))
    assert disabled < enabled and disabled_text < text
    assert active < .5 < text  # light text keeps contrast on hover
    assert w.execute_button.instate(['disabled'])
    assert not errors


def test_alarm_banner_only_for_unconfirmed_stop_or_held_idle_load(ui):
    w, errors = ui; root = w.root; m = w.manager; banner = w.alarm_banner
    packs = []; original = banner.pack
    banner.pack = lambda **k: (packs.append(k), original(**k))
    try:
        root.update(); w.update_banner()
        assert banner.winfo_manager() == ''
        # Complete stand-in: the manager's supervisor thread also inspects it.
        m.cycle = SimpleNamespace(state='STOP_UNCONFIRMED', active=False, reason='STOP_TIMEOUT', last_feedback=None,
                                  status=lambda: {}, stop=lambda *a, **k: None, tick=lambda: None)
        w.update_banner(); w.update_banner(); root.update_idletasks()
        assert banner.winfo_manager() == 'pack' and len(packs) == 1
        slaves = root.pack_slaves()
        assert slaves.index(banner) == slaves.index(w.livebar)+1 < slaves.index(w.tabs)
        text = banner.cget('text')
        assert '停止未确认' in text and '不是硬件急停' in text and '不会自动释放' in text and '现场' in text
        assert root.winfo_rgb(banner.cget('bg'))[0] > 2*root.winfo_rgb(banner.cget('bg'))[1]
        m.vacuum.holding = True; w.update_banner()
        assert '吸具可能仍持箱' in banner.cget('text') and len(packs) == 1
        m.cycle = None; m.busy = True  # an operation is running: no idle alarm
        w.update_banner(); assert banner.winfo_manager() == ''
        m.busy = False; w.update_banner()
        assert banner.winfo_manager() == 'pack' and '仍持箱' in banner.cget('text') and '不会自动释放' in banner.cget('text')
        m.vacuum.holding = False; w.update_banner(); assert banner.winfo_manager() == ''
        assert len(packs) == 2
    finally:
        m.cycle = None; m.vacuum.holding = False; m.busy = False
    assert not errors


def wide_observation(generation):
    # 4:1 frame, so the centered thumbnail always leaves a vertical offset.
    H, W = 200, 800
    f = replace(frame(), observation_id='frame-click', rgb=np.zeros((H, W, 3), np.uint8),
                depth=np.full((H, W), 500, np.uint16),
                intrinsics=dict(width=W, height=H, fx=400, fy=400, ppx=W/2, ppy=H/2, model='SYNTHETIC_PINHOLE', coeffs=[0]*5))
    masks = [np.zeros((H, W), bool) for _ in range(2)]
    masks[0][40:160, 40:300] = True; masks[1][40:160, 450:760] = True
    detections = tuple({'label': 'box', 'score': .9, 'bbox_xyxy': b, 'center_m': None, 'frame_id': 'camera_optical'}
                       for b in ([40, 40, 300, 160], [450, 40, 760, 160]))
    result = DetectionResult(f.observation_id, f.stamp.source, 1, detections, tuple(masks))
    pose = np.eye(4); pose[:3, 3] = [.2, 0, 1.]
    box = dict(id='box-1', detection_index=1, corners_m=corners(pose, [.1, .1, .1]).tolist())
    return replace(observation(f), generation=generation), result, [box], masks


def click_at(w, u, v):
    view = w._frozen_view; (W, H), (sw, sh) = view['source'], view['shown']
    ox = (w.frozen_image.winfo_width()-sw)/2; oy = (w.frozen_image.winfo_height()-sh)/2
    return ox+(u+.5)*sw/W, oy+(v+.5)*sh/H, ox, oy


def test_click_on_frozen_frame_maps_scale_and_offset_and_selects_only_recovered_box(ui):
    w, errors = ui; root = w.root; m = w.manager
    obs, result, boxes, masks = wide_observation(w.backend.observation_generation)
    m.observation, m.result, m.boxes, m.plan, m.phase = obs, result, boxes, None, 'TARGET_READY'
    root.update(); w.draw_observation(); root.update()
    view = w._frozen_view
    assert view['source'] == (800, 200) and view['shown'][0] < 800  # thumbnail really scaled
    x, y, ox, oy = click_at(w, 600, 100)
    assert oy > 5  # centered label offset is present and must be undone
    w.frozen_image.event_generate('<Button-1>', x=int(round(x)), y=int(round(y))); root.update()
    assert w.target.get() == '1' and '已选择目标：检测编号 1' in w.notice.get()
    # Highlight: chosen mask tinted orange, the other blue.
    shown = np.asarray(ImageTk.getimage(w._photos['frozen']))[:, :, :3].astype(int)
    sx, sy = view['shown'][0]/800, view['shown'][1]/200
    chosen = shown[int(100*sy), int(600*sx)]; other = shown[int(100*sy), int(170*sx)]
    assert chosen[0] > chosen[2] and other[2] > other[0]
    x, y, *_ = click_at(w, 170, 100)
    w.frozen_image.event_generate('<Button-1>', x=int(round(x)), y=int(round(y))); root.update()
    assert w.target.get() == '1' and '未恢复为可信箱体' in w.notice.get()
    x, y, *_ = click_at(w, 380, 100)
    w.frozen_image.event_generate('<Button-1>', x=int(round(x)), y=int(round(y))); root.update()
    assert '没有检测到箱体' in w.notice.get()
    w.frozen_image.event_generate('<Button-1>', x=int(round(x)), y=max(0, int(oy)-3)); root.update()
    assert '不在冻结帧图像内' in w.notice.get()
    m.phase = 'PLANNING'
    with pytest.raises(ValueError, match='目标就绪后'):
        w.pick_target(SimpleNamespace(x=x, y=y))
    m.phase = 'TARGET_READY'; m.observation = replace(obs, generation=obs.generation-1)
    with pytest.raises(ValueError, match='观测会话已失效'):
        w.pick_target(SimpleNamespace(x=x, y=y))
    m.observation = None; m.phase = 'IDLE'
    assert not errors


def preview_plan():
    segments = []
    for k, stage in enumerate(('approach', 'contact', 'detach', 'return')):
        q = np.linspace(k*.1, (k+1)*.1 if k < 3 else 0., 1000)
        points = [dict(t=i*.005, q=[float(x), .2, .3, 0., 0., 0.]) for i, x in enumerate(q)]
        segments.append(dict(stage=stage, package=dict(prepared=dict(points=points, send_duration_s=5., time_scale=1.))))
    return dict(segments=segments, target=dict(id='box-1', detection_index=1))


def test_preview_has_rotatable_orthographic_3d_tcp_view_with_boxes_and_axes(ui):
    w, errors = ui; root = w.root; m = w.manager; c = w.canvas
    def tcp(q):
        T = np.eye(4); T[:3, 3] = np.asarray(q)[:3]; return T
    w.model = SimpleNamespace(tcp=tcp)
    target = np.eye(4); target[:3, 3] = [.2, .2, .3]
    neighbor = np.eye(4); neighbor[:3, 3] = [.2, .5, .3]
    m.boxes = [dict(id='box-1', detection_index=1, corners_m=corners(target, [.1, .1, .1]).tolist()),
               dict(id='box-2', detection_index=2, corners_m=corners(neighbor, [.1, .1, .1]).tolist())]
    m.plan = preview_plan(); m.phase = 'PLAN_READY'
    m.observation = replace(observation(), generation=w.backend.observation_generation)
    root.update(); w.preview(); root.update()
    assert w.preview_mode == 'joints' and len(c.find_all()) > 0
    w.view_button.invoke(); root.update()
    assert w.preview_mode == '3d' and w.view_button.cget('text') == '切换为关节曲线'
    texts = [c.itemcget(i, 'text') for i in c.find_all() if c.type(i) == 'text']
    assert any('不是机器人反馈' in t for t in texts)
    assert {'approach', 'contact', 'detach', 'return', 'X', 'Y', 'Z', 'q_start'} <= set(texts)
    lines = [i for i in c.find_all() if c.type(i) == 'line']
    paths = {color: [i for i in lines if c.itemcget(i, 'fill') == color] for color in w.SEGMENT_COLORS}
    assert all(len(v) == 1 and len(c.coords(v[0])) <= 2*202 for v in paths.values())
    assert len([i for i in lines if c.itemcget(i, 'fill') == '#ffc814']) == 12  # selected target envelope
    assert len([i for i in lines if c.itemcget(i, 'fill') == '#5f8a74']) == 12  # other box
    before = c.coords(paths[w.SEGMENT_COLORS[0]][0])
    assert c.bind('<B1-Motion>') and c.bind('<ButtonPress-1>')
    w._drag_start(SimpleNamespace(x=100, y=100)); w._drag_move(SimpleNamespace(x=180, y=400))
    assert abs(w.view_angles[1]) < np.pi/2
    after = [i for i in c.find_all() if c.type(i) == 'line' and c.itemcget(i, 'fill') == w.SEGMENT_COLORS[0]]
    assert c.coords(after[0]) != before
    m.plan = dict(m.plan); w.render_preview()
    assert any('已过期' in c.itemcget(i, 'text') for i in c.find_all() if c.type(i) == 'text')
    w.view_button.invoke(); assert w.preview_mode == 'joints'
    m.plan = None; m.phase = 'IDLE'; m.observation = None
    assert not errors


def test_refresh_loops_survive_exceptions_and_never_reschedule_after_close(ui):
    w, errors = ui; root = w.root
    def cancel_all():
        for ident in root.tk.call('after', 'info'): root.after_cancel(ident)
    def boom(): raise RuntimeError('vision snapshot boom')
    original = w.live_vision.snapshot; w.live_vision.snapshot = boom
    cancel_all(); before = len(w.backend.logs)
    w.update_task(); w.update_task()
    assert 'vision snapshot boom' in w.notice.get()
    assert len(w.backend.logs) == before+1  # repeated identical error logged once
    assert len(root.tk.call('after', 'info')) == 2  # both calls rescheduled
    w.live_vision.snapshot = original; cancel_all(); w.update_task()
    assert w._task_refresh_error is None
    cancel_all(); w._closed = True; w.update_task()
    assert len(root.tk.call('after', 'info')) == 0
    w._closed = False
    # Base observation refresh loop: periodic call survives; direct call still raises.
    def broken(): raise RuntimeError('sync boom')
    w._sync_observations = broken; cancel_all()
    w.refresh(); assert 'sync boom' in w.notice.get() and len(root.tk.call('after', 'info')) == 1
    with pytest.raises(RuntimeError, match='sync boom'):
        w.refresh(schedule=False)
    del w._sync_observations; cancel_all()
    # A failed background status read never latches feedback_pending.
    def unreadable(): raise ValueError('status read failed')
    w.service.process = object(); w.service.feedback = unreadable
    try:
        w.update_task(); assert w.feedback_pending
        end = time.monotonic()+2
        while w.jobs.qsize() < 2 and time.monotonic() < end: time.sleep(.01)
        w.service.process = None; w.update_task()  # process the error and the feedback result
        assert not w.feedback_pending and 'status read failed' in w.details.get()
    finally:
        w.service.process = None; cancel_all()
    assert not errors

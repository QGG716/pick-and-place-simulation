"""Operator panel for the TCP-based camera recalibration, on the
"坐标、TCP 与场景" tab (user request 2026-11-27).

The panel only reads: the latest RGB-D frame and the latest joint sample. The
operator moves the robot with the teach pendant; nothing here commands motion
or writes valves. A sample is stored only when the joints did not move by
more than ``THRESHOLDS['stationary_deg']`` between a reading before and a
reading after a frame received after the click.
"""
from __future__ import annotations

import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, ttk

import cv2
import numpy as np
from PIL import Image, ImageTk

from . import tcp_calibration as tc

# Field 2026-11-27: the camera looks ~90 deg away from the arm plane and turns
# with J1, so it never sees the tool: the touch method is the default.
MODE_LABELS = {'TIP_TOUCH': '标定板固定在场地 + TCP 尖端触点（推荐）', 'BOARD_ON_TOOL': '标定板固定在吸具上（仅当相机能看到吸具）'}
AUTO_TIP = ('', '自动', 'auto', 'AUTO')
GENERATED_BOARDS = {
    'A3 打印板 5×3 · AprilTag 36h11 · 60 mm': dict(rows=3, cols=5, marker_m=.06, gap_m=.015, border_bits=1),
    'A4 打印板 4×3 · AprilTag 36h11 · 45 mm': dict(rows=3, cols=4, marker_m=.045, gap_m=.01, border_bits=1),
}
STATIONARY_WAIT_MS = 400
GUIDE = {
    'TIP_TOUCH': (
        '步骤（本界面只读图像和关节角，不发任何运动或阀门指令；机械臂用示教器点动，触碰时慢速）：\n'
        '0. 在吸具上牢固装一个尖端（螺钉、钢针、削尖的硬杆都行，要比吸盘更突出，触碰时吸盘不能先碰到板），整个会话中不能松动。'
        '尖端坐标保持“自动”即可，程序会求出来；已知准确值时也可以填 link_6 下的坐标（m）。\n'
        '1. 标定板固定在机械臂够得到、且 J1 转回观测位时相机能看到的地方（例如箱垛位置，离相机 0.7–1.3 m），整个板位期间不能动。点“新建会话”。\n'
        '2. 尖端标定（四点法）：选一个板中间的角点（如 tag12_c2），用尖端轻触它，点“记录触点”；'
        '保持尖端在同一点，把吸具朝前后左右各倾斜 20–30° 再各记一次，共 5 次左右。下面会显示姿态分散度，需 ≥ 10°。\n'
        '3. 再轻触至少 5 个其他角点（分散到板的四个角和边上），每个记一次。\n'
        '4. 尖端离开板，把 J1 转回让相机看到整块板（左侧预览要有绿框），在 3 个以上不同 J1（相差约 5°）下点“采集当前姿态”。\n'
        '5. 建议再做一个板位：把板换个位置或角度，点“换板位置”，重复 3–4（第二个板位不用再做第 2 步）。\n'
        '6. 点“求解”，全部检查通过后点“生成使用新外参的现场配置”，用新配置重启程序生效。'),
    'BOARD_ON_TOOL': (
        '步骤（本界面只读取图像和关节角，不发任何运动或阀门指令；机械臂用示教器点动）：\n'
        '仅适用于相机能看到吸具的安装；现场相机与手臂相差约 90° 且随 J1 转动，看不到吸具上的板。\n'
        '1. 把标定板刚性固定在吸具上（胶带/夹具均可，位置不限，算法会一并求出板相对 TCP 的位姿；'
        '板面要平、不能晃）。选好标定板，点“新建会话”。\n'
        '2. 用示教器把板移到相机前 0.6–1.3 m，整块板尽量都在画面里（左侧预览绿色框=已识别的标签）。\n'
        '3. 停稳后点“采集当前姿态”，界面会确认前后关节角不变才保存。\n'
        '4. 换姿态重复，共 12–20 个：板面要绕不同方向倾斜 ±15–35°（上下俯仰、左右偏转、绕板面法向旋转都要有），'
        '远近和画面左右上下也要分散。只平移不转动的姿态没有用。\n'
        '5. 点“求解”。全部检查通过才会生成可用外参；再点“生成使用新外参的现场配置”，用新配置重启程序生效。'),
}


class CalibrationPanel:
    def __init__(self, parent, root, backend, model, config, calibration, log, *, before=None):
        self.root, self.backend, self.model, self.config = root, backend, model, config
        self.calibration, self.log = calibration, log
        self.session = None; self.result = None; self.result_path = None
        self.busy = False; self._preview = None; self._shown_frame = None
        self.record_root = Path(config.get('record_root') or 'outputs/single_carton')
        box = self.frame = ttk.LabelFrame(parent, text='相机外参重新标定（基于 TCP · 只读 · 示教器点动）', padding=8)
        box.pack(fill='both', expand=True, pady=(8, 4), **({'before': before} if before is not None else {}))
        left = ttk.Frame(box); left.pack(side='left', fill='y')
        self.canvas = tk.Label(left, bg='#101820', width=480, height=360, text='相机画面（打开本标签页时识别标定板）',
                               fg='#8aa4b8', compound='center')
        self.canvas.pack()
        self.preview_text = tk.StringVar(value='')
        ttk.Label(left, textvariable=self.preview_text, width=1, font=('DejaVu Sans Mono', 9)).pack(fill='x')
        right = ttk.Frame(box); right.pack(side='left', fill='both', expand=True, padx=(10, 0))
        row = ttk.Frame(right); row.pack(fill='x')
        ttk.Label(row, text='方式').pack(side='left')
        self.mode = tk.StringVar(value=MODE_LABELS['TIP_TOUCH'])
        ttk.Combobox(row, textvariable=self.mode, values=list(MODE_LABELS.values()), state='readonly', width=38).pack(side='left', padx=4)
        ttk.Label(row, text='标定板').pack(side='left', padx=(8, 0))
        self.boards = self._boards()
        self.board_name = tk.StringVar(value=next(iter(self.boards)))
        self.board_combo = ttk.Combobox(row, textvariable=self.board_name, values=list(self.boards), state='readonly', width=34)
        self.board_combo.pack(side='left', padx=4)
        self._button(row, '加载标定板 JSON…', self.load_board)
        self._button(row, '导出打印图', self.export_board)
        row = ttk.Frame(right); row.pack(fill='x', pady=(6, 0))
        ttk.Label(row, text='尖端（link_6，m）').pack(side='left')
        self.tip = tk.StringVar(value='自动')
        ttk.Entry(row, textvariable=self.tip, width=16).pack(side='left', padx=4)
        ttk.Label(row, text='触点角点').pack(side='left', padx=(8, 0))
        self.corner = tk.StringVar()
        self.corner_combo = ttk.Combobox(row, textvariable=self.corner, state='readonly', width=12)
        self.corner_combo.pack(side='left', padx=4)
        self._button(row, '角点示意图', self.corner_map)
        self._button(row, '记录触点', self.capture_touch)
        self._button(row, '换板位置', self.next_placement)
        self.place_text = tk.StringVar(value='')
        ttk.Label(row, textvariable=self.place_text).pack(side='left', padx=6)
        row = ttk.Frame(right); row.pack(fill='x', pady=6)
        for text, fn in (('新建会话', self.new_session), ('采集当前姿态', self.capture_photo), ('删除最后一条', self.drop_last),
                         ('求解', self.solve), ('生成使用新外参的现场配置', self.write_config)):
            self._button(row, text, fn)
        cols = ('n', 'kind', 'tags', 'q', 'rms')
        self.tree = ttk.Treeview(right, columns=cols, show='headings', height=6)
        for c, (t, w) in zip(cols, (('#', 40), ('类型', 70), ('标签', 50), ('关节角 J1..J6（度）', 330), ('残差 px', 80))):
            self.tree.heading(c, text=t); self.tree.column(c, width=w, stretch=c == 'q')
        self.tree.pack(fill='x')
        self.text = tk.Text(right, height=10, bg='#0d1720', fg='#d8e6f0', font=('DejaVu Sans Mono', 9), wrap='word')
        self.text.pack(fill='both', expand=True, pady=(6, 0))
        self.mode.trace_add('write', lambda *_: self.show(GUIDE[self.mode_key()]))
        self.board_name.trace_add('write', lambda *_: self._corner_names())
        self._corner_names()
        self.show(GUIDE['TIP_TOUCH'])

    # ---------------------------------------------------------------- helpers
    def _button(self, parent, text, fn):
        b = ttk.Button(parent, text=text, command=lambda: self.safe(fn))
        b.pack(side='left', padx=3)
        return b

    def safe(self, fn):
        try:
            fn()
        except Exception as exc:
            self.show(f'错误：{exc}', append=True)
            self.log(f'CALIBRATION ERROR {type(exc).__name__}: {exc}')

    def show(self, text, append=False):
        if not append:
            self.text.delete('1.0', 'end')
        self.text.insert('end', text+'\n'); self.text.see('end')

    def mode_key(self):
        return next(k for k, v in MODE_LABELS.items() if v == self.mode.get())

    def _boards(self):
        boards = {}
        ref = getattr(self.calibration, 'raw', None) or {}
        settings = ref.get('settings') or {}
        if settings.get('board_type') == 'aprilgrid':
            boards['现场 AprilGrid 5×5 · 80 mm（上次标定用板）'] = tc.Board.from_extrinsics_settings(settings)
        for name, g in GENERATED_BOARDS.items():
            boards[name] = tc.Board.grid(g['rows'], g['cols'], g['marker_m'], g['gap_m'], border_bits=g['border_bits'], name=name)
        return boards

    def board(self):
        return self.session.board if self.session else self.boards[self.board_name.get()]

    def _corner_names(self):
        names = self.boards[self.board_name.get()].corner_names()
        self.corner_combo.configure(values=names)
        if self.corner.get() not in names:
            self.corner.set(names[0])

    def load_board(self):
        path = filedialog.askopenfilename(filetypes=[('JSON', '*.json')])
        if path:
            b = tc.Board.from_json(path)
            self.boards[b.name] = b; self.board_combo.configure(values=list(self.boards)); self.board_name.set(b.name)

    def export_board(self):
        b = self.boards[self.board_name.get()]
        out = self.record_root/'calibration'/'boards'; out.mkdir(parents=True, exist_ok=True)
        stem = ''.join(ch if ch.isalnum() else '_' for ch in b.name)[:60]
        cv2.imwrite(str(out/f'{stem}_300dpi.png'), b.printable(300))
        (out/f'{stem}.json').write_text(__import__('json').dumps(b.to_dict(), ensure_ascii=False, indent=1))
        self.show(f'已导出 {out/(stem+"_300dpi.png")}（300 dpi，按 100% 原尺寸打印，不要缩放）。\n'
                  f'打印后用尺子量下方 100 mm 比例尺和标签黑框边长；与设定不符就不要用（或改 JSON 里的尺寸后加载）。', append=True)

    # ---------------------------------------------------------------- session
    def new_session(self):
        mode = self.mode_key(); board = self.boards[self.board_name.get()]
        tool = getattr(getattr(self.model, 'tool', None), 'matrix', None)
        tip = None
        if mode == 'TIP_TOUCH' and self.tip.get().strip() not in AUTO_TIP:
            tip = [float(x) for x in self.tip.get().replace('，', ',').split(',')]
        from ..ui import URDF_SHA
        self.session = tc.Session.create(self.record_root, mode, board, tool, URDF_SHA, tip,
                                         basis='USER_REQUEST_2026_11_27_RECALIBRATION_OPERATOR_JOGGED')
        self.result = self.result_path = None
        self.tree.delete(*self.tree.get_children())
        self.show(f'会话 {self.session.data["id"]} · {MODE_LABELS[mode]} · {board.name} · 尖端 '
                  f'{"自动求解" if tip is None else tip}\n目录 {self.session.folder}')
        self.place_text.set('板位 1')
        self.log('Calibration session '+self.session.data['id']+' '+mode)

    def _require_session(self):
        if self.session is None:
            raise ValueError('请先点“新建会话”')
        if self.busy:
            raise ValueError('上一次采集/求解还在进行')

    def _stationary(self, then):
        """Joint reading, then (after the wait) a new frame and another joint
        reading; ``then(frame, q, evidence)`` only when nothing moved."""
        j0 = self.backend.joint; f0 = self.backend.frame
        if j0 is None:
            raise ValueError('没有机械臂关节读数（机械臂未连接？）')
        clicked = time.monotonic_ns(); self.busy = True

        def check():
            self.busy = False
            j1 = self.backend.joint; frame = self.backend.frame
            try:
                if j1 is None or j1.sample_id == j0.sample_id:
                    raise ValueError('关节读数没有更新，未保存')
                if frame is None or (f0 is not None and frame.observation_id == f0.observation_id) \
                        or frame.stamp.received_monotonic_ns <= clicked:
                    raise ValueError('没有点击之后的新相机帧，未保存')
                moved = float(np.degrees(np.max(np.abs(np.asarray(j1.q_rad)-np.asarray(j0.q_rad)))))
                if moved > tc.THRESHOLDS['stationary_deg']:
                    raise ValueError(f'机械臂仍在运动（{moved:.3f}° > {tc.THRESHOLDS["stationary_deg"]}°），停稳后再采集')
                if j1.stamp.received_monotonic_ns <= frame.stamp.received_monotonic_ns:
                    raise ValueError('帧之后没有新的关节读数，未保存')
                evidence = dict(joint_before=j0.sample_id, joint_after=j1.sample_id, moved_deg=moved,
                                frame=frame.observation_id, frame_received_ns=frame.stamp.received_monotonic_ns,
                                joint_after_received_ns=j1.stamp.received_monotonic_ns, mapping=j1.mapping_id,
                                rule=f'max |q_after - q_before| <= {tc.THRESHOLDS["stationary_deg"]} deg around a post-click frame')
                then(frame, np.asarray(j1.q_rad, float), evidence)
            except Exception as exc:
                self.show(f'采集失败：{exc}', append=True)
        self.root.after(STATIONARY_WAIT_MS, check)

    def capture_photo(self):
        self._require_session()

        def store(frame, q, evidence):
            item = self.session.add_photo(frame.rgb, frame.depth, frame.depth_scale_m, dict(frame.intrinsics), q, evidence)
            n = len(self.session.data['photos'])
            self.tree.insert('', 'end', iid=f'p{n}', values=(n, f'照片 板位{item["placement"]}', len(item['tags']), self._q(q), ''))
            warn = '' if len(item['tags']) >= 4 else '（少于 4 个标签，求解时不会使用）'
            self.show(f'照片 {item["name"]}：{len(item["tags"])} 个标签{warn}', append=True)
        self._stationary(store)

    def capture_touch(self):
        self._require_session()
        if self.session.data['mode'] != 'TIP_TOUCH':
            raise ValueError('当前会话是“标定板固定在吸具上”，不需要触点')
        name = self.corner.get()

        def store(frame, q, evidence):
            item = self.session.add_touch(name, q, evidence)
            n = len(self.session.data['touches'])
            self.tree.insert('', 'end', iid=f't{n}', values=(n, f'触点 板位{item["placement"]} {name}', '', self._q(q), ''))
            same = [t for t in self.session.data['touches'] if t['corner'] == name and t.get('placement', 1) == item['placement']]
            note = ''
            if len(same) >= 2:
                spread = tc.rotation_spread_deg([self.model.frames(np.asarray(t['q_rad'], float))['link_6'] for t in same])
                note = (f'；此角点已触碰 {len(same)} 次，姿态分散度 {spread[1]:.1f}°' +
                        ('（尖端标定需 ≥ 3 次且 ≥ 10°）' if len(same) < 3 or spread[1] < tc.THRESHOLDS['min_pivot_spread_deg'] else '（够用）'))
            self.show(f'触点 {name} 已记录{note}', append=True)
        self._stationary(store)

    def next_placement(self):
        self._require_session()
        p = self.session.next_placement()
        self.place_text.set(f'板位 {p}')
        self.show(f'换到板位 {p}：先触碰本板位至少 6 次（≥ 4 个不同角点），再转 J1 拍照。', append=True)

    def _mark_corner(self, img, found, board, frame):
        name = self.corner.get()
        if self.mode_key() != 'TIP_TOUCH' or not name or not hasattr(frame, 'intrinsics'):
            return
        try:
            px = _corner_pixel(found, board, name, tc.camera_matrix(frame.intrinsics), tc.distortion(frame.intrinsics))
        except Exception:
            px = None
        if px is not None:
            centre = tuple(np.round(px).astype(int))
            cv2.circle(img, centre, 9, (255, 40, 40), 2)
            cv2.putText(img, name, (centre[0]+10, centre[1]-10), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 40, 40), 2)

    def corner_map(self):
        """The board seen from its printed face, the selected corner in red."""
        board = self.board(); name = self.corner.get()
        img = cv2.cvtColor(board.printable(40), cv2.COLOR_GRAY2RGB)
        for tag, C in board.corners.items():
            centre = board.print_px(C.mean(0), 40)
            cv2.putText(img, str(tag), tuple(np.round(centre+[-8, 5]).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, .5, (230, 120, 0), 2)
        if name:
            p = board.print_px(board.corner(name), 40)
            cv2.circle(img, tuple(np.round(p).astype(int)), 10, (220, 20, 20), 3)
            cv2.putText(img, name, tuple(np.round(p+[12, -12]).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, .6, (220, 20, 20), 2)
        top = tk.Toplevel(self.root); top.title(f'角点示意图 · {board.name}（从印刷面看）')
        self._map_image = ImageTk.PhotoImage(Image.fromarray(img))
        tk.Label(top, image=self._map_image).pack()
        tk.Label(top, text=f'红圈 = {name}。板在相机画面里时，左侧预览也会用红圈标出同一个角点，以画面为准。',
                 fg='#202020').pack(pady=4)

    def drop_last(self):
        self._require_session()
        kids = self.tree.get_children()
        if not kids:
            return
        last = kids[-1]
        self.session.drop_last('touches' if last.startswith('t') else 'photos')
        self.tree.delete(last)
        self.show('已删除最后一条', append=True)

    @staticmethod
    def _q(q):
        return ' '.join(f'{x:8.3f}' for x in np.degrees(q))

    # ------------------------------------------------------------------ solve
    def solve(self):
        self._require_session()
        self.busy = True; self.show('求解中…', append=True)
        reference = None
        if self.calibration is not None:
            reference = dict(matrix=np.asarray(self.calibration.matrix).tolist(), sha256=self.calibration.sha256,
                             path=self.config.get('camera_extrinsics'))
        box = {}

        def work():
            try:
                box['result'] = tc.solve_session(self.session, self.model.frames, reference)
            except Exception as exc:
                box['error'] = exc
        thread = threading.Thread(target=work, daemon=True); thread.start()

        def poll():
            if thread.is_alive():
                self.root.after(100, poll); return
            self.busy = False
            if 'error' in box:
                self.show(f'求解失败：{box["error"]}', append=True); return
            self.result_path, self.result = box['result']
            self.show(summary(self.result, self.result_path))
            for info in self.result.get('pnp', []):
                for iid in self.tree.get_children():
                    if self.tree.set(iid, 'n') == str(int(info['name'])) and iid.startswith('p'):
                        self.tree.set(iid, 'rms', f'{info["model_rms_px"]:.2f}')
        self.root.after(100, poll)

    def write_config(self):
        if self.result is None or not self.result.get('passed'):
            raise ValueError('还没有通过全部检查的标定结果')
        cfg = Path(self.config['config_path'])
        out = cfg.with_name(f'{cfg.stem}.calib-{self.session.data["id"]}.json')
        tc.field_config_with(cfg, self.result_path, out)
        self.show(f'已生成 {out}\n现在运行中的程序仍用旧外参；用这个配置重启程序后新外参才生效。', append=True)

    # ---------------------------------------------------------------- preview
    def tick(self, visible):
        """Called by the workbench refresh; detects tags only while visible."""
        if not visible:
            return
        frame = self.backend.frame
        if frame is None or frame is self._shown_frame:
            return
        self._shown_frame = frame
        board = self.board()
        found = tc.detect(frame.rgb, board)
        img = np.ascontiguousarray(frame.rgb).copy()
        for tag, c in found.items():
            cv2.polylines(img, [np.round(c).astype(np.int32)], True, (40, 220, 90), 2)
            cv2.putText(img, str(tag), tuple(np.round(c.mean(0)).astype(int)), cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 230, 60), 1)
        self._mark_corner(img, found, board, frame)
        h, w = img.shape[:2]; s = min(480/w, 360/h)
        self._preview = ImageTk.PhotoImage(Image.fromarray(cv2.resize(img, (int(w*s), int(h*s)), interpolation=cv2.INTER_AREA)))
        self.canvas.configure(image=self._preview, text='')
        q = self.backend.joint
        joints = ' '.join(f'{x:.1f}' for x in np.degrees(q.q_rad)) if q is not None else '无读数'
        self.preview_text.set(f'标签 {len(found)} 个 · J1..J6 {joints}°')


def _corner_pixel(found, board, name, K, dist):
    """Image position of a named board corner, with the corner order decided on
    this frame (the same rule as the solve)."""
    tag, c = name[3:].split('_c'); tag, c = int(tag), int(c)
    if tag not in found or 4*len(found) < 8:
        return None
    order, info = tc.choose_corner_order([dict(found=found)], board, K, dist)
    if info['best_median_px'] > 3:
        return None
    s, rev = order
    k = next(k for k in range(4) if (s+(-k if rev else k)) % 4 == c)
    return found[tag][k]


def summary(report, path):
    ok = lambda c: '通过' if c['ok'] else '不通过'
    names = dict(samples='可用样本数', rms_px='总重投影 RMS（px）', sample_rms_px='单张最大 RMS（px）',
                 consistency_m='单张一致性 平移（m）', consistency_deg='单张一致性 角度（°）',
                 leave_one_out_px='留一预测最大误差（px）', translation_std_m='外参平移标准差（m）',
                 rotation_std_deg='外参角度标准差（°）', corner_order='角点顺序判定（px）',
                 rotation_spread_deg='姿态转动多样性（°）', touch_rms_m='触点配准 RMS（m）',
                 touches_per_placement='每个板位触点数', distinct_corners_per_placement='每个板位不同角点数',
                 pivot_rotation_spread_deg='尖端标定姿态分散度（°）', tip_std_m='尖端坐标标准差（m）')
    lines = [('结果可用' if report['passed'] else '结果不可用（有检查未通过）')+f'：{path}']
    for k, c in report['checks'].items():
        v = c['value']; v = f'{v:.4f}' if isinstance(v, float) else v
        lines.append(f'  {names.get(k, k):<22} {v}  限值 {c["limit"]}  {ok(c)}')
    tip = report.get('tip')
    if tip:
        lines.append(f'尖端（link_6）：{", ".join(f"{x*1000:.1f}" for x in tip["link6_m"])} mm（{"自动求解" if tip["mode"] == "AUTO" else "给定"}）；'
                     f'触点残差 RMS {report["touch_registration"]["rms_m"]*1000:.2f} mm，最大 {report["touch_registration"]["max_m"]*1000:.2f} mm')
    ref = report.get('against_reference')
    if ref:
        lines.append(f'与当前外参相比：平移 {ref["translation_m"]*1000:.1f} mm，转角 {ref["rotation_deg"]:.2f}°'
                     f'（绕光轴 x {ref["about_optical_x_deg"]:+.2f}° / y {ref["about_optical_y_deg"]:+.2f}° / z {ref["about_optical_z_deg"]:+.2f}°），'
                     f'1 m 处的点移动 {ref["point_at_1m_moves_m"]*1000:.1f} mm')
    depth = [p['depth']['median_error_m'] for p in report.get('pnp', []) if p.get('depth')]
    if depth:
        lines.append(f'深度诊断：D435 深度 − 几何预测，各张中位数 {np.median(depth)*1000:+.1f} mm（范围 {min(depth)*1000:+.1f} … {max(depth)*1000:+.1f} mm）')
    if report.get('skipped'):
        lines.append('未使用（角点太少）：'+', '.join(s['name'] for s in report['skipped']))
    return '\n'.join(lines)

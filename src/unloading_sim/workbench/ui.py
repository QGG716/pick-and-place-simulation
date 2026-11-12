"""Ubuntu Tk workbench, using portable patterns from the reviewed reference GUI."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from queue import Queue, Empty
import threading
from time import monotonic_ns
import tkinter as tk
from tkinter import ttk, filedialog
from tkinter.scrolledtext import ScrolledText

import numpy as np
from PIL import Image, ImageTk, ImageDraw

from .stable_ui import StableStringVar, fixed_text, constrain_labels, append_log
from .backend import WorkbenchBackend
from .contracts import Source, static_pair
from .coordinates import CoordinateModel, Extrinsics, Installation, JointMapping, ToolFrame, rigid
from .devices import D435iObserver, RobotObserver, VacuumObserver

ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT / "assets/robots/realman_eco65/official/description/rm_models/ECO65/urdf/ECO65-B/urdf/ECO65-B.urdf"
URDF_SHA = "08b33a0588839f9fef4f08e13bb04ff27071c332f5ac40e688949a1ea5e9970b"
BG, PANEL, TEXT, MUTED, CYAN = "#0b1420", "#152435", "#e2edf5", "#a3b6c6", "#66dce6"


def load_mapping(path):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    for key in ("source_names", "target_names", "source_indices", "signs", "zero_offsets_rad"):
        data[key] = tuple(data[key])
    return JointMapping(**data)


class Workbench:
    def __init__(self, root, backend, *, snapshot=None, scene_input_root=None, detector_config=None, installation=None, snapshot_geometry_only=False):
        self.root, self.backend = root, backend
        self.snapshot_path, self.detector_config = snapshot, detector_config
        self.scene_input_root = scene_input_root
        self.snapshot_geometry_only = snapshot_geometry_only
        self.calibration = None
        self.installation = Installation.load(installation) if installation else None
        self.detecting = False
        self.detect_busy = False
        self.detector = None
        self.inference_generation = 0
        self._observed_generation = backend.observation_generation
        self.scene_generation = 0
        self.scene_windows = []
        self.last_inferred_id = None
        self.results = Queue()
        self.original_detection_frame = None
        self.original_detection_joint = None
        self._photos = {}
        self._shown_frame = None
        self._shown_detection = None
        self._closed = False
        self.tools = [ToolFrame.from_dict(t) for t in json.loads((ROOT/"configs/workbench/tool_frames.json").read_text())]
        active_tool = self.tools[1]
        if self.installation:
            active_tool = self.installation.tool
            self.tools = [active_tool] if active_tool else []
        self.world_from_base = np.eye(4)
        self.world_from_scene = np.eye(4)
        self.snapshot = None
        self.coordinate_basis = "Identity world=base_link; no installed scene loaded"
        if snapshot and Path(snapshot).is_file():
            from unloading_sim.geometry import rotation_matrix_from_rpy
            from .scene import load_snapshot
            self.snapshot = load_snapshot(snapshot, scene_input_root, geometry_only=snapshot_geometry_only)
            scene = self.snapshot["scene"]
            self.world_from_base[:3, :3] = rotation_matrix_from_rpy(*scene.get("base_rpy_rad", [0., 0., 0.]))
            self.world_from_base[:3, 3] = scene["base_position_m"]
            self.world_from_scene = rigid(scene['T_W_D'])
            self.world_from_base = self.world_from_scene @ self.world_from_base
            self.coordinate_basis = "Viewer uses planning frame D; world means external W (+X into trailer, +Y left, +Z up). T_W_base = T_W_D @ T_D_base"
        if self.installation:
            # Scene geometry remains in D. Express its display transform in the
            # user-defined world=base without modifying any frozen scene inputs.
            if self.snapshot:
                self.world_from_scene = np.linalg.inv(self.world_from_base) @ self.world_from_scene
            self.world_from_base = self.installation.world_from_base.copy()
            camera_mount = "相机随 link_1 转动" if self.installation.camera_parent_frame == "link_1" else "相机为外部固定支架"
            self.coordinate_basis = f"现场定义：world 与机器人 base_link 原点和坐标轴重合。{camera_mount}；历史仿真场景保持原几何。"
        self.model = None
        try:
            self.model = CoordinateModel(URDF, URDF_SHA, self.world_from_base, active_tool)
        except Exception as exc:
            backend.log("Model unavailable: " + str(exc))
        root.title("ECO65-B · Ubuntu observation workbench · READ ONLY")
        root.geometry("1440x1000")
        root.minsize(1120, 780)
        root.configure(bg=BG)
        style = ttk.Style(root)
        style.theme_use("clam")
        style.configure(".", background=PANEL, foreground=TEXT, font=("DejaVu Sans", 10))
        style.configure("TButton", padding=(9, 6), background="#264358")
        style.configure("TEntry", fieldbackground="#203346", foreground=TEXT)
        style.configure("TCombobox", fieldbackground="#203346", foreground=TEXT)
        style.map("TCombobox", fieldbackground=[("readonly", "#203346")])
        style.configure("Treeview", background=BG, fieldbackground=BG, foreground=TEXT, rowheight=26)
        style.configure("TNotebook.Tab", background="#264358", foreground=TEXT)
        style.map("TNotebook.Tab", background=[("selected", "#35627c")])
        ttk.Label(root, text="ECO65-B  /  只读观测工作台", font=("DejaVu Sans", 20, "bold"), foreground=CYAN).pack(fill="x", padx=16, pady=(12, 0))
        ttk.Label(root, text="阶段一 · 设备需手动连接 · 静止定位检查 · 硬件验证未完成 · 相机外参待独立验证", foreground="#ffd17b").pack(fill="x", padx=16, pady=8)
        tabs = self.tabs = ttk.Notebook(root)
        self.observe = ttk.Frame(tabs, padding=10)
        self.coords = ttk.Frame(tabs, padding=12)
        tabs.add(self.observe, text="观测与检测")
        tabs.add(self.coords, text="坐标、TCP 与场景")
        self._build_observe()
        self._build_coordinates()
        self.logbox = tk.Text(root, height=6, bg=BG, fg=MUTED, font=("DejaVu Sans Mono", 9), wrap="word", state="disabled")
        self.logbox.pack(side="bottom", fill="x", padx=12, pady=8)
        tabs.pack(fill="both", expand=True, padx=12)
        self._log_rows = (); self._coordinate_content = None
        constrain_labels(root)
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(80, self.refresh)

    def safe(self, function, *args):
        try:
            return function(*args)
        except Exception as exc:
            self.backend.log(f"ERROR {type(exc).__name__}: {exc}")
            self.notice.set(str(exc))

    def button(self, parent, text, function, **pack):
        widget = ttk.Button(parent, text=text, command=lambda: self.safe(function))
        widget.pack(side="left", padx=3, **pack)
        return widget

    def _build_observe(self):
        self.notice = StableStringVar(value="未连接设备 · 数据来源等待输入")
        self.camera_serial = StableStringVar()
        self.robot_host = StableStringVar(value="192.168.1.18")
        self.robot_port = StableStringVar(value="8080")
        self.serial_port = StableStringVar(value="/dev/ttyUSB0")
        self.profile_text = StableStringVar()
        self.profiles = {}
        for title, build in (("D435i 相机", self._camera_controls), ("ECO65-B 实测关节", self._robot_controls),
                             ("真空板 · 115200 / 8N1", self._vacuum_controls)):
            box = ttk.LabelFrame(self.observe, text=title, padding=7)
            box.pack(fill="x", pady=3)
            build(box)
        toolbar = ttk.Frame(self.observe)
        toolbar.pack(fill="x", pady=8)
        self.button(toolbar, "加载回放目录", self.choose_replay)
        self.button(toolbar, "下一条回放", self.replay_step)
        self.button(toolbar, "停止回放", self.stop_replay)
        self.button(toolbar, "开始检测", self.start_detection)
        self.button(toolbar, "停止检测", self.stop_detection)
        self.button(toolbar, "导入连接配置", self.choose_config)
        fixed_text(self.observe, self.notice, height=2)
        pictures = ttk.Frame(self.observe)
        pictures.pack(fill="both", expand=True, pady=8)
        self.images = {}
        for col, (key, title) in enumerate((("rgb", "实时 / 回放 RGB"), ("depth", "对齐 RGB 的深度"), ("detection", "原始检测与分割 · 非当前画面跟踪层"))):
            pictures.columnconfigure(col, weight=1, uniform="images")
            pane = ttk.LabelFrame(pictures, text=title, padding=4)
            pane.grid(row=0, column=col, sticky="nsew", padx=3)
            pane.pack_propagate(False)
            label = ttk.Label(pane, text="无观测数据", anchor="center", width=1)
            label.pack(fill="both", expand=True)
            self.images[key] = label
        pictures.rowconfigure(0, weight=1)
        self.metadata = StableStringVar(value="曝光时间未知时明确保留 UNKNOWN；主机接收时间不代表曝光时间。")
        fixed_text(self.observe, self.metadata, height=3)
        self.detection_status = StableStringVar(value="NO DETECTION")
        ttk.Label(self.observe, textvariable=self.detection_status, foreground="#ffd17b", width=1).pack(fill="x")
        self.detection_list = ttk.Treeview(self.observe, columns=("label", "score", "xyz", "observation"), show="headings", height=3)
        for key, text, width in (("label", "检测目标", 180), ("score", "置信度", 80), ("xyz", "原始 camera_optical 坐标 / m", 280), ("observation", "原始观测 ID / 来源", 400)):
            self.detection_list.heading(key, text=text)
            self.detection_list.column(key, width=width)
        self.detection_list.pack(fill="x", pady=(8, 0))

    def _camera_controls(self, box):
        ttk.Label(box, text="序列号").pack(side="left")
        ttk.Entry(box, textvariable=self.camera_serial, width=16).pack(side="left", padx=4)
        self.button(box, "连接 / 查询配置", lambda: self.backend.command("camera", "connect", self.camera_serial.get()))
        self.profile_combo = ttk.Combobox(box, textvariable=self.profile_text, values=(), width=21, state="readonly")
        self.profile_combo.pack(side="left", padx=5)
        self.button(box, "开始采集", lambda: self.backend.command("camera", "start", self.profiles[self.profile_text.get()]))
        self.button(box, "停止采集", lambda: self.backend.command("camera", "stop"))
        self.button(box, "断开", lambda: self.backend.command("camera", "disconnect"))
        self.camera_status = StableStringVar(value="DISCONNECTED")
        ttk.Label(box, textvariable=self.camera_status, foreground=CYAN).pack(side="right")

    def _robot_controls(self, box):
        ttk.Entry(box, textvariable=self.robot_host, width=16).pack(side="left", padx=4)
        ttk.Entry(box, textvariable=self.robot_port, width=6).pack(side="left", padx=4)
        self.button(box, "只读连接", lambda: self.backend.command("robot", "connect", self.robot_host.get(), int(self.robot_port.get())))
        self.button(box, "断开", lambda: self.backend.command("robot", "disconnect"))
        self.joints_text = StableStringVar(value="J1 —  J2 —  J3 —  J4 —  J5 —  J6 —  | DISCONNECTED")
        ttk.Label(box, textvariable=self.joints_text, font=("DejaVu Sans Mono", 9), width=1).pack(side="left", padx=10, fill="x", expand=True)

    def _vacuum_controls(self, box):
        ttk.Entry(box, textvariable=self.serial_port, width=20).pack(side="left", padx=4)
        self.button(box, "只读串口连接", lambda: self.backend.command("vacuum", "connect", self.serial_port.get()))
        self.button(box, "断开", lambda: self.backend.command("vacuum", "disconnect"))
        self.pressure_text = StableStringVar(value="吸具 —  储气罐 —  真空泵 —  压缩机 —  | DISCONNECTED")
        ttk.Label(box, textvariable=self.pressure_text, font=("DejaVu Sans Mono", 9), width=1).pack(side="left", padx=10, fill="x", expand=True)

    def _build_coordinates(self):
        bar = ttk.Frame(self.coords)
        bar.pack(fill="x")
        self.button(bar, "加载相机外参 JSON", self.choose_calibration)
        if self.installation:
            self.button(bar, "加载实测工具 TCP", self.choose_tool)
        current_tool = self.model.tool if self.model else (self.tools[-1] if self.tools else None)
        self.tool_name = StableStringVar(value=current_tool.name if current_tool else "正式吸具：TCP 待标定")
        combo = ttk.Combobox(bar, textvariable=self.tool_name, values=[t.name for t in self.tools], state="readonly", width=30)
        self.tool_combo = combo
        combo.pack(side="left", padx=10)
        combo.bind("<<ComboboxSelected>>", lambda e: self.change_tool())
        self.stationary = tk.BooleanVar(value=False)
        ttk.Checkbutton(bar, text="人工确认当前设备静止（非自动检测）", variable=self.stationary,
                        command=self.confirm_stationary).pack(side="left")
        self.calibration_text = StableStringVar(value="未加载相机外参；相机坐标不得用于宣称真实定位精度。")
        fixed_text(self.coords, self.calibration_text, height=3, pady=6)
        self.pairing_text = StableStringVar(value="未配对")
        ttk.Label(self.coords, textvariable=self.pairing_text, width=1).pack(fill="x", pady=6)
        chain = ("外部固定相机：T_world_camera = T_base_camera（与机器人关节角无关）；吸具 TCP 需要实测标定。"
                 if self.installation and self.installation.camera_parent_frame == "base_link"
                 else "T_world_camera = T_world_base × T_base_link1(q_capture) × T_link1_camera")
        self.camera_chain_text = StableStringVar(value=chain)
        ttk.Label(self.coords, textvariable=self.camera_chain_text, foreground=CYAN).pack(anchor="w", pady=10)
        self.coordinate_text = ScrolledText(self.coords, height=16, bg=BG, fg=TEXT, font=("DejaVu Sans Mono", 11), wrap="word", state="disabled")
        self.coordinate_text.pack(fill="both", expand=True)
        scene_bar = ttk.Frame(self.coords)
        scene_bar.pack(fill="x", pady=10)
        self.pose_source = StableStringVar(value="SIMULATED")
        ttk.Combobox(scene_bar, textvariable=self.pose_source, values=["SIMULATED", "MEASURED_OR_REPLAY", "PLANNED_PREVIEW"], width=24, state="readonly").pack(side="left")
        self.scene_view = StableStringVar(value="前方斜视")
        ttk.Combobox(scene_bar, textvariable=self.scene_view, values=["前方斜视", "前视", "左侧", "右侧", "后视"], width=10, state="readonly").pack(side="left", padx=4)
        self.button(scene_bar, "查看完整场景", self.open_scene)
        self.button(scene_bar, "加载规划轨迹预览", self.load_plan)
        self.plan_index = tk.DoubleVar(value=0)
        self.plan_slider = ttk.Scale(scene_bar, from_=0, to=0, variable=self.plan_index)
        self.plan_slider.pack(side="left", fill="x", expand=True, padx=10)
        self.plan = None
        ttk.Label(self.coords, text="仿真姿态、实测姿态与规划预览分别标记；预览时间不代表真实执行进度。\n场景复用主工程 NativeSceneWorld，保留机器人、12 杯完整几何、厢体、箱体和传送带。", foreground=MUTED).pack(anchor="w")

    def choose_config(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if path:
            self.load_config(path)

    def load_config(self, path):
        self.backend.import_config(path)
        c = self.backend.config
        self.robot_host.set(c.robot_host)
        self.robot_port.set(str(c.robot_port))
        self.serial_port.set(c.serial_port)
        self.camera_serial.set(c.camera_serial)

    def choose_replay(self):
        path = filedialog.askdirectory()
        if path:
            self.load_replay(path)

    def _sync_observations(self):
        """Invalidate GUI work when the backend revokes an observation generation."""
        if self._observed_generation == self.backend.observation_generation:
            return
        self._observed_generation = self.backend.observation_generation
        self.stop_detection()
        self.last_inferred_id = None
        self.original_detection_frame = self.backend.frame
        self.original_detection_joint = self.backend.joint
        self._shown_frame = self._shown_detection = None
        self._invalidate_scenes()

    def _inference_token(self):
        return (self.backend.observation_generation, self.inference_generation)

    def _invalidate_scenes(self):
        self.scene_generation += 1
        for window in self.scene_windows:
            if window.winfo_exists():
                window.destroy()
        self.scene_windows.clear()

    def load_replay(self, path):
        try:
            self.backend.load_replay(path)
            self.notice.set("离线回放 · " + self.backend.replay.manifest['provenance'])
        except Exception as exc:
            self.notice.set("Replay load failed: " + str(exc))
            raise
        finally:
            self._sync_observations()
            self.refresh(schedule=False)

    def replay_step(self):
        try:
            advanced = self.backend.step_replay()
            self.inference_generation += 1
            self.original_detection_frame = self.backend.frame
            self.original_detection_joint = self.backend.joint
            self._invalidate_scenes()
            self.notice.set("离线回放 · " + self.backend.replay.manifest['provenance'] if advanced
                            else "HISTORICAL_END_OF_REPLAY · 已到末尾，保留历史末帧")
        except Exception as exc:
            self.notice.set("Replay read failed: " + str(exc))
            raise
        finally:
            self._sync_observations()
            self.refresh(schedule=False)

    def stop_replay(self):
        self.backend.stop_replay()
        self._sync_observations()
        self.notice.set("Replay stopped / 观测已清空")
        self.refresh(schedule=False)

    def choose_calibration(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if path: self.load_calibration(path)

    def load_calibration(self, path):
        self.calibration = None  # An invalid replacement cannot leave an old calibration active.
        self._invalidate_scenes()
        self.refresh(schedule=False)
        self.calibration_text.set("加载检查中；旧外参已停用")
        try:
            calibration = Extrinsics.load(path)
            if self.installation:
                self.installation.check_camera(calibration)
            self.calibration = calibration
        except Exception as exc:
            self.calibration_text.set("外参被拒绝 / 未加载: " + str(exc))
            self.notice.set(str(exc))
            raise
        parent = self.calibration.parent_frame
        self.calibration_text.set(self.calibration.label + f"\n父 {parent} → 子 camera_optical；矩阵把光学坐标转换到 {parent}。")
        if parent == "base_link":
            self.camera_chain_text.set("外部固定相机：T_world_camera = T_world_base × T_base_camera（与机器人关节角无关）")
        else:
            self.camera_chain_text.set("T_world_camera = T_world_base × T_base_link1(q_capture) × T_link1_camera")
        self.backend.log("Loaded extrinsics SHA-256 " + self.calibration.sha256 + "; " + self.calibration.label)

    def change_tool(self):
        self._invalidate_scenes()
        if self.model:
            self.model.tool = next(t for t in self.tools if t.name == self.tool_name.get())
        self.backend.log("Tool display profile changed; original camera calibration unchanged")

    def choose_tool(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if path:
            self.load_tool(path)

    def load_tool(self, path):
        self._invalidate_scenes()
        if self.model:
            self.model.tool = None
        self.tools = []
        self.tool_combo.configure(values=[])
        self.tool_name.set("正式吸具：TCP 待标定")
        self.refresh(schedule=False)
        raw = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        if raw.get("unit") != "m" or not raw.get("status") or not raw.get("source"):
            raise ValueError("TOOL_REQUIRES_METRES_STATUS_AND_SOURCE")
        tool = ToolFrame.from_dict(raw)
        self.tools = [tool]
        self.tool_combo.configure(values=[tool.name])
        if self.model:
            self.model.tool = tool
        self.tool_name.set(tool.name)
        self.backend.log("Loaded tool TCP: " + tool.name + "; " + tool.status)

    def confirm_stationary(self):
        self.backend.manual_stationary = bool(self.stationary.get())
        self.backend.log("Stationary evidence: MANUAL_CONFIRMATION" if self.stationary.get() else "Stationary evidence cleared")

    def start_detection(self):
        if self.backend.replay_active and self.backend.detections is None:
            raise RuntimeError("当前回放记录没有已有检测输出；请选择包含原始检测结果的记录")
        if not self.detector_config and not self.backend.replay_active:
            raise RuntimeError("Select --detector-config with existing GroundingDINO/SAM weights, or load a replay with existing outputs")
        self.detecting = True
        self.backend.log("Detection enabled: existing replay output" if self.backend.replay_active else "Detection enabled: existing GroundingDINO + SAM adapters")

    def stop_detection(self):
        self.detecting = False
        self.inference_generation += 1
        self.backend.log("Detection stopped; preserved original results are historical")

    def _infer(self, frame, joint_at_capture, generation):
        try:
            if self.detector is None:
                from .detection import ExistingGroundingSAM
                config = json.loads(Path(self.detector_config).read_text(encoding="utf-8"))
                self.detector = ExistingGroundingSAM(**config)
            self.results.put(("detection", (generation, frame, joint_at_capture, self.detector.detect(frame))))
        except Exception as exc:
            self.results.put(("detection_error", (generation, str(exc))))

    def load_plan(self):
        path = filedialog.askopenfilename(filetypes=[("JSON", "*.json")])
        if not path: return
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        q, times = np.asarray(data["q_rad"], float), np.asarray(data["times_s"], float)
        if data["joint_names"] != [f"joint_{i}" for i in range(1, 7)] or q.ndim != 2 or q.shape[1] != 6 or not len(q):
            raise ValueError("Invalid planned preview joint order")
        if times.shape != (len(q),) or not np.isfinite(q).all() or not np.isfinite(times).all() or np.any(np.diff(times) < 0):
            raise ValueError("Invalid planned preview samples")
        self.plan = (q, times)
        self.plan_slider.configure(to=len(q)-1)
        self.pose_source.set("PLANNED_PREVIEW")
        self.backend.log("Trajectory loaded for visualization only; no execution task created")

    def open_scene(self):
        if not self.snapshot_path or not Path(self.snapshot_path).is_file():
            raise RuntimeError("Scene snapshot missing: start with --scene-snapshot; see Ubuntu setup guide")
        source = self.pose_source.get()
        q = np.zeros(6)
        if source == "MEASURED_OR_REPLAY":
            joint = self.backend.joint
            if joint is None or self.backend.sample_state(joint.stamp, "robot") not in {"LIVE_VALID", "REPLAY", "SIMULATED"}:
                raise RuntimeError("Measured/replay pose missing or stale")
            q = np.asarray(joint.q_rad)
            source = joint.stamp.source.value + " JOINT OBSERVATION"
        elif source == "PLANNED_PREVIEW":
            if self.plan is None: raise RuntimeError("No planned preview loaded")
            i = min(len(self.plan[0])-1, int(float(self.plan_index.get())))
            q = self.plan[0][i]
            source += f" {self.plan[1][i]:.3f}s / NOT EXECUTION"
        scene_from_world = np.linalg.inv(self.world_from_scene)
        optical = scene_from_world @ self.model.camera(q, self.calibration)["T_world_camera"] if self.model and self.calibration else None
        tcp = scene_from_world @ self.model.tcp(q) if self.model and self.model.tool else None
        azimuth = {"前方斜视": -30, "前视": 0, "左侧": 90, "右侧": 270, "后视": 180}[self.scene_view.get()]
        self.backend.log("Rendering complete original scene: " + source)
        generation = (self.backend.observation_generation, self.scene_generation)
        def render():
            renderer = None
            try:
                from .scene import SceneRenderer
                renderer = SceneRenderer(self.snapshot_path, self.scene_input_root, geometry_only=self.snapshot_geometry_only)
                image = renderer.render(q, optical, tcp, azimuth=azimuth)
                self.results.put(("scene", (generation, image, source)))
            except Exception as exc:
                self.results.put(("scene_error", (generation, "Scene unavailable: " + str(exc))))
            finally:
                if renderer is not None: renderer.close()
        threading.Thread(target=render, daemon=True).start()

    def image(self, key, pixels):
        im = Image.fromarray(pixels)
        im.thumbnail((380, 210))
        self._photos[key] = ImageTk.PhotoImage(im)
        self.images[key].configure(image=self._photos[key], text="")

    def stamp_text(self, stamp, device):
        age = stamp.age_s()
        return self.backend.sample_state(stamp, device) + (f" · age {age:.2f}s" if age is not None else " · age UNKNOWN")

    def refresh(self, schedule=True):
        if self._closed: return
        b = self.backend
        self._sync_observations()
        for name, kind, value in b.poll():
            if name == "camera" and kind == "state" and value["action"] == "connect":
                self.profiles = {f"{p.width}×{p.height} @ {p.fps}": p for p in value["value"]}
                self.profile_combo.configure(values=list(self.profiles))
                if self.profiles: self.profile_text.set(next(iter(self.profiles)))
            if kind == "error": self.notice.set(name + ": " + value)
        while True:
            try: kind, value = self.results.get_nowait()
            except Empty: break
            if kind == "detection":
                generation, frame, capture_joint, result = value
                self.detect_busy = False
                if self.detecting and generation == self._inference_token():
                    self.original_detection_frame, b.detections = frame, result
                    self.original_detection_joint = capture_joint
            elif kind == "scene":
                generation, pixels, source = value
                if generation != (b.observation_generation, self.scene_generation):
                    continue
                window = tk.Toplevel(self.root)
                self.scene_windows.append(window)
                window.title("ECO65 scene / " + source)
                ttk.Label(window, text=source + " · geometry only · no execution", foreground="#e59524").pack()
                photo = ImageTk.PhotoImage(Image.fromarray(pixels))
                label = ttk.Label(window, image=photo); label.image = photo; label.pack()
                ttk.Label(window, text="RGB arrows: loaded optical frame / selected TCP. Original shell and full tool remain nominal. 待独立验证").pack()
            elif kind == "detection_error":
                generation, message = value
                self.detect_busy = False
                if generation != self._inference_token():
                    continue
                self.detecting = False
                b.log(message)
                self.notice.set(message)
            elif kind == "scene_error":
                generation, message = value
                if generation == (b.observation_generation, self.scene_generation):
                    b.log(message)
                    self.notice.set(message)
        frame, joint = b.frame, b.joint
        live_camera = b.workers["camera"].device
        live_robot = b.workers["robot"].device
        camera_active = frame is not None and b.sample_active(frame.stamp, "camera")
        self.camera_status.set(self.stamp_text(frame.stamp, "camera") if frame else ("CONNECTED / STOPPED" if live_camera.connected else "DISCONNECTED"))
        if frame:
            if frame is not self._shown_frame:
                self._shown_frame = frame
                self.image("rgb", frame.rgb)
                import cv2
                scaled = np.uint8(np.clip(frame.depth*frame.depth_scale_m/2.0, 0, 1)*255)
                self.image("depth", cv2.applyColorMap(scaled, cv2.COLORMAP_TURBO)[:, :, ::-1])
            origin = frame.stamp.acquisition_source
            self.metadata.set(f"{frame.stamp.source.value} / original={origin.value if origin else 'UNKNOWN'} · {frame.device_id} · RGB#{frame.rgb_number} Depth#{frame.depth_number}\n"
                f"RGB device={frame.rgb_device_time.value_ms}ms/{frame.rgb_device_time.domain}; Depth device={frame.depth_device_time.value_ms}ms/{frame.depth_device_time.domain}; scale={frame.depth_scale_m}m; frame={frame.frame_id}\n"
                f"Host receive={frame.stamp.received_monotonic_ns}ns; processing complete={frame.stamp.processed_monotonic_ns}ns; {self.stamp_text(frame.stamp,'camera')}")
            if (self.detecting and not b.replay_active and not self.detect_busy and camera_active
                    and frame.observation_id != self.last_inferred_id
                    and frame.stamp.state(camera_active, b.config.max_age_s) == "LIVE_VALID"):
                self.detect_busy = True
                self.last_inferred_id = frame.observation_id
                threading.Thread(target=self._infer, args=(frame, b.joint_for_frame(frame), self._inference_token()), daemon=True).start()
        else:
            self.metadata.set("No RGB-D observation / DISCONNECTED")
            self.pairing_text.set("NO_OBSERVATION / 未配对")
            self._shown_frame = None
            for key in ("rgb", "depth"):
                self.images[key].configure(image="", text="NO OBSERVATION")
        if joint:
            self.joints_text.set("  ".join(f"J{i+1} {v:6.1f}°" for i, v in enumerate(np.rad2deg(joint.q_rad))) + " | " + self.stamp_text(joint.stamp, "robot"))
        else: self.joints_text.set("J1 —  J2 —  J3 —  J4 —  J5 —  J6 — | " + ("WAITING" if live_robot.connected else "DISCONNECTED"))
        vacuum = b.workers["vacuum"].device
        if b.pressure:
            p = b.pressure
            self.pressure_text.set(" / ".join(f"{n} {v:.1f}" for n, v in zip(("吸具", "储气罐", "真空泵", "压缩机"), p.kpa)) + " kPa | " + self.stamp_text(p.stamp, "vacuum") + " | " + p.integrity + " " + ",".join(p.alarms))
        else:
            values = []
            for i, name in enumerate(("吸具", "储气罐", "真空泵", "压缩机"), 1):
                sample = vacuum.gauges()[i] if vacuum.connected else None
                values.append(name+" —" if sample is None else f"{name} {sample.kpa:.1f}kPa {sample.integrity} {self.stamp_text(sample.stamp,'vacuum')}")
            self.pressure_text.set(("CONNECTED | " if vacuum.connected else "DISCONNECTED | ") + " | ".join(values)
                + " | 报警 " + ",".join(v["name"]+" "+self.stamp_text(v['stamp'],'vacuum') for v in tuple(vacuum.alarms.values()) if vacuum.connected))
        result = b.detections
        if result is None:
            self.detection_status.set("NO DETECTION")
            self._shown_detection = None
            self.detection_list.delete(*self.detection_list.get_children())
            self.images["detection"].configure(image="", text="NO DETECTION")
        elif self.original_detection_frame:
            self.detection_status.set("Detection " + ("RUNNING" if self.detecting or hasattr(self,"live_vision") else "STOPPED / historical output")
                + " · " + result.observation_id + " · " + result.source.value + " · "
                + self.stamp_text(self.original_detection_frame.stamp, "camera"))
        if result is not None and result is not self._shown_detection:
            self._shown_detection = result
            self.detection_list.delete(*self.detection_list.get_children())
            for item in result.detections:
                self.detection_list.insert("", "end", values=(item["label"], f"{item['score']:.3f}", str(item.get("center_m")), result.observation_id+" / "+result.source.value))
            original = self.original_detection_frame
            if original and original.observation_id == result.observation_id:
                pixels = original.rgb.copy()
                for mask in result.masks:
                    pixels[mask] = (pixels[mask]*.55+np.array([45, 230, 180])*.45).astype(np.uint8)
                self.image("detection", pixels)
        self.stationary.set(b.manual_stationary)
        details = (self.coordinate_basis + "\nT_W_D:\n" + str(np.round(self.world_from_scene, 5))
                   + "\nT_world_base:\n" + str(np.round(self.world_from_base, 5)))
        if self.model and self.model.tool:
            details += "\n\nTool: " + self.model.tool.name + "\n" + self.model.tool.status + "\nT_link6_tcp:\n" + str(self.model.tool.matrix)
        elif self.installation:
            details += "\n\n正式吸具已安装；实际 TCP 平移和朝向待测量。未采用历史 70 mm 探针或 140 mm 吸具数值。\nT_link6_tcp: 未标定，不计算实际 TCP。"
        if self.snapshot and self.calibration and self.model and self.calibration.parent_frame == "link_1":
            camera_delta = np.linalg.inv(rigid(self.snapshot['scene']['permanent_camera']['T_link1_optical'])) @ self.calibration.matrix
            details += ("\n\nINSTALLATION DISAGREEMENT / 原仿真外壳保留；加载的标定光学坐标单独显示\n"
                "T_nominal_camera_calibrated_camera:\n" + str(np.round(camera_delta, 6)))
            if self.model.tool:
                tool_delta = np.linalg.inv(rigid(self.snapshot['tool']['T_flange_tcp'])) @ self.model.tool.matrix
                details += "\nT_nominal_tcp_configured_tcp:\n" + str(np.round(tool_delta, 6))
        if frame:
            capture_joint = b.joint_for_frame(frame)
            pairing = static_pair(frame, capture_joint, camera_connected=camera_active,
                                  robot_connected=capture_joint is not None and b.sample_active(capture_joint.stamp, "robot"),
                                  manual_stationary=b.manual_stationary, max_age_s=b.config.max_age_s)
            self.pairing_text.set(pairing.status + f" | receive skew={pairing.receipt_delta_s}s | reliable capture pairing=False")
            if self.model and self.calibration and pairing.q_capture_rad is not None:
                try:
                    transform = self.model.paired_camera(pairing, self.calibration)["T_world_camera"]
                    details += "\n\nSTATIC VISUALIZATION APPROXIMATION ONLY\nT_world_camera:\n" + str(np.round(transform, 6))
                except ValueError as exc: details += "\n" + str(exc)
        original = self.original_detection_frame
        if original and result and self.model and self.calibration and original.observation_id == result.observation_id:
            association = static_pair(original, self.original_detection_joint, camera_connected=b.sample_active(original.stamp, "camera"),
                robot_connected=self.original_detection_joint is not None and b.sample_active(self.original_detection_joint.stamp, "robot"),
                manual_stationary=b.manual_stationary, max_age_s=b.config.max_age_s)
            if association.q_capture_rad is not None:
                try:
                    transforms = self.model.paired_camera(association, self.calibration)
                    details += "\n\nORIGINAL DETECTION / MANUAL STATIC APPROXIMATION ONLY: " + original.observation_id
                    for item in result.detections[:3]:
                        if item.get('center_m') is not None:
                            point = np.r_[item['center_m'], 1.]
                            details += "\n" + item['label'] + " base_m=" + str(np.round((transforms['T_base_camera']@point)[:3],5))
                            details += " world_m=" + str(np.round((transforms['T_world_camera']@point)[:3],5))
                except ValueError as exc: details += "\n" + str(exc)
        if details != self._coordinate_content:
            self._coordinate_content = details
            scroll_position = self.coordinate_text.yview()[0]
            self.coordinate_text.configure(state="normal")
            self.coordinate_text.delete("1.0", "end"); self.coordinate_text.insert("end", details)
            self.coordinate_text.configure(state="disabled")
            self.coordinate_text.yview_moveto(scroll_position)
        self._log_rows = append_log(self.logbox, b.logs, self._log_rows)
        if schedule:
            self.root.after(100, self.refresh)

    def close(self):
        self._closed = True
        self.stop_detection()
        self.backend.close()
        self.root.destroy()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--config", type=Path)
    p.add_argument("--replay", type=Path)
    p.add_argument("--calibration", type=Path)
    p.add_argument("--installation", type=Path, help="Explicit physical mounting and world/base definition")
    p.add_argument("--scene-snapshot", type=Path)
    p.add_argument("--scene-input-root", type=Path, help="Explicit archived input repository for historical snapshot config verification")
    p.add_argument("--detector-config", type=Path)
    p.add_argument("--make-demo", type=Path)
    a = p.parse_args()
    if a.make_demo:
        from .replay import generate_demo
        print(generate_demo(a.make_demo))
        return
    mapping = load_mapping(ROOT / "configs/workbench/joint_mapping.json")
    backend = WorkbenchBackend(D435iObserver(), RobotObserver(mapping), VacuumObserver())
    root = tk.Tk()
    ui = Workbench(root, backend, snapshot=a.scene_snapshot, scene_input_root=a.scene_input_root, detector_config=a.detector_config, installation=a.installation)
    if a.config: ui.safe(ui.load_config, a.config)
    if a.calibration: ui.safe(ui.load_calibration, a.calibration)
    if a.replay:
        ui.safe(ui.load_replay, a.replay)
    root.mainloop()

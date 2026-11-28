"""Tk panels of the operator assistant.

Two parts (user request 2026-11-28 "把'立即评估现场'换成'XD-Bot'，把对话框放到'观测与检测'标签页，
把对话的字调大两个字号 … 加入语音播报的开关"):
- `BrainPanel` (continuous task tab, right column): the XD-Bot button (assess
  now), automatic assessment and the verdict;
- its chat part, built in `chat_parent` (the "观测与检测" tab): the
  conversation in a font two sizes larger, typed questions, 🎤 voice input
  and 🔊 voice playback of the model's replies through the local speaker.
All brain events arrive through a queue and are drawn on the Tk thread. The
verdict is advisory only, as the panel says on its face.
"""
from queue import Queue, Empty
import time
import tkinter as tk
from tkinter import font as tkfont
from tkinter import ttk

LEVEL_COLOURS = dict(OK=('#1b5e20', '#e8f5e9'), CAUTION=('#8a5a00', '#fff4e0'), ALERT=('#b71c1c', '#fdecea'))
# Dark workbench theme (workbench/ui.py BG/PANEL/TEXT/MUTED).
CHAT_BG, CHAT_TEXT, CHAT_MUTED = '#152435', '#e2edf5', '#a3b6c6'
CHAT_TAGS = dict(op='#8ec5ff', ai='#e2edf5', sys='#ffd17b', stop='#ff8a80')
FONT_STEP = 2  # "把对话的字调大两个字号"


def chat_font_size(widget, step=FONT_STEP):
    """Default Tk font size plus `step` (Tk reports pixels as negative sizes)."""
    size = tkfont.nametofont('TkDefaultFont', root=widget).actual('size')
    return size + step if size > 0 else size - step


class BrainPanel(ttk.LabelFrame):
    def __init__(self, parent, *, settings, issues, make_brain, on_voice_stop_ui, start=True, chat_parent=None):
        super().__init__(parent, text='智能助手 · Qwen VLM（仅提示，不影响任何安全检查和作业）', padding=4)
        self.events = Queue()
        self.on_voice_stop_ui = on_voice_stop_ui
        self.brain = None
        voice = (settings or {}).get('voice', {})
        row = ttk.Frame(self)
        row.pack(fill='x')
        self.state_text = tk.StringVar(value='未启动')
        self.assess_button = ttk.Button(row, text='XD-Bot', command=self._assess)
        self.assess_button.pack(side='left')
        self.auto = tk.BooleanVar(value=bool((settings or {}).get('assessment', {}).get('auto', True)))
        ttk.Checkbutton(row, text='自动评估', variable=self.auto, command=self._toggle_auto).pack(side='left', padx=(6, 0))
        ttk.Label(row, textvariable=self.state_text, width=1).pack(side='left', fill='x', expand=True, padx=(8, 0))
        self.verdict = tk.Label(self, text='尚无评估', anchor='w', justify='left', font=('TkDefaultFont', 11, 'bold'),
                                fg=CHAT_MUTED, bg=CHAT_BG, padx=6, pady=3)
        self.verdict.pack(fill='x', pady=(3, 0))
        self.detail = tk.Label(self, text='', anchor='w', justify='left', wraplength=700, padx=6, fg=CHAT_TEXT, bg=CHAT_BG)
        self.detail.pack(fill='x')
        self.detail.bind('<Configure>', lambda e: self.detail.configure(wraplength=max(200, e.width - 12)))
        self.chat = self._build_chat(chat_parent or self, voice)
        controls = [self.assess_button, self.listen_button, self.speak_button, self.entry]
        if not voice.get('enabled'):
            self.listen_button.configure(state='disabled')
            self.speak_button.configure(state='disabled')
        if settings is None or not settings.get('enabled', True):
            self.state_text.set('智能助手未启用' + ('：' + '；'.join(issues) if issues else ''))
            self.chat_state.set(self.state_text.get())
            for w in controls:
                w.configure(state='disabled')
            return
        if issues:
            self._append('sys', '配置提示：' + '；'.join(issues))
        if not start:
            self.state_text.set('智能助手未启动（预览/测试模式，不连接 VLM、不打开麦克风和扬声器）')
            self.chat_state.set(self.state_text.get())
            return
        try:
            self.brain = make_brain(lambda k, v: self.events.put((k, v)))
        except Exception as error:
            self.state_text.set(f'智能助手无法创建：{error}')
            return
        self.brain.start()
        self.after(200, self._poll)

    def _build_chat(self, parent, voice):
        frame = ttk.LabelFrame(parent, text='XD-Bot 对话（Qwen VLM · 仅提示）', padding=4)
        frame.pack(fill='both', expand=True, pady=(3, 0))
        size = chat_font_size(frame)
        self.chat_font = ('TkDefaultFont', size)
        row = ttk.Frame(frame)
        row.pack(fill='x')
        self.listen = tk.BooleanVar(value=False)
        self.listen_button = ttk.Checkbutton(row, text='🎤 语音对话', variable=self.listen, command=self._toggle_listen)
        self.listen_button.pack(side='left')
        self.speak = tk.BooleanVar(value=False)
        self.speak_button = ttk.Checkbutton(row, text='🔊 语音播报', variable=self.speak, command=self._toggle_speak)
        self.speak_button.pack(side='left', padx=(8, 0))
        self.chat_state = tk.StringVar(value='')
        ttk.Label(row, textvariable=self.chat_state, width=1).pack(side='left', fill='x', expand=True, padx=(8, 0))
        ask = ttk.Frame(frame)
        ask.pack(side='bottom', fill='x', pady=(4, 0))
        self.question = tk.StringVar()
        self.entry = tk.Entry(ask, textvariable=self.question, font=self.chat_font, bg='#203346', fg=CHAT_TEXT,
                              insertbackground=CHAT_TEXT, relief='flat')
        self.entry.pack(side='left', fill='x', expand=True, ipady=3)
        self.entry.bind('<Return>', lambda e: self._ask())
        ttk.Button(ask, text='发送', command=self._ask).pack(side='left', padx=(4, 0))
        self.log = tk.Text(frame, height=8, wrap='char', state='disabled', relief='flat', bg=CHAT_BG, fg=CHAT_TEXT,
                           font=self.chat_font, padx=6, pady=4)
        for tag, colour in CHAT_TAGS.items():
            self.log.tag_configure(tag, foreground=colour)
        self.log.tag_configure('stop', font=('TkDefaultFont', size, 'bold'))
        self.log.tag_configure('time', foreground=CHAT_MUTED)
        self.log.pack(side='top', fill='both', expand=True, pady=(4, 0))
        return frame

    # -- actions ------------------------------------------------------------------
    def _assess(self):
        if self.brain:
            self.brain.request_assessment('operator')
            self.state_text.set('评估请求已排队…')

    def _toggle_auto(self):
        if self.brain:
            self.brain.auto = bool(self.auto.get())

    def _toggle_listen(self):
        if not self.brain:
            return
        on = bool(self.listen.get())
        if on:
            self._set_chat_state('正在加载本地语音模型…')
        if not self.brain.set_listening(on):
            self.listen.set(False)

    def _toggle_speak(self):
        if not self.brain:
            return
        on = bool(self.speak.get())
        if on:
            self._set_chat_state('正在加载本地语音模型…')
        if not self.brain.set_speaking(on):
            self.speak.set(False)
        elif on:
            self._set_chat_state('语音播报已打开（本机扬声器）')
        else:
            self._set_chat_state('语音播报已关闭')

    def _set_chat_state(self, text):
        self.chat_state.set(text)
        self.update_idletasks()

    def _ask(self):
        text = self.question.get().strip()
        if text and self.brain:
            self.question.set('')
            self._append('op', f'操作员（键入）：{text}')
            self.brain.ask(text, 'typed')

    # -- drawing --------------------------------------------------------------------
    def _append(self, tag, text):
        self.log.configure(state='normal')
        self.log.insert('end', time.strftime('%H:%M:%S '), 'time')
        self.log.insert('end', text + '\n', tag)
        lines = int(self.log.index('end-1c').split('.')[0])
        if lines > 300:
            self.log.delete('1.0', f'{lines - 300}.0')
        self.log.see('end')
        self.log.configure(state='disabled')

    def _poll(self):
        try:
            while True:
                kind, value = self.events.get_nowait()
                self._draw(kind, value)
        except Empty:
            pass
        if self.brain:
            self.brain.tick()
        self.after(200, self._poll)

    def _draw(self, kind, value):
        if kind == 'state':
            self.state_text.set(value['detail'] or value['state'])
            self.chat_state.set('VLM ' + {'READY': '已就绪', 'CONNECTING': '连接中…', 'ERROR': '连接失败'}.get(value['state'], value['state']))
            if value['state'] == 'ERROR':
                self._append('sys', value['detail'])
        elif kind == 'assessment':
            fg, bg = LEVEL_COLOURS[value.level]
            stamp = time.strftime('%H:%M:%S', time.localtime(value.stamp or time.time()))
            rt = value.timing.get('round_trip_s') if value.timing else None
            self.verdict.configure(text=f'{value.headline()}（{stamp} · RGB #{value.rgb_number}'
                                        + (f' · {rt:.1f} s' if rt else '') + '）', fg=fg, bg=bg)
            parts = []
            if value.summary:
                parts.append(value.summary)
            if value.hazards:
                parts.append('隐患：' + '；'.join(value.hazards))
            if value.reasons:
                parts.append('依据：' + '；'.join(value.reasons))
            if value.status != 'OK':
                parts.append(value.error or value.raw_text[:200])
            self.detail.configure(text='\n'.join(parts))
            if self.brain and self.brain.state == 'READY':
                self.state_text.set('就绪 · 上次评估 ' + stamp)
        elif kind == 'heard':
            self._append('op', f'操作员（语音）：{value}')
        elif kind == 'reply':
            self._append('sys' if value.get('error') else 'ai', f'XD-Bot：{value["text"]}')
        elif kind == 'voice_stop':
            self._append('stop', f'语音停止：“{value["text"]}” → {value["outcome"]}（与结束作业按钮相同的软件停止，不释放吸住的箱子）')
            self.on_voice_stop_ui()
        elif kind == 'voice_status':
            self.chat_state.set(value)
            if '错误' in value or '失败' in value:
                self._append('sys', value)
                if '麦克风' in value:
                    self.listen.set(False)
                if ('播放' in value or '合成' in value) and self.brain:
                    self.speak.set(False)
                    self.brain.set_speaking(False)

    def close(self):
        if self.brain:
            self.brain.close()

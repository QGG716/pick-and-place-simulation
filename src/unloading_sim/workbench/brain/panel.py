"""Tk panel of the operator assistant (display, typed questions, voice toggle).

All brain events arrive through a queue and are drawn on the Tk thread. The
panel says on its face that the VLM verdict is advisory only.
"""
from queue import Queue, Empty
import time
import tkinter as tk
from tkinter import ttk

LEVEL_COLOURS = dict(OK=('#1b5e20', '#e8f5e9'), CAUTION=('#8a5a00', '#fff4e0'), ALERT=('#b71c1c', '#fdecea'))


class BrainPanel(ttk.LabelFrame):
    def __init__(self, parent, *, settings, issues, make_brain, on_voice_stop_ui, start=True):
        super().__init__(parent, text='智能助手 · Qwen VLM（仅提示，不影响任何安全检查和作业）', padding=4)
        self.events = Queue()
        self.on_voice_stop_ui = on_voice_stop_ui
        self.brain = None
        row = ttk.Frame(self)
        row.pack(fill='x')
        self.state_text = tk.StringVar(value='未启动')
        self.assess_button = ttk.Button(row, text='立即评估现场', command=self._assess)
        self.assess_button.pack(side='left')
        self.auto = tk.BooleanVar(value=bool((settings or {}).get('assessment', {}).get('auto', True)))
        ttk.Checkbutton(row, text='自动评估', variable=self.auto, command=self._toggle_auto).pack(side='left', padx=(6, 0))
        self.listen = tk.BooleanVar(value=False)
        self.listen_button = ttk.Checkbutton(row, text='🎤 语音对话', variable=self.listen, command=self._toggle_listen)
        self.listen_button.pack(side='left', padx=(6, 0))
        ttk.Label(row, textvariable=self.state_text, width=1).pack(side='left', fill='x', expand=True, padx=(8, 0))
        self.verdict = tk.Label(self, text='尚无评估', anchor='w', justify='left', font=('TkDefaultFont', 11, 'bold'),
                                fg='#555', bg='#f2f2f2', padx=6, pady=3)
        self.verdict.pack(fill='x', pady=(3, 0))
        self.detail = tk.Label(self, text='', anchor='w', justify='left', wraplength=700, padx=6)
        self.detail.pack(fill='x')
        self.detail.bind('<Configure>', lambda e: self.detail.configure(wraplength=max(200, e.width - 12)))
        self.log = tk.Text(self, height=5, wrap='word', state='disabled', relief='flat', bg='#fafafa')
        self.log.tag_configure('op', foreground='#0d47a1')
        self.log.tag_configure('ai', foreground='#222')
        self.log.tag_configure('sys', foreground='#8a5a00')
        self.log.tag_configure('stop', foreground='#b71c1c', font=('TkDefaultFont', 10, 'bold'))
        self.log.pack(fill='x', pady=(3, 0))
        ask = ttk.Frame(self)
        ask.pack(fill='x', pady=(3, 0))
        self.question = tk.StringVar()
        entry = ttk.Entry(ask, textvariable=self.question)
        entry.pack(side='left', fill='x', expand=True)
        entry.bind('<Return>', lambda e: self._ask())
        ttk.Button(ask, text='发送', command=self._ask).pack(side='left', padx=(4, 0))
        voice_on = bool((settings or {}).get('voice', {}).get('enabled'))
        if not voice_on:
            self.listen_button.configure(state='disabled')
        if settings is None or not settings.get('enabled', True):
            self.state_text.set('智能助手未启用' + ('：' + '；'.join(issues) if issues else ''))
            for w in (self.assess_button, self.listen_button, entry):
                w.configure(state='disabled')
            return
        if issues:
            self._append('sys', '配置提示：' + '；'.join(issues))
        if not start:
            self.state_text.set('智能助手未启动（预览/测试模式，不连接 VLM、不打开麦克风）')
            return
        try:
            self.brain = make_brain(lambda k, v: self.events.put((k, v)))
        except Exception as error:
            self.state_text.set(f'智能助手无法创建：{error}')
            return
        self.brain.start()
        self.after(200, self._poll)

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
            self.state_text.set('正在加载本地语音模型…')
            self.update_idletasks()
        if not self.brain.set_listening(on):
            self.listen.set(False)

    def _ask(self):
        text = self.question.get().strip()
        if text and self.brain:
            self.question.set('')
            self._append('op', f'操作员（键入）：{text}')
            self.brain.ask(text, 'typed')

    # -- drawing --------------------------------------------------------------------
    def _append(self, tag, text):
        self.log.configure(state='normal')
        self.log.insert('end', time.strftime('%H:%M:%S ') + text + '\n', tag)
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
            self._append('sys' if value.get('error') else 'ai', f'助手：{value["text"]}')
        elif kind == 'voice_stop':
            self._append('stop', f'语音停止：“{value["text"]}” → {value["outcome"]}（与结束作业按钮相同的软件停止，不释放吸住的箱子）')
            self.on_voice_stop_ui()
        elif kind == 'voice_status':
            self.state_text.set(value)
            if '错误' in value or '失败' in value:
                self._append('sys', value)
                self.listen.set(False)

    def close(self):
        if self.brain:
            self.brain.close()

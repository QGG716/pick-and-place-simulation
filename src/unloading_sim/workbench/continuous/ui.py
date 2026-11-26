"""ECO65-B continuous multi-carton entry (``eco65-workbench-continuous``).

The single-carton workbench with one more panel: one operator click starts one
job of consecutive single-carton cycles (``continuous.job.ContinuousJob``).
Startup, connection, observation, preview and planning never move hardware;
motion starts only inside a job the operator started, through the unchanged
single-carton execute path. The stop button and Esc stop the job and the
current cycle; a stop never releases a held load.
"""
import argparse
import json
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk

from ..stable_ui import StableStringVar, fixed_text
from ..single_carton import ui as single_ui
from ..single_carton.ui import SingleWorkbench
from .job import ContinuousJob, config_issues, settings as job_settings, END_REASONS

# Execute-button blockers that belong to the operator's manual single-cycle
# path (a job plans and executes by itself) or to the job's own activity.
MANUAL_ONLY_BLOCKERS = ('无有效计划', '未查看完整轨迹预览', '操作进行中')

JOB_STATES = dict(IDLE='未开始', STARTING='启动中', RECEIVER_CLEARING='等待落箱清空', WAIT_OPERATOR_CLEAR='等待确认落箱位已清空',
                  OBSERVING='观测中', PLANNING='规划中', EXECUTING='执行中', ENDED='已结束')


class ContinuousWorkbench(SingleWorkbench):
    def __init__(self, root, backend, planner, service, vacuum, config, model, calibration, *, auto_start=True):
        self.continuous_job = None
        self.job_gate_blockers = ['执行条件尚未刷新']
        super().__init__(root, backend, planner, service, vacuum, config, model, calibration, auto_start=auto_start)
        root.title('ECO65-B · 视觉连续抓放 · 实机模式')
        labels = [w for w in root.winfo_children() if isinstance(w, ttk.Label)]
        if labels:
            labels[0].configure(text='ECO65-B / 视觉连续抓放（每轮为完整单箱循环）')
        self.tabs.tab(self.task_tab, text='连续抓放任务')
        self._build_job_panel()

    # -- panel ------------------------------------------------------------------
    def _build_job_panel(self):
        problems = config_issues(self.config_single)
        try:
            job = job_settings(self.config_single)
        except ValueError:
            job = None
        panel = ttk.LabelFrame(self.task_tab, text='连续作业', padding=8)
        panel.pack(fill='x', pady=(6, 0), before=self.task_tab.winfo_children()[1])
        row = ttk.Frame(panel)
        row.pack(fill='x')
        ttk.Label(row, text='本次作业箱数').pack(side='left')
        self.job_count = tk.IntVar(value=(job or {}).get('max_cartons', 1))
        ttk.Spinbox(row, from_=1, to=(job or {}).get('max_cartons', 1), textvariable=self.job_count, width=5).pack(side='left', padx=(4, 12))
        self.job_start_button = ttk.Button(row, text='▶ 开始连续作业', command=lambda: self.safe(self.start_job))
        self.job_start_button.pack(side='left')
        self.job_stop_button = ttk.Button(row, text='结束作业（停止当前循环）', command=lambda: self.safe(self.stop_cycle))
        self.job_stop_button.pack(side='left', padx=(8, 0))
        self.job_clear_button = ttk.Button(row, text='落箱位已清空', command=lambda: self.safe(self.confirm_clear))
        self.job_clear_button.pack(side='left', padx=(8, 0))
        if job is None or job['receiver_clearing']['mode'] != 'OPERATOR_CONFIRM_EACH':
            self.job_clear_button.pack_forget()
        if job is None:
            policy = '作业配置无效'
        else:
            r = job['receiver_clearing']
            clearing = {'USER_CONFIRMED_SELF_CLEARING': f'落箱自动清空（用户确认，等待 {r["clearing_time_s"]:.1f} s，非传感器证据）',
                        'OPERATOR_CONFIRM_EACH': '每轮由操作员确认落箱位已清空'}.get(r['mode'], r['mode'])
            policy = (f'{clearing} · 抓取顺序：先高后低、同层先中间后两边；上方压着箱子的不先抓'
                      f' · 计划有效期 {job["plan_validity_s"]:.0f} s · 起点关节核对必须开启')
        self.job_policy = StableStringVar(value=policy + ('\n不能开始：' + '；'.join(problems) if problems else ''))
        fixed_text(panel, self.job_policy, height=2, pady=2)
        self.job_text = StableStringVar(value='连续作业未开始 · 实机执行 NOT_RUN')
        fixed_text(panel, self.job_text, height=2, pady=2)
        columns = ('round', 'task', 'order', 'target', 'result')
        self.job_rounds = ttk.Treeview(panel, columns=columns, show='headings', height=4)
        for key, title, width in (('round', '轮', 40), ('task', '任务', 90), ('order', '尝试顺序（检测编号）', 170),
                                  ('target', '抓取目标', 90), ('result', '结果', 420)):
            self.job_rounds.heading(key, text=title)
            self.job_rounds.column(key, width=width, stretch=key == 'result', anchor='w')
        self.job_rounds.pack(fill='x', pady=(4, 0))
        self._job_rows = None

    # -- actions ----------------------------------------------------------------
    def job(self, kind, fn):
        if self.job_running():
            raise ValueError('连续作业进行中：单箱操作已禁用，请先结束作业')
        return super().job(kind, fn)

    def job_running(self):
        return self.continuous_job is not None and self.continuous_job.running

    def start_job(self):
        if self.job_running():
            raise ValueError('连续作业已在进行')
        if getattr(self, 'job_pending', False) or self.manager.active:
            raise ValueError('有单箱操作进行中，不能开始连续作业')
        blockers = list(self.job_gate_blockers)
        if blockers:
            raise ValueError('尚未满足执行条件：' + '、'.join(blockers))
        count = int(self.job_count.get())
        config = dict(self.config_single, continuous_job=dict(self.config_single['continuous_job'], max_cartons=count))
        from tkinter import messagebox
        settings = job_settings(config)
        text = (f'开始连续作业：最多 {count} 箱。每轮自动观测 → 规划 → 执行完整单箱循环，'
                f'释放后{"等待 %.1f s 落箱自动清空" % settings["receiver_clearing"]["clearing_time_s"] if settings["receiver_clearing"]["mode"] == "USER_CONFIRMED_SELF_CLEARING" else "等待你确认落箱位已清空"}，再拍新帧开始下一轮。\n'
                '任一轮未完成、设备或会话变化、没有可抓目标或没有完整计划时作业结束，不自动恢复。\n'
                '请手持急停。确认现在开始？')
        if not messagebox.askokcancel('确认开始连续作业', text, parent=self.root):
            self.details.set('已取消，未开始连续作业。')
            return
        self.previewed = False
        job = ContinuousJob(self.manager, config, notify=lambda k, v: self.jobs.put((k, v)), execute_gate=self.job_gate)
        job.start()
        self.continuous_job = job
        self.job_text.set(f'连续作业 {job.job_id[:8]} 已开始 · 记录 {job.folder}')

    def job_gate(self):
        """Called by the job thread right before each round's execute."""
        return list(self.job_gate_blockers)

    def confirm_clear(self):
        if not self.job_running():
            raise ValueError('没有进行中的连续作业')
        self.continuous_job.confirm_receiver_clear()

    def stop_cycle(self):
        if self.continuous_job is not None and self.continuous_job.running:
            self.continuous_job.stop()
        super().stop_cycle()

    # -- refresh ----------------------------------------------------------------
    def _update_task_once(self):
        super()._update_task_once()
        if self._closed:  # the base refresh destroyed the window on close
            return
        blockers = getattr(self, 'execute_blockers', None)
        if blockers is not None:
            self.job_gate_blockers = [b for b in blockers if b not in MANUAL_ONLY_BLOCKERS and not b.startswith('计划')]
        running = self.job_running()
        if running:
            for button in (self.observe_button, self.plan_button, self.execute_button, self.reset_button,
                           getattr(self, 'move_button', None)):
                if button is not None:
                    button.configure(state='disabled')
        m = self.manager
        startable = (not running and not getattr(self, 'job_pending', False) and not m.active and not m.vacuum.holding
                     and not (m.cycle and m.cycle.state == 'STOP_UNCONFIRMED') and not self.job_gate_blockers
                     and not config_issues(self.config_single))
        self.job_start_button.configure(state='normal' if startable else 'disabled')
        self.job_stop_button.configure(state='normal' if running else 'disabled')
        job = self.continuous_job
        if job is None:
            # Refreshed on every tick: a reason shown at start-up (devices still
            # connecting) must not stay once the gate is clear (2026-11-26).
            if startable:
                self.job_text.set('连续作业未开始 · 可以开始：点“▶ 开始连续作业” · 实机执行 NOT_RUN')
            elif not running:
                why = self.job_gate_blockers or config_issues(self.config_single)
                self.job_text.set('连续作业未开始' + (' · 尚不能开始：' + '、'.join(why) if why else '') + ' · 实机执行 NOT_RUN')
            return
        status = job.status()
        if job.settings['receiver_clearing']['mode'] == 'OPERATOR_CONFIRM_EACH':
            self.job_clear_button.configure(state='normal' if status['state'] == 'WAIT_OPERATOR_CLEAR' else 'disabled')
        head = (f'作业 {status["job_id"][:8]} · {JOB_STATES.get(status["state"], status["state"])} · '
                f'已完成 {status["completed"]}/{status["max_cartons"]} 箱')
        if status['end']:
            head += f' · 结束原因：{status["end"]["label"]}'
        self.job_text.set(head + '\n' + (status['detail'] or '') + f' · 记录 {status["folder"]}')
        rows = [(str(r['round']), (r.get('task_id') or '—')[:8],
                 ' → '.join(map(str, (r.get('target_order') or {}).get('order', []))) or '—',
                 str(r.get('target_detection_index', '—')), self._round_result(r)) for r in status['rounds']]
        if rows != self._job_rows:
            self._job_rows = rows
            self.job_rounds.delete(*self.job_rounds.get_children())
            for row in rows:
                self.job_rounds.insert('', 'end', values=row)

    @staticmethod
    def _round_result(r):
        skipped = (r.get('target_order') or {}).get('skipped') or []
        note = ('；跳过 ' + ', '.join(f'{s["detection_index"]}（上方压着 {s["above"]}）' for s in skipped)) if skipped else ''
        if r.get('cycle_state'):
            return f'{r["cycle_state"]} · {r.get("physical_result")}' + (f' · {r.get("cycle_reason")}' if r.get('cycle_reason') else '') + note
        if r.get('planning') == 'FAILED':
            return '规划失败：' + str(r.get('reason', ''))[:160] + note
        if r.get('execution') == 'REFUSED':
            return '执行被拒：' + str(r.get('reason', ''))[:160] + note
        if r.get('planning') == 'PLAN_READY':
            return '计划就绪 / 执行中' + note
        return ('观测：检测 %s 个，可测 %s 个' % (r.get('detections', '—'), len(r.get('measured_boxes', []) or []))) + note


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--ui-smoke', action='store_true')
    p.add_argument('--check-config', action='store_true')
    args = p.parse_args()
    config = single_ui.load(args.config)
    problems = config_issues(config)
    if not problems:
        from .job import apply_to_config
        apply_to_config(config)
    if args.check_config:
        print(json.dumps(dict(single_carton_issues=single_ui.issues(config), continuous_job_issues=problems,
                              continuous_job=None if problems else job_settings(config),
                              plan_validity_s=config.get('plan_validity_s'), hardware_execution='NOT_RUN'),
                         ensure_ascii=False, indent=2))
        return
    from ..coordinates import Extrinsics, CoordinateModel, Installation
    from ..ui import URDF, URDF_SHA, load_mapping
    from ..controlled.service import ExecutionService
    from ..controlled.trajectory import Authority
    from ..devices import D435iObserver
    from ..single_carton.network import NetworkedRobotView
    from ..single_carton.live import SingleBackend
    from ..single_carton.planner import SinglePlanner
    from ..single_carton.vacuum import VacuumOwner, VacuumView
    installation = Installation.load(config['execution']['installation'])
    model = CoordinateModel(URDF, URDF_SHA, installation.world_from_base, installation.tool)
    calibration = Extrinsics.load(config['camera_extrinsics']) if config.get('camera_extrinsics') else None
    if calibration:
        installation.check_camera(calibration)
    mapping = load_mapping(config['execution']['mapping'])
    authority = Authority('eco65_single_carton_segment_v1')
    planner = SinglePlanner(config, mapping, authority, model)
    service = ExecutionService(mapping, config['execution'], authority, planner.binding)
    vacuum = VacuumOwner(config['vacuum'])
    backend = SingleBackend(D435iObserver(), NetworkedRobotView(service, mapping, config.get('robot_network', {})), VacuumView(vacuum), service)
    root = tk.Tk()
    ui = ContinuousWorkbench(root, backend, planner, service, vacuum, config, model, calibration, auto_start=not args.ui_smoke)
    if args.ui_smoke:
        root.after(int(__import__('os').environ.get('ECO65_SMOKE_MS','700')), ui.close)
    root.mainloop()


if __name__ == '__main__':
    main()

# POC step 3.1 精简证据

主报告：[../../m710_poc_step31_release_reserves.md](../../m710_poc_step31_release_reserves.md)。CPU 源码 `d3905098ec537de88fee4cad8ea4e7b99cc82c52`，执行源码 `53537b783ba55e3e3054e91aab692e656a33767d`；执行目录 35 个 CPU 文件和 22 个执行文件与提交一致。

| 文件 | 内容 |
|---|---|
| cpu_delivery_summary.json | 完整任务、实际阶段采样、释放高度及候选身份 |
| prefix_binding_check.json | 原始实际 q、精确保留的 126 个带载节点、新旧输入 SHA |
| effective_run_configuration.json | 实际配置、有限额度、无墙钟截止、场景/策略/实现指纹 |
| delivery_status.json | CPU 交付时的 preflight/export/readback 与只读 first_feasible 哈希 |
| prior_tracking_point_audit.json | 旧实跑同帧对应箱角偏移，不是全局误差上界 |
| suffix_sensitivity.json / suffix_sensitivity.py | 新后缀的名义及两组同帧实测扰动结果与复查脚本 |
| source_verification.json | 与 Git 的逐文件源码比对及无关旧 demo 换行差异说明 |
| focused_tests.txt / final_execution_tests.txt | 156 passed / 20 skipped，以及最后相关 47 passed；后者是复核子集，不能相加为独立总数 |
| initial_archive_binding.json | 新世界静置后的实际初态绑定 |
| physical_summary.json / analyze_physical.py | 唯一实跑、三个释放时刻、实测净空、事件来源和真实状态应用链路 |
| video_audit.json / video_audit.py | 640×360、5 fps、全部 890 帧解码与物理时间比对 |
| run_once.sh | 本次对应工况的单次执行命令；已有输出时拒绝覆盖/重复运行 |

`delivery_status.json` 的 `isaac_executed=false` 是 **CPU 交付当时**的状态，保留原记录；后续唯一物理结果见 `physical_summary.json`，不倒改已保存的 CPU 证据。

服务器 CPU 原始结果：`/root/autodl-tmp/m710-poc-step31-20260918/outputs/recovery_001/`。

服务器执行与离线证据目录：`/root/autodl-tmp/m710-poc-step31-v3-20260918/outputs/`。完整包在 `recovery_delivery/first_feasible/`，物理原始结果和视频在 `isaac_step31_once/`。复查脚本在该项目根目录、`PYTHONPATH=src` 下使用既有环境运行；它们读取这里记录的存档，不恢复已结束的旧世界。

原始实际状态、历史 motion、完整日志及视频不提交到 Git；原始文件哈希保留在摘要中。物理接收为 0、理想接收为 1；历史实际接收四箱没有被改写成理想接收或本轮新增成果。

# 本轮精简证据

完整说明见 [报告](../../m710_poc_legal_release_handoff.md)。源码提交为
`1d9f3fed5513197e4d66b116e224957ff06b6355`，唯一世界 `1789707683.8917766`，完整流程 1/5。
原状态和视频按记录中的绝对路径留在服务器。

| 文件 | 用途 |
| --- | --- |
| `old_transition_*` | 旧源码两个合法下降案例的错误拒绝、输入和源码 SHA |
| `targeted_final.txt` | 本轮九个定向测试模块，128 passed |
| `source_manifest.json`、`source_verification*.json` | 106 个源码/配置的 Git 字节哈希，运行前后核对 |
| `history_sources.json` | 历史候选来源及 SHA；不是本轮物理完成证据 |
| `continuation_search_quota.json`、`execution_config_pairing.json` | 实际有限配额和执行配置配对 |
| `continuation_driver.json` | 原 preflight 拒绝及同一世界暂停后重新接线 |
| `initial_world_scope.json` | 新世界、实际最高排身份、原始 40 箱状态文件 SHA |
| `initial_cpu_delivery.json`、`second_cpu_delivery.json` | 完整路径、耗时、preflight/export/readback 和输入 SHA |
| `physical_summary.json` | 逐箱事件、分别保存的释放/接管验收、计数、高度、耗时和原始文件 SHA |
| `state_event_retention.json` | 第一箱结束与第二箱失败状态的事件来源保留；失败箱仍附着 |
| `video_001_audit.json`、`video_002_audit.json` | 两段录像 SHA、分辨率、帧率、逐帧解码和时长核对 |
| `second_carton_failure_fixture.json` | 第二箱拒绝的实际 q、对象对姿态、上下文与策略 |
| `second_carton_failure_review.json`、`failure_replay_result.json` | 接触分类与几何距离离线复查及局限 |

在沿用的服务器 CPU 环境、仓库根目录执行：

```sh
PYTHONPATH=src /root/autodl-tmp/m710-official-dynamics-20260910/cpu-venv/bin/python \
  docs/evidence/m710_poc_legal_release_handoff/replay_second_carton_failure.py
```

这只重放保存输入上的生产几何查询和接触分类，不启动 Isaac，也不恢复已结束的世界。

# 2026-10-02 阶段验证优化与单次正式验证

同输入完整冷验证已通过：transit **510.394 → 91.212 秒（5.596×）**，extraction **156.211 → 70.262 秒（2.223×）**；采样网格和访问状态数不变。定向回归 **140 项通过**。新正式完整规划、443/443 非零边来源和全部交付检查通过，但这次唯一 Isaac 执行在抓取、抽离后因接触来源／连续性未解释停止；**未释放、未完成取放**。

详细说明见[报告](../../../m710_stage_validation_optimization_20261002.md)，正式机器摘要见 [summary.json](summary.json)。保留[上一轮成功证据](../m710_native_cold_20261002/README.md)与此前失败原件。

## 离线同输入基准

- [完整对照摘要](benchmark/comparison.json)：冷缓存、模型准备、实际访问与唯一状态、查询与复用计数、历史 tracker 一致性、反例拒绝。
- [与旧原始归档逐项核对](benchmark/input-archive-verified.json)：motion、实际场景、初始化状态、bootstrap 和 run 五个输入的 SHA-256 全部一致；未采用近似场景。
- [实际串行命令](development/launch-benchmark.sh)与[完整运行日志](development/benchmark-final.log)。每个模式只做一次完整对照；早期带 cProfile 的三边片段单列，仅用于归因。
- [冻结 optimized 源码身份](development/source-git-audit-optimized.json)、[reference 源码身份](development/source-git-audit-reference.json)、[传输核对](development/source-transfer-verified.json)、[最终回归](development/pytest-optimized-final.log)。

完整 `benchmark/final-reference.json` 和 `benchmark/final-optimized.json`（各约 7.4 MB）、初始／最终 tracker、完整来源审计，以及短片段 cProfile 原件位于 development 归档的 `benchmark/` 目录。没有热结果缓存测量，也没有把这些路径输入正式规划。

## 新正式运行

正式源码 `58eabd9b872dbb4ba6c1b7849d3298a8cbe971fc`；run `native-cold-3400e986d19047658f6396c46cae793e`；world `1790922679.8234546`；Task `native-cold-413ade2df3f54675a25f919f9dadda57`。完整 40 箱，目标 `carton_l07_c02`，种子 `71070`，官方 FANUC M-710iD/70、20 kg 工具和 42.5 kg 箱体。

- [实际启动脚本](development/launch-formal.sh)、[监督器命令](formal/commands.json)、[运行及预算](formal/run.json)、[本次输入身份](formal/input-manifest.json)。
- [分阶段独立验证剖析](formal/plan/independent-stage-profiles.json)为从原始 `motion.native_backend_evidence` 提取的只读派生记录；原始请求、路径、失败候选和完整来源均在 formal 归档中。
- [依赖及二进制身份](development/environment-identities.json)、[运行后源码核对](development/post-run-source-check.json)。原生和 CPU 包锁与上一轮一致；本次不升级依赖，不修改 Isaac 环境。
- [最终失败结果](formal/physics/result.json)、[首个阻塞摘要](development/execution-blocker-summary.json)、[完整接触窗口](formal/physics/stack_clearance_steps.json)、[安全停止反馈](formal/physics/runtime_feedback.json)和[实际事件](formal/physics/execution_events.json)。目标箱与 `carton_l06_c01` 在 76.504167 s 的 `CONTACT_PERSIST` 无法关联到连续合法接触，约 1 s 后停止；`confirmed_violation=null`，不能称为已确认穿透。
- [原始失败录像](formal/physics/replay.mp4)：本次实际抓取、抽离和失败前 transit，640×360、5 fps、1× 物理时间。实际运动 77.5 s，录像 387 帧／77.4 s，无拼接或转码；[元数据](formal/video-probe.json)与[全片解码检查](formal/video-decode-status.json)保留。没有另开世界或补跑。

本次抓取 1、释放 0、理想接收 0、理想出料 0；后两项为 **NOT_REACHED**。真实物理接收和实测驱动力矩资格为 **NOT_EVALUATED**，`workflow_cycle_completed=false`、`qualification_passed=false`。正常进程退出不等于流程通过，监督器最终状态为 `ISAAC_FAILED`。

## 完整原始归档与校验

| 归档 | 内容 |
| --- | --- |
| [formal-evidence.tar.gz](formal-evidence.tar.gz) | 根目录 `native-cold-validation-opt-once/`；完整 motion、preflight、replay_bundle、first_feasible、原生请求／输出和日志、初始化／实际执行事件、遥测、录像、USD。 |
| [development-evidence.tar.gz](development-evidence.tar.gz) | `evidence/` 含两份冻结源码快照、正式 ELF、实际包锁、环境／源码审计和回归；`benchmark/` 含同输入完整两版结果、片段剖析和对照。 |

[archives.json](archives.json)记录归档字节数与 SHA-256，[archive-members.json](archive-members.json)记录各成员，[manifest.json](manifest.json)覆盖交付文件。展开的原始记录逐字节与归档成员核对；派生摘要单独标识。正式 frozen-source-optimized.tar.gz 的 SHA-256 为 `c7fcfbfa6750c85cb8feb61803592a98d454eba098903aec45bf48039c260451`；reference frozen-source.tar.gz 为 `8d384361eba27fcda37791a53defb1a0753a0ab9ddca832e8fc7cc5d07c9ccd9`。

所有保存的轨迹仅供审计与明确授权的离线验证，不能成为后续 native-cold 的答案输入。离散检查不等于连续安全证明；暂停物理进行离线规划不代表在线连续规划。PoC、理想接收、真实物理接收和实测驱动力矩资格分别报告。

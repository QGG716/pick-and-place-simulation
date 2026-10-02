# 2026-10-02 原生阶段衔接与单次正式试验

本轮完成 **一次新世界、一次完整 native-cold 规划、一次 Isaac 单箱运动**。阶段身份修补和真实 worker 衔接通过；436/436 条非零运动边具有本次原生来源。实际抓取、搬运、释放、撤离完成，理想接收和理想出料各 1 次。真实物理接收与实际驱动力矩资格均为 `NOT_EVALUATED`，`qualification_passed=false`。

完整分层结论见[报告](../../../m710_native_cold_parent_chain_20261002.md)与[机器可读摘要](summary.json)。旧正式失败及其证据保持不变；开发探针不计入正式成功数。

| 身份 | 本次记录 |
| --- | --- |
| 正式源码提交 | `f94a1ca4c48b4b4bf5805466549c941ed07dd830` |
| Run | `native-cold-c66166b61f2548bfabe896a5010fa444` |
| World | `1790909283.6163392` |
| Native Task | `native-cold-1c240bb2901b431a91c624133df99eca` |
| 目标／随机种子 | `carton_l07_c02` / `71070` |
| 场景 | `m710id70_unloading_layout_v1`，完整 40 箱 |
| 模型与质量 | 官方 FANUC M-710iD/70，工具 20 kg，目标箱 42.5 kg |
| Worker SHA-256 | `e933e868ac5c50879847dc1161046b6b9ad081adf23fe4f6359338bc9caf723c` |

## 直接审阅

- [阶段来源与实际调用](formal/plan/source-coverage-summary.json)：17 次原生 IK、5 PTP、5 LIN、2 OMPL；选中父链 `:1 → :2 → :3 → :10 → :11 → :12`。历史读取、旧运动生成器调用、禁止入口尝试均为 0。
- [真实 worker 定向探针](development/parent-probe-summary.json)：17 项通过；包含完整 40 箱场景的 PTP→LIN 和独立稀疏合成场景的附着／释放所有权检查。所有输入当次新生成，Isaac 未运行。
- [最终 101 项 Python 回归](development/pytest-source-final.log)、[原生编译与 11 项工艺检查](development/build-parent-final.log)。早期探针失败保留在 development 目录及归档内。
- [实际启动脚本](development/launch-formal-20261002.sh)、[监督器命令](formal/commands.json)、[正式运行状态与有效预算](formal/run.json)、[输入身份](formal/input-manifest.json)、[源码清单](formal/source-manifest.json)。
- [最终执行结果](formal/physics/result.json)、[执行前同世界绑定](formal/physics/initial_native_plan_binding.json)、[抓取／约束移除事件](formal/physics/execution_events.json)、[理想接收／出料事件](formal/physics/ideal_transport_events.json)。同世界绑定记录针对执行包加载时刻；后续有明确登记的有界理想接收修正。
- [原始单次视频](formal/physics/replay.mp4)：640×360、5 fps、615 帧、123.0 秒、1× 物理时间，无拼接或转码。[元数据](formal/video-probe.json)与[全片解码检查](formal/video-decode-status.json)已保留。物理仿真为 240 Hz / 29,552 步 / 123.133 秒。

## 完整原始归档

为避免直接提交数百 MB 的重复 JSON，较小控制文件、结果、事件、日志、图片和原视频直接展开；完整路径、原生请求／返回证据、执行包、遥测和源文件快照完整保存在以下归档。展开文件与原始归档成员保持字节一致；摘要是另行生成的只读派生文件。

| 归档 | 内容与内部路径 |
| --- | --- |
| [formal-evidence.tar.gz](formal-evidence.tar.gz) | 根目录 `native-cold-20261002-once/`；`plan/motion.json` 含完整路径、所有原生记录、来源和冷启动保护计数；`plan/preflight.json`、`plan/replay_bundle.json`、`plan/first_feasible/`、`native-requests.jsonl`、完整 `physics/`、USD 与监督器记录。 |
| [development-evidence.tar.gz](development-evidence.tar.gz) | 根目录 `evidence/`；`frozen-source.tar.gz` 是实际传输并冻结的 335 文件源码快照；`m710_moveit_worker` 是正式 ELF 二进制；另含依赖锁、编译／回归日志、全部开发探针结果和请求。 |

[归档校验](archives.json)给出大小与 SHA-256，[归档成员清单](archive-members.json)列出完整内容，[交付文件校验清单](manifest.json)覆盖本目录所有文件（清单自身除外）。源码快照 SHA-256 为 `12fdf31623c69f5935dfde1078cb91372caef35ec7a2c5e2c03461e0929de955`；[源码 Git 对照](development/source-git-audit.json)记录实际实现字节一致，以及十个未改动文件的 CRLF 差异。

依赖锁见 [ROS/系统包](development/packages.lock)和 [Python 包](development/python-packages.lock)。沿用既有 Ubuntu 22.04 / ROS 2 Humble 隔离环境，无依赖升级。正式请求使用新 worker；允许静态模型预热，未导入开发探针 Task 或轨迹缓存。

上述轨迹和探针仅用于审计，不得成为后续 native-cold 请求的解答输入。规划期间物理世界暂停，本次结果不代表在线连续规划。

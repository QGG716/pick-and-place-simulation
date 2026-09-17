# 一次性感知运行的退出状态（2026-09-17，第六步）

分支 `feat/v0.5-perception-ros2`；开始时读取 AGENTS.md、fetch，工作区干净，
本地与远端均为 `19329cada68c03547719dc98d020b6ff065f852f`。
生产修改仅 `tools/run_workcell_perception_once.py`。前五步、执行门控、分割/几何阈值和 ROS 接口未改。

## 原始失败回归

先新增 CPU 替身测试，再修改生产脚本。实际调用旧 `main()`：第一模组的 infer 替身抛出
`RuntimeError: CPU_TEST_SAM_FAILURE`，第二模组正常返回合法的无可接受面片结果。
旧 summary 两行分别是 `TECHNICAL_FAILURE`、`COMPLETED_WITH_ALGORITHM_RESULTS`，
但实际返回 **0**，新增退出码断言失败（1 failed）。修复后相同反例返回 **1**，第二行结果仍保留。

模型、worker 和图像边界运算是明确标注的测试替身；main、manifest 校验、异常处理、
状态归约、汇总写入和 CLI/SystemExit 均使用生产实现，没有实际 SAM/MoGe 推理。

## 退出码与结果契约

- **0**：非空有效 manifest 中所有模组技术完成，必需结果可读，最终汇总保存成功。
- **1**：任一模组失败，或初始化、输入/结果读取、报告保存等运行环节失败。
  最后一个模组成功不能重置前面的失败；合法空模组 manifest 也不能成功。
- 参数错误保留 argparse 的 **2**。KeyboardInterrupt 在 main 中记录后重新抛出，CLI 映射为 **130**；
  下游 SystemExit 保留 1～255 的非零码，0/None/非标准码转为 1 并保留原因，不能把未完成任务当成成功。

`infer()` 正常返回 None，将 `COMPLETE` 响应写入文件；非 COMPLETE 会抛异常。
`_worker_artifacts()` 检查响应、输入/模型/产物哈希和来源关联。
`_run_secondary_module()` 正常返回 observed_face_sets/observation 等字段并写 rgbd_cuboids.json，
worker 非零、超时、读写错误主要通过异常报告。脚本也拒绝显式 FAILED/TECHNICAL_FAILURE 等技术状态，
包括返回的 observation 状态；检查本调用依赖的实例列表、camera_facing_faces、accepted 及面片集合。
JSON 实例数按该直接下游的逐掩膜输出契约核对，不新增几何精度门槛。

accepted=false、无可接受面片、完整体数为 0 都是合法算法结果，仍可返回 0。
真实来源关联函数 `load_instance_lineage()` 接受结构合法的空实例/掩膜集合；测试替身的正常和空产物
也经过该真实函数校验。现有几何点图构建器要求至少一个掩膜，因此**经过成功 SAM 响应、产物和掩膜
结构校验的空分割**直接保存带 `EMPTY_SEGMENTATION` 原因的空实例结果，metric_attempts=0。
异常、缺文件、损坏文件不会走此分支，不以伪造空结果掩盖失败。

保留 `raw_image_automatic=false`、`ISAAC_GROUND_TRUTH_ORACLE_PROPOSAL` 和
`planning_admissible=false`。退出 0 不表示算法通过、完整体已认证或可执行；没有修改 candidate_eligible。

## 汇总、隔离和资源

保留 `model_manifest`、`runs` 及原有每模组结果字段。新增运行 ID、阶段、错误列表、overall_status、
exit_code、finalized，以及 expected/completed/failed/not_run/running 计数。
模组状态为 NOT_RUN → RUNNING → COMPLETED_WITH_ALGORITHM_RESULTS / TECHNICAL_FAILURE，
失败记录包含 failure_stage、error_type、error。已完成结果保留，未开始模组记录停止原因。

创建新输出目录后立即写初始汇总；有效 manifest 读入后登记完整名单，再初始化模型。
manifest 尚不可读时 expected 为 null，表示未知；空 manifest 的 expected 为 0 并明确失败。
RGB 哈希、模组目录创建、输入复制和各处理阶段都处于统一异常路径。每阶段及每模组结束保存进展。
进展报告 finalized=false、exit_code=1；只有最终技术成功且原子写入成功，才发布 finalized=true、exit_code=0。

一般模组故障继续收集另一独立模组。共享初始化/汇总存储失败或中断停止后续推理，剩余模组不伪装成功。
summary 使用同目录临时文件、UTF-8、flush/fsync 和 os.replace。保存失败写 stderr 并维持非零；
最后最多尝试一次发布“报告写失败”本身，绝不重新尝试发布成功状态。
存储持续不可写时只能保留最后的未完成进展或没有报告，stderr 明确报错；原始模组异常已单独打印，
不会被二次写入失败遮蔽。临时文件清理失败单独报告，不覆盖原异常。

仍拒绝已有 `capture/perception-once`，不删除或修改旧运行和历史报告。
本轮只复制六个必需捕获输入到新建的 `perception-once/<module>`，新生成的 oracle proposal、
worker 响应和几何结果都留在该目录；采集原目录中的旧响应/旧 rgbd_cuboids.json 完全不消费、不改写。
每模组新增 artifact_directory 指明位置。SAM 文件必须在本轮 sam-runs 中且匹配本次 request_id，
其产物必须属于该 request 的目录，再由原有产物校验器验证哈希和来源。

每模组至多调用一次 infer 包装器和一次几何入口，没有推理重试。
sam_attempts/metric_attempts 记录实际入口调用数；目录、哈希、初始化失败不计为推理。
NumPy 归档使用上下文管理关闭，图像写入返回 false 也报技术失败。
ResidentRuntime 没有 in-process close/shutdown API；脚本释放本次引用，进程退出回收模型，
没有伪造协议 shutdown 方法或重写 worker 生命周期。

## 验证与调用边界

新增 51 项编排测试，覆盖正常、合法零完整体/空分割、两种部分失败顺序、全部失败、初始化、
输入/目录故障、输出损坏/缺结构、显式失败状态、残留结果、已有输出目录、原子汇总写入失败、
进展写入失败停止后续调用、KeyboardInterrupt/SystemExit 和一次调用约束。
`tests/workcell_once_cli.py` 仅注入测试替身后用 runpy 执行真实脚本的 `__main__`；父进程通过
subprocess.run 检查真实 returncode、summary 和 stderr：成功 0、部分失败 1、中断 130、下游 SystemExit(0) 转 1。
缺必需 CLI 参数直接启动原脚本，验证 argparse 返回 2。

| 检查 | 实际结果 |
|---|---|
| 旧脚本失败回归 | 1 failed；记录失败却返回 0 |
| 首批新脚本测试 | 49 passed，23.05 s |
| 脚本及来源关联、handoff、融合相关回归 | 94 passed，34.17 s |
| Windows 默认 GBK 首次全量 | 554 passed，1 failed，1 deselected，70.99 s；既有 test_effective_scene.py 的 UTF-8 文件解码失败 |
| Windows PYTHONUTF8=1 最终全量 | 557 passed，1 deselected，64.04 s |
| 服务器 CPU 最终全量 | 557 passed，1 deselected，12.15 s |
| 服务器全部 tests_metric | 31 passed，1.10 s，包含第四步正常/冲突完整体回归 |

首批检查后补充了两个初始报告/运行身份初始化失败测试；最终两端全量包含全部 51 项新测试。
未增加 skip 或放宽断言；1 deselected 是项目原有默认 marker 排除项。未扩展为 Windows 全仓编码整改。

```powershell
.venv310/Scripts/python.exe -m pytest -q -s tests/test_workcell_perception_once.py --basetemp .test-tmp/step6-red
.venv310/Scripts/python.exe -m pytest -q tests/test_workcell_perception_once.py tests/test_instance_lineage.py tests/test_algorithm_surface_contract.py tests/test_fusion_face_reduction.py --basetemp .test-tmp/step6-related
.venv310/Scripts/python.exe -m pytest -q --basetemp .test-tmp/step6-default
$env:PYTHONUTF8='1'
.venv310/Scripts/python.exe -m pytest -q --basetemp .test-tmp/step6-utf8
```

服务器在 `workcell-exit-20260917/repo` 中使用基线提交的隔离归档及四个上传的脚本/测试文件；原仓库未改。
运行 `env -u PYTHONPATH /root/autodl-tmp/v05-acceptance/cpu/venv/bin/python -m pytest -q`，
metric 复用 `boundary-evidence-20260917/venv/bin/python`，以
`PYTHONPATH=src:packages/unloading_contracts/src:tools:.` 执行 `-m pytest -q tests_metric`。
两个日志在隔离目录上一层 default.log/metric.log；上传文件与本地 SHA-256 一致。

搜索 Python/shell/PowerShell/workflow 未发现直接调用本脚本的仓库入口，只有历史文档中的命令，未虚构调用方。
外部调用应检查进程退出码；若通过管道保存日志，使用 `set -o pipefail`，不能只检查 tee 的退出码。
summary 的 finalized=true 表示终态，不能把运行中进展当成最终成功记录。

没有启动 Isaac、SAM/MoGe、训练、重采集、视频或规划；替身验证不代表真实感知质量验收。
ROS 接口未变；复用现有 Humble CI，其当前提交的实际查询状态在交付回复报告。
未修改历史归档，未开始第七步 RViz。

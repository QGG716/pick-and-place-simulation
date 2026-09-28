# Workcell 几何子进程接入

基线 `676ea475aab78394df36d8d3dbb14a107da4889b`，分支
`feat/v0.5-perception-ros2`。本轮仅改变运行边界、结果传递和验证工具；
几何生产文件与历史 B-blas-1 的文件 SHA 完全相同。版本保持
`0.5.0.dev0`，ROS 包、消息及独立契约不升版。

## 入口与职责

`workcell_video_worker.RuntimeCache` → `run_workcell_perception_once.main`
→ SAM 推理与已有产物/实例谱系检查 → 共享 `dispatch_geometry`
→ 新 Python 解释器 → 原 `_run_secondary_module`
→ 父进程验收 → 原融合与 `publish_algorithm_run`。

单组和视频 worker 均已接线。默认 `inline` 保持旧行为；显式选择：

```bash
python tools/run_workcell_perception_once.py \
  --capture CAPTURE --vision VISION --models MODELS_JSON \
  --output-directory NEW_RUN \
  --geometry-backend subprocess --geometry-blas-threads 1 \
  --geometry-python /root/v05-gpu-venv/bin/python --geometry-timeout 1200
```

`tools/workcell_video_worker.py` 接受并透传同样的四个选项。
subprocess 下省略 `--geometry-blas-threads` 则继承；inline 下指定线程数
在读取输入、初始化模型前报错。超时沿用原几何任务的 1200 秒默认值，
现在覆盖整个子进程的输入检查、计算和结果写出。

每个非空模组只启动一个几何进程，同一调用串行等待、验收后才处理下一模组。
没有进程池、跨帧缓存、自动重试或失败回退。空 SAM 分割保持原语义：
零次几何计算、显式 unknown；有 mask 但无认证面片仍是合法算法结果，
与技术失败分开。父级仅在全部模组成功后发布一次融合 artifact。

Python 路径使用 `absolute()`，保留 venv 符号链接；fresh exec 不继承
SAM 的 CUDA 运行状态。子环境移除 `PYTHONHOME`，将 `PYTHONPATH` 明确设为
当前代码的 tools、src 和 contracts/src；其他环境保持继承。
父进程不调用 BLAS setter，不修改父环境或 OpenCV 调度。
Native OpenBLAS 策略只在子进程设置一次，计算前后均通过实际 getter 核查。
本轮实际数值验证环境为 Linux，未认证 Windows 数值库后端。

原 `run_isaac_rgbd_geometry.py` 的数学与几何入口没有复制或改写。
固定输入 A/B 启动器没有成为生产 worker。ROS/RViz 启动器、视频 demo UI
及其他常驻/在线入口未接入本轮新选项。

## 请求、结果与失败

小型 JSON 请求绑定新的 task/run 身份、module、原始 capture/epoch/sequence/
capture_time、输入 SHA、SAM metrics/masks/instances/proposals、标定 K 与采集时
T_W_C、完整 manifest、有效配置及其指纹、BLAS 策略和本次输出目录。
子进程重新加载绑定 RGB-D，核查实际读取的原始字节，重新执行已有 SAM
引用/版本/谱系校验；不使用 pickle，也不传送大数组。

新采集入口使用父进程刚创建的快照；历史 SAM 验收保留原谱系路径，
由原几何入口写入新的输出快照。原始输入、旧 SAM 和历史输出只读。
几何产物、契约序列化的模块 observation、ObservedFaceSet、线程报告先写完，
`completed.json` 最后原子发布。父进程要求退出码 0，并核验完成状态、PID、
request SHA、task/run/capture 身份、固定输出引用、文件 SHA、域结构、实例清单、
配置指纹和线程报告。仅目录里存在一个 JSON 不构成成功。

模块 observation 与面集恢复为原领域数据，供既有融合直接消费。
大 pointmap、mask、hypotheses 和完整 payload 不跨进程传递。
`metric_runtime_policy` 与明确的 `geometry_runtime` 进入正常配置指纹，
父进程、模块、融合 artifact 和校验器使用同一真实配置身份。

超时、stop-worker、KeyboardInterrupt、SIGTERM、启动失败或非零退出均失败关闭，
保留 stderr 与失败阶段。只终止并 wait 本次拥有的子进程；POSIX 清理其独立
进程组，Windows 清理该 PID 树。stop-worker 后不继续启动下一模组。
不可捕获的宿主机断电/SIGKILL 不属于本轮协作式关闭保证。

## 一次真实验收

这是**复用 SAM 产物的几何子进程验收**，没有运行 SAM、MoGe、Isaac、ROS、
RViz、完整视频、执行桥或机器人，也没有再次做 inherit/1 性能 A/B。

输入仍是原固定计划 `metric-blas-20260922/fixed_plan.json`：
capture-f24f63a，上 16、下 31，2592×1944，原 depth/K/T_W_C/mask、顺序、seed、
上游版本和全部算法参数。计划 SHA
`a7578fe294008dfda82b3181c4dd71477b8ce9adbdcf658e97661d8fd1664a0e`。
输出位于服务器：
`/root/autodl-tmp/v05-acceptance/workcell-geometry-process-20260928/subprocess-blas-1-verified`。

首次前置检查发现 git archive 改变了历史混合换行文件的字节 SHA；
该次 0 个模组、0 个实例开始计算，0.367 秒即失败。逐文件确认仅换行差异后，
上传当前工作区原始字节，使生产 SHA 精确匹配，再在独立新目录运行一次几何。
前置失败报告保留在本轮证据中，未改比较规则或历史报告。

| 项目 | 上模组 | 下模组 |
|---|---:|---:|
| 几何 PID | 20938 | 21136 |
| 实例 / 面片 | 16 / 23 | 31 / 48 |
| 父级输入核查，秒 | 1.369 | 2.400 |
| Popen 启动，秒 | 0.0011 | 0.0011 |
| 子级输入加载/核查，秒 | 1.606 | 2.718 |
| 线程初始化/验证，秒 | 0.093 | 0.074 |
| 原完整模块几何调用，秒 | 34.054 | 67.360 |
| 父进程等待子进程，秒 | 36.490 | 71.603 |
| 父级结果接纳，秒 | 0.376 | 0.801 |
| 调度整体，秒 | 36.897 | 72.432 |
| 子进程峰值 RSS，KiB | 822452 | 1388976 |

组级墙钟 **116.299 秒**，包含前置核查、输入读取、启动/等待/结果接纳、
融合与发布（2.571 秒）、artifact 重新加载校验（0.257 秒）及最终原输入 SHA 核查。
严格历史比较在交付后另行执行。上表为包含关系，不能将父子计时直接相加。
本次没有同环境同步 inline 性能基线，不作加速倍数结论。

两个子进程 NumPy/SciPy OpenBLAS 均为 **64 → 1 → 1**（设置前/计算前/计算后），
OpenCV 始终 **22**，非目标 OpenBLAS 为 **1**，环境变量和 affinity 未改变。
Python 3.10.12、NumPy 1.26.4、SciPy 1.14.1、OpenCV 4.10.0。
affinity 可见 208 CPU；可读 cgroup v2 `cpu.max=2200000 100000`，即 22 CPU 时间配额。
未挂载父级限制未知。报告中的 cpu.stat 是共享 cgroup 汇总，不能归因于本任务；
已创建线程数、配置线程数也不代表同时执行量。

新增 `geometry-subprocess` 严格比较模式核验原 B 的 BLAS=1、两子进程的真实
线程报告、新配置身份、完整计划和相同几何生产 SHA 后，只归一化已声明的
运行路径、处理时间、run/config 身份和 `geometry_runtime`。
47 个完整几何记录（包括 cost）、面片、支持、残差与判定、模块 observations、
融合关系及 unknown **精确一致，0 个差异**。未引入 allclose、舍入或 cost 例外；
旧严格模式和数值复核模式的规则不变。

结果保留 **37 个融合对象、84 个 unknown、0 个完整箱体接纳**。
ORACLE 提示来源、`raw_image_automatic=false`、历史采集时间与回放门控保留；
`planning_admissible=false`。新运行身份与哈希不同，不能复用旧 plan/grant。

小型报告、线程原始记录与大产物 SHA 位于
[本轮证据目录](evidence/workcell-geometry-process-20260928/evidence_manifest.json)，
完整几何数组留在服务器，没有复制到仓库。

## 测试与范围

本地命令（Windows，使用仓库内临时目录避免默认 TEMP 权限问题）：

```powershell
.venv310/Scripts/python.exe -m pytest -q --basetemp=tmp/workcell-geometry-final-suite
```

本地默认套件 **972 passed、3 skipped、1 deselected**（173.41 秒），
随后新增的快照/空分割两项补充测试均通过。详情见
[test_results.json](evidence/workcell-geometry-process-20260928/test_results.json)。服务器定向命令：

```bash
PYTHONPATH=/root/autodl-tmp/v05-acceptance/metric-profile-20260922/test-deps:tools:tests \
/root/v05-gpu-venv/bin/python -m pytest -q \
  tests/test_workcell_geometry_process.py tests/test_metric_thread_policy.py \
  tests/test_workcell_perception_once.py tests/test_video_demo.py \
  tests/test_metric_faces_ab_comparison.py tests/test_metric_blas_cost_review.py \
  tests_metric/test_offline_payload_consumption.py
```

服务器 **174 passed**，随后两项补充测试也均通过。覆盖实际 once/video 参数透传、两次任务复用一个模型、
每模组单次调用、真实 exec PID/环境、原生数值库父子隔离、超时/停止/中断回收、
旧/错身份、坏 SHA、缺完成标记、不完整实例、非法线程、空分割与技术失败，
以及新旧比较模式和既有独立最终验证。
测试中的模型/部分几何替身明确是 CPU 编排测试，不是真实 SAM 连续推理。
服务器首次测试包漏带既有 manifest fixture，补齐后重跑通过；未重建真实输入。

可显式在已接入的独立几何路径选择单线程，默认仍继承/inline。
本轮未验证真实常驻 SAM 与子进程同时工作的显存/内存压力、完整视频时序、
ROS、真实硬件或未来输入逐位复现；不宣称 RGB→ROS 端到端或实时感知达标。

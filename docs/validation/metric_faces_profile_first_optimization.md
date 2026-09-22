# 米制面片函数剖析与支持区域局部计算

## 基线与唯一改动

实际基线 `02a20ad856f9910d8c1e093e51c6502beff2a27b`，分支始终为
`feat/v0.5-perception-ros2`；开始时工作区干净，fetch 后远端一致。已读取根 AGENTS.md，相关目录无额外指令。
此前执行上下文、取消/停止、尺寸先验、深度连通域/单次滤波和可选旧诊断均保留。
所有本轮真实性能运行都设置 `legacy_cuboid_diagnostic=false`。

先完成完整固定输入 P 剖析，保存 `selected_hotspot.json` 后才修改生产代码。
**唯一生产热点改动位于 `metric_support.observation_support()`**：
深度间断及腐蚀从整幅图运算改为当前 `instance & face` 的包围框加一像素邻域内运算。
输出仍为原分辨率两个 bool 掩码和完全相同的审计字段，没有重采样、减实例、改阈值或改变支持定义。
其余改动仅为本轮测量/比较入口、测试和说明，不修改其他算法阶段。

## 实测定位：不是最小二乘拟合

P 使用 cProfile；这些数值只作定位，**不与无 profiler 的 B 比较加速**。
单位秒；累计时间包含子调用，表内父子项不能相加。

| 实际调用位置 / 函数 | 次数 | cProfile 自身 | 累计 |
|---|---:|---:|---:|
| 支持选择 `observation_support` | 660 | 116.035 | 127.577 |
| 平面残差检查 `check_metric_plane` | 430 | 1.578 | 105.230 |
| 拟合返回与独立转换中的 `validate_metric_record` | 93 | 8.081 | 83.776 |
| 每实例 `fit_metric_faces`（含支持、构造及返回前验证） | 47 | 4.124 | 135.458 |
| 标签/初始平面提取 `extract_observation_labels` | 47 | 1.631 | 47.993 |
| 观测转换 `_observation`（含独立验证） | 2 | 0.024 | 51.265 |
| 转换入口 `validate_final_record` | 47 | 0.001 | 41.993 |
| 谱系读取及核验 `load_instance_lineage` | 4 | 0.030 | 5.290 |
| NPZ `__getitem__`（整次运行） | 85 | 0.001 | 1.967 |
| 支持绑定 `_binding` | 140 | 0.001 | 3.207 |

由调用位置记录的其他轻量包含计时：固定上游 `fit_planes` 91 次 / 32.818 秒；
`_orthogonal_fit` 37 次 / 0.914 秒；最大观测矩形 71 次 / 3.453 秒；
`_boundary_kinds` 93 次 / 1.227 秒；`_complete` 13 次 / 1.309 秒。
这些仍然都是剖析运行统计，不是修改版性能数字。
动态导入的上游文件存在相同 filename/line/name 的函数，标准 pstats 合并键可能覆盖该行；
上游完整次数/包含耗时以上述调用包装器为准，不将 pstats 中某一次导入的行冒充全部 91 次。
所选 `observation_support` 是单一模块函数，其 660 次同时由轻量包装器和 cProfile 核对。

660 次支持计算分布为：提取阶段 47 次、拟合阶段 471 次（含返回前验证）、
后续独立最终验证 142 次。剖析中包含耗时分别为 8.534、92.119、28.262 秒。
这说明支持区域的整帧数组运算同时拖慢两个主要区间；没有据函数名称假定最小二乘是瓶颈。
输入解压、谱系读取、标签/平面提取、拟合、面片/边界构造、独立验证和输出转换均单独保留测量。
输出 JSON 编码/写出与点图压缩仍在原阶段内，未关闭审计来制造加速。

## 等价性与生命周期

原调用关系为 `extract_observation_labels` / `fit_metric_faces` / `check_metric_plane`
→ `observation_support`；拟合返回前和 `_observation` 的独立检查都会再次走这条路径。
调用关系、次数、验证内容不变，仅缩小单次函数内部不必要的工作范围：

- `selected = instance & face` 和深度合法性仍在本次调用中重新计算。
- 包围框之外 `selected` 恒为 false，腐蚀的零填充与原全图相同，包括真实图像边缘和多次腐蚀。
- 深度连续性只依赖上下左右相邻边；一像素邻域涵盖所有选中像素的真实邻居，包含实例外像素和无效深度。
- 减法仍使用相同 NumPy 输入 dtype、float 输出和严格 `> discontinuity_m` 判断；不改变浮点求值公式。
- 掩码与审计在同一候选无关定义下计算，最终仍回填完整图像形状。空区域保留原错误检查与空输出语义。

没有跨调用准备数据、全局缓存、外部“已验证”参数或旧 PASS 复用。
局部 NumPy view 只活在当前函数栈内，不修改调用方数组；函数返回后不保留它们。
因此不依赖 frozen dataclass 来宣称数组不可变，也不要求新增缓存身份。
改变原始数组内容后，下一次调用使用新内容；文件重新加载仍经过原哈希、来源和采集身份边界。
`validate_metric_record` 的深度/K/mask 绑定、从最终坐标重算法向/残差/支持/边界，以及完整箱体检查均未删除。

## 固定输入与可复现入口

继续使用 `capture-f24f63a` 的既有已绑定 RGB-D/SAM，上模组 IDs 1–16、下模组 IDs 1–31，
全部 2592×1944，输入、K、采集 T_W_C、seed=17、固定上游版本不变。
掩码像素数范围 670–96,789；所有 47 个实例的像素数、初始平面标签数、连通标签数、
最终面数、接受/拒绝及主要函数计时均在 P/A/B `function_timings.json` 中，未筛选成功对象。
`initial_plane_count` 是返回 seeds 中不同 initial_plane_index 的数量，`connected_label_count` 是 seeds 总数；
两者不是最终面数，也不将未形成有效标签的 RANSAC 尝试算作观测面。

本轮独立目录：`/root/autodl-tmp/v05-acceptance/metric-profile-20260922/`。
`baseline-code` 来自实际基线；`modified-code` 的生产函数仅 `metric_support.py` 变化。
完整大产物留在 `P-profile`、`A-baseline`、`B-support-window`；不覆盖历史输入/summary/binding。
`profile_metric_faces.py` 包装现有 `validate_metric_faces_ab.run`，仍调用真实生产入口、融合发布及正式 artifact loader；
不另建算法流水线。计时不写入几何记录或其指纹。

```bash
cd /root/autodl-tmp/v05-acceptance/metric-profile-20260922
/root/v05-gpu-venv/bin/python baseline-code/tools/profile_metric_faces.py \
  --plan fixed_plan.json --output P-profile --profile
/root/v05-gpu-venv/bin/python baseline-code/tools/profile_metric_faces.py \
  --plan fixed_plan.json --output A-baseline
/root/v05-gpu-venv/bin/python modified-code/tools/profile_metric_faces.py \
  --plan fixed_plan.json --output B-support-window
/root/v05-gpu-venv/bin/python modified-code/tools/compare_metric_faces_ab.py \
  --before A-baseline --after B-support-window --plan fixed_plan.json \
  --mode hotspot-off --code-change code_change.json --output comparison.json
```

三个真实流程各运行一次。P 开重型 profiler；A/B 均关闭，使用完全相同的轻量函数计时包装器。
轻量计时的 `excluding_instrumented_children_seconds` 仅扣除其他被包装函数，包含未包装的 NumPy 等操作，
不是 cProfile 的 Python 自身耗时。不能把它与 cProfile self 字段混用。
原 `elapsed_seconds` / `legacy_geometry` 仍为既有几何区间，模块总计和交付墙钟另列。

比较器保留原默认“开/关”模式及测试；新 `hotspot-off` 要求双方诊断关闭、计时清单完整、
重型 profiler 均关闭、环境匹配，且仅声明的一个生产文件具有指定前后 SHA。
完整几何、模块观测、融合及正式加载结果仍精确比较；没有增加几何/残差白名单或数值容差。

## 环境与测量限制

同一 `/root/v05-gpu-venv/bin/python`：Python 3.10.12、NumPy 1.26.4、SciPy 1.14.1、OpenCV 4.10.0。
Linux CPU 可见/affinity 为 208 个，cgroup `cpu.max=2200000 100000`（约 22 核时间），
内存上限 118111600640 bytes。可见 CPU 并非独占资源。
OMP/OPENBLAS/MKL 等线程环境变量均未设置，threadpoolctl 不在环境中，未为性能实验安装它。
另用同解释器的只读原生 getter 查询：NumPy OpenBLAS 64、SciPy OpenBLAS 64、
OpenCV 自带 BLAS 1，OpenCV 调度线程 22，见 `numeric_threads_complete.json`。
这是一份独立环境探针，不冒充在每个性能进程内部测到的瞬时线程数；本轮没有改变线程配置。
打包/传输轻量测试文件和短只读环境查询发生在运行期间；未并行运行另一套几何算法或服务器测试。
测试依赖从本地现有纯 Python pytest 复制到独立 `test-deps`，未安装/升级运行环境数值库。

## 无 profiler A/B 与结果

严格比较退出 0：`plan_complete=true`、`results_equal=true`、`differences=[]`。
全部 47 个实例的完整几何记录精确一致，上模组 23 张、下模组 48 张面片。
lower mask 1 仍无有效面片；完整箱体 accepted 均为 0。没有将“两版相同拒绝”当作成功重建。
两组融合均为 37 个对象、84 个 unknown；来源绑定、采集时刻和历史门控保持不变。
仍为 ORACLE-PROMPTED / `raw_image_automatic=false` / `planning_admissible=false`。
两次真实运行和一次剖析均成功，没有换样本、重跑挑最快或增加比较容差。

单位秒；分项合计上下模组。最后三行是包含区间，不与前面重复相加。

| 范围 | A 基线 | B 支持窗口 |
|---|---:|---:|
| 初始输入哈希/环境核验 | 0.0651 | 0.0621 |
| 输入加载、绑定和 worker 产物核验 | 4.0519 | 4.2024 |
| 模块输入检查/快照 | 0.0557 | 0.0604 |
| 点图构建（含逐箱审计 NPZ） | 42.5835 | 43.2566 |
| 合并点图 NPZ 写入 | 1.8078 | 1.8278 |
| 旧诊断 | 0 | 0 |
| 最终面片阶段 | 171.5715 | 99.8689 |
| 最终校验 / 观测转换 | 45.9521 | 25.7019 |
| 评价 / 观测写入 | 1.7468 | 1.7630 |
| 滤波审计 JSON 写入 | 0.0017 | 0.0017 |
| 融合及 artifact 发布 | 2.5436 | 2.4999 |
| 正式 artifact 加载核验 | 0.3117 | 0.2656 |
| 最终输入哈希复核 | 0.0405 | 0.0373 |
| 上模组模块总计 | 87.8923 | 59.8770 |
| 下模组模块总计 | 175.8805 | 112.6575 |
| **完整几何交付墙钟（summary 写入前）** | **270.8686** | **179.6891** |

唯一热点 `observation_support` **660 → 660 次**，轻量包含时间
**124.7566 → 8.7785 秒，减少 115.9781 秒（92.96%）**。
几何面片区间减少 71.7026 秒（41.79%），最终校验/转换减少 20.2503 秒（44.07%），
完整交付减少 **91.1795 秒（33.66%）**。局部收益没有直接当作整体同比例收益。
拟合返回前 `validate_metric_record` 46 次、独立转换再验证 47 次，前后均保留；
无组的 lower mask 1 仍走独立检查。所有函数/实例记录见证据，不只列最快或最慢对象。

Linux 进程生命周期峰值 RSS：A=1,602,968 KiB（1565.40 MiB），
B=1,618,988 KiB（1581.04 MiB），**增加 16,020 KiB（15.64 MiB）**。
这包含解压掩码、点图、拟合、观测和 Python/NumPy 分配器保留内存，不是单函数内存峰值。
局部 `difference`/`jumps`/腐蚀数组的尺寸受窗口约束，但未测量逐分配器峰值，不能据此声称整体内存降低。

A 的系统 load average（1/5/15 分钟）从 (17.56,19.55,18.66) 到 (36.31,28.28,22.50)，
B 从 (17.88,21.97,21.12) 到 (20.42,21.53,21.11)。没有独占主机、没有连续资源隔离测量；
每版仅一次，收益是该次观测，不是稳定速度保证。

最大交付阶段仍为最终面片计算 B=99.87 秒，其次点图与逐箱审计 43.26 秒。
被细分的函数中 `check_metric_plane` 合计 430 次 / 36.57 秒（含子调用），
固定上游平面提取 91 次 / 24.90 秒。剩余支持扫描、数值运算和压缩均只记录，不开展第二项优化。

证据目录 [evidence/metric-profile-20260922](evidence/metric-profile-20260922/) 包含原始 P/A/B 汇总和
47 实例计时、函数剖析、固定清单、修改前热点选择、精确比较、线程探针及代码/产物哈希。
本轮固定计划 SHA256：`fa8b699ce71586c597a7241329ce5c89c57241f564828fff93a126cfd8f32f24`。
修改前后 `metric_support.py` 文件 SHA 见 `code_change.json`；输入 SHA、实际解释器和上游身份见固定计划。

## 测试与失败记录

```powershell
$env:PYTHONUTF8='1'
.venv310/Scripts/python.exe -m pytest -q tests/test_metric_support_window.py tests/test_metric_support.py tests/test_metric_patch_adapter.py --basetemp=tmp/pytest-metric-window
# 46 passed in 0.85s
.venv310/Scripts/python.exe -m pytest -q tests/test_metric_faces_ab_comparison.py --basetemp=tmp/pytest-hotspot-comparator
# 13 passed in 3.74s，保留原开/关模式测试
.venv310/Scripts/python.exe -m pytest -q --basetemp=tmp/pytest-support-window-full
# 862 passed, 1 deselected in 135.70s
```

服务器几何专项（在 B 开始前完成，未与 A/B 性能运行并行）：

```bash
cd modified-code
PYTHONPATH=../test-deps:tools:tests /root/v05-gpu-venv/bin/python -m pytest -q \
  tests_metric tests/test_metric_support_window.py tests/test_metric_support.py \
  tests/test_metric_patch_adapter.py --basetemp=../pytest-metric-specialized-complete
# 80 passed in 3.20s
```

精确对照原全帧参考覆盖：空/全满/边缘/孔洞/分离/细长区域、多次腐蚀、阈值相等及 nextafter 超阈、
实例外邻居、NaN/Inf/非正深度、float32/float64、固定种子 571823 的 160 组随机和非连续数组。
污染测试先得到 PASS，再原位修改最终面片、depth、K、mask 或冻结支持区域，均重新检查或拒绝。
专项还保留可 accepted 的完整解析箱体、独立坐标变更拒绝、边界冲突和不可投影候选，未因真实数据 accepted=0 删除补全计算。

首次远端测试准备遗漏 pytest 的 `py.py` 兼容模块；补齐同一安装中的文件后可运行。
首次专项收集缺 tools 搜索路径，随后 79 通过 / 1 因未上传既有 fixture 文件而失败；
补齐 `isaac_joint_fixture.py` 及其 manifest 后 80 项通过。这些是测试打包/路径错误，不是几何结果不一致。
旧快照专项原本依赖必跑的旧子进程触发源文件替换；已改在真实 metric 入口前触发，
保持原验证后的源文件被污染也只消费已绑定快照的断言，并新增旧诊断为 DISABLED 的断言。

未启动 Isaac、SAM、MoGe、ROS/Humble、RViz、执行桥、运动规划或机器人；未新增 GT 几何补全。
本轮是复用 SAM 的几何处理测量，不代表 RGB 到 ROS 实时吞吐率。
版本保持核心 `0.5.0.dev0`、contracts `1.1.0`、ROS interfaces `1.2.0`、bridge `1.2.1`。
剩余热点只记录，本轮结束后停止。

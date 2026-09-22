# 独立米制几何进程的显式 BLAS 线程策略

## 基线和范围

实际基线 `f0214e1067e2c9c5892ca41503937154051026c9`，工作分支保持
`feat/v0.5-perception-ros2`；已读 AGENTS.md，工作区初始干净，fetch 后远端一致。
本轮没有修改任何 `src/unloading_perception` 几何数学实现、阈值、采样量或验证判据。
支持窗口、独立最终检查、单次滤波、旧诊断默认关闭和此前执行修复均保留。

新增入口 `tools/run_metric_geometry_blas.py`：标准库解析参数后，在该独立进程内加载
NumPy、SciPy.linalg 和 OpenCV，再用已有 OpenBLAS 原生 getter 查询真实加载的库。
只有显式 `--blas-threads N` 才通过对应库支持的 setter 改变 NumPy/SciPy 的线程预算。
省略参数不调用 setter，继承原值。正整数以外参数在数值库初始化前拒绝。
没有修改环境变量、shell、容器配额或服务器全局配置，也没有把设置推广到常驻 worker。

原生 API 在库初始化后真实控制线程，不是导入后写环境变量。
控制范围仅为 `/proc/self/maps` 中已经加载、能够确认归属 NumPy/SciPy wheel 的 OpenBLAS；
未知归属、缺 getter、线程值无法读取、缺必要 setter 或设置后值不符均不能作为有效实验。
其他后端/安装布局尚未验证，明确失败，不猜测已生效。
OpenCV 调度及非目标 BLAS 不调用 setter，前后还检查它们的设置、环境和 affinity 未变。

## 运行、身份和失败规则

入口仍委托已有 `profile_metric_faces.main` → 固定输入交付入口 → 真实几何、融合和正式 artifact loader，
没有复制算法流水线。两个模块及所有实例共用一种进程策略，不在回调中调整。
该入口不提供重型 profiler 开关；A/B 使用同一份轻量计时代码。

运行策略加入新产物配置的 `metric_runtime_policy`，由现有 canonical fingerprint 正常计算新身份；
原入口未传策略时行为不变。线程证据独立写入 `thread_policy_report.json`，不混入几何记录。
报告包含同一 PID 的 `before_policy`、`before_geometry`、`after_geometry` 快照、原生 setter 调用、
策略指纹、计划/代码身份和本次 artifact 引用。
显式设置前和几何完成后都验证 NumPy/SciPy 实际值。
设置不生效/无法核实时返回 `THREAD_POLICY_NOT_VERIFIED` 和非零退出；启动前失败不运行几何。
完成后策略漂移也使整组失败，不能因 artifact 存在就认作有效单线程实验。

严格比较器新增 `--mode blas-threads`，要求冻结策略 `[null, 1]`、双方旧诊断关闭、
几何和入口代码完全相同、两组重型 profiler 均关闭，并核对线程报告 PID、策略、真实库值及 artifact 身份。
原 `diagnostic-toggle` 和要求一个生产文件变化的 `hotspot-off` 模式及其测试保留。
允许归一化的新增项仅为已验证的线程策略请求值和控制方式；仍逐实例完整比较几何、支持证据、
接受/拒绝、观测、融合、unknown 和原始来源。没有舍入、容差或几何字段白名单。

本轮复用 capture-f24f63a 的全部 upper 16 / lower 31 个 SAM mask，2592×1944，
原 depth/K/T_W_C、顺序、seed、上游 SHA 和配置不变；全部审计继续写出。
新固定清单从上轮清单继承输入哈希，运行前后复核，实际 worker 绑定仍由既有校验器加载。
只有两次完整运行，顺序 A 后 B；未引用上轮 179.6891 秒作本轮基线。

```bash
cd /root/autodl-tmp/v05-acceptance/metric-blas-20260922
/root/v05-gpu-venv/bin/python code/tools/run_metric_geometry_blas.py \
  --plan fixed_plan.json --output A-inherit
/root/v05-gpu-venv/bin/python code/tools/run_metric_geometry_blas.py \
  --plan fixed_plan.json --output B-blas-1 --blas-threads 1
/root/v05-gpu-venv/bin/python code/tools/compare_metric_faces_ab.py \
  --before A-inherit --after B-blas-1 --plan fixed_plan.json \
  --mode blas-threads --output comparison.json
```

## 真实进程资源与结果

两次运行均成功完成真实几何、融合、正式产物加载校验，线程策略均通过验证；严格结果比较返回 **1（不精确一致）**。
因此保留默认继承，不推荐把单线程作为生产默认；显式选项可用于受控离线实验，尚欠数值可靠性论证。
没有为通过比较舍入、增加容差或重跑。

证据位于 [evidence/metric-blas-20260922](evidence/metric-blas-20260922)：冻结计划、A/B summary、function timings、同进程 thread report、严格 comparison 和 69 份新文件的 SHA/大小清单。
两组使用同一个只读代码快照（基线加本轮入口与记录修改），压缩包 SHA256 为
`423fc3aa202af9a33e16460e80714a9c6194ceb65ac2d0bda8b28608d032821f`；
计划 SHA256 为 `a7578fe294008dfda82b3181c4dd71477b8ce9adbdcf658e97661d8fd1664a0e`。
运行过程中没有改代码；最终提交中的入口代码与证据内 `entry_code_sha256` 对应，几何生产文件 A/B 哈希完全相同。
完整原始新输出保存在上述服务器目录下的 `A-inherit` / `B-blas-1`，未覆盖历史输入或验收文件。

### 真实进程与资源

Python 3.10.12、NumPy 1.26.4、SciPy 1.14.1、OpenCV 4.10.0；A PID 29567，B PID 30095。
实际加载 NumPy OpenBLAS 0.3.23.dev（USE64BITINT）和 SciPy OpenBLAS 0.3.27.dev，
均报告 `get_parallel=1`、`MAX_THREADS=64`，通过各自导出的原生 get/set API 独立控制。
threadpoolctl 不存在，未安装依赖；库路径、构建字符串、getter/setter 名称均保存在同进程报告。

| 配置或资源 | A 继承 | B 显式 1 |
|---|---:|---:|
| NumPy / SciPy BLAS，策略前 | 64 / 64 | 64 / 64 |
| NumPy / SciPy BLAS，几何前及完成后 | 64 / 64 | 1 / 1 |
| 原生 setter 调用次数 | 0 | 2（各库一次） |
| OpenCV 调度线程，前后 | 22 | 22 |
| OpenCV 自带非目标 BLAS，前后 | 1（SINGLE_THREADED） | 1（未控制） |
| affinity / os.cpu_count | 0–207 / 208 | 0–207 / 208 |
| 已创建进程线程，几何前→后 | 127→148 | 127→148 |
| 同时实际执行线程数 | 未测量 | 未测量 |

OMP/OPENBLAS/MKL/GOTO/NUMEXPR/VECLIB 的相关线程环境变量两组均未设置，也未写入。
配置线程数不等于活跃线程数，两个 BLAS 池的值不能相加当作同时使用量；原生 setter 也没有销毁已创建线程。
当前 cgroup v2 为 `0::/`，可见挂载 `/sys/fs/cgroup`，`cpu.max=2200000 100000`：
每 0.1 秒最多 2.2 CPU 秒，即 22 CPU 时间配额，不是独占 208 核。
已检查可访问路径至挂载根；命名空间之外的父级约束不可见，明确未知。

| cgroup 累计统计的前后增量（聚合范围） | A | B |
|---|---:|---:|
| usage_usec | 2188146101 | 157125398 |
| user_usec | 607185682 | 127047143 |
| system_usec | 1580960419 | 30078255 |
| nr_periods | 1813 | 1213 |
| nr_throttled | 912 | 0 |
| throttled_usec | 3412614300 | 0 |

这些是 cgroup 聚合值，可能包含其他进程；不能归因全部属于本任务，限流累计值也不是墙钟延迟。
主机 1 分钟 load 在 A 为 16.35→18.94、B 为 13.70→19.67；共享负载不恒定，不能确定归因于某个外部进程。
进程自己的 CPU 时间另列如下，未混用 cgroup usage。

### 单次固定对照的计时

单位秒；分项为两模块合计，函数行含其子调用，不能与父阶段相加。
点图构建包含实例滤波及压缩审计写出；保持既有计时字段含义和全部审计。

| 阶段 / 指标 | A 继承 | B 单线程 |
|---|---:|---:|
| 最终面片阶段 | 100.2117 | 49.0381 |
| 最终校验 / 观测转换 | 25.6268 | 17.2482 |
| check_metric_plane（430 次） | 34.8959 | 17.4946 |
| 固定上游平面提取（91 次） | 24.6452 | 6.4804 |
| observation_support（660 次） | 8.6441 | 4.3740 |
| 点图构建，含滤波与压缩审计 | 44.2180 | 43.8401 |
| 点图写出 | 1.8483 | 1.8564 |
| 滤波审计 JSON 写出 | 0.001733 | 0.001665 |
| 融合与产物发布 | 2.6473 | 2.6010 |
| 正式 artifact loader 校验 | 0.2673 | 0.2654 |
| 完整几何交付墙钟（summary 既有字段） | 181.1943 | 121.2175 |
| 入口总墙钟（含库加载 / 线程核验） | 181.6173 | 121.6227 |
| 几何范围进程 CPU 时间 | 2181.2859 | 126.2167 |
| 入口总进程 CPU 时间 | 2188.0806 | 133.0044 |
| 进程生命周期峰值 RSS（MiB） | 1571.3125 | 1540.2539 |

本次完整交付墙钟下降 33.10%，几何范围 CPU 时间下降 94.21%，峰值 RSS 下降 1.98%。
这只是该共享机器、该固定输入的一次 A/B，未做重复统计，不能证明普遍最优或全视觉实时。
两组均写出 47 条 PASS 滤波审计、51 个 NPZ；NPZ 总大小 A 44718568、B 44718560 字节。
配置身份和输出路径正常变化，各产物使用真实新哈希；不要求所有容器文件哈希保持旧值。

### 精确性结论

上模组 16/16、下模组 30/31 的完整实例记录精确一致，严格比较共报告两处路径差异，实为同一数值及其观测副本：

- `module_1_lower/final/instances/8/metric_solver/cost`（mask_id=9）：
  A `1.0654886446379104`，B `1.0654886446379102`。
- `module_1_lower/observation/cargo/8/raw_result/record/metric_solver/cost`：同上。

最大绝对差 `2.220446049250313e-16`，相对 A 的最大相对差 `2.0839696982457254e-16`。
差异没有被忽略；`comparison.json` 保持 `plan_complete=true, results_equal=false`，比较命令退出 1。
其余被比较字段精确一致，包括全部 71 张面片的身份、法向、位置、3D 坐标、支持区域/点数、残差和边界证据，
接受/拒绝原因，来源、标定、采集时间，以及最终融合对象、关联关系和 unknown。
下模组 mask 1 仍无面片，两组均 0 个完整 cuboid accepted、37 个融合对象、84 个 unknown，`planning_admissible=false`。
业务判定未变化，但 **不能把 46/47 精确一致写成 47/47**，也不能以面数相同替代完整记录比较。
单线程有本次测得的性能收益，却未满足全部确定性字段精确一致的要求；本轮不作生产默认推荐。

## 测试、限制和版本

```powershell
$env:PYTHONUTF8='1'
.venv310/Scripts/python.exe -m pytest -q tests/test_metric_thread_policy.py tests/test_metric_faces_ab_comparison.py --basetemp=tmp/pytest-blas-targeted
# 35 passed, 2 skipped in 9.51s
.venv310/Scripts/python.exe -m pytest -q --basetemp=tmp/pytest-blas-full
# 884 passed, 2 skipped, 1 deselected in 144.78s
```

本地两个 skip 是缺 SciPy/OpenCV 的真实原生库子进程测试；在服务器实际数值环境补测：

```bash
cd code
PYTHONPATH=/root/autodl-tmp/v05-acceptance/metric-profile-20260922/test-deps:tools:tests \
 /root/v05-gpu-venv/bin/python -m pytest -q \
 tests/test_metric_thread_policy.py tests/test_metric_faces_ab_comparison.py tests_metric \
 --basetemp=../pytest-blas-native
# 71 passed in 5.03s
```

覆盖默认继承不调用 setter、真实子进程入口/参数/原生库加载与设置、父进程保持原线程配置和环境、
非法值在数值初始化前拒绝、多个池核验、不可验证/未生效拒绝、未知 cgroup 权限、非目标设置改变拒绝，
以及新比较模式拒绝缺模块/实例/双空/文件/哈希/数值差异/代码变化/错误线程值。
真实子进程测试只替换几何计算；原生 NumPy/SciPy/OpenCV、控制和查询均真实。
完整真实 A/B 则不替换任何几何函数。测试在性能运行前完成，没有与两次几何性能运行并行。

未运行 SAM/MoGe、Isaac、ROS/Humble、RViz、机器人执行；未验证视频或 SAM 常驻 worker 接入。
此策略只覆盖上述独立固定输入几何入口，不声明全视觉实时，也不宣称单线程普遍最优。
不新增依赖、不替换 BLAS、不升级 NumPy/SciPy/OpenCV。沿用已有独立 pytest 测试目录。
版本保持 `0.5.0.dev0`、contracts `1.1.0`、ROS interfaces `1.2.0`、bridge `1.2.1`。
本轮结束即停止，不追加线程档位或其他算法优化。

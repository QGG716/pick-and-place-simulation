# 返回后的求解器 cost：依赖与精确性复核

初始基线 `659a7de6a426d602783d0aff5e67588e887d6588`；恢复验证时实际基线
`6ffa344496bb418719883a3c91d39cab897d5d64`，分支 `feat/v0.5-perception-ros2`。
已读根 AGENTS.md，初始工作区干净，fetch 后远端一致。本轮仅改只读复核、单实例诊断工具、测试及说明；
几何数学、采样、阈值、支持窗口、独立验证、线程入口和默认继承均未修改。

**真实复核已完成：完整记录仍不精确一致，几何/支持/判定精确一致。**
旧端口两次连接失败的记录保留在 `availability_and_local_review.json`，不改写历史事实。
用户提供更新后的既有实例连接后，已完成全量只读复核和 mask 9 的两个独立进程真实求解。
没有再跑完整 47 实例性能 A/B，没有重跑单实例来挑选结果。

## 完整历史产物复核

路径来自上一轮仓库说明和绑定报告：
`/root/autodl-tmp/v05-acceptance/metric-blas-20260922/{A-inherit,B-blas-1}`。
初次本地检查的 8 份归档哈希均匹配，但当时未取得完整产物。
恢复连接后的新输出位于 `/root/autodl-tmp/v05-acceptance/metric-cost-review-20260926`。
`historical_review.json` 核验 **69 份历史文件、30 份输入**、上游和代码身份；
正式 artifact loader 核查索引、模块观测、融合产物和引用 SHA，不是只读 summary。
完整遍历 **1,181,575 对值**，覆盖全部 47 实例、71 面片、模块观测、融合/关联/unknown；
只有下面 **2 处差异**，受保护字段差异计数为 0，详情上限未截断检查或计数。

历史 comparison 保持 `results_equal=false`，未覆盖、修改或重新解释其退出 1。
本次确认 46/47 个完整实例记录精确一致；两组 37 个融合对象、84 个 unknown、0 个完整 cuboid accepted 不变：

| 完整归一化路径 | A | B |
|---|---|---|
| `module_1_lower/final/instances/8/metric_solver/cost` | 1.0654886446379104 | 1.0654886446379102 |
| `module_1_lower/observation/cargo/8/raw_result/record/metric_solver/cost` | 同上 | 同上 |

上述索引对应 mask_id=9。JSON 解码后的类型均为 Python float / binary64；
十六进制分别 `0x1.10c3dd22faac4p+0`、`0x1.10c3dd22faac3p+0`。
绝对差 `2.220446049250313e-16`，相对 A 差 `2.0839696982457254e-16`，ULP 距离 **1**。
初次不可访问时的记录保持不变。当前完整核查见
[historical_review.json](evidence/metric-cost-review-20260926/historical_review.json)，
真实求解解释见 [explained_review.json](evidence/metric-cost-review-20260926/explained_review.json)。
新复核与解释命令也均退出 1：解释诊断差异不改变完整记录不精确一致的事实。

## cost 的依赖链与字段角色

本轮检索整个当前分支的 src、packages、tools、tests、tests_metric，并逐段阅读以下调用链。
`metric_solver` 的生产写入位于 metric_faces；工具 complete_metric_stage_evidence 整块保存它。
还检查了 record/raw_result 的间接传递、排序、资格筛选和指纹逻辑，未仅依靠一次 cost 文本搜索。

| 层次 / 代码 | 实际用途 |
|---|---|
| `metric_faces._orthogonal_fit` 内的 `robust_residual` 与 `least_squares` | 内部目标参与优化与收敛，不能视作无关诊断。初值含 SO(3) 旋转向量与各面偏移；变换后的残差按各组 sqrt(N) 归一化。调用没有传 loss；真实签名与捕获确认实际 loss=linear、method=trf。 |
| 求解器返回后 | `solved.x` 更新法向、偏移和 axes；`solved.cost` 转为 float 写入返回字典；没有用这个返回字段更新几何或选择候选。不能据此推断两个运行的 x/fun 相同。 |
| `fit_metric_faces` / `_complete` | solver 字典写入 `metric_solver`。几何接受看测量支持、残差、物理边界、可观测性及独立校验；`_complete` 使用 axes 和 bounds，并另存 boundary_fit。后者仍受精确比较保护，本轮不把整个 solver 块归为可忽略。 |
| `validate_metric_record` / `validate_final_record` | 深拷贝记录并复核 depth/mask/K 绑定、最终平面、支持、残差和完整 cuboid。返回后的 cost 不用于接受/拒绝；字典复制保留它。 |
| `_observation` / `rgbd.hypotheses_from_geometry_record` | `_observation` 再做独立验证。假设由 accepted、面数、几何一致性、残差与不确定度决定；cost 不排序、不授予抓取资格。无完整假设时，原记录进入 `raw_result.record`。有假设的分支只保存指定证据，不保证每个对象都有 cost 副本。 |
| `observed_faces` / `fusion` | 从最终支持及面片角点生成 ObservedFaceSet，后者不携带 metric_solver。关联按世界面几何与支持打分，排序使用关联分、身份及支持点数；不读取 cost。 |
| `algorithm_handoff` | 已融合对象用 source_members / fusion_diagnostics 重建 raw_result；未消费实例保留原 cargo 的 raw_result。不能泛称 cost 永远不进入融合观测。 |
| `planning_geometry` | 几何来自 observed_surfaces、显式先验及关联状态；不读取 solver cost。来源观察整体指纹、artifact 引用仍保留内容身份影响。 |
| `algorithm_artifact` 发布/加载、replay | 模块文件哈希包含 raw_result 副本；索引及融合观测 provenance 引用模块哈希，内容身份随之变化。loader 核查真实文件 SHA；replay 核查原始 observation 指纹。不会把几何语义相同变成同一身份。 |
| `scene.build_scene_update` / contracts `_canonical` | scene 指纹包含 cargo 和 unknown；cargo 的 raw_result 虽声明 `compare=False`，canonical 仍遍历所有 `init=True` 字段，因此有该副本的 cargo 会改变场景指纹。场景资格判断不读 cost，但身份/修订可变，旧 plan/grant 不能跨运行复用。 |
| 比较和评价工具 | 严格比较整份记录；complete_metric_stage_evidence 保存 solver 块；_evaluate 读取几何/假设与评价输入，不读取 cost 作决策。 |

结论限于当前分支已检查的代码：没有找到**返回后的该字段**参与候选排序、面片/体积接受或抓取资格的直接数值决策；
内容身份传播是实际影响，不能说“全系统没有影响”。外部分支、外部消费者和未验证入口不在结论范围内。
故障注入测试验证了：只改已返回 cost，资格及 unknown 不变，但 observation/scene 指纹变化；这不是真实求解器试验。

## 本轮工具与严格模式

新增 `tools/review_metric_blas_cost.py`，只读使用旧严格比较器的全部 artifact 校验、计划/实例清单、
线程策略、代码同一性检查与既有路径/运行身份归一化。
旧模式、旧比较报告和 CLI 退出规则保持不变；仅增加可选逐值收集回调。
新收集器完整遍历所有共同字段，对缺字段/长度/类型也记录结构差异；详情限量不截断遍历和计数。
全部诊断差异单独完整保留，只有普通详情有上限。

CLI 还先核对冻结 16+31 / 2592×1944 计划、清单内全部历史文件 SHA/大小、原输入 SHA、
上游文件 SHA、原代码快照中的生产/入口 SHA；缺失、重复 ID、hash 错误或副本不一致失败。
只有 schema 为 `DEPTH_METRIC_PATCHES_V1`、objective 为 `ROBUST_METRIC_POINT_TO_SHARED_SO3_PLANES`
的 `metric_solver.cost` 及指定模块观测副本单独分类，不按 mask 9 设置例外。
其他字段全部精确比较，包括 metric_solver.success、boundary_fit、几何、法向、残差、支持、边界、判定、关联和 unknown。
新出现的其他路径即使名称也是 cost，仍保守作为受保护差异，需另行依赖核查。

缺失/非浮点/非有限/负 cost 明确失败。工具还有 cost >= 1e6 的防御性人工复核门槛，
只用于阻止异常巨大值被“诊断”标签掩盖，不是算法阈值、允许差异或数值可靠性界限；
门槛之下的差异同样保持 `UNEXPLAINED_REQUIRES_REAL_SOLVER_EVIDENCE`。
本工具没有解释证据就不返回“已解释”；不使用 allclose、ULP 容差或舍入。只要完整记录不一致，CLI 仍返回 1。

实际使用的只读复核命令（code-root 指向历史快照；输出须为新文件）：

```bash
python /path/to/current-checkout/tools/review_metric_blas_cost.py \
  --archive-root /root/autodl-tmp/v05-acceptance/metric-blas-20260922 \
  --evidence-manifest /path/to/current-checkout/docs/validation/evidence/metric-blas-20260922/evidence_manifest.json \
  --plan /root/autodl-tmp/v05-acceptance/metric-blas-20260922/fixed_plan.json \
  --code-root /root/autodl-tmp/v05-acceptance/metric-blas-20260922/code \
  --output /path/to/new-output/cost_review.json
```

## 固定单实例真实复现与解释

`tools/reproduce_metric_cost.py` 只选择下模组 mask 9，顺序启动两个独立进程，各求解一次：

```bash
cd /root/autodl-tmp/v05-acceptance/metric-cost-review-20260926
/root/v05-gpu-venv/bin/python code/tools/reproduce_metric_cost.py \
  --plan ../metric-blas-20260922/fixed_plan.json \
  --archive-root ../metric-blas-20260922 --output mask9-inherit
/root/v05-gpu-venv/bin/python code/tools/reproduce_metric_cost.py \
  --plan ../metric-blas-20260922/fixed_plan.json \
  --archive-root ../metric-blas-20260922 --output mask9-blas-1 --blas-threads 1
/root/v05-gpu-venv/bin/python code/tools/explain_metric_cost_capture.py \
  --review historical_review.json --before mask9-inherit --after mask9-blas-1 \
  --output explained_review.json
# 最后一条退出 1：解释完成也不是完整记录精确相等
```

先核验原始绑定及历史输入 SHA，再读取已绑定 depth、K、SAM mask 和谱系歧义标志。
不解析 GT 中心、姿态或角点；完整性检查对 GT 文件仅计算历史字节哈希，不输入算法。
使用生产 `extract_observation_labels`、固定上游 `load_extractor`、`fit_metric_faces`，
种子 17、原分辨率/深度/配置不变，support_capture_binding 与历史记录相同。
不重建点图，不调用评价；该生产拟合路径本来就消费原始 depth，而非过滤点图值。

包装器仅截获 `robust_residual` 的真实 least_squares 调用，原样传参并返回原对象；
不包裹残差函数、不改迭代，独立目标参考在求解器返回后计算。
捕获初值、输入组身份、x/fun/jac/grad/cost 及终止信息。
独立 `validate_final_record` 再次检查后记录不变；两组新的完整 mask 9 记录分别精确匹配历史 A 和 B。

| 项目 | 继承进程 PID 2199 | 单线程进程 PID 2369 |
|---|---|---|
| NumPy / SciPy BLAS，几何前及后 | 64 / 64 | 1 / 1 |
| OpenCV 调度 / 非目标 BLAS | 22 / 1 | 22 / 1 |
| 返回 cost / 本进程独立 dot 复算 | 1.0654886446379104 | 1.0654886446379102 |
| success / status | true / 3 | true / 3 |
| nfev / njev | 33 / 23 | 33 / 23 |
| 终止原因 | xtol | xtol |
| optimality | 0.000804355598361346 | 同左 |

两组拟合输入组、法向/偏移、关联、符号、配置及初值身份精确相同。
初值 (7)、最终 x (7)、**fun (10365)**、jac (10365×7)、grad (7) 的 dtype、shape 和字节全部相同。
因此此次差异不来自最终参数或最终残差变化。
环境为 Python 3.10.12 / NumPy 1.26.4 / SciPy 1.14.1 / OpenCV 4.10.0；
affinity 可见 208 CPU，当前 cgroup `cpu.max=2200000 100000`（22 CPU 时间配额）。
实际库和线程 getter 证据在捕获报告中；环境变量与非目标线程未变。配置数量不是同时执行线程数。
没有以这次单实例运行作新的性能对照。

实际 loss=linear、method=trf；输入已经是生产变换后的残差
`sign(t)*sqrt(2*(sqrt(1+t*t)-1))/sqrt(N)`，其中 `t=(point·normal+offset)/noise_scale`。
当前 SciPy `trf_no_bounds` 线性分支使用 `0.5*np.dot(f,f)`，源文件 SHA256 为
`1255479c1d949f779a4388c9f0a1c7a7e8705df607332a674ad85f44edf73fdd`，相关行随报告保存。
对相同返回 fun，各线程配置下独立 dot 精确重现对应历史 cost，因此定位为
**相同最终残差的浮点点积归约结果差异**。没有追踪 BLAS 内核的具体指令/归约树，不能断言某种特定求和顺序。

另用 Fraction 对已返回 binary64 残差逐项平方、求和、除以 2，得到精确有理数参考；
正确舍入的 cost 为 `1.0654886446379104`，本次 `0.5*math.fsum(v*v)` 也相同。
单线程结果比该参考低 1 ULP，不能宣称它在数值上更精确。
Fraction 只精确计算已返回残差的平方和；残差求值和参数求解误差仍在。
fsum 的乘积先舍入，二者都不是整个优化过程的精确证明，参考值未写回生产输出。

`explain_metric_cost_capture.py` 核验数组 SHA、逐位相等、输入/源码身份、实际线程、
独立验证及历史完整记录匹配，再生成 `EXPLAINED_SAME_FINAL_RESIDUAL_DOT_REDUCTION`。
新诊断路径、变化的 x/fun/判定、异常 cost、错误线程或哈希均失败，没有 ULP 接纳窗口。
报告仍保留 `records_exact_equal=false`，两个几何/支持/判定字段为 true。
新增证据仅两个单实例捕获数组（各约 391 KiB）和轻量报告，没有复制历史大体积几何产物。

## 测试与策略

本地定向命令：

```powershell
$env:PYTHONUTF8='1'
.venv310/Scripts/python.exe -m pytest -q tests/test_metric_cost_capture.py tests/test_metric_blas_cost_review.py tests/test_metric_faces_ab_comparison.py tests/test_metric_thread_policy.py --basetemp=tmp/pytest-cost-completed-targeted
# 76 passed, 2 skipped in 17.53s
.venv310/Scripts/python.exe -m pytest -q --basetemp=tmp/pytest-cost-completed-full
# 925 passed, 2 skipped, 1 deselected in 143.93s
```

覆盖诊断分类后严格比较仍失败；坐标、法向、残差、支持和判定/未知字段不能被掩盖；
异常/缺失 cost、副本不一致、缺模块/实例/重复 ID/文件/hash、详情上限后的差异。
生产独立验证对修改后的最终面片仍拒绝，对 depth/K/mask 改动仍报绑定不符。
测试中的返回 cost 改动明确标为合成故障注入，不代替真实优化器验证。
新增包装器测试验证原样调用与原对象返回；真实捕获归档测试验证解释成功仍严格退出 1，
并拒绝损坏的 x/fun/hash/cost/status/线程、未通过独立验证和新增差异。
真实求解由上述两个独立进程完成，不以故障注入代替。
服务器用原解释器、原独立 pytest 目录运行上述测试加 tests_metric：111 passed，
1 项因新快照漏传纯 CPU isaac_joint_fixture.py/manifest 而失败。
补齐这两个已有夹具后仅重跑 tests_metric/test_offline_payload_consumption.py，1 passed in 0.32s。
112 项最终通过，包含本地跳过的两个原生库子进程测试；没有安装或升级数值库，没有启动 ROS/Isaac。
服务器命令为 `PYTHONPATH=/root/autodl-tmp/v05-acceptance/metric-profile-20260922/test-deps:tools:tests /root/v05-gpu-venv/bin/python -m pytest -q`
加上述四个测试文件及 `tests_metric`，初次目录 `../pytest-native-completed`；补测目录 `../pytest-payload-fixture-completed`。
`completed_evidence_manifest.json` 保存 12 份新文件的 SHA/大小、旧 comparison SHA，以及三份核心几何文件与历史代码完全相同的核查结果。

**可推荐 `--blas-threads 1` 作为已验证独立离线入口的显式配置，范围限于上述环境和 capture-f24f63a 固定输入；默认仍继承。**
依据为历史单次性能收益、全量几何/支持/判定精确一致以及唯一诊断差异的真实复现和归约定位。
这不是完整记录逐位可重现，也不是未来输入的通用数值保证；历史 121 秒不是在线感知达标证明。
历史采集、ORACLE-PROMPTED、raw_image_automatic=false、planning_admissible=false 保持；
不授予执行资格，不修改 canonical_fingerprint、不伪造相同 artifact 身份。
未再运行完整 47 实例性能 A/B，未启动 SAM/MoGe、Isaac、ROS/Humble、RViz、执行桥或机器人；
没有线程档位搜索和算法优化。SAM 常驻 worker、视频、ROS、真实硬件及未来输入均未覆盖。
版本保持 `0.5.0.dev0`；契约和 ROS 包版本未改。

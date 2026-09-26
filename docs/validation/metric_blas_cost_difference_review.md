# 返回后的求解器 cost：依赖与精确性复核

实际基线 `659a7de6a426d602783d0aff5e67588e887d6588`，分支 `feat/v0.5-perception-ros2`。
已读根 AGENTS.md，初始工作区干净，fetch 后远端一致。本轮仅改只读比较工具、测试及说明；
几何数学、采样、阈值、支持窗口、独立验证、线程入口和默认继承均未修改。

**本轮真实复核未完成。** 2026-09-26 两次连接既有服务器
`connect.westd.seetacloud.com:13599` 均报 `NoValidConnectionsError`（解析到 `36.103.198.204`）。
已询问既有实例是否停机或地址变化；没有租用新服务器或重建输入。
完整产物重遍历和 mask 9 的真实求解复现均未运行，因此不新增“单线程已通过数值可靠性验证”的结论。

## 可复核的历史证据与未完成部分

路径来自上一轮仓库说明和绑定报告：
`/root/autodl-tmp/v05-acceptance/metric-blas-20260922/{A-inherit,B-blas-1}`。
本地只有冻结计划、线程报告、计时、summary、comparison 和哈希清单，没有这两组完整几何/模块观测、
原始绑定输入或求解器 `x/fun`。本轮对其中 8 份归档文件逐个复算 SHA256，与原 evidence_manifest 一致。
这验证的是本地归档未变，不等于重新验证远端输入、代码或 artifact 引用链。

历史 comparison 保持 `results_equal=false`，未覆盖、修改或重新解释其退出 1。
历史报告记载 47 实例、71 面片、46 个完整实例记录精确一致；两处差异为：

| 完整归一化路径 | A | B |
|---|---|---|
| `module_1_lower/final/instances/8/metric_solver/cost` | 1.0654886446379104 | 1.0654886446379102 |
| `module_1_lower/observation/cargo/8/raw_result/record/metric_solver/cost` | 同上 | 同上 |

上述索引对应 mask_id=9。JSON 解码后的类型均为 Python float / binary64；
十六进制分别 `0x1.10c3dd22faac4p+0`、`0x1.10c3dd22faac3p+0`。
绝对差 `2.220446049250313e-16`，相对 A 差 `2.0839696982457254e-16`，ULP 距离 **1**。
本轮实际运行了这一数值表示检查，证据在
[availability_and_local_review.json](evidence/metric-cost-review-20260926/availability_and_local_review.json)。
它不是最小真实求解复现，也没有证明舍入发生在残差求值、乘方还是最终归约。
新证据将 `plan_complete=false`、几何/支持/判定精确一致状态设为 `null`，不能把旧 summary 当作新验证完成。

## cost 的依赖链与字段角色

本轮检索整个当前分支的 src、packages、tools、tests、tests_metric，并逐段阅读以下调用链。
`metric_solver` 的生产写入位于 metric_faces；工具 complete_metric_stage_evidence 整块保存它。
还检查了 record/raw_result 的间接传递、排序、资格筛选和指纹逻辑，未仅依靠一次 cost 文本搜索。

| 层次 / 代码 | 实际用途 |
|---|---|
| `metric_faces._orthogonal_fit` 内的 `robust_residual` 与 `least_squares` | 内部目标参与优化与收敛，不能视作无关诊断。初值含 SO(3) 旋转向量与各面偏移；变换后的残差按各组 sqrt(N) 归一化。调用没有传 loss；没有在本轮服务器上重新核实 SciPy 默认实现。 |
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

服务器恢复后只读复核命令（使用本轮工具的检出路径，code-root 指向历史快照；输出须为新文件）：

```bash
python /path/to/current-checkout/tools/review_metric_blas_cost.py \
  --archive-root /root/autodl-tmp/v05-acceptance/metric-blas-20260922 \
  --evidence-manifest /path/to/current-checkout/docs/validation/evidence/metric-blas-20260922/evidence_manifest.json \
  --plan /root/autodl-tmp/v05-acceptance/metric-blas-20260922/fixed_plan.json \
  --code-root /root/autodl-tmp/v05-acceptance/metric-blas-20260922/code \
  --output /path/to/new-output/cost_review.json
```

尚需的真实步骤：只针对固定下模组 mask 9，在两个独立进程中通过原生产提取/拟合路径，
只读捕获真实 least_squares 初值、x、fun、cost 和退出信息，验证实际线程池。
应核实实际 loss 与变换后 robust_residual，再区分参数差、残差差和相同残差的归约差。
没有这些证据，不断言是求和顺序；也不把 fsum 称为整个优化过程的精确证明。
本轮没有运行这个步骤，没有添加未经真实验证的生产包装器。

## 测试与策略

本地定向命令：

```powershell
$env:PYTHONUTF8='1'
.venv310/Scripts/python.exe -m pytest -q tests/test_metric_blas_cost_review.py tests/test_metric_faces_ab_comparison.py tests/test_final_geometry.py --basetemp=tmp/pytest-cost-review-final
# 57 passed in 16.70s
.venv310/Scripts/python.exe -m pytest -q --basetemp=tmp/pytest-cost-review-full
# 911 passed, 2 skipped, 1 deselected in 150.22s
```

覆盖诊断分类后严格比较仍失败；坐标、法向、残差、支持和判定/未知字段不能被掩盖；
异常/缺失 cost、副本不一致、缺模块/实例/重复 ID/文件/hash、详情上限后的差异。
生产独立验证对修改后的最终面片仍拒绝，对 depth/K/mask 改动仍报绑定不符。
测试中的返回 cost 改动明确标为合成故障注入，不代替真实优化器验证。
没有“解释成功”的真实测试，因为真实 x/fun/cost 证据仍不可取得。
两个 skip 是本地缺 SciPy/OpenCV 的原生库子进程测试，本轮服务器不可用，未补测；没有安装或升级数值库。
全量之后又确保观察回调不参与严格 verdict：即使回调为空，严格比较仍失败；最终定向 57 项覆盖该加强。

保持默认继承；已有 `--blas-threads 1` 仍仅可作为独立离线入口的显式实验配置，
本轮不提升为完成数值可靠性验证的推荐配置。历史 121 秒不是在线感知达标证明。
历史采集、ORACLE-PROMPTED、raw_image_automatic=false、planning_admissible=false 保持；
不授予执行资格，不修改 canonical_fingerprint、不伪造相同 artifact 身份。
未运行完整 47 实例 A/B、单实例真实求解、SAM/MoGe、Isaac、ROS/Humble、RViz、执行桥或机器人；
没有线程档位搜索和算法优化。SAM 常驻 worker、视频、ROS、真实硬件及未来输入均未覆盖。
版本保持 `0.5.0.dev0`；契约和 ROS 包版本未改。

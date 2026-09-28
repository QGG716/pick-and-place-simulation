# Tesseract + OMPL v0.3：LazyPRM 固定 FANUC 绕障实验

本轮基线为 `eab8d9de69786e2069f243fd8b8d994af86e4162`，起始本地/远端一致、工作树干净。仅修改本实验分支；复用已有依赖，没有安装环境、启动 Isaac、执行完整单箱任务、12 个接近请求或参数扫描。工程搜索仅一次。完整证据在 [v0.3 目录](validation/evidence/backend_tesseract_ompl_v0_3/README.md)。

## 1. 是否真实延迟验证，而非放松检查

**是。** 使用安装包 OMPL 1.7.0 `py314hc49c504_2` 的真实非 star `ompl::geometric::LazyPRM`。安装的 LazyPRM.h 与对应上游头文件逐字节一致；包配方指向 1.7.0 源码归档。新增子类仅读取参数、迭代计数和路网有效性标记，不重写建图、A* 或候选验证算法。

实际核对 [LazyPRM::setup / solve / constructSolution](https://github.com/ompl/ompl/blob/1.7.0/src/ompl/geometric/planners/prm/src/LazyPRM.cpp)：非 star 默认路径长度目标的满足阈值为正无穷，第一条完整验证的有限成本路径即退出；solution 在循环结束后才发布。本轮沿用这一行为，没有外部轮询 `hasExactSolution()` 假定首解时机，也没有优化、平滑或 shortcut。头文件注释中的近邻数与实现不一致，实际默认 **k=5**，由运行中的 KBoundedStrategy 对象读取确认。

后端名称仍是 `tesseract_ompl`，正式入口使用 `OMPLPlannerConfig(name="lazy_prm")`，CLI 可显式指定 `--ompl-planner lazy_prm`。普通入口仍默认 legacy；Tesseract 后端仍默认 `rrt_connect`、range=0.18 rad。未知配置拒绝，未回显 LazyPRM 配置的旧 worker 也不能静默交付 RRT 结果。

| 工程固定项 | 实际值 |
|---|---|
| 场景/阶段/端点 | 冻结 37 箱、空载 pregrasp、原 78→208 |
| seed / max_attempts | 71070 / 1 |
| 连接策略 | 非 star 默认 KBoundedStrategy，k=5 |
| 最大连接距离 | 4.431260361343388 rad，关节空间无权重 L2 |
| 状态空间最大 extent | 22.15630180671694 rad |
| 实际状态计算总限额 | 100000，端点、直连、所有候选共享 |
| 累计采样/路网节点/无向边上限 | 10000 / 10002 / 50010 |
| 细分 | 固定 4 m / 1.25 mm，L1=.0003125 rad，角度=.04 rad，refinement=0 |
| 墙钟/优化 | 无新增业务截止时间；首条原生完整有效路径即停 |

连接距离由 setup 自动计算，未机械套用 RRT 的 .18 rad。运行前 `configure` 已将实际参数写入 configuration.json 和 request.json；它不建路网、不检查碰撞，但六维空间初始化会产生下述 100 次投影采样。工程请求重新创建空路网，只接收两个固定端点，没有历史路径或节点注入。

`Context`（含 FK、Jacobian/SVD、径向、变换、FCL、精确缓存和预算轮询）、`DenseMotion` 与细分校验代码块保持字节一致，权威源码及冻结输入哈希也一致。[身份复用证据](validation/evidence/backend_tesseract_ompl_v0_3/engineering-request/checker-identity.json)绑定 v0.2 最终 worker `8196a417…` 的 29 点 legacy 路径、28 条边通过及 31,639 个实际 q 零差异结论；本轮未重复完整旧路径审计，不把旧计时归入本轮。

## 2. 无效连接是否反馈给搜索

**是。** 上游 constructSolution 删除候选中的无效节点及第一条无效边，重新处理连通分量并继续寻路。节点必须完整状态检查；边只有完成原 DenseMotion 全部网格才被标记有效，同请求已有有效记录可复用。所有 UNKNOWN 标记仍不等于通过。

独立 XY 单元场景中，两条搜索候选连接被拒绝后仍找到绕行 exact 解：31 次实际建图迭代、13,411 次实际状态计算、7 条有效边，未耗尽 100000 限额；最终路径每个必需网格点还用独立盒体距离公式核验。这是机制测试，不作为 FANUC 成功替代证据。

资源、取消中断保持独立原因。30 次状态预算回归产生 1 条未完成边且不交付；OMPL 内部可能因布尔 false 删除当时的边，外部不将其记为碰撞。每次请求重新构造 planner、清空精确缓存，请求结束丢弃整个路网和标记。authority 前后的取消、revision、显式墙钟保护，以及原有限修复逻辑均保留。

## 3. 同一工程请求是否 exact 且通过 authority

**是：VERIFIED，可交付本阶段路径。** 原生返回完整 4 点、3 条边的 exact 路径，项目权威复检仅执行一次且通过。没有追加搜索、换 seed、加预算或注入已知路径。

- 原生有效路径返回时刻：328.277 s（转换完成后观察值，不是内部循环回调时间）。
- 原生总耗时：328.277 s。
- 权威复检：492.366 s，accepted=True。
- adapter 请求总耗时：820.787 s；包含参数准备及本次请求的外层观测：821.187 s。

完整关节路径、原生结果、权威结果分别保存在 [native-result.json](validation/evidence/backend_tesseract_ompl_v0_3/engineering-request/native-result.json)、[authority.json](validation/evidence/backend_tesseract_ompl_v0_3/engineering-request/authority.json)、[result.json](validation/evidence/backend_tesseract_ompl_v0_3/engineering-request/result.json)。`complete_task_executable=false`：阶段交付不是完整任务或物理执行完成。

## 4. 计算是否更集中于候选解路径

本次留下 193 个路网节点（已知有效 52、UNKNOWN 141）及 559 条无向边（有效 3、UNKNOWN 556）。节点数不等于有效探索量；边数直接来自底层无向图，每条只计一次，不使用 PlannerData 的双向 arcs 与 RRT 树比较。

| 真实计数 | 值 |
|---|---:|
| 状态检查请求 / 实际计算 / 精确缓存命中 | 49717 / 49703 / 14 |
| 边内网格样本请求 | 49322 |
| 完整边检查调用：有效 / 违例 / 未完成 | 12：3 / 9 / 0 |
| FCL 查询 | 49685 |
| 累计 sampler 调用 / 实际 LazyPRM 迭代 | 632 / 532 |
| 候选路径验证次数 / 重新寻路次数 | null / null（非虚内部接口，未复制上游实现计数） |

632 次采样包含 100 次默认投影边界估计和 532 次路网迭代。源自 [ProjectionEvaluator::estimateBounds](https://github.com/ompl/ompl/blob/1.7.0/src/ompl/base/src/ProjectionEvaluator.cpp) 与安装头文件 `PROJECTION_EXTENTS_SAMPLES=100`；这 100 次不做状态/FCL 检查，也不进入路网。统计没有将其冒充建图进展。计数在无效节点删除时不重置。采样资源保护在搜索终止条件处检查；初始化本身有固定采样成本，冻结上限高于该成本。边上限预留一次默认 k 邻居批次，可能提前至多 k-1 条边停止，不越过上限；这不是硬实时取消承诺。

由最终路径和原细分公式计算，三条有效边需要 13,720、17,498、16,252 个区间，合计 **47,473 个状态样本请求**，约占全部边采样 **96.3%**。只有 395 个状态请求位于边检查之外。剩余 556 条 UNKNOWN 边未被当作有效边交付。由此支持本次检查集中在候选路径上，但没有反事实 eager-PRM 运行，不能估算每条延后连接节省的时间。

## 5. FCL 调用量与端到端耗时是否改善

v0.2 固定 RRT 请求为预算耗尽、没有 exact 交付；本轮在 49,703 次实际计算内成功。FCL 调用由 v0.2 的 100000 降到 49685（少约 50.3%），但原生耗时 **308.046→328.277 s**，并没有同步下降。不同 q 的 FCL 查询成本及运行噪声未被控制，不能仅用调用数推断时间收益。

v0.2 没有成功交付耗时，不能用其失败的 308.252 s 与本轮包含权威复检的成功耗时计算“加速比”。本轮配置查询预热了环境，因此同时记录包含准备的外层耗时。两轮是同输入、同验收规则下的验证调度实验，不是参数和算法工作量完全等价的纯库速度实验。

## 6. 主要时间花在哪里

本轮没有失败；主要瓶颈仍是 **少量长边的大量密集 FCL 查询**，以及随后必要的完整 authority 复检。原生分项如下，均为聚合插桩观测：

| 时钟 | 秒 |
|---|---:|
| FK / 场景状态 | 3.085431 |
| Jacobian / SVD | 0.294960 |
| 碰撞对象变换 | 6.511857 |
| FCL contactTest | 317.005271 |
| 预算 / 取消轮询 | 0.133936 |
| 边调度（排除状态检查） | 0.023973 |
| 状态检查 inclusive | 328.242300 |
| OMPL solve inclusive | 328.266890 |

FCL 占 native 总耗时约 **96.6%**。实际计算的拒绝分类为碰撞 331、关节裕量 11、径向 5、奇异性 2；没有中断边。未单独测建图/A* 时间，不能把父子时钟差额直接命名为建图时间。

native_total 包含 OMPL、状态检查等；edge inclusive 包含状态检查，state inclusive 又包含 FK、SVD、变换、FCL 和内部轮询。轮询也发生在 OMPL 终止条件中，边调度计时排除了内部状态检查。这些父子时钟不能重复相加。首条有效路径后没有继续优化。

插桩延续 v0.2，计数聚合输出，不逐状态记录工程搜索日志。本轮没有重跑 profile 参数扫描或校准，不能把这些时间称为无插桩性能；v0.2 的两对固定状态开销观测仅属于原构建和原运行条件，不作为本次校正系数。

## 7. 是否支持继续沿此配置推进，哪些结论不足

这一次成功支持保留 **显式可选的非 star 默认 LazyPRM 配置**继续研究：它在相同实际状态计算限额内完成了此前 RRT 未完成的本阶段绕障，且没有放宽检查。不过一次固定 seed 的结果不能证明普遍成功率、端到端更快、其他场景可行或算法最优；普通入口不因此切换默认算法。

主要未解决成本是三条通过边的必需网格检查和最终权威复检。没有证据允许调粗网格、削减几何/净空或跳过权威检查。候选重试次数接口不可得；路网 UNKNOWN 数量不代表可行状态。没有 Isaac、真机、完整工艺周期或其他任务集结论。

定向测试最终 **56 项通过**：47 项 Python 合同/身份测试、9 项真实 native 测试（其中 7 项 LazyPRM/RRT 单元机制、2 项原网格/缓存回归）。首轮 46 通过；随后 9 通过、1 项因服务器源副本未包含已提交的 v0.2 归档而失败；补齐原样归档后仅重跑该身份项，1 通过。原始失败日志保留。此前 Windows 本地 29 项合同通过是旧基线预检，不重复计入这 56 项，也不作为原生证据。

本轮真实 worker SHA256：`12e2e94a5ea530d389e5d097fca4bd0daa117c30016b00049827e1b33860e288`；C++ 源码 SHA256：`c7510871593bf7646ea6092de278c30037094737b5cff29a9249f8db54dafcf6`。运行源码归档、每轮测试源码和各证据 SHA 索引与 [provenance.json](validation/evidence/backend_tesseract_ompl_v0_3/provenance.json) 一起保留。旧 v0.1/v0.2 证据与失败均未覆盖。

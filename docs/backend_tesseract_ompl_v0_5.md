# Tesseract + OMPL v0.5：真实 home→pregrasp 的端点接入

本轮只处理 v0.4 保存的 `native-01.request.json`，不运行完整单候选抓放，不运行其他接近候选，不启动 Isaac。工程调用限额为一次诊断复现及一次改后验证。普通入口、默认 uniform 采样、非 star LazyPRM、k=5、setup 连接距离和所有几何/运动学判据保持不变。

结果：唯一改后请求得到 **9 点 exact 路径，native 完整检查及一次 authority 复检通过，最终 VERIFIED**。这解决了该冻结 pregrasp 段的本次阻塞，不等于完整抓放或执行资格通过。

## 1. 起点和终点分别连到了哪里

只读诊断复现结束时，**起点与终点连向不同的候选分量**：

| 端点 | 度数 | 已验证/UNKNOWN 邻边 | 已验证/UNKNOWN 邻居 | 候选分量规模 | 已验证子图分量规模 |
| --- | ---: | --- | --- | ---: | ---: |
| home 起点 | 22 | 0 / 22 | 6 / 16 | 9617 | 1 |
| pregrasp 终点 | 1 | 1 / 0 | 1 / 0 | 2 | 2 |

候选图有 2 个分量，已验证子图有 47 个分量；两种图中起终点都不连通。终点唯一保留邻居与它相距 **1.223854499273956 rad（L2）**，关节状态为：

```text
[0.1363173038746086, -0.43601813802914235, -0.47086408728821916,
 -1.6903150293117175, 0.10834216369147054, -2.5218997320256538]
```

这个节点是当次 uniform 搜索自然生成并检查的邻居，仅用于说明诊断，**没有作为改后采样的中心、路网或中间节点输入**。起点各邻居的关节状态、边/节点状态以及分量信息见诊断原始结果（邻居明细最多 16 条，计数覆盖所有邻居）。起终点 q 与 v0.4 原请求完全相同：

```text
start = [-0.5284209847450256, 0.09451229125261307, -0.21063973009586334,
         -3.1416585445404053, 1.2654187679290771, 0.5286964774131775]
goal  = [0.5000137206581776, -0.18736779957354116, -0.15311780513086534,
         -1.6334118791635104, 0.5008636650157714, -1.4994301903402696]
```

## 2. 采样用完时，为何没有可交付路径

诊断支持 **A：终点长期缺少连接，B：两端区域仍分离**。不能把大量 UNKNOWN 单独称为错误，也没有观察到上游 component 维护错误。

| 累计采样 | 实际状态计算 | 起点度数/分量规模 | 终点度数/分量规模 | 候选连通（当时 OMPL 标签） |
| ---: | ---: | --- | --- | --- |
| 100，端点刚加入 | 3 | 1 / 2 | 1 / 2 | 是，只有 UNKNOWN 直连边 |
| 1000 | 691 | 12 / 710 | 0 / 1 | 否 |
| 2500 | 742 | 18 / 2170 | 0 / 1 | 否 |
| 5000 | 8682 | 21 / 4648 | 1 / 2 | 否 |
| 7500 | 8707 | 20 / 7130 | 1 / 2 | 否 |
| 10000，终止 | 8720 | 22 / 9617 | 1 / 2 | 否 |

原生先检查直连并拒绝；上游随后把端点之间的候选边加入 UNKNOWN 图，再按正常懒验证将其删除。候选连通从来不代表安全路径。其后多个终点相关长连接因箱体、侧墙、顶板碰撞或奇异性被真实检查拒绝。有限拒绝记录保存两端 q、端点关联、失败网格位置、对象对、距离及分类；预算/取消中断单独分类，不附会为碰撞。

LazyPRM 上游只在 `solutionComponent` 找到起终点候选连通时尝试完整路径检查。终点孤立或停留在两节点分量时，新增其他区域节点主要消耗采样/建图资源，而不会持续消耗完整边检查额度。**后 5000 次采样只增加 38 次实际状态计算**，正是本次“采样已耗尽，实际计算只用了 8720/100000”的关键证据。

终止后对实际图做独立遍历：候选图全部保留边（含 UNKNOWN）与已验证子图分别计算，OMPL 活跃 component 标签的分区和大小均与遍历一致。未复制上游 A*、建图或有效性实现。端点原始 vertex flag 仍可能是 UNKNOWN，因此明确记录它们已由同一个状态检查器验收；已验证子图只纳入这些已验端点、上游 VALID 节点和 VALID 边。

**诊断复现与 v0.4 的全部 counters、search_progress、首次失败点和实际计算拒绝统计完全相等**：10000 次累计采样、9900 次迭代、8720 次实际状态计算、8753 次状态请求、19 条边（1 有效、18 无效、0 中断）。这来自实际比对，不以“只加日志”代替验证。最终原因仍是 `CUMULATIVE_SAMPLE_LIMIT`；最后一次碰撞以及最初直连中点碰撞都不是整体终止原因。

## 3. 实际改了哪个机制，为什么

只新增 **StateSamplerAllocator 下的端点邻域＋全域混合采样**，通过 `EndpointSamplingConfig` 显式开启。默认 wire 配置和默认 SeededSampler 保持原样；未补连、改 k、改连接距离、改 seed 或增加检查预算。

工程验证前冻结的唯一配置：

```json
{
  "name": "lazy_prm",
  "max_samples": 10000,
  "max_roadmap_vertices": 10002,
  "max_roadmap_edges": 50010,
  "sampling": {
    "type": "endpoint_mixture",
    "local_probability": 0.5,
    "joint_span_half_width_fraction": 0.05
  }
}
```

建图时每次生成一个样本：50% 概率继续全域 uniform；其余等概率选择当前起点或终点，在各关节 `q_endpoint ± 0.05 × joint_span` 与原关节范围的交集中均匀采样。各维直接生成，没有有效性拒绝采样、夹紧已生成点、隐藏重试或额外配额。所有生成点仍交给上游以 UNKNOWN 加入，只有原完整检查才能赋予 VALID。

设计依据：基线显示终点接入稀少，长边多次失败；六维全域 uniform 落入该局部盒的体积比例至多 `0.1^6=10^-6`，9900 次建图采样的期望局部样本数至多 0.0099。5% 是基于关节范围的固定小邻域，50% 保留全域探索机会，未读取历史成功路径，也未依据改后结果反复调比例或尺度。现有诊断不证明图中绝对没有其他近邻，但足以支持尝试这一项有界端点密度改进；没有证据支持手工补连或修上游组件算法。

setup 投影估计保持原 uniform，单独统计且继续计入累计上限。局部、全域、setup 和其他 sampler 调用的总和必须与累计采样一致；无 sampler 拒绝，所以 `rejected_generation_attempts=0` 不代表几何拒绝为零。所有真实状态/边检查仍共享该请求的 100000 次实际计算预算。

配置及选择依据见 [冻结配置](validation/evidence/backend_tesseract_ompl_v0_5/improvement-config.json) 和 [运行前决策](validation/evidence/backend_tesseract_ompl_v0_5/decision-before-improvement.json)。

## 4. 同预算下端点接入与有效连接

端点接入和实际有效连接均改善，且没有提高资源上限：

| 指标 | 诊断基线（uniform） | 唯一改后请求 |
| --- | ---: | ---: |
| 累计采样 / 上限 10000 | 10000 | 1389 |
| setup / 真正建图采样 | 100 / 9900 | 100 / 1289 |
| 建图全域 / 起点局部 / 终点局部 | 9900 / 0 / 0 | 644 / 314 / 331 |
| 实际状态计算 / 上限 100000 | 8720 | 97494 |
| 状态检查请求 / 精确缓存命中 | 8753 / 33 | 97539 / 45 |
| 边检查：有效 / 几何或运动学拒绝 / 中断 | 1 / 18 / 0 | 18 / 21 / 0 |
| 起点度数 / 已验证邻边 | 22 / 0 | 33 / 1 |
| 终点度数 / 已验证邻边 | 1 / 1 | 16 / 6 |
| 候选图起终点连通 | 否：分量 9617 和 2 | 是：同一分量 751 |
| 已验证子图起终点连通 | 否：分量 1 和 2 | 是：同一分量 19 |
| 节点 / 无向边 | 9619 / 47446 | 751 / 2835 |
| component 独立遍历核对 | 一致 | 一致 |
| native 结果 | 采样预算耗尽 | exact，原生完整检查通过 |

改后 102 个节点已知有效、649 个 UNKNOWN；18 条边 VALID、2817 条 UNKNOWN。仍有大量 UNKNOWN 并不妨碍一条**完整验证**的路径交付；安全连通性只来自已验证子图。内部有效边共 18 条，返回路径使用其中 8 条，不能把所有候选边都算成安全路网。

改后在累计采样 1000 时已记录起点度数 32、终点度数 21、实际状态计算 16494；最终在采样 1389 时停止于首条有效解，没有继续优化。没有拒绝采样：100+644+314+331=1389，所有生成尝试都计入同一个累计上限。

实际状态计算用量达到原上限的 **97.494%**，仅剩 2506 次额度。本次得到路径不表示预算充裕或普遍成功率提高。与基线的低成本失败相比，改后花了更多时间完成真实检查，不是算法速度竞赛或加速结论。

## 5. 是否获得 exact 且 authority 通过的真实 pregrasp 路径

**是。** 原生 `exact_solution=true`、`native_validated=true`；production backend 完成一次现有 `connector._path_failure` 复检后返回 `VERIFIED`、`authority_validated=true`、`deliverable_stage_path=true`。没有第三次 native 请求、authority 拒绝后的探针或优化搜索。

authority 用时 **645.802044 s**，检查 61162 个边细分样本、实际状态计算 61156 次、精确缓存命中 6 次；failure=null。统计字段 `edge_validation_calls=1` 是整条路径入口调用次数，实际处理 8 条相邻路径边，不是只检查了一条边。authority 工作量单独记录，不混入 native 的 100000 次实际状态计算额度。

完整首条路径保留在 [improved/result.json](validation/evidence/backend_tesseract_ompl_v0_5/improved/result.json)，对应 [原生结果](validation/evidence/backend_tesseract_ompl_v0_5/improved/native-01.result.json) 和 [authority 结果](validation/evidence/backend_tesseract_ompl_v0_5/improved/authority.json)。未做 shortcut 或平滑。几何指标：

- 9 个路径点、8 条边；不等于 9 个控制步。
- 关节空间 L2 折线长度：10.441596014539387 rad。
- J1–J6 各关节总变化：`[5.9468934874, 0.9699207868, 2.5853177005, 1.8296352833, 1.8599322223, 5.9170280094] rad`。
- 执行时长为 null，时间参数化和执行 preflight 均 NOT_RUN；不宣称更短、更平顺或已适合物理执行。

## 6. 下一步能否回到完整任务流程

**可以在下一轮回到单候选工艺接线，但不能把本段成功直接当作完整任务成功。** 应先把已冻结的可选采样配置正式传给单候选工具；普通 `--ompl-planner lazy_prm` 入口仍按要求保留原 uniform 默认。后续仍需检验接触、附着、抽离、持物 transit、放置、释放和撤离。当前只验证了这一对真实 home/pregrasp 端点，一次成功不证明其他阶段、seed 或场景的普遍可解性。

本轮已收束为两次工程调用，无完整抓放、无 Isaac。新策略用掉约 97.5% 的 native 状态额度，余量小；native 和 authority 的检查成本仍是后续瓶颈，不能宣称在线性能达标。

后续明确待办：`run_tesseract_ompl_task.py` 的成功分支尚未调用时间参数化和执行 preflight。未来完整任务即使返回几何路径，也需要接上已有时间参数化、实际执行轨迹验收及 preflight；本轮不把 NOT_RUN 写成 PASS，不提前重构执行链。

## 实现边界、只读诊断和测试

`ObservedLazyPRM` 的诊断放在 `roadmap_diagnostics.h`。端点加入后以及累计采样 1000/2500/5000/7500/10000 的里程碑读取摘要，次数固定、有资源上界；中途不做 BFS。终止后才用独立本地索引遍历候选图和已验证子图，不写上游 vertex index/component/validity，不调用 nearest-neighbor 查询、不额外消耗随机数、不验证 UNKNOWN、不导出完整路网。

`ObservedMotion` 只包装原 DenseMotion，暂存其诊断槽以获取本次拒绝见证；每次状态和完整网格的检查仍由原实现执行。最多保存首 16 条拒绝/中断边及首 16 条端点相关记录，两组可能重叠，不能相加当成边数量。中断记录的 `failure=null`，另存 `interruption`，不拿上一碰撞原因解释中断。终止后的图可能已经由上游移除了中断检查所涉候选边，该删除不构成几何碰撞证明。

`Context`（包含 FK、Jacobian/SVD、FCL、成对策略、缓存和资源保护）及 DenseMotion 源码块逐字节未变，SHA256 为 `732a645117609c352a60a1f11c84b9fbc864f533129705e993be0bf4a87a0b8b`。细分合同、authority 检查实现、模型资产和场景导出均未修改，未重新运行旧的完整路径审计。新增采样器不接触图或有效性标记；内部图的建立、删除、NN、component 和路径检查仍走安装的 OMPL 1.7.0 实现。

定向验证：**87 项 pytest 通过（0.49 s）**，其中 82 项为 Python/fake/合同及既有身份回归，5 项使用真实 worker 的小型合成场景；另外一个 C++ 可执行文件运行了 3 个真实上游图夹具和 1 个采样器夹具。

- 图夹具区分 UNKNOWN 候选连通与已验证连通；通过上游 `constructSolution` 删除无效边/无效节点后，度数、分量与标签审计随之变化；仅在测试中故意破坏标签，确认审计能检出。
- 采样器夹具验证 setup uniform、局部范围、全域与两个端点局部采样均有发生、累计计数、原端点不动。采样器接口仅接收当前端点/关节范围及参数，不接收历史路径。
- Python/native 测试覆盖默认 wire 配置不变、参数合法性/未知字段拒绝、实际 sampler 回显、只读诊断不改变小型搜索的计数和路径、中断边分类、请求间路网清空，以及 exact 和 authority 必须同时满足。
- 既有取消、场景过期、失败停止传播和“不支持时不 fallback”定向合同测试继续通过。合成测试不是 FANUC 工程成功证明，也不是物理执行。

## 请求、基线及构建身份

本地和远端初始 HEAD 均为审阅提交 `92471392b5d25fa76f2846c361cc7a159ecfa5aa`，工作树干净。没有 reset、force push、其他分支修改、依赖重装、额外候选或额外工程调用。

薄入口 `tools/run_tesseract_ompl_v5_request.py` 直接构造同一生产 backend 的 FreeMotionRequest：从 v0.4 原始 native 请求读取两个真实端点，原 seed=71081、pregrasp、attachment=null、100000 次实际计算/10000 次累计采样/10002 节点/50010 边上限、max_attempts=1。fixture_context 只恢复相同世界与 authority；不调用 connector.plan 或 IK。重新导出的完整 scene 与保存值做结构精确比较，资产绝对路径未变化，未手改 fingerprint。

冻结世界仍为 37 箱，scene fingerprint `ec7c545c2233047bdad2c42d49bea2d438e7fd7504b6553407399bf380161b90`。诊断与改后请求均从空路网开始；运行目录必须新建，防止覆盖证据或无意重跑。production backend 继续要求 exact、原生完整有效、起终点/场景一致和 authority 通过，authority 拒绝也不会触发探针或第三次规划。

| 构建 | worker SHA256 |
| --- | --- |
| v0.4 原 worker（未覆盖） | `12e2e94a5ea530d389e5d097fca4bd0daa117c30016b00049827e1b33860e288` |
| 只读诊断构建 | `a04aecc8ddf6c6bbcfdc201b91acd00aa067713801607475b2e95873cdc7d683` |
| 改后构建 | `742607df1eca48881f48f463c9bf021be5144183e2959addcdca70d1a572d53f` |

两个新 worker 在独立 build 目录以原依赖编译，Tesseract 0.35.0 / OMPL 1.7.0 / FCL 0.7.0 不变。各次 identity.json 记录真实 worker、源文件、原请求和配置 SHA，另保留两份执行源码归档及构建日志。诊断结果不冒充改后二进制结果；v0.4 失败及 v0.1–v0.3 证据不覆盖。

## 时钟和证据口径

诊断复现 native 总时间 12.856020 s，原 v0.4 为 12.515146 s。只读图摘要和终止遍历聚合计时 0.012749 s；它不包含所有包装器/JSON 开销，两个总时间之差也不是受控测得的插桩开销。两次都保留原 profile，不能据此声称无插桩性能或普遍加速。

| 唯一改后请求时钟 | 秒 |
| --- | ---: |
| native 总计 | 459.985003 |
| 其中环境初始化 | 0.201425 |
| 其中 OMPL solve | 459.764636 |
| worker 往返（不含冷启动） | 460.100221 |
| worker 冷启动 | 0.039900 |
| 图诊断聚合（中途＋终止） | 0.001026 |
| FK/场景状态 | 6.599427 |
| Jacobian/SVD | 0.552704 |
| 碰撞对象变换 | 11.784400 |
| FCL contactTest | 438.218281 |
| 预算/取消轮询 | 0.269273 |
| 边调度自身 | 0.042852 |
| 状态检查 inclusive | 459.703425 |
| 边检查 inclusive | 458.966102 |
| authority | 645.802044 |
| backend 调用及结果落盘合计 inclusive | 1106.047419 |

FCL 占 native 约 95.3%。改后原生检查明显更多，耗时也更长；这里的改善是同资源上限内接入了端点并完成真实有效路径，不是壁钟加速。没有另跑无 profile 的工程请求测插桩开销。

native 总时间包含环境初始化、端点/直连和 OMPL solve；solve 包含其中的状态/边检查及中途诊断；终止后图遍历在 solve 外、native 总时间内。`roadmap_diagnostics_s_inclusive` 横跨中途及终止诊断，不能再加到 native 总时间。状态 inclusive 与边 inclusive 相互嵌套，FCL/FK/Jacobian/变换计时是其中子项；worker 往返还包含序列化/通信，backend 总时间另包含输入校验、证据写入和条件触发的 authority。所有这些时钟不得重复相加。

证据目录：`docs/validation/evidence/backend_tesseract_ompl_v0_5/`。保留原始请求/结果、拒绝边、里程碑、终止图审计、参数冻结依据、源码与 worker 身份、构建及测试日志。时间参数化、执行 preflight 和 Isaac 始终不在本轮运行范围内。

入口：[summary.json](validation/evidence/backend_tesseract_ompl_v0_5/summary.json)、[基线复现比对](validation/evidence/backend_tesseract_ompl_v0_5/baseline-comparison.json)、[改后输入与源码核对](validation/evidence/backend_tesseract_ompl_v0_5/improved-input-source-verification.json)、[检查器字节身份](validation/evidence/backend_tesseract_ompl_v0_5/checker-identity.json)。实际命令见 [诊断调用](validation/evidence/backend_tesseract_ompl_v0_5/run-diagnostic.sh)、[构建及定向测试](validation/evidence/backend_tesseract_ompl_v0_5/build-and-test.sh)、[唯一改后调用](validation/evidence/backend_tesseract_ompl_v0_5/run-improved.sh)。

# 无先验链路：持箱搬运至固定预放置（2026-10-16）

**c02 完整生成到 preplace，8.586939 s；c04 未完成，全部有界尝试共 21.991508 s。** 两例均在最终冻结源码上各运行一次，没有调目标、种子、预算或重跑。成功终点仍附着，未释放、未接收、未卸箱。

状态：PLANNING_ONLY_NOT_EXECUTABLE；qualification_status=NOT_EVALUATED；continuous_swept_proof=false；physical_execution=NOT_RUN。

[机器可读汇总](experiments/m710_single_preplace_20261016/summary.json)保存四段实际起终 q、来源、Task/stage 父链、attachment、目标/实际位姿及失败；[逐面计时](experiments/m710_single_preplace_20261016/timing.csv)；[原始文件摘要](experiments/m710_single_preplace_20261016/raw-manifest.json)对应本轮完整结果、轨迹、请求/响应和 worker 日志。归档只用于复核。

## 改动与固定目标

新增显式互斥选项 **--through-preplace**。默认 contact-only 及 --through-extraction 保持原行为。SinglePreplaceConnector 复用原三段，在 extraction 成功且 _contact_context() 尚未退出时接续，使用实际末 q、receipt、固定 attachment 和终端几何 mask，未重新捕获或改末样本。

唯一目标采用配置 conveyor_longitudinal 中心、允许列表首项 TOP_DOWN：物理接触 Z 轴向下，长杯阵 Y 轴沿纵带出料方向 −X，X 轴由右手系确定。由实际 T_CB 求箱体旋转，再用旋转后包络决定高度。本次正面抓取对应箱体局部 X 轴近似竖直，未假定局部 Z 竖直。

未来落放意图中心约 [-1.6, -0.75, 0.9] m；预放置中心约 [-1.6, -0.75, 1.0] m；接收平面 Z=0.6 m。沿用 0.10 m 退距、0.01 m 边缘储备和既有接收运行净空储备，复用完整底面多边形并集覆盖、交叉区纵带优先、占用及正常净空规则。目标处检查完整负载和 **202 个工具实体**。两例正面唯一目标均几何适用，支撑检查针对未来落放意图，预放置仍悬空。

工具目标严格使用 T_WC = T_WB × inverse(T_CB)，再转换虚拟 TCP，核验 SE(3) 与往返一致性。transit 从实际 extraction 末 q 做一次有限 KDL IK，成功才发一次持箱 OMPL RRTConnect。未使用旧完整 plan、_approach、_connect_pose、_transit 多方法调度或旧放置候选；无 PTP、先验、多 IK 候选或换 seed。

transit 使用 FREE_LOADED_TRANSFER、空 support_names 和空 initial_proximity，保留正常机器人/工具/负载碰撞及密封/压缩要求。新 check_transit_end 仅检查实际终点一个状态，绑定同一 Task 的 transit 父记录；白名单、模式限制及请求证据已同步。没有新运动求解、密集路径复检或将 transit 伪装为 extraction。

## 实际结果

| 案例 | 完整到 preplace | 选面及回退 | 总规划 s | IK / OMPL / LIN / PTP / 先验 |
|---|---|---|---:|---|
| c02 | 是 | 首个 FRONT x−，无回退 | 8.586939 | 2 / 2 / 2 / 0 / 0 |
| c04 | 否 | FRONT→TOP→SIDE y−→SIDE y+ | 21.991508 | 5 / 2 / 3 / 0 / 0 |

c02 四段分别返回 **43 / 31 / 159 / 109** 个点，相邻实际 q 完全连续，父链同属一个 Task；最终六轴速度为 0。本次 48 个命令杯通过终端几何接触检查，实际 mask 保持至 transit 完成。只有一次规划 ATTACH，目标不重复出现在 world，没有 PLACE/RELEASE 事件。实际终点箱体位置误差 **1.4963e−6 m**、旋转误差 **5.6432e−7 rad**，物理接触/虚拟 TCP 残差也满足原阈值；原生终点正常净空检查通过，目标仍 attached。

c04 的失败完整保留，不称为全局不可达：

| 面 | 该面总尝试 s | 到达位置与失败 |
|---|---:|---|
| FRONT x− | 12.565255 | 三段和实际分离通过；唯一 preplace 单次 KDL IK 返回 NATIVE_IK_FAILED（原码 −6），未派发 transit OMPL |
| TOP z+ | 0.158353 | 单次接近 IK 未完成 |
| SIDE y− | 9.118010 | 接近、接触成功；既有上界内返回 NO_POSITIVE_GEOMETRIC_EXTRACTION_WITHIN_BOUND，未派发抽离 LIN |
| SIDE y+ | 0.129250 | 单次接近 IK 未完成 |

每面从原始未附着起点和原世界开始；实际请求无废弃附着、mask、父链继承。c04 最终 complete=false，没有把诊断前缀作为四段结果。

## 耗时及保留的检查

单位秒；c04 分项累计全部面尝试。初始化包含场景准备，不计规划。阶段时间包含请求构造、通信和返回处理；原生求解/检查时间嵌套于阶段，不再相加。

| 项目 | c02 | c04 |
|---|---:|---:|
| 目标列表生成 | 0.014386 | 0.015490 |
| 当前面目标/杯几何 | 0.013604 | 0.037917 |
| 显式接近 IK | 0.058752 | 0.212480 |
| pregrasp | 1.048770 | 18.247877 |
| contact | 0.251899 | 0.517868 |
| 终端接触核查 | 0.008761 | 0.017680 |
| 附着上下文 | 0.007414 | 0.014322 |
| 抽离目标生成（含失败面） | 0.339947 | 0.642173 |
| extraction | 0.471310 | 0.450882 |
| 返回抽离点核查 | 1.208360 | 1.117555 |
| 抽离末态自由空间检查 | 0.085526 | 0.079264 |
| 唯一 preplace 目标 | 0.138640 | 0.131966 |
| 显式 transit IK | 0.051722 | 0.049432 |
| transit | 4.615810 | 未到达 / null |
| 实际 preplace 终点核查 | 0.090951 | 未到达 / null |
| 完整尝试总规划 | 8.586939 | 21.991508 |
| 初始化（不计规划） | 1.691467 | 1.701699 |
| 其中场景准备 | 0.013161 | 0.012877 |

c02 主要耗时为 transit（4.615810 s）；c04 两次 pregrasp 累计约 18.248 s。正面唯一预放置 IK 未完成及其余面的明确失败，是本轮具体未完成原因。本轮不修改搜索、不增加 IK 重启，也不外推五箱时间。

| 实际 Clearance 工作量 | c02 | c04 |
|---|---:|---:|
| 状态检查 | 3,849 | 14,662 |
| AABB 测试 / 跳过 | 29,771,551 / 29,563,024 | 91,710,868 / 90,936,723 |
| FCL 窄查询（含补充） | 208,897 | 774,271 |
| 内部检查秒（嵌套） | 1.442996 | 4.960111 |

c02 transit 单独检查 2,984 状态、FCL 窄查询 197,560 次，内部检查 1.232602 s。当前工作区重新建立 AABB/窄阶段资格 **8 / 122**，各复用 **21,960 / 334,897** 次，122 次补充查询成本计入请求。搜索真实拒绝了负载/车厢右壁等碰撞状态，未关闭检查。复用数不等于省掉的 FCL 调用。

ProcessSealInput 每请求编译一次；transit 实际执行 2,211 次密封几何、159,192 次压缩 SAT，最终密封判定复用为 0。既有优化保持开启，所有实际检查响应 pair_profile_enabled=false、逐对象对细项为 null；没有新增逐对象对/逐吸盘计时。汇总只读取独立请求响应并按 request_id 查重，不重复加祖先统计。

## 版本、验证与复现

起始本地/远端均为 fd950d71b2e1adb7b07bd721bc9e8c60b4fc4328，工作区干净，无增量。实测源码 **b14c64e67dd7ff36e9cf5b2aadf234fed2aeee78**，实测后未修改规划代码。最终交付在此提交上只追加报告与证据，因此最终交付 SHA 与实测源码 SHA 不同；新增源码及报告均为 UTF-8。

- 定向 Python **105 passed / 1.78 s**，新增预放置回归 42 项，覆盖旧入口、末态/attachment、mask、旋转底面/方向/占用/边界/净空、协议、来源、失败前缀、换面恢复及请求级中止。
- Release 重建 worker；m710_native_semantic_event_test、m710_request_evidence_test 通过。
- 正式案例前向已初始化真实 worker 发一次非法作用域 check_transit_end，返回 TRANSIT_END_CHECK_SCOPE_INVALID，证明入口可达；**零 IK、零运动请求**。关闭探针进程后，案例使用新 worker。c02 实际通过该操作成功分支。
- 运行后只核对已有记录的身份、连续性、权限、mask、调用计数，没有额外求解或密集几何检查。

[冻结身份](experiments/m710_single_preplace_20261016/freeze.json)：worker SHA256 **9923c9061983ac3c2162e473c6f99a77d579c9e409baea5afdc5e00f4ba924f5f**，99 个模型/配置资产与前轮逐项相同。

Ubuntu 22.04.5 / ROS Humble，MoveIt/Pilz 2.5.10、MTC 0.1.3、FCL 0.7.0、g++ 11.4，Release -O3 -DNDEBUG；Xeon Platinum 8352V、CPU 0–21、OMP/OpenBLAS=1。未升级环境。官方机器人、完整工具、20 kg 工具、42.5 kg 箱体及 5 mm 正常净空不变。

c02 从配置完整 40 箱静止场景开始；c04 仅恢复冻结输入的实际起始 q 及已理想移除四箱，剩余 36 箱。输入内旧 FAST/SLOW 等元数据不参与本入口调度，不含成功终点/路径/mask。均非本次物理测量。

[完整实际命令](experiments/m710_single_preplace_20261016/run-cases.sh)。任务共享各面/阶段截止 300 s，阶段 90 s，IPC 100 s，seed=71070，未重置预算。隔离环境内入口：

~~~bash
source /opt/ros/humble/setup.bash
cd /work-single-contact-20261016
export PYTHONPATH=src OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONNOUSERSITE=1
export M710_MOVEIT_ASSET_ROOT=/work-single-contact-20261016
O=/tmp/single-preplace-20261016
taskset -c 0-21 /opt/planning-opt-20261003-venv/bin/python tools/plan_m710_approach_contact.py \
  --through-preplace --target carton_l07_c02 --output "$O/c02" \
  --worker-command "$O/worker.sh" --worker-binary "$O/build/m710_moveit_worker" \
  --source-commit b14c64e67dd7ff36e9cf5b2aadf234fed2aeee78 \
  --seed 71070 --task-budget-s 300 --stage-budget-s 90 --ipc-timeout-s 100
# c04 独立输出 c04，并增加 --initial-state "$O/c04-input.json"。
# 复现另选不存在的输出目录，入口拒绝覆盖已有结果。
~~~

运行 ID：c02 single-contact-db9082cd675d4671873cfa8d871cc27d；
c04 single-contact-7628da1301f14f60b202478cd73cc64f。

旧完整取放及五箱入口尚未迁移到新链路。新链路止于 attached preplace，未接 place/release/withdrawal；未运行视觉、真实吸附、Isaac、真机、preflight、执行包、连续扫掠/重型资格验收、全量 pytest、五箱/40 箱或历史任务集。保留本轮结果后停止。

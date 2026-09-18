# 箱箱净空语义、脱垛转换与机制验证

## 基线与实现

基线 `2a6fbda630d83929e973c020f7aa37f636d3dd71`；生产净空修复提交
`b49847461f105a6ea43f1ffda98558c625d0d490`，追加执行保持保护为
`b5c457086709d92d3b065c2256cda329c2c94bc9`。原第二箱失败记录及上一轮完成数不变。
隔离目录为 `/root/autodl-tmp/m710-stack-clearance-20260918/`，沿用服务器环境，未安装依赖。

PhysX 接触点 `separation` 是接触特征沿其法向的分离量，不是两实体的欧氏最短表面距离。
当前安装 Isaac Sim `6.0.1-rc.7+release.42383.32955d8d.gl`，PhysX 扩展
`110.1.13+110.1.2.lx64.r.cp312.u7f4`。实际 API 暴露位置、法向、冲量、分离量和两个
face index；未暴露 manifold ID，也未暴露接触生成的精确子时刻。接触缓冲仅在当前步有效，
回调立即复制数据。参见 [NVIDIA 接触报告接口](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/extensions/runtime/source/omni.physx/docs/dev_guide/contact_reports.html)
和 [PxContactPairPoint 定义](https://nvidia-omniverse.github.io/PhysX/physx/5.1.0/_build/physx/latest/struct_px_contact_pair_point.html)。
本机接口文件、版本与 SHA 见精简证据 `installed_contact_api.json`。

生产新增单任务 `StackClearanceStep`，仅接管当前目标与具名原堆垛箱对。
适配器验证每箱一个原生 Cube、刚体及 collider 所有权、单位、缩放、局部几何与张量视图顺序。
静态形状来自实际创建并验证的碰撞体；当前位姿和速度来自 PhysX 张量，未使用 USD 显示位姿。
contactOffset 只参与接触点来源合理性核查，不膨胀原始实体尺寸或降低工程净空。

回调采集原始接触，步后一次计算对应箱体的欧氏距离和相交信息，供接触裁决与
`ActualStackContactMonitor` 共同消费。明确分别标记步前测量、步内接触生成和步后测量；
不把相同 step 编号声称为相同子时刻。小正 separation 不再直接冒充工程距离。
机器人、刚性工具、接收机及未知对象仍走原保护链路。

进入阈值仍为 **5.2 mm**，维持阈值仍为 **5 mm**。自由空间锁存不回退，正式 free-transit
边界门另外保留；阶段时间到达本身不授予通行。负 separation、响应与当前几何存在未解决
矛盾时保存首个冲突，冻结轨迹推进并沿用既有有界物理等待后停机，不通过后续 LOST 擦除。
零点记录显式转移并记录来源；缺少必要证据不放行，也不伪造 LOST。
每任务新建目标、邻箱、pending 和锁存上下文。转换及冲突附近各保留有限前后步证据。

追加执行保护使未决保持期间不能触发抓取、实际释放或首次理想接管；合法理想接管完成后
结束该目标的物理箱箱监测。该变化不新增碰撞豁免。执行实现指纹加入新模块，保留完整绑定校验。

**未改** 5 mm 外部净空、5.2/5 mm 转换门槛、20–50 mm 实际释放范围、25/35/45 mm 名义候选、
5 mm 高度储备、3 mm 定向储备、墙体、工具 CAD、接收顶面身份、阶段接触许可、杯邻箱及
具名 J5/J6 自有工具例外、布局、质量/惯量、IK/FK、关节/Jacobian/径向门槛、PD、力矩、
速度、插值、240 Hz 物理频率、contactOffset/restOffset 或接触响应。未恢复规划墙钟截止。

## 定向测试与旧失败 fixture

服务器 canonical `b498474` 运行八个定向模块：**122 passed in 16.81 s**。
模块为 stack_clearance_semantics、isaac_contact_callback_index、m710_runtime_contact_policy、
zero_point_contact_resolver、poc_release_reserves、ideal_release_handoff、poc_continuation_state、
obb_query_identity。未跑全量 pytest、旧 104/129、40 箱或性能扫描。

覆盖 6 mm 通过、自由规则下 4 mm 拒绝、阶段内有限接触及非法相交拒绝、5.2 mm 进入/
5 mm 保持、微小正接触分离量与充分实体间距、负值/冲量/错误对象/无效测量/零点 pending、
LOST 不擦除首个冲突、任务切换、冻结输入不被修改。实际运行器回调被加载并调用生产裁决器，
没有替换核心裁决。原 4.958109 mm transit、释放交接和 step1.1 续箱回归通过。

追加保持保护后在独立服务器目录运行本轮模块：**18 passed in 0.36 s**。
新增用例执行运行器实际抓取、释放、首次接管条件，验证生产冲突保持会阻止动作。

原失败 fixture 的真实 OBB 查询得到 **5.388677305227511 mm**，原记录 separation 为
**0.06643665983574465 mm**。两者来源均保留。旧 fixture 没有同一步接触点、法向和冲量，
不能用保存姿态的几何通过追认旧整个过程安全，也不能据此确认缓存陈旧。
同输入定向用例证明：小正接触分离量曾被旧分类器按 5 mm 拒绝，新接线在验证形状、
测量和无异常响应后以真实 6 mm 几何通过；低于 5 mm 的几何仍拒绝。

## 一次连续滑出机制试验

仅执行一次双箱机制世界；不是机器人任务，也不贡献最高排完成数。
两原生箱均为 0.6 × 0.4 × 0.3 m、42.5 kg，使用原材料、惯量、阻尼、16/4 求解器迭代、
CPU PhysX/MBP/CCD/Fabric、240 Hz、contactOffset 10 mm/restOffset 0。
下箱固定支撑，上箱先自由静置 0.5 s，记录 **388 个初始有响应接触点**，随后以测得的实际
状态建立有限力直线约束，连续沿边滑出。机制驱动 1000 N、20000 N/m、2000 Ns/m，
不修改机器人 PD；运动期间刚体位姿重置为 0 次。录像 640×360、5 fps、正常物理时间。

结果 **REVIEW_REQUIRED**，物理时间 **13.616667 s**。步 3027、extraction、仍附着、
尚未锁存自由空间时，`/Probe/target` 与 `/Probe/neighbor` 的步后欧氏距离为
**5.281648280816620 mm**，但接触点 separation 约 **−0.000337304 / −0.000352205 mm**，
法向近 +Z，两个非零法向冲量为 **0.044600356 / 0.033761516 N·s**。
保留 `STACK_CONTACT_GEOMETRY_EVIDENCE_CONFLICT`，保持约 1 s 后停止；最终几何距离
6.060562 mm 不用于擦除步内证据。

本试验没有重现“微小正 separation 与充分几何距离”的分歧，没有确认具体 manifold 路径。
负接触点值也不能独自证明原始实体在步后穿透。试验没有到达自由空间锁存或反向逼近 4 mm
环节，因此其物理保护覆盖不完整；4 mm/相交拒绝仅有本轮生产定向测试证据。
当时试验修订仅保留末尾环和自由转换环，首冲突保存接触与距离，未保存该步完整前后位姿；
邻近 5 fps 状态仍在服务器。之后补齐冲突窗口，未为补日志重跑机制世界。
机制实际源码四文件 SHA 单独记录于 `mechanism_source_sha256.json`，不能把它当成最终源码实跑。

精简 fixture 另保存实际采样步 3023 和 3071 的前后状态及原始接触，可用
`PYTHONPATH=src python docs/evidence/m710_poc_stack_clearance_semantics/replay_mechanism_conflict.py`
离线重放最终生产裁决。步 3023 的 4.447302 mm 尚处于既有脱垛接触许可、未进入自由空间；
步 3071 的 6.060562 mm 仍有矛盾接触证据，复查保持且不锁存自由空间。
这不是把两个 5 fps 样本插值成未保存的完整物理过程。旧记录中的原生形状与单独保存的
实际 backend contactOffset 按原刚体顺序显式关联，未补造测量。

## 整机阶段与证据边界

机制尚未安全通过，按“机制通过后”阶段条件，尚未启动新整机 Isaac 世界；第二箱原阻塞
尚未通过本轮整机复验。**本轮新增同世界完成数 0/5（整机未执行）**，不是旧世界 1/5。
五箱的抓取、脱垛、释放、理想接收、撤离和送出均未执行；新增实际接收与理想接收均为 0。
真实驱动力矩遥测缺失仍单列，未作为前置阻塞。

首箱及续箱配置已提前配对核查，见 `config_pairing_before_planning.json`。CPU 完整初始路径
准备使用正常选择、当前政策与历史候选重验，没有历史完成事件，没有外层 timeout。
正常入口选择 `carton_l07_c02`，完整路径通过，规划墙钟 **1334.471503 s**，无后续优化。
原 `b498474` 的结果和执行包保存在 `repo/outputs/initial_plan/first_feasible/`。
规划进程结束后才同步执行保持修复；对相同 motion 使用正常 preflight/export/readback
重新生成绑定 `b5c4570` 的执行包，三项均通过，耗时分别 **0.289296 / 0.309670 / 0.114442 s**。
新包在 `repo/outputs/initial_plan_delivery/first_feasible/`，文件及目录已设为只读。
原包不改标签，不把 CPU 准备记作整机物理结果。

下一步应解决上述负接触点值/非零冲量与箱体几何的证据冲突及允许接触阶段的关系；
在来源无法核实前保留停机，不以几何已分离自动抹去响应，也不扩大穿透许可。

精简证据：`docs/evidence/m710_poc_stack_clearance_semantics/`。
服务器原始机制结果、接触、邻近状态与视频：
`/root/autodl-tmp/m710-stack-clearance-20260918/repo/outputs/stack_slide/`。
视频 `slide.mp4`，68 帧均已解码检查，SHA-256
`cfd0f9197431c11dc4331fb9b947a2817a02a4b701cd9097549a0833055006d49`。
大日志位于任务根目录 `stack_slide.log`；旧物理报告和证据未改写。

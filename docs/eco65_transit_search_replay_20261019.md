# ECO65 带载转运与真实搜索查看器（2026-10-19）

本轮已接入真实 OMPL 搜索记录与离线三维查看器，修复了抽离空间、目标 IK 选择、J1 方向和多段 transit 合同。新的冷规划通过 HOME→pregrasp→contact→extraction→第一段带载 transit。**完整单箱尚未完成**：第二段到预放置点仍被桅杆间隙与带载环境约束阻挡。没有完整计划、完整运动 HTML 或 Isaac 预览；局部成功不作为完整任务交付。

基线 `c35845e7bac74b923089a4a49b0020c035bb7a96`；工作分支 `feat/v0.6-eco65-moveit2-native-single-carton`。仅从参考提交 `9fbd6021a54d20dfa7fb219bc154058a18c2ba7e` 读取模块，未修改或推送参考分支。固定场景、ECO65-B、12 杯、桅杆/相机及全部碰撞几何保持原身份。快照 SHA-256：`ad20a4a45f4a9461a2b9a458d82360fe37374e3dd8bf01c4cfae1052dbd89f51`。

精简原始结果见 [summary.json](experiments/eco65_transit_20261019/summary.json)。其中 `SUCCESS` 请求、诊断结束和完整任务成功是不同字段；报告列出未完成和中断，没有把近似解拼接为完整轨迹。

## 失败证据与有限修复

1. 固定旧正面请求重新实际求解，使用新诊断身份。原抽离只有 47.5 mm，箱体后缘 X=0.2325 m，仍在邻箱前沿 X=0.13 m 之后。`diag02` 原生超时，实际得到 START 7 / GOAL 186 个顶点、182 条边、344 次扩展、8898 次自定义检查，无截断。主要拒绝为目标与邻箱；既有约 2.4 mm 输出门槛与 3.4 mm 带载搜索门槛分别保留。该旧请求没有启用新 J1 策略，是固定基线诊断。
2. 顶部抽离改为整箱离开堆垛前沿：163.4 mm 外退配合 10/40 mm 抬升，两者都通过 native LIN 和一次基本检查。但转动时仍可能触及邻箱；仅使轴对齐箱体离开前沿不等于有足够转动空间。
3. 加入箱体半对角线与 20 mm 额外空间，向上取整至 5 mm 格点，得到 213.4 mm 外退、40 mm 抬升，后缘 X=0.0666 m。实际 LIN 通过；未改变净空阈值。较早的未取整 211.4776406 mm 分支发生 1.146/2.067 μm 末端误差，超过原有 1 μm 门槛，连同第三次低速重算失败一起保留。
4. 增加四组有限肘/腕 IK 种子、用原生 IK 寻找较近的腕部分支。只改变种子和排序，不修改返回 q，不加减 2π。预放置必须允许随后 J1 非减返回固定 HOME；因此旧 J1≈+1.073 的正面目标不能用于新完整任务。
5. 低位前方过渡点实际碰撞 base_link/link_1；仅抬高后碰撞 link_2 或未找到 IK。转向后的两个前方提案也未找到可用 IK。保留这些失败，没有移除几何。
6. 当前 HOME 姿态携带本次真实附件后通过搜索状态检查，故加入“当前 HOME 位姿、带载重新求解”的中间提案。`top08` 和新冷规划都实际通过 213.4 mm LIN 抽离及第一段 native PTP：J1 从约 2.924035 降到 −0.503195 rad。这个带载中间姿态仍叫 transit，目标未释放，不是完成后的 HOME。
7. 从该带载姿态到预放置的 PTP 被 link_3/桅杆拒绝；90 s OMPL 得到近似解，实际 START 84 / GOAL 48、337 次扩展、5920 次检查。主要同次失败为 link_3/桅杆和目标/右墙。没有接受近似路径。
8. 四个定向状态检查进一步定位固定几何限制：J2=0.6 rad 时 link_2/`eco65_mast_mast_part_019` 表面间隙 0.0018142383897602975 m，小于原有 0.0024 m；J2=0.7 时同样不足。桅杆挂在 link_1，link_2 也是其关节子系，这一对象对的相对变换只依赖 J2；改变其余关节的数值核对差为 1.18e-16。J2 有限范围为 [−3.106, 2.356] rad，不能绕模跨过该间隙障碍。
9. 对四种已有顶部朝向做有限接触/预放置 IK 筛选：一种未找到通过检查的接触 IK；另外三种找到的可返回预放置构型均为 J2≈1.94–1.98 rad，位于上述障碍另一侧。此结论针对已检查的构型，不是对全部 1488 个几何候选或全部 IK 解的无解证明。

没有改 HOME、接收区、箱体、桅杆、相机或碰撞间隙以制造通过。仍需找到与当前初态处于同一可行连通区域、同时满足返回方向的预放置构型。当前有限方案未找到它。

## 代码与记录语义

- `search_trace.{h,cpp}`、项目内 pluginlib manager、XML/CMake 来自固定参考提交；保留 OMPL BSD 许可和 Three.js 许可，未修改系统 OMPL/MoveIt。现有 worker 按原控制流合入接口，未整文件覆盖。
- 在实际 planner/context 清理之前调用实际 `getPlannerData()`，保存原 ID、边、树标签、root、join、原始精确/近似解。扩展事件来自实际 growTree；失败 q/对象对/距离/阈值只来自同一次既有 Clearance/ProcessPolicy 调用。无对应见证标为 UNKNOWN，不复用残留失败，也不追加碰撞计算。
- 默认记录关闭；`--search-trace` 开启，`--trace-stage` 按语义阶段筛选，`--trace-request` 按唯一 stage_id 筛选。默认仅记录 transit。事件/样本上限 20000/40000；截断只影响存储。独立 gzip trace 通过 SHA-256 引用，回溯不 deepcopy 大 trace；完整请求/回复保存在 gzip JSONL。
- 区分实际 READY 回复中的 resident seed 与请求 seed 标签：OMPL RNG 在驻留进程初始化时设种子，并随真实调用推进，不把每个请求标签冒称为重新设种子。旧 trace 的该元数据从同次归档的 READY 回复补充，有明确来源；不重算树。归档耗时另记于 trace 引用。
- J1 原始、未取模：contact/extraction/transit/place 非增，withdrawal/residence 非减，数值容差 1e-7 rad；pregrasp 不加这项方向限制。START 检查 parent→new，GOAL 检查 new→parent。记录关闭时仍执行方向策略；Pilz 和最终基本输出循环也检查每条边。
- 完整合同允许 1–4 个相邻 transit，总计 7–10 段；每段保留 native stage_id、真实 parent、原始请求/结果、相同附件和独立时间。没有把拼接段伪装成一次原生求解。
- 对完整计划每个 `goal_pose` 重新计算实际末端 FK，与当前请求比较，位置/角度各 1e-6。求解、基本输出和返回预留分别入预算；RPC 看门狗覆盖请求预算及额外返回宽限。
- `chain_integration_v1` 保持原一次基本采样检查；未恢复 Python 密集 LIN 审计、独立 MuJoCo、task_audit/compose/validate 或动力学。全部产物仍为 `PLANNING_ONLY_NOT_EXECUTABLE` / `KINEMATIC_REPLAY`。

## 查看器

本地交付文件（均被 Git 忽略）：

- `outputs/eco65_native/transit_20261019_view05/comparison.html`：固定正面失败与修复后 HOME 过渡失败的对照，5 次真实请求。
- `outputs/eco65_native/transit_20261019_view02/top-transfer.html`：两种抽离高度、12 次真实请求的顶部转运调查。
- `outputs/eco65_native/transit_20261019_browser05/checked.png`：当前对照查看器的真实 Edge 测试截图。

Windows Edge/Chrome 直接双击 HTML。全部脚本和几何嵌入，无 CDN、fetch 或本地服务器要求。场景含 78 个可视网格、792 个碰撞形状、全部 12 杯；不可见未选杯仍保留碰撞形状。真实 mesh/凸体使用其局部变换；可视/碰撞两层独立。重复 world 几何按内容键共享。

FK 按真实 URDF 父子链计算；J1 桅杆/相机跟随 link_1。附件仅应用 `T_D_link6(q) @ attachment.pose`，不再把目标重复放入 world。D 坐标下显示，不重复应用 T_W_D。失败仅高亮有真实对象 ID 的几何，不生成未记录的接触点。

实际树的起点蓝、目标紫，不随 RRT 内部交换改变。节点/拒绝扩展可点击；原始解、返回路径、完整计划层分开。无返回路径时，关节曲线可以显示原始搜索解的点序，并明确不是运动时间。搜索事件播放与运动播放独立；诊断路径只作单请求显示。完整计划模式只能读取经合同校验的完整计划，以重新定时后的 q/times 和共同进度五次插值播放。没有完整计划时，该选项禁用，不拼接部分成功分支。

## 检查与耗时

定向 Python 六模块：140 passed，2 skipped，16.18 s，包括双 transit 的完整合成合同/错误 parent/附件回归。范围限于 ECO65 profile/合同/分段/方向/FK 及保留的 native-cold/TCP 相关回归。原生 semantic-event 测试通过，为合成作用域测试，不是物理执行。

原生真实 OMPL 记录测试六组全部通过，另一个预算测试通过：

| 方向 | 记录关闭 / 开启 / 上限=1 | 结果 |
|---|---|---|
| 无方向策略 | 均 868 次检查、25 点路径 | 完整检查序列、q 路径逐项相同 |
| J1 非减 | 均 2453 次检查、24 点路径 | 完整检查序列、q 路径逐项相同，START/GOAL 边方向正确 |

记录限额未终止规划。慢检查、小扩展步长的 20 ms 预算测试在 20.24508 ms 返回，未生成精确解。有限墙钟求解仍可能因开销和运行调度出现不同结果；上述确定性测试不是完整场景性能保证，真实请求超时问题仍在。

`diag02` 请求往返 122.339589 s、总计 126.057237 s。`top03` 434.614466 s；修复后 `top08` 170.211211 s。这些是记录开启的诊断成本，不是完整任务成功用时。`top08` 记录回调 0.001996619 s，PlannerData 快照 0.001352523 s；这个口径不包含 worker JSON、传输、gzip 和导出，不能用作全部记录开销。

最终对照 HTML 导出 6.626313 s，63,362,387 字节，SHA-256 为 `2339a450e04eeed04b92957051ce2c6ef2c5515b80a950cb3340a13da704d89c`。Edge 的 17 项实际检查通过，耗时 22.796248 s，浏览器 FK 最大矩阵差 1.11e-16；完整播放明确为 NOT_RUN_NO_COMPLETE_PLAN。此前 12 请求 HTML 导出 9.825840 s，Edge 校验 27.585136 s。没有本轮记录关闭的完整任务性能样本；完整播放/Isaac 时间为空，不以诊断动画替代。

新冷规划 `cold02` 的第一条有效前缀如下。耗时为该原生请求往返，不包含该阶段之前的 IK、失败尝试或全任务回溯。

| 阶段 / 原生 ID | 请求耗时 s | 基本检查耗时 s | 原始 J1 起点 → 末点 rad |
|---|---:|---:|---|
| pregrasp-39 | 25.267995 | 22.065764 | −0.503195341 → 3.037461410；无去程方向限制 |
| contact-41 | 3.420580 | 1.970217 | 3.037461410 → 3.037462055 |
| extraction-42 | 7.105595 | 4.838761 | 3.037462055 → 2.924034883 |
| transit-81，带载 HOME | 28.691128 | 25.487030 | 2.924034883 → −0.503195341 |
| transit-125，到预放置，TIMED_OUT | 92.473395 | 未运行 | 没有返回末点 |

上述成功请求均为 BASIC_SAMPLED_PASS，去程每条采样边采用 1e-7 rad 方向容差；contact 的净增加约 6.46e-7 rad 是多个小于单边容差的数值变化，不能描述为严格数学单调。抽离和第一段转运没有事后改写关节值。place、withdrawal、residence 尚未生成，返回方向只有规则/合成检查，未取得完整实际链验证。

## 复现入口

以下使用已有隔离环境，无升级。所有输出目录必须新建，不能覆盖证据。

```bash
cd /root/autodl-tmp/eco65-native-20261015/repo
export PYTHONPATH=src:tools OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
P=/root/autodl-tmp/envs/isaacsim-clean/bin/python
S=outputs/eco65_native/small_20261019_run61/build/snapshot.json
H=outputs/eco65_native/small_20261019_run61/build/home_candidate.json
W="$PWD/outputs/eco65_native/transit_20261019_build08/worker03.sh"

# 固定旧正面请求；新运行身份，旧请求无树不会补造。
"$P" tools/run_eco65_transit_diagnostic.py --snapshot "$S" \
  --source-request outputs/eco65_native/transit_20261019_case01/transit-22756.json \
  --worker-command "$W" --solver-seconds 120 --rpc-seconds 300 --output NEW_DIAGNOSTIC_DIRECTORY

# top08：从归档实际接触端点重新做抽离/分段调查，不能输出完整计划。
"$P" tools/diagnose_eco65_extraction.py --snapshot "$S" \
  --contact-record outputs/eco65_native/transit_20261019_case02/contact-41.json \
  --home-candidate "$H" --worker-command "$W" --stage-seconds 90 --task-seconds 450 \
  --max-lifts 1 --lift-start 2 --transit --waypoints-only --output NEW_EXTRACTION_DIRECTORY

# cold02：从合法 HOME 重新生成；仅优先一个当前几何候选，不输入旧路径。
"$P" tools/run_eco65_moveit2_single_carton.py plan --snapshot "$S" --home-candidate "$H" \
  --worker-command "$W" --evaluation-profile chain_integration_v1 --preferred-grasp-face top \
  --preferred-candidate-id 1c77faae78228a5bc6a2ac8b7ded4cad843a3a332f4a30be7960ce8241f9091a \
  --stage-seconds 90 --rpc-seconds 240 --task-seconds 900 --attempts 1 \
  --search-trace --trace-stage transit --output NEW_COLD_DIRECTORY

# 普通完整规划入口：默认不记录搜索，仍从合法 HOME 冷生成并执行 J1 策略。
"$P" tools/run_eco65_moveit2_single_carton.py plan --snapshot "$S" --home-candidate "$H" \
  --worker-command "$W" --evaluation-profile chain_integration_v1 --preferred-grasp-face top \
  --stage-seconds 90 --rpc-seconds 240 --task-seconds 900 --attempts 1 --output NEW_NORMAL_DIRECTORY

"$P" tools/export_eco65_search_replay.py --snapshot "$S" \
  --diagnostic outputs/eco65_native/transit_20261019_diag02 \
  --task-diagnostic outputs/eco65_native/transit_20261019_top08 \
  --output outputs/eco65_native/NEW_VIEW/comparison.html
```

完整计划入口为 `export_eco65_search_replay.py --snapshot ... --plan ... --output ...`，会先执行完整合同和 replay_inputs。只有成功生成完整计划后，才能按 `eco65_chain_integration_v1.md` 导出 workstation，并运行现有 `scripts/isaacsim_replay_eco65_native.py --preview`；本轮不满足前置条件，未执行这两项。

Windows 实际浏览器检查：

```powershell
.venv/Scripts/python.exe tools/check_eco65_search_replay_browser.py `
  --html outputs/eco65_native/transit_20261019_view05/comparison.html `
  --browser 'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe' `
  --output outputs/eco65_native/NEW_BROWSER_CHECK
```

原生记录测试可在既有 ROS rootfs 中构建 `m710_search_trace_test`，依次运行 `off on cap directed-off directed-on directed-cap budget`，比较输出的 `check_sequence` 和 `path`；实际结果已归档 `transit_20261019_build14/trace_test_summary.json`。来源/许可见 [SEARCH_TRACE_SOURCES.md](../ros2/m710_moveit_backend/src/SEARCH_TRACE_SOURCES.md)。

## 开发中断保留

首次 launcher 缺少项目 pluginlib 安装前缀，`diag01` 在建树前失败，保留为 NOT_CAPTURED。随后使用现有 ROS 环境加项目局部 AMENT_PREFIX_PATH/LD_LIBRARY_PATH，未升级系统库。

`cold02` 总计 744.816666 s，最终为 `NO_BASIC_CHECKED_COMPLETE_PLAN`，尝试当前优先候选，仍有 1487 个几何候选未搜索。最后 `transit-284` 分配 90 s 求解、84 s 输出检查、5 s 返回预留、240 s 请求预算，仍在 255.253254 s 触发 RPC 中断；该次没有返回树，明确标记 NOT_CAPTURED，不能借用别次树。检查发现上游 CONNECT 内循环没有终止检查，补丁在定向模式内部传递同一个 solver termination condition，与记录开关无关。最初同时修改额外目标采样的尝试导致定向回归失败，已撤销，保留原非阻塞采样；相关失败保存在 build13，最终检查在 build14。补丁不抢占单次在执行的 FCL 查询，不能声称整个原生库具备硬实时中断保证。

最终补丁对同一失败请求的新运行 `diag03` 仍在 240 s RPC 看门狗触发后结束，总耗时 248.022706 s，状态 `INTERRUPTED_NOT_CAPTURED` / `INCOMPLETE`，没有返回树。因此真实超时尚未解决；CONNECT 检查只修复已确认的循环终止缺口，不能据此断言该请求超时根因。复测命令：

```bash
"$P" tools/run_eco65_transit_diagnostic.py --snapshot "$S" \
  --source-request outputs/eco65_native/transit_20261019_case03/request.json \
  --worker-command "$W" --solver-seconds 90 --rpc-seconds 240 --output NEW_RPC_RETEST_DIRECTORY
```

共享数据盘达到 100% 时，`top07` 与 `cold01` 的 gzip 尾部和最终报告写入失败。原损坏文件保留；独立 `recovery_summary.json` 只汇总可完整读出的 51 / 229 条记录，标为 INCOMPLETE_STORAGE_INTERRUPTION。日志中的一次 SUCCESS 没有被冒充为保存完整的结果。空间恢复后，以新目录重新做 `top08` 和 `cold02`。

为恢复空间，将历史 `chain_20261019_run01/plan01/run_report.json` 无损压缩为同目录 `run_report.json.gz`，完整解压 SHA-256 为 `ae8d949f43752c46ac3a9b4563d41d5c7ecb545bf36831422fc1f4408360a7a3`，与原 3206645009 字节完全一致；压缩后 279430922 字节。校验记录在该目录 `report_compression_20261019.json`。未删除失败证据或更改历史内容。

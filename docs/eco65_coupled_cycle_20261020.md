# ECO65 联动单箱流程，2026-10-20

本轮基线 `959c55451eb83e7ab01882fb90c605527e71cf04`，工作分支
`feat/v0.6-eco65-moveit2-native-single-carton`。只读参考
`6416c192d1bae5a93e82418b4c8ae3d6c3e6280a` 的实际 `_seam_coupled_state()` 分支。
没有移植 FANUC 的 q、机器人、箱体或机构尺寸。

入口与改造位置：

| 原入口 | 本轮入口与职责 |
|---|---|
| small_carton / native_12carton builder | `build_eco65_coupled_carton.py` 从已验证当前快照重新生成身份 |
| native_scene / native_model | `coupled_scene.py` 统一实体状态；model 绑定精确碰撞策略和原始对象清单 |
| native_task / transfer_policy | 连续任务独立入口；保留历史七段回 HOME 入口兼容 |
| clearance / eco65_process | 精确对象对同时用于 ACM、净空工作区、独立安装 FCL |
| search replay | `export_eco65_coupled_replay.py` 离线 HTML、真实记录、完整 ZIP |

## 阶段一：场景与碰撞策略

保留 ECO65 官方运动学、工具/TCP、十二物理吸盘和 150×100×100 mm 十二箱。
横纵带有效尺寸均为 0.4443197215708027×0.200 m，低位带面 0.180 m。
新 `upper_seam_lower_slider_v3` 配置：横带 X=[−0.18,0.02]、
Y=[−0.300,0.1443197215708027]；纵带 Y=[−0.300,−0.100]。
这次明确的横向布局改动使纵带与原机器人安装平台保持 30 mm 间隔。
硬件范围 [0.18,0.45] m 与冻结工艺候选 0.43/0.33 m 分开表达。

纵带刚性有效长度不变；上端 XY 固定、随 H 升降；下端 Z=0.18、沿固定导轨移动。
最大水平滑移 0.09144578519 m。横带两组两层剪叉杆长 0.205 m，纵带两组三层
杆长 0.170 m。交叉杆层中心间隔 8 mm，6 mm 实体之间留 2 mm。
下销轴高 26 mm，按 ECO65 低位重算短地脚。低端外置导轨距中心 Y=±0.115 m，
滑车、立柱和轴承座跟随 lower X，纵带侧耳绕有效端点的 Y 轴转动。
杆件、带体不缩放，固定脚与轨不移动。

原 belt 凸体先恢复局部坐标，再施加刚体变换。新增横带接缝侧底部固定切让、
框体、剪叉、导轨和支撑均来自同一状态函数；原静态支架与过渡板被替换。
新增结构属于概念工程几何，不是制造图或承载证明。USD 和回放使用同一实体。

策略 `eco65_pair_collision_policy_v2` 逐项绑定实际源资产：422 个桅杆、安装座、
摄像头及转接件 link；与 `base_link, link_1..link_6` 组成 2954 个精确豁免对。
完整 ID、角色和源指纹列在快照 `native_model.init.collision_policy.robot_mast_exemption`，
并随离线包交付。`robot_mount_plate` 不在名单中。工具、全部吸盘、负载与桅杆的
碰撞检查保留；请求 `allowed_pairs` 仍仅保留原具名安装接触，未扩大权限。

实际定向验证：

- 固定杆长、接缝、脚轨不动、轴承连接、全凸体顶点参与面，以及策略范围检查。
- 真实资产快照下四个高度的跨机构与安装平台有向盒检查无相交。
- 正式 `worker.sh`、trace 关闭、原生 MoveIt/Pilz/OMPL 加载成功。
- 原始合法初态在 H=0.18、0.43、0.33 m 的必要原生碰撞检查均 `SUCCESS`。
- 修复并保留之前证据：ROS setup 与 `set -u` 不兼容；切让凸包有未被面使用的
  内部顶点导致 FCL 拒绝。最终新快照为
  `c1049721b47cdedb6fb6386e79ee8d38cea2a1ea0cf3d24a9c556801b0e6b6ab`。

构建与检查命令（远端既有隔离环境，`PYTHONPATH=src:tools`）：

```bash
P=/root/autodl-tmp/envs/isaacsim-clean/bin/python
S=outputs/eco65_native/coupled_20261020_scene03
$P tools/build_eco65_coupled_carton.py \
  --source outputs/eco65_native/small_20261019_run61/build/snapshot.json \
  --home outputs/eco65_native/small_20261019_run61/build/home_candidate.json \
  --output "$S" --native-asset-root /work-eco65-20261015 --usd
WORKER='chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /usr/bin/env M710_MOVEIT_INSTALL_PREFIX=/work-eco65-20261015/install-coupled-20261020 /bin/bash /work-eco65-20261015/ros2/m710_moveit_backend/worker.sh'
$P tools/check_eco65_coupled_scene_native.py --snapshot "$S/snapshot.json" \
  --home "$S/home_candidate.json" --worker-command "$WORKER" \
  --output outputs/eco65_native/coupled_20261020_stage1_checks03
$P tools/export_eco65_coupled_replay.py --snapshot "$S/snapshot.json" \
  --output outputs/eco65_native/coupled_20261020_preview03/preview.html \
  --preview-q -.5031953406584946 -.30537401468411235 -1.3414169996571763 \
  .07599468559211806 1.5707926322178625 4.17131980283628
```

各输出目录须新建，不覆盖先前运行。新模型 collision 资产需同步至 chroot 的相同
仓库相对路径。CMake 以 `-DCMAKE_INSTALL_PREFIX=/work-eco65-20261015/install-coupled-20261020`
安装；启动不依赖临时 worker 脚本。

全部结果为 `planning_only=True`、`physical_execution=NOT_RUN`、
`execution_ready=false`。机构预览和起态检查不代表完整任务已经成功。

## 阶段二：连续任务合同

`run_eco65_coupled_cycle.py` / `coupled_task.py` 使用独立版本
`eco65_coupled_cycle_v1`，保留原七阶段入口。记录明确分成 robot_motion、
external_axis 和 event；每个已采用记录保存前后完整状态与同一 resident MTC
父记录。非机器人事件通过无机器人轨迹的原生 SubTrajectory 回执加入父链。

状态含真实 q、带高、统一机构状态、全部世界对象、目标、吸盘选择、局部附件变换、
完成箱及接收历史。原生合同绑定最初十二箱和固定环境，检查身份、尺寸、静态部件、
真实附件所有权及段间连续性；升降固定 q 并有限采样完整联动世界。

业务方向为 FIRST_APPROACH / OUTBOUND / RETURN，去程 −1、回程 +1。下一箱
pregrasp/contact 属于 RETURN。原始 J1 路径的反向总量（包括段间继承）上限
1e-6 rad；不取模、不改端点。HOME 只作为初态提案，不要求经过 loaded HOME、
residence 或最终回 HOME。

候选优先正面抓，有限集合内保留顶抓。接收区域由动态横带有效边界及完整箱投影
确定；高低接收候选冻结在目标初态。尝试先升后抽及先水平脱垛后升；低位接收需要
脱垛后真实下降。释放不另造零长度运动段。附着转 world 位姿连续，名义下落保持
姿态、q 不变；展示时间取零初速重力时间公式，位置只是规定的运动学采样，无动力学。
退出时箱体仍在世界中；原生确认工具至少分离 10 mm 后才能 IDEAL_RECEIVER_DRAIN。
清空是本轮假设，不表示真实横纵带输送。随后从真实末态规划至下一箱接触并附着。

定向 Python 检查 27 项通过；C++ `eco65_cycle_test` 的 16 组协议/数值检查通过，
`m710_search_trace_test` 生成 exact 路径。新 worker 已在既有 chroot 原生编译、
安装。合成测试不是完整任务证据；完整运行与实际导出结果在阶段三另记。

## 阶段三：原生运行与问题修正

首次原生运行 `coupled_20261020_run01` 完成首箱接触、附着和固定 q 升带；
直接前抓退到横带的释放点被原生 LIN 报告 `NO_IK_SOLUTION`，没有当作完成。
第二次运行 `coupled_20261020_run02` 增加几何生成的脱垛点与水平箱体转向候选。
脱垛 LIN 已生成路径，但箱体刚离开原下层支撑箱边缘时，间隙约 0.102 mm，
小于原策略要求的 2.4 mm。原始请求、返回、失败 q 和搜索 trace 均保留。

用户随后明确补充：“脱垛过程中，允许箱子与邻箱接触和碰撞。”
据此采用 `eco65_pair_collision_policy_v3`，不追改上述旧运行证据。
新许可仅适用于 `eco65_coupled_cycle_v1` 的 `extraction` 阶段：
实际 attached 的当前目标与冻结原始箱身份中仍在 world 的其余纸箱。
对象对按 ID 精确枚举，允许接触及交叠；邻箱在本轮运动学假设下保持其世界位姿。
工具、全部吸盘、机器人、输送机构及其他环境不获得该许可；
`transit`、`place` 和回程恢复正常碰撞检查。该许可同时作用于原生 ACM、
净空工作区和原支撑箱的独立 FCL 检查，未扩展请求 `allowed_pairs`。
新配置、模型、计划与查看器均绑定新策略身份。

原生求解的绝对截止时间现在覆盖 RRT 扩展、CONNECT 循环和碰撞查询边界；
单个 FCL 调用仍不可抢占，调用前后检查截止。20 ms 的定向测试在 trace 开启/
关闭时分别约 21.30/21.31 ms 退出，未产生 exact 路径。状态检查也共享
任务和连接的剩余预算；额外 10 s RPC 窗口仅用于接收回复。
脱垛末态还采用真实 6 mm 微抬，后续释放继续依据实际箱底和带面检查名义间隙。

`run03` 的首箱接近在输出检查用尽固定 35% 子预算时被拒绝，当时请求尚余
55.51 s，未发现碰撞。中止该候选搜索并保留诊断；连续入口改为让必需输出检查
共享请求截止时间内的剩余额度，不增加 900 s 任务、100 s 连接或 8 s 求解上限。
历史入口保留原预算方法。

`run04` 的脱垛路径通过采样碰撞和 J1 检查，但实际终点位置误差为
3.588719 µm，超过保持不变的 1 µm 门限。安装版 MoveIt 为 2.5.10，其
[KDL 默认收敛精度为 1e-5](https://github.com/moveit/moveit2/blob/2.5.10/moveit_kinematics/kdl_kinematics_plugin/src/kdl_kinematics_plugin.cpp#L190-L195)。
因此只在新联动模型中设置 `robot_description_kinematics.<group>.epsilon=1e-9`，
重新生成模型身份和快照；不修改已生成轨迹的端点，不放宽验收。

### 当前提交的实际完成边界

`run05` 在高位带载转运时遇到机械臂与纵带净空不足，随后暴露连接预算耗尽被误报为
整个任务预算耗尽的问题。保留失败记录，修正预算异常归属，并优先尝试低位转运后升带。

`coupled_20261020_run06` 使用本次 ECO65 原生 worker，在
`coupled_20261020_scene05` 中生成首箱的完整接受链：
`pregrasp-119 → contact-121 → attach-122 → extraction-123 → transit-137 →
coupled_lift-138 → release-139 → nominal_receive-140 → withdrawal-141 →
ideal_receiver_drain-142`。
首箱为 `carton_r00_l03_c01`，实际采用顶抓、`part_024_seal`，
低位抽离及带载转运后保持 q 升至 0.43 m；实际名义下落约 0.05 m。
首箱清空后 world 箱体 11、attached 箱体 0、完成箱体 1。
首箱 checkpoint 规划墙钟 291.948673 s，接受机器人轨迹 148.596280 s，
机构展示 4.166667 s；墙钟不包含随后下一箱搜索。

用户随后要求“不要再验证，先看结果”，因此停止下一箱搜索及后续验证。
停止时尚未到达下一箱实际接触态，`complete=false`；末个未完成请求为
`pregrasp-650`（调用索引 688），不能作为成功回程。
本提交包含随后修正的接收阶段独立共享截止时间，以及完成自由转运后升带保持
自由转运碰撞范围；这两项最新修改尚未重新原生运行。最终完整任务验收未运行，
不以首箱 checkpoint 代替“一箱完整流程＋下一箱实际接触态”的完成证据。

已有记录导出至本地忽略目录
`outputs/eco65_native/coupled_20261020_result06/`：
`first_carton.html` / `first_carton.zip` 为首箱轨迹版，
`full_diagnostics.zip` 包含同次运行的完整诊断页面、684 条可展示请求及原始记录。
导出使用 `--skip-cycle-validation`，明确记录 `NOT_RUN_USER_REQUEST`，
未追加浏览器复核；完整诊断 HTML 约 1.2 GB，建议先打开首箱版。
快照指纹为 `98d7aaca771a37afc632bb4b260ed06c98db63a919bc660eaa921545c9302f6e`。
私有资产、快照和生成回放保持忽略，通过实际离线文件交付，不加入 Git。

本次提交保存阶段三的当前进展，不声明完整任务成功。
`planning_only=True`、`physical_execution=NOT_RUN`、`execution_ready=false`。

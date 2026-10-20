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

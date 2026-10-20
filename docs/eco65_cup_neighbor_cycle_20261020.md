# ECO65 吸盘邻箱接触与原生循环试算，2026-10-20

本次在 `feat/v0.6-eco65-moveit2-native-single-carton` 的
`e3c18ff8d0015a0b2219a8e822937303a56d2472` 上增量实现。
**首箱实际附着、带载抽离已跑通，完整循环未跑通。**
没有完成释放、接收、退出、理想清空及下一箱附着；不把成功前缀算作整箱完成。

## 碰撞策略

新增 `eco65_pair_collision_policy_v4` /
`eco65_cup_neighbor_contact_v1`，执行本次用户的吸盘邻箱接触许可：

- 枚举 12 个真实柔性末端带：`part_002_lip`、`part_004_lip`，依次至
  `part_024_lip`，包括未启用杯。保留各杯全部实体。
- 仅针对冻结的 12 个原始箱体中，当前仍存在、未完成、非当前目标的箱体；
  首箱对应 132 个明确对象对。没有使用名字前缀匹配。
- 仍检查当前目标的实际完整密封与板边对齐、杯与其他环境、刚性工具、
  机器人、已释放目标及其他已有规则。
- Python 模型、身份、原生 ACM、净空工作区、过程结果和查看器读取同一策略。
  请求 `allowed_pairs` 未扩大。
- 机械臂与桅杆的原有精确豁免保留；工具与桅杆、相机安装件的检查保留。
  原先的“附着箱体与邻箱脱垛接触”许可仍只在 extraction 生效。

相机绕真实立杆轴旋转 180°、下俯 30°、名义外参未重新标定、工具板边对齐、
原始 J1 的 FIRST_APPROACH/OUTBOUND 顺时针与 RETURN 逆时针规则保持。
当前输入仍为冻结场景，不是真实视觉识别闭环。

## 原生结果

最终试算：`cup_contact_20261020_run05`；场景：
`cup_contact_20261020_scene01`。
快照指纹：
`e91048393065f13dd548d24cf3a43dbfbea1d9bd652d09c6603dfcffb6231aaf`。

全程有限预算为 900 s；最终运行报告耗时 **902.121055 s**，含收尾记录，
状态 `INCOMPLETE / NATIVE_TASK_BUDGET_EXHAUSTED`。
失败回退后的 `partial_plan.json` 没有采用轨迹。
查看器另外展示同一运行保存的真实成功前缀
`first_carton_extracted.json`，不拼接不同初态试算：

`eco65-ae46af94592c4c58b1956a10f01b4a22-initial-17-0`。

| 阶段 | 原生记录 | 结果 | 机器人回放时间 |
|---|---|---|---:|
| 预抓取 | pregrasp-1010 | SUCCESS | 18.487182 s |
| 实际接触 | contact-1011 | SUCCESS | 6.145514 s |
| 实际附着 | attach-1012 | SUCCESS | 事件 |
| 带载抽离 | extraction-1124 | SUCCESS | 29.106192 s |
| 后续带载转运 | 实际拒绝请求及搜索 trace | 未完成 | 无采用路径 |
| 释放至下一箱附着 | — | 未到达 | — |

该前缀目标为 `carton_r00_l03_c01`，front 抓取，启用
`part_002_seal`；其实际 FK 密封及边对齐通过，板边最大角误差
`2.58095683e-8 rad`。其他 11 杯仍保留碰撞几何。
末态统计：world 11 箱、attached 1 箱、completed 0 箱；
下一目标尚未被接受，无第二箱完成计数。

初态、首抓和抽离末态 q，单位 rad：

```text
initial = [1.064413739, 0.222510379, 1.018801820, 1.900297600, 2.635206393, 4.714536911]
grasp   = [0.415696392, 0.440636049, 0.718287262, 1.982672914, 1.986489046, 4.714524780]
extract = [0.050930841,-0.247409336, 1.455468744, 1.933533597, 1.621723494, 4.714522657]
```

首次接近累计顺时针 `0.648717347 rad`，抽离顺时针 `0.364765551 rad`；
两段累计反向量均为 0。未产生回程，不报告回程方向通过。
展示前缀的机器人运动时间 **53.738889 s**，外部机构运动时间 **0 s**，
附着事件的查看停留 **0.35 s** 单列。三个机构高度预览不是已执行升降段。
完整运行耗时不使用前缀保存时刻代替。

## 失败证据和有界恢复

实际原生 PTP 转运被刚性工具与桅杆上部、相机安装件的碰撞或净空拒绝。
例如请求 `transit-62`：`part_041_region_1` /
`eco65_mast_mast_part_019` 的记录表面间隙为 `0.001406207 m`，
所需间隙 `0.0024 m`；同一路径后续样本记录相交。
其他请求包含 `part_040` / `eco65_camera_pitch_riser_left`、
`part_000` / `eco65_camera_camera_part_001`。
随后原生 OMPL 在有限预算内未获得可接受连接。
这些是已尝试路线的失败证据，不是全局不可行证明。

增加的有限恢复包括从实际附着位姿和剩余箱垛生成斜向上移、横向让位、
半高度抽离候选，以及有条件的一次较慢 LIN。保留官方限位、J1、附件、
起终状态和必要路径检查。没有将 LIN 失败改称 transit 来扩大碰撞权限。
不同仿真初态使用不同原生 task ID，避免更换一个任务的根状态；
全局截止时间不重置。每个失败候选恢复自己的已接受状态。

run01/02/03 保留了之前的抓取后失败；run02 暴露的
`NATIVE_TASK_INITIAL_STATE_CHANGED` 已修复。
run04 在 580.14 s 进程墙钟处主动停止，状态为
`INTERRUPTED_FOR_TARGETED_GEOMETRY_RECOVERY`，不是正常完成。
两个独立端点诊断只用于选择几何候选，未借用其轨迹或当作完整任务成功。
所有失败原始记录均在本地交付目录保留；已验证哈希后移走了远端的重复副本以释放空间。

## 检查及交付文件

- 43 项定向 Python 检查通过：碰撞豁免范围、状态、J1、几何路线、
  初态任务隔离，以及已有相机与板边对齐回归。
- 另 1 项导出回归通过：读取时共享重复几何，序列化再恢复后仍逐项等于原始
  请求，保留不同 trial 和目标，避免大型记录包在导出时重复占用数 GB 内存。
- `m710_clearance_test` 通过，包括真实 FCL/ACM/净空工作区的 132 对检查，
  当前目标、刚性部分和未知环境未获得许可。
- `eco65_cycle_test` 的 25 项原生协议检查通过。
- `cycle.html` 是完整离线页面，内嵌三维资源，显示成功前缀和同一次运行的
  原生请求、失败对象对及 7 份实际搜索 trace。不同候选没有串成一条轨迹。
- Edge 已实际打开离线页面，2915 项页面检查通过；Python/JS FK 最大矩阵误差
  `2.22044605e-16`。截图已查看，明确显示流程未完成、world 11 / attached 1 /
  completed 0；记录见 `browser01/checks.json`、`browser01/checked.png`。

本地目录：`outputs/eco65_native/cup_contact_20261020_result/`。
`cycle.zip` 含 HTML、数据、快照、采用的前缀以及 run05 的原始请求/返回/trace；
`evidence.zip` 含构建、部署、当次源码包与端点诊断；
`previous_runs.zip` 单独保留前四轮失败证据。
`cup_contact_20261020_run05/summary.json` 分别列出成功前缀和最终回退状态，
原始 `partial_plan.json` 未被改写。私有资产和生成文件继续忽略，不入 Git。

## 实际命令

沿用现有环境；下列重跑须换新的输出目录：

```bash
export PYTHONPATH=src:tools OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
P=/root/autodl-tmp/envs/isaacsim-clean/bin/python
S=outputs/eco65_native/cup_contact_20261020_scene01
RUN=outputs/eco65_native/cup_contact_20261020_run05
WORKER='chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /usr/bin/env M710_MOVEIT_INSTALL_PREFIX=/work-eco65-20261015/install-coupled-20261020 /bin/bash /work-eco65-20261015/ros2/m710_moveit_backend/worker.sh'
chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /bin/bash -c \
  'source /opt/ros/humble/setup.bash; cmake --build /work-eco65-20261015/build-coupled-20261020 --target m710_moveit_worker m710_clearance_test eco65_cycle_test -j2 && cmake --install /work-eco65-20261015/build-coupled-20261020 && /work-eco65-20261015/build-coupled-20261020/m710_clearance_test && /work-eco65-20261015/build-coupled-20261020/eco65_cycle_test'
$P tools/build_eco65_coupled_carton.py \
  --source outputs/eco65_native/small_20261019_run61/build/snapshot.json \
  --home outputs/eco65_native/small_20261019_run61/build/home_candidate.json \
  --camera-config configs/workcells/eco65_camera_axis_yaw180_v1.yaml \
  --output "$S" --native-asset-root /work-eco65-20261015 --usd
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs/work-eco65-20261015
mkdir -p "$R/$S"
cp -a "$S/model" "$S/camera_attachments" "$R/$S/"
$P tools/run_eco65_coupled_cycle.py --snapshot "$S/snapshot.json" \
  --home-candidate "$S/home_candidate.json" --worker-command "$WORKER" \
  --output "$RUN" --task-seconds 900 --connection-seconds 100 \
  --solver-seconds 8 --rpc-seconds 100 --ik-seeds 8 --max-grasps 24 --search-trace
```

本地实际导出（PowerShell）：

```powershell
$env:PYTHONPATH='src;tools'
$result='outputs/eco65_native/cup_contact_20261020_result'
$run="$result/cup_contact_20261020_run05"
$snapshot='outputs/eco65_native/cup_contact_20261020_scene01/snapshot.json'
.venv/Scripts/python.exe tools/summarize_eco65_coupled_run.py --run $run --snapshot $snapshot --prefix-plan "$run/first_carton_extracted.json"
.venv/Scripts/python.exe tools/export_eco65_coupled_replay.py --snapshot $snapshot --plan "$run/first_carton_extracted.json" --run $run --output "$result/cycle.html"
.venv/Scripts/python.exe tools/check_eco65_coupled_replay_browser.py --html "$result/cycle.html" --browser 'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe' --output "$result/browser01"
```

保持 `planning_only=True`、`physical_execution=NOT_RUN`、`execution_ready=false`。
未做硬件执行、动力学或整层运行。

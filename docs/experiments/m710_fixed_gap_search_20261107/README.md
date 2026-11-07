# 固定安装、单变量距离搜索（2026-11-07）

本轮只认可新约束下从完整原始 40 箱出发的单一状态父链。历史移动基座 40 箱结果不计入本轮。运行结果见同目录 `result.json` 与 `RESULTS.md`；原始请求、失败日志、轨迹、检查点和压缩证据保留在 GPU 服务器，不作为本地下载包。

## 固定参数与模型

| 参数 | 本轮数值 |
|---|---|
| 安装面绝对高度 | 1.000 m |
| base_link/J1 原点在底盘 A 坐标系 | [0.600, −0.200, 1.000] m |
| 轴线到 +X 前沿 | 1.050 − 0.600 = 0.450 m |
| 原底盘 / 实心台座高度 | 0.600 / 0.400 m |
| 横带上表面范围 / 初态 | [0.550, 1.500] / 0.600 m |
| 唯一布局自由变量 | g：箱堆正面到横带 +X 前沿 |
| 总成世界平移 | [−1.750−g, 0.350, 0] m |
| 机器人基座世界位置 | [−1.150−g, 0.150, 1.000] m |
| 出料平面 | 世界 X=−3.200 m，整个箱体越过后才提交 |

每个 g 独立冻结布局与完整 40 箱。机器人及底盘在该世界中始终固定。FANUC 官方模型、工具 200 mm 加长、72 个独立吸盘、42.5 kg 箱体、箱型、厢体与原有限地板保持原配置。

沿用最后接受的 `low_profile_200_outfeed20_v3` 机构设计，改变的是规定的运行高度区间。横带、纵带、剪叉、纵带倾角、接缝和支撑件均由当前高度重新构造。旧导轨公式依赖厢门世界坐标，本轮将其在参考 g=0.200 m 的几何固定在设备坐标系：导轨 A.x=[−2.000, −0.950] m，支撑站 A.x=−1.180、−1.020 m；随 g 平移，不再隐式改变长度。旧斜切避让几何仍按原设计最大高度 2.100 m 构造，运行上限 1.500 m 不用于重新切削机构。

原有限地板 X=[−3.2, 3.2] m 未扩大。底盘后沿为 −2.800−g；g>0.400 m 时越过门平面。更早出现的支撑脚越界也单独记录。完整支撑足迹超出现有地板的样本标为 `UNKNOWN_EXTERNAL_SUPPORT`，不作为已验证正式解，也不据此证明该距离全局无解。结构强度未验证。

## 工艺与碰撞规则

原始箱底 ≥1.500 m 的三层只允许正吸，采用 `FRONT_DIRECT_HIGH_DROP`：有效接触、完整脱垛、完整投影进入接收区域、同姿态解除附着、名义下落、工具分离和完整出料。没有强制带载下降。最高层在横带 1.500 m 时实际名义落差约 0.600 m，按每次释放姿态计算。

其余五层按 TOP→FRONT→SIDE 公平轮次尝试，每个暴露面保留一个明确上游接触姿态。顶吸先 +Z 脱垛，再带载转移；正吸和侧吸先沿实际法向完整脱垛，必要时 +Z 抬升，再进入横带接收区。最低接收面 0.550 m 对应释放箱底至少 0.600 m。上层不回退到其他面。侧面未开放时不生成侧吸候选。

新入口 `qualified_planes_only` 清空 `robot_mount_pairs()`，同时关闭整臂/底盘和目标箱工具/底盘例外。只保留经几何验证的固定 base_link/台座安装平面，以及台座/底盘实际装配接触。运动连杆、全部工具碰撞体和附着箱体继续检查底盘、台座和机构；普通外部间隙仍为 5 mm。原有 J5/J6 自有工具规则、命名柔性吸盘邻箱规则不扩大到环境。

原生路径、外部升降轴、释放、出料和末态父链全部复核。名义下落和出料对完整机器人、工具及箱体作离散检查；这不是连续扫掠证明。有限出料支撑要求箱体与实际有限纵带表面存在接触重叠、横向覆盖合法，允许明确记录的理想悬伸；未验证重心稳定、冲击、反弹或倾覆。

`physical_execution=NOT_RUN`。接收为 `RECEPTION_ASSUMED`，出料为理想模型；没有 Isaac 或实体执行，没有实测接收计数。

## 可复现入口

服务器独立环境：`/root/m710-gap-search-20261107/rootfs`。以下路径均为 chroot 内部路径。

```bash
cd /work-gap-20261107-delivery
export PYTHONPATH=src:tools:tests
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
/opt/planning-venv/bin/python -m pytest -q \
  tests/test_gap_search.py tests/test_joint_search_layout.py \
  tests/test_joint_search_process.py tests/test_joint_search_ledger.py

# 新运行；实际历史修订及使用的 manifest 见 result.json。
/opt/planning-venv/bin/python tools/plan_m710_gap_search.py \
  --output /gap-search-20261107/new-run \
  --worker-command /gap-worker-v1.sh \
  --worker-binary /gap-search-build-v1/m710_moveit_worker \
  --source-manifest /gap-search-20261107/delivery-source-manifest.json \
  --max-trials 120 --trial-seconds 240 --prefix-seconds 90 \
  --stage-seconds 120 --refine

# 恢复必须使用相同源码、原生二进制及既有检查点；累计成本不会清零。
# 已有 run-v1 应在 /work-gap-20261107-v1 使用 source-manifest-v6.json 恢复。
# delivery 入口仅追加终态保存修复；不要把它的 manifest 直接套到旧检查点。
# --focus-gap 0.2 仅继续该已准入世界，其余世界和成本仍保留。
# --stop-file PATH 在完成当前尝试后保存退出，不截断进行中的原生请求。
/opt/planning-venv/bin/python tools/verify_m710_joint_search.py \
  /gap-search-20261107/run-v1
/opt/planning-venv/bin/python tools/export_m710_joint_search.py \
  /gap-search-20261107/run-v1 --output /gap-search-20261107/best-replay.html
/opt/planning-venv/bin/python tools/report_m710_gap_search.py \
  /gap-search-20261107/run-v1 /gap-search-20261107/result.json
```

初始粗搜 g=0.050…1.200 m、步长 0.050 m，另加 0.005 m 近端边界。出现完整周期后显式登记 ±20、±10、±5 mm 邻域，后续调用准入。数值种子、有限单次预算、候选游标、状态父链及失败成本全部保存；不存在整项任务的业务截止时间。单次调用有明确资源上限，未访问候选及待细化距离不被标为不可达。

种子明确作用于 MoveIt/OMPL 请求；没有声称 KDL 内部随机性全部受同一种子控制。并行调度和有限墙钟预算也可能改变访问次数。源码、参数和已生成证据可逐项复核，不代表重新运行会逐位生成相同轨迹。

实际运行发现某些接触 IK 分支会在许多相邻落带采样点重复触发 LIN 速度拒绝。新入口将一次正吸区域访问限制在最多 12 s 的数值时间片内，保留原整体任务与前缀绝对截止时间，允许现有合法 IK 池继续回访其他配置。时间片结束标为局部搜索未完成；不缩小区域、不改接触姿态、不重置预算，也不放宽关节速度。中下层的分段抬升保持原有完整检查。

时间片仅限制数值连接搜索。一旦原生路径找到，释放、出料和撤离继续使用进入该分支时已确定的原始阶段绝对截止时间；完成检查不会被 12 s 时间片截断。g=190、195、205、210 mm 的细化在独立目录、独立完整 40 箱状态中并行运行。合并报告只取一个分支的最大完成数，绝不累加不同世界的成功箱数。

主搜索在 59 次记录后于尝试边界保存，独立复核已有 3 箱分支，再用本批剩余 15 次分配显式回访它的两箱父节点，探索其他顺序。`revisit-parent-branch.py` 是实际服务器驱动；`branch-revisit-event.json` 记录驱动哈希、前后节点、原检查点哈希和保留的 4984.799 s 成本。最佳三箱节点、所有状态、游标和失败均未修改；主搜索总分配仍为 74 次记录。新原生请求标为 restart，没有伪称延续旧求解树。

代表性原生阶段和最终回放复核通过后方计入完整周期。测试不运行历史 104/129 箱群、不借用历史解，也不以本轮规划结果替代物理资格。

## 源码身份与验证范围

运行时 v6 manifest 的 SHA-256 为 `fdff0ac49b2b45bde9544687f4416def4284ce15b2ddf47319fd6117470f595e`；主搜索与四个细化世界使用同一份源码身份。原生 worker SHA-256 为 `4bf8a96c3c7f6b84ec498629a67d06541a71eae3e7da64697e426a362fcd3fe2`，全部运行修订均未更换该二进制。

交付版本另修复了没有准入候选时的终态检查点保存，并增加报告工具和一个对应回归；没有修改规划候选、运动生成或碰撞接受条件。交付 manifest SHA-256 为 `73b00613f6e12b0dd0f9c7305d849eb028403e56b8d1d50488b233d5d83ceb8f`。本地 47 项通过，服务器独立交付目录 47 项通过；实际原生搜索使用的是此前 46 项通过的 v6。两份源码保留在不同目录中。

原生构建为 Release，ROS 2 Humble / MoveIt 2.5.10，构建目录 `/gap-search-build-v1`，安装目录 `/gap-search-install-v1`。在已配置 ROS/MTC 依赖的 chroot 内可重新构建到新目录：

```bash
source /opt/ros/humble/setup.bash
cd /work-gap-20261107-delivery
export PYTHONPATH=src:tools:tests
cmake -S /work-gap-20261107-delivery/ros2/m710_moveit_backend \
  -B /gap-search-build-reproduce -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/gap-search-install-reproduce
cmake --build /gap-search-build-reproduce --target m710_moveit_worker m710_gap_search_test -j 2
cmake --install /gap-search-build-reproduce
/opt/planning-venv/bin/python tests/native_gap_search_fixtures.py /gap-fixtures-reproduce.json
/gap-search-build-reproduce/m710_gap_search_test /gap-fixtures-reproduce.json
```

实际 `m710_gap_search_test` 已通过；CPU 与原生碰撞反例结果见 `counterexamples-summary.json`。碰撞反例中 CPU 与原生可能先报告不同碰撞对，只宣称两端均拒绝该姿态，不把不同第一证据冒充同一碰撞对证明。工具保守代理相交不写成已证明原始 CAD 穿透。

源码逐文件清单见 `runtime-source-manifest-v6.json` 与 `delivery-source-manifest.json`。如需从交付代码还原运行时 v6，可在独立检出目录执行 `git apply -R docs/experiments/m710_fixed_gap_search_20261107/runtime-v6-to-delivery.patch`，再按运行时 manifest 核对；该反向补丁已通过检查。它仅还原入口终态保存和对应回归，报告工具不参与规划。

原运行上传保留了 31 个文本文件的 CRLF，而 Git 保存的是 LF。为保持原始证据哈希，不改写历史 manifest。Linux 新检出目录可用以下命令恢复**仅换行符不同且转换后精确命中既有 SHA-256** 的文件；任意语义内容不同、缺失文件或越界路径都会拒绝，且内容错误时不会先改写其他文件。运行时 v6 使用相应的运行时 manifest 替换参数。

```bash
python docs/experiments/m710_fixed_gap_search_20261107/restore_manifest_line_endings.py \
  docs/experiments/m710_fixed_gap_search_20261107/delivery-source-manifest.json \
  --restore-line-endings
```

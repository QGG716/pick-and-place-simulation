# ECO65 横带接收完整循环（2026-10-23）

本轮实际原生运行 `cycle04` 完成 **`carton_r00_l03_c01` 抓放、退出及理想清空 → `carton_r00_l03_c00` 实际接触并附着**。第二箱未放置。终态 world 10 箱、attached 1 箱、completed 1 箱。所有运动来自同一冷启动任务，没有导入 FANUC q、旧成功轨迹或固定 HOME 返回段。

`planning_only=True`；`physical_execution=NOT_RUN`；`execution_ready=false`。落带为保持姿态的名义运动学下落，清空为 `IDEAL_RECEIVER_DRAIN`，均不代表真实输送或硬件执行。

## 基线与提交

- 工作分支：`feat/v0.6-eco65-moveit2-native-single-carton`。
- 开始时本地：`b4fdd592a02c9eb0fd040b35722ec890989e90ce`，工作区干净；保留了相对远端的五个已有提交。
- 开始时远端目标：`becdb86e7652cbd610eacbbc19f3d73bfd0f312a`。
- 实际获取的最新只读参考：`eb30f3e0a4bd0d957b02aec3f66c39fb9a8f28d1`。检查了相对分析基线 `1d9783be281be9f8b7c99cdf394e68157fd17551` 的差异。采用其独立最低带高、负倾角和完整机构状态同步语义；其后来新增的“指定层顶吸”及“箱顶阈值”许可不迁移，本轮始终使用用户要求的冻结箱底 `< 0.100 m`。
- 实现：`4a139048b64f74ad264aa9b6f042c2edea9a17ef`。
- 总截止时间及升降准备回滚：`8cacc1eeedc574ae5f500de2779fe2b1b5864fa6`。
- 定向测试时钟修正：`b04cd3871402c9bb0c80fbf7c4219f9efbc2a8d7`。
- 原生接触后停稳升降的附着父状态合同：`6279c2e687ccd337a026769445e7e2d2ff54b159`。
- 离线跨平台机构比较：`547eaf9ddfd267bb8c6fd8da0b87c8b3f64ef7bf`。仅允许生成浮点数的 `1e-12` 绝对尾差，原始记录指纹、ID 和整数拓扑仍检查；不修改规划 q 或记录的实体。

实际运行快照指纹：`c948eebc2a35cc6acec97f1252ee0fd5c64a0edc6bcc24c7f56b0b2f48638714`。源输入为 `outputs/rear100_rigidcarton_20261022/scene/`；保留该布局、ECO65 官方模型、工具 TCP、12 个物理杯、已有探针 TCP 标定及观测绑定。本轮没有重新采集视觉图像。

## 实现位置与工艺边界

| 入口 | 本轮修改 |
| --- | --- |
| `run_eco65_coupled_cycle.py → CoupledTask` | 新流程；历史七阶段入口保留。先选几何工艺，再选杯组及原生 IK。 |
| `transverse_process.py` | 八角点冻结箱底、严格顶吸阈值、原始端箱角色、横带整箱投影、交汇区、侧移通道、高低位及区域细分搜索。 |
| `eco65_coupled_carton_v2.yaml`、`coupled_scene.py` | 横带最低 100 mm、最高 450 mm，初始实际 180 mm；纵带远端仍固定在 180 mm，允许负倾角。横带剪叉下销中心从 26 改为 22 mm，相应固定支脚降低 4 mm；带长、杆长和固定地脚运行中不变。新支撑版本为 `eco65_100mm_transverse_pin22_fixed_lower180_v3`。 |
| `native_contact.py`、`native_cups.py`、`seal_contours.h`、`eco65_process.h` | 扩展实际左右侧吸枚举、法向和完整密封环检查，继续使用 ECO65 的 12 杯实体及 TCP。 |
| `coupled_task.py`、`coupled_state.py`、`native_task.py`、`eco65_cycle.h`、`worker.cpp` | J1 自由双向策略 `RAW_FINITE_JOINTS_BIDIRECTIONAL_NO_J1_MONOTONICITY_V4`；接近前／预抓取／接触态同步，附着抽离前高度必须就绪；真实附着、释放、落带、退出、清空及下一箱衔接。 |
| `export_eco65_coupled_replay.py`、`eco65_coupled_viewer.html` | 工艺、杯组、冻结箱底、带高、落差、完成数量及每个失败请求自己的世界。完整离线 HTML 内嵌依赖与几何。 |

接收面全部为横带。普通前吸高位优先、低位备选；交汇区前吸低位；区域外前吸侧移后直退；原左右端内侧吸取为低／高位备选；合法低箱顶吸先 +Z 再 −X。保持箱体和工具空间朝向，不锁腕关节，不采用斜上绕行、转箱或 yaw 释放备选。

`H_high=z_b−0.050`、`H_low=z_b−h−0.050` 是候选，越界剔除，不 clamp。普通低位前吸保持原释放箱底，因此候选落差为 `h+0.050`。顶吸判据仅取抓取前实际八角点最低世界 Z 减统一地面，不随搬动改变，也不由 H_low 推断。

杯组按允许工艺内完整密封杯数优先排序。首箱 672 个几何工艺／杯组候选中，196 个为 4 杯、332 个为 2 杯、144 个为 1 杯。其余物理杯可以不命中箱面，仍保留全部几何和现有碰撞语义；启用的杯必须在实际 FK 下完整密封。本轮首箱和下一箱均实际采用 4 杯。没有据此宣称硬件独立气路或吸附承载能力已验证。

保留已有 v6 精确碰撞策略及此前用户明确许可。目标箱与邻箱的脱垛许可只在 extraction 阶段有效；进入 `place` 自由转运恢复正常负载检查。落带箱体在工具退出期间始终在 world 中，安全分离后才理想清空。

## 本次采用的原生链

冻结箱底为 **481.182794 mm**。首箱 `FRONT_DIRECT_HIGH`，前吸，启用 `part_004_seal / part_006_seal / part_010_seal / part_012_seal`。

| 原生记录 | 动作及结果 |
| --- | --- |
| `pregrasp-34`、`contact-35` | 从实际拍摄 q 原生 PTP 接近、LIN 接触；带高 180 mm。 |
| `coupled_lift-36` | 保持接触 q，机构升至 431.182794 mm。 |
| `attach-37` | 实际 FK 密封 PASS；原生追溯接触来源为 `contact-35`。 |
| `extraction-38`、`place-40` | 同一附件变换、同一杯组，−X LIN 脱垛及转运至横带完整投影区域。 |
| `release-41`、`nominal_receive-42` | 同姿态释放，实际落差 **49.999997 mm**；下落期间机器人停止。 |
| `withdrawal-45`、`ideal_receiver_drain-46` | 箱体保留在碰撞世界中完成退出，再理想清空。 |
| `next_target-47`、`coupled_lift-48` | 选择 `L03_C00` 的 `FRONT_JUNCTION_LOW`，保持实际退出 q 将横带降至 331.182794 mm。 |
| `pregrasp-65`、`contact-66`、`attach-67` | 从同一末态原生到达下一箱接触并附着；密封 PASS，停止本轮任务。 |

下一箱启用 `part_014_seal / part_016_seal / part_020_seal / part_022_seal`。其 150 mm 工艺落差只是该模板的后续候选，本轮未释放第二箱。

15 条接受记录：7 条机器人运动、2 条外部轴运动、6 条事件。状态、附件、关节、机构及原生父链核对 PASS。去除旧方向约束有实际路径证据：释放后的退出 J1 从 −1.050195 降至 −1.303954 rad，随后接近下一箱又增加；原始角度未取模或改端点。

## 检查、耗时与失败保留

- 57 项改动相关 Python 检查通过；跨平台浮点比较修复后，相关 30 项检查再次通过，新增用例覆盖整数拓扑／对象身份不放宽以及实际约 10⁻¹⁸ m 的平台尾差。
- 原生构建成功。C++ 合同测试通过，包括三个业务区间的双向 J1 和五个接触后升降父状态边界。四种抓面的原生密封轮廓定向夹具通过；这些夹具不是抓放成功证据。
- 实际机构凸体在 100／180／330／430／450 mm 五点检查通过，无非装配干涉，杆长、带长、固定件及地面关系通过。具名轴承／销轴代理体的装配接触单独记录。
- trace 关闭时，仓库 `worker.sh` 入口加载 Pilz 与 OMPL，实际原生 180→100 mm 升降通过，q 不变。没有临时 worker 脚本依赖。
- 本次 `cycle04` 有 78 条原生调用：READY 1、SUCCESS 38、IK 失败 36、规划失败 3；全部保留。成功机器人段使用 PTP／LIN，未调用 RRT，查看器不生成搜索树。
- 本次任务记录的规划墙钟 **361.390 s**，其中原生调用往返累计 **207.006 s**；接受轨迹机器人时间 **220.739 s**，机构展示时间 **5.853 s**，名义下落时间 **0.101 s**。这些分别统计，不能相加作为规划墙钟；事件检查停留和导出／浏览验证另外记录。`planning_wall_s` 截至成功终点附着及结果构造，最终合同复核、文件写入和导出不在该计时字段内。
- Windows 离线导出再次核对完整链并通过，导出耗时 **142.264 s**；HTML 内含 15 段、73 个可显示原生请求、5 个机构预览。查看器事件停留额外 1.75 s，仅供检查，不是机器人运动。
- 实际 Edge 离线浏览检查 **5089 项 PASS**，完整任务 `TESTED`，world／attached／completed 为 10／1／1；浏览 FK 矩阵最大差为 `2.22e-16`，检查耗时 **49.164 s**。完整结果在 `delivery/browser/checks.json`。
- `cycle01/02/03` 的报告墙钟分别为 955.729／932.535／901.009 s，均未计为完成。保留原生请求和失败。修复了接触与接近共享预算、重复复制大场景、准备超时回滚，以及 `ECO65_CYCLE_ATTACH_WITHOUT_NATIVE_CONTACT` 对合法接触后升降的误拒绝。

首箱只尝试了同一 4 杯接触的三个同步位置，在 CONTACT 同步时完成。剩余有限候选及区域细分未继续搜索；不声称最优或其余分支不可行。侧吸及低层顶吸完成实现和定向几何／合同检查，本轮没有运行它们的完整抓放。没有进行十二箱卸完、吞吐量对照或硬件执行。

## 重现与交付路径

工作目录为本仓库。Windows 使用 `.venv/Scripts/python.exe`，`PYTHONPATH=src;tools`：

```powershell
.venv/Scripts/python.exe tools/build_eco65_transverse_process.py --snapshot outputs/rear100_rigidcarton_20261022/scene/snapshot.json --home-candidate outputs/rear100_rigidcarton_20261022/scene/home_candidate.json --output outputs/transverse_process_20261023/scene04
.venv/Scripts/python.exe tools/check_eco65_coupled_geometry.py --snapshot outputs/transverse_process_20261023/scene04/snapshot.json --output outputs/transverse_process_20261023/mechanism_final_check.json
.venv/Scripts/python.exe -m pytest -q tests/test_eco65_transverse_process.py tests/test_eco65_coupled_scene.py tests/test_eco65_coupled_state.py tests/test_eco65_independent_cups.py tests/test_eco65_coupled_replay_sharing.py
```

原生环境为既有远端隔离环境，没有升级。完整构建／运行脚本存于交付的 `logs/commands.sh`，核心命令：

```bash
C=/root/autodl-tmp/m710-moveit2-20260922/rootfs
chroot "$C" /bin/bash -c 'source /opt/ros/humble/setup.bash; cmake --build /work-eco65-20261015/build-coupled-20261020 --target m710_moveit_worker eco65_cycle_test eco65_seal_contours_test -j2 && cmake --install /work-eco65-20261015/build-coupled-20261020 && /work-eco65-20261015/build-coupled-20261020/eco65_cycle_test'
cd /root/autodl-tmp/eco65-native-20261015/repo
export PYTHONPATH=src:tools OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6:/usr/lib/x86_64-linux-gnu/libgcc_s.so.1
W='chroot /root/autodl-tmp/m710-moveit2-20260922/rootfs /usr/bin/env M710_MOVEIT_INSTALL_PREFIX=/work-eco65-20261015/install-coupled-20261020 /bin/bash /work-eco65-20261015/ros2/m710_moveit_backend/worker.sh'
/root/autodl-tmp/envs/isaacsim-clean/bin/python tools/run_eco65_coupled_cycle.py --snapshot outputs/transverse_process_20261023/scene04/snapshot.json --home-candidate outputs/transverse_process_20261023/scene04/home_candidate.json --worker-command "$W" --output outputs/transverse_process_20261023/cycle04 --initial-mode inherited_state --task-seconds 900 --connection-seconds 80 --solver-seconds 8 --rpc-seconds 100 --search-trace
```

回放导出与浏览检查：

```powershell
.venv/Scripts/python.exe tools/export_eco65_coupled_replay.py --snapshot outputs/transverse_process_20261023/scene04/snapshot.json --plan outputs/transverse_process_20261023/cycle04/plan.json --run outputs/transverse_process_20261023/cycle04 --output outputs/transverse_process_20261023/delivery/replay.html
.venv/Scripts/python.exe tools/check_eco65_coupled_replay_browser.py --html outputs/transverse_process_20261023/delivery/replay.html --browser "C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe" --output outputs/transverse_process_20261023/delivery/browser
```

已有输出不会被覆盖；重现时使用新的输出目录。私有资产及大型生成文件保持忽略，实际交付位置为 `outputs/transverse_process_20261023/`：离线查看器／资源包在 `delivery/`，本次计划、事件、请求／返回在 `cycle04/`，历史失败在 `cycle01/02/03/`，100 mm 原生升降证据在 `native_minimum/`。源文件哈希、快照指纹、逐阶段事实和检查记录也保留在该目录。

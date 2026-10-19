# 十二箱顶层中箱：2026-10-17 实际联调

历史记录：2026-10-19 起用户授权小箱和仿真独立选杯，
当前工作见 [小箱与独立选杯](eco65_independent_small_carton_20261019.md)。
本页全十二杯凸包及 47.388 mm 缺口结论不能套用于新的部分杯候选。

本轮从 `b0e5de10a28b92ae74ff07991edbfd0411071721` 续作，保留当前分支和完整布局。
**单箱抓放未完成：冻结布局中的 `carton_r00_l03_c01` 顶吸接触几何不可达。**
服务器、私有资产、完整 worker 和 HOME 验收已经恢复并实际运行；没有生成完整计划或 Isaac 运动视频。

## 当前证据

最终运行目录为 `outputs/eco65_native/top_middle_20261017_run08/`。
快照身份为 `5ac359d68b2947f44907a810af9fc7bc422ece99359b20124ae0deccaaaa7404`。
源十二箱快照、当前 30° 相机和真实吸具保持不变；临时标定板、探针没有进入卸箱模型。
保留十二个箱体、完整仓库、220 个输送带凸体，以及随 J1 运动的永久附件。

- 宿主机及 ROS rootfs 各实际读取并核对 **837 个文件**；rootfs 进一步读取 **710 个 URDF 资源 URI**，均通过。
- 在原 ROS Humble / MoveIt 2.5.10 环境编译 `m710_moveit_worker`，使用隔离的 `build-top-middle-20261017`，未升级环境。
- 主入口两次均读取本轮 `home_candidate.json`，状态为 `ACCEPTED_SIMULATION_INITIAL_STATE`。
  关节限位、当前完整场景 FCL、安装接口和 FK 检查通过。四组 FK 最大矩阵误差为 `8.88e-16`。
  这只接受仿真初始配置，没有验证真机回零路径。
- 独立 MuJoCo 初始状态检查通过；四组实际 USD 读回最大矩阵误差 `2.78e-17`，TCP 误差为零，十二箱身份保持。
- 两次原生冷启动各搜索 12 个几何合格抓姿、调用 384 次原生 IK，全部返回 `NATIVE_IK_FAILED`。
  有限候选集合中的未搜索数为零。另一次新 worker 诊断对接触与预抓取姿态共调用 96 次 IK，也全部失败。
- 全部十二条真实 CAD 轮廓的 Python / 实际编译 C++ 对照一致：名义接触通过，30 mm 间隙和越界位置拒绝。
  此项不代表该姿态可达，也不代替 FCL 查询。

## 可达性证明

`tools/inspect_eco65_native_reach.py` 先验证完整快照、原 URDF 和真实密封轮廓。
TCP 位于十二条轮廓的凸包内部，所以全部轮廓落在箱面时，TCP 必须也位于箱面内。
以下计算仍允许 TCP 使用整个箱面，不扣除 3 mm 密封边距，忽略障碍及关节限位，并把所有允许倾角和横向偏移都用于增加可达性。

| 必要条件 | 数值 |
|---|---:|
| J2→J5 三段长度之和，即任意姿态的距离上界 | 617.117 mm |
| 目标接触所需 J2→J5 距离的宽松下界 | 664.505 mm |
| 仍然缺少的距离下界 | **47.388 mm** |
| TCP→J5 沿工具轴的距离 | 222.250 mm |
| 允许的接触法向偏差 | 0.01 rad |
| 允许的接触位置误差 | 0.4 mm |

上界来自官方关节原点位移：`0.26 + norm([0.24, 0, -0.05888]) + 0.11`。
下界同时考虑任意 J1 方位、整个箱面的 TCP 平移以及允许倾角；并计入官方 rpy 截断产生的亚微米横向偏移。
因此这不是把有限 IK 失败解释成全局无解，而是必要几何条件已经不成立。

正面吸取也无法保持全部十二条轮廓：真实轮廓凸包在任意面内角度的最小宽度为 **131.114 mm**，箱体正面高 **100 mm**，尚未扣除封边距离。

若继续完成同一箱体，至少需要用户允许调整机器人与箱堆的相对布局，之后重新绑定快照并验收。
47.388 mm 是机构距离缺口下界，**不是已经验证的设备平移量**。本轮没有移动设备或箱体来制造成功。

## 实际修补

1. 凸网格使用足够的 OBJ 精度，并保留 Qhull 闭合拓扑；避免微小面被默认后处理删除。
2. 银色连接件使用原始网格分解；官方 J1 外壳使用原外壳凸分解。
   负体积内壳先确认完全封闭在外壳内，作为保守填充的内部空腔处理，不把它们当作独立实体分解。
3. 贴合座最初的分组凸包引入额外体积。新细分保留全部 **4731 个源实体单元**，从 174 个分组细化到 251 个；没有移动或缩小实物几何。
4. 原生模型、独立 MuJoCo 和 USD 支持同一个 J1 下的多个碰撞凸体。所有安装分件继续执行原 **0.4 mm** 阈值，未增加碰撞豁免。
5. 固定 J1 接口失败立即记录，不再通过重复 HOME 搜索尝试消除不随关节姿态变化的错误。
   增加逐调用进度文件；RPC 等待覆盖场景导入和返回路径检查开销，超时仍使 worker 失效。

实际同源凸体距离对照：

| 对象 | FCL 有符号距离 | 独立 MuJoCo |
|---|---:|---:|
| 本轮最深的 J1 / 贴合座安装分件 | -0.349002729 mm | -0.349002155 mm |
| 本轮最深的 J1 / 银色连接件分件 | -0.000841252 mm | -0.000841322 mm |

两者均在原安装容差内。原单凸包约 -2.252 mm 的拒绝，以及中间细分失败，均保留在 `run02` 至 `run07`。

## 阶段与耗时

| 项目 | 本轮实际结果 / 耗时 |
|---|---|
| 主入口初始化 | 两次分别 4.554 / 4.439 s |
| HOME 与 FK 原生调用 | 两次分别 2.525 / 2.548 s |
| 有限抓姿搜索 | 两次分别 0.819 / 0.792 s |
| 完整 plan 入口总墙钟时间 | 35.668 s，含宿主核验、证据保存和 worker 关闭 |
| 最终模型细分及重新绑定 | 29.980 s；复用已校验的几何，不重转 STEP |
| 最后一次 worker 编译 | 56 s |
| 私有包准备 | 19.802 s |
| 宿主 / rootfs 资产核对 | 0.846 / 0.890 s |
| 独立初始状态：构建 / 检查 | 10.650 / 0.354 s |
| 诊断 USD 导出 / 含读回总时间 | 22.840 / 23.594 s |
| 七阶段运动 | 完成 0 阶段；没有附着或释放事件 |
| 整轨独立复核、轨迹导出、Isaac 回放 | 未运行，缺少可达的接触姿态和完整计划 |
| 轨迹模拟时长 | 不存在 |

诊断 USD 只验证静态变换，不作为 Isaac 渲染或运动回放交付。
本轮保持 `PLANNING_ONLY_NOT_EXECUTABLE`、`Dynamics: NOT_EVALUATED`、`NO_MOTION_EXECUTED`。
`KINEMATIC_REPLAY` 是后续回放的约定模式；本轮回放运行状态是 `NOT_RUN`。

## 命令与文件

宿主工作副本：`/root/autodl-tmp/eco65-native-20261015/repo`。
Python：`/root/autodl-tmp/envs/isaacsim-clean/bin/python`。
所有命令使用 `PYTHONPATH=src:tools OPENBLAS_NUM_THREADS=1`。

```bash
P=/root/autodl-tmp/envs/isaacsim-clean/bin/python
O=outputs/eco65_native/top_middle_20261017_run08
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs

chroot "$R" /bin/bash -c 'source /opt/ros/humble/setup.bash; cd /work-eco65-20261015; cmake --build build-top-middle-20261017 --target m710_moveit_worker -j2'

# 首次完整构建使用既有 build 入口；最终细分复用了 run07 的校验后几何。
# 逐次实际命令和最终细分脚本保存在交付目录 commands/。
"$P" tools/package_eco65_assets.py \
  --model "$O/build/native_model_prepared.json" --output "$O/transfer02"

"$P" tools/verify_eco65_transfer.py --root . \
  --manifest "$O/transfer02/private_asset_manifest.json" \
  --scope LINUX_HOST --output "$O/host_asset_check.json"

chroot "$R" /bin/bash -c 'cd /work-eco65-20261015; PYTHONPATH=src python3 tools/verify_eco65_transfer.py --root . --manifest outputs/eco65_native/top_middle_20261017_run08/transfer02/private_asset_manifest.json --scope ROS_ROOTFS --output outputs/eco65_native/top_middle_20261017_run08/rootfs_asset_check.json'

"$P" tools/run_eco65_moveit2_single_carton.py plan \
  --snapshot "$O/build/snapshot.json" --home-candidate "$O/build/home_candidate.json" \
  --worker-command "$PWD/$O/worker.sh" \
  --task-seconds 900 --stage-seconds 60 --attempts 2 --output "$O/plan01"

"$P" tools/inspect_eco65_native_reach.py \
  --snapshot "$O/build/snapshot.json" --output "$O/reach_bound.json"
```

这些是已运行命令；重跑须使用新目录。rootfs 验证命令、编译命令和完整输出同样保存在 `commands/` 与对应日志中。

关键文件：

- `runtime_summary.json`：分项状态与耗时。
- `reach_bound.json`：与当前快照绑定的几何必要条件证明。
- `plan01/attempt_00/failure.json`、`attempt_01/failure.json`：HOME 验收、全部原生调用和抓姿失败。
- `plan01/attempt_*/native_progress.jsonl`：实际逐调用进度。
- `native_initial_diagnostic.json`、`fcl_mujoco_pair_compare.json`：真实 FCL、FK 与独立距离对照。
- `independent_initial.json`、`usd_diagnostic/readback.json`：初始状态与静态显示变换核验。
- `host_asset_check.json`、`rootfs_asset_check.json`：目标端实际文件核对。

本地核心定向测试为 **66 passed、2 skipped**，命令为
`python -m pytest tests/test_eco65_native.py tests/test_eco65_native_12scene.py tests/test_eco65_reach_bound.py -q`。
跳过项依赖本地未提供的 C++ 探针 / USD；另一次单独的 USD 层级测试模块也因缺少本地 pxr 而跳过。
服务器没有 pytest，未安装或升级环境；改以真实 CAD 的 C++ 数值对照和四组实际 USD 读回核验相关运行路径。
这些检查不计为整轨通过。私有 CAD、网格、USD 和运行产物均留在忽略目录，不公开推送。

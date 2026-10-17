# 十二箱顶层中箱：单箱原生链路接入

本轮只处理现有四层、三列场景的顶层中箱 `carton_r00_l03_c01`。
源码基线为 `335ba91251e41e44cf623b8cef6b1b89446f441e`，分支为
`feat/v0.6-eco65-moveit2-native-single-carton`。

**2026-10-17 实际续跑：worker 已编译、资产及 HOME 复查通过；冻结布局的顶吸接触几何不可达。**
完整单箱及 Isaac 回放仍未完成。实际调用、几何证明和修补见
[本轮运行记录](eco65_top_middle_runtime_20261017.md)。
下文保留 `b0e5de1` 接入时的任务约定；当时只有代码与定向回归，没有服务器运行结果。
现有四箱历史运行中的前三阶段成功不能计入这次十二箱任务。

## 任务与抓取选择

目标由实际箱体几何再验证：最高顶面、最接近箱堆横向中线，
且无上方被支撑箱。原始十二个箱体的 ID、尺寸、位姿、支撑关系保持绑定。
当前尺寸为 250 × 210 × 100 mm，四层中心高度为 D-Z
230/330/430/530 mm，选中箱顶面为 580 mm。

先做顶吸。已有 CAD 审查记录的全部十二条密封轮廓，在任意面内方向的
最小整体宽度为 131.113578 mm，超过 100 mm 的箱体正面高度。
参见 [CAD 尺寸记录](eco65_full_face_static_review.md)。
顶面是否可用仍由当前源 CAD 的全部轮廓和原生 IK、碰撞结果决定。

吸取位置不局限于箱面中心。`top_grasp_candidates()` 在
0/±90/180 度面内朝向下，以全部 CAD 轮廓和 3 mm 封边距离计算允许的
TCP 平移范围，提出靠近机器人、前缘居中、有效区域中心等有限候选。
边界提案留有额外 0.1 mm 的数值余量，实际验收阈值仍为 3 mm。
到法兰的距离仅用于排序，不是可达性证明。

同一候选必须完成以下七个运动阶段和两个目标事件：

| 顺序 | 运动/事件 | 约束 |
|---|---|---|
| 1 | pregrasp | 原生 PTP，必要时 OMPL |
| 2 | contact | TCP 原生 LIN；全部 12 条密封轮廓合格 |
| 3 | ATTACH_PREDICTED | 绑定同一个目标及法兰到箱体变换 |
| 4 | extraction | 原生 LIN；只允许目标原支撑顶面的限定接触 |
| 5 | transit | 带载原生 PTP/OMPL，恢复普通环境净空 |
| 6 | place | 原生 LIN，箱底完整落在真实接收带面范围 |
| 7 | SUPPORTED_RELEASE_PREDICTED | 释放同一箱体，保留实际规划末点位姿 |
| 8 | withdrawal | 原生 LIN 撤离 |
| 9 | residence | 返回本次已验收的仿真初始姿态 |

某个候选的阶段搜索穷尽时，保留原始请求/响应和失败前缀，
从同一初始关节值、初始世界、空父阶段开始下一个离线规划分支。
worker 异常或通信超时不能作为普通几何失败继续使用同一进程。
`grasp_search` 记录已尝试和未搜索的候选，未搜索不解释为无解。

## 场景、模型和复核

新入口以经过验证的 `eco65_full_face_static_snapshot_v1` 为输入，生成
新的 `eco65_native_snapshot_v1`。不能直接重命名旧快照 schema 或复用
旧四箱的场景初始化身份。

- 保留其余 11 箱、全部仓库对象、台面、侧/后板和当前输送带。
- 读取真实派生输送带 USD 的每个碰撞连通分量，保留其保守凸包。
  接收带面的矩形只用于支撑条件，不能替代这些碰撞实体。
- 桅杆、连接座、当前相机及俯仰支架作为 J1 固定附件进入原生 URDF，
  随 `link_1` 运动，不能冻结成世界障碍物。
- `link_1` 与源绑定的桅杆安装件/安装座仍逐状态计算实际接触距离，
  最多接受 0.4 mm 穿透；没有沿用旧静态 touch 入口的直接跳过。
  若真实凹形安装件的单凸包产生额外重叠，必须依据源 CAD 改善几何分解，
  不能扩大豁免来制造通过。
- 相机配置只贡献永久设备；临时标定板和探针不进入这次卸箱任务。
  相机原件、变换、替换的桅杆部件和派生支架保持来源绑定。
- 全部密封轮廓、目标身份、原支撑和接收支撑集合绑定到 worker 的
  `process_geometry`。普通运动请求不能扩大允许接触的集合。
- `NativeSceneWorld` 用独立 MuJoCo 距离查询复核所有阶段，包含新增
  输送带凸体及移动附件。它不调用 `mj_step`，不提供动力学执行结论。
- HOME 文件只是同源静态姿态提案，必须在新场景中重新检查限位、
  安装关系、FK/TCP 和碰撞；不声称已经验证从旧 HOME 到新 HOME 的运动。

## 外观与回放修复

机器人 FK 写入 `Robot/<link>` 父节点一次，Visual/Collision 子节点保持
单位局部变换。J1 附件继承父节点运动，并逐项检查其局部变换与世界读回。
原来的“父节点已含 FK，子网格又写一次 FK”错误已修复。

导出使用新快照中的布局、真实 CAD 外观和同源规划凸体。合成器会重新
核验外部输送带 USD 哈希，并读取实际带面 mesh 的世界包围盒。
视频目标标签由快照读取，不再固定为旧四箱目标。

只有同一快照、完整计划和 PASS 独立复核结果可进入回放。
默认保留 640 × 360、5 fps、1 倍轨迹时间。
状态仍是 `PLANNING_ONLY_NOT_EXECUTABLE` / `KINEMATIC_REPLAY`，
动力学为 `NOT_EVALUATED`；预测附着/支撑释放不等于实测吸附和物理接收。

## 原隔离环境中的复跑步骤

以下是待实际执行的命令，不能当成本轮运行记录。
先将本分支源码同步到已有宿主工作副本及其 ROS rootfs 副本，检查没有
覆盖其他未提交工作，再在原 rootfs 中重新构建 `m710_moveit_worker`。
继续使用原环境，不升级 ROS、MoveIt、Isaac 或核心依赖。

已知历史路径：

- 宿主仓库：`/root/autodl-tmp/eco65-native-20261015/repo`
- Python：`/root/autodl-tmp/envs/isaacsim-clean/bin/python`
- ROS rootfs：`/root/autodl-tmp/m710-moveit2-20260922/rootfs`
- rootfs 仓库：`/work-eco65-20261015`
- worker：`/root/autodl-tmp/eco65-native-20261015/worker.sh`

根文件系统为独立副本，宿主的新网格不会自动出现在其中。
`ECO65_CAMERA_CONFIGURATION` 必须指向实际找到并核验过的当前 30 度相机
配置，不能猜测 `tilt30_touch_20261017_run02` 下的内部目录。
每次复跑更换新输出目录。

```bash
cd /root/autodl-tmp/eco65-native-20261015/repo
export PYTHONPATH=src:tools OPENBLAS_NUM_THREADS=1
ECO65_PYTHON=/root/autodl-tmp/envs/isaacsim-clean/bin/python
ECO65_RUN=outputs/eco65_native/top_middle_20261017_run01
ECO65_SOURCE=outputs/eco65_native/j1_mast_20261016_connected_run01/build
: "${ECO65_CAMERA_CONFIGURATION:?Set the verified current camera configuration path}"

"$ECO65_PYTHON" tools/run_eco65_moveit2_single_carton.py build \
  --static-snapshot "$ECO65_SOURCE/snapshot.json" \
  --static-home "$ECO65_SOURCE/pose_A.json" \
  --camera-configuration "$ECO65_CAMERA_CONFIGURATION" \
  --native-asset-root /work-eco65-20261015 \
  --output "$ECO65_RUN/build"

"$ECO65_PYTHON" tools/package_eco65_assets.py \
  --model "$ECO65_RUN/build/native_model_prepared.json" \
  --output "$ECO65_RUN/transfer"
```

将新同步包按相对路径放入原 ROS 副本，并使用
`verify_eco65_transfer.py --scope ROS_ROOTFS` 校验实际文件和 URI。
本次 HOME 在宿主由新快照验收；不要通过旧打包器的 `--home-candidate`
选项把它加入同步包，因为旧传输验证器仍按四箱场景构造 HOME 语义。
同步和 worker 构建完成后：

```bash
"$ECO65_PYTHON" tools/run_eco65_moveit2_single_carton.py plan \
  --snapshot "$ECO65_RUN/build/snapshot.json" \
  --home-candidate "$ECO65_RUN/build/home_candidate.json" \
  --worker-command /root/autodl-tmp/eco65-native-20261015/worker.sh \
  --task-seconds 900 --stage-seconds 60 --attempts 2 \
  --output "$ECO65_RUN/plan"

"$ECO65_PYTHON" tools/run_eco65_moveit2_single_carton.py export \
  --snapshot "$ECO65_RUN/build/snapshot.json" \
  --plan "$ECO65_RUN/plan/plan.json" \
  --output "$ECO65_RUN/export"

"$ECO65_PYTHON" scripts/isaacsim_replay_eco65_native.py \
  --snapshot "$ECO65_RUN/build/snapshot.json" \
  --plan "$ECO65_RUN/plan/plan.json" \
  --review "$ECO65_RUN/plan/review.json" \
  --assets outputs/eco65_native/warehouse_assets \
  --workstation "$ECO65_RUN/export/workstation_D.usdc" \
  --output "$ECO65_RUN/replay"
```

计划入口已经执行独立整轨迹复核，成功后才写顶层 `plan.json` 和
`review.json`；失败时保留 `attempt_*/failure.json` 及 `run_report.json`。
回放入口还会重验完整计划、事件连续性和同一复核结果。

## 本地验证的实际范围

本轮恢复并校验了固定版本的 16 个官方 ECO65-B 文件。
使用隔离目录中的 trimesh 5.1.0、MuJoCo 3.8.0、USD 26.8 做定向验证，
没有更改原服务器环境。

测试覆盖新任务身份、密封候选、支撑阶段权限、HOME 的新设备绑定、
真实 MuJoCo 的移动附件和偏心/旋转世界网格、真实 USD 父子层级和十二箱
合成，以及几何/资产变更负控。测试生成的小型场景明确是回归夹具，
不作为当前私有资产或真实规划任务的验收记录。

最终定向集实际结果为 **80 passed、1 skipped，2.57 秒**：

```bash
PYTHONPATH=src:tools PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 OPENBLAS_NUM_THREADS=1 \
python -m pytest -q \
  tests/test_eco65_native.py \
  tests/test_eco65_native_12scene.py \
  tests/test_eco65_native_replay_usd.py \
  tests/test_eco65_native_composition.py \
  tests/test_eco65_bridges.py \
  tests/test_eco65_static_conveyor.py
```

该命令需使用已具备上述依赖及 pytest 的隔离 Python。
跳过项需要 `ECO65_CONTACT_PROBE` 指向实际编译的 C++ 探针，当前环境没有该探针。
主入口 `--help` 和 Python 语法检查通过，`git diff --check` 无错误。
本地运行记录保存在忽略目录 `outputs/eco65_native/top_middle_local_20261017/`，
其中 `runtime_inputs.json` 明确标记 `BLOCKED_RUNTIME_INPUTS`；
未进行 ROS worker 编译、当前私有十二箱完整构建、原生任务运行或 Isaac 回放。

目前仍需恢复原服务器的 SSH 入口和可用凭据，才能访问其私有 STEP、
`tool.json`、十二箱快照与实际 ROS/Isaac 运行环境，并继续完成用户要求的
这一箱抓取、搬运、放置和撤离。

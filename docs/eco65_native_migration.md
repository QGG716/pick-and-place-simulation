# ECO65-B 原生迁移记录（2026-10-13）

分支：feat/v0.6-eco65-moveit2-native-single-carton。
开发底座：3a84888f85fd490b538e5ce643aa135c2e8cabdb。
资产/场景来源：c07fb73b0caaca9965d8c5b3f09ede18dfb703d4。
fetch 和交付前 ls-remote 均确认两个远端 head 与指定提交一致。
原 ECO65 worktree 只有 .git 和 514 个既有删除，未修改；开发使用新 sibling worktree。
没有采用 FANUC 工作树较新的本地 HEAD，没有整分支 merge、批量 cherry-pick 或覆盖 src/configs。
定向迁入清单见 configs/integration/eco65_native_sources.lock.json；忽略规则先于资产恢复。

## 实际交付状态

**代码与定向测试已完成；真实资产和远程环境阻塞完整验收。未生成真实单箱轨迹或回放视频。**

| 项目 | 本轮实际结果 |
|---|---|
| 官方 ECO65-B | B 型专用清单 16/16 文件恢复且 SHA-256 一致，原件未修改 |
| 官方机械链 | 零位及两组非零腕姿态通过独立 XML 链 / Python FK 对照；测试 TCP 为显式合成变换 |
| 真实工具 / TCP | tool.json、CAD 清单、原始 CAD 和派生网格缺失，导入未运行 |
| 设备场景 | 四箱、完整厢体、双输送带、安装座、2×2 m 桌板及 12 个腿/横撑对象同源 |
| 仓库环境 | 原缓存缺失，环境边界未构建，因此未生成可规划快照 |
| C++ | 纯接触几何程序本机编译运行；完整 ROS/MoveIt worker 未编译或运行 |
| 当前合法 HOME / MoveIt / USD FK | 未评估，没有继承历史 HOME 合法性 |
| 完整原生单箱规划 | 未生成；实际任务原生 IK / Pilz / OMPL 调用未发生 |
| 最终独立几何复核 | 未运行，没有 PASS 结果 |
| 仓库外观及同轨迹回放 | 入口已实现；没有新仓库 USD、外观图或视频 |
| 动力学 / 硬件能力 | NOT_EVALUATED，没有硬件接口 |

缺项入口：configs/local/eco65_desktop/tool.json、assets/tools/desktop_suction/derived/manifests/cad.json。
其引用的 raw STEP、visual/collision 网格也不在本机；缺少清单时无法可靠列出逐零件文件名。
没有用 FANUC 皖泰 STEP 替代。
记录的 SSH 入口 connect.westb.seetacloud.com:31245、connect.nmb1.seetacloud.com:45398、
connect.westd.seetacloud.com:50561 均未建立会话，出现 banner / connection refused。
没有升级系统、ROS、Isaac 或 GPU。复用的 sibling .conda-cpu 有 NumPy/SciPy/Pytest，缺 MuJoCo/trimesh/pxr。

## 关键适配

- native_model.py / 机器人 YAML：绑定专用 B 型、joint_1..6、eco65_arm、base_link、link_6、TCP 和资产。
  派生 URDF/SRDF 保留官方链、轴、零位、比例。基座 D=[-.36,.1,.18]，W=[-.04,0,.98]。
  官方关节限制与仿真运动限制分开，不继承 FANUC 硬件能力。
- native_robot.h / worker / clearance：组和法兰来自绑定身份，每次同步 JSONL dispatch 恢复当前上下文。
  保留 FANUC 缺省配置、MTC 父子链、请求 ID、取消/超时隔离、原生 IK 和几何 workspace。
- native_contact.py / seal_contours.h / eco65_process.h：增加 cad_seal_contours_v1，保留 circular_72_v1。
  来源脚本生成拟合圆；本轮从现有 CAD 网格终端平面提取真实外边界，检查闭合、内外环嵌套、
  12 个身份/组件、接触带归属、法向和 TCP 平面。不符合提取前提会报错；真实 CAD 尚未验证此前提。
  普通净空 .002 + 2*.0002 = .0024 m；指定安装/支撑/密封带接触 .0004 m；密封边缘 .003 m。
  进入/退出近接触包络与严格端点分开；touch links 仅十二个终端带，无 FANUC 腕部/邻箱豁免。
- 搜索使用显式凸包 FCL Convex，独立 MuJoCo 从来源网格凸化。路径相同不作为等价证明；
  真实网格及 FCL/MuJoCo 数值边界对照待运行。API 对照了
  [FCL Convex](https://github.com/flexible-collision-library/fcl/blob/0.7.0/include/fcl/geometry/shape/convex.h) 和
  [DistanceRequest](https://github.com/flexible-collision-library/fcl/blob/0.7.0/include/fcl/narrowphase/distance_request.h)。
- native_scene.py / native_io.py：一个 D 场景，唯一 T_W_D=Translation(.32,-.1,.8)。
  仓库提取遍历 instance proxies 和所有 mesh/collider，冻结原件哈希及地面归一化。
  保守纳入全部 AABB，可能误拒绝，不按 HOME 删除障碍。环境缺失只输出设备尺寸图，禁止规划。
- native_task.py / CLI：固定 carton_r00_l01_c00、顶吸、路线 B；冷启动原生 IK、自由 PTP/OMPL、工艺 LIN。
  默认总预算 900 s、单请求规划预算 60 s、最多 2 次冷候选循环。保存原生返回与失败。
  附着/释放是规划预测，变换由本次接触状态计算；保留三箱。选中完整候选独立复核一次，失败保存对象对并有界重试。
- native_replay.py / Isaac 离线入口：同一未简化结点、C2 quintic 插值、事件和快照。
  复核采样最多 20 ms、关节 L1 步长 .0008 rad；不声称连续扫掠数学证明。
  USD 读回 D/W、法兰/TCP 和全部纸箱。停止时间轴，仅设置显示姿态，640×360 / 5 fps。
  保持 PLANNING_ONLY_NOT_EXECUTABLE / KINEMATIC_REPLAY；原正式 validate/compose/task_audit 门禁未开放。
  未新增理想出料运动，出料计数为 0。

没有迁入旧 compact_task.py、pipeline.py 主规划器。来源几何仅供场景与独立复核。
FANUC 配置、入口和历史结果保留。

## 已执行验证

新 worktree 根目录，PowerShell：

```powershell
python tools/eco65_official_archive.py --restore --path-prefix description/rm_models/ECO65/urdf/ECO65-B/
$env:PYTHONPATH='src'
$env:ECO65_CONTACT_PROBE=(Resolve-Path .tmp/seal_contours_test.exe).Path
$ecoPython='..\pick-and-place-simulation-v0.5-feasibility-core\.conda-cpu\python.exe'
& $ecoPython -m pytest -q tests/test_eco65_native.py tests/test_moveit2_tcp.py tests/test_moveit2_native_cold.py tests/test_moveit2_backend.py tests/test_planning_only_timing.py --junitxml=outputs/eco65_native/directed_tests_final.xml
& .tmp/process_geometry_test.exe
& $ecoPython tools/run_eco65_moveit2_single_carton.py build --output outputs/eco65_native/migration_20261013_delivery
```

107 passed / 0 skipped，6.24 s。新增 36 项中，35 项合成/协议/数值，1 项固定官方 URDF 与文件哈希；
另 71 项为 FANUC/TCP/通信/冷启动/时序回归。C++ FANUC 72 杯程序 15 cases PASS。
新 C++ probe 与 Python 对照 60 组合成多边形姿态，gap/edge 误差 <1e-12；合成轮廓仅在测试中。
本机使用既有 MSVC 18 和 sibling Eigen；测试头 nlohmann/json v3.11.3 留在忽略的 .tmp/include。
ROS 构建目标为 eco65_seal_contours_test；完整 worker/FCL 仍未测试。
Python AST 与 git diff --check 通过。没有全量 pytest、104/129 或五箱扫描。

实际输出（均忽略）：

- outputs/eco65_asset_restore/report.json：16 个官方文件恢复、0 失败。
- outputs/eco65_native/directed_tests_final.xml：最终测试清单。
- outputs/eco65_native/migration_20261013_delivery/assets.json：资产与缺项。
- 同目录 resolved_scene.json：组合设备/桌架几何。
- 同目录 layout.png：已生成并查看的尺寸示意图，不是仓库外观图。
- 同目录 build_report.json：BLOCKED。

最终资产核对 0.0176 s，场景准备及尺寸图 1.1663 s。
真实模型初始化、纯规划、独立复核、USD 导出、渲染/回放、轨迹模拟时长均未运行/无数值。
没有记为 0 s，没有套用 FANUC 历史性能。

## 最小续跑步骤（本轮未运行成功）

恢复来源 tool.json 与 assets/tools/desktop_suction/{cad/raw,derived}、完整锁定 Warehouse 6.0 缓存，
并提供有效原授权 SSH 入口。优先复用哈希一致的既有资产，不重新转换 CAD。
CPU_PYTHON / ISAAC_PYTHON 指向原有环境；ECO65_WORKER 指向本分支编译的既有 rootfs worker launcher；
NATIVE_REPO 为 rootfs 内同一工作副本绝对路径；ASSETS 为仓库缓存根目录。变量按实际恢复位置设置。

```bash
export PYTHONPATH=src
# 原 Humble rootfs 内，按既有构建流程：
colcon build --base-paths ros2 --packages-select m710_moveit_backend
# bounds/export 需要原有 pxr/trimesh 环境；输出目录必须全新。
"$ISAAC_PYTHON" tools/run_eco65_moveit2_single_carton.py bounds --assets "$ASSETS" --output outputs/eco65_native/bounds_run
"$CPU_PYTHON" tools/run_eco65_moveit2_single_carton.py build --environment outputs/eco65_native/bounds_run/environment_bounds.json --native-asset-root "$NATIVE_REPO" --output outputs/eco65_native/build_run
"$CPU_PYTHON" tools/run_eco65_moveit2_single_carton.py plan --snapshot outputs/eco65_native/build_run/snapshot.json --worker-command "$ECO65_WORKER" --task-seconds 900 --stage-seconds 60 --attempts 2 --output outputs/eco65_native/plan_run
# plan 已自动复核选中候选；仅需单独复核时：
"$CPU_PYTHON" tools/run_eco65_moveit2_single_carton.py review --snapshot outputs/eco65_native/build_run/snapshot.json --plan outputs/eco65_native/plan_run/plan.json --output outputs/eco65_native/review_run
"$ISAAC_PYTHON" tools/run_eco65_moveit2_single_carton.py export --snapshot outputs/eco65_native/build_run/snapshot.json --plan outputs/eco65_native/plan_run/plan.json --output outputs/eco65_native/usd_run
"$ISAAC_PYTHON" scripts/isaacsim_replay_eco65_native.py --snapshot outputs/eco65_native/build_run/snapshot.json --plan outputs/eco65_native/plan_run/plan.json --review outputs/eco65_native/plan_run/review.json --assets "$ASSETS" --workstation outputs/eco65_native/usd_run/workstation_D.usdc --output outputs/eco65_native/replay_run
```

补齐后先验证完整 worker 编译、真实轮廓提取、FCL/MuJoCo 边界一致性和当前 HOME，再运行固定目标完整原生链。
复核失败不得改小余量、删除对象或强行回放。

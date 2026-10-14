# ECO65-B 实际总成接入（2026-10-14）

本轮从已审查的 `455384129665e34ddf3cb2fd2b2ede0f2189bae4` 继续同一分支，fetch 后没有后续远端提交。
下面的 10 月 13 日记录保留为历史；其中“缺少原件/工具配置”的阻塞已被本轮解除。

## 实际输入、安装与接触

指定输入为 `assets/tools/desktop_suction/cad/raw/KVGL180-130-260909-01.STEP`，2,998,104 字节，
SHA-256 `aa8d820d56fe3853b1b052e0c76dd77e666abd562427f4dff1b07ef1d6f6cf6c`，转换前后保持一致。
STEP 的毫米单位由 OCCT 按毫米读取，网格、实例平移和面数据仅一次转换为米。
真实总成为 43 个拓扑有效实体，其中 12 个重复吸附实例各含吸附体和独立固定环；没有以旧实体数量约束新输入。
总成树、实体身份、可视网格哈希、分段碰撞网格哈希及最终配置形成完整私有追溯链。

识别到真实工具侧法兰，其安装面、中心轴及六孔圆与官方 link_6 端面网格进行对照。
机器人端和工具端分别保留自己的真实实体，无重复导入；旧生成的圆盘、横板、四支腿全部取消。
安装距离由实际端面确定，没有增加额外 standoff。法兰按实际轴向台阶分成三个碰撞区域，保留杆身附近空隙。
实际工具共 57 个碰撞分区，包括 12 个单独终端接触带；机器人加工具共 64 个凸体。

F 为官方 link_6，C 为原始 CAD，T 为实际自由态接触线定义的任务 TCP；使用 `T_F_T = T_F_C × T_C_T`。
TCP 原点为十二条接触线中心的均值，轴为 CAD +X、+Z、-Y；接触数据仍明确位于 C 坐标系。
六孔圆提供六种等价滚转候选，选取最接近零的一个；官方网格的额外定位特征没有在工具侧找到对应特征。
因此仍为安装仿真假设，未声称实机角度、装配紧固件、实测 TCP、质量、气路或压缩/承载能力已验证。
详细变换、孔位、组件角色和候选角度仅写入私有 `assembly_review.json` 与 `tool.json`，不公开精细几何。

真实唇缘由两个经裁剪的圆环曲面拼接构成，最低处并不存在旧脚本假设的平面环带。
现在直接采样原始 CAD 曲面上的轴向切触线，核对闭合、唯一顶点、平面性、法向及组件/接触带身份。
这是圆滑唇缘的自由态几何切触轮廓，不是拟合圆，也不伪称为平面密封面的内外边界。
Python 和 C++ 消费同一份绑定 CAD/网格哈希的轮廓；9 组真实轮廓案例共检查全部 12 个组件，判定一致，数值误差小于 1e-12 m。

## 代码与实际运行结果

转换器增加 `--input`，默认单文件发现调用仍兼容；旧派生产物在重新生成前备份到忽略目录。
现有工具入口识别带法兰总成后调用测量装配器。安装接触对来自私有配置并绑定到原生初始化几何，
native_model、eco65_process、独立 Python 检查及 USD 导出不再强制寻找旧圆盘名称。
接触权限仅覆盖实际法兰接口分区；其余机器人/工具组合仍正常检查。

普通净空保持 2.4 mm、指定接触压入保持 0.4 mm、封边余量保持 3 mm；距离比较统一使用 1e-9 m 数值 epsilon。
8 组实际 MuJoCo 正/负距离边界与生产 C++ 距离判定函数一致；C++ 标量探针不等于 FCL 运行时测试。
实际 MuJoCo 编译几何与待交给 FCL 的凸体输入按双向面法向支撑值对照，64 个对象最大差异 1.1932e-8 m。
**完整 worker/FCL 尚未编译运行，不将上述输入一致性称作 FCL 实际运行通过。**

零位、旧种子和非零腕姿态的独立 XML 链/Python FK、法兰—CAD—TCP 组合和实际 MuJoCo 工具体随动一致。
旧种子发生 link_3/厢体顶板碰撞，已保留失败；第 6 个有界静态候选得到新的本地合法 HOME。
检查未移动设备、删除箱体/顶板或缩小工具。这个 HOME 只在当前本地设备几何中成立，仍需原生及完整仓库检查。
本轮没有 Python RRT、IK 主规划替代、历史轨迹输入或原生轨迹生成。

| 项目 | 实际状态 | 成功运行分项耗时 |
|---|---|---:|
| 指定 STEP 转换 | 43 实体，原件哈希不变 | 139.913 s |
| 测量安装与工具配置 | 12 轮廓、57 分区 | 38.101 s |
| 资产与设备场景准备 | PASS，本地完整四箱设备 | 4.116 s |
| 派生原生模型准备 | URDF/SRDF/初始化内容已生成；非 worker 初始化 | 11.881 s |
| 实际 MuJoCo 初始化 | 已运行 | 1.239 s |
| 静态 FK/HOME 候选检查 | PASS_LOCAL_STATIC | 0.377 s |
| 真实轮廓 Python/C++ 对照 | 9 案例一致 | 0.454 s |
| MuJoCo/C++ 距离判定边界 | 8 案例一致 | 0.319 s |
| 首版四图 / 改善观察视角后的四图 | 均真实渲染并查看 | 99.001 / 138.662 s |
| 119 文件私有同步包 | PREPARED_NOT_SENT | 30.676 s |
| 117 项定向 pytest | 全部通过 | 12.09 s |
| 完整 worker、原生规划、选定轨迹独立复核、仓库回放 | NOT_RUN | 未运行，无耗时 |

以上是各段实测值，不是端到端总耗时；凸体对照首轮未独立计时。
早期依赖下载重试、两次构建性能排查中断和首轮探针缺少 target_face 的调用错误均保留日志，未算作通过。
修正探针请求后完整执行真实轮廓检查，未改变接触阈值来通过。
117 项含 ECO65 合成协议、实际官方链及保留的 FANUC/TCP/通信/时序回归，不能把全部用例称为真实机器人验收。
本机另编译运行纯 C++ 72 杯/过程几何测试 15 cases；它不代替完整 worker。

## 私有交付与复现

所有模型原件、完整配置、详细快照、网格、检查图和同步包均被忽略，没有公开提交。

- `assets/tools/desktop_suction/derived/{visual,collision,manifests/cad.json}`：本次实际派生资产。
- `configs/local/eco65_desktop/tool.json`：实际安装变换、分区、接触和源身份。
- `outputs/eco65_native/assembly_20261014_run02/`：FK/HOME、实际凸体对照、轮廓/距离边界和原生模型准备结果。
- `outputs/eco65_native/assembly_20261014_figures_v2/`：A 总成、B 末端安装/TCP、C 十二轮廓叠加、D 四箱设备图；已逐张查看。
- `outputs/eco65_native/assembly_20261014_delivery/`：详细装配审查、私有同步包、文件哈希清单和实例检查。
- `outputs/eco65_native/assembly_20261014_native_build_gate/`：实际入口结果，资产 PASS、仓库 NOT_EVALUATED，未产生可规划快照。
- `outputs/eco65_native/directed_tests_20261014.xml`：定向测试记录。
- `.tmp/conversion.log`、`.tmp/assembly_build*.log`、`.tmp/inspection_run*.log`：运行与排错日志。

本次在任务工作树的独立 `.venv` 使用 OCP 7.9.3.1.1、trimesh 5.1.0、SciPy 1.18.1、MuJoCo 3.13.0；
复用本地 OCP/VTK 缓存及既有基础 Python 包，下载包校验发布哈希。未升级任何原 ROS/Isaac 环境。

```powershell
$env:PYTHONPATH='src'
$env:PYTHONIOENCODING='utf-8'
$env:OPENBLAS_NUM_THREADS='1'
.venv\Scripts\python.exe tools/convert_desktop_suction.py --input assets/tools/desktop_suction/cad/raw/KVGL180-130-260909-01.STEP
.venv\Scripts\python.exe tools/build_desktop_suction.py
cmd.exe /c .tmp\build_contact.cmd
.venv\Scripts\python.exe tools/inspect_eco65_assembly.py --output outputs/eco65_native/assembly_20261014_run02 --cpp-probe .tmp/seal_contours_test.exe
$env:ECO65_CONTACT_PROBE=(Resolve-Path .tmp/seal_contours_test.exe).Path
.venv\Scripts\python.exe -m pytest -q tests/test_eco65_native.py tests/test_eco65_mounting.py tests/test_moveit2_tcp.py tests/test_moveit2_native_cold.py tests/test_moveit2_backend.py tests/test_planning_only_timing.py
.venv\Scripts\python.exe tools/run_eco65_moveit2_single_carton.py build --output outputs/eco65_native/assembly_20261014_native_build_gate
```

输出目录为真实运行名称；复跑必须使用新的目录，不能覆盖这些证据。
四图 v2 使用同一工具哈希及已检查 HOME 调用 `inspect_eco65_assembly.render_figures` 改善视角；没有重规划。
辅助 C++ 探针可由原有 CMake 的 `eco65_seal_contours_test` 目标构建，本地使用既有 MSVC/Eigen。

## 尚未解除的最小运行条件

原授权 SSH 三个已知入口本轮仍报 banner/connection refused，未建立远端会话。
完整仓库缓存本地不存在；本轮没有仓库父级聚合 AABB/重复障碍/空隙的实际导入检查。
不能把设备静态图当作完整仓库渲染。私有资产已打包待同步，尚未发送，目标端和 rootfs 内哈希核对尚未执行。

恢复原授权环境后，先在宿主机及原 Humble rootfs 的实际工作副本中解包并逐文件核对
`private_asset_manifest.json`，再在本分支编译完整 `m710_moveit_backend`，检查 worker 的真实 init/FK/状态/安装接触边界。
原环境路径仍为既有 CPU venv、Isaac clean 和 20260922 Humble rootfs，不升级、不改系统。
随后沿用下方历史记录中的 bounds/build/plan/review/export/replay 命令，但输入必须是本次新资产；
先审查实际仓库导入表示，再生成新的完整快照。任务仍为目标顶吸、路线 B、七阶段、900/60 s 和最多两次有界尝试。
当前本地 HOME 必须再次通过真实原生和仓库检查；不得直接宣布可执行或沿用 FANUC 成功结果。

保持 `PLANNING_ONLY_NOT_EXECUTABLE` / `KINEMATIC_REPLAY` / Dynamics `NOT_EVALUATED`，不连接真机。

---

# 历史：ECO65-B 原生迁移记录（2026-10-13）

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

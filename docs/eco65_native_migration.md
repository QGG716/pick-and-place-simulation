# ECO65-B 完整 worker 联调与单箱失败记录（2026-10-15）

用户提供新授权入口后，使用原有 SSH 密钥恢复连接，未使用或保存密码，未升级原有环境。
代码分两阶段提交：`fd825033aaa35ea92abbfbea56b2e50f1d7efb03`（实际 worker 接入修补和显式 HOME 提案）、
`36570454284350502c22d92d404ab711041675f0`（支撑面高度修正、重复 IK 分支去重、URI 读取计数）。
此前已推送的 `7382203`/`10b55c9` 路径、孔位与 HOME 衔接成果保留；旧工作树、私有原件及历史失败均未覆盖。

**实际结果：完整仓库四箱、顶吸、纵向输送带路线 B、固定目标 `carton_r00_l01_c00`，
原生 pregrasp → contact → extraction 通过，transit 未通过。尚无完整七阶段候选，整轨迹独立复核和仓库回放未运行。**
所有结果保持 `PLANNING_ONLY_NOT_EXECUTABLE`；后续回放限定 `KINEMATIC_REPLAY`，Dynamics `NOT_EVALUATED`。

## 资产与真实运行环境

Linux 工作副本为 `/root/autodl-tmp/eco65-native-20261015/repo`。
复用 `/root/autodl-tmp/m710-moveit2-20260922/rootfs` 内 ROS Humble、MoveIt 2.5.10；
CPU 几何/仓库读取使用已有 `/root/autodl-tmp/envs/isaacsim-clean/bin/python`（Python 3.12.3、MuJoCo 3.8.0）。
该环境有现成 SciPy/USD 依赖；原旧 CPU 环境缺 SciPy，因此未使用它，也未安装依赖。
容器不允许 bind mount，故 rootfs 中 `/work-eco65-20261015` 是独立实际文件副本，按清单同步并重新核对。

原先 198 文件私有包在两端解包后逐文件验证通过；完整场景模型、新 HOME 和官方清单加入后的运行清单为 **203 文件**。
Linux 宿主机与 rootfs 各实际读文件、校验 SHA256、核对引用覆盖，均 PASS；rootfs 另实际读取 **71 个资源 URI**。
宿主机还执行 HOME 语义身份校验；精简 rootfs 执行候选字节身份校验，原生主入口另外执行完整语义及几何检查。
标准相对 POSIX 资产身份、宿主机实际文件路径和 rootfs URI 分开处理，没有扫描磁盘寻找同名文件。

联调暴露 Windows `git archive` 对官方 SOURCE_MANIFEST 的行尾转换：仓库对象及本地原件是 LF，归档曾导出 CRLF，导致精确哈希失败。
已用该文件的 `-text` 属性保护字节，并将它显式纳入资产包；复验恢复到原固定哈希
`217b391cb47fa0b7270a887511b80554587a9441fcfcca856ab1b042b44e157c`。
清单审计抽到轻量模块，rootfs 无需 SciPy 即可验证资产。未重新转换 STEP，也未修改任何原始网格。

最终部署的源码归档及两端 **273 个源码文件**逐字节一致，完整 worker 重新编译通过且重新启动检查 HOME。
源码/二进制绑定见私有 `worker_20261015/final_source_binding.json`，二进制 SHA256 为
`a386aac0cdc5be3e924c5eeaaadd131605411c5cc36c64cad82566c9c4eed57a`。
失败定位期间的临时 FCL 查询追踪已恢复；最终编译来自上述提交，不包含临时追踪代码。

## 完整仓库、法兰与 HOME

恢复既有仓库缓存，158 个依赖文件通过清单核对。实际源包含 781 个 Mesh 和 1 个 Plane。
首次读取因 guide-purpose 地面碰撞平面未进入 BBoxCache 而失败，原始失败保存在 `bounds_20261015_initial`。
修补为读取 default/render/proxy/guide 目的几何，保留全部 **782 个叶级对象**。
导入审计没有父子聚合 AABB 重复、重复 prim 或指定工作区空隙封堵；D/W 往返最大误差 `4.44e-16 m`。
状态如实为 `ALL_BOUNDS_INCLUDED_CONSERVATIVELY`，没有空环境替身、删除障碍或绕过场景门禁。
完整快照指纹：`0abef6de8a0c872fe0becbc2ff82c6f85074d25115fc1d9f17ec1637e65b7bb4`。

真实六孔最终匹配最大残差 **2.9552844412e-9 m**，RMS **2.5659693872e-9 m**；安装面、轴与法向残差为零。
当前安装/TCP 变换没有变化，继续保留 STEP 自带法兰。六种滚转候选不等于实机唯一定位，硬件安装角度仍 `NOT_VERIFIED`。

原本地 HOME 在完整场景中通过静态原生检查，但到 pregrasp 的路径未通过。
保留此失败，基于本次真实原生 IK 返回值提出最多两个显式关节候选，首个通过独立完整场景状态检查。
候选携带工具、设备、场景身份、来源失败哈希、原生 IK 请求/响应与关节偏置；不读取旧成功轨迹。
`plan --home-candidate` 实际读取该提案，重新检查限位、安装接口、FK/TCP 和原生碰撞/净空后接纳。
另外运行完整四箱仓库 HOME 的原生/MuJoCo 独立对照，均通过，14 个接触判定一致。
这是 `SIMULATION_INITIAL_STATE_NOT_A_VERIFIED_HOMING_MOTION`，不表示从旧 HOME 到新 HOME 的运动已验证。

## 真实 FCL 与原生单箱调用

完整 C++ worker 实际 READY；模型身份、关节顺序、法兰/TCP 初始化通过，四组 FK 最大矩阵误差 `8.88e-16`。
设备级诊断明确标记 `EQUIPMENT_ONLY_DIAGNOSTIC`，不冒充完整场景验收。
FCL 数据来自生产距离查询调用，新增开关仅保存查询结果，不以标量判定函数或纯几何探针替代查询。
三组状态的基座、法兰、12 个唇缘共 **42 项**对照，在正常 `0.0034 m` 查询范围内判定全部一致；
截断远距离明确标记 censored，安装接触最大数值差约 `1.20e-7 m`。
宽范围查询保留一项异常：旧种子状态下一个唇缘距离 FCL 约 `0.2042 m`，MuJoCo 返回 `0.0`。
改变 MuJoCo 原生 CCD 的临时诊断没有证明远距离可靠，因此生产设置未改、未升级、未放松阈值；不能声称宽范围距离全部一致。

每个开发运行最多 2 次尝试、任务预算 900 s、单次求解器预算 60 s；超时请求保留异常且关闭该 worker。
原生调用的完整请求、响应、错误、耗时、首个关键失败均在对应 `attempt_*/failure.json` 的 `native_calls` 中。

| 运行 | 实际结果 | 总耗时 |
|---|---|---:|
| plan01 | 原 HOME，首个 pregrasp PTP 返回 INVALID_MOTION_PLAN；OMPL 请求超时 | 197.259 s |
| plan02 | 有界原生 IK 按关节距离排序，先尝试 PTP；仍停在 pregrasp | 197.396 s |
| plan03 | 显式新 HOME；前三阶段通过，transit PTP 拒绝；一次求解器 TIMED_OUT，另一次请求超时 | 305.767 s |
| plan04 | 支撑面高度修正、近似重复 IK 去重；前三阶段通过，两次均 transit 请求超时 | 219.455 s |

真实失败包括目标箱/输送带、目标箱或工具/右墙及机器人/顶板接触。
发现旧抽离仅抬升 10 mm，使箱底从 0.16 到 0.17 m，低于所跨输送带的 0.18 m 台面。
小范围修正以路径跨越的支撑面为基准保留 10 mm 净空，此场景原生 LIN 抬升 30 mm 至箱底 0.19 m。
这不改变安装 standoff、工具、墙体、邻箱或碰撞阈值；不能保证后续任意 PTP 都不下降，仍由真实碰撞检查拒绝。
在 plan04 第一次尝试，pregrasp/contact/extraction 分别返回 7/12/12 个原生点，输出检查均 PASS；
三段原生生成耗时 `0.02965/0.04565/0.04850 s`，输出检查 `0.4650/2.7001/2.9281 s`。
随后首个 transit PTP 返回 66 点但 `INVALID_MOTION_PLAN`；OMPL 在客户端 90 s 超时，没有伪造返回或接受未检查路径。
短预算复现曾收到真实 OMPL `TIMED_OUT`，但尚未确定长请求超时根因；不能断言全局无解。

## 命令、输出与验收边界

以下相对路径都位于忽略的 `outputs/eco65_native/`。详细私有证据已下载至本地工作树，未公开推送。

| 项目 | 输出 | 实测耗时 |
|---|---|---:|
| 完整 worker 初次编译 | worker_20261015/build.log、build_timing.json | 31 s |
| FCL 证据版本编译 | worker_20261015/build_contact_evidence.log | 29 s |
| 最终源码编译及完整 HOME 重查 | worker_20261015/build_final.log、final_source_binding.json | 34 s |
| 实际设备 worker/FK/状态 | worker_20261015/diagnostic02/report.json | 7.081 s |
| 运行资产两端验证 | worker_20261015/runtime_host_check.json、runtime_rootfs_check.json | 0.588 / 0.306 s |
| 资产/场景/模型初始化 | full_20261015_build/build_report.json | 0.170 / 0.400 / 3.115 s |
| 导入表示审计 | worker_20261015/warehouse_complete_audit.json | 0.175 s |
| 新 HOME 提案 | native_HOME_20261015/proposal_report.json | 1.479 s |
| 完整 HOME 独立对照（初次，本地保留） | worker_20261015/full_HOME_initial_native_MuJoCo.json | 3.799 s |
| FCL/MuJoCo 正常范围对照 | worker_20261015/FCL_MuJoCo_policy_range.json | 0.491 s |
| 最终定向 Python 回归 | directed_tests_20261015_worker_final.xml | 122 passed / 12.52 s |

实际主要命令（在新 Linux 工作副本；`P` 复用原环境）：

```bash
export PYTHONPATH=src OPENBLAS_NUM_THREADS=1
P=/root/autodl-tmp/envs/isaacsim-clean/bin/python
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs
O=outputs/eco65_native
chroot "$R" /bin/bash -c 'source /opt/ros/humble/setup.bash; cd /work-eco65-20261015; cmake -S ros2/m710_moveit_backend -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build --target m710_moveit_worker -j2'
"$P" tools/verify_eco65_transfer.py --root . --manifest "$O/worker_20261015/runtime_asset_manifest.json" --scope LINUX_HOST --output "$O/worker_20261015/runtime_host_check.json"
chroot "$R" /bin/bash -c 'cd /work-eco65-20261015; PYTHONPATH=src python3 tools/verify_eco65_transfer.py --root . --manifest outputs/eco65_native/worker_20261015/runtime_asset_manifest.json --scope ROS_ROOTFS --output outputs/eco65_native/worker_20261015/runtime_rootfs_check.json'
"$P" tools/run_eco65_moveit2_single_carton.py bounds --assets "$O/warehouse_assets" --output "$O/bounds_20261015_run02"
"$P" tools/run_eco65_moveit2_single_carton.py build --environment "$O/bounds_20261015_run02/environment_bounds.json" --native-asset-root /work-eco65-20261015 --output "$O/full_20261015_build"
"$P" tools/check_eco65_native_worker.py --model "$O/bridge_20261015_home/native_model_prepared.json" --home-candidate "$O/bridge_20261015_home/home_candidate.json" --worker-command /root/autodl-tmp/eco65-native-20261015/worker.sh --output "$O/worker_20261015/diagnostic02"
"$P" tools/propose_eco65_home_from_native_ik.py --snapshot "$O/full_20261015_build/snapshot.json" --failure "$O/full_20261015_plan02/attempt_00/failure.json" --output "$O/native_HOME_20261015"
"$P" tools/run_eco65_moveit2_single_carton.py plan --snapshot "$O/full_20261015_build/snapshot.json" --home-candidate "$O/native_HOME_20261015/home_candidate.json" --worker-command /root/autodl-tmp/eco65-native-20261015/worker.sh --task-seconds 900 --stage-seconds 60 --attempts 2 --output "$O/full_20261015_plan04"
```

常驻 worker 启动脚本实际执行 `chroot ... /bin/bash -c 'source /opt/ros/humble/setup.bash; export ROS_HOME=/tmp/ros-eco65-20261015; exec /work-eco65-20261015/build/m710_moveit_worker'`。
全场景 HOME/FCL 对照使用本机 `.tmp/` 中保留并上传到新工作副本 `tools/` 的私有驱动脚本：
`"$P" tools/full_home_check.py`、`"$P" tools/compare_native_contacts.py`、`"$P" tools/compare_policy_range.py`。
最终重新启动 worker 的 HOME 结果为 `worker_20261015/full_HOME_native_MuJoCo.json`，原初次结果也保留在下载的归档内。
上述命令记录保留在本机 `.tmp/remote_*.sh`，运行 JSON 内保存原生请求，足以定位本次失败。
本轮没有缺失 SSH、STEP、tool.json、ROS 环境或完整仓库缓存。
后续最小待解问题是当前完整场景中可通过实际检查的 transit 原生连接，以及长 OMPL 请求的超时原因；
完成七阶段后才可做一次独立整轨迹复核，再以同轨迹和事件进行仓库外观回放。不能将当前部分成功称为单箱验收。

---

# 历史阶段：ECO65-B 衔接修补与本地验证（2026-10-15，连接恢复前）

从已审查的 `ca71dd64a06f7a032f7184768f2b51edf388a825` 续作，定向 fetch 确认当前远端分支无后续提交。
实际工作树为 `pick-and-place-simulation-v0.6-eco65-moveit2-native-single-carton`；未改动旧工作树的既有删除。
代码提交：`73822038d304bd609e0ce704373a1ef6f9870783`。
当前 SSH 配置入口一次连接返回 `banner exchange: Connection to UNKNOWN port -1: Connection refused`。
用户随后明确“先在本地开发”，未继续重试其他入口，也未升级 ROS、MoveIt 或 Isaac。

## 路径、身份与受控迁移

资产身份路径现在为仓库相对 POSIX 路径，涵盖 source、visual、collision、工具 mesh、凸体 source/convex_path 和同步清单键。
普通读取拒绝反斜杠、绝对路径、盘符、目录越界和越界符号链接；只有显式迁移允许将旧 Windows 分隔符转为正斜杠。
`resolve_asset` 负责当前机器挂载路径，`resource_uri` 单独处理 ROS rootfs 内的绝对 POSIX 挂载根。
Windows 生成器不再使用 Windows Path 语义拼接 Linux URI。没有全盘查找同名文件。

首次迁移在写入前发现旧清单 collision 哈希过期：历史装配器覆盖了转换器同名碰撞文件，工具分区保有当前文件的精确哈希。
修复仅接受已有 tool collision 绑定能够证明的相同字节，并在 migration.json 逐项记录旧、新哈希和依据。
后续装配器写入独立 `collision/assembly/`，不再覆盖转换器文件。
成功迁移先备份 cad.json/tool.json 原始字节，校验 STEP、全部引用网格，更新 CAD 清单哈希及工具绑定，再复验资产清单。
STEP 和网格字节均未变，没有重新执行 STEP 转换或装配网格生成。
原历史报告、旧 HOME 碰撞记录、旧同步包保持原样；新 contact/model/TCP/asset 身份在新目录重新派生。

本次同步包为 **198 文件**（包含新派生的 64 个凸体、模型参数和 HOME 候选），状态 `PREPARED_NOT_SENT`。
Windows 原目录及本地解包副本均逐文件读取、校验哈希和引用覆盖，结果 PASS。
**Linux 宿主机、ROS rootfs 实际解包/资源 URI 读取均 NOT_RUN，不能由本地 PASS 推断。**
准备模型中的 `/work-eco65-20261015` 仅为预备 URI 根，没有声称目标端已挂载；恢复后必须按真实挂载位置生成参数。

## 实际安装与 HOME

安装求解改为机器人六孔相位减工具六孔相位，生成最终变换后做六孔一一匹配，并核对安装平面、中心轴与对向法向。
重新读取已绑定 CAD 面/圆柱记录和官方 STL 孔边界，确认当前存储孔位确实与实际输入一致。
当前最大残差 **2.9552844412e-9 m**，均方根残差 **2.5659693872e-9 m**；平面、轴心和法向残差为零。
**当前 T_flange_tool_cad、T_tool_cad_tcp、T_flange_tcp 均原样保留**。
六种等价安装角仍与实机唯一定位分开，硬件安装角度 `NOT_VERIFIED`；没有恢复模拟圆盘、横板或支腿。

主入口新增 `plan --home-candidate`，输入包含关节顺序、关节值、当前工具与设备指纹、来源报告/迁移哈希。
`NativeTask.run()` 读取该候选；在完整快照中重新检查限位、安装接口、原生状态碰撞/净空和 FK/TCP 后才接纳。
失败保留原生响应及候选拒绝记录，继续原有最多 64 个原生 IK 候选的有界搜索；FK 不一致作为模型故障停止。
初始化、FK、状态、IK 和规划调用均保留请求、响应或异常及耗时。初始化失败也保留已发出的请求。
候选明确为 `SIMULATION_INITIAL_STATE_NOT_A_VERIFIED_HOMING_MOTION`，不表示从旧种子移动到新 HOME 的路径已验证。

本次从旧报告提取关节提案，核对迁移前后工具身份和实际几何未变，再运行新的本地四箱设备 MuJoCo 检查：
限位、碰撞/净空、安装接口均通过；独立 XML FK 最大矩阵误差 3.3307e-16，TCP 组合误差 0。
范围为 `EQUIPMENT_ONLY_DIAGNOSTIC`。候选入口读取与拒绝搜索的回归已通过，**实际原生 HOME 复查尚未运行**。

## 实际运行与输出

所有以下目录都在忽略的 `outputs/eco65_native/` 下，未公开提交详细资产。

| 项目 | 输出目录/文件 | 本轮实测耗时 |
|---|---|---:|
| 元数据迁移与资产复验 | bridge_20261015_migration_run02/migration.json、before/ | 7.251 s |
| 实际接口重新测量 | bridge_20261015_delivery/actual_interface_remeasurement.json | 0.291 s |
| 新身份 HOME 本地复查 | bridge_20261015_home/local_home_check.json、home_candidate.json | 2.988 s |
| 原生参数与凸体准备 | bridge_20261015_home/native_model_prepared.json、model/ | 7.907 s |
| 私有同步包 | bridge_20261015_delivery/private_assets.tar.gz、private_asset_manifest.json | 39.098 s |
| 原目录文件/引用校验 | bridge_20261015_delivery/local_verification.json | 1.262 s |
| 本地解包与逐文件验证 | bridge_20261015_delivery/unpacked_local_verification.json | 9.236 s |
| 121 项定向回归 | directed_tests_20261015.xml | 14.51 s |
| 异常记录补丁后的 16 项相关回归 | pytest 终端输出 | 1.16 s |
| 正式 build 门禁 | bridge_20261015_native_gate/build_report.json | 资产 0.483 s；场景 0.989 s |

新增仅四项小型回归，涵盖 Windows→POSIX→Linux 根路径、越界拒绝、非零工具孔相位、HOME 身份及入口拒绝后有界搜索。
合成协议测试不作为实际 FCL/worker 验收。当前 build 实测为 assets PASS、environment NOT_EVALUATED、BLOCKED，没有生成完整规划快照。
完整 worker **未编译、未启动**，真实 FCL 状态/接触查询 **未运行**。
固定目标 `carton_r00_l01_c00` 顶吸/路线 B 七阶段 **尚未规划**，无选中轨迹独立复核，无仓库回放。
本轮没有用纯 C++ 标量探针替代 FCL 结果，也没有关闭正式场景校验。

本轮实际命令（PowerShell，复跑时换用新输出目录；迁移备份不可覆盖）：

```powershell
$env:PYTHONPATH='src'
$env:OPENBLAS_NUM_THREADS='1'
.venv/Scripts/python.exe tools/migrate_eco65_asset_paths.py --output outputs/eco65_native/bridge_20261015_migration_run02
.venv/Scripts/python.exe tools/prepare_eco65_home.py --report outputs/eco65_native/assembly_20261014_run02/report.json --migration outputs/eco65_native/bridge_20261015_migration_run02/migration.json --output outputs/eco65_native/bridge_20261015_home --native-asset-root /work-eco65-20261015
.venv/Scripts/python.exe tools/package_eco65_assets.py --output outputs/eco65_native/bridge_20261015_delivery --model outputs/eco65_native/bridge_20261015_home/native_model_prepared.json --home-candidate outputs/eco65_native/bridge_20261015_home/home_candidate.json
.venv/Scripts/python.exe tools/verify_eco65_transfer.py --root . --manifest outputs/eco65_native/bridge_20261015_delivery/private_asset_manifest.json --scope LOCAL --output outputs/eco65_native/bridge_20261015_delivery/local_verification.json
$env:PYTHONPATH='src;tools'
.venv/Scripts/python.exe .tmp/audit_bridge_delivery.py
$env:ECO65_CONTACT_PROBE=(Resolve-Path .tmp/seal_contours_test.exe).Path
.venv/Scripts/python.exe -m pytest -q tests/test_eco65_bridges.py tests/test_eco65_native.py tests/test_eco65_mounting.py tests/test_moveit2_tcp.py tests/test_moveit2_native_cold.py tests/test_moveit2_backend.py tests/test_planning_only_timing.py --junitxml=outputs/eco65_native/directed_tests_20261015.xml
.venv/Scripts/python.exe tools/run_eco65_moveit2_single_carton.py build --output outputs/eco65_native/bridge_20261015_native_gate
```

`.tmp/audit_bridge_delivery.py` 是保留在本机的真实接口重测及解包审计脚本，公共校验入口为 `verify_eco65_transfer.py`。
初次未成功的迁移目录为 `bridge_20261015_migration`，终端错误 `PRE_MIGRATION_HASH_MISMATCH:.../collision/part_000.obj`；未写入资产。

## 恢复联调所需的最小条件

需要原授权 Linux 的有效入口，以及原有 Humble/MoveIt、CPU/Isaac 环境和完整仓库缓存；不缺 STEP 或 tool.json。
资产包在真实 Linux 工作副本解包后，先执行宿主机校验，再在原 ROS rootfs 内用对应实际仓库根执行同一校验：

```bash
# 在 Linux 宿主机；HOST_REPO / MANIFEST / CPU_PYTHON 为恢复后的实际位置。
"$CPU_PYTHON" tools/verify_eco65_transfer.py --root "$HOST_REPO" --manifest "$MANIFEST" --scope LINUX_HOST --output outputs/eco65_native/host_asset_check.json
# 在原 ROS rootfs 内执行；资源 URI 必须已按 NATIVE_REPO 重新派生并绑定到同步清单。
python3 tools/verify_eco65_transfer.py --root "$NATIVE_REPO" --manifest "$MANIFEST_IN_ROOTFS" --scope ROS_ROOTFS --output outputs/eco65_native/rootfs_asset_check.json
colcon build --base-paths ros2 --packages-select m710_moveit_backend
```

上述远端命令本轮未执行。rootfs 校验除原件/清单/配置/网格外，还逐项打开 URDF file URI 并核对凸体绑定；错误挂载根不会通过。
恢复后优先完整常驻 worker init、关节顺序、FK、状态和少量真实 FCL/MuJoCo 对照，可先进行明确标记的设备级诊断。
完整仓库需实际检查父级聚合 AABB、重复父子对象、空隙及 D/W 变换；本轮没有缓存，未声称这些检查完成。
随后使用下方历史 bounds/build/plan 命令，给 plan 增加
`--home-candidate outputs/eco65_native/bridge_20261015_home/home_candidate.json`；保留 900/60 s、两次尝试、原生 IK/PTP/OMPL/LIN。
完整候选通过一次独立复核后才使用同一轨迹和事件回放。
始终保持 `PLANNING_ONLY_NOT_EXECUTABLE` / `KINEMATIC_REPLAY` / Dynamics `NOT_EVALUATED`。

---

# 历史：ECO65-B 实际总成接入（2026-10-14）

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

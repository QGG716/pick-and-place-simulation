# 顶排五箱 planning-only 诊断与实测（2026-10-03）

GPU 服务器上完成唯一一次新的连续 **5/5** 批次，纯规划合计 **129.041351 秒**。c00 **9.417096 秒**，c04 **40.711788 秒**。结果始终为 **PLANNING_ONLY_NOT_EXECUTABLE**，**qualification_status = NOT_EVALUATED**，不是卸货执行节拍。

| 顺序 | 箱体 | 规划秒数 | 累计秒数 | IK / PTP / LIN / OMPL（含失败） |
|---|---|---:|---:|---|
| 1 | carton_l07_c02 | 8.433218 | 8.433218 | 11 / 5 / 5 / 1 |
| 2 | carton_l07_c01 | 9.845886 | 18.279103 | 28 / 11 / 5 / 1 |
| 3 | carton_l07_c03 | 60.633364 | 78.912467 | 41 / 20 / 9 / 2 |
| 4 | carton_l07_c00 | 9.417096 | 88.329563 | 15 / 11 / 5 / 1 |
| 5 | carton_l07_c04 | 40.711788 | 129.041351 | 39 / 20 / 6 / 2 |

同一服务器、同一隔离环境的固定输入局部对照：

| 箱体 | 基线秒数 | 优化秒数 | 本次观测耗时减少 | place 失败 |
|---|---:|---:|---:|---:|
| carton_l07_c00 | 462.276250 | 10.764707 | 97.67% | 18 → 0 |
| carton_l07_c04 | 202.405340 | 31.425726 | 84.47% | 6 → 0 |

c00 调用从 **263 / 77 / 39 / 19** 降为 **36 / 10 / 5 / 1**；c04 从 **90 / 42 / 18 / 8** 降为 **34 / 18 / 6 / 2**（IK/PTP/LIN/OMPL）。每侧只有一次完成的局部测量；42.944 和 6.441 是本次观测比值，不能泛化为稳定加速倍数。OMPL 实际墙钟存在波动，固定种子不保证跨进程逐位相同路径。**没有用旧机器的 846.982 秒计算加速倍数。**

## 根因与七项诊断回答

1. 历史 c00 的 19 次 OMPL 全在 **transit**，都从同一 extraction 前驱 `:9` 连接不同接收目标/IK 状态。阶段编号为 `14,20,26,33,40,47,54,61,68,75,82,89,95,100,105,121,128,135,142`，后续 place 是编号加一。目标位置、目标 q、候选 ID、前驱和历史/本服务器求解时间逐项列在 [c00-attempts.csv](experiments/m710_top_row_planning_only_20261003/c00-attempts.csv) 与 JSON。同一抓取 candidate_id 不代表同一接收分支。
2. 历史通用失败本身不能证明碰撞或超时。本轮在未优化源码补诊断后，固定 c00 输入重现相同的 263 IK / 77 PTP / 39 LIN / 19 OMPL 计数。**18 次 place 全是 MTC/Pilz 返回成功、非空轨迹、恰好 1 点，被 `getWayPointCount()<2` 拒绝**；不是这 18 次中的求解器失败、空轨迹或时间参数化异常。TCP 残差约 3.23e-11～1.39e-7 m、1.84e-10～3.58e-7 rad。第 19 次因较大数值残差产生 2 点而通过，偶然绕过接口错误。
3. [layout_trajectory.py](../src/unloading_sim/layout_trajectory.py) 约 3613 行的放置构造以 `max(0, receiver_clearance-release_height)` 增加 preplace 高度。现配置 clearance **0.0084 m**，理想接收高度 **0.025/0.035/0.045 m**，故 preplace 与 place 请求目标相同。此结构能在 transit 前发现；实际末态、阶段权限仍需验证。此次修复错误拒绝，没有把候选判为不可达，也未额外预求解整个后缀。
4. c00 的 19 个 place 目标位置不同，是不同接收尝试，不能都按重复请求永久缓存失败。固定 c00 同时存在 **19 组各重复一次的相同 PTP 请求**（相同 Task、前驱、起点、目标、候选、规划器、种子；例如 `10/12`、`16/18`）；c04 存在 **10 组、11 次额外 PTP 请求**。详见两份 `*-equivalent-requests.json`。正确 place 及早完成后省掉后续尝试；没有新增失败缓存，也未宣称消除全部调度重复。
5. c04 基线也重现历史调用数。8 次 OMPL 是 **1 次 pregrasp（:18）+7 次 transit（:26,:33,:40,:47,:54,:61,:77，前驱 extraction :20）**。六次 place `:27,:34,:41,:48,:55,:62` 均为成功单点被拒，`:78` 两点通过。详见 [c04-attempts.csv](experiments/m710_top_row_planning_only_20261003/c04-attempts.csv)。优化侧第一次 place 即通过并完成撤离。
6. 公平重试在外层，内层完整连接仍可长期占用：c00 只评估 108 个抓取候选中的第一个，107 个未尝试；c04 为 144 中第一个，143 个未尝试。修复接口错误后第一候选很快完成，本次未扩大搜索或重写调度。如果真实下游不可达，机会分配问题仍需后续处理。
7. 主要代价是 **昂贵 transit 后廉价 place 被错误拒绝，引发更多求解**。历史 c00 transit 累计约 517.691 秒，place 约 0.066 秒。本服务器 c00 基线运动内核 442.935 秒，优化 8.209 秒；它们是总计的嵌套分项。真实失败也保留：c00 的 `INVALID_MOTION_PLAN (-2)` 分别为 pregrasp 6、transit 86 次；c04 为 pregrasp 14、transit 35 次，不进一步一概归类为碰撞或超时。`INVALID_GOAL_CLEARANCE` 另记 c00 9、c04 11 次。

## 修复、诊断与测试

[worker.cpp](../ros2/m710_moveit_backend/src/worker.cpp) 保留 MTC 返回 bool/message、可恢复的原始 MoveIt 错误名/码、轨迹存在性、点数和具体处理分支。[moveit2_backend.py](../src/unloading_sim/moveit2_backend.py) 每个运动请求记录一行 JSONL，包含阶段、候选、前驱、种子、耗时及回退前驱，不重复大型静态场景。MTC Humble 的 PipelinePlanner 返回层只有 bool/message；失败消息为原始 MoveIt 错误名时恢复数字并标明来源，取不到时保留 null，不捏造错误码。IK 数量与时间另存于 timing。

仅 planning-only 初始化模式、Pilz LIN、attached place 接受原生静止单点，且须满足：
- 求解器明确成功；唯一点与起始 q **逐值完全相同**，六维速度/加速度有限且全零。
- 用原 solver **1e-6** 位置/姿态目标约束检查真实点，保留原有 clearance 和阶段工艺谓词。
- Task、目标附着关系、place 阶段、真实 transit 前驱均正确，不用“位姿相同”替代阶段身份。
- 保留合法非负原生时间戳，输出绑定阶段的 `NATIVE_STATIONARY_PLACE` 事件；没有复制成两点、改原生点、插入运动边或反转轨迹。
- Python 只接受该显式事件；普通模式仍拒绝单点，错 Task/前驱、移动关节、缺标记、多点伪装均拒绝。

实现/回归见 [native_zero_motion.h](../ros2/m710_moveit_backend/src/native_zero_motion.h)、[native_zero_motion_test.cpp](../ros2/m710_moveit_backend/src/native_zero_motion_test.cpp)、[planning_only.py](../src/unloading_sim/planning_only.py)、[test_planning_only.py](../tests/test_planning_only.py)。正式 c01/c00/c04 各一个静止 place 事件，其他箱正常两点；每箱均有生成的 withdrawal。

开发第一版错误要求时间戳为零，局部试跑发现原生时间非零后停止，保留 `local-comparison/optimized-c00/`，未算作完成对照或正式批次。修正后的 c00 单点原生时间 **1.05e-7 s**，保持原值。最终 **79 项 Python 定向测试 + 10 项原生回归通过**；基线 53 项通过，日志随附。

## 对照条件与源码身份

开始时核对远端 HEAD 确为 **70e8c5369d8bc62fcb8c35940c5df918b7404459**；与 b1635ed 的相关差异是交付/序列化，没有单点修复。唯一源码编辑处为服务器既有 rootfs 内 `/work-planning-opt-20261003-source`，始终在 `feat/v0.5-moveit2-native-single-carton`；没有 reset/stash/force-push。

| 用途 | 源码提交 |
|---|---|
| 诊断基线（仅补日志/固定输入，未改成功门槛） | `26658f2b426d2d827d2429f6c2d7e47d152bb1e8` |
| c00 完成优化对照 | `92bc73fa33c446625e4b78bed2c2c18a94a4a67f` |
| c04 优化对照、正式五箱 | `755c01744d6ecd8ed41e604604aed2ae8594f468` |

最后一版仅增加结果写出计时和资格元数据，worker/规划算法相同。正式 run_id：`planning-only-5e4a709ffaca4ad78a8b308dde26368d`。正式 worker SHA-256：`5126aa3275288827554ab8e14db98d1e3eca8b5191645cb3430892a35e049d09`；基线：`c9bb92f887ae35bf15bc53bef83d0724e5fd3740acfb1156b4f93d7317c40611`。正式冻结清单包含 341 个运行相关源码文件的哈希。基线受控 snapshot 对照 Git blob，清单随附。

每侧使用相同目标、归档起始 q、已移除目标、候选范围与 seed offset，input SHA-256 相同；未读取历史成功路径。每次新 worker、新 Task、空结果缓存、相同模型/BVH预加载。场景、policy、TCP、validator 指纹一致。原始 model_tool 指纹因 URDF 的绝对 mesh URI 随源码根目录改变而不同；仅归一化根前缀后完整参数哈希相同：`34717b053a4575db5fb5ef8cf7cb6460e4e869ea6ea940b0d1d290bd44382637`，66 个资产逐个内容哈希相同。Task/派生 candidate_id 会新生成，这不改变候选几何范围。

顺序为 c00 基线、保留的失败开发试跑、c00 优化、c04 基线、c04 优化、正式五箱。没有并发基准、跨运行轨迹结果复用、挑最快重跑。所有失败尝试、IPC 和新增静止事件检查均计入规划时间。

## 服务器、依赖与实际命令

复用既有 Ubuntu **22.04.5** rootfs、ROS **Humble**、MoveIt/Pilz **2.5.10**、MTC **0.1.3**、OMPL **1.7.0**、GCC **11.4**、CMake **3.22.1**。Python 和常驻 worker 均在同一个 rootfs。只补齐独立 `/opt/planning-opt-20261003-venv` 的 Python **3.10.12** 必需包，未升级 ROS/OS/驱动/CUDA/系统 Python，未修改 Isaac 环境。依赖见 `python-lock.txt`；numpy **2.2.6** 为 Python 3.10 兼容版本，两侧完全相同，不声称等同于旧环境。

CPU **Xeon Platinum 8470Q**，宿主 208 逻辑 CPU，容器配额实际 **22 CPU、110 GiB**。亲和性 **0–21**，Release **-j2**；`OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1`。worker 观测最多 11 系统线程（含 ROS/DDS），求解主要占一个忙 CPU，亲和性不等于 22 路并行。采样最大 worker VmHWM **1,811,388 KiB**（约 1.73 GiB）；采样覆盖 c04 和正式批次，c00 基线峰值未捕获。宿主负载可能含其他租户噪声；容器未见其他重型规划任务，未结束其他项目进程。磁盘和原始低频资源记录随附。

GPU **RTX PRO 6000 Blackwell Server Edition，97887 MiB，595.71.05 驱动**；检查时显存 0 MiB/利用率 0%。没有新增 GPU 规划后端。

官方 FANUC 固定来源 `FANUC-CORPORATION/fanuc_description@fb40c9803a826ba68c7c8e28ba904a25efa7fcd2`，完整工具 CAD、网格、配置和 STEP 复用既有只读资产。初次预加载缺 `res/上海皖泰真空吸盘三分区.STEP`，从服务器已有资产补齐后通过，没有简化几何。无本任务缺失 LFS；唯一 KUKA 子模块不属此 FANUC 链。

实际正式启动：
```bash
R=/root/autodl-tmp/m710-moveit2-20260922/rootfs
taskset -c 0-21 timeout 7500 chroot "$R" /bin/bash /tmp/planning-opt-20261003/run-formal.sh
```
脚本调用普通 `tools/plan_m710_top_row_only.py`，没有 `--diagnostic-input`。精确参数、wrapper 及局部对照脚本均在 [commands](experiments/m710_top_row_planning_only_20261003/commands/)；seed **71070**，阶段 **300 s**、单箱 **1800 s**、整批 **7200 s**、IPC **360 s**。外层 watchdog **7500 s**。输出目录禁止覆盖。异步任务保留 PID/退出码/日志；连接中断后确认原基线已运行完成才继续。

实际构建在 rootfs：
```bash
source /opt/ros/humble/setup.bash
cmake -S /work-planning-opt-20261003-source/ros2/m710_moveit_backend -B /tmp/planning-opt-20261003/build-optimized -DCMAKE_BUILD_TYPE=Release
cmake --build /tmp/planning-opt-20261003/build-optimized -j2
/tmp/planning-opt-20261003/build-optimized/m710_native_zero_motion_test
cd /work-planning-opt-20261003-source
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 /opt/planning-opt-20261003-venv/bin/python -m pytest -q tests/test_planning_only.py tests/test_moveit2_native_cold.py tests/test_moveit2_backend.py
```

## 计时、连续性与边界

计时从每箱候选生成前至完整结果返回内存，含失败求解、IK、MTC/Pilz/OMPL、内部条件、IPC、状态转换和拼接。**T_plan_5 = 五箱墙钟之和 = 129.041351 s**。初始化 **0.887724 s**、场景准备/箱间更新 **0.428450 s**、结果写出 **0.045663 s** 另计（写出侧文件自身及 stdout 不含在写出时间）。批次墙钟 **129.470860 s**。运动内核 **115.609982 s** 是嵌套分项，不重复相加；内部 clearance 查询 **146,088 次**。

完整 40 箱、`m710id70_unloading_layout_v1`、20 kg 工具、42.5 kg 箱体。首箱配置 q；四个箱间边界逐值等于上一箱真实 withdrawal 末点，场景剩余数 40/39/38/37/36。接触后附着，释放后在放置位置保留箱体参与撤离，撤离完成后才理想移除。未回 home、重置机器人或搜索箱序排列。每箱新目标绑定 Task，共用单一常驻 worker，首个完整结果后立即下一箱。正式 history_inputs_read / legacy_motion_generator_calls / forbidden_entry_attempts 均为 0。

保留关节、碰撞、完整几何、阶段权限、正确 TCP、目标所有权、原生时间参数化。ACM、裕量、网格精度和规划器参数未改。独立重型验收仍标记 `SKIPPED_PLANNING_ONLY`，未运行 Isaac/preflight/执行包/动力学或额外密集复检。静止事件只用于该不可执行实验接口。

## 交付与剩余问题

[summary.json](experiments/m710_top_row_planning_only_20261003/summary.json)、
[正式 CSV](experiments/m710_top_row_planning_only_20261003/formal-once/timing.csv)、
[正式 timing.json](experiments/m710_top_row_planning_only_20261003/formal-once/timing.json)、
[不可执行轨迹](experiments/m710_top_row_planning_only_20261003/formal-once/trajectories.json)、
[对照 JSON](experiments/m710_top_row_planning_only_20261003/comparison.json)、
[结构化请求](experiments/m710_top_row_planning_only_20261003/formal-once/requests.jsonl)、
[原生日志](experiments/m710_top_row_planning_only_20261003/formal-once/worker.log.gz)。

同目录包含局部基线/优化原始记录、失败开发试跑、OMPL 对应表、重复组、环境/依赖/版本指纹和测试日志。旧实验保持原样，不提交环境、构建产物或模型副本。

c03 本批仍耗时 60.633 秒，真实绕障成本未专项优化；候选机会分配和少量相同 PTP 调用仍在。未扩展为通用缓存或参数搜索。每侧一次局部对照、一次正式批次，不能推出统计置信区间或真实卸货节拍。

发布说明：普通 Git 推送因连接/凭据通道不可用，使用已授权 GitHub 接口发布服务器生成的完全相同 Git 文件树。API 提交的作者/时间不同，所以提交 ID 不同；四个代码提交的 tree SHA 均逐一完全相等。测试源码提交保持原始 ID，不重新标称测试了另一版。正式源码的等价发布提交为 **5c5fcf37eb05ef194be5b190dbec60da8a64d3de**，详细对应见 [publication.json](experiments/m710_top_row_planning_only_20261003/publication.json)，原始服务器代码提交另保存在小型 Git bundle。源码只在服务器编辑，Windows 仅传输对象和交付物。

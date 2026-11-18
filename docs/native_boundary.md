# ECO65 现场链路原生边界契约（实时化第 3 步，2026-11-16）

本文件冻结视觉、运动规划、在线调度三个模块与原生代码之间的五个边界（A–E）。后续第 4–15 步按此实现，不得引入第二套通道或并行概念。
范围只限 ECO65 现场单箱及连续作业链路；FANUC / M710 / Isaac / 离线仿真入口不受本契约约束，也不改动。

通用约定：
- 单位一律 SI：米、弧度、秒、千克；关节顺序为 `planning_group` 的变量顺序（`worker.cpp Worker::run` 的 `init` 分支校验 `JOINT_ORDER_MISMATCH`）。
- 时钟：Python `time.monotonic()` 与 C++ `std::chrono::steady_clock` 在 Linux 上同为 CLOCK_MONOTONIC；跨进程绝对时刻只用这一时钟（第 2 步 `Worker::applyAbsoluteDeadline` 校验时钟域）。
- 位姿：4×4 齐次矩阵（行优先列表），`worker.cpp matrix()` 校验正交误差 ≤ 1e-8。
- 不得改变（AGENTS.md 2026-11-16 §2）：2.4 mm 物体对间隙及全部物体对规则；715 个机器人/工具碰撞实体与 12 杯；接触、密封覆盖、杯唇穿入（0.4 mm、0.01 rad）、附着规则；完整不确定包络；未知空间、目标包络点、承接策略及其依据；接触候选生成与优先级；关节限值 0.10 rad/s、0.20 rad/s²；5 ms CANFD；所有监控阈值；压力门（−60 / −30 / −10 kPa，两条新心跳跨度 ≥ 1 s，3 s 超时）；全部停止语义。

## 0. 跨语言通道（唯一选择）

**当前实现：现有 worker 进程管道上的二进制帧，数据块随帧传输。** 不使用 pybind11；共享内存尚未实现，不作为当前能力或计时收益。

理由：
1. 进程隔离保留：MoveIt / FCL / ROS 运行在独立进程中，崩溃或卡死只影响该 worker；UI、执行和压力监视进程不受影响（现有 `ResidentMoveItClient` 的隔离语义）。
2. 并行按进程：候选车道与发送路径检查池本来就是多进程（`SinglePlanner.parallel_candidates`、`sent_check`）；pybind11 会把 MoveIt 拉进 Python 进程，受 GIL 和单一 ROS 上下文限制。
3. 现场与开发环境一致：开发服务器上 worker 在 chroot 中运行，Python 在宿主机；只有管道（及双方可见路径上的共享内存文件）能跨越。
4. 旧 JSONL 协议无需拆除，作为 `PYTHON` 实现的通道保留。

帧格式（协议版本 2，由 worker 在 `init` 回复的 `capabilities.binary_protocol=2` 声明；未声明时只用 JSONL）：

| 字段 | 类型 | 说明 |
|---|---|---|
| magic | 4 字节 `E65B` | 帧起始 |
| version | u16 | 2 |
| kind | u16 | 1 请求、2 回复、3 事件、4 取消 |
| request_id | u64 | 请求编号；取消与回复按它匹配 |
| header_length | u32 | 头部字节数 |
| block_count | u32 | 数据块个数 |
| header | UTF-8 规范 JSON | `sort_keys`、无空白、`allow_nan=False`（与 `controlled/trajectory.py digest()` 相同的规范化） |
| block[i] | dtype u8（1=float64，2=int32，3=uint8）、ndim u8、shape u32×ndim、payload | 小端；大块可改为共享内存描述（名称、偏移、长度、SHA-256），接收端核对哈希 |

规范二进制摘要：`SHA-256(header 规范 JSON ‖ 每块 dtype ‖ shape ‖ payload)`，签发（`Authority.register`）与核对（`Authority.verify` / `claim`）两端用同一函数计算（AGENTS §3g）。

第 7 步实际实现：
- 帧格式与上表一致：`binary_frames.h` 和 `workbench/binary_frames.py`。
  - 数据块规则两边相同：64 个及以上 JSON 浮点组成的数组，或等长浮点行组成的矩阵，按排序键深度优先编号。
  - 接收端还原出相同的 float64 位模式。
- **共享内存未实现**，数据块随帧在管道中传输。
- 取消帧为 kind 4。
- JSONL 仍是缺省，只有 `planning_module=NATIVE` 的客户端在 init 确认后才改用帧。

## 1. 模块开关（PYTHON | NATIVE）与机制开关

模块选择（AGENTS §2：缺省 PYTHON，直到在保存记录上有判定完全一致的等价报告；NATIVE 失败只报告，绝不静默回退）：

| 配置键 | 取值 | 缺省 | 覆盖内容 | 对应步骤 |
|---|---|---|---|---|
| `vision_module` | PYTHON \| NATIVE | PYTHON | 现场场景构建与自滤波（契约 C） | 第 9 步 |
| `planning_module` | PYTHON \| NATIVE | PYTHON | 复合原生请求、worker 内 prepare、二进制数据面（契约 A、B） | 第 7 步 |
| `scheduling_module` | PYTHON \| NATIVE | PYTHON | C++ 执行进程、四段预装（契约 D） | 第 10–11 步 |

机制开关（不改变判定，只改性能；缺省保持现行为）：

| 配置键 | 取值 | 缺省 | 步骤 |
|---|---|---|---|
| `planning_deadline_s` | 秒或不填 | 不填 | 第 2 步（已实现） |
| `resident_workers` | bool | false | 第 4 步 |
| `scene_handles` | bool | false | 第 4 步 |
| `async_evidence` | bool | false | 第 4 步 |
| `path_endpoint_reuse` | bool | false | 第 4 步 |
| `binding_hash_cache` | bool | false | 第 4 步 |
| `collision_check_mode` | EXACT \| TWO_LEVEL | EXACT | 第 5 步 |
| `sent_path_audit` | SAMPLED \| ADAPTIVE | SAMPLED | 第 6 步 |

所有开关只在单箱配置（`single_carton/config.py issues()` 校验）中出现；空载受控入口与只读入口不读取它们。

## A. 规划 worker

现有实现：`ros2/m710_moveit_backend/src/worker.cpp`（`Worker::dispatch`、`Worker::run`、`main`），客户端 `src/unloading_sim/moveit2_backend.py ResidentMoveItClient.request`，调用方 `single_carton/planner.py SinglePlanner.native_call / endpoint_ik / plan_segment / sent_check`。

### A.1 操作

| 操作 | 状态 | 输入 | 输出 | 来源 |
|---|---|---|---|---|
| `init` | 现有 | `parameters`（URDF/SRDF/限值）、`identity`（`model_tool_fingerprint`、`policy_scope=PHYSICAL_SINGLE_CARTON`、`task_tcp_link`、`flange_from_task_tcp`）、`joint_names`、`seed=6518`、`collision_policy`、`tool_links`、`process_geometry`、`expected_collision_shapes` | `status=READY`、`capabilities`（`bounded_single_carton_planning`、`partial_observed_contact_envelope`、`single_carton_output_check_deferral`） | `EmptyPlanner.build`、`SinglePlanner.build`；`Worker::run` init 分支 |
| `ik` | 现有 | `goal_pose`、`q_start`（种子）、`position_tolerance_m=2e-5`、`orientation_tolerance_rad=2e-4` | `status`、`q`、`position_error_m`、`orientation_error_rad` | `SinglePlanner.endpoint_ik`；`Worker::run` ik 分支 |
| `validate`（单状态） | 现有 | 上下文（见 A.2）+ 可选 `probe_states` | `native_valid`、`legacy_intersection_valid`、`failure`、`clearance` | `SinglePlanner.endpoint_precheck.check`、`screen_candidate`；`Worker::run` validate 分支 |
| `validate`（路径） | 现有 | 上下文 + `probe_path`（N×6） | 另加 `path_failure`（`edge`、`fraction`、`q_rad`） | `SinglePlanner.sent_check`；`Clearance::checkPath` |
| `plan` | 现有 | 上下文 + `pipeline_id`（Pilz / OMPL）、`planner_id`（PTP / LIN / RRTConnectkConfigDefault）、`q_goal` 或 `goal_pose`、`velocity_scale`、`acceleration_scale`、`allowed_planning_time_s`、可选 `defer_output_check_to_sent_path` | `status`、`points`（`t,q,v,a`）、`native_output_status` | `SinglePlanner.plan_segment`；`Worker::run` 末段 |
| `sc_task_reset` | 新（第 4 步） | `seed` | 清空世界物体、场景句柄、计数器，重建 OMPL 管线并按固定种子重置 RNG；`status=RESET` | — |
| `sc_scene_upload` | 新（第 4 步） | `world`、`scene_fingerprint` | `scene_handle`、`scene_version`、`scene_fingerprint`（worker 重算核对） | `SinglePlanner.context` 中的 `world`、`digest(world)` |
| `sc_scene_release` | 新（第 4 步） | `scene_handle` | `status=RELEASED` | — |
| `cancel` | 新（第 4 步，带外帧） | `target_request_id` | 被取消请求按截止时间到期返回，回复带 `cancelled_by_request=true` | `ResidentMoveItClient.request` 的 `cancelled` 回调 |
| `sc_ik_multi` | 新（第 7–8 步） | `goal_pose`、种子列表（本任务的 q_start、本任务解及 J6 四分之一转变体、64 个固定伪随机种子） | 按离 `q_reference` 由近到远排序的不同解（差 ≥ 1e-3 rad） | `SinglePlanner.ik_seeds`、`spread_seeds`、`endpoint_ik` |
| `sc_endpoint_screen` | 已实现（第 7 步） | 每个候选由 Python 生成的模板：带载起点 validate、接近/接触/脱离 validate、两个 IK 模板、工具连杆、箱号、世界编号；64 个固定种子、限值 | 每个候选的结论（CLEAR / NOT_CONCLUSIVE / LOADED_START_REJECTED / ENDPOINT_TOOL_OR_PAYLOAD_BLOCKED / TARGET_BLOCKED），以及每个子请求的请求（不含 world）与结果 | `SinglePlanner.screen_candidate`、`endpoint_precheck`、`endpoint_ik`、`ik_seeds`、`payload_world_blocked` |
| `sc_plan_candidate` | 已实现（第 7 步），分两段：`CONTACT`（接近，屏蔽时已知则跳过；接触直线及降速重试），`DETACH_AND_VERIFY`（三种脱离方案、接近、带载返回、发送路径检查、换方案重试） | Python 生成的规划请求与上下文模板；密封、附着、承接、脱离目标在两段之间由 Python 计算 | 子请求记录、worker 的 prepare 结果、结论 | `SinglePlanner.plan_candidate_segments`、`plan_segment`、`verify_candidate`、`sent_check` |
| `sc_prepare` | 已实现（第 7 步） | `points`、`limits`、`time_scale`、`period_s`、`reach_bound_m` | 与 `prepare` 相同的字段或相同的错误串 | `controlled/trajectory.py prepare` |

### A.2 请求上下文字段（`SinglePlanner.context`）

`identity`、`q_start`、`start_velocity`（必须全 0，否则 `UNSUPPORTED_NONZERO_START_VELOCITY`）、`path_constraints`（必须空）、`scene_fingerprint`、`world`（物体列表：`id`、`pose`、`size` 或 `convex_mesh`、`category`）或第 4 步起的 `scene_handle`、`allowed_pairs`（`[base_link, robot_mount_plate]`、接触或带载时目标对杯唇、脱离时目标对支撑）、`attachment`（`id`、`size`、`pose` 法兰坐标、`touch_links`）、`clearance_policy`（`schema=eco65_native_single_carton_clearance_v1`、`stage`、`target_id`、`payload_id`、`edge_resolution_rad=0.0005`）、`single_carton_contact`（几何、目标位姿、面、`active_ids`、`target_lip_contact_ids`、`allow_contact`、`supports`、`geometry_uncertainty`）、`stage`、`planning_only=false`、`seed=6518`、`site_workspace_bounds_m`。第 2 步起可带 `request_budget_s`、`absolute_deadline_monotonic_s`、`deadline_clock_reference_monotonic_s`。

### A.3 错误码

- 回复 `status`：`SUCCESS`、`NATIVE_IK_FAILED`、`NATIVE_PLANNING_FAILED`、`NATIVE_INVALID_MOTION_PLAN`、`SEARCH_EXHAUSTED`、`NATIVE_SOLVER_BUDGET_EXHAUSTED`、`NATIVE_PATH_REJECTED`、`INVALID_START_CLEARANCE`、`INVALID_START_PROCESS`、`INVALID_START_COLLISION`、`INVALID_GOAL_CLEARANCE`、`INVALID_GOAL_PROCESS`、`INVALID_GOAL_COLLISION`、`ERROR`（带 `detail`）。来源 `Worker::run`、`main`。
- 碰撞失败 `failure.reason`：`CLEARANCE_INSUFFICIENT`、`INTERSECTION_OR_CONTACT`、`DISTANCE_UNKNOWN`、`DISTANCE_QUERY_FAILED`、`NATIVE_ABSOLUTE_DEADLINE_EXPIRED`、`JOINT_BOUNDS`（`Clearance::check`）；`SINGLE_CARTON_*` 与 `ROBOT_OUTSIDE_OBSERVED_OR_MEASURED_SITE_WORKSPACE`、`PAYLOAD_UNCERTAINTY_OUTSIDE_SITE_WORKSPACE`（`SingleCartonContact::check`）。
- `ERROR.detail`（选列）：`OUTPUT_CHECK_DEFERRAL_SCOPE_INVALID`、`ABSOLUTE_DEADLINE_SCOPE_INVALID`、`INVALID_ABSOLUTE_DEADLINE`、`DEADLINE_CLOCK_DOMAIN_MISMATCH`、`MODEL_OR_POLICY_MISMATCH`、`SCENE_FINGERPRINT_REUSED_WITH_DIFFERENT_CONTENT`、`SINGLE_CARTON_EXTRA_COLLISION_PERMISSION`；第 4 步起新增 `SCENE_HANDLE_UNKNOWN`、`SCENE_HANDLE_FINGERPRINT_MISMATCH`、`TASK_RESET_SCOPE_INVALID`、`CANCEL_TARGET_NOT_RUNNING`。
- Python 侧归类（`single_carton/planner.py`）：`SegmentRejected`（段被拒，带 `stage`）、`TargetBlocked`（目标级）、`PlanningBudgetExhausted`（第 2 步，预算耗尽，不得记为不可行）、`RuntimeError`（IPC 失败）。`failure_category` 的分类键不变。

### A.4 取消与截止时间

- 每个规划请求一个绝对单调截止时间（第 2 步，`planning_deadline_s`），传给每次原生调用；worker 用它收紧本请求的 `NativeDeadline`（`native_deadline.h`），OMPL 扩展与距离循环都检查它。到期报告预算耗尽，区别于 IPC 失败与物理不可行，并给出已搜索 / 未搜索候选数（AGENTS §3d）。
- 第 4 步起，取消与超时按请求编号生效：Python 发送带外 `cancel` 帧，worker 的读取线程把正在运行请求的 `NativeDeadline` 置为已到期；worker 在宽限时间内回复则保留进程，否则判为卡死并关闭，不再分配请求。
- 单次 FCL 调用不可抢占（`out["absolute_solver_deadline"].single_fcl_call_preemption=NOT_AVAILABLE_CHECKED_BEFORE_AND_AFTER`）。

### A.5 线程与重入

- worker 按请求串行处理（`main` 的 `std::getline` 循环）；同一进程可驻留至多 4 个模型上下文（`UNSUPPORTED_RESIDENT_CONTEXT_CAPACITY`）。第 4 步只新增一个读取线程处理带外取消，请求处理仍串行。
- Python 侧每个 worker 同一时刻只有一个在途请求（`ResidentMoveItClient._lock`；第 2 步的每 worker IPC 线程）。并行靠多进程：主 worker + `native_check_workers` 个检查池 worker，候选车道 `parallel_candidate_lanes` 条，车道 k 固定分配排名 `k mod N` 的候选。
- 常驻（第 4 步，AGENTS §3c）：每个任务开始时 `sc_task_reset`，任何上一任务的世界物体、附着、碰撞许可、随机数状态都不得影响下一任务。

### A.6 Python 侧保留

候选生成 `geometry.contact_candidates`（顺序即优先级）、密封检查 `candidate_contact_check`、原位包络检查 `payload_rest_check`、承接检查 `release_check`、许可签发 `Authority.register`、全部记录与 UI 进度。第 7 步若把候选级流程下沉，Python 仍保留旧流程（`planning_module=PYTHON`），结论必须逐项一致。

第 7 步（`planning_module=NATIVE`）的结论来源：worker 按与 Python 相同的分支执行复合请求，并记录每个子请求和结果；Python 随后用回放客户端运行**原样不变**的 `screen_candidate` 和 `plan_candidate`。
- Python 发出的每个请求都必须与记录一致：易变的预算和截止字段、world 除外；world 按 `scene_fingerprint` 核对，探测路径按长度及完整规范二进制 SHA-256 核对。
- 不一致即报 `NATIVE_FLOW_DIVERGED`，不回退。
- 结论、失败信息和证据文件仍由原 Python 代码产生，prepare 结果来自 worker。
- 候选按进程车道运行，每条车道一个 worker，一个候选的全部规划都在该车道的 worker 上进行。并行车道流程（示例配置 7 条车道）与 PYTHON 并行流程中每个 worker 的规划请求序列相同。只有一条车道时（PYTHON 顺序流程），PYTHON 把接近和带载返回分到两个 worker 并行规划，NATIVE 仍在一个 worker 上依次规划；若需要 OMPL，worker 的随机数历史可能因此不同。

### A.7 不得改变

`allowed_pairs` 的构成规则；`clearance_policy.source_policy` 的 2.4 mm / 2.0 mm / 0.2 mm 值（`Clearance::Clearance` 校验 `UNSUPPORTED_ECO65_CLEARANCE_POLICY`）；IK 容差 2e-5 m / 2e-4 rad；分支数 `IK_BRANCHES_CHECKED=3`（离 q_start 近者优先）；`LIN_RETRY_SCALES=(0.5,0.25,0.1)`；`DETACH_ALTERNATIVES`；`SEND_SLOWDOWN_STEPS`；`TARGET_BLOCK_MARGIN_M=1e-4`；`PAYLOAD_REST_DEPTH_M=0.001`；"优先级最高、规划与最终检查都通过的候选被选中"。

## B. 轨迹准备

现有实现：`src/unloading_sim/workbench/controlled/trajectory.py prepare(points, limits, time_scale, period, reach_bound_m)`；调用方 `SinglePlanner.plan_segment`、`EmptyPlanner.plan`。

### B.1 输入

| 字段 | 单位 | 约束 | 错误码 |
|---|---|---|---|
| `points[i].t / q / v / a` | s / rad / rad/s / rad/s² | t 严格递增、t₀=0、端点 v、a = 0（容差 1e-6） | `INVALID_TIMED_TRAJECTORY`、`INVALID_TIME_AXIS`、`NONZERO_ENDPOINT_VELOCITY`、`NONZERO_ENDPOINT_ACCELERATION` |
| `limits` | `Limits(lower, upper, velocity, acceleration)` | 每项 6 个有限值；现场为官方位置限值、0.10 rad/s、0.20 rad/s² | `INVALID_LIMITS` |
| `time_scale` | 无量纲 | 1 ≤ s ≤ 100，`t·s ≤ 600 s` | `INVALID_TIMING`、`INVALID_TIME_AXIS` |
| `period` | s | 0.005 ≤ p ≤ 0.009（现场 0.005） | `INVALID_TIMING` |
| `reach_bound_m` | m | > 0；`EmptyPlanner.model_reach_bound` 加带载包络半径 | `INVALID_REACH_BOUND` |

### B.2 输出（`prepare` 返回字典）

`schema=eco65_timed_send_v1`、`period_s`、`time_scale`、`source_points`、`source_duration_s`、`scaled_duration_s`、`send_duration_s`、`points`（5 ms 网格上的 `t,q,v,a`，终点保持 ≤ 1 个周期）、`collision_path`（检查用几何路径的稠密状态，含全部发送点与节点）、`limits`、`collision_sampling`（`reach_bound_m`、`analytic_speed_bounds_rad_s`、`maximum_pair_displacement_between_checks_m=0.001`、`subdivisions_per_send`）、`interpolation=quintic_q_v_a_nonuniform_knots`、`terminal_hold_s`。第 1 步起单箱首段可另带 `start_joint_check`。

插值规则：保留原生非均匀节点，五次 Hermite（q、v、a），`v/s`、`a/s²`；位置 / 速度 / 加速度在节点和导数零点处全部检查（`INTERPOLATION_LIMIT:j:order`）；离散命令速度、加速度（`DISCRETE_COMMAND_LIMIT`）；相邻命令差 ≤ 0.002 rad（`COMMAND_DISCONTINUITY`）；网格点数 ≤ 120001（`TRAJECTORY_TOO_LARGE`）；检查路径点数 ≤ 500000（`COLLISION_PATH_RESOURCE_LIMIT`）。

第 6 步（`sent_path_audit=ADAPTIVE`，实际实现）：`prepare` 输出不变；worker 沿与 SAMPLED 完全相同的 `collision_path` 采样点序列，只跳过由逐对力臂位移上界从上一个精确检查点证明通过的采样点（`docs/sent_path_adaptive_audit.md`），判定与首个失败与 SAMPLED 相同。原计划的 `geometric_path` 输出不需要，未实现。第 7 步实际：`native_prepare.h` 按 scipy 1.14 / numpy 1.26 的运算顺序实现；越限拒绝两边一致，点数、细分数、检查状态数相同；q 与检查路径相差 ≤ 4e-14，v、a 受 numpy 在 AVX-512 上的数组 `pow`（与 libm 差 1 ulp）影响，最大相差 5.4e-12 与 8.4e-10（见第 7 步报告）。

### B.3 不得改变

5 ms 周期、两个限值、1 mm 物体对位移规则（第 6 步只按 AGENTS §3b 改变检查步长的推导，保证的间隙不低于现行）、`SEND_SLOWDOWN_STEPS` 逐级放慢规则、发送点集合。

## C. 现场场景构建

现有实现：`single_carton/site_scene.py build_scene(observation, result, boxes, config, self_filter, neighbor_depth_m)`、`load_site`；自滤波 `single_carton/self_filter.py RobotSelfFilter.contains`；调用方 `TaskManager.observe`、`tools/replay_single_carton_record.py`。

### C.1 输入

- 冻结 RGB-D：`observation.frame.depth`（uint16，乘 `depth_scale_m` 得米）、`rgb`、`intrinsics`（fx、fy、ppx、ppy、width、height、coeffs、model）、`observation_id`（`contracts.RGBDFrame`）。
- 掩码：`result.masks[i]`（bool，与帧同尺寸）、`result.observation_id` 必须等于帧编号（`SCENE_FRAME_MISMATCH`）。
- `observation.q_capture`（自滤波用的采集姿态）、`observation.world_camera`（world=base_link，`CoordinateModel.camera`）。
- 箱体 `boxes`（`perception.recover_boxes`：`id`、`pose`、`size_m`、`corners_m`、`faces`、`quality.observed_support_plane`、`detection_index`）。
- 现场配置（`load_site`）：`workspace_bounds_m`、`voxel_m`（0.005–0.08，网格 ≤ 300000）、`unknown_policy`（OCCUPIED / USER_CONFIRMED_CLEAR + `unknown_clearance_basis`）、`measured_clear_regions`、`fixed_obstacles`、`target_envelope_points`（OBSTACLE / TARGET_USER_CONFIRMED + basis + margin 0–0.02 m）、`receiver`。
- `neighbor_depth_m`（箱型先验空间对角线 + 容差，`TaskManager.observe`）。

### C.2 输出（`build_scene` 返回字典）

`schema=eco65_real_rgbd_scene_v1`、`frame=base_link`、`world`（`category`：measured_fixed、observed_support、observed、unknown、neighbor_shadow、workspace_boundary）、`observation_id`、`q_capture`、`T_base_camera`、`unknown_policy`、`unknown_voxels`、`unknown_collision_voxels`、`user_confirmed_clear_unknown_voxels`、`unreconstructed_detections`、`neighbor_shadow_voxels`、`neighbor_shadow_depth_m`、`target_envelope_points`（及 basis、margin、evidence）、`unknown_clearance_basis`、`unknown_clearance_evidence`、`observed_voxels`、`target_local_scene`、`observed_supports`、`near_target_measured_extent_runs`、`near_target_refined_runs`、`near_target_refined_boxes`、`near_target_subcell_m`、`camera_facing_measured_extent_runs`、`camera_facing_rule`、`target_local_rules`、`measured_clear_regions`、`self_returns_removed`、`self_filter_pose=q_capture`、`imported_simulation_objects=0`、`hardware_execution=NOT_RUN`。

### C.3 规则（第 9 步逐条保留）

完整投影足迹有效且更远的深度才算已观测自由体素；目标点归属（掩码、目标平面 4 mm 带、侧面、15 px 带）；飞点（深度跳变 > 12 mm）；用户确认的包络点归属（不向包络底面以下）；目标下方支撑面；近目标 6 cm 内按实测范围、1 cm 子格；朝相机一侧收紧（仅 USER_CONFIRMED_CLEAR）；未重建邻箱的阴影体素；六面 4 cm 边界墙；自滤波只去除落在同次 q_capture 源 CAD 凸实体内的点，体素 8 角都在同一实体内才算机器人占用，非凸网格不参与。

### C.4 取消、线程、Python 保留

构建在 `TaskManager.observe` 的工作线程中运行，期间检查 `stop_event` 与观测代际（`OBSERVATION_INVALIDATED`）。`vision_module=NATIVE` 时 C++ 输出直接进入 worker 的场景句柄，`site-scene.json` 仍异步写出；DINO / SAM 推理、停稳取景、`recover` / `recover_partial`（RANSAC 确定性）留在 Python，除非实测耗时显著。

## D. 执行进程

现有实现：`controlled/service.py worker`（执行子进程）、`ExecutionService`；`controlled/engine.py Supervisor`、`MonitorConfig`；`controlled/transport.py RealmanTransport`、`StopChannel`；`controlled/trajectory.py Authority`；调用方 `single_carton/cycle.py Cycle`。

### D.1 操作（`service.worker` 的管道协议）

| 操作 | 输入 | 输出 / 语义 |
|---|---|---|
| `connect` | host、port | 能力（SDK 1.1.7、ECO_65、控制器第 3 代、固件 V1.7.0、UDP 推送节奏）；`STOP_UNCONFIRMED` 时拒绝 |
| `snapshot` | — | 最新 UDP 样本：`source=LIVE`、`sequence`、`received_s`（内核时间戳回推）、`q`、`velocity`、`velocity_source`、`enabled`、`alarm` |
| `feedback` | — | `Supervisor.feedback`：`task_id`、`state`、`reason`、`command_q`、`goal_q`、`actual_q`、`tracking_error_rad`、`feedback_age_s`、`sent_points`、`arrival_confirmed`、`stop_confirmed` |
| `check_payload` | — | 控制器工具负载核对（空闲时） |
| `submit` | 轨迹包（`schema`、`session`、`plan_id`、`prepared`、`binding`、`evidence`）、`cancel_epoch` | 写 `trajectory-package.json`、`task.json`、`controller.json`、打开 `events.jsonl` 后 `Supervisor.submit`；返回运动任务号 |
| `cancel` / `disconnect` / `close` | 任务号 | 软件停止；停止后等待反馈确认 |

第 10–11 步新增：`preload`（PLAN_READY 时一次交入四段，完成限值检查与落盘）、`start_segment(k)`（每段开始时仍做许可核销、起点与停稳检查、控制器负载核对）、事件推送（到位、停止确认、故障，取代 20 ms 轮询，轮询保留为看门狗）。

### D.2 阈值与停止原因（逐条照搬）

`MonitorConfig`：起点 0.01°（1.745e-4 rad）、跟踪 3°持续 0.10 s、到位 0.5°、停稳 0.5°/s、反馈超时 0.10 s、到位停留 0.4 s、完成超时 5 s、停止超时 2 s、抖动 2 ms、最大发送间隔 9 ms；现场文件只能收紧（`field_monitor_config`）。续接 1e-7 rad（`CONTINUATION_RAD`）；用户关闭起点核对时只放行首段且须带依据（`Supervisor.submit`）。
停止原因：`SEND_DEADLINE_MISSED`、`SDK_SEND_OVERRUN`、`SDK_SEND_ERROR`、`SDK_SEND_EXCEPTION`、`FEEDBACK_EXPIRED`、`CONTROLLER_ALARM_OR_DISABLED`、`TRACKING_ERROR`、`ARRIVAL_TIMEOUT`、`SEND_THREAD_STALLED`、`OPERATOR_SOFTWARE_STOP`、`LOG_WRITE_FAILED`、`TERMINAL_INTERRUPT_SOFTWARE_STOP`；状态 `READY / PREPARING / EXECUTING / SUCCEEDED / STOPPING / CANCELLED / FAILED / STOP_UNCONFIRMED`，`STOPPING → STOP_UNCONFIRMED` 锁存；停止后在途发送返回时重新请求停止（`after_inflight_send`）。

### D.3 许可核对摘要

`Authority.register` 对整个包求 `digest`（第 7 步起为规范二进制摘要）；`ExecutionService.submit` 在提交前 `claim`（核对 `schema`、`session`、`binding`、摘要后核销）；`binding` 由 `EmptyPlanner.binding` / `SinglePlanner.binding` 生成（文件哈希、模型、场景、提交号、映射、质量、控制器负载、外参、`physical_cycle`）。

### D.4 线程、取消、Python 保留

执行子进程内：定时发送线程（每 0.5 ms 一次 tick，至多每 tick 一条命令）、看门狗主循环（20 ms 无 tick 则 `SEND_THREAD_STALLED`）、记录线程、UDP 接收线程、停止线程。取消走独立 `stop_event` 与 `cancel_epoch`，不等待 GUI 锁。`scheduling_module=NATIVE` 时这些移入一个 C++ 进程，Python 只保留提交、取消、查询和界面；SDK 二进制、调用方式、停止通道、UDP 套接字与内核时间戳、连接租约锁、断开时恢复实时推送配置都不变；能取得 SCHED_FIFO / mlockall / 绑核则用，否则如实报告。

## E. 连续作业层

依据 AGENTS.md 2026-11-16 §6；入口 `unloading_sim.workbench.continuous`（`eco65-workbench-continuous`），复用 `TaskManager`（`observe`、`prepare_plan`、`execute`、`stop`）与 `Cycle`（`ready`、`start`、`tick`、`status`）。

### E.1 与 TaskManager / Cycle 的接口

作业层只调用 `TaskManager` 的公开操作并读取 `Cycle.status()`；每个周期仍是不变的单箱周期，有自己的任务目录与 `result.json`。作业层不直接调用 worker、执行进程或真空设备。

### E.2 目标排序

输入：本帧重建的箱体列表（`perception.recover_boxes` 输出：`id`、`pose`、`size_m`、`corners_m`、`detection_index`）、本帧现场世界（契约 C）。输出：有序列表 `[{target_id, rank, top_z_m, layer, base_distance_m, covered_by:[...], reason}]` 与排序规则版本。规则：顶面最高一层优先（容差可配）、同层内离基座近者优先、再按编号；上方被其他箱体包络或实测障碍压着的排到最后（借用 `online_unload.restrict_to_highest_active_layer`、`exposed_carton_order` 与 `support.SupportRelationGraph` 的思路，输入换为本帧重建包络）。可行性仍由原位包络检查、终态预检和四段规划决定。单箱入口保持检测顺序。

### E.3 receiver_clearing 状态机

策略：`SENSOR_OBSERVED_CLEAR`（释放包络在新冻结帧中实测为自由；仅当实测承接区在视野内）、`USER_CONFIRMED_SELF_CLEARING`（用户确认每个已释放箱在 `clearing_time_s` 内离开；记为现场确认，非传感器证据）、`OPERATOR_CONFIRM_EACH`。缺策略或依据即配置错误。状态：`RELEASED → AWAITING_CLEARING → CLEARED | CLEARING_TIMEOUT（作业结束）`；未满足前不开始下一周期；释放位为实测支撑时，已释放箱的完整包络保留为碰撞物体；`USER_CONFIRMED_CLEAR` 永不覆盖已释放箱。

### E.4 作业记录

`job.json`：`job_id`、启动点击时刻、配置摘要、`receiver_clearing` 及依据、`plan_validity_s`（缺省 5 s，只能收紧）、首个 q_start、`cycles=[{index, task_id, target_order, skipped:[{target_id, reason, searched, unsearched}], result, physical_result}]`、`end_reason`（OPERATOR_STOP、CYCLE_NOT_COMPLETE、PRESSURE_LOSS、SESSION_CHANGED、RECORD_WRITE_FAILED、Q_START_DRIFT、NO_TARGET_DETECTED、NO_COMPLETE_PLAN、RECEIVER_NOT_CLEARED、CARTON_COUNT_REACHED）、`PHYSICAL_JOB_COMPLETE` 仅当每个周期都有 `PHYSICAL_CYCLE_COMPLETE`。目标失败汇总格式参照 `contact_scheduler.ContactCandidateScheduler.summary`。

### E.5 不得改变

自动开始下一周期的全部条件（AGENTS §6 第二条）；不跨周期复用观测、计划、IK 种子或路径；无 N+1 预规划；作业结束后不自动恢复或重试；停止行为与单箱入口完全一致。

## F. 字段来源核对

| 契约 | 字段组 | 代码来源 |
|---|---|---|
| A | 请求上下文 | `single_carton/planner.py SinglePlanner.context` |
| A | init 内容 | `controlled/planner.py EmptyPlanner.build`、`single_carton/planner.py SinglePlanner.build` |
| A | 回复状态与失败原因 | `worker.cpp Worker::run`、`clearance.h Clearance::check`、`eco65_single_carton.h SingleCartonContact::check` |
| A | 截止时间 | `single_carton/planner.py deadline_fields / bounded_request`、`worker.cpp Worker::applyAbsoluteDeadline`、`native_deadline.h` |
| A | 并行与池 | `single_carton/planner.py start_check_pool / parallel_candidates / sent_check / checkers` |
| B | 全部 | `controlled/trajectory.py prepare`、`Limits` |
| C | 输入输出与规则 | `single_carton/site_scene.py build_scene / load_site / _target_claims / _support_below`、`self_filter.py RobotSelfFilter` |
| D | 协议与阈值 | `controlled/service.py worker / ExecutionService`、`controlled/engine.py Supervisor / MonitorConfig / field_monitor_config`、`controlled/transport.py RealmanTransport / StopChannel` |
| D | 许可 | `controlled/trajectory.py Authority`、`controlled/planner.py EmptyPlanner.binding` |
| E | 周期与状态 | `single_carton/runtime.py TaskManager`、`single_carton/cycle.py Cycle`、AGENTS.md 2026-11-16 §6 |

没有两套并行概念：截止时间只有一个（第 2 步）；通道只有一种（§0）；每个模块只有一个 PYTHON | NATIVE 开关；机制开关不改变判定。

## 2026-11-17 审查修正

本节修正上述契约中的证据与生命周期边界，不修改碰撞、接触、轨迹或执行判据。

- `prepare_plan` 在入口建立绝对单调 deadline。`SinglePlanner.plan_cycle` 使用独立任务状态和代次，四段许可先写入任务私有 Authority；binding、摘要、必要证据、`Cycle.ready` 均完成后，检查代次、session、取消与剩余预算，再发布完整计划和四段许可。待提交文件含代次，不能作为可执行计划导入。
- 调用者返回与后台工作结束分别记录。`last_task` 和 `planning-task-lifecycle.json` 表示清理完成/错误；旧任务的 worker、writer、场景和许可不交给新任务。正常成功的常驻池可复用，但下一任务必须按剩余预算等待 reset。后台尚未结束不能标成已确认空闲。
- `M710_WORKER_CANCEL=1` 的 JSONL 读取端解析 JSON 识别取消；二进制 kind 4 保持原通道。取消不进入普通请求队列；仅作用于运行中或已接收的 request ID，错误/迟到 ID 不污染后续请求。客户端分别记录 `caller_result_invalid`、`cancel_sent`、`worker_finished`；无法在宽限期确认结束则退役。
- 每次 composite 显式传递任务目录和 task ID。内部 journal 的 START/END 带 `task_id`、`composite_request_id`、`worker_pid`、`index`。打开/写入/flush 失败关闭成功通路；缺少 END 的 START 表示未确认结束。不同任务目录不串写；不增加逐条 fsync。
- replay 保留完整 `probe_path`，用现有 `canonical_digest` 绑定所有中间状态。长度和首尾相同不能替代完整路径身份；不匹配报 `NATIVE_FLOW_DIVERGED`。
- ADAPTIVE 报告 `anchor_dynamic_pair_distance_lower_bound_m` 和 `anchor_workspace_margin_m`，范围为 `EXACT_ANCHORS_DYNAMIC_CONSTRAINTS_ONLY_EXCLUDES_SKIPPED_STATES_AND_FIXED_PAIRS`；`whole_path_minimum_distance_status=NOT_COMPUTED`。它们不是整段最小净空。原跳点证明和碰撞通过/拒绝规则保持不变。
- 原生 prepare 补齐限值长度、有限数、严格上下界与正速度/加速度检查。数值等价性仍按 `1e-12`，本轮存在未通过项，详见修复报告。
- 启动前必须 flush 本次新写入的观测、规划和任务 journal；等待可取消。运行中记录设施失效仍进入原停止/持箱处理。

统计契约：成功规划是 `prepare_plan` 入口到 `PLAN_READY`；失败另外保留入口到返回及分类。局部回放标为 `PARTIAL_REPLAY_NOT_PREPARE_PLAN`，不能据此判定完整入口达标。调度为 execute 到首点，加四个交接区间扣除既有规则必要等待后的超额时间；重叠等待取区间并集。缺少任一交接证据时只报告部分口径。Python 调度和不可抢占调用不构成硬实时保证。

合并说明：并行提交 `6477f32` 的事故执行锁、物理单箱工具—机器人 40 mm 策略及腕部限制保持原样。上述 A–F 修正不撤销这些保护，也不将其推广到 FANUC/M710/离线模式。具体条件和例外以该提交的现场配置与范围检查为准。

### 2026-11-17 复审补充：复合腕部策略

`sc_endpoint_screen` 和 `sc_plan_candidate` 的公共输入增加解析后的 `wrist_branch_limits` 及冻结 `wrist_reference_q_start`（弧度）。参考始终是任务 q_start，不是各段起点；无策略的历史请求保持无策略。C++ 在与 Python 相同的位置应用严格度数判据：IK 去重/排序后、最多三分支检查前过滤；每段 prepare 后在完整 collision_path 上检查，再决定是否尝试下一脱离方案。末次 Python 审计仍保留，回放未消费调用不得被普通候选拒绝掩盖。全部 scene handle 上传传播取消回调。

### 2026-11-18 超时返回的证据调度

截止报告不再由已超时的规划调用者启动新线程；由该任务的既有后台线程在记录回滚和资源清理之后写入。`last_task.deadline_record_status` 为 `PENDING_TASK_CLEANUP`、`WRITTEN` 或 `FAILED`，并保留写入完成时间或错误。异常上的 `record_thread` 此时是任务线程，等待它也包含任务收尾，不能据调用者返回推断记录、worker 或清理已经完成。

真实 TaskManager 的拒绝记录与 prepare_plan 计时写入使用已经启动的任务 journal。回执提供 `join`、`is_alive`、`error`，journal 在接收后失效或关闭拒收均可观察。计时写失败不改变规划判定；执行前必要证据的 flush、许可事务和记录设施失效后的原停止行为保持不变。100 ms + 50 ms 回归覆盖新线程启动延迟，但不构成操作系统硬实时保证。

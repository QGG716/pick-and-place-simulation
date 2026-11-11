# S50 接口与原生规划链路审计

日期：2026-11-10。基线：`a3f08cc2283138e5905b4cc9829cefd7b4aac6b7`。

## 本轮继续实施：真实 CAD 六轴与实际 native FK

用户授权 STEP 装配状态作为仿真零位、测得轴线按右手正向旋转后，已接入真实 S50 分连杆 CAD 的六关节 URDF。此仿真坐标约定不等于厂家控制器零位。`cad_preview_fk_only` 的角度区间只供预览；正式控制器映射、实际安装 clocking、吸具物理连接和碰撞资格仍分别检查。

服务器独立目录 `/work-s50-cad-motion-20261110-v1` 的第一轮实际结果见 [`../s50_cad_motion_20261110/native-fk-report.json`](../s50_cad_motion_20261110/native-fk-report.json)：15 组姿态比较核心 URDF、Pinocchio/Coal 与 C++ MoveIt worker，最大矩阵误差分别为 `1.11e-15`、`4.44e-16`，加载 9 个真实碰撞几何对象。worker 实际拒绝 IK 请求，返回 `CAD_PREVIEW_FK_ONLY_NO_MOTION`。这是实际模型加载与 FK 验证；没有进行碰撞接受、初态求解、单箱运动或动力学验证。模型定义 SHA-256 为 `38c6f9c501b7539ffc4417a3c397dfc2e363ff85da18bec83afa3cd834b38036`。

本轮新增实际实现：

- `PinocchioHppFclBackend.from_binding` 使用同一源 URDF/SRDF/网格；仅输出目录内副本补解析字段，世界基座变换只应用一次。没有胶囊来源时禁用胶囊接口，不猜半径。OBJ 几何由同一绑定解析；读取顶点边界只用于 broadphase，实际碰撞权威仍为 Coal 网格。
- `native_fk_model_request` 和 C++ `cad_preview_fk_only` 会话只提供 FK；不初始化规划 pipeline，不允许 IK、碰撞资格或运动请求。未知 effort 的零值仅为 URDF parser 占位，源文件不变；源配置已声明的 effort 占位也记录为未知实际力矩。
- `configured_layout_trajectory.make_configured_context` 从同一绑定读取实际 Coal 底座包络，建立新机器人完整指纹、冻结环境、原吸具及显式法兰连接。`build_configured_layout_trajectory_connector` 真正构建现有 exact validator/connector，核对 FK、Jacobian 与工具 frame。没有复制 FANUC 径向 reach 或大臂/小臂姿态限制。
- `tools/plan_s50_single_carton.py --mode single-carton` 在 `geometric_planning.ready` 为真后执行独立、确定种子的初态 IK；严格检查限位、自碰撞、机器人/工具/环境。合格后才创建新 40 箱快照，再调用现有 `gap_search_recipes.candidate_options` 和 `plan_m710_joint_search.run_trial`。原高层 FRONT 抽离后直接释放、退出、返回与输送带升降流程保留。`front_arm_posture=none`，不使用旧 FANUC 初始 q。
- native 数值资源只读取冻结实际 launch 的 `600/360/360/30/20` 秒配置；共享单箱候选总资源，不重置已消耗预算。可显式指定资源上限；不是新设业务截止时间。初态搜索另外记录全部有限种子、迭代上限、失败构型与未搜索数量。
- S50 基座不继承 FANUC `base_link/mount_plinth` 接触例外；未核定实际安装面接触时，支座保持普通严格碰撞体，可能真实导致初态失败。未通过移动支座、重命名机器人或宽免碰撞消除此问题。
- 原生时序从明确的速度/加速度资料或显式工程假设读取，记录来源与有效值，不给 S50 套用 FANUC 的默认加速度。时序资料缺失不阻断静态 FK 或初态几何检查，但阻断需要时序的 native 轨迹生成。
- `verify_m710_joint_search.verify_cycle` 接受绑定的真实 base 角色，并逐段核对当前任务身份、保存的 native SUCCESS 响应及原始中间轨迹点；端点相同不能掩盖中间点替换。只有通过原生完整周期审查的轨迹才导出 `s50_native_joint_trajectory_v1`。文件带模型/场景身份、验证及 plan 哈希、当前任务 provenance；回放范围明确为 `robot_joint_only`，环境是 `initial_scene_static_reference`，不能冒充箱体/横带状态动画。

当前正式入口实际运行结果为 `NOT_RUN_MODEL_NOT_READY`，在启动 worker/IK 前列出物理数据缺项。正式 context、初态、run_trial、轨迹导出调用链已接线，但 S50 正式安装/工具定义未齐，尚未实际跑过该完整单箱链路；测试中的人工 fixture 不作为其成功证据。不能写“机械臂替换完成”。

最新本地聚焦测试：

```powershell
& 'D:/code/eco65 pick and place/.venv/Scripts/python.exe' -m pytest `
  tests/test_configured_native_entry.py tests/test_robot_model_binding.py `
  tests/test_joint_search_layout.py tests/test_moveit2_tcp.py `
  tests/test_contact_simulation_policy.py --basetemp=outputs/pytest-s50-chain-final -q
```

结果：`57 passed, 1 skipped`。跳过项为本地未安装 Pinocchio 的真实 FANUC 新旧 Pin 加载器三姿态 FK/Jacobian/网格包络回归，供服务器同依赖环境执行。其余覆盖 preview 拒绝 motion、frame/TCP、缺 effort、明确时序、旧 FANUC native 默认、安装例外保持严格、初态碰撞失败保留、全新 40 箱 `run_trial` 调用接线、导出到 viewer 合同、中间轨迹点篡改拒绝。接线测试明确使用人工 fixture；不是 S50 规划成功。

最终集成复核进一步覆盖 9 个相关测试模块：Windows 为 `155 passed, 1 skipped`（19.59 秒）；服务器为 `156 passed, 0 skipped`（46.58 秒）。服务器逐一核对 577 个同步文件 SHA 后运行，实际完成上述 FANUC 三合法姿态的核心/新旧 Pinocchio FK、Jacobian 和 7 个原碰撞网格包络一致性检查。这是共享加载接口回归，没有启动 FANUC 搜索。实际 native worker 二进制 SHA-256 为 `0fbb1bd6211b68981c16c03dc3056d1db5ddc686470a78ab00d94cdf6d414015`。汇总证据由 `s50_cad_motion_20261110/verification-summary.json` 记录。

可复现入口（在独立工作目录，`PYTHONPATH=src:tools`）：

```bash
python tools/plan_s50_single_carton.py --mode preview-fk --output outputs/s50-preview-fk --worker-command '<独立编译的 worker 启动命令>'
python tools/plan_s50_single_carton.py --mode single-carton --output outputs/s50-single-carton --worker-command '<独立编译的 worker 启动命令>'
```

正式入口失败时返回非零并保留 JSON。当前不启动全 40 箱搜索，不触碰 FANUC worker 或其检查点。

## 第一轮历史记录（尚未获准 CAD 仿真坐标时）

以下记录保留首轮范围：当时官方标量规格和用户 STEP 不足以核定厂家控制器零位、方向及完整坐标映射，因此只完成通用接口、合成 fixture 的协议测试，没有宣称真实 S50 原生规划成功。其“未完成”清单以本轮新增实现与实际结果为准。

## 已实现的接口

- `robot_model_binding.RobotModelBinding.from_config(path)`：从显式机器人配置读取 URDF/SRDF、六关节顺序、base/flange/tip、安装面到 URDF base 的变换及包映射。验证完整六转动关节链、真实法兰处于最后主动关节之后、关节轴、限位、速度、网格路径及 SRDF 链。配置中的限位/速度与 URDF 不一致时拒绝。缺字段通过 `ModelNotReady.missing` / `.errors` 返回；不把缺惯量、缺力矩当作运动学阻断。
- `URDFRobot.from_config(...)`：`W_base = W_mount @ mounting_from_base`。安装工具时必须显式给出 `tip_from_tcp`；未安装工具可使用裸机 tip 做 FK。不会继承 FANUC 胶囊尺寸或自碰撞例外；未提供可核验胶囊半径时，FK 可用，胶囊碰撞调用明确失败，必须使用合格网格后端。
- `robot_native_binding.native_model_from_binding(binding, world_from_base, project_root=..., asset_root=...)`：使用同一份 URDF/SRDF 生成 native XML 输入，解析配置的网格路径，可显式重定位到服务器资产目录。支持 base 为根，或已有固定 world→base；未知上游坐标链拒绝。若真实法兰不叫 `flange`，只添加同坐标、零长度、无碰撞体的固定别名供旧 C++ 协议使用，不添加物理连接件。
- `moveit2_backend.model_request(..., model_binding=...)`：显式配置路径或对象进入新支路；检查核心机器人与 native binding 身份一致，不回退 FANUC。init 身份保留模型/config/URDF/SRDF 以及全部 visual/collision 网格的加载时哈希、关节顺序、frame roles、实际场景指纹；native 构建前复核当前文件，防止旧 FK 实例接受新模型资产。Python 抓取附件的法兰读取改为配置角色。`check_fk` 对已绑定机器人使用合法关节范围生成探针和配置连杆名；历史 FANUC 探针保留。
- `SimulationCollisionPolicy.for_robot_binding(binding, existing_policy)`：保留原净空、柔性吸盘及阶段规则，只从显式配置的 `collision.owned_tool_exempt_robot_links` 读取腕部/自有工具例外。非空例外还需要 `owned_tool_exemptions_source`。配置绑定 model ID 与 URDF SHA256，拒绝近端机械臂链接和旧整臂/底盘免碰撞选项；空列表是明确的无例外配置，`null` 是资料未就绪。FANUC 默认允许的 J5/J6 语义和历史序列化指纹保持不变。

这些入口仍要求上层提供经核验的工具坐标、碰撞 authority 和场景。仅得到一个 `RobotModelBinding` 不代表完整 S50 环境碰撞或完整卸箱流程已接通。

## 缺失力矩的 URDF 解析兼容

urdfdom 的 `parseJointLimits` 在缺少 `effort` 属性时返回失败，尽管几何规划本身不需要真实驱动力矩。[官方解析实现](https://raw.githubusercontent.com/ros/urdfdom/master/urdf_parser/src/joint.cpp)

新 native builder 仅在内存中的传输 XML 副本为缺失的 `effort` 补 `0`。这是解析占位，含义是实际执行力矩未知，不是额定力矩，也不是无限驱动力；不补惯量，不回写模型源文件，不修改已有合法 effort。已有非有限、负数或非数字 effort 明确拒绝，不能被占位值覆盖。`identity.native_urdf_parser_compatibility` 记录补值关节、未知实际力矩关节、零值、用途、源文件未修改，以及 `dynamics_qualified=false`；显式绑定的 native identity 同样标记动力学未验证。

分析 fixture 的原始/适配 URDF 和元数据输出在 `outputs/s50_binding_fixture/`，它们不属于 S50 模型资产。GPU 服务器已经使用实际 `/opt/ros/humble/bin/check_urdf` 对照验证：原始缺 effort 的 fixture 退出码 255，报告缺少 joint effort；适配后的 fixture 退出码 0，XML 解析成功，链为 `world → mount_base → segment_1…segment_6 → physical_tool_face → flange`，机器人名称仍为 `analytical_fixture_not_s50`。原始日志位于 `/work-s50-integration-20261110-v1/outputs/s50_model_integration/source-parser.log` 与 `native-parser.log`。这是实际 ROS URDF 解析兼容验证，不是 S50 模型、MoveIt 规划、碰撞或动力学验证。

## 未接通的真实 S50 生产链

以下旧入口不能仅替换 YAML 后直接作为 S50 接入完成的证据：

| 环节 | 仍存在的耦合 | 真模型准备好后必须处理 |
| --- | --- | --- |
| `tools/plan_m710_mount_comparison.py:make_context` | 固定加载 M710 validation、调用旧 layout audit、初态审计及自动 connector | 使用冻结的替换 layout、S50 新解初态和有模型作用域的 policy；不得复用 FANUC 六个初始角度或历史完成检查点 |
| `layout_single_carton._build_automatic_trajectory_connector` | 从工作单元 config 构建，但最终调用 `build_m710_layout_trajectory_connector`；传递旧 mount/TCP 和径向 reach 参数 | 新增显式绑定构建分支，使用真实 W_base、工具完整变换和 S50 数据，保留旧 FANUC 路径 |
| `layout_trajectory.build_m710_layout_trajectory_connector` | 固定 FANUC manifest/URDF/SRDF 路径与审核；固定 `tool0`、J1…J6、FANUC 工具矩阵、法兰名及部分探针 | 以同一 binding/source evidence 替代机型身份假设；对核心、Pinocchio、native 做实际 FK/Jacobian/网格一致性检查，保持合格工具和 5 mm 规则 |
| Pinocchio/Coal 适配 | 构造器本身接受 URDF、tip 和基座变换，但旧 factory 不把新 binding 传到实例；部分肘部净空诊断固定 J2 | 使用同定义的解析兼容副本（若源 URDF 缺 effort），显式检查关节顺序，传递模型绑定；肘部/腕部角色需真实来源，不能按旧名称盲配 |
| `validation_physics.urdf_collision_shapes` | 网格包解析使用目录名查找，碰撞网格仅接受二进制 STL | 从 binding 的包映射解析并验证每连杆网格；使用 STEP 转换时保存单位、局部坐标及转换尺寸证据 |
| `joint_search_layout.qualified_mount_pairs` 与 C++ `process_policy.h` | 安装接触检查使用 `base_link`、旧 mount 数值与固定站位 contract；C++ 把 URDF base pose 直接当真实安装面 | 明确真实安装面与 URDF base 的坐标映射；仅对真实安装接触给出证据，不通过重命名整机或移动支座掩盖差异 |
| `tools/plan_m710_joint_search.py:run_trial` | 可复用单箱流程，但仍选择旧 recipe、posture policy 与 `verify_m710_joint_search` | 用新 context 保留抓取/抽离/直接高层释放/退出/返回/升降顺序；姿态偏好先禁用旧结构判断或按真模型适配，不新增抓取端点硬门槛 |
| `front_arm_posture.py`、Pinocchio 上臂净空诊断 | 结构判断使用 J4/J5 或 J2 等 FANUC 连杆名 | 用核定的 S50 肩肘腕角色计算，数值求解失败不能称为结构不可达 |
| 离线回放 | `export_m710_search_replay.export_model` 可读取 native XML、关节列表和同一碰撞网格，但 viewer 的 contact/附件仍使用 canonical flange | 真模型就绪后可消费新 builder 的同坐标 flange alias；必须用真实 native 输入导出和实际轨迹验证，而不是为整机 CAD 启用假关节滑条 |

C++ worker 已按收到的 `robot_description`、`robot_description_semantic` 建模，并检查规划组的变量顺序与请求的关节顺序一致；`manipulator` 规划组仍是本适配的明确兼容要求。C++ 多处 payload 和过程规则仍使用 `flange`，本次提供的同坐标别名解决该 frame 名称需求；它不解决未核定的安装接触或工具物理关系。本次没有修改 C++ 算法、启动 S50 worker 或生成 S50 规划成功记录。

## 实际测试

在独立 S50 worktree 使用现有只读 Python 环境执行：

```powershell
& 'D:/code/eco65 pick and place/.venv/Scripts/python.exe' -m pytest `
  tests/test_robot_model_binding.py tests/test_moveit2_tcp.py `
  tests/test_contact_simulation_policy.py -q `
  --basetemp=outputs/s50_binding_pytest_effort_01 --tb=short
```

结果：**29 passed**。其中 binding 测试覆盖解析 fixture 的安装坐标、正负转向、限位/速度冲突拒绝、缺失几何拒绝、native 模型输入生成、同坐标 flange alias、作用域明确的腕部规则、缺失 effort 的源文件不变以及官方 FANUC 已有 effort 原样保留。官方 FANUC 新旧加载接口在三组合法姿态的 FK 与所有链上 frame 完全一致。

最终独立审查另外修复了完整网格身份与加载后资产变化检测；拒绝未声明连杆、非独立 mimic 关节和无效关节坐标；准入检查拒绝空来源、非法工具 SE(3)、缺失的逐连杆 visual/collision、未经一致核对的 SRDF 排除，以及未解决的速度版本资料冲突。准入仍是资料/定义检查，不代替真实网格碰撞或运动验证。完成这些修复后的聚焦命令为：

```powershell
& 'D:/code/eco65 pick and place/.venv/Scripts/python.exe' -m pytest `
  tests/test_robot_model_binding.py tests/test_robot_replacement_layout.py `
  tests/test_s50_integration_admission.py tests/test_moveit2_tcp.py `
  tests/test_contact_simulation_policy.py -q `
  --basetemp=outputs/s50_review_pytest_02 --tb=short
```

结果：**54 passed**。布局中任何运行网格改变后，都必须重新生成 binding 身份；旧完整场景和旧 native 模型输入不能继续复用。

此前执行 `tests/test_moveit2_backend.py` 为 11 passed、1 failed；失败为 `test_authority_rejection_continues_ompl_and_deduplicates` 的旧 `SimpleNamespace` fixture 缺少 `robot_mount_pairs` 方法。使用固定基线原始 `moveit2_backend.py` 已独立复现相同失败，不是本次改动引入，未通过修改碰撞检查掩盖该失败。

尚未完成的真实 S50 验证：六轴模型来源交叉核对、核心/原生/viewer 多姿态 FK、S50 新初态限位/自碰撞/环境碰撞、顶层单箱 native 规划、实际失败构型/碰撞对象回放、真实肘腕包络与最小间隙。因此当前交付不能称为“机械臂替换完成”或“原生单箱规划通过”。

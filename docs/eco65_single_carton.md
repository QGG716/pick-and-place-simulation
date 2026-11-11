# ECO65-B 视觉单箱抓取并返回释放

此入口处理一箱：停稳采集 → 检测与 SAM 分割 → 150×100×100 mm 长方体恢复 → CAD 密封检查 → 完整原生规划 → 操作者执行 → 实测接触到位 → 真空确认 → 脱离 → 带载返回本次 `q_start` → 释放确认。程序、设备连接、加载配置和预览均不自动动作。

默认只读入口和原有低速空载入口保留。没有输送带、升降、多箱、在线并行规划或视觉伺服。本轮实机抓箱验收 **NOT_RUN**；软件测试通过不代表实机完成。

## Ubuntu 启动与操作

在项目根目录运行（使用现有 `.venv`、Humble/MoveIt 安装）：

```bash
.venv/bin/python -m unloading_sim.workbench.single_carton.ui --check-config
.venv/bin/python -m unloading_sim.workbench.single_carton.ui \
  --config configs/workbench/single_carton.example.json
```

新增安装后的命令名为 `eco65-workbench-single-carton`，模块方式无需重新安装包。安装缺少的可选 Python 依赖时使用 `workbench-perception`；当前工作台已具备实际可加载的本地 HF 权重，不需要更名权重或下载另一份模型。

1. 关闭先前占用机器人/串口的工作台；旧工作台有活动任务时先按其停止流程处理。新入口使用机器人独占连接和串口独占锁，也检查旧进程是否仍占用串口。
2. 在“观测与检测”页手动连接 D435i、选择图像格式并开始采集。在“视觉单箱任务”页连接机械臂及真空板。两个连接按钮均不发运动或阀门命令。
3. 原空载控制器设置为 **3 kg**。单箱配置保留实际工具 **1.5 kg**、箱子 **2 kg**，当前带载控制器预期为 **3.5 kg**。请在现场控制器工具配置中核对该设置，再点击“刷新控制器负载（只读）”。本入口不会在连接或执行中偷偷修改工具坐标、质心或负载。负载不匹配时禁止发运动。
4. 在已确认的空旷范围内停稳，安排一个可见箱子，点击“准备观测 / 锁定起始点”。此时锁定实际六关节 `q_start`、起始 TCP、RGB-D 原帧和采集关联关节。不是零位、HOME 或配置初态。
5. 查看冻结图像、检测编号和恢复长方体。目标编号留空会选择完整流程可行的一个；也可明确填写图上的编号。点击“规划所选箱”。规划会先检查带载终态，再规划并验证全部四段运动；不存在“先抓起来再尝试找返回路”。
6. 点击“查看完整轨迹预览”，检查目标、接触 TCP、预计密封杯、四段关节轨迹、时长及返回位置。默认时间放大 10 倍，发送周期仍为 5 ms，速度和加速度分别按 1/10、1/100 缩放。
7. 点击一次“执行单箱”。执行前再次停稳取景，比较原观测与新观测的目标、场景、相机身份和实测起点，并重验文件绑定；任何变化均撤销计划，需重新准备。之后各阶段依据反馈自动转换，无需逐段点击。
8. 正常验收检查：接触实测 FK 满足密封条件；吸具压力稳定达标后才脱离；箱子实际离开原处；带箱返回同一 `q_start` 后释放；释放压力稳定达标后显示完成。执行中停止检查须由现场人员另行安排，确认轨迹不再发送、实际停止被确认且未自动释放。

用户本轮明确“起始点承接区先不考虑”。因此软件不以承接区域资格拒绝任务，仍检查带箱返回与释放包络碰撞，并记录 `RECEIVER_QUALIFICATION_DEFERRED_BY_USER`。这不是承接条件已核验的声明；程序不会擅自改放置点。

## 现场配置和来源

| 项目 | 当前文件 / 内容 |
|---|---|
| 单箱参数 | `configs/workbench/single_carton.example.json`，已填入本轮确认参数 |
| 设备、关节映射、完整场景、工具 | 复用 `configs/workbench/controlled.example.json` 指向的当前安装配置和资产 |
| 安装关系 | `outputs/workbench/installation.local.json`：world=base、相机 link_1、正式吸具 TCP |
| 相机外参原文件 | `configs/workbench/reference/active_extrinsics.json` |
| 原策略参数与来源哈希 | `configs/workbench/reference/strategy.json`、`single_carton_sources.json` |
| 检测权重 | 配置内指定 `models/huggingface/IDEA-Research/grounding-dino-base/...` 目录 |
| 分割权重 | 配置内指定 `models/huggingface/facebook/sam-vit-base/...` 目录 |

外参与真空依据来自用户指定的 [参考工程固定提交](https://github.com/zhangyong333/Unloading-Robot-Control-Platform/tree/f0353d720374066528f258de6f4de24502408535)。原 `active_extrinsics.json` 字节及 `SOLVED_REQUIRES_VALIDATION / TRAINING_ONLY_TEST` 历史标签原样保留；本轮用户确认现场关系并指定引用该工程，是单独的现场使用依据，没有改写历史验收结论。这里加载的是参考工程的原始活动配置，不是 `training_extrinsics.example.json`，也不要求找回其 Windows 历史会话才能开发。

12 杯全部连通一个总阀，用户确认具有自动关闭止回结构。CAD 密封轮廓总范围约 176.114×131.114 mm；在 150×100 mm 面上可生成 6 杯完整覆盖的候选。记录分别列出全部阀门连接杯、预计完整密封杯、预计被动关闭杯。后者不是软件独立控制，全部杯及刚性吸具仍有碰撞几何，实际是否吸牢由压力判断。

| 真空参数 | 值与依据 |
|---|---|
| 吸附命令 / 释放命令 | 参考实现 `0x12 / 0x13`；实际字节 `EE 16 02 03 12 15` / `EE 16 02 03 13 16` |
| 吸牢条件 | 吸具通道 `0x61`，≤ −60 kPa；来自参考 `unloading/strategy.json` |
| 带载失压 | > −30 kPa；参考阈值的一半，来自参考 `scheduler.py` |
| 释放条件 | ≥ −10 kPa；来自参考 `scheduler.py` |
| 稳定判据 | 至少两次新心跳，覆盖 ≥1 s；本轮用户确认新增 |
| 吸附 / 释放超时 | 各 3 s；本轮用户确认，替代原 2 / 1.5 s |
| 压力反馈年龄 | ≤3 s，参考 UI 新鲜度；原协议心跳约 1 Hz，界面刷新不代表更高采样率 |

只接受校验通过的小端 float32/kPa 吸具心跳；储气罐、泵压力及无校验 ASCII 不用于吸附判定。不会发送阈值设置、查询假协议、输送带命令；吸附超时不重试，释放超时不当作完成。调用返回或板卡耗时回报不替代压力确认。

## 数据与规划约束

每次准备新建唯一 `task_id`。采集使用当前连接代际、实际接收时间及停稳窗口：关节持续至少 0.4 s 满足位置范围 ≤0.02°、实测差分速度 ≤0.5°/s；RGB-D 年龄 ≤100 ms，关联关节接收时间差 ≤30 ms。这是停稳条件下的接收时间配对，不宣称硬件触发同步。推理结束后不重取关节覆盖 `q_capture`。

所有图像点和法向采用同一链：

```text
T_world_camera = T_world_base × T_base_link1(q_capture) × T_link1_camera
```

使用真实 GroundingDINO / SAM HF 加载器读取各自 `config.json` 和 safetensors。掩码内部去边缘混合深度、无效点及平面离群点；通过确定种子的 RANSAC、可见边界和已知尺寸恢复中心、朝向、角点与面。单面情形只把该面标为观测，另外五面为尺寸先验推断。保存残差、有效点数、面覆盖、尺寸误差和对称性歧义。掩码截断、混入邻箱、延续平面却无可辨边界、尺寸不符等会明确拒绝。中位数不充当箱体中心或吸取点。

接触朝向来自 CAD 密封轮廓及其法向与真实 TCP 变换。全部选定密封圈须有 ≥3 mm 边缘余量；接触 FK 最大间隙 0.4 mm、法向误差 ≤0.01 rad。仅目标与这些杯唇允许受约束接触，原生逐状态检查穿入深度、覆盖及朝向；未选杯、刚性工具和其他物体无此权限。带载支撑接触只允许明确命名的支撑，绑定真实几何顶面并检查完整箱体不穿入；不使用仿真邻箱/刚性工具豁免。

完整场景保留机器人、吸具、相机支架、厢体和静态输送带几何；箱体由本次恢复的实测对象替换历史仿真箱体。检测到却无法恢复的邻箱会拒绝整次任务。模型、CAD、关节映射、外参、配置、原生 worker 与代码版本均绑定。坐标显示、原场景渲染和受控规划共用快照校验；明确排除的三个历史仿真观测文件保留原哈希但不参与本次几何运行，不作为验收门槛。

原生 MTC/Pilz（必要时 OMPL）生成四段：接近、LIN 接触、带载 LIN 脱离、带载单独规划返回。脱离按可见面外法向、箱体尺寸与支撑清除要求确定，并验证完整箱体。带载固定 `T_flange_box`，终点严格为冻结的 `q_start`，保留 2 kg 箱体与估计惯量。原始非均匀时间轴经既有 quintic q/v/a 重采样与极值检查，最终发送点及更密碰撞路径再次验证。非单调或重复时间点等非法原生输出会拒绝，不改数组下标拼造时间轴。

## 执行、停止与记录

同一机器人 owner 管理连接、200 Hz 透传、实际 UDP 反馈及独立软件停止通道；GUI/推理/日志均不进入发送循环。既有空载执行阈值继续生效：起点 0.01°，跟踪超差 3°持续 0.1 s，到位 0.5°且低速 0.5°/s 持续 0.4 s，机器人反馈超时 0.1 s，发送允许抖动 2 ms / 最大间隔 9 ms。接触建立真空前另做实际 FK 密封核验；最终返回与释放时再要求关节与 `q_start` 差 ≤0.01°。不因 SDK 成功或最后一点发完报告成功。

```text
IDLE → OBSERVING → TARGET_READY → PLANNING → PLAN_READY
  → APPROACH → CONTACT → ESTABLISH_VACUUM → ATTACHMENT_CONFIRMED
  → DETACH → RETURN_LOADED → RELEASE → COMPLETE
故障或停止 → STOPPING → FAILED / CANCELLED / STOP_UNCONFIRMED
```

停止请求立即禁止继续阀门发送，并通过独立通道请求机器人停止。停止与故障不自动释放、不自动返回；接近阶段停止也不执行后续阶段。已经明确进入释放阶段时，已发出的释放不能撤回，后续异常仍保留“不确定是否持箱”的锁定。通信丢失显示 `STOP_UNCONFIRMED`，禁止重连后自动续跑。持箱或停止未确认时关闭窗口会保留连接并提示现场处置；只读入口退出行为不变。

断开、重连、相机停启立即清除相关旧样本、关节历史和检测；在途结果按连接代际丢弃，新连接必须收到新有效样本才显示 LIVE_VALID。故障码及反馈年龄显示在任务区。

记录在 `outputs/single_carton/<task_id>/`：

- `observation.json/.npz`：同组 RGB、原始深度、内参、scale、序列号、帧号、时钟、`q_capture`、`q_start`。
- `segmentation.npz`、`geometry.json`：真实推理掩码、几何结果及拒绝原因。
- 各候选子目录：完整原生请求/结果和最终发送路径检查，包括失败候选。
- `complete-plan.json`：完整四段计划、几何和配置绑定；不可通过修改标记导入执行。
- `pre_execute_observation.json/.npz`：执行前新的实测观测，旧观测时间戳原样保留。
- `events.jsonl`：阶段耗时、阀门字节与时间、各通道压力原接收时间、实际关节、跟踪误差、运动子任务 ID、停止及结果。
- `motion/<motion_task_id>/`：每段输入和最终发送轨迹、控制器信息、SDK 返回码、发送时刻与抖动、实际反馈和停止结果。
- `result.json`：最终状态；只有实机分支所有反馈条件通过才标记 `PHYSICAL_CYCLE_COMPLETE`。假设备测试明确另标，不进入此结论。

软件验证结果及合成记录示例见 `docs/evidence/eco65_single_carton_20261111/`。现场首箱的真实观测、规划、吸附、脱离、带载返回、释放以及带箱停止均 **NOT_RUN**，没有创建实机成功报告。

## 本轮软件验证与复现

最终定向测试：**224 passed，0 failed，0 skipped**。覆盖单箱几何、同帧关联、真实来源隔离、假设备闭环、压力/反馈异常、取消/重连、界面及原只读回归。Tk 测试夹具在主线程销毁变量；没有放宽 IPC 超时或删除安全断言。

```bash
xvfb-run -a .venv/bin/pytest -q \
  tests/test_single_carton_*.py \
  tests/test_controlled_status_ui.py tests/test_controlled_read_ui.py \
  tests/test_controlled_execution.py tests/test_controlled_payload_setting.py \
  tests/test_controlled_goal_planning.py tests/test_workbench_readonly.py \
  tests/test_workbench_coordinates.py tests/test_workbench_installation.py
```

原生 C++ 接触/带载支撑检查在 optimized 和 reference 两种路径均通过；原生组件检查在完整静态场景中生成并校验 491 个低速发送点。该组件检查仅使用固定离线姿态和合成目标，**不是完整四段实机任务**。本轮曾保留一次 Pilz 重复终点时间导致的 `INVALID_TIME_AXIS` 拒绝记录，没有修改时间轴掩盖问题。

```bash
source /opt/ros/humble/setup.bash
cmake --build build/moveit --target m710_moveit_worker m710_clearance_test -j2
cmake --install build/moveit --prefix "$PWD/install/moveit"
# 以下仅规划/碰撞检查，不连接 SDK，不注册可执行任务：
.venv/bin/python tools/check_eco65_single_carton_native.py
```

GroundingDINO 与 SAM 的实际本地权重均完成加载和前向计算检查；输入为合成图像，DINO 返回零目标，SAM 输出尺寸为 `1×3×240×320`。这证明加载器和推理调用可运行，不代表现场识别质量已验收。设备未连接的 Ubuntu 界面也已通过启动、预览入口和初始禁用状态检查。

`docs/evidence/eco65_single_carton_20261111/software_report.json` 保存结果、范围和关键哈希；同目录 `fake_cycle_record_example.json` 是假设备记录示例，明确标为不可执行和实机 NOT_RUN。真实记录由现场操作者按上述流程启动后产生，不能拿该示例冒充现场证据。

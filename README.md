# Trailer Unloading Geometric Simulator v0.5 feasibility core

> 当前分支以 **M-710iD/70 V3 单箱完整几何可行性恢复**为核心目标。V3 已修正
> 坐标、SO(3)、严格 IK/FK 残差、刚体附着、完整碰撞路径、负载和时间参数化；
> 原始 104 个任务完整几何成功为 **0/104**，连续场景为 **0/129**。
> 冻结 V2 的 50% 完整覆盖率只保留为历史证据，不再作为有效 baseline。
> 当前实施顺序见
> [V3 可行性恢复开发优先级](docs/development_priorities_m710id70_v3_recovery.md)，
> 已完成验证证据见
> [V3 技术验证报告](docs/validation/technical_qualification_report_m710id70_v3.md)
> 和[分阶段记录](docs/validation/m710id70_v3_stages.md)。

```powershell
# 复现不可变 V2，并输出实际调用参数及逐任务追踪
.venv\Scripts\python.exe tools/reproduce_m710_v2.py
# 当前 V3 P0：grasp-only 与固定传送带原始 104 任务
.venv\Scripts\python.exe tools/run_m710id70_v3.py --phase all --workers 4
# 独立 FK/Jacobian、负载、惯量与可执行轨迹数值证据
.venv\Scripts\python.exe tools/audit_m710_v3.py
.venv\Scripts\python.exe -m pytest -q
```

V3 配置为 `configs/validation/m710id70_v3.yaml`，允许 `extends` 覆盖并
记录每个参数来源，拼错或未定义的字段会报错。证据写入
`outputs/m710id70_v3/`；其中 `v3/effective_config.json` 是完整生效参数，
`v3/tasks/*.json` 保留每次候选、碰撞失败点、实际姿态及路径。
`--workers 1` 可串行复现，任务种子不随并行度改变。

## 当前开发优先级

当前工作先恢复可解释的单箱完整几何可行性，不以旧成功路径、路径长度或周期为
优化目标：

1. **P0-1 contact-aware separation**：只对已登记的目标箱—邻箱初始近接建立
   不恶化、趋向分离的临时语义；真实穿透、其他碰撞对及全局安全 margin 不放宽。
2. **P0-2 task-set grasp IK**：在有效箱面上搜索面内位置、法向 roll、小范围姿态、
   多确定性种子及合法腕部构型；每个结果仍执行 V3 全部严格验证。
3. **P0-3 escape-path extraction**：保留 620 mm 纯直退基线，但在受限分离的每个
   小步寻找安全斜移、抬升或转向路径，并区分四种脱垛/释放距离指标。
4. **P0-4 support release**：由 `SupportRelationGraph` 决定是否插入标准
   `GRASP -> SUPPORT_RELEASE -> EXTRACTION -> TRANSIT -> PLACE` 阶段。
5. **P1 暂缓**：原始任务产生稳定非零完整成功集合之前，不继续 dynamic
   conveyor A/B、base Z 或 lift range 优化。

不得通过关闭碰撞、降低安全余量、降低 IK/FK 数值标准、缩小场景、修改分母或
改写 FANUC 负载曲线来提高成功率。负载资格继续独立记录，不能短路几何探索。

当前开发实现已产生一个**尚未替换冻结全量基线**的原始任务 witness：
`grid_022` 在固定传送带、top/90°、面内偏移 25 mm 下完成严格完整几何链路。
其 pure straight clearance 为 0.320 m，局部侧向 escape 在约 0.010002 m 后恢复
普通 collision margin。P0-1 探针中的原 24 个初始净空失败均通过新的初始门，
但该门通过本身不计作任务成功。P0-2 grasp-only 全 104 扫描把严格有效 grasp
从固定 TCP 的 24 个提高到 52 个：原 44 个 `GRASP_CONSTRAINT_FAILED` 恢复
27 个，原 36 个 `NO_IK` 恢复 1 个；这同样不等于完整任务成功。当前固定传送带
的原始 104-task P0 复跑已得到
**10/104** 完整几何成功（grasp 52、extraction 26）；10 个均为 top grasp，
其中 5 个执行正式 `SUPPORT_RELEASE`。该开发结果不改写冻结 V3 历史报告，连续
129 场景仍待复跑，且不代表负载或动力学资格通过。

2026-09-08 的[可行性核心加固记录](docs/validation/m710id70_v3_hardening_round.md)
补齐了模型资产内容指纹/缓存失效、配置数值校验、实际接触终点闭合和跨 station/
方向的确定性 escape 调度。相同 104 任务的新鲜前后复跑保持 10/104，新增成功 0、
丢失成功 0、共同成功 10；这是正确性加固结果，不是放宽约束后的成功率优化。

面向厢式货车自动卸货的六轴工业机器人第一层几何仿真与工程资格评估工具。项目重点是确定性、可测试、可审计的 CPU 几何/运动学主链路，不依赖 ROS 2、Isaac Sim 或 GPU；PyBullet、Pinocchio 和 Isaac Sim 均位于可选适配层。

v0.4 的主评估对象是 **FANUC M-710iD/70 + 20 kg 三分区吸具 + 42.5 kg 箱体**。历史 FANUC M-20iD/35 与 KUKA KR 50 R2500 配置继续保留，但不能混用不同机器人的报告结论。

世界坐标固定为：`+X` 指向车厢内部、`+Y` 向左、`+Z` 向上。全部配置与计算使用 SI 单位：米、弧度、秒、千克。

## 冻结的 v0.4 / V2 功能摘要

- 新增可追溯的 FANUC M-710iD/70 URDF/SRDF、关节限制、厂商负载证据和独立工具配置。
- 建立 `BoxNeighborhoodState`：使用箱体 OBB 投影重叠和表面间隙识别左、右、上方邻箱、底部支撑与暴露抓取面。
- 删除固定 250 mm 直退判据；按箱体尺寸、抓取面和当前邻接状态实时求解 `minimum_clearance_extraction_distance`。
- 同时生成 front、left、right、top 抓取/脱垛候选，并按带载路径、脱垛距离、关节运动、碰撞风险、奇异性和关节裕量综合评分。
- 将 L 形传送带建模为独立外部机构，而不是底盘刚性 link 或机器人负载；支持 `conveyor_extension` 和 `conveyor_z` 联合预定位及碰撞拒绝。
- 将几何运动能力与 FANUC 负载—质心资格分开输出：`GEOMETRICALLY_REACHABLE`、`PAYLOAD_QUALIFIED`、`QUALIFIED_TASK`。
- 新增动态连续卸货选箱、四层任务热力图、失败原因空间统计、负载包络和 `base_x × base_z` 安装位扫描。

完整变更与已知边界见 [`docs/releases/V0.4.md`](docs/releases/V0.4.md)。

## 冻结的 v0.4 / V2 历史结论

以下数字来自 v0.4 验收脚本的 0.30 m 离散横截面扫描，不是连续空间证明，也不替代 FANUC 官方负载软件或真机安全认证：

| 指标 | v0.4 结果 |
| --- | ---: |
| 抓取覆盖率 | 78.846% |
| 几何脱垛覆盖率 | 63.462% |
| 传送带交接覆盖率 | 50.000% |
| 完整几何任务覆盖率 | 50.000% |
| 负载资格覆盖率 | 0.000% |
| 双侧受限箱正面脱垛距离 P50 / P95 | 0.620 / 0.620 m |

旧版固定直退下的完整任务覆盖率为 25.609%；v0.4 新定义下为 50.000%，增加 24.391 个百分点。两版任务模型不同，旧的不可达面积和连续场景结论不再作为 v0.4 结论。

四组动态连续场景的几何卸出箱数为：

- `regular`: 0 / 40；
- `random_seed_71071`: 2 / 27；
- `random_seed_71072`: 0 / 32；
- `random_seed_71073`: 0 / 30。

当前第一项独立硬失败是 `PAYLOAD_CG_FAILED`。20 kg 工具与 42.5 kg 箱体总质量为 62.5 kg，但组合质心约 0.422 m，超出当前保守 FANUC 70 kg 曲线。即使 TCP 缩短到 0.10 m，当前 600 mm 深箱仍未通过；按现有假设反算 TCP 上限约 0.037 m。因此应优先调整整体有效负载中心或机器人负载包络，而不是只减轻吸具。

运动规划中的主要失败为 `NO_IK`。完整腕部动力学因缺少连杆质量、质心、惯量和驱动允许转矩继续标记 `NOT_EVALUATED`，项目没有填入虚构数据。

## 任务管线

```text
车厢、箱堆、机器人、吸具和独立传送带配置
  -> OBB 邻接图与当前可移除箱识别
  -> 多抓取面候选与吸附面积检查
  -> 确定性多起点 IK、限位、奇异性和碰撞检查
  -> 几何最小脱垛距离与笛卡尔扫掠验证
  -> 传送带伸缩/升降联合预定位
  -> 带载清堆、交接支撑和真空释放门控
  -> 几何任务结果与负载资格分层统计
```

核心包使用 OBB、胶囊体、URDF 运动链和确定性随机种子。解析球面工作空间只可作为快速排除条件，不能作为任务覆盖结论。

## 安装

需要 Python 3.10 或更高版本。

```bash
git clone --recurse-submodules https://github.com/QGG716/pick-and-place-simulation.git
cd pick-and-place-simulation
python -m venv .venv
```

激活环境并安装核心/测试依赖：

```bash
# Linux / macOS
source .venv/bin/activate

# Windows PowerShell
.venv\Scripts\Activate.ps1

python -m pip install -e ".[dev]"
```

需要 PyBullet 回放时：

```bash
python -m pip install -e ".[dev,viz]"
```

也可以使用项目锁文件：

```bash
uv sync --extra dev
```

## 快速开始

### CPU 核心演示

```bash
python -m unloading_sim.demo --config config/demo.yaml
```

默认演示保持在普通笔记本 CPU 五秒级，结果写入被 Git 忽略的 `outputs/demo/`。

### 冻结的 M-710iD/70 v0.4 / V2 验收复现

```bash
python tools/reproduce_m710_v2.py \
  --output-dir outputs/m710id70_v2
```

该命令会运行安装位扫描、两种箱体朝向的 2.3 m × 2.7 m 横截面任务扫描、负载包络以及四组连续卸货场景。运行时间取决于 CPU；缩小网格步长会显著增加 IK 与碰撞查询数量。

主要输出包括：

- `technical_qualification_report_m710id70_v2.md`；
- `task_reachability.csv`；
- `failure_reason_statistics.csv`；
- `payload_envelope.csv`；
- `base_x_z_scan.csv` 与二维热力图；
- `continuous_unloading_results.csv`；
- `grasp_strategy_statistics.csv`；
- grasp、extraction、conveyor handoff、full task 四层热力图。

当前 `tools/run_m710id70_acceptance.py` 已转发到严格 V3 验收入口；
V2 历史算法须通过上面的冻结提交复现命令执行，避免导入新数学实现后
把不同版本的结果混在一起。V3 分阶段运行选项为 `--phase`；当前 `all` 只运行
P0 grasp-only 与固定传送带 104 任务。`small`、`grid`、`continuous` 和 `lift`
保留为显式阶段，不会由 `all` 启动动态传送带 A/B 或升降轴工作。

### 历史 M-20iD/35 工具链

```bash
python tools/run_qualification_suite.py

python -m unloading_sim.online_unload \
  --config config/fanuc_m20id35.yaml \
  --max-picks 8 \
  --top-layer-only \
  --output-dir outputs/fanuc_m20id35/top_layer
```

M-20iD/35 的 v0.3 报告仅是历史基线，不能外推到 M-710iD/70。

## 配置入口

| 文件 | 用途 |
| --- | --- |
| `config/fanuc_m710id_70.yaml` | v0.4 运行、规划、传送带和确定性种子 |
| `configs/robots/fanuc_m710id_70.yaml` | 机器人身份、限制与证据状态 |
| `configs/tools/unloading_gripper_20kg.yaml` | 20 kg 工具质量属性、TCP 和碰撞尺寸 |
| `configs/trucks/2p3x2p7_m710id70.yaml` | 2.3 m × 2.7 m 车厢配置 |
| `configs/qualification/fanuc_m710id_70_42p5kg.yaml` | 42.5 kg 箱体资格案例 |
| `configs/vendor/fanuc_m710id_70_load_evidence.yaml` | FANUC 负载/腕部限制证据映射 |

传送带与底盘的相对几何、行程和净空均位于配置中，不写死在碰撞查询内。传送带质量不计入底盘或机械臂负载，但其所有构件都作为外部碰撞体参与规划。

## 项目结构

```text
src/unloading_sim/
  geometry.py             OBB、胶囊体、SO(3) 几何工具
  robot.py                DH/URDF 机器人、FK、Jacobian 和碰撞体
  scene.py                场景模型与递归 YAML 配置
  ik.py                   阻尼最小二乘多起点 IK
  planner.py              RRT-Connect、捷径和平滑
  grasp.py                通用抓取与放置规划
  depalletizing.py        v0.4 邻接拓扑、动态脱垛、传送带和候选评分
  fanuc_m710id70.py       M-710iD/70 运动学、扫掠碰撞和负载接口
  robot_load/             负载、空间惯量与厂商证据适配

config/                   运行时配置
configs/                  机器人、工具、车厢、资格和证据配置
tools/                    可复现分析与验收命令
studies/                  较重的专项研究
scripts/                  PyBullet / Isaac Sim 导出与回放适配器
assets/                   机器人、吸具和材质资源及来源说明
tests/                    单元、回归和后端边界测试
docs/                     架构、服务器与发布说明
```

`geometry.py`、`robot.py`、`scene.py`、`ik.py`、`planner.py`、`grasp.py` 和 `depalletizing.py` 保持独立可测试。ROS 2、Isaac Sim、PyBullet 或 GPU 功能不得进入核心依赖。

## 测试

```bash
pytest -q
```

v0.4 发布检查结果为 `189 passed, 1 deselected`。默认配置排除标记为 `simulation`、`slow`、`pybullet` 和 `isaac` 的重型测试。

Windows 受限环境若无法访问用户临时目录，可显式指定工程内临时目录：

```powershell
python -m pytest -q --basetemp .tmp/pytest-v0.4
```

新增回归覆盖双侧受限最小直抽、左右单侧开放的对称侧吸、四面候选评分、不同箱体尺寸的不同脱垛距离、传送带 20 mm 表面净空、动态 Z 上下限、传送带碰撞拒绝以及交接完成后的真空释放。

## 证据语义与安全边界

- `PASS`：在明确声明的输入、模型和离散检查下通过。
- `FAIL_*`：已发现具体硬约束失败。
- `NOT_EVALUATED`：缺少数据或本流程未执行，不能解释为通过。
- `NOT_QUALIFIED`：当前证据不足以形成完整工程或厂商资格结论。

当前没有完整模拟或认证纸箱柔性/坍塌、真空密封与剥离、真实控制器插补、完整关节动力学、安全 PLC、工业节拍和现场风险。进入真机前必须使用厂商精确模型、实测工具质量属性、FANUC 负载设定软件或 ROBOGUIDE、控制器日志及吸具试验重新验证全部轨迹。

机器人模型证据边界见 [`assets/robots/fanuc_m710id_70/SOURCE.md`](assets/robots/fanuc_m710id_70/SOURCE.md)。Isaac Sim 服务器和导出说明见 [`docs/isaacsim_server.md`](docs/isaacsim_server.md)，后端边界见 [`docs/digital_twin_backend.md`](docs/digital_twin_backend.md)。

## 版本历史

- v0.4：M-710iD/70、动态 L 形传送带、拓扑脱垛与分层任务资格。
- v0.3.1：机器人/工具身份与缓存基线清理。
- v0.3：FANUC M-20iD/35 负载感知资格评估。
- v0.2：可审计数字孪生与回放链路。

发布说明位于 [`docs/releases/`](docs/releases/)。Python 包版本为 `0.4.0`。

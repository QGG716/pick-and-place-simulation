# M-710iD/70 首箱可视化与真实输送迭代

日期：2026-09-12

分支：`feat/v0.5-feasibility-core`

运行基线：`648e177a03d2a5a5d8a2f11e9141c57763325805` 加本轮工作树；最终提交 SHA 由交付回复及远端分支共同确认。

## 交付结论

- 保留 `contact_round33_full_colored`，新结果写入独立目录 `contact_round34_industrial_closed_conveyor_final_v3`。
- 首箱仍为 `carton_l07_c02`，正面吸取；规划器在有限 roll/pitch/yaw 换面候选中选择保持原箱姿态，接收面为 `conveyor_longitudinal`，完整足迹由横、纵两带联合支撑。
- 真实抓取、脱垛、带载搬运、支撑、释放、撤离和输送：`PASS`；`physical_cycle_completed=true`，无运行时停止原因。
- 纵向传送带在实际释放、箱体独立、杯面脱离及工具离开 0.20 m 短时输送扫掠包络后启动；箱体由 PhysX 表面速度和摩擦输送，未直接赋速或改位姿。
- 实际带速命令 `0.30 m/s`，启动时间 `123.433333 s`，停止事件进度 `0.600789 m`（输送审计距离 `0.599485 m`），实测投影速度 `0.300024 m/s`，停止原因 `SAFE_IN_BELT_WAIT_POSITION_REACHED`。
- 几何/仿真执行资格与机器资格继续分开记录。最终总资格为 `FAIL`；唯一未通过项是 `joint_efforts_within_limit`。峰值投影关节力矩为 `[149.224, 2005.884, 1307.575, 319.390, 364.992, 105.589] N·m`。关节位置均在官方限位内，且没有修改 FANUC 力矩曲线或真空能力假设来制造通过。

## 本轮实现

1. 放置搜索新增离散的 ±90° roll/pitch 与既有 yaw 候选。每个候选按真实三维姿态计算朝下支撑面、中心高度、完整足迹联合覆盖、附着反解 TCP、路径和撤离净空；没有写死 J5/J6 动作。
2. 脱垛自由空间判据仍为 20.2 mm，进入后允许 0.2 mm 数值回摆但不得丢失 20.0 mm 工程余量。规划端仅增加 3 mm 运行时余量，未改变接受阈值。
3. 传送带启动联锁消费实时 PhysX J6 与箱体位姿，并逐个变换 58 个刚性吸具代理和 72 个杯碰撞体；不再使用可能停留在旧关节姿态的 USD 默认时间包围盒，也不把离散碰撞体合并为填满间隙的大包围盒。
4. 纵、横两带分别累计实际行程相位；只有物理表面启用时，深色带面的接缝和端辊才运动，停止后相位冻结。首箱落在纵带，因此本次只启动纵带（−X，厢内到厢外）；横带保留 −Y（画面左到右）方向但不为演示强制空转。
5. 传送带改为深色橡胶带面、灰色金属机架与端辊，底盘保持黑色，颜色和材质可区分。HUD 使用动态测量的五行状态和高不透明深色底板。
6. CPU 与 Isaac 共用五面开发车厢边界：开口 `X=-3.2 m`、封闭端壁 `X=+3.2 m`、长 `6.4 m`、高 `2.7 m`、壁厚 `0.05 m`。这些均明确标记为未实测开发假设，不是确认尺寸。

## 最小验证

- 服务器 CPU 重新生成运动预检与回放 bundle：`READY`，40 个动态箱体，preflight fingerprint `4149a1e40f0b74f7fb8c106d63b1e7727b2012f22fa2c15f8a022ea445759ff3`。
- 隔离服务器 CPU 重点回归：16 项通过，覆盖换面支撑、两带联合覆盖、实时工具扫掠联锁、严格自由空间边界、运行时余量和封闭车厢序列化。
- `py_compile`：本轮 8 个修改模块通过。
- `git diff --check`：通过。
- Isaac 完整回放：`126.433333 s` 仿真时间、`3793` 帧、`1603.543 s` 墙钟时间；支撑法向间隙 `0.000014465 m`、完整足迹覆盖率 `1.0`，释放中心误差 `0.000165977 m`。
- 按本轮范围未运行 104/129 任务、40 箱清空或全量 `pytest`。

## 复现

服务器 CPU 预检与 bundle：

```bash
PYTHONPATH="$ROUND/repo/src" "$CPU_VENV/bin/python" \
  tools/prepare_m710id70_dynamic_execution.py \
  --config configs/simulation/m710id70_dynamic_execution_v1.yaml \
  --motion-result "$ROUND/outputs/single_carton_motion_audit_v4.json" \
  --output "$ROUND/outputs/preflight_v5.json"

PYTHONPATH="$ROUND/repo/src" "$CPU_VENV/bin/python" \
  scripts/export_isaac_fanuc_replay.py \
  --preflight "$ROUND/outputs/preflight_v5.json" \
  --output "$ROUND/outputs/replay_bundle_v5.json"
```

Isaac 6.0.1 回放（`$OFFICIAL` 指向已验证的官方 USD 归档）：

```bash
OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH="$ROUND/repo/src" \
  "$ISAAC_VENV/bin/python" scripts/isaacsim_fanuc_replay.py \
  --bundle "$ROUND/outputs/replay_bundle_v5.json" \
  --project-root "$ROUND/repo" \
  --usd-directory "$ROUND/usd-final" \
  --reuse-usd-entrypoint "$OFFICIAL/isaac_usd/m710id_70_official_8/m710id_70_official.usda" \
  --reuse-usd-run-evidence "$OFFICIAL/isaac_outputs/initialization_render_sync_logged_final/run_status.json" \
  --reuse-usd-source-contract "$OFFICIAL/repo/outputs/m710_official_dynamics_20260910_final/initialization_contract.json" \
  --output "$ROUND/outputs/contact_round34_industrial_closed_conveyor_final_v3" \
  --record-video --video-preview-speed 4
```

录像为 1920×1080、30 FPS、1× 物理时间；`replay_4x.mp4` 仅是同一帧流的观看加速副本。PNG 关键帧包括总览、接触、脱垛、搬运、放置、撤离和输送。

## 轻量证据

- `replay.mp4`：104,533,771 bytes，SHA-256 `0aacabfc9781ce2038f1886ec54ef2a45d8567b7c2bb0c71896686646594013e`。
- `result.json`：SHA-256 `2ebaef7e7320aef08d6cdd53f4af2290b29f6bc24ac8d35bce0952079d91a8e`（由完成状态文件记录）。
- `replay_bundle_v5.json`：SHA-256 `669952099c1f3a89ce823ea8b7643d33b56cd73968196850bc882168b4bae46c`。
- `preflight_v5.json`：SHA-256 `2799c1d018d3064a959976e067b285e41f5909a02b1ca77b04f8cc05f77200a0`。
- `phase_conveyor_transport.png`：SHA-256 `ef9cd3f14ec831870be3e91070744f0c3dba7a53b824ac5a55615119e4dbe394`。
- 启动联锁在唯一一次评估中读到：约束已解除、目标箱已独立、实时工具到 0.20 m 输送扫掠的最小净空 `0.0299697 m ≥ 0.0202 m`。

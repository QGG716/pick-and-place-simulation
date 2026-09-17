# ECO65-B 桌面卸货布局 v1：DESIGN_CANDIDATE

2026-09-17，分支 `feat/v0.6-eco65-desktop`，从本地/远端一致的 `985e6d3338eb75d10d61b8e8c037e5d6edbf4a30` 继续。本轮交付完整布局与有限姿态检查，停止在用户确认排布和尺寸方向之前。**没有执行整堆、完整抓放路径、输送动力学或真机动作。**

旧“单箱＋静态接收台”仅保留为基础回归；配置、原 STEP、网格、TCP/装配及上一轮所有输出未覆盖。未发现需继承的后续提交或未提交修改；已有规划/执行代码没有重写。共享几何构造仅扩展为支持多箱列表，其余布局策略位于独立模块。

## 候选布局与来源映射

权威来源为 `9db77cb8a51e9bf1821c6e84e90a7632fbce6b26:configs/workcells/m710id70_unloading_layout_v1.yaml`，同时读取指定的两份辅助报告。未采用历史代理基座 X=−1.41 m；原官方模型安装原点由配置算出 X=−1.325、Y=0.35、Z=0.6 m。

公开候选配置：[eco65_desktop_unloading_layout_v1.yaml](../configs/workcells/eco65_desktop_unloading_layout_v1.yaml)。独立来源及变化记录：[eco65_desktop_layout_sources.json](../configs/integration/eco65_desktop_layout_sources.json)。原有三来源源码锁与引用不变，不整体合并分支。

| 项目 | 固定原布局 | 桌面候选 |
|---|---|---|
| 世界轴 | +X 入厢、+Y 左、+Z 上 | 保持；原点为箱垛前面中心在桌面的投影 |
| 装配坐标 | A 随底盘，世界平移 [−1.95,0.35,0] m | 固定安装坐标与 W 重合，显式单位 SE(3)；不保留底盘自由度 |
| 机器人/吸具 | FANUC 与原工具 | 官方 ECO65-B 与已解析真实吸具 **1:1**，无缩放；原完整 TCP 和假设转接架不变 |
| 基座原点 m | [−1.325,0.35,0.6] | **[−0.445,0.145,0.024]** |
| 横带 XY 范围 m | X[−0.9,−0.2]，Y[−0.4,1.1] | **X[−0.36,−0.08]，Y[−0.125,0.415]** |
| 纵带 XY 范围 m | X[−3.0,−0.2]，Y[−1.1,−0.4] | **X[−1.05,−0.08]，Y[−0.415,−0.135]** |
| 横带/纵带有效宽度 | 700 / 700 mm | **280 / 280 mm**；按箱体及支撑余量选取 |
| 横带/纵带长度 | 1500 / 2800 mm | **540 / 970 mm** |
| 箱体 X/Y/Z | 600/400/300 mm | **240/200/160 mm**，0.30 kg 为原有限仿真假设 |
| 箱堆 | 1 排×5 列×8 层 | **1 排×3 列×3 层，共 9 箱**；列缝 10 mm，层缝 0 |
| 厢体净宽/净高 | 2300/2700 mm | **900/800 mm** |
| 厢口/后壁内面 X | −3.2/+3.2 m | **−1.10/+0.30 m**，作业段内长 1400 mm |

约 1/3 仅用于平面初值；没有整体 scale。纵带位于 −Y 侧，横带在机器人和箱堆之间，两带近箱端同为 X=−0.08 m。机器人与输送线均在完整厢体作业段中。箱堆仍在 X≥0，前方为开放厢口，出口沿 −X。

900 mm 内宽取初始候选上限：280 mm 纵带＋10 mm 转接缝＋540 mm 横带，占宽 830 mm，两侧各留 35 mm。箱体没有缩小：CAD 全部 12 条轮廓在顶面和正面均满足 3 mm 边距要求；朝右壁的侧面也兼容。气路仍未知，没有启用独立吸盘或缩减吸盘数量。

原 6.4 m 厢长是开发场景假设；本方案只保留工作段，箱体后端 X=0.24 m 到后壁留 60 mm。它不是只罩箱堆的小盒，机器人、双带大部与箱堆都处于同一有顶厢体内。尺寸调整原因保存在来源映射和私有 `dimension_mapping.json`。

## 高度、支撑与最小占地

桌面定义为 Z=0，未假定实际办公桌高度。连续厢底板厚 12 mm、上表面 Z=12 mm；固定机器人安装板另厚 12 mm，安装面 Z=24 mm。没有车轮、底盘、移动轴、升降轴或大高台。

输送带表面 Z=130 mm：带面层 4 mm、机身 40 mm，厢内支脚高 74 mm，均落在厢底板上；出口承接台支脚直接落桌面，高 86 mm。简化机身、带面和支撑是不同部件，有限厚度，没有穿入桌面或在转角处相互穿透。端部保留 20 mm 接收缓冲区域；它不是不存在支撑的洞，也不是已完成设计的真实滚筒。

三层箱心 Z=92/252/412 mm，垛顶 Z=492 mm；9 个稳定 ID 为 `carton_r00_l00_c00` 至 `carton_r00_l02_c02`。底层直接由连续厢底支撑，其余由同列下层箱支撑。初始只有最上层 3 箱不支撑其他箱。邻箱和下层箱在所有检查及图片中保留。

左右墙、后端墙、底板、顶板均有有限实体范围。顶板内面 Z=812 mm，外顶面 Z=824 mm；厢口开放。透明显示仅便于观察，**右墙和顶板仍参与碰撞**，没有拆顶结果混入。

含出口承接台的设备最小静态占地：**1782×924 mm**，范围 X[−1.47,0.312]、Y[−0.462,0.462] m。建议各边预留 100 mm，台面约 **1982×1124 mm**；另建议厢口前方留 600 mm 操作空间。这些是候选安装预留值，不是加工尺寸、结构承载确认或已验证的动态扫掠范围。

桌面容纳状态为 **PENDING_MEASURED_DESK_DIMENSIONS**。1200×900 mm 旧仿真桌面不是用户实物尺寸，而且不足以容纳本候选的静态外廓。尚需用户提供办公桌可用长宽、边缘/墙面障碍和允许承载，再确认实际排布。

## 两条路线、转接与出口

独立区域为 `conveyor_transverse`、`conveyor_longitudinal`、`transfer`、`outlet`。`regions.json` 由同一快照计算：包含物理支撑面、有效接收范围、端部缓冲、机身与支脚 ID、工艺归属和初始空占用关系。所有带面初始均为空；箱堆和每个放置候选另外记录实际支撑/占用关系。

- **路线 A**：横带接收中心 [−0.22,0.145] m，沿 −Y 到纵带中心线 Y=−0.275 m，再沿 −X 出厢。
- **路线 B**：纵带直接接收中心 [−0.52,−0.275] m，再沿 −X 出厢。

两带交界 Y[−0.135,−0.125] m 的 10 mm 缝由独立、共面 6 mm 厚桥板覆盖，工艺归纵带。桥板为工程结构假设，未验证 90°转接动作、摩擦、驱动或安装强度。接收姿态保留可选项，不强制只采用一种吸取面。

纵带端部 X=−1.05 m；100 mm 出口桥板延伸至 X=−1.15 m，接上 320×320 mm 静态承接台。最终箱心 [−1.31,−0.275] m，整箱已越过 X=−1.10 m 厢口且底面有完整几何支撑。没有越过空白区域、删除箱体或直接消失的送出机制，本轮也没有实际驱动箱体沿路线移动。

以箱底矩形与实际支撑面的并集计算，路线 A 的 304 个位置、路线 B 的 159 个位置（约 5 mm 间距）最小覆盖率均为 100%（浮点误差约 1e-15）。这只是**有限位置的几何支撑检查**，不是实体传送/转接通过。负例测试移除转接桥板后覆盖率降到 95%；移除出口桥板后，厢口附近覆盖率不足 60%，不会因工艺归属而自动判为有支撑。

## 初始状态与有限姿态结果

原 ECO65 单箱 HOME 关节值在新安装位置、全部厢体/传送线/9 箱环境中重新检查通过。它不是继承 FANUC HOME，也没有向真机下发。

保持上一轮几何策略：2 mm 余量，加每体 0.2 mm 网格储备，一般碰撞阈值 2.4 mm；接触容差 0.4 mm。官方逐连杆模型、真实吸具全部刚性附件、假设转接架、墙/顶/地板、机身和箱体均参与。仅明确的安装界面、几何贴合后的目标唇口/箱面、箱底/实际支撑顶面允许必要接触；没有整手腕、整工具或邻箱豁免。

固定种子 **6517**；每个姿态最多 16 个初值、每次 220 次 IK 迭代、每姿态上限 20 秒。只检查下面 9 个代表姿态，没有扫描所有箱面或搜索任何完整路径。实际姿态检查累计约 12.53 秒。

| 代表姿态 | IK、限位及完整几何碰撞 | 当前可移除/支撑结论 |
|---|---|---|
| 上层右列 C0 正面抓取 | POSE_VALID | 没有支撑上层箱；完整移除路径未检查 |
| 上层中列 C1 正面抓取 | POSE_VALID | 同上 |
| 上层左列 C2 正面抓取 | POSE_VALID | 同上 |
| 上层中箱顶吸 | IK_NOT_FOUND_WITHIN_BUDGET，16 初值未找到解 | 不是不可达的数学证明，不是碰撞失败 |
| 中层中箱正面抓取 | POSE_VALID | **SUPPORT_DEPENDENCY_BLOCKED**：仍支撑上层箱，不算当前可卸 |
| 横带顶吸放置 | POSE_VALID | 箱底由横带支撑 |
| 横带正面侧吸放置 | IK_FOUND_COLLISION | 找到 1 个 IK 解，但 link_1/link_5 自碰撞，约 −25.41 mm |
| 纵带顶吸放置 | POSE_VALID | 箱底由纵带支撑 |
| 纵带吸具工作面朝右壁放置 | POSE_VALID | 箱底由纵带支撑 |

总计 **7/9 姿态通过**，其中 1 个仍受上层支撑依赖限制；不要将其写成 7 次卸货成功。通过姿态的最大位置残差约 21.54 μm、方向残差约 5.56e-5 rad。放置图是同一目标箱在假设接收位置的独立姿态检查，其他 8 箱仍保留，不是已执行的搬运或释放。

**本轮完整卸货执行数为 0，完整路径与动力学均为 NOT_EVALUATED。** 不用有限 IK 结果替代接近、抽取、带载运动及支撑释放的后续验证。

## 一份快照、实际模型图片与导出

新结果根目录：

`outputs/eco65_desktop_layout/layout_v1_20260917_candidate02/`

快照指纹：`61d6833610f836d3cf41f251a572c4f07e096f9ef5f367eda649be130875facf`。

`scene_snapshot.json` 冻结公开候选配置、私有工具、真实资产哈希和完整场景；模型构造、碰撞、投影图、三维图和后端导出均消费它。尺寸标注从配置计算，二维机器人/工具轮廓由实际模型投影，不使用生成式图片或通用吸具。侧视图叠加投影所有 Y 位置，二维轮廓重叠不代表三维实体穿透，应结合俯视图查看。

**以下图纸和完整快照只在本地，未上传 GitHub：**

- [原布局与桌面对照](../outputs/eco65_desktop_layout/layout_v1_20260917_candidate02/figures/01_reference_mapping.png)
- [带尺寸俯视图](../outputs/eco65_desktop_layout/layout_v1_20260917_candidate02/figures/02_dimensioned_top.png)
- [带高度侧视图](../outputs/eco65_desktop_layout/layout_v1_20260917_candidate02/figures/03_height_side.png)
- [ECO65-B＋真实吸具＋9 箱三维总览](../outputs/eco65_desktop_layout/layout_v1_20260917_candidate02/figures/04_assembly_overview.png)
- [输送方向、转接与出口](../outputs/eco65_desktop_layout/layout_v1_20260917_candidate02/figures/05_conveyors_outlet.png)
- [代表性姿态及未通过位置](../outputs/eco65_desktop_layout/layout_v1_20260917_candidate02/figures/06_representative_poses.png)

工程投影图同时保存 SVG；`pose_01.png` 等保存实际求得的代表姿态。未找到 IK 的点只标目标位置，不伪造一个成功关节姿态。

`backend/scene.xml` 是本地 MuJoCo **几何**场景导出，含同一初始姿态、133 个几何体、所有 9 个箱体和全部墙顶；重新加载后几何位置最大误差为 0。它使用几何 mocap 表达和显式距离检查，不是动力学执行器；不能把此文件的静态加载称为 Isaac/摩擦输送通过。本轮没有安装或运行 Isaac。

## 验证、复现与保护

`tests/test_eco65_unloading_layout.py`：**9 passed in 2.25 s**。覆盖拓扑/尺寸、9 箱支撑、真实轮廓与过小箱面、转接和出口缺支撑负例、硬件与拆顶拒绝、完整几何姿态、功能区域、旧单箱 HOME/接触入口兼容。没有全量 pytest、104/129、整排、整堆、完整路径或性能扫描。

生成过程中第一次输出因 NumPy 布尔类型不能 JSON 序列化而停止，未声称布局失败或成功；该目录 `layout_v1_20260917_candidate01` 保留失败记录。修复序列化后使用新的 candidate02，设计尺寸没有改动。随后仅细化支撑部件 ID/占用/端区的记录和图面取景；几何、关节解与数值结果未变。

本轮使用已存在环境，无重新克隆、CAD 重转换或新增重型依赖。以下入口和参数均实际运行过：

```powershell
& .venv\Scripts\python.exe tools/run_eco65_unloading_layout.py build --run-id layout_v1_20260917_candidate02
& .venv\Scripts\python.exe tools/run_eco65_unloading_layout.py poses --run-id layout_v1_20260917_candidate02
& .venv\Scripts\python.exe tools/run_eco65_unloading_layout.py render --run-id layout_v1_20260917_candidate02
& .venv\Scripts\python.exe tools/run_eco65_unloading_layout.py export --run-id layout_v1_20260917_candidate02
& .venv\Scripts\python.exe tools/run_eco65_unloading_layout.py verify --run-id layout_v1_20260917_candidate02
& .venv\Scripts\python.exe -m pytest tests/test_eco65_unloading_layout.py -q
```

`build` 拒绝覆盖已有快照，`poses` 拒绝覆盖已有姿态证据。复现新的设计时更换 run-id；查看当前结果可直接用 render/export/verify，都会验证资产与快照指纹。

`reports/layout_audit.json`、`reports/poses.json`、`regions.json`、`dimension_mapping.json`、`backend/manifest.json` 保存详细结果。`reports/previous_round_preserved.json` 保存上一轮全部输出的哈希，verify 已确认未变；`reports/delivery_evidence.json` 记录本轮代码/资产/配置、依赖与输出指纹。原始 STEP 与派生网格保持本地私有。

本轮只提交/推送公开代码、候选配置模板、来源映射和本说明。下一步先确认 L 形排布、出口承接方向及所需台面范围，再依据实际办公桌尺寸调整；在这之前不自动继续完整卸货或物理仿真。
交付代码树 SHA-256：`86c3695b6d79ea343aaeb5f3dcce20954bcd79030bf62610c8a15b294d967660`；实际 HOME 的机器人/工具及全部静态模型几何外廓已核对包含在上述最小静态占地内。

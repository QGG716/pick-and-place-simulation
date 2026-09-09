# M-710iD/70 已确认卸货工作站布局专项验证

日期：2026-09-09  
布局 ID：`m710id70_unloading_layout_v1`  
起始代码：`1bb80f5c8400fbcb5f627882c79eb280fe0bd6c2`  
证据代码：`bd30eaef7878b60fb364c953a80f347a4bae1912`

## 结论与资格边界

已把确认尺寸落为独立、严格版本化的布局定义，并使 CPU 几何、碰撞初始化检查、
尺寸图和 Isaac 初始化合同都消费同一个冻结快照。数值布局检查、已知几何中的初始
状态检查、资产哈希和 CPU 快照重放一致性均为 **PASS**。

本结果不是完整工作单元、物理抓取、负载、输送或完整单箱周期资格。车厢长度/高度、
真实机器人安装底板 CAD 和完整输送机制造几何尚未确认，因此完整工作单元净空为
`NOT_EVALUATED`。新布局接收区转角输送为 `NOT_IMPLEMENTED`，没有调用旧
`validation_receiver` 路线。旧 V3 104-task/129-carton 场景没有重跑，也没有通过
新配置重新解释。

## 用户约束、程序实测与状态

| 用户约束 | 程序实测 | 状态 |
| --- | --- | --- |
| 世界原点为箱垛前表面宽向中心的厢底投影；+X 入厢、+Y 左、+Z 上 | `X_stack_front=0`，地板 `Z=0`；快照保存轴语义 | PASS |
| 底盘 `2.100 × 1.500 × 0.600 m` | `[2.1, 1.5, 0.6] m`；范围 `X[-3,-0.9] Y[-0.4,1.1] Z[0,0.6]` | PASS |
| 横带 `0.700 × 1.500 m`，范围 `X[-0.9,-0.2] Y[-0.4,1.1]` | 尺寸和四边坐标一致；后边与底盘前边零间隙 | PASS |
| 纵带 `2.800 × 0.700 m`，范围 `X[-3,-0.2] Y[-1.1,-0.4]` | 尺寸和四边坐标一致；内边与底盘右边零间隙 | PASS |
| 底盘顶面和两段输送表面 `Z=0.600 m` | 三个表面均为 `0.6000000000000001 m`（浮点表示） | PASS |
| 固定装配外廓 `2.800 × 2.200 m` | `X[-3,-0.2]`、`Y[-1.1,1.1]` | PASS |
| 车厢净宽 `2.300 m`，左右壁 `Y=±1.150 m` | 两侧固定外廓净距各 `0.04999999999999982 m` | PASS |
| 两带前缘 `X=-0.200 m`，到箱垛前表面 200 mm | 横带 `-0.20000000000000007`、纵带 `-0.20000000000000018`；净距 `0.20000000000000007 m` | PASS |
| 安装面 `Z=0.600 m`、底盘宽向中心 `Y=0.350 m`、固定基座前缘 `X=-1.100 m` | 基座代理原点 `[-1.41,0.35,0.6] m`，代理前缘 `-1.0999999999999999 m`，支撑范围有效 | PASS（工程代理） |
| 箱体 `[0.600,0.400,0.300] m`，1×5×8，列缝 20 mm、层缝 0、无托盘 | 40 个唯一 ID；总宽 `2.08 m`、总高 `2.4 m`；所有下层支撑箱保留 | PASS |
| 车厢长度、高度不可臆造 | 两项均为 `null` 且状态为 `NOT_DEFINED_BY_CONFIRMED_LAYOUT` | PASS（未定值正确保留） |

完整数值数据见
[`numeric_audit.json`](evidence/m710id70_layout_v1/validation/numeric_audit.json)。

## 单一来源与身份

布局尺寸只在
[`m710id70_unloading_layout_v1.yaml`](../../configs/workcells/m710id70_unloading_layout_v1.yaml)
维护。整机坐标系 A 位于底盘底面投影中心，初始
`T_W_A.translation=[-1.950,+0.350,0]`。底盘、横带、纵带与机器人安装变换均由
`T_W_component = T_W_A @ T_A_component` 生成；箱垛和车厢仍固定在 W 中。

- 布局指纹：`dada3b46c570791cf49745e2fb61003b9aac39c9529e8be94c9caf27a7938804`
- 场景指纹：`9ef799617486ee8986777d96d0cdde5cf46c807782ac8cd8f6ad72a90a2a8698`
- Isaac 合同指纹：`33ca659b731678a3f8fa0784b413c152054c07f54af046b54a9465d456758863`

固定布局身份不含整机当前世界位姿；世界状态身份包含 `T_W_A` 和当前 `q`。修改关键
尺寸或资产内容会改变布局指纹，仅修改渲染 DPI 不会。快照保存 3 个固定组件、40 个
箱体、7 个 M-710 连杆碰撞 OBB、工具包络、关节状态、法兰/TCP、资产内容哈希和能力
边界；丢失或指纹不符时显式失败。

## 机器人安装、工具与初始状态

当前 URDF 的 `base_link` 是半径 `0.310 m`、高 `0.565 m` 的工程圆柱代理，没有
厂商安装底板 CAD。只在“代理前向偏置等于半径且无额外安装偏置”的明确假设下：

`X_base_origin = X_mount_front - proxy_radius = -1.100 - 0.310 = -1.410 m`

因此使用 `[-1.410,+0.350,+0.600] m` 作为 URDF 原点。该值是工程代理定位，不是
厂商认证安装尺寸。真实底板 CAD 到位后必须替换定位依据并改变布局身份。

吸具显示/碰撞引用现有 STEP 衍生资产，物理极值长度是 `0.2275 m`；20 kg 工具
配置中的 `0.250 m` 是显式工作 TCP。二者分开记录，质量配置没有改变几何长度，
法兰、tool0、机械 TCP 和任务 TCP 的完整变换只组合一次。未编造 M-710 连杆惯量
或驱动力矩。

旧 V3 home 在新布局中为 **FAIL**：`J3_link` 的 Y 范围上界
`1.1550788481702754 m` 越过带 10 mm 安全 margin 的左壁允许范围。固定种子
`71070`、最多 3000 个随机 draw 的有界搜索在第 4 个随机 draw 得到：

```text
[3.1135901287083176, 0.7854566731593875, 0.6496218708958161,
 0.06722349609001022, -1.2117503086182402, -1.9415185231007088]
```

该状态在已知侧壁平面、地板、固定装配、40 箱、机器人和工具代理中通过严格检查；
从旧非法状态到它的运动路径为 `NOT_EVALUATED`。

## 图纸

两张图都直接投影同一 `scene_snapshot.json`，机器人使用同一 M-710 URDF、安装变换
和初始 `q`；工具是 STEP 衍生等比例工程包络。各投影视图等比例，无手工拼图。

- 图 A，尺寸约束版：[7200×4200 PNG](evidence/m710id70_layout_v1/figures/m710id70_layout_v1_dimensions.png)；[SVG](evidence/m710id70_layout_v1/figures/m710id70_layout_v1_dimensions.svg)
- 图 B，两视图修正版：[7200×3000 PNG](evidence/m710id70_layout_v1/figures/m710id70_layout_v1_two_view.png)；[SVG](evidence/m710id70_layout_v1/figures/m710id70_layout_v1_two_view.svg)

PNG 为 300 DPI，长边 7200 px；SVG 的线条、几何和文字为矢量对象。车厢长度/高度
没有虚构边界，图注也不声明姿态具有完整运动或负载资格。

## 可复现命令与测试

```bash
python tools/run_m710id70_v3.py --phase layout \
  --config configs/validation/m710id70_layout_v1.yaml \
  --output-dir outputs/m710id70_layout_v1
python tools/render_m710id70_layout.py \
  --snapshot outputs/m710id70_layout_v1/scene_snapshot.json \
  --config configs/validation/m710id70_layout_v1.yaml \
  --output-dir outputs/m710id70_layout_v1/figures
python tools/export_m710id70_isaac_layout.py \
  --snapshot outputs/m710id70_layout_v1/scene_snapshot.json \
  --project-root . --output outputs/m710id70_layout_v1/isaac/contract.json
python -m pytest -q
```

本机实际结果：`313 passed, 4 deselected in 138.78s`。其中新增布局/Isaac 合同专项
为 `14 passed`。4 个 deselected 是默认配置显式排除的可选重型仿真测试，不被写成
通过。

Isaac 初始化入口：

```bash
python scripts/isaacsim_m710_layout_replay.py \
  --contract outputs/m710id70_layout_v1/isaac/contract.json \
  --project-root . --usd-directory outputs/m710id70_layout_v1/isaac/usd \
  --output outputs/m710id70_layout_v1/isaac/run
```

适配器保留全部 40 箱并高亮 `carton_l07_c02`，读取实际创建的 USD 物体位姿/尺寸
和机器人 `q` 形成 backend dump，再由 CPU 合同反审计。当前远端实际运行状态将在
获得 NVIDIA Omniverse EULA 明确授权后补入本报告；在此之前不声明后端 PASS。

## 未完成项

- 真实机器人安装底板 CAD 与 base_link 偏置核对；当前为 0.31 m 半径工程代理。
- 车厢真实长度、高度、顶棚/门框和全部制造 CAD；完整工作单元净空未评估。
- 输送带厚度 `0.120 m` 是保留的工程假设；支腿、护栏、电机未获确认。
- 物理抓取、吸附、箱体动力学、连续输送和完整单箱周期未评估。
- 新布局的接收区转角输送未实现；不会退回旧 L 带清空逻辑。
- 新布局专属任务集合和验收分母尚未定义；本轮不重跑旧 104/129，也不做 IK/RRT、
  base Z、升降、伸缩或节拍优化。

完整文件哈希、运行环境和代码身份见
[`evidence_manifest.json`](evidence/m710id70_layout_v1/evidence_manifest.json)。

# M-710iD/70 layout v1 跨分支接入说明

本说明只定义后续接入方式，不修改或推送在线、视觉、ROS 等其他分支。

## 最小公共提交

公共布局能力从提交 `121ec17` 开始，建议按下列最小集合摘取：

1. `configs/workcells/m710id70_unloading_layout_v1.yaml`
2. `configs/validation/m710id70_layout_v1.yaml`
3. `src/unloading_sim/workcell_layout.py`
4. `src/unloading_sim/isaac_layout_replay.py`（仅在需要后端合同/反审计时）
5. 对应的 `tests/test_workcell_layout.py` 与 `tests/test_isaac_layout_replay.py`

`tools/run_m710id70_v3.py --phase layout` 是 feasibility 分支路由；在线分支不需要
复制旧 V3 task runner。`tools/render_*`、`tools/archive_*` 和 `scripts/isaacsim_*`
是验证/适配入口，不应成为在线控制核心依赖。

## 在线规划分支

在线进程应在启动时加载严格布局 schema，冻结 `scene_snapshot.json`，并把
`layout_id`、`layout_fingerprint`、`scene_fingerprint` 和世界状态身份写入每份计划。
计划执行前重新核验快照和资产哈希；不接受只有可变 config 路径的历史计划，也不在
身份失败时回退 `common_unloading`。整机移动时只更新 `T_W_A` 和世界状态身份，不能
分别移动底盘和两段带。

## 视觉分支

视觉输出必须继续使用 W：箱垛前表面中心地板投影为原点，+X 入厢、+Y 左、+Z 上。
外参结果应组合为 `T_W_object`；设备对象使用统一的 `T_W_A @ T_A_component`，不要
从图像显示坐标重新推断横/纵带方向。检测到的 40 箱身份可与快照箱 ID 关联，但任何
位姿更新都会产生新的世界场景身份，不能改写固定布局指纹。

## ROS/控制平面分支

建议发布一个只读的版本化布局消息/参数包，包含 `layout_id`、布局/场景指纹、
`T_W_A`、固定组件局部变换、机器人安装变换和明确的未定字段。SI 单位和轴定义不可
在消息桥中缩放或换向。控制器收到未知 schema、资产内容不符、缺失快照或 legacy
轨迹时应 fail closed。

新布局当前没有接收区输送能力；调用方必须处理 `NOT_IMPLEMENTED_FOR_LAYOUT_V1`，
不能把旧 `validation_receiver` 的 L 带路线解释成新方案。物理抓取/输送上线前仍需
真实安装底板、车厢边界、制造 CAD、动力学和安全控制的独立资格验证。

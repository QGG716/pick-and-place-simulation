# S50-2000 CAD 六转轴预览模型

`cad_kinematic_model.json`、`../urdf/s50_cad_preview.urdf` 和独立配置 `configs/robots/huayan_s50_2000_cad_preview.yaml` 描述真实 S50 七个部件及六条实测转轴。它们使用用户授权的仿真约定：**源 STEP 装配姿态为 q=0，各关节围绕记录的 canonical axis 按右手规则为正**。这不是厂家控制器零位或正负号映射。

所有连杆保持原始网格的顶点数量、三角形索引和尺度，通过固定刚体变换转换到连杆局部坐标。原视觉网格不修形、不缩放；两张缺面修复另作为 J1/J2 自有碰撞补片，保留原面号、几何哈希和局部修复报告，不据此宣称整个 J1 或机械臂碰撞覆盖合格。

## 坐标约定

- `CAD_from_link_zero`、`origin` 等均为行优先 4×4 SE(3)，长度单位米、角度弧度。
- 安装 frame 位于实测底部支承面与 J1 轴的交点，即 CAD `[0, -0.003, 0] m`。预览坐标的 X/Y/Z 分别采用 CAD 的 +X/−Z/+Y；绕安装轴的方向是明确的仿真约定，不是已核验的实际安装 clocking。
- 各移动连杆的零位坐标轴与安装 frame 平行。原点选在实测转轴线上，具体为子部件 CAD 实例原点在该轴线上的投影。JSON 保留原测量轴点、投影参数和选取规则；沿同一轴改变原点不改变转动几何。
- `s50_flange` 原点位于实测 J6 远端法兰面中心，+Z 为该面的实测外法向；+X 为 CAD +Y 在该平面上的投影。这是独立物理面与仿真定向，未冒充 controller tool0。
- 未添加工具、虚拟 TCP 或新的物理连接件。末端 FK 返回上述法兰面。

六轴 `[-π, π]` 范围只供预览滑块和解析使用，不是机械限位。URDF 中 `velocity=1`、`effort=0` 是明确标注的解析/显示字段，不是硬件参数、动力学输入或执行许可。所有接口标记 `model_usage: cad_preview_fk_only`，正式规划和执行应拒绝此配置。

## 生成与验证

在工程根目录执行：

```sh
python assets/robots/huayan_s50_2000/build_cad_kinematic_model.py
python -m pytest tests/test_s50_cad_model.py -q
```

生成器读取原 CAD 清单、六轴测量、真实 STEP 来源身份及存在的 `collision_repair/coverage_report.json`。连杆局部 OBJ 以 `.obj.gz` 无损保存；`ensure_preview_assets()` 会校验并按需还原标准 URDF reader 所需的原始 OBJ。

`kinematics_validation.json` 是实际运行结果，包含全部原 CAD 顶点与三角形在 q0 的还原、每轴正负转动时正确后代的运动，以及 25 组姿态的核心 URDF FK、递推矩阵 FK、独立空间轴线乘积 FK 比较。测试另以速度叉积核对六轴右手正方向，并核对非平凡世界安装变换和修补面归属。

`load_definition()` 返回该模型定义及其文件 SHA；`load_preview_robot()` 返回现有 `URDFRobot` 实例；`chain_fk()` 和 `poe_fk()` 返回安装坐标或明确提供的世界坐标下各连杆的变换。默认没有实际世界安装或工具适配的隐含偏移。

预览零位本体约高 2.359 m；叠加现有 900 mm 安装高度后会超过 2.7 m 车厢顶棚。q0 是 CAD 运动学参考，不是碰撞合格的作业初态。本验证不声称初态、环境碰撞、机械限位、动力学或单箱规划通过。

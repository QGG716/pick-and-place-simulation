# 横纵带剪叉与倾斜联动（2026-10-19）

完成运动学几何和 USD 场景接入：横带水平升降；纵带远端固定，近端跟随横带高度，保持原 2.8 m 刚性长度。两带均增加剪叉杆、交叉销、端销、滑轨、支座和安装头。没有运行机器人规划或物理卸箱。

基线为 `b99493ed6f2cff9d2477c9a0ecda848284245020`，开始和提交前远端一致；保留原有 `artifacts/`。实测源码为 `c9e0555fc42a796148d8d6c0a3af3bb45b5cf579`；报告和结果属于后续交付提交，源文件 SHA-256 见机器汇总。

[交互查看器与 USD](experiments/m710_coupled_conveyor_lift_20261019/viewer-and-usd.zip)（解压打开 `preview.html`） · [机器汇总](experiments/m710_coupled_conveyor_lift_20261019/summary.json) · [验证记录](experiments/m710_coupled_conveyor_lift_20261019/validation.json)。HTML 自包含；USD 引用合法官方资产缓存，换机器请重新生成。

## 官方资源

查阅 [Isaac 输送带工具](https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.asset.gen.conveyor/docs/index.html)、[Omni Physics 关节/闭环文档](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.0/dev_guide/rigid_bodies_articulations/articulations.html)，并检查服务器安装的 NVIDIA physics demos。此次检索未确认到可直接套用的完整剪叉台资产，不把自建模型称为 NVIDIA 成品。

复用已锁定的官方 [ConveyorBelt_A06.usd](https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/Props/Conveyors/ConveyorBelt_A06.usd)，读取时校验 SHA-256 `0e1b606d4cfbfbc2ce51300b7773f08cb4656771faa105d3366dc85745f042dc`。沿用 `workcell_visuals.attach_a06_belts()` 的带体/材质适配，不把原高支架缩放成剪叉。查看器使用同源 OBB；USD 引用实际官方带体视觉资产。

剪叉为项目概念参数：横带两组两级、杆长 1.25 m；纵带两组四级、杆长 0.60 m。杆长保持不变，固定销、滑动端和摆动安装头分开表示。没有制造尺寸、质量惯量或驱动规格，因此 USD 使用受控运动学碰撞体；RevoluteJoint 只记录禁用的交叉销拓扑，不冒充完整闭环动力学，也未用线性 mimic 代替非线性联动。

## 联动关系

| 项目 | 低位 | Hmax |
|---|---:|---:|
| 横带面高 / 纵带近端高 (m) | 0.600 | 2.050 |
| 纵带远端铰点 (m) | (-3.000, -0.750, 0.600) | 相同 |
| 纵带倾角 | 0° | 31.1886° |
| 纵带近端 X (m) | -0.200 | -0.604692 |
| 纵带刚性长度 (m) | 2.800 | 2.800 |

按 `sin(theta)=(H-H_rear)/L` 求倾角，近端 X 从刚性长度推导，所以后移 404.692 mm，不拉长纵带。近端仍在横带 X 范围 [-0.9,-0.2] 内，但端部高度对齐不代表箱体转接合格；倾斜输送、防滑和滚筒过渡未评估。

`coupled_conveyor_state()` 统一产生全部世界几何、机构姿态和身份。现有 `LiftedConveyorScene.build(..., coupled_lift=True)` 显式接入；默认仍是旧纯 Z 横带场景。其他环境和 40 箱没有移动。查看器机器人为配置 q 的官方模型静态参考，不代表新机构下的碰撞许可。

USD `apply_state()` 写入前核对原身份、原位姿、对象集合和几何尺寸；固定支座、远端铰点及其他场景对象不能随更新移动。显示和碰撞体共用位姿。新场景不能沿用旧原生 `LIFT_Z` 单盒纯 Z 扫掠证明，未把旧 PASS 迁移到转动纵带与剪叉。

## 实测

- 新机构与原升降几何：31 passed，1.18 s。
- 已安装 USD 的更新、篡改拒绝、固定支座、碰撞体及读写：5 passed，0.933 s。
- 真实 USD 26.8、官方 A06 缓存：101 个高度参数状态、139 个对象的全部世界矩阵一致，保存重载通过。101 点是展示参数取样，不是连续碰撞证明或运动时间。
- 浏览器低位/Hmax及滑块检查通过，无脚本异常；见 `low.png`、`high.png`。
- 额外旧 `test_workcell_layout.py`：9 passed / 4 failed。涉及装配变换假设一项、旧初态应 FAIL 的断言三项；该测试及相关布局/初始化源码、配置与基线逐文件一致。本轮保留失败，不修改旧断言掩盖问题，详见验证记录。

沿用 Linux 既有环境。USD 使用现有 Isaac Sim 6.0.1.0 环境中的 Python/pxr，没有启动 SimulationApp或升级依赖。几何测试在既有 Ubuntu 22.04 隔离环境。代码、报告采用 UTF-8。

`qualification_status=NOT_EVALUATED`，`continuous_swept_proof=false`，`physical_execution=NOT_RUN`。没有真实升降、Isaac 取放、真机、执行包或新五箱实验；承载、速度、硬件行程、落带和扫掠碰撞资格均未评估。旧抓取/抽离规划保留；包含新机构的原生协调规划尚未接入。

## 复现

仓库根目录，使用已有 numpy/PyYAML/pxr 环境，仅生成场景：

```bash
/root/autodl-tmp/envs/isaacsim-clean/bin/python tools/preview_m710_conveyor_lift.py \
  --output /root/autodl-tmp/m710-coupled-lift-20261019 \
  --source-commit c9e0555fc42a796148d8d6c0a3af3bb45b5cf579 \
  --usd-asset-cache /root/autodl-tmp/v05-assets/workcell/mirror
```

输出须用新目录。省略 `--usd-asset-cache` 仅生成 HTML/JSON，不需要 USD。官方资产清单是 `configs/isaac/conveyor_assets.json`。USD 停在 Hmax；停止的几何编辑场景中可调用 `apply_state(stage, coupled_conveyor_state(layout, H))`，一次更新两带与剪叉，不直接移动孤立碰撞体。

```bash
python -m pytest -q tests/test_conveyor_lift.py tests/test_lifted_reception.py
# 已有 USD 环境：
python tests/test_conveyor_lift_usd.py
```

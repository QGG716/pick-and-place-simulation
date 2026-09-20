# 右壁—顶板黑带：原图、命中与局部检视灯

基线 `9b85f8b0b287b4ecd43d915018f19f22b3eea60e`，原数据为服务器
`/root/autodl-tmp/v05-acceptance/video-demo-20260920/recording-01`。
原录制、视频及历史报告均未改写。没有运行 SAM、米制几何恢复、完整演示或性能测试。

## 原始局部图

原 PNG 的整数像素裁剪，没有提亮、补色、填缝、缩放或叠画。
坐标采用左上角原点，裁剪右/下边界不包含：上相机 `(1550,500,2592,1020)`，
下相机 `(1550,0,2592,520)`。下相机实际看不到顶板接缝；重新采集的三帧中 Roof 命中像素数均为 0。

| 原帧号 / 序号 | 上相机接缝 | 下相机右上区域 |
|---|---|---|
| 32 / 0 | [原图](evidence/roof-wall-seam-20260920/before-32-module_0_upper.png) | [原图](evidence/roof-wall-seam-20260920/before-32-module_1_lower.png) |
| 320 / 48 | [原图](evidence/roof-wall-seam-20260920/before-320-module_0_upper.png) | [原图](evidence/roof-wall-seam-20260920/before-320-module_1_lower.png) |
| 602 / 95 | [原图](evidence/roof-wall-seam-20260920/before-602-module_0_upper.png) | [原图](evidence/roof-wall-seam-20260920/before-602-module_1_lower.png) |

## 实际来源和几何

原 Isaac 日志 `kit_20260920_110153.log` 的命令行加载 `roof-mast-20260916/bundle-f24f63a`。
该 bundle 的 `index.json` 指向 `effective_isaac_contract.json`，其 fingerprint 为
`69d25f0766d55b06894e92bf5a6966f3df241c003b83c0ae5e9347e61b9c0bb9`。
原 manifest 的 `source_manifest_fingerprint` 与该 bundle 的 FULL_STACK_NOMINAL 一致。
原始实例 ID 表包含 `p_047_Roof`、`p_046_RightWall`，不是旧兼容路径的 `RoofRim`。
核查使用这些实际输入，不以当前 YAML 代替来源证明。

原连续采集没有导出其内存 USD，也未保存完整 first-hit 数组。
本轮按原 bundle、原场景构建代码和原 manifest 相机姿态重建，导出实际运行 USD，
再用原始深度交叉核对新实采 ID；不将重建 USD 冒充历史原件。
[USD 边界、逐级父变换与设置](evidence/roof-wall-seam-20260920/actual-usd-and-settings.json)、
[来源哈希、原始深度和新命中对照](evidence/roof-wall-seam-20260920/pixel-and-source-evidence.json)。

| USD 实体 | 世界 X 范围 m | 世界 Y 范围 m | 世界 Z 范围 m |
|---|---|---|---|
| Roof | −3.800000095～4.000000095 | −1.210000038～1.210000038 | 2.700000001～2.759999999 |
| RightWall | −3.800000095～4.000000095 | −1.209999999～−1.150000001 | −0.000000024～2.700000024 |

两者都是单个连续 Cube，父级为单位变换，沿完整局部长度 7.8 m 接触。
浮点转换产生约 `2.32e-8 m` 接触重叠，没有正间隙。内宽 2.30 m、内高 2.70 m 未变。
局部段 X 两端没有端板；`door_world_x_m=null`，不能把 X=4 或 X=−3.8 指认为确定的门口。

## 黑色不是同一种原因

以下像素来自上相机原帧 32。深度为原始 optical-Z 米值，实体名为同姿态新实采 first-hit ID：

| 像素 (u,v) | 原 RGB | 深度 m | 命中 / 回投 |
|---|---|---:|---|
| (1800,750) | (3,2,1) | 2.068882 | Roof，世界 Z≈2.70035 |
| (1600,900) | (3,2,2) | 4.668856 | RightWall，世界 Y≈−1.15000 |
| (2000,600) | (31,24,17) | 1.229993 | 同一 Roof 的较亮区域 |
| (1350,900) | (0,0,0) | 无命中 | 背景，射线从 X=4 局部段敞口离开 |

在接缝上下各 3 像素、四个不同横向位置的采样中，命中由 Roof 连续过渡为 RightWall。
其距离在 1.17～2.82 m；实际相机裁剪为 0.05～12 m，黑带并非远裁剪面。
Roof 和 RightWall 使用同一均匀 TrailerWall 材质，没有黑色贴图条。

[光源至表面的可见性射线](evidence/roof-wall-seam-20260920/light-visibility-rays.json)
使用实际 USD Cube 的局部坐标盒相交和实际 Mesh 三角形相交。
到黑色 Roof 样点的下模组灯路径被顶层 `carton_l07_c02` 网格遮挡，
到相邻较亮 Roof 样点的路径没有该遮挡；外部环境球灯到内表面被 Roof/RightWall 遮挡。
上模组灯仍有未遮挡路径，故不能声称所有灯都被挡住。
实际 GI 已开启；这里是实物遮光、入射角和照度差异形成的深色表面，另叠加远端敞口背景，
没有缺失顶板、板间开缝或错误场景版本。

末帧左侧另有近处灯壳遮挡，命中深度约 0.065 m；它不是本轮右壁—顶板问题，没有移动相机遮掩。

## 修改与复现

用户选择增加局部检视灯。新增显式 `--seam-inspection-light`，默认不开启；
它是模拟接缝检视光源，不宣称为已安装硬件，也不改变执行环境契约。
来源 manifest 明确标记 `LOCAL_ROOF_RIGHT_WALL_INSPECTION_LIGHT; NOT_NOMINAL_LIGHTING`。
仅新增朝向接缝的局部 RectLight；原灯、全局曝光、材质及所有几何保持不变。
原端部敞口仍然开放。

[首、中、末帧原图与最终检视灯对照](evidence/roof-wall-seam-20260920/before-after-inspection.png)。
拼图只在图像外添加标签，图内仍是整数像素原样复制。最终六张独立原始裁剪：

| 原姿态 | 上相机 | 下相机 |
|---|---|---|
| 32 | [检视灯开启](evidence/roof-wall-seam-20260920/after-32-module_0_upper.png) | [检视灯开启](evidence/roof-wall-seam-20260920/after-32-module_1_lower.png) |
| 320 | [检视灯开启](evidence/roof-wall-seam-20260920/after-320-module_0_upper.png) | [检视灯开启](evidence/roof-wall-seam-20260920/after-320-module_1_lower.png) |
| 602 | [检视灯开启](evidence/roof-wall-seam-20260920/after-602-module_0_upper.png) | [检视灯开启](evidence/roof-wall-seam-20260920/after-602-module_1_lower.png) |

最终检视灯位于世界 `(1.0, −0.8, 2.45) m`，面积 `4.0 × 0.03 m`，
绕 X 轴 −125.54°，朝向右上接缝，强度 5000、exposure=0、normalize=false、5000 K。
这些是仿真检视参数，不是实测灯具的光度标定。没有改变全局亮度或把墙面改成自发光材质。

在原服务器仓库目录使用与原录制相同的 bundle、资源缓存及 Python：

```bash
OMNI_KIT_ACCEPT_EULA=YES /root/autodl-tmp/envs/isaacsim-clean/bin/python \
  scripts/isaacsim_perception_capture.py --project-root "$PWD" \
  --feasibility-root /root/autodl-tmp/m710-official-dynamics-20260910/repo-feasibility-core-648e177 \
  --bundle-directory /root/autodl-tmp/v05-acceptance/roof-mast-20260916/bundle-f24f63a \
  --usd-directory /root/autodl-tmp/v05-acceptance/dual-rgbd-v4-221b296/usd \
  --output ../my-new-seam-check --carton-assets configs/isaac/carton_assets.json \
  --asset-cache /root/autodl-tmp/v05-assets/cartons/mirror \
  --workcell-asset-cache /root/autodl-tmp/v05-assets/workcell/mirror \
  --seam-check-recording ../recording-01/sequence-prefix.json --seam-inspection-light
```

去掉最后的 `--seam-inspection-light` 即为原照明复核。
只采索引首、中、末三组，新的 epoch、时间、绑定、RGB-D、first-hit 数组和 USD
保存在新目录；已有目录拒绝覆盖。采集同时检查实际 Camera 子节点世界矩阵。

对照使用原 manifest 指定的名义姿态。原连续录制中间/末帧与稳定重采有毫米级壁面深度差异，
完整图的 p99 差异最高约 24.7 mm，不能宣称与历史图逐像素一致。
本轮没有扩大为连续采集时间同步修复；检视灯开关之间的几何/深度不变性单独核对。

## 验证结果

原照明稳定重采目录 `seam-before-02`，最终检视灯目录 `seam-inspection-07`，
均位于原服务器 `video-demo-20260920/` 下。完整 RGB-D、绑定、命中数组和 USD 留在这些新目录。
[最终对照检查及运行代码哈希](evidence/roof-wall-seam-20260920/inspection-comparison.json)、
[检视灯实际 USD / Camera / 渲染设置](evidence/roof-wall-seam-20260920/inspection-usd-and-settings.json)。

- 两组共 12 个模组绑定通过现有 `verify_capture`；没有运行模型。
- 检视灯开关前后，六张全分辨率深度逐像素完全一致，含无效深度位置。
  数值实例 ID 随新场景注册重新编号，按 ID 表映射后的逐像素实体路径完全一致。
- 相机 K、请求姿态、箱堆和机器人 manifest 完全一致；实际 Camera 子节点姿态也核对通过。
  Roof/RightWall 的实际 USD 边界及父变换、原有六盏灯、全局渲染/曝光设置完全一致。
- 六组原始 RGB、深度及 binding 哈希仍与开始审计时一致。
- 首帧接缝上方 `(1800,806)` 的 RGB 从原图 `(2,2,1)` 变为 `(46,37,28)`，
  深度仍为 2.754278 m，实体仍为 Roof；接缝下方仍命中 RightWall。
- `PYTHONUTF8=1 python -m pytest -q`：646 passed、1 deselected；脚本编译和 `git diff --check` 通过。
  本轮未重新运行已知会 GBK 解码失败的非 UTF-8 默认环境。

中途检视灯试采及失败诊断目录保留在服务器，没有挑换相机姿态或删除原记录。
最终仅使用普通面积灯；早期尝试的光束整形属性未进入最终实现。
这是只读图像检查灯，不是原照明感知算法验收、灯具工程设计或执行准入。

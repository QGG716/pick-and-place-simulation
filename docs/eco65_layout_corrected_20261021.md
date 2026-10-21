# ECO65 布局纠正，2026-10-21

在工作分支 `feat/v0.6-eco65-moveit2-native-single-carton` 的
`e00a89708ea29414982a925792ae1b56cbf89774` 上增量修复。
本轮按用户最新要求只处理布局，未运行原生抓放或下一箱回程。

三个问题已修正：

1. 桅杆安装底座相对上一版绕 J1 的 Z 轴再转 180°，原始关节角不变。
   新配置 `eco65_base_mast_remount_v2.yaml` 从未改装的原始安装计算总计 270°，
   即上一版 90° + 本轮 180°，没有将“再转 180°”误写成绝对 180°。
   机械臂开关面仍朝 −X。圆立柱与相机转接件一起反向调整，
   在相同原始 J1 下相机世界朝向不变，安装高度不变。
   按新的支架方向重新生成贴合垫座，替换原有垫座碰撞单元。
2. 两条有效带面的实际 Ymin 均为 −0.300 m。原始 STL 离散精度使两个边缘
   相差 `1.2984764e-9 m`；在 H=0.18、0.33、0.43、0.45 m 时一致。
   纵带有效刚性长度仍为 `0.4443197215708027 m`；端点接缝、剪叉杆长和固定地脚关系不变。
3. “多出两条传送带”的根因是 `make_coupled_spec()` 重建已联动场景时，
   将已经处于局部坐标的带体再次作为旧 D 坐标转换，使两条带体脱离两条正常支撑框架。
   不是新增了两个输送机总成。修复后重用不可变的局部带体模板，并检查真实网格边界；
   对已经被错误转换的输入明确拒绝，不静默传播。没有通过隐藏支架或关闭碰撞来掩盖问题。

上一版错误低位实体 Ymin 为横带约 −0.397840 m、纵带约 −0.159686 m，
纵带 X 还额外偏移 −0.250 m。原生碰撞和三维显示都使用了该错误几何，
因此上一版规划失败只作为历史记录保留，不能推断本轮正确布局是否可行。
新场景从几何正确的 `cup_contact_20261020_scene01` 重新建立，未借用旧轨迹。

## 交付

结果目录：`outputs/eco65_native/layout_corrected_20261021_result/`

- `layout.html`：内嵌 Three.js、实际机器人/工具/桅杆几何及联动场景数据的离线三维页面。
  下拉选择低位 0.18 m、高位接收 0.43 m、低位接收 0.33 m，支持俯视、侧视和碰撞几何显隐。
  紫红线标出共同 Ymin；信息栏显示实际边缘值与两个输送机总成。
- `layout.zip`：上述 HTML、压缩数据、快照和 Three.js 许可证的离线包。
- `layout_top.png`、`layout_low.png`、`layout_high.png`：同一页面的实际渲染截图。
- `mast_comparison.png`：同一 J1=0 下上一版与修正后的桅杆安装俯视对比。
- `geometry_checks.json`：实际网格、两带/框架关系、固定结构、杆长和安装矩阵检查。
- `browser/checks.json`：页面检查，包括独立读取浏览器网格顶点核对 Ymin。

浏览器检查通过，共 2833 项；页面明确记录 `NOT_RUN_NO_PLAN` 和
`NOT_RUN_NO_NATIVE_REQUESTS`，未将布局检查标记为循环成功。
HTML SHA256：`5acaa73c4985b4e54454f8285bdef15d90067d08fe44a429eb729f7de7da0d10`。

新快照：`outputs/eco65_native/layout_corrected_20261021_scene01/snapshot.json`

快照指纹：`f8c80cb93b0028078df3d8dc11cac517e582766730492a87a41e939c3f17d379`。
原生模型、显示、碰撞世界和可选 USD 导出继续使用同一个场景状态函数。
本轮未启动 GPU/原生 worker，也未导出新的 USD；交付的是实际构建的原生模型和离线布局预览。
私有资产、生成快照和图片继续遵守仓库忽略规则。

## 定向检查与实际命令

安装和联动几何定向回归 25 项通过，包括重复构建、从抬升状态重建、损坏模板拒绝、
实际实体边缘对齐、杆长/地脚不变以及桅杆转 180°且相机方向保持。
新快照的 12 个箱体 ID、尺寸、位姿保持；world=12、attached=0、completed=0。
原始 q=[0,0,0,0,0,0] 仅用于静态布局展示，不宣称其为通过原生碰撞检查的运动初态。

在仓库根目录的 PowerShell 执行：

```powershell
$env:PYTHONPATH='src;tools'
.venv/Scripts/python.exe tools/build_eco65_coupled_carton.py `
  --source outputs/eco65_native/cup_contact_20261020_scene01/snapshot.json `
  --home outputs/eco65_native/cup_contact_20261020_scene01/home_candidate.json `
  --layout-config configs/workcells/eco65_base_mast_remount_v2.yaml `
  --output outputs/eco65_native/layout_corrected_20261021_scene01 `
  --native-asset-root /work-eco65-20261015
.venv/Scripts/python.exe -m pytest tests/test_eco65_coupled_scene.py tests/test_eco65_layout_installation.py -q
.venv/Scripts/python.exe tools/check_eco65_layout_geometry.py `
  --snapshot outputs/eco65_native/layout_corrected_20261021_scene01/snapshot.json `
  --previous outputs/eco65_native/layout_remount_20261021_scene01/snapshot.json `
  --output outputs/eco65_native/layout_corrected_20261021_result/geometry_checks.json
.venv/Scripts/python.exe tools/export_eco65_coupled_replay.py `
  --snapshot outputs/eco65_native/layout_corrected_20261021_scene01/snapshot.json `
  --preview-q 0 0 0 0 0 0 `
  --output outputs/eco65_native/layout_corrected_20261021_result/layout.html
.venv/Scripts/python.exe tools/check_eco65_coupled_replay_browser.py `
  --html outputs/eco65_native/layout_corrected_20261021_result/layout.html `
  --browser 'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe' `
  --output outputs/eco65_native/layout_corrected_20261021_result/browser
```

重新构建/导出须选择新的输出目录，历史产物不覆盖。
构建墙钟 97.765 s，离线导出墙钟 34.694 s；规划墙钟 `NOT_RUN`，机器人运动时间 0 s，
机构动画时间 0 s（三个离散高度预览，不是假造外部轴运动段）。
`planning_only=True`、`physical_execution=NOT_RUN`、`execution_ready=false`。
完整单箱循环与下一箱接触回程未在本轮运行，也未标记完成。

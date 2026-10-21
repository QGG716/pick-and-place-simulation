# ECO65 桅杆底座顺时针 90°及相机 +Y 安装，2026-10-21

本次直接基于已交付的 `layout_corrected_20261021_scene01/snapshot.json`
及其 `layout.html`，在提交 `17797a5dfcf3de0574ac93417f7c217d444f4157` 上增量修改。
使用 `eco65_base_mast_remount_v3.yaml`，仅调整桅杆安装及相机方向。

- 桅杆底座相对上一版绕 J1 的 Z 轴顺时针旋转 90°，即 −90°；
  当前预览 q=[0,0,0,0,0,0] 不变。底座的 CAD 水平轴转后为世界 −X，与 X 轴平行。
- 相机绕真实圆立柱轴调整，圆立柱的上部安装口、俯仰转接件及相机保持刚性连接。
  在当前预览 J1=0 下，实际 optical +Z 视线为 `[0, 0.866025403784, -0.5]`，
  水平投影朝 +Y，下俯 30°。相机继续保持随 J1 运动的原有关系。
- 机械臂安装位姿及官方关节定义、箱体、横纵带体、剪叉和支撑直接保留，未重新布置。
  新旧场景的 `native_world_objects`、`boxes`、`coupled_state`、`coupled_conveyor_spec`
  以及环境几何逐项完全一致；不同 J1 下的机器人 FK 也保持一致。
- 重新生成适配旋转后支架的贴合垫座及其碰撞几何；原垫座不再参与新场景。
  相机名义外参更新并重新绑定安装指纹。显示与原生模型使用同一场景。

安装变换的 11 项定向回归通过，新快照的几何对照检查通过。
离线浏览器检查 2835 项通过，包括底座顺时针 90°、水平轴平行 X、相机 +Y/下俯 30°，
以及实际机器人 FK、两带实体 Ymin 和三高度场景显示。
预览始终是静态布局，不使用历史运动轨迹，不宣称已完成原生抓放或碰撞可行性验证。

新快照指纹：`dd393241e34d092cde91ca24b5a02ce6bb7022419bdb1b5fd92b1fd42ce8bce3`。
新场景：`outputs/eco65_native/layout_mast_cw90_20261021_scene01/`。
交付目录：`outputs/eco65_native/layout_mast_cw90_20261021_result/`。

交付包含离线 `layout.html`、完整 `layout.zip`、`mast_comparison.png`、
`layout_top.png`、低/高位截图、`geometry_checks.json` 和 `browser/checks.json`。
HTML 内嵌模型、Three.js 和场景数据，不依赖网络或本机资产路径。
历史布局文件保留；请打开新交付目录中的 `layout.html`。

实际构建和导出命令（仓库根目录 PowerShell；重跑应选新的输出目录）：

```powershell
$env:PYTHONPATH='src;tools'
.venv/Scripts/python.exe tools/build_eco65_coupled_carton.py `
  --source outputs/eco65_native/layout_corrected_20261021_scene01/snapshot.json `
  --home outputs/eco65_native/layout_corrected_20261021_scene01/home_candidate.json `
  --layout-config configs/workcells/eco65_base_mast_remount_v3.yaml `
  --output outputs/eco65_native/layout_mast_cw90_20261021_scene01 `
  --native-asset-root /work-eco65-20261015
.venv/Scripts/python.exe -m pytest tests/test_eco65_layout_installation.py -q
.venv/Scripts/python.exe tools/check_eco65_layout_geometry.py `
  --snapshot outputs/eco65_native/layout_mast_cw90_20261021_scene01/snapshot.json `
  --previous outputs/eco65_native/layout_corrected_20261021_scene01/snapshot.json `
  --output outputs/eco65_native/layout_mast_cw90_20261021_result/geometry_checks.json
.venv/Scripts/python.exe tools/export_eco65_coupled_replay.py `
  --snapshot outputs/eco65_native/layout_mast_cw90_20261021_scene01/snapshot.json `
  --preview-q 0 0 0 0 0 0 `
  --output outputs/eco65_native/layout_mast_cw90_20261021_result/layout.html
.venv/Scripts/python.exe tools/check_eco65_coupled_replay_browser.py `
  --html outputs/eco65_native/layout_mast_cw90_20261021_result/layout.html `
  --browser 'C:/Program Files (x86)/Microsoft/Edge/Application/msedge.exe' `
  --output outputs/eco65_native/layout_mast_cw90_20261021_result/browser
```

本轮未运行原生规划和硬件；`planning_only=True`、`physical_execution=NOT_RUN`、
`execution_ready=false`。world 箱体 12、attached 0、完成 0。

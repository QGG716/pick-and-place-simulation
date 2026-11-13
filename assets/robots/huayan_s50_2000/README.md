> 发布范围：用户已明确授权将 S50 原始 STEP、7份无损gzip派生网格、内嵌离线查看器及官方手册发布到 `feat/v0.5-s50-moveit2-native-single-carton`，这些资产随本轮追加提交纳入。此次用户授权不等同于取得厂家明确的 CAD 再分发许可；厂家归属、来源及许可缺项继续保留。历史验证数据保持原样。

# 华沿 S50-2000 真实 CAD 与来源审计

当前状态：**真实 CAD 已取得并拆分，控制器坐标语义和碰撞模型尚未就绪，不能称为机械臂替换完成。**

原始文件为用户提供的 `source/S50.STEP`，SHA256：
`7296244abc02bca236a7ed26a0f10f5cc56fc63b7381fc1293b2edbb92509bc3`。
文件头标识 S50-2000、SolidWorks 2022、2025-08-01、58 模组整机、AP214。没有替换成 S60、缩放其他机器人或创建占位 URDF。

七个独立装配部件为底座及 J1–J6；`derived/*_cad.obj` 是已应用装配变换的米制 **CAD 整机坐标**，不是各关节局部坐标。读取时不得再次乘 `cad_placement`。CAD 竖直方向是 +Y；它不能直接当作本工程世界 +Z 的控制器基坐标。

- `sources.json`：来源、原始哈希、官方标量参数、手册/网页速度冲突、许可边界与剩余缺项。
- `derived/cad_audit.json`：7 部件、15 个实体、网格哈希、真实尺寸、所有解析圆柱/平面及缺面记录。
- `derived/compressed_meshes.json`：7份OBJ的无损gzip传输清单，分别记录原始/压缩路径、SHA256、字节数和逐字节解压核对。
- `derived/interface_measurements.json`：从相邻真实部件同轴圆柱测得的 6 条无向轴线及安装/法兰面。
- `derived/manual_cad_crosscheck.json`：13 项 CAD/官方尺寸交叉核对，全部在 0.01 mm 阈值内。
- `derived/shape_fix_attempt.json`：实际 OCCT 修复尝试未改善，保留失败。
- `source/S-series-mechanical.pdf`：官方机械手册，SHA256 `2122d5d29d2e8e33c04f36183caa820d2f80e6a93ca03f8d229f0a8fdf69cd02`。

实际底部支承为 CAD Y=-0.003 m 的六个共面支承面；Y=0 是内凹中心面。安装面到 J2 轴为 283.3 mm，已与官方尺寸图核对。安装孔为 6×Ø13.5、PCD302；法兰为 8-M6 深10、PCD92。CAD 中 Ø5 圆柱是螺纹底孔表现，不是 M6 的公称直径。

J1 原始拓扑检查失败，J1 面74、J2 面75未能三角化；每个面的有符号面积约 -1.292529 mm²。ShapeFix 后问题仍存在，七份导出网格与原始导出逐字节相同。网格可显示实际证据，不能据此宣布碰撞检查通过。

仍缺控制器零位/正向与 CAD 参考姿态的映射、安装绕轴定向、法兰工具坐标定向与保留工具的物理连接对应、速度版本确认，以及两处缺面的合格碰撞覆盖。缺少惯量和力矩属于动力学资料缺项，不是本次运动学停在上述检查处的理由。

复现转换需 Python 3.12、NumPy、trimesh 和 `cadquery-ocp-novtk==7.9.3.1.1`；核心运行包不依赖它们：

```powershell
$env:PYTHONPATH='outputs/s50_model_sources/cad_python312'
& 'D:/code/eco65 pick and place/.venv/Scripts/python.exe' -X utf8 assets/robots/huayan_s50_2000/audit_step.py
& 'D:/code/eco65 pick and place/.venv/Scripts/python.exe' -X utf8 assets/robots/huayan_s50_2000/measure_interfaces.py
& 'D:/code/eco65 pick and place/.venv/Scripts/python.exe' -X utf8 assets/robots/huayan_s50_2000/crosscheck_manual.py
```

可选修复复现使用 `audit_step.py --repair --output assets/robots/huayan_s50_2000/derived/healed`；原始 STEP 保持不变。报告记录的是本轮实际失败结果，不会将修复尝试当作修复成功。

原始 CAD 来源是用户提供的本地文件；其哈希是否与官网需要登录的 ZIP 相同尚未核实。项目负责人已于 2026-11-13 书面确认 CAD 与手册可随本仓库分发，见 [分发授权记录](../../../docs/redistribution_authorization_20261113.md)；官方 ROS 仓库的 BSD-3-Clause 许可仍不套用于这些文件。保留厂商归属与来源说明。

## 无损网格发布

按用户对本次发布的明确授权，7份 `derived/*_cad.obj.gz` 与原始 STEP、官方手册及内嵌查看器随追加提交纳入新分支；未压缩OBJ只在本地保留，可无损恢复。GitHub传输接口对部分13 MB文本OBJ返回错误，因此采用gzip传输；没有简化网格、减少三角形或更改坐标。`cad_audit.json` 中的原始OBJ路径及SHA256保持不变；查看器在原始OBJ不存在时读取同名 `.obj.gz`，解压后按原始SHA256核验。

`pack_meshes.py` 使用固定 `mtime=0`、空文件名和级别9，验证重复压缩结果一致，以及解压后与原始文件逐字节一致。压缩不会修复J1/J2缺面，也不会改变未就绪结论。

```powershell
& 'D:/code/eco65 pick and place/.venv/Scripts/python.exe' -X utf8 assets/robots/huayan_s50_2000/pack_meshes.py
# 在只含gzip的干净checkout中恢复原始OBJ；仅需Python标准库，不必重新安装CAD依赖。
& 'D:/code/eco65 pick and place/.venv/Scripts/python.exe' -X utf8 assets/robots/huayan_s50_2000/pack_meshes.py --unpack
```

也可通过前述 `audit_step.py` 从原始STEP重新生成OBJ。`--unpack` 遇到不相符的现有OBJ会报错，不覆盖本地不同内容。

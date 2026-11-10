> 发布范围：用户已授权发布真实 STEP、派生网格、嵌入网格的离线查看器及官方手册；本次追加提交纳入这些资产。此前 `cff77ce` 为代码/报告版，其历史记录保持原样。用户授权不等于厂家再分发许可，详见 [发布范围说明](PUBLICATION.md)和 [资产发布验证](asset-publication-validation.json)。

# S50 CAD 与冻结布局查看器

下载本次追加提交中的 [viewer.html](viewer.html)，双击即可打开，无需联网或启动服务。离线文件内嵌现有 Three.js、OrbitControls、许可文本及索引化网格，不内嵌 STEP 原文件。JSON 数据使用 `gzip(mtime=0) + base64` 无损内嵌，全部几何数值保持不变。

浏览器最低能力要求：支持 `DecompressionStream('gzip')`、`Blob.stream()`、`Response.text()` 和 WebGL。建议使用具备这些能力的新版 Chrome、Edge、Firefox 或 Safari；不支持解压 API 时页面明确显示加载错误，不会静默显示空场景或生成替代模型。解压在浏览器本地完成，不需要 CDN 或外部资源。

页面明确显示 `MODEL_NOT_READY`，提供两个独立视图：

- **冻结完整 40 箱场景**：显示固定环境、真实安装轴线，以及原工具在吸盘目标位姿的参考几何。工具是浮动目标参考，不代表 S50 的 FK、合法初态或已通过碰撞检查。横带可选择 550、600、1000、1500 mm 的冻结几何采样。
- **真实 STEP 装配**：显示实际 7 个 CAD 部件、1,312,676 个三角面。保留 CAD 装配坐标（+Y 竖直）和米单位，不重复应用装配实例变换，也不套用未确认的世界安装变换。可分别开关部件、查看坐标轴、线框及对象信息。URL 后添加 `#cad` 可直接进入该视图。

六个关节滑块保持禁用并说明缺失定义；没有真实运动链和运行轨迹时不生成动画、不复制 FANUC 的关节角。J1/J2 各有一张源几何面未能三角化，J1 拓扑检查不通过，查看器保留这些失败字段。静态装配可见不等于碰撞网格验收。

重建命令（在仓库根目录）：

```sh
python tools/freeze_robot_replacement_layout.py
python tools/export_s50_model_viewer.py --output docs/experiments/s50_model_integration_20261110/viewer.html
python -m pytest tests/test_robot_replacement_layout.py tests/test_s50_model_viewer.py -q
```

导出会核验每个 OBJ 的 SHA-256、顶点/三角面数量及包围盒，再生成 `viewer-checks.json`。布局模块在 4 个高度各比较 148 个实体，并比较全部机构定义、行程和工具固有几何；完整 40 箱不继承运行检查点。

本次提交的 CAD 网格保存为 `*.obj.gz`。导出器优先读取原始 OBJ；原始文件不存在时，自动解压同名 `.obj.gz` 并按 `cad_audit.json` 中原始 OBJ 的 SHA-256 验证，随后使用相同解析路径。清单里的原始文件名和查看器数据不变，压缩损坏或解压后哈希不匹配时拒绝导出。

跨平台原始环境指纹可能不同，应与几何比较分开记录。本轮 Windows/Linux 对比发现仅 6 个机构派生 `angle_rad` 的最后浮点位不同，最大差值约 `1.11e-16 rad`；所有实体尺寸、位姿及配置相同。原始指纹继续保留，不对业务数据整体取整，也不把原始哈希不同伪装成相同。严格的同平台 FANUC/替换布局差异检查不使用数值容差。

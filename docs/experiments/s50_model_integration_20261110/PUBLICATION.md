# 本次公开分支的资产范围

代码、配置、测量报告、来源哈希与实际验证记录已准备发布。原始 STEP、官方机械手册、七个完整网格包和嵌入完整网格的生成版 `viewer.html` 暂留本地及隔离服务器目录，没有纳入此次公开分支的目录树。自动审批拒绝发布原始 CAD/文档，理由是现有代码和报告推送授权没有明确覆盖这些资产；资产发布确认仍在等待用户回复。这不改变已有 CAD、网格、报告的内容或模型未就绪结论。

提交的 CAD 测量报告含解析面参数、轴线、尺寸、部件变换和哈希，不含完整网格顶点/索引、裁剪边界或 BRep/NURBS 拓扑。它们是测量证据，不是可运行的替代机器人模型。

历史测试与尺寸验证是在本地或服务器具有真实资产的环境中实际运行的；不能将这些结果解释为公开 checkout 已包含资产。新 checkout 的准入命令会报告 `source.STEP_SHA256_NOT_VERIFIED` 和 `MODEL_NOT_READY`；不会启动规划 worker。仅依赖真实本地 STEP 的测试会明确跳过，另有独立的资产缺失拒绝测试。

本地完整资产目录仍为 `assets/robots/huayan_s50_2000/`，本地查看器仍位于本报告目录 `viewer.html`。准备好本地资产后，可在仓库根目录运行：

```sh
python assets/robots/huayan_s50_2000/pack_meshes.py --unpack
python tools/export_s50_model_viewer.py --output docs/experiments/s50_model_integration_20261110/viewer.html
```

没有网格时导出器会报告 `CAD_MESH_FILE_MISSING`，不会生成伪造的 S50 显示模型。查看器模板、导出器和验证代码均保留在公开分支。需要用户确认后，才会另行发布完整模型资产；即使允许发布，关节零位、正方向、工具映射和碰撞覆盖的缺口仍然存在。

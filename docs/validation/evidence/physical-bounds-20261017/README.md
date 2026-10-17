# 实际重放证据

对应报告：[双面 / 三面有限边界复算](../../physical_bounds_20261017.md)。

- `baseline/`：未修改 50a9701 的实际生产几何重放。
- `finite-v1/`：只修复轮廓定位时的中间结果，保留错误接受。
- `delivery-geometry/`：最终生产结果。JSON 仅无损 gzip 压缩，没有改写历史记录。
- `regression.*`、`six_boundaries.*`：全部实例和同轴有符号边界误差；GT 仅事后评价。
- `mask-2-edges/`、`mask-6-edges/`：原生像素诊断图；`index.json` 列出来源裁剪和逐边原因。红色候选线不是正式完整体。
- `final-invariants.json`、`execution-audit.json`、`geometry_run_identity.json`：冻结输入、实例映射、执行源码和后续阻断检查。
- `pytest-baseline.txt`、`pytest-final.txt`：完整测试均有三个原有失败；`delivery-tests.txt` 为服务器 CPU 针对性测试。
- `viewer-check.json`：代码绘图分支和静态资源检查；真实浏览器交互未运行。

`manifest.json` 记录本目录文件及完整交付包的 SHA-256。完整 HTML、RGB 及几何数组在本地 `outputs/physical-bounds-20261017/delivery-final.tar.gz`；解压后打开 `review-final/index.html`。重新求解仍需上一轮 `notch-multiface-20261016` 的冻结采集和 SAM 数组。本轮无新采集、DINO/SAM 推理、模型下载或依赖升级。

最终真实完整体准确恢复为 **0**；这不是几何精度达标的证明。两个代表保留可靠平面和未确定边界，圆角模型偏差、底边缺失及提面遗漏均未用 GT 补偿。

# 近似长方体实际重放证据

对应[结果与复跑报告](../../approximate_cuboid_20261017.md)。

- `strict-baseline/`、`uncoupled/`、`final/`：三路实际生产重放的全部几何 JSON（无损 gzip）、运行身份与状态。
- `primary_result.json`：mask 6 的六边界、三路独立网格指标、完整近似模型与全部局部残差/稳定性记录。
- `six_boundaries.*`、`regression.*`：全部实例，保留双面负向、来源歧义和单面状态。
- `primary-*.png`：原图、青色近似模型、等米制点云叠加；不画 GT，不代表碰撞包络。
- `execution-audit.json`、`final-invariants.json`：冻结输入哈希、proposal→mask 映射、测量域不变及 unknown/执行阻断检查。
- `pytest-*.txt`、`delivery-tests.txt`：修改前后完整测试及服务器 CPU 针对性测试。
- `refinement-probe*.json.gz`：早期距离/权重实现与修正后的数值探查，无损压缩；不替代生产重放或精度证据。
- `viewer-check.json`：静态资源和绘图代码路径检查；无可用浏览器，真实交互 NOT_RUN。

完整 HTML 和全部数组在本地 `outputs/approximate-cuboid-20261017/delivery-final.tar.gz`；解压后打开 `review/index.html`。复跑需要此前已交付的原采集与冻结 SAM 数组。完整包同时保存首次 refined 运行的扁平 K 接口失败日志。`manifest.json` 给出包哈希、各证据哈希及文本换行归一化哈希。

本开发样本的近似尺寸通过原独立精度门槛；原始过渡区域仍有明显形状残差。近似模型不是实测尖角，不保证真实保守外包络，不能用于开放执行资格。

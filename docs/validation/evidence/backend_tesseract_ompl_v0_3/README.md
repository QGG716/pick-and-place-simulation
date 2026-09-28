# v0.3 独立证据

- `engineering-request/`：唯一冻结 FANUC LazyPRM 请求；request.json 在搜索前记录 setup 后参数，native-result.json 保存完整路径，authority.json 为唯一权威复检。
- `engineering-summary.json`：由原始记录提取的计数、结果和分项时间。
- `checker-identity.json` 位于 engineering-request/：v0.2 旧路径证据复用身份，无新完整路径审计或历史节点注入。
- `lazy-unit-tests.json`、`rrt-selection-tests.json`、`checker-regressions.json`：真实 worker 记录；前两项使用明确的 XY 单元场景，不能代替 FANUC 结果。
- `directed-tests.log`、`additional-tests.log`、`identity-test.log`：保留全部原始测试结果和一次归档缺失失败，汇总见 test-summary.json。
- `validated-source.tar.gz`：工程请求实际执行源码及最终测试源码；initial-tests-source.tar.gz 为首轮测试源码。provenance.json 绑定 worker、代码和库身份。
- `upstream-source-review.json`：实际安装头文件、源码出处、SHA256 和首解/采样统计核对；官方源码原件保留于其中记录的服务器目录，不加入项目算法实现。
- `baseline.json` 和 `preceding-local-preflight.json`：起始 Git 状态与此前本地受依赖限制的预检；本地合同测试不是原生验证。
- `compressed-evidence.json`：较大 JSON 的无损压缩映射，含原始字节 SHA256；服务器保留未压缩原件。evidence-index.json 索引最终仓库证据字节。

计数包含缓存命中/未命中、初始化采样与建图迭代的区别。632 次 sampler 调用含 100 次默认投影初始化，实际 LazyPRM 迭代为 532；UNKNOWN 边不代表通过。所有本轮 native 记录使用 provenance.json 中的本轮二进制，v0.2 复用记录保留原二进制身份。

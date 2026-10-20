c00 完整规划验收 1/1
delta=+0.331875 m；Hmin=1.75 m；接近前降至 Hmin，在实际末态检查通过后抽离。
18 个预测有效 / 18 个命令 / 18 个实际 contact FK 几何有效杯。
保持全部 72 个物理杯；有效杯承载足够为 ideal_independent_cups 理想假设。
37→36 箱，c04 和下方 35 箱保留；前序三箱是历史，不计本轮成绩。

先打开 c00-complete.html：本次实际原生机器人轨迹、同源机构升降、偏心附着、
沿 −X 抽离、同位姿解除附着、名义下落、工具分离、理想清空的完整路线。
c00-complete.mp4：640×360，5 fps，1×；181 帧，36.2 s。
视频包括机构演示时间和事件停留；不是硬件周期或物理录像。

eccentric-inspection.html：切换 pregrasp/contact/extraction/withdrawal，
在下方原生请求列表选择本次实际请求；显示 J2_link→J3_link 结构方向、q2 与仰角，
真实面中心、偏心参考和全部 72 个杯。contact 成功请求显示 18 个实际几何接触杯。
“显示见证”重现当前请求原始受检 q 并高亮碰撞对象；不是接受轨迹。
60 个未派发偏移仅为静态几何预览，不移动机器人，不算规划候选。
HTML 完全离线，不需要启动服务器。

本轮正式实测源码：13ce5f88bb65e26f29fed2022a5d24c1ab9a6911
查看器源码：9d701375b3d9df132559e4302491a2498641ff3d
worker SHA256：5aaff6025df88c7c931e25e5bc2d2d7ff11e22a102c19c8dc2c3b5d6a92b0f29
seed=71070，任务预算 300 s，共享前缀预算 90 s。
本轮七个候选，六个未成功，第七个完整成功；规划墙钟 98.927871 s。

formal-raw.tar.gz：完整正式结果、每次尝试、输入/输出、trace、异常、worker 日志与 checkpoint。
formal-result.json.gz：正式完整 result 的原样压缩副本。
run-metrics.json：每次派发的 delta、高度、杯掩码、IK/目标/路径/contact 分级结果，
真实 q 的生产后端结构 FK、软评分、当前碰撞见证、全部请求和候选成本。
summarize_run.py：生成上述指标的只读脚本，不调用求解器。
integration-raw.tar.gz：贯通诊断首版测试断言错误和修正后预期探针退出，分别保留。
production-chain-v2.log：7 个定向测试通过。ProbeComplete 为测试主动停止，不是正式生产异常。
failure-index.json：区分历史 AttributeError、测试断言错误、探针预期停止和正式候选失败。
verification.json：已保存父链、事件、几何、杯掩码、箱体数量和保留对象核对。
viewer-check.json、browser-checks.json：浏览器 FK、接触杯、见证、固定释放 q、最终箱数检查。
measured-source-manifest.json / delivery-source-manifest.json：源码身份和逐文件 SHA256。
source-check.json：实测与交付的 196 个规划核心/原生/配置文件逐字节相同。
manifest.json：仓库交付文件 SHA256；离线包内 bundle-manifest.json 核对包内各文件。
归档和离线包在生成后逐字节核对。

完整复现命令及分析位于仓库 docs/m710_c00_resume_20261020.md。
同一分支执行 git log -1 --format=%H -- docs/m710_c00_resume_20261020.md 可取得最终证据提交。
输出目录必须全新，禁止覆盖本轮记录。未将任何旧 c00 成功数据注入本轮。

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false
contact 是实际返回 q 的规划几何复核，名义首次触带和理想清空都不是物理测量。

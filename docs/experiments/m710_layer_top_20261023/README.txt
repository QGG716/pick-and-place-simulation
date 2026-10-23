从l02_c01所在原始层向下开放顶吸：实际续跑完成，新增0/12，累计仍28/40，剩余12箱。
正式实测源码：8770a4ad99bf35f9346ac321b940c880089c097c。最终交付提交和推送状态见聊天/git日志。
打开failures.html查看本轮实际派发的顶吸接近目标和原生起点；没有目标q、接近轨迹或碰撞见证。

策略
按冻结原始箱体几何，以l02_c01箱顶900 mm为包含边界，开放其原层及下层15个原始箱的顶吸权限；本轮其中12箱剩余。
未解析箱名层号授权。保留原正面/侧面业务选项；顶吸作为本次新增选项优先，沿用300 s任务/90 s共享前缀预算。
累计−X500 mm、−Y100 mm布局、纵带宽600 mm、横带剪叉中心距260 mm均保持。
配置最低带高250 mm保留；顶吸使用当前已受检末态550 mm固定横带，不尝试构造250 mm剪叉。
路线为实际contact附着后先+Z分离，再−X转移至完整接收区域；不横移箱体、不升带。
理想独立杯吸力足够假设保留；实际contact和命令杯完整环封检查、IK、位置限位、全部速度/加速度/减速度、碰撞和完整释放后缀检查保留。

实际结果
5个暴露且可移除的箱体均派发了本轮TOP_FIXED_CURRENT_RECEIVER候选：l02_c01、l02_c00、l01_c02、l01_c03、l01_c04。
总计2081次原生IK请求，数值IK成功0；目标碰撞通过0、接近通过0、contact通过0、完整抓放接受0。
5箱均在约90秒共享前缀预算内未求得顶吸接近IK，不是程序异常，也没有本次碰撞见证。
因此先+Z再−X路线未被执行/验证；不能从本次结果断言这段抽离路线不可行，不能将有限预算失败称为全局不可达。
没有实际contact有效杯，未发生附着、释放、名义下落或理想清空。预估/命令杯不计为实际contact。
剩余7箱因上方支撑依赖未搜索：l00_c00..c04分别支撑l01_c00..c04；l01_c00、l01_c01分别支撑l02_c00、l02_c01。
本轮顶吸耗尽前缀预算后，其余合法正面/侧面选项未实际重新派发，逐箱未搜索数量见result.json；不拿上一轮失败作为本次碰撞原因。
历史28箱单列；这次没有移除任何箱体，不重新注入旧成功解或旧轨迹提示。

成本和验证
正式环路454.560571 s，嵌套单箱任务合计453.604528 s（不相加）；原生请求2106次。
实际搜索候选5个，数值初值/IK次数不是业务候选数。每箱请求成本、最后原生返回、未搜索范围见all-targets.json。
本地与GPU同组38项针对性测试通过；本地首次误用未含pytest的系统Python后改用已有venv，未安装或升级依赖。
正式入口创建PinocchioHppFclBackend，12箱、28历史台账、实际q及550 mm同源机构初态通过检查；无旧原生父链。
完整保存的原生请求、失败、源模型、预算与支撑关系经verify_m710_population.py通过。
失败查看器逐箱q、生产FK见证、场景、零轨迹/零附着/零碰撞高亮检查通过，见qa/browser-checks.json。

复现
冻结源包在artifacts/m710_layer_top_20261023/measured-source.tar.gz，source-receipt.json记录SHA256和资产哈希。
python docs/experiments/m710_layer_top_20261023/reproduce/prepare-source.py 可从固定git源码及已保存的min250源包重新打包。
GPU使用现有rootfs /root/autodl-tmp/m710-l05-x500-20261022/rootfs，依赖不升级。
chroot内正式入口：bash /layer-top-20261023/formal-v1.sh /layer-top-20261023/reproduction-new
完整参数见reproduce/formal-v1.sh；seed=71070，原生worker和trace库哈希见source-check.json。
输入为/min250-20261023/formal-v2；tools/layer_top_continuation.py固定核验前序summary、protocol、verification及每个task result哈希，仅恢复已接受末态，清空历史树。
原始证据formal-v1-raw.tar.xz内嵌file-sha256.json；含5箱所有候选、原始请求/模型/日志、测试和初态检查，原失败不覆盖。
运行源码环境/work-layer-top-8770a4a；查看器导出脚本放在独立结果目录，没有修改实测源码环境。
顶吸失败查看器导出（GPU现有环境）：
PYTHONPATH=/work-layer-top-8770a4a/src:/work-layer-top-8770a4a/tools /opt/planning-venv/bin/python /layer-top-20261023/export-top-ik-failures.py /layer-top-20261023/formal-v1 /layer-top-20261023/analysis-v1/failures.html --source-root /work-layer-top-8770a4a
浏览器检查：node tools/check_m710_top_ik_failures.mjs docs/experiments/m710_layer_top_20261023/failures.html artifacts/layer-top-browser "C:/Program Files/Google/Chrome/Application/chrome.exe"

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false

250 mm续跑结果：本轮8/20完整抓放接受（40%）；历史20箱单列；累计28/40（70%），剩余12箱。
实测源码：阶段A a300d234633e122c6d8328dddb259f3c9c38ad91；修复后阶段B 3f449e89da518ac89b3c112b4a83eb9a83000b11。
最终交付提交及推送状态见聊天回复和git日志；实测规划源码与展示/交付提交严格区分。

打开 index.html 查看报告；failures.html 为12箱失败原因三维查看器。
accepted-v1.html 回放阶段A接受的3箱；accepted-v2.html回放阶段B接受的5箱。
两阶段之间因真实集成错误重建原生根状态，不伪装成同一原生父链。
所有HTML均内嵌模型和渲染库，可直接离线打开；offline-replay.zip包含报告、查看器及精简结果。

本次策略与结果
累计安装位置−X500 mm、−Y100 mm；纵带0.60 m、横带剪叉中心距0.26 m保持。
最低横带表面配置250 mm；箱顶严格低于250 mm才顶吸。最底层箱顶300 mm，故本轮无人使用顶吸。
8个成功目标均正面抓取delta=0，48个几何有效杯、48个命令杯、实际contact48杯。
采用ideal_independent_cups理想吸力足够假设，不加承载杯数门槛；保留IK、位置、全关节动态限值及碰撞/工艺检查。
实际接收高度550或850 mm，详见result.json的逐箱delta、临时高度、同步位置、q2和上臂仰角。
l03_c02完整通过，先在550 mm接近，在pregrasp受检升到850 mm后完成后续抓放。
末态横带550 mm；请求250 mm未被机构实现。

剩余12箱
l02_c01：交汇区要求Hmin250 mm，下降采样在约360 mm发生SCISSOR_GEOMETRIC_RANGE_EXCEEDED。
l02_c00：同上；Hmax550 mm内侧抓取候选在当时13箱现场被l02_c01、l02_c04遮挡；临时250 mm选项也被剪叉几何拒绝。
l01_c02、l01_c03：Hmax250 mm剪叉无法构造；Hmin−50 mm在配置范围外。
l01_c04：同上，另一个内侧Hmin选项也为−50 mm、范围外。
l00_c00..c04分别仍支撑l01_c00..c04；l01_c00、l01_c01分别仍支撑l02_c00、l02_c01。7箱未派发原生搜索。
这些最终失败没有IK/运动目标或碰撞见证，不称为机器人IK不可达。历史失败搜索与成本在raw中独立保留。
250 mm请求下：上铰点=250−120−70=60 mm，下铰点100 mm，两级每级升高−20 mm，违反>35 mm构造条件。
仅此条件要求带面高于360 mm；这不是360 mm以上所有构型安全的证明。
三维查看器使用每箱当时实际根状态q、同源机构和箱体。青色250 mm仅为请求带面及高度示意，未捏造无法构造的剪叉或机器人轨迹。

真实集成错误及修复（均保留）
1. 初始打包遗漏已跟踪测试fixture：GPU测试30通过1失败；补入原始fixture后31通过，随后才启动正式v1。
2. 新min250策略的普通非交汇区选项缺少“接近前下降到同一既定接收高度”的路径；补入该既有高度选项，不改抓面或机构。
3. v1已提交3箱后，l02_c04在contact后抽离入口抛出RECEIVE_MECHANISM_WORLD_CHANGED。
   起止高度0.5499999999999998与0.5499999999999999只差浮点尾数，Python误用容差跳过同步而原生要求完整世界JSON严格一致。
   修复为只有完整机构世界完全相等才跳过，否则走现有受检原生外轴转换。没有放宽原生检查或给坐标取整。
   v1失败任务007及全部记录保留；v2仅恢复3个已提交末态，清除旧原生父链/搜索树，不用旧q作为求解目标。
4. 修复后本地及GPU各38项针对性测试通过；正式v2完成其余5箱后正常退出并保留几何失败。
   对上述精确同步错误另做生产后端隔离回归：重新生成l02_c04完整周期，2个外轴检查采样通过；10.55 s、17次原生请求。
   隔离回归人口完成计数为0，不计为第9箱。

成本与分阶段结果
正式v1环路197.96 s，v2环路99.37 s，合计297.32 s；任务嵌套成本279.05 s，不能与环路相加。
正式共593次原生请求、14个真正派发IK/运动的候选；另1候选仅worker初始化/FK后被面筛选拒绝。
数值IK成功43次；显式目标检查通过17次；接近通过9次；contact通过9次；完整抓放仅8次。
IK或前缀的成功可能属于随后失败候选，不能计为完整箱数。加上隔离同步回归共610次请求，回归计数单列。
all-attempt-costs.json包含各操作、每候选、各任务成本及未提交异常任务；旧分析表中的dispatched_candidates是初始化数，最终口径以该文件为准。
保留有限候选与原共享预算，未把预算内失败称为全局不可行。未搜索的支撑阻挡箱为7箱；没有额外扫底座、尺寸或规划器。

复现
使用既有GPU chroot /root/autodl-tmp/m710-l05-x500-20261022/rootfs，原生worker、trace库和ROS依赖哈希在records/v2/analysis-source-check.json。
不用升级依赖。冻结v1/v2源码打包收据分别为source-receipt-v2.json、source-receipt-v3.json，完整资产由既有前序measured-source.tar.gz提供。
源码源包仍保留在artifacts/m710_min250_20261023/prepared-source-v2.tar.gz与prepared-source-v3.tar.gz。
初始准备命令与旧GPU不可用记录保留于preparation-readme.txt和preparation-delivery-status.json，属于历史准备状态。
重建冻结源包：在仓库根执行python docs/experiments/m710_min250_20261023/reproduce/prepare-repair-source.py。
正式v1命令见formal-v1.sh；修复后命令见reproduce/formal-v2.sh。它们拒绝覆盖已有输出目录。
在已部署chroot内复现修复版本：bash /min250-20261023/formal-v2.sh /min250-20261023/reproduction-new
旧初始20箱来自previous-formal-v1；修复追加3箱来自formal-v1。输入根状态恢复哈希由tools/min250_continuation.py严格核验。
阶段A原始包raw/formal-v1.tar.xz含所有97文件（包括未提交task007）；阶段B包含109文件。SHA256清单在压缩包内。
修复回归脚本见reproduce/sync-regression.py，完整结果raw/sync-regression.tar.xz，计数禁止并入批次。
重新生成失败查看器：
python tools/export_m710_min250_failures.py docs/experiments/m710_min250_20261023/raw/formal-v2.tar.xz docs/experiments/m710_min250_20261023/accepted-v2.html artifacts/min250-inspection/failures.html
查看器QA：
node tools/check_m710_min250_failures.mjs docs/experiments/m710_min250_20261023/failures.html artifacts/min250-browser-check "C:/Program Files/Google/Chrome/Application/chrome.exe"
12箱原始q、生产FK见证、现场箱数、零轨迹、零碰撞见证逐项通过；两组成功回放浏览器检查通过。未重复物理执行。

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false

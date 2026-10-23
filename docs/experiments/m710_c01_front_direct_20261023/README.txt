c01正面吸取、直接−X：本次0/1，未完成。没有+Y侧移。
当前横带250 mm、连接件加长100 mm及其他设定全部保留。现有交汇区规则要求Hmin=-50 mm，低于机构最低250 mm，正式入口在工艺高度筛选拒绝。
这不是IK失败或碰撞失败：本次IK=0，原生运动请求=0，实际contact未到达；命令杯/实际杯均未计算。理想独立杯吸力假设不改变接收高度规则。

为什么直线方案被拒绝
箱底z_b=300 mm，箱高h=300 mm，工艺余量d=50 mm；原有Hmin=z_b-h-d=-50 mm。Hmax=z_b-d=250 mm。
c01不改变Y时，箱体完整Y投影[-620,-220] mm与纵带交汇走廊[-1100,-500] mm重叠120 mm。
判定使用完整箱体投影，不按箱体中心、不按名字。正面吸取朝向+X，出料−X，因此现有JUNCTION_FRONT_REQUIRES_HMIN规则适用。
正式候选为FRONT_DIRECT_HMIN，方向仅[−X]，要求高度−50 mm；允许范围[250,2100] mm，返回RECEIVE_HEIGHT_OUTSIDE_EXISTING_MECHANISM_RANGE。
入口先核验同源机构和生产Pinocchio/HPP-FCL起点，再执行上述工艺筛选。没有向机械机构发送负高度，没有启动IK/轨迹搜索，没有静默将Hmin改为Hmax。
该结论只是当前工艺规则与高度范围冲突，不是证明正吸−X在运动学上不可行。接近、contact、附着、直线抽离、释放和分离的运动可行性均未搜索。

已完成、但被用户路线纠正取代的独立结果
在收到“直接−X”纠正前，正面吸取、先+Y再−X的同设定方案已完整通过，1/1；它单列在superseded-y-x/，不计入本次直线结果。
该方案实际+Y约120.198942 mm后转−X，横带250 mm，偏移delta=0；杯数见result.json和accepted-cycle.json（三份独立掩码均完整保存）。
预测/命令/实际杯：{'actual_contact_mask': 48, 'commanded_active_mask': 48, 'geometrically_eligible_mask': 48}。保持ideal_independent_cups，不加承载杯数门槛。
已经通过原生接近、contact及实际末态杯检查、附着、两段抽离、完整接收区域验收、同位姿解除、名义下落、首次触带、工具分离和IDEAL_RECEIVER_DRAIN。
名义下落约50 mm期间工具q不随箱体下降；未进行真实动力学接收或物理出料。离线回放由本次真实原生q和父链驱动。
17个原生请求；3个数值IK调用（2成功、1失败）；先前被拒绝的接近路径及其工具/横带净空见证已保留。最终成功路径是当前run重新生成的局部恢复结果，不导入历史轨迹。
该独立规划世界8→7箱、历史32→累计33；最新直线试验仍从原历史32箱的同一8箱初态建立，没有把先前成功末态当成直线任务初态。

成本与验证
直线：任务0.239114985 s，调度循环0.316044912 s；1个业务候选被工艺筛选，0个目标搜索候选，0碰撞见证。
先前两段：任务12.314407870 s，循环12.945441112 s；完整路径时长56.803458735 s（名义回放时间）。
计时嵌套，不能相加。保留seed71070、任务300 s/共享前缀90 s的资源约束，未重设预算、改初态或加固定尝试上限。
直线GPU定向测试34通过（4.40 s），本地33通过/1生产后端测试跳过（4.52 s）；两段GPU33通过（4.96 s）。原始结果验证均PASS，源码哈希核验均PASS；成功回放浏览器验证PASS。
直线首次启动脚本误写--c01-direct-suction，被argparse拒绝，尚未建立任务；保留原formal-v1.sh及production-check.log。修复脚本为formal-v2.sh后重新执行生产检查与正式入口，未改实测源码。它是启动配置错误，和后续工艺拒绝分开记录。

源码、复现与交付
最新直线实测源码：75f98fce8e68bd807c84590097782c638eb36629；冻结目录/work-c01-direct-75f98fc；结果/c01-direct-20261023/formal-v1。
先前两段实测源码：94e249a3cd58930727a41e5d9556039427c23653；冻结目录/work-c01-front-94e249a；结果/c01-front-20261023/formal-v1。
在原GPU隔离环境用 reproduce/formal-v2.sh 指定一个不存在的输出目录复现直线入口：--c01-front-direct-x --c01-front-suction --tool-connector-extension-m 0.1；不要使用保留的错误formal-v1.sh。
完整命令、原worker哈希、资产/源码清单与失败记录均随交付保存。formal-v1-raw.tar.xz含内部逐文件SHA256清单，全部核验。
最终提交及推送状态在交付回复给出。两份结果绑定各自源码，不混计、不覆盖，不将展示模型或静态投影当作搜索结果。

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false

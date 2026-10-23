本轮 l01 完整抓放：2/5；历史30箱单列，累计32/40，现场保留8箱。
本轮采用原正吸/侧吸业务规则，增加用户明确批准的“仅l01工具与chassis接触”仿真规划豁免。
carton_l01_c02：delta=0 mm；接收高度=250 mm；同步位置=PREGRASP；预测/命令/实际均48杯；q2=1.292030 rad；上臂仰角=15.972°。
carton_l01_c03：delta=0 mm；接收高度=250 mm；同步位置=ALREADY_AT_RECEIVE；预测/命令/实际均48杯；q2=1.286604 rad；上臂仰角=16.283°。
承载采用 ideal_independent_cups：所选有效杯吸力足够，不增加最低承载杯数门槛。

l01_c02：完整抓放接受；正吸，Hmax=250 mm，delta=0；沿−X抽离810.2 mm，释放后名义下落50 mm、工具分离和理想清空全部通过；实际有效48杯。；单箱任务9.529516205 s。
l01_c03：完整抓放接受；正吸，Hmax=250 mm，delta=0；沿−X抽离810.2 mm，释放后名义下落50 mm、工具分离和理想清空全部通过；实际有效48杯。；单箱任务11.582766049 s。
l01_c01：未完成；所需Hmin=−50 mm，低于最低带高250 mm；工艺几何筛选拒绝，未派发IK或轨迹搜索。；单箱任务0.404066667 s。
l01_c00：未完成；正吸Hmin=−50 mm越界；允许的+Y侧吸面仍被l01_c01、l01_c04遮挡，未派发IK或轨迹搜索。；单箱任务1.456280567 s。
l01_c04：未完成；接近/contact及实际48杯几何通过；抽离LIN的91个端点尝试均失败，90 s阶段预算耗尽。J2—左壁728条重复见证（19个不同q）：273条净空0.87–4.66 mm<5 mm，455条接触/相交；未到释放。；单箱任务103.116322212 s。

实测源码：e9dfef3215b55bb9905ed0671d43ca4b9716a71a
继承前序：f17d636ed547a86389150f268445d44bae6f58c4 的已验收30箱末态，输入 /lowprofile250-20261023/formal-v1。
初态10箱、250 mm带高、q、历史台账逐记录重建；不注入旧成功IK、轨迹或attachment；清空旧原生父链后建立本轮根。
这不是从40箱初态独立运行得到的当前策略成功率。l00五箱在场景中保留，本轮不派发，也不计入5箱分母。

权限和不变规则
工具权限由冻结原始40箱几何选定箱底0.30 m、箱顶0.60 m的一层；恰好5箱。未通过名字解析层号。
只允许当前目标属于这一层时，模型拥有的工具碰撞体与精确chassis对象接触；原生worker绑定工具所有权和策略指纹。
保留已有base_link及J1..J6对chassis豁免；不扩大到载荷、邻箱、墙、输送带、剪叉或其他自碰撞对象。
CPU固定环境缓存区分目标权限；原生豁免仅接受planning_only=true和cold_from_scratch。
全部IK、关节位置/速度/加速度/减速度、实际contact杯几何、父链和抓放工艺规则保留。
保持安装累计−X500 mm、−Y100 mm，纵带0.60 m、横带剪叉中心距0.26 m，low_profile_250_v1最低带高250 mm。
本轮没有继续使用上次顶吸专试；箱顶严格低于最低带高才自动采用顶吸，因此本轮l01仍走正吸/侧吸规则。
seed=71070，单箱300 s，共享前缀90 s，IPC100 s；无新增固定IK/RRT尝试上限。
原生记录注明seed作用于OMPL；KDL内部随机性并不由该seed完全控制，不宣称跨次搜索次数逐位确定。

证据与成本
全部结果区分数值IK、目标检查、接近、contact和完整抓放，详见all-targets.json及records/verification.json。
本轮138次原生请求，9次数值IK中6次成功；5次显式目标检查通过；3箱接近及contact通过；2箱完整接受。
真正进入IK/运动搜索的业务候选只有3个；另1个worker初始化后因侧面遮挡拒绝，不把它计作运动搜索候选。
c04抽离阶段91次请求均失败；728条J2—左壁见证来自19个不同q，不是728个独立候选。
c04接近时也有工具—横带的8条失败分支见证，但后来接近已通过；这些不是抽离失败的原因。
INTERSECTION_OR_CONTACT的负值是碰撞哨兵，不是穿透深度；未生成碰撞接触点，不把代理距离称作CAD穿透证明。
c02的PREGRASP外轴检查从0.25到0.24999999999999994 m，仅浮点端点差，没有实际升降；c03已在接收高度。
回放中的4秒机构演示时长不代表实测硬件升降时间。
完整接受要求同一箱经过实际contact末态附着、抽离、区域验收、同位姿解除附着、名义下落、名义首次触带、工具分离、IDEAL_RECEIVER_DRAIN。
名义下落时机器人保持释放q；理想清空是假设，不是物理传送、实测接收或Isaac执行。
任务循环128.906200208 s，嵌套单箱任务合计126.088951699 s，不能相加。
全部请求成本、候选初值、目标构型、成功轨迹和失败见证在原始记录中；静态预览未计入规划候选。
未搜索部分包括本轮范围外的l00五箱及当前预算之外的构型/路线；预算内失败不等于全局不可达。

测试夹具错误单列
57e0957 的原生回归把测试对象对写成JSON对象，触发 PROCESS_ALLOWED_PAIR_INVALID，尚未开始正式规划。
e9dfef3 仅修正测试数组编码并重新构建；失败日志和源码清单保留在integration-v1/，不是5箱规划失败。
本地18项针对性回归通过；GPU针对性Python与原生reference/optimized检查见records/。
正式PinocchioHppFclBackend初态检查和原生链路已实际运行。source-check.json逐文件核对本次运行源码未变。

复现和交付
GPU rootfs：/root/autodl-tmp/m710-l05-x500-20261022/rootfs。
进入chroot：bash /l01-tool-chassis-v2-20261023/formal-v1.sh /l01-tool-chassis-v2-20261023/reproduction-new
输出目录必须不存在；完整参数见reproduce/formal-v1.sh。需保留已验前序 /lowprofile250-20261023/formal-v1。
本轮worker和trace库独立构建在 /l01-tool-v2-build 与 /l01-tool-v2-install，未升级环境/依赖。
accepted-cycles.html为本轮接受的原生q和机构运动回放；未派发或失败目标不伪造成功轨迹。
原始记录formal-v1-raw.tar.xz：SHA256=c8e5ba46998d9926b3ae96cb0cf7bbf11e896a502c9ea1c12dd79a95eaea4d3e，逐份校验61份文件。
source-manifest.json、source-receipt.json绑定所有实测src/tools/ros2/configs和模型资产。
最终证据提交SHA、推送状态见本轮交付消息和git历史；实测源码与证据提交明确区分。

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false

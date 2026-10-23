l01五箱正吸/侧吸失败原因三维查看器

打开 index.html。顶部选择箱体，左侧切换原因或对象对，滑条逐个显示原始拒绝构型。
默认打开l01_c02的工具/底座见证；“已通过的contact末态”按钮可对照抽离起点。
拖动旋转、滚轮缩放、右键平移；可聚焦原因、切换全局、隐藏机器人或调整背景透明度。

本页对应用户表格中的正吸/侧吸轮次，实测源码为f17d636ed547a86389150f268445d44bae6f58c4。
不混用后来4337f91顶吸IK失败记录，没有重新规划或改变任何规则。
该正吸/侧吸轮次成功2/12，历史28箱单列，累计30/40；本页l01五箱均未完成完整抓放。

l01_c02、l01_c03：接近/contact通过，实际contact48杯有效；抽离LIN搜索中工具tool_rigid_13/11与chassis出现净空不足以及相交/接触。
l01_c04：接近/contact通过，实际contact48杯有效；工具tool_rigid_13与chassis，以及J2_link与trailer_left_wall出现净空不足以及相交/接触。
三箱分别尝试64、66、102个接收区域端点，最终阶段预算耗尽；不能据此断言全局不可行。
l01_c01：所需Hmin=−50 mm，低于最低带面250 mm，差300 mm；未派发IK。
l01_c00：同样Hmin越界；另一个允许的+Y内侧吸面被l01_c01/c02/c03/c04遮挡，属于面暴露筛选失败，未派发IK。
下方l00五箱仍提供支撑，本页按原始世界保留，不把它们当作碰撞或IK失败箱。

原始净空见证：
carton_l01_c02:
  tool_rigid_13 / chassis: 原始记录192次；正间隙0.501454..4.501430 mm，要求5 mm；另有INTERSECTION_OR_CONTACT。
  tool_rigid_11 / chassis: 原始记录320次；正间隙2.502482..2.502482 mm，要求5 mm；另有INTERSECTION_OR_CONTACT。
carton_l01_c03:
  tool_rigid_13 / chassis: 原始记录198次；正间隙0.501244..4.501223 mm，要求5 mm；另有INTERSECTION_OR_CONTACT。
  tool_rigid_11 / chassis: 原始记录330次；正间隙2.501285..2.501285 mm，要求5 mm；另有INTERSECTION_OR_CONTACT。
carton_l01_c04:
  tool_rigid_13 / chassis: 原始记录306次；正间隙0.499930..4.499930 mm，要求5 mm；另有INTERSECTION_OR_CONTACT。
  J2_link / trailer_left_wall: 原始记录510次；正间隙0.868447..4.659914 mm，要求5 mm；另有INTERSECTION_OR_CONTACT。
红色表示当前选中的原生拒绝对象对；正间隙不足与INTERSECTION_OR_CONTACT分别显示。
FCL负距离标记不是本页可解释的穿透深度。原记录没有最近点或接触点，因此不绘制推测碰撞点。
工具/底座和J2/左壁不属于已批准的“官方机器人连杆/安装底座”例外。

机器人按每个原始q显示，世界与attachment来自同一原生请求。
contact末态与失败搜索构型分开；失败构型是静态检查样本，不是机器人已执行位置，也不连接成合法运动轨迹。
Hmin低于地面的青色带面只作高度示意，实际250 mm剪叉几何保留。
紫色箱为原始面暴露筛选记录中的遮挡箱，不是刚性碰撞见证。
每个场景保留10箱（附着状态为9个世界箱+1个附着箱），没有跳到工具中心或删除失败箱。

验证：原始归档84份文件逐份校验；1856条见证记录去重为79个原始拒绝构型，另有3个contact末态和2个筛选根状态，共84个状态。
全部84个状态的渲染FK与正式PinocchioHppFclBackend的named_link_frames()一致。
浏览器17项展示核验通过，包含5箱、contact/各对象对、世界坐标、附着变换、全部10箱、侧面遮挡及高度示意；零JS错误。
没有增加IK/规划调用；没有生成运动或进行物理执行。核验见source-checks.json、qa/browser-checks.json。

来源：../m710_lowprofile250_20261023/formal-v1-raw.tar.xz
SHA256: 88638eb2ebf8e8f3cac7067f2a1b8a72af9d755af778680f4b6e57d7793b9d20
模型来自同轮accepted-cycles.html，逐项匹配官方URDF、工具、关节、TCP及物理模型身份；各任务独立场景身份保留在原生请求中。
生成工具：tools/export_m710_l01_failures.py，叠加界面：tools/search_replay/l01_failures.js。
导出器和界面SHA256见source-checks.json；最终提交与推送状态见git记录及本轮交付消息。

在现有GPU chroot中复现（导出脚本和JS复制到/l01-failure-viewer-20261023后）：
export PYTHONPATH=/work-lowprofile-f17d636/src:/work-lowprofile-f17d636/tools
export M710_MOVEIT_ASSET_ROOT=/work-lowprofile-f17d636
/opt/planning-venv/bin/python /l01-failure-viewer-20261023/export.py /lowprofile250-20261023/formal-v1-raw.tar.xz /lowprofile250-20261023/analysis-v2/accepted-cycles.html /l01-failure-viewer-20261023/output/index.html --source-root /work-lowprofile-f17d636 --overlay /l01-failure-viewer-20261023/overlay.js
本地界面核验：node tools/check_m710_l01_failures.mjs docs/experiments/m710_l01_failures_20261023/index.html artifacts/m710_l01_failure_viewer_20261023/new-qa "C:/Program Files/Google/Chrome/Application/chrome.exe"

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false

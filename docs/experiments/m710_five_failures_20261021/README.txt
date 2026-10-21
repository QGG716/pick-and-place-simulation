5 箱失败原因：只读三维检查

打开 index.html；左侧选择 l05_c03、l04_c02、l03_c01、l03_c04、l03_c00。
拖动旋转，滚轮缩放，右键平移；支持局部聚焦、全局视角和背景透明度。
也可直接访问 index.html#l03_c01 等箱号锚点。

l05_c03 / l04_c02：保持实际 contact 末态及原生抽离请求的 attachment。
显示每箱 346 个已派发区域端点，线框箱和虚线仅表示请求，不是生成轨迹。
最后一次抽离返回 NO_IK_SOLUTION；预算耗尽，无本次碰撞见证。
l03_c01：显示本次原生拒绝的 pregrasp q；tool_rigid_0 / carton_l04_c02 高亮。
l03_c04：显示本次原生拒绝的 pregrasp q；tool_rigid_0 / carton_l04_c03 高亮。
l03_c00：显示最后 IK 请求的真实起点，场景取自其受检降带末态；没有解或目标 q。

每箱使用各自任务当时的场景。没有重新运行规划，没有补造运动或碰撞点。
接触和附着均是规划记录；实际 contact 杯数指记录姿态下的几何复核，非物理测量。
两处红色碰撞是当前原生碰撞表示的相交/接触见证，不声明测得原始 CAD 穿透深度。
程序预算内未完成不代表全局不可行。

实测源码：f37448f998d0e82401c096a72c198b28e58fd2c2
原始记录：../m710_xy_top_population_20261021/formal-raw.tar.xz
模型：复用该目录 failed-goals.html 中已核验的完整官方模型及工具显示数据。
source-checks.json：读取的每箱源文件 SHA 校验、实际 q/world、碰撞及区域计数检查。
browser-checks.json：五箱浏览器渲染和姿态/场景/高亮检查；两箱对照原生产 FK。

仓库根目录复现：
python tools/export_m710_five_failures.py
node tools/check_m710_five_failures.mjs docs/experiments/m710_five_failures_20261021/index.html docs/experiments/m710_five_failures_20261021 "C:/Program Files/Google/Chrome/Application/chrome.exe"

qualification_status=NOT_EVALUATED
continuous_swept_proof=false
physical_execution=NOT_RUN
execution_ready=false

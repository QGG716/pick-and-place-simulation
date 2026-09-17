# ECO65-B 官方资料本地归档

用户已确认实机型号 ECO65-B；控制器代际、固件及具体法兰修订未确认。
全部原件按官网链接下载，详见 SOURCE_MANIFEST.json 的实际 URL、时间、版本、尺寸和 SHA-256。
原始文件未修改、未转 CAD、未修订 URDF。完整性通过不等于运动学/动力学或实机验证。

- cad/rm_models/ECO65/robot_model：ECO65-B、6F、6FB 原始 STEP，B 为用户型号；其余仅参考。
- description/rm_models/ECO65/urdf：三种变体分别保留原目录、URDF、mesh、配置及 package.xml。
- description/ros2_rm_robot：官网 humble 模型路径中的 ECO65 URDF/xacro、mesh、配置，以及驱动源码参考快照。
- description/RM_API2：API2 Python 接口及 C 头文件参考；未安装/导入厂家 SDK。
- drawings/rm_models/ECO65/dimension：外形、底座孔位和末端转接孔位三个官方 PDF。
- manuals/Dev_Center/RobotGen3 与 RobotGen4：各自 ECO V1.2.0 使用手册，原中文文件名相同但内容哈希不同。
- web/robot 与 web/robot4th：三/四代模型、手册、参数/D-H、硬件接口、ROS2、二次开发页面原 HTML。
- web/attachments：原页面图片，含公式/接口/尺寸资料；HTML 保留原链接，可按清单映射至本地附件。
- web/inventory：官方仓库固定提交的发现清单。没有 ZIP 下载或解压，均为原文件直接下载。

## 完整性与已知缺项

207 个条目已落盘，其中 206 个非空；唯一空文件为上游原有的 API2 `__init__.py`，
已按 Git blob 验证，并非伪造的模型。所有 CAD/PDF/mesh 均非空。
20 个 XML/URDF/xacro/launch 文件可解析，134 个 mesh 引用存在；未展开 xacro或启动 ROS。
下载的原件和图片已校验；所有 GitHub 原文件另按上游 Git blob 校验。

官方许可证资料存在缺项/冲突：rm_models 根 LICENSE 为 Apache-2.0 声明，ECO65
package.xml 写 BSD；ROS2 模型/驱动 package.xml 为 `TODO: License declaration`，
未找到适用的 LICENSE；API2 未提供 LICENSE。没有拿其他示例的许可证替代。
这些原件仅保存在本地，Git 跟踪清单和说明；再分发范围待官方确认。

官方机械库 ECO65-B 与 ROS2 标准 ECO65 URDF 有参数差异，详情见准备报告。
官网四代入口明确说明部分模型尺寸图以三代控制器为例，不据此认定实机控制器代际。
三代页面将 6FB 与老式 6F 分开，四代页面使用不同六维力命名；均保留原文，不混配。

## 固定版本恢复

在已准备的 .venv 中显式运行：

```powershell
.venv\Scripts\python.exe tools/eco65_official_archive.py --restore
.venv\Scripts\python.exe tools/check_eco65_bootstrap.py
```

restore 按现有清单的固定 URL 和哈希恢复缺失文件，不覆盖已存在原件，也不连接实机。
必要时由使用者为当前进程设置 HTTPS_PROXY；脚本不存代理凭据。
`--discover`/`--download` 仅用于首次发现/构建清单，不应在固定来源恢复时调用。
本轮不处理用户 STEP，等待用户明确下一轮指令。

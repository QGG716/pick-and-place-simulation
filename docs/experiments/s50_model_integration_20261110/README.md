> 发布范围：用户已明确授权发布真实 STEP、派生网格、嵌入网格的离线查看器及官方手册；这些资产随本次追加提交纳入。用户授权不等于厂家授予再分发许可，原有来源与许可说明继续保留。历史代码/报告版为 `cff77ce`，本次资产清单与验证见 [资产发布验证](asset-publication-validation.json)。

# S50-2000 接入阶段报告（2026-11-10）

**当前状态：真实 CAD 已取得并核对，接口与布局冻结已实现；机械臂替换尚未完成。**

本轮从 `a3f08cc2283138e5905b4cc9829cefd7b4aac6b7` 创建独立 worktree 和 `feat/v0.5-s50-moveit2-native-single-carton`。没有改动源分支、复用 FANUC 卸箱检查点，或把其他机型改名为 S50。因关键运动学和碰撞资料尚未核定，按用户指定的缺资料处理方式保留可验证成果，S50 准入入口明确返回 `MODEL_NOT_READY`（退出码 2）。

此前发布的 [`cff77ce` 代码/报告版](https://github.com/QGG716/pick-and-place-simulation/commit/cff77ce) 未包含完整模型资产；该版本的历史测试和当时的资产缺失处理记录保持原样。本次追加纳入获授权的资产与查看器，不把资产发布当作运动学、碰撞或规划验证通过。

## 已完成

- 用户提供的真实 STEP 按底座、J1–J6 拆成七份米制网格；保留原文件、装配变换、网格哈希、转换脚本及失败面记录。
- 从相邻部件解析圆柱测得六条无向转轴。官方机械手册与 CAD 的 13 项尺寸交叉核对通过，最大差约 0.0001984 mm；这不是控制器零位/正方向的验证。
- 提供配置化 `RobotModelBinding`、`URDFRobot.from_config` 和 native 模型输入分支，同源读取关节、frame、URDF/SRDF 和网格。安装变换、工具变换、腕部碰撞规则必须显式绑定；保留 FANUC 加载路径。
- 冻结实际运行布局、完整 40 箱及原工具，不搜索 gap、安装点、安装高度、工具长度或输送带尺寸。
- 提供离线 [查看器](viewer.html)、[使用说明](viewer-usage.md)和机器可读 [查看器检查](viewer-checks.json)。可切换完整布局、目标接触面及真实 CAD 装配；模型未就绪时六关节控件禁用，不显示虚构轨迹。

## 冻结布局

| 参数 | 固定值 |
| --- | --- |
| 横带前沿至箱堆前面 | 700 mm |
| 真实安装轴线/安装面世界坐标 | `[-1.85, 0.15, 0.90] m` |
| 安装朝向 | 与基线相同；世界安装 frame 旋转为单位矩阵 |
| 安装轴线至底盘 +X 前沿 | 450 mm |
| 横带高度 | 550–1500 mm，初始 600 mm |
| 吸具连接延长段 | 200 mm |
| 世界坐标 | +X 朝箱堆深处，+Y 向左，+Z 向上 |
| 箱体初始状态 | 完整 40 箱；历史完成 0，假定移除 0 |
| 工具/单箱质量 | 20 / 42.5 kg，未降低 |

冻结依据为基线 `m710_gap700_front_working_20261110` 的 README、launch、checked home，以及 `gap_search_layout.py`，并核对已检查初态和布局身份。旧 FANUC 外壳前沿和底座包络不写入 S50 定义；S50 的未知 mounting transform 不默认为单位变换。

本地和 GPU 隔离目录的非机器人布局对比均 **PASS**：四个高度（0.55/0.60/1.00/1.50 m），每个高度 148 个实体；另比较尺寸、机构行程、40 个箱体身份和工具固有几何/安装尺寸/TCP。工具随机械臂运动后的世界位姿不参加等值比较。

压缩前 `4464b04` 快照的聚焦回归：**Windows 本地 62 passed；GPU 服务器 Linux 62 passed**。该次服务器逐项核验 202 个输入文件 SHA256，无不匹配；见 [服务器摘要](server-summary.json)、[服务器准入记录](admission-server.json)、[本地准入记录](admission-local.json)和 [布局对比](layout-freeze-report.json)。这组结果不自动代表后续查看器压缩改动已经在服务器复测。

查看器压缩及 OBJ.gz 回退读取完成后，本地重新运行相同六个模块，**65 passed（其中查看器 11 项）**，JUnit 保存在 `outputs/s50_model_integration/compressed-final-local-tests.xml`；新增三项覆盖仅含 gzip 网格的 checkout 导出等值、压缩损坏拒绝与解压后 SHA 不匹配拒绝。最终补丁在 GPU 服务器 Linux 上复测 **65 passed in 10.73s，TESTS_EXIT=0**，补丁包 SHA-256 `5139174ca2ed5d053b67248eddb5f312237df58f0c6bf4b5075dab93596709bd` 核验通过。测试时间、源码补丁 SHA 和结果见 [压缩验证记录](viewer-compression.json)。测试覆盖接口、布局、故障拒绝、查看器导出与 FANUC 回归，不将解析 fixture 当成真实 S50。

Windows 与 Linux 的原始环境指纹分别保留；两者仅有六个剪叉派生角度的末位浮点差异，最大约 `1.11e-16 rad`，实体几何和位姿没有差异。不能将原始哈希写成相同。S50 完整场景指纹仍为 `null`，因为尚无合格运动模型；配置身份不冒充完整场景身份。

## 关键阻塞与真实运行范围

详细来源、13 项尺寸、版本、许可说明和失败记录见 [模型来源报告](../s50_model_sources_20261110/README.md)及 [模型资产说明](../../../assets/robots/huayan_s50_2000/README.md)。

1. STEP 装配姿态和六条无向转轴已知，但尚无可核验的六轴控制器零位、正方向及参考姿态。官方手册的外形图没有提供这些映射，不能据此猜六轴角度。
2. CAD 的真实底部支承面在原始 Y=-3 mm，安装面到 J2 为 283.3 mm，已由图纸核对。绕安装轴的定向、完整 flange/tool0 定义及原吸具物理接口对应仍须确定；没有移动安装点或增加虚拟工具长度。
3. J1 原始拓扑检查失败；J1/J2 各一面未三角化。实际 ShapeFix 尝试没有改善。静态显示网格不能据此认定为完整碰撞模型。
4. 官方当前网页和机械手册的 J1/J2/J6 最高速度不同，已分别记录，尚未核定本 CAD 对应版本。

S50 初态 IK、自碰撞/环境碰撞、多姿态核心/native/viewer FK、代表顶层单箱规划均为 **NOT_RUN**，不是规划失败，更不是结构不可达。代表箱由完整场景最高表面及中心位置选择，具体身份在准入报告中；没有生成成功抓放记录或启动全 40 箱搜索/Isaac。

官方额定载荷记录为 50 kg；原工具加箱体合计 62.5 kg，额定负载资格不满足。这个结果与几何搜索独立，没有用它提前结束几何资料和接口工作。缺失逐连杆惯量/真实力矩另列为动力学缺项，不阻止单独的运动学建模，也不声明动力学通过。

新 native 分支当前完成模型输入与 frame 适配。旧单箱 factory、精确网格验证、真实安装接触和姿态角色仍需在真模型确定后接通，详见 [原生链路审计](native-adapter-audit.md)。测试用解析六轴 fixture 明确不是 S50；其 FK 或 URDF parser 通过不计为 S50 验证。

## 复现

在本 feature 分支仓库根目录，用 Python 3.10+ 和 `numpy PyYAML pytest`：

```bash
python tools/freeze_robot_replacement_layout.py
python tools/check_s50_integration.py
python tools/export_s50_model_viewer.py --output docs/experiments/s50_model_integration_20261110/viewer.html
python -m pytest tests/test_robot_model_binding.py tests/test_robot_replacement_layout.py tests/test_s50_integration_admission.py tests/test_s50_model_viewer.py tests/test_moveit2_tcp.py tests/test_contact_simulation_policy.py -q --tb=short --basetemp outputs/s50_model_integration/focused-test-tmp
```

第二条命令目前应退出 2，并写入 `outputs/s50_model_integration/admission.json`。这条入口是准入检查，不会启动 worker；还没有可宣称完成的 S50 连续卸箱入口。

双击 `viewer.html` 可离线打开，`viewer.html#cad` 直接进入 CAD 装配视图。内嵌 JSON 使用 `gzip(mtime=0) + base64` 无损压缩，HTML 从 32,403,507 字节降为 18,077,694 字节；实际解压后的 31,752,879 字节与 `4464b04` 原 JSON **逐字节一致**，7 个 CAD 部件、1,312,676 个三角面、完整 40 箱和全部数值/标注保持不变。完整字节数与哈希在 [压缩验证记录](viewer-compression.json)。

浏览器须支持 `DecompressionStream('gzip')`、`Blob.stream()`、`Response.text()` 和 WebGL，解压全部在本地完成，不使用 CDN；缺少解压 API 时页面明确报错。查看器导出通过了资产、三角面、包围盒、布局和禁用控件检查；本轮浏览器可视复核遇到浏览器访问 host 不可用和现有窗口最小化无法读取的问题，未记为可视验收通过。

CAD 网格以 `*.obj.gz` 无损保存以减小发布文件。导出器在原 OBJ 缺失时自动读取同名 gzip，解压后仍按未改变的 `cad_audit.json` 核验原始 OBJ 哈希，再解析相同几何；清单及查看器保留原网格路径。另在只含七个真实 gzip 网格及原清单的独立临时目录实际导出，CAD payload 与原始 OBJ 导出逐字节一致，记录在 [压缩验证记录](viewer-compression.json) 中。

GPU 检查隔离于 rootfs 内 `/work-s50-integration-20261110-v1`，输出位于该目录 `outputs/s50_model_integration/`。未创建或复用 S50 规划 worker；源 FANUC 进程 5155 保持运行。服务器完整证据留在服务器，仅将小型检查摘要纳入本报告目录。

最需要补齐的资料：对应 S50-2000 修订的厂家 URDF/DH、带六轴角度及正方向的官方参考姿态、基坐标和法兰坐标定义，以及与现有吸具连接的接口关系。补齐后才能继续真实初态求解、完整碰撞核验和一个顶层箱的原生规划。

本次追加资产范围、历史版本区别和来源许可说明见 [发布范围说明](PUBLICATION.md)。

# S50 CAD 运动模型：来源与局部碰撞修复证据

本轮沿用用户指定的真实 S50 STEP，并按新指示允许把 CAD 装配姿态定义为仿真 `sim0`、以明确的右手轴定义仿真正方向。这个约定可以建立可动仿真链；它不是厂商控制器零位/正向已被核实的证明。历史 `cad_audit.json`、原始 STEP 和原7份网格不作修改。

## 两处缺面已经取得精确局部补片

新增资料位于 `assets/robots/huayan_s50_2000/collision_repair/`。接口清单为 `coverage_report.json`；每条 `patches[]` 包含 `cad_body`、`mesh`、`mesh_sha256`、`coordinate_frame=CAD_METRES`、`source_face_index` 和 `qualification`。网格顶点在原始整机 CAD 坐标中，单位米；模型生成器应以同一 CAD→link 变换转换它们，作为所属连杆的附加碰撞表面。

J1面74、J2面75各由两条近重合BSpline边界围成。每条面长约105.023 mm；J1包围盒约11.172×2.993×105.023 mm，J2是相应旋转后的几何，面积约1.292529 mm²。面积很小不代表可以填整个包围盒，后者会引入明显不真实的几何。

分别按0.1、0.01、0.001 mm偏差对原面重网格，结果均为0三角形、OCCT状态4（`IMeshData_Failure`）；局部 `ShapeFix_Face.Perform` 也未改善。失败证据在 `local_face_inspection.json`。

以原始平面和**同一条原始wire**调用 `BRepBuilderAPI_MakeFace(plane, wire, Inside=True)`，修正了trim外边界方向。两个重建面均BRep有效，按0.0001 mm线偏差生成各1023个三角形。补片朝向使用REVERSED，以保持原solid的有符号体积。导出坐标保留17位小数，输出后重新加载核对。

| 检查 | J1面74 | J2面75 |
| --- | ---: | ---: |
| 原边界Edge身份保留 | 是 | 是 |
| 原/重建边界曲线几何Hausdorff | 0 | 0 |
| 精确包围盒最大变化 | 0 m | 0 m |
| 面积绝对值变化 | 9.10e-19 m² | 1.48e-18 m² |
| 三角形数 | 1023 | 1023 |
| 原边界8194点到补片边界最大距离 | 约50 nm | 约50 nm |
| 网格面积误差 | 约0.002096 mm²（0.162%） | 同左 |
| 新增物理厚度/法向偏移 | 0 | 0 |

几何边界的零差来自相同Edge及其曲线的身份核对；50 nm是稠密离散核对结果，不冒充网格的解析Hausdorff证明。线偏差请求值为100 nm。没有使用凸包、AABB填充、删除面或移动机器人。补片不会引入人为的毫米级外扩；与横带的实际间隙仍须在冻结布局中把补片随机器人变换后检查，不能用局部结果代替完整场景碰撞检查。

## 整体拓扑继续单独核对

`coverage_report.json` 记录了两种补片朝向替换原face后的完整连杆检查：

- J2使用REVERSED重建面后，整体BRep有效，6个shell均闭合，有符号体积与原始值完全一致。FORWARD替换会使整体无效，未采用该朝向。
- J1使用REVERSED重建面后，体积保持，但整体仍有 `BRepCheck_SubshapeNotInShape`。不能把两个局部面修复写成“整个J1实体修复完成”。

`j1_shell_containment.json` 进一步表明J1共有32个独立闭合、各自有效的shell；主外壳单独形成的solid有效，其余31壳为负体积内腔。检查到的内腔边界顶点均位于主外壳内部，未发现OUT顶点。该检查只是定位证据，不是连续曲面包含性证明。内部壳问题与两个缺面的问题分开保留；原始网格中的内外表面均未删除。

`j1_shell_isolation.json` 对主外壳加每个单独内腔的31种组合全部返回valid；逐个加入时从shell16开始出现invalid。进一步定位到内腔14与16的组合，但 `j1_void_overlap.json` 中这两个内腔的精确距离为64.139661 mm，Boolean Common为空（零面、零边、零顶点、零体积）。因此不能把这个BRep检查失败描述为真实几何重叠、外壳穿透或贴近横带。这里保留CAD内壳组织/检查器判定的不一致，不删除任何内腔来制造通过。

`j1_shell_order_check.json` 保留原序、反转内壳顺序、交换14/16、把16置于首位四种重建尝试；全部保持原face身份及体积，整体检查仍失败，未把单次重新排列当作修复。这个残余问题不应被混同于已修复的局部外表面缺面；针对三角表面的运行时碰撞仍须使用所有原表面与补片并独立验证。

## 控制器范围、零位与安装接口资料

重新视觉复核官方[机械手册](https://www.huayan-robotics.net/media/upload/LargeFile/HuaYan%20Robotics%20Mechanical%20Instruction%20Manual(S-series).pdf) PDF第25、33、55、56、65页。来源副本SHA256仍为 `2122d5d29d2e8e33c04f36183caa820d2f80e6a93ca03f8d229f0a8fdf69cd02`。

第33页的S50运动范围图展示参考外形和P点范围，含1841.5、1858.9、410.4、508、158.5等几何标注。图没有给出该外形对应的J2/J3控制器角度，也没有正转箭头或带轴标的控制器参考框架。因此不能把图中的水平姿态当作某一组明确的零位角，不能从外观推出 `q_controller = sign*q_sim + offset`。

安装图与CAD尺寸可以核对：支承平面在CAD Y=-3 mm，安装孔6×Ø13.5/PCD302；法兰为8-M6深10/PCD92，CAD中的Ø5几何是螺纹底孔。图中的机械孔位和定位特征可以约束机械接口，但没有证明厂商base/tool0坐标绕轴的定向，也没有提供现有Wantai吸具与S50法兰的专用实体连接件。

关节范围仍为J1/J4/J5/J6±360°、J2−190°至+10°、J3±160°。特别是J2非对称范围，在控制器offset/sign未知时不能直接当作对仿真sim0的已核实范围。仿真采用的范围或有界映射假设必须由本轮模型定义明确记录，不能标成厂商映射确认。

官网与机械手册的速度差异保留：网页180/120/120/180/180/224 °/s，手册244/150/120/180/180/180 °/s，未据此选择未核实的控制版本。本轮重新检索官方域名及GitHub中的“S50 zero joint Huayan”和“S50 零位 华沿”，未获得新的控制器/DH说明；普通产品/新闻搜索结果没有用于推导运动学。

## 复现

工作目录为本S50 worktree，使用隔离OCP目录，不修改FANUC环境：

```powershell
$env:PYTHONPATH='outputs/s50_model_sources/cad_python312'
$cadPython='D:/code/eco65 pick and place/.venv/Scripts/python.exe'
& $cadPython -X utf8 assets/robots/huayan_s50_2000/collision_repair/inspect_missing_faces.py
& $cadPython -X utf8 assets/robots/huayan_s50_2000/collision_repair/try_exact_face_rebuild.py
& $cadPython -X utf8 assets/robots/huayan_s50_2000/collision_repair/build_exact_patches.py
& $cadPython -X utf8 assets/robots/huayan_s50_2000/collision_repair/inspect_j1_shells.py
& $cadPython -X utf8 assets/robots/huayan_s50_2000/collision_repair/isolate_j1_shell_issue.py
& $cadPython -X utf8 assets/robots/huayan_s50_2000/collision_repair/inspect_j1_void_overlap.py
& $cadPython -X utf8 assets/robots/huayan_s50_2000/collision_repair/check_j1_shell_order.py
```

上述是来源和局部几何证据；完整sim0 FK、关节运动、场景碰撞和单箱原生规划由集成验证单独记录。


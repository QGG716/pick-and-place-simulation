# 本地与 GPU 工程资产归档（2026-11-10）

按用户本次要求，将当前工程在本地与 GPU 工作区中现存的资产和完整模型权重纳入
`feat/v0.6-eco65-ubuntu-real-integration`。代码基线为
`49d8f69fea431a9fef0d73cd4e0b5800bdbcbab3`。本次不改规划、只读硬件边界或标定状态，不执行模型权重。

归档清单为 `assets/ASSET_INVENTORY.json`，包含 4,084 个文件路径，逻辑文件总大小
3,033,128,667 字节；按 SHA-256 去重后为 1,859 份内容、2,506,905,739 字节。
其中 3,920 个路径使用 Git LFS，对应 1,752 份独立内容、2,504,236,136 字节。
CAD、网格、USD、纹理和二进制模型使用 LFS，其他单文件达到 1 MiB 时也使用 LFS；
小型 URDF、配置及来源文档保留普通 Git 内容。已有 Git 历史不迁移、不重写。

## 内容与来源

* 本地 `assets/`、`res/`、`configs/local/`：ECO65-B、FANUC、S50 原始 CAD/URDF/网格，
  吸具、相机、立柱、碰撞几何、材质及其原有来源说明。
* GPU 工程资产目录：当前 ECO65 集成、原生工程以及保留的历史部署资产，按原文件哈希合并。
  已下载的 cuRobo 配套资产位于 `assets/third_party/curobo/`，仓库场景原件位于
  `assets/environments/warehouse/`。
* GPU ECO65 原生工程 `outputs/` 中的 USD/USDA/USDC、OBJ、MTL、MDL，以及 warehouse_assets、
  conveyor_assets 下的纹理和依赖文件，保留原相对路径。当前工作台场景快照位于
  `outputs/workbench/scene_snapshot.json`。
* GPU Hugging Face 缓存中的三份完整权重及已缓存配置/词表：复制实际文件，不提交失效软链接或未完成下载。

| 模型 | 上游来源及固定版本 | 权重文件 | 字节数 |
|---|---|---|---:|
| Grounding DINO Base | [IDEA-Research/grounding-dino-base](https://huggingface.co/IDEA-Research/grounding-dino-base/tree/12bdfa3120f3e7ec7b434d90674b3396eccf88eb) | model.safetensors | 933400872 |
| SAM ViT Base | [facebook/sam-vit-base](https://huggingface.co/facebook/sam-vit-base/tree/70c1a07f894ebb5b307fd9eaaee97b9dfc16068f) | model.safetensors | 374979480 |
| MoGe 2 ViT-S Normal | [Ruicheng/moge-2-vits-normal](https://huggingface.co/Ruicheng/moge-2-vits-normal/tree/26b477f41595707c5db6770294c0d1721e8ed4ed) | model.pt | 140550416 |

权重在 `models/huggingface/<组织>/<模型>/<固定版本>/`。
这些是 GPU 上实际存在的 Hugging Face 版本，不等同于工作台示例配置中的
`groundingdino_swint_ogc.pth` 和 `sam_vit_b_01ec64.pth`；本轮没有改名、伪装格式、自动替换适配器，
也没有宣称完成权重推理验证。参考仓库清单列出的其他 `.pth`、ONNX、TensorRT 权重在这两处工作区未找到，
因此没有虚构或以其他模型替代。

本地文件优先保留原件。15 个规范路径在不同部署间存在 52 条版本差异记录；不同内容保存在
`asset_variants/<来源标识>/<原相对路径>`，清单 `sources` 记录每个源文件对应的发布路径。
同内容使用相同 SHA-256，Git LFS 按对象去重；不为了减少差异而改写换行、矩阵、CAD 或原始来源记录。
已有来源/许可证文件保留，各资产仍遵循其原始授权；本归档没有重新授予第三方内容许可证。

未纳入账号凭据、认证缓存、虚拟环境、系统/Isaac/ROS 安装包、不完整下载、重复传输压缩包，
以及运行日志、报告、录屏等非资产产物。服务器原件保持不变。

## 获取和核验

本次本地检查：4,084/4,084 个文件的大小与 SHA-256 通过；暂存区 4,084/4,084 项通过，
其中 3,920 项 LFS 指针中的 SHA-256 和大小与清单一致。对原始内容的凭据特征扫描未发现匹配项。
没有运行神经网络、仿真或真实硬件；资产完整性检查不表示这些运行验收已经完成。

需要安装 Git LFS。普通源码 ZIP 是否含 LFS 实体取决于 GitHub 仓库设置，建议通过 Git 获取：

```bash
git clone --branch feat/v0.6-eco65-ubuntu-real-integration \
  https://github.com/QGG716/pick-and-place-simulation.git
cd pick-and-place-simulation
git lfs install --local
git lfs pull
python tools/verify_asset_inventory.py
git lfs fsck
```

`verify_asset_inventory.py` 对所有 4,084 个路径检查大小和原始 SHA-256，明确报告缺失、
LFS 指针未下载、大小或哈希不符，不加载 pickle 或执行神经网络。
清单不包括自身、此说明文档和校验脚本，避免自引用哈希。

归档的 USD 和历史派生几何仍可能包含原 GPU 绝对路径，作为原始资产保留；直接执行旧场景前仍需
沿用工程既有的路径恢复、快照身份校验和依赖检查，不能把归档成功视为任意旧场景已可独立运行。

GitHub 普通 Git 文件上限与 LFS 用法见
[GitHub 大文件说明](https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage)。
本次只使用现有仓库与 LFS 上传通道，不修改账户付费预算。

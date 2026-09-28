# WP4 起点（2026-09-28 晚，本地；站离线）

## 范围出处（不靠记忆）

`docs/ADR-001/docs/07_WORK_PACKAGES.md:23` —— **WP4 正文在"任务表"第 4 行，不在小节里**：

> mode-bound calibration manifest、采集/标定/验证工具、只读 pose history、变换测试；
> 触及 `calibration/`、`camera_install.yaml`、`tracking/`、controller 与 vision 协议；前置 WP1/3；
> **Done/不做：原始/模型/预览坐标闭合，未知标定拒绝资格；不伪造 FOV。**

闸门 `08_ACCEPTANCE.md:22` 的 **C1**（多距离/方向/姿态跨镜留出集，LOS 映射残差 p95 ≤ 0.5°）
**需要真机与真相机**：本轮一律 `NOT_RUN`。

## 现状（本轮实际查过，不是推测）

| 资产 | 状态 |
|---|---|
| `calibration/camera_intrinsics.yaml` | **存在** |
| `calibration/camera_extrinsics.yaml` | **存在** |
| `config/camera_install.yaml` | **存在** |
| `calibration/charuco_board_P9.pdf` | 存在（采集治具定义） |
| mode-bound manifest | **代码里零命中**（`mode_bound`/`calibration_manifest` 均无） |
| pose history | **零命中** |

⇒ 三件标定资产**存在但互不认识**：没有任何东西说明"这套内外参与这个安装姿态属于哪一次标定、
哪个模式、坐标闭合是否成立"。WP4 的第一刀因此不是写采集算法，而是**把已有事实绑成一份可被拒绝的清单**。

## 第一刀（下一轮，全部本地可验）

1. `calibration/manifest.yaml`（新）：列出 intrinsics/extrinsics/install 三项的**路径＋内容哈希＋标定会话号＋适用模式**，
   并声明 `closure` 是否成立。数字与路径只进配置（工作区规矩）。
2. 读取器＋校验（`calibration/manifest.py`）：加载即校验——**文件缺失/哈希不符/会话号缺失 ⇒ 拒绝资格**
   （返回明确的拒绝原因，不静默降级、不伪造 FOV）。
3. **变换测试**：原始像素 → 模型输入 → 预览坐标的**闭合**（往返残差）；这是 WP4 的 Done 条件里唯一
   不需要真机的部分，因此它是本地唯一可以声称的东西。
4. 自检形式沿用 WP3 的做法：`--selftest` 里构造"哈希不符"与"缺会话号"两种坏清单，断言**被拒且带原因**
   （红必须吵）。

## 本轮不做也不声称

C1 的任何数值结论；FOV 的任何补值；detail 相机的标定（硬件尚不存在）；
`tracking/` 与协议侧改动（等 manifest 立住再说）。

### F-WP4-1：朝向**词表**宽于**实现**（图像路径没有 rotate_90/270）

`common/image_corrections.py:46` 的 `apply_orientation_image` 只实现 `none`／`rotate_180`／
`flip_horizontal`／`flip_vertical`，其余 `raise ValueError`；而 `validate_orientation` 的词表更宽
（含 90/270）。⇒ 配置里写 `rotate_90` 时，**画面会抛、几何（`apply_orientation_bbox`）却可能已按 90 校正**，
正是这份文件开头警告的那类事故的反面："画面看起来正常而几何差 180°"——这里会是"几何转了、画面没转"。

需要裁决（不在本轮擅自做）：**两边都支持 90/270**，还是**把词表收窄到实现支持的四种**。
倾向后者：本站实测安装是 `rotate_180`，没人需要 90/270，而词表每宽一项就多一条没人走过的路。

### 本轮已证

`perception/tests/test_preview_closure.py`：图像路径实现的四种朝向**都能自我撤销**（预览半程闭合成立）。
我自己先写错过一次——断言"所有朝向都是对合"，被 `rotate_90` 连做两次等于 180° 当场推翻；
闭合来自**互逆**，而这里恰好四种都是自逆。**是测试抓住了我，不是我把测试改成能过。**

### 交接：`模型 ↔ 原始` 半程闭合（WP4 最后一件，未做）

**本轮探明的现状**（不是推测）：

- `common/image_corrections.py` 只有**正向** `apply_orientation_bbox`，无逆变换；但已实现的四种朝向
  对 bbox 同样**自逆**，所以预览半程闭合已用真函数测掉（`test_preview_closure.py`，3 passed）。
- `perception/model/` 里**没有** `unmap`／`to_raw`／`to_frame`；只有 `input_size` 上报
  （`model/adapter.py:103`）与 `compatibility_probe` 的 `input_width` 字段。

**唯一挡住结论的问题**：检测器返回的框是**相对模型输入的归一化**（配置项 `bbox_normalized`），
还是**已按 letterbox 撤过缩放与补边**？
- 若是前者：`模型→原始` = 撤补边 → 除以 scale → 乘帧尺寸，需要一个小函数；
- 若是后者：那步已经在检测器里做完，闭合测试应改为**验证它真的做过**（否则框会整体偏移）。

**不猜的理由**：这两种情况下"闭合成立"的断言完全不同；而我这一晚已有三次因仓促写下、后来自己拆的结论。
读 `perception/model/imx500_yolo.py` 的解码段即可判定——**下一刀从那里起**。

**当前 WP4 资格姿态不变**：manifest 仍然**拒绝给资格**（`closure=claimed`／install 无会话），
所以这块没做完**不会让任何东西被错误地放行**——这正是清单存在的意义。

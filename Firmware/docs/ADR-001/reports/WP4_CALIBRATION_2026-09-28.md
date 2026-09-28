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

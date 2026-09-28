# WP7 起点（2026-09-28 深夜，本地；站离线）

范围（`07_WORK_PACKAGES.md:26` 任务表第 7 行）：关联（association）／仲裁（arbitration）／AprilTag。
本轮不设计、先清点——**空手规划 WP7 会凭空造零件**。

## 清点结果（grep 全仓 `perception control config tools`，非文档）

| 词 | 命中 | 含义 |
|---|---|---|
| `apriltag` | **0 个代码/配置命中** | AprilTag 这一块是**绿地**：没有解码器、没有配置、没有测试 |
| `associat` | `perception/replay/evaluator.py`、`perception/protocol/selected_target.py`、`perception/protocol/native_wire.py` | 关联的**消费者**已在（回放评估、被选目标的线上协议），但**实现处不在这些文件** |
| `arbitrat` | `control/tests/test_reference_manager.cpp`、`test_tracking_integration.cpp`、`test_tracking_observability.cpp` | 仲裁**已在 C++ 侧存在**（参考源管理＝reference manager），且有测试 |
| `fiducial` | `control/src/calibration/camera_calibration.hpp`、`installation_pose.hpp`、`control/src/geometry/turret_kinematics.hpp` | **基准标记（fiducial）已写进标定与运动学的接口** |

## 由此定三件事（都是省事的结论）

1. **不要新造"仲裁器"**：C++ 已有 `reference_manager` 带测试。WP7 的仲裁活是**接上感知侧**，
   不是再写一个——先读那三个测试，确认它现在仲裁什么、接口是什么。
2. **AprilTag 今晚只能是"契约"不能是"检测器"**：本容器无 Hailo、无真相机，`apriltag` 零命中说明
   连依赖都没有。所以本包能本地做的是**接口与资格**：谁产生 tag 观测、id 命名空间与
   `camera_id`/`generation` 怎么绑、错位时如何降级——**与 WP4 的标定闭合直接相关**：
   `fiducial` 已出现在 `camera_calibration.hpp`／`installation_pose.hpp`，
   ⇒ **AprilTag 是 WP4 里 `closure: claimed → measured` 的一条真路径**（跨镜留出集之外）。
3. **关联已存在，别再写第二套**（这条已从猜测改为事实）：`perception/tracking/track_manager.py` 内
   `iou`/`overlap`/`gate`/`mahalanobis`/`hungarian` 类词共 **18 处**，公开入口是
   `update(dset: DetectionSet, now_ns, *, image…)`（:192）。
   ⇒ WP7 的"关联"活是**读懂并按需扩展它的门限/代价**，不是新建模块；
   而这些门限**必须是配置项**（AGENTS.md 第 1、2 条：数字只有一处、闸门从配置取）。

## 本轮不做也不声称

任何关联/仲裁/AprilTag 的行为验证：WP7 的 Done 条件依赖真机真目标，一律 `NOT_RUN`。

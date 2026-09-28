# WP8 起点（2026-09-28 深夜，本地；站离线）

范围（`07_WORK_PACKAGES.md:27` 任务表第 8 行）：IMU **shadow**（影子模式，不参与控制，只记录与对比）＋ UI。

## 清点（grep 全仓，非文档）

| 词 | 命中 | 判断 |
|---|---|---|
| `gyro` | **`control/src/control/imu_trace_ingest.hpp` / `.cpp`**，并被 `control/src/main.cpp` 引用 | **IMU 轨迹摄取已存在且已接进主程序** ⇒ WP8 的采集侧不是绿地 |
| `shadow` | `control/tests/test_watchdog_trip_events.cpp`、`test_control_loop.cpp`、`perception/tracking/appearance.py` | 影子/让位概念在**控制侧测试里已有**；感知侧那个是别的语义（遮挡），**别混为一谈** |
| `imu` | `perception/pipeline.py`、`selection/service.py`、`tests/test_association.py` | **未逐条核实**：大小写不敏感的子串搜索，**可能是误命中**（如单词内含 `imu`）。记为待查，不当结论用 |
| `accelerat` | 感知侧三个文件 | 同上待查 |

## 由此定的三步（都不需要真机就能开始）

1. **先读 `imu_trace_ingest.hpp` 的接口**：它落什么、往哪落、时间戳谁给、掉样怎么记。
   **shadow 的"影子"必须体现在数据上**（同一时间轴与编码器对比），不能只是"我们收了 IMU"。
2. **UI 侧先定"暴露什么字段"再动手**：影子状态要能被看出来是**在影子中**而不是"没接"——
   这跟 WP4 的 `closure: claimed/measured` 是同一件事：**分不清"未接"与"未通过"的 UI 会说谎**。
3. `imu`/`accelerat` 在感知侧的命中**先核实再引用**，别拿子串误命中当"已经有实现"。

## 本轮不做也不声称

IMU 参与控制、任何停止包络/滑环相关验证：`NOT_RUN`（需真机）。UI 未动一行。

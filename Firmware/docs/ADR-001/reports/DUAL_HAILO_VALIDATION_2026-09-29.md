# 双摄 → 单个 Hailo-8：硬件验证总账（rpi-turret 真机，2026-09-29）

架构师的 7 项交付都在这里收口；每节的数字都能在最后一节的原始文件里找到出处。
**规矩**：所有数字来自 Pi 上真跑过的命令；"请求帧率"从不冒充实测；相机靠 **by-path 身份**认定，不靠 0/1 索引。

---

## 1. 盘点（Phase 1，`DUAL_HAILO_PHASE1_2026-09-29.md`）

Pi 5 Model B Rev 1.1 / Debian 13 / kernel `6.18.39+rpt-rpi-2712` / rpicam-apps 1.13.0 / libcamera 0.7.2 / Picamera2 0.3.37 / 4 核 / 8062 MB。
**Hailo-8**（`Board Name: Hailo-8`、`HAILO8`）在 PCIe `0001:01:00.0`，HailoRT 4.23.0；**链路协商到 Gen3 但宽度只有 x1**（`current_link_width=1`，max 4——Pi5 FPC 只给一条 lane；按指示只报告、不动 boot 配置）。
`hailortcli` 有 `benchmark`/`run2`；**没有 GStreamer（`ldconfig -p | grep -c gstreamer` = 0）、没有 hailoapp/hailomux、`HailoMultiStreamPipeline` 不可导入、没有 `hailortc`、没有 sudo**。
身份：`i2c@88000/imx500@1a`（广角）、`i2c@80000/imx477@1a`（窄角）。**所有传感器模式都是 30 fps ⇒ 这台机器上不存在 >30 fps 的模式可测**；广角无原生 1080p（1080p 是 ISP 缩放）。

## 2. 管线说明（推荐形状 = 今天测出来的 B 路）

```
广角 imx500 ─┬─ ISP main 1920×1080 RGB888 ──────────────→ 预览 / PIP / 录制（不进 Python 热路径）
             └─ ISP lores  640×360  RGB888 ──映射 691 KB→ 一次拷贝进预分配 640×640 画布 ─┐
窄角 imx477 ─┬─ ISP main 1920×1080（同上）                                              │
             └─ ISP lores  640×360 ──映射→一次拷贝→画布 ─┐                                │
                                                        ├→ 每路一条有界队列(depth=2，新帧赢)
                                                        └→ 单一 InferVStreams(batch-1, PCIe)
                                                            → 设备侧 NMS → 后处理 → DetectionFrame
                                                              {camera_id, capture_timestamp_ns(SOF),
                                                               inference_timestamp_ns, detections[]}
```
关键：**缩放发生在 Pi 的 ISP 里**（libcamera `lores` role，`stream_configuration('lores')` 核验），Python 只做编排、一次小拷贝、元数据。

## 3. 复现命令

```bash
R=/home/eamars/workspace/OpenAutoTurret/run/releases/<release>
H=/home/eamars/workspace/OpenAutoTurret/run/hailo-probe/yolov8n.hef     # SHA-256 已对 manifest
export OTA_RUN_DIR=/tmp/ota-stack-1000
P=$R/run/station-venv/bin/python

bash $R/Firmware/scripts/collect_station_inventory.sh                    # ① 只读盘点
$P $R/Firmware/tools/bench_dual_capture.py --selftest                    # ② 双摄同捕基线（无 Hailo）
$P $R/Firmware/tools/bench_dual_capture.py --seconds 20 \
   --spec 'imx500=2028x1520@30,imx477=2028x1080@30' --out /tmp/phase2_C.json
hailortcli benchmark --time-to-run 45 $H                                 # ③ 裸吞吐（官方工具）
$P $R/Firmware/tools/bench_hailo_pipeline.py --selftest                  # ④ 端到端矩阵
$P $R/Firmware/tools/bench_hailo_pipeline.py --hef $H --seconds 60 \
   --inference-stream lores --spec 'imx500=1920x1080@30,imx477=1920x1080@30' --out /tmp/B.json
$P $R/Firmware/tools/bench_hailo_pipeline.py --hef $H --seconds 600 --queue-depth 2 \
   --inference-stream lores --spec 'imx500=1920x1080@30,imx477=1920x1080@30' --out /tmp/soak.json
```
（跑前 `bash $R/Firmware/scripts/run_application.sh stop`，并确认没有别的进程握着 `/dev/media*`——今天四次假 `EBUSY` 是我自己的僵死工具造成的，不是平台限制。）

## 4. 基准表（交付速率都是实测）

| 阶段 | 配置 | 请求 | **实测** | 丢帧 | e2e p50/p95/p99 (ms) | CPU / 单核 | 温度 |
|---|---|---|---|---|---|---|---|
| 2 | 双摄同捕，无 Hailo（原生模式） | 30+30 | **30.026 / 30.010** | **0** | — | 3.69 % | 46.3 °C |
| 2 | 双摄 1080p（ISP 缩放） | 30+30 | 30.014 / 30.008 | 0 | — | 7.00 % | 46.3 °C |
| 5-A | `hailortcli benchmark` | — | **431.6 hw_only / 329.2 streaming** | — | HW 3.35 ms | — | — |
| 3 | 单路 广角 1080p→Python | 30 | **29.67** | 0 | 61.5/63.3/64.6 | 20.8 % | ↑1.7 |
| 3 | 单路 窄角 1080p→Python | 30 | **29.67** | 0 | 46.7/48.7/51.1 | 20.2 % | ↑0.6 |
| 4 | 双路 都 1080p→Python | 30+30 | 25.9 / 26.1（聚合 **51.3**） | 0 满 / 25+12 被顶 | 98.1/118.4/129.1 · 86.1/106.4/119.4 | 43.2 % | ↑4.4 |
| 4 | 双路 都 640×480→Python | 30+30 | 29.50 / 29.77（**59.27**） | 0 满 / 23+6 | 50.7/57.9/72.7 · 25.1/35.8/36.5 | 18.2 % | 持平 |
| 4 | 广角 1080p + 窄角 640×480 | 30+30 | 29.80 / 29.95（**59.75**） | **0 / 0** | 72.6/101.9/102.9 · 22.2/23.0/26.9 | 30.8 % | ↑3.3 |
| **B** | **双路 1080p 主图 + ISP lores 推理** | 30+30 | **29.93 / 29.93（59.87）** | **0 / 0** | **45.1/52.3/57.0 · 30.1/39.0/40.8** | **14.2 % / 15.8 %** | **↓2.2** |
| A′ | 同一负载、推理取主图（受控对照） | 30+30 | 25.85/26.10（**51.95**） | 0 满 / 41 被顶 | 96.7/119.7/132.7 · 86.2/106.3/119.5 | 43.6 % / 47.8 % | ↑6.1 |
| 长跑 | 基线（主图进 Python）600 s | 30+30 | 29.90 / 30.02（**59.92**） | 0 满 / 6 被顶 | 83.3/97.9/101.1 · 22.3/24.2/27.5 | 30.0 % | 52.9 峰 |
| **长跑** | **B 配置 600 s（最终候选）** | 30+30 | **30.02 / 30.02（60.04）** | **0 / 0** | **45.9/53.2/55.7 · 29.0/38.4/39.8** | **15.2 % / 15.6 %** | **47.9（降）** |

**长跑稳定性证据（B，600 s）**：36,024 次推理、队列峰值 1（设定 2）、`MemAvailable` 7193→**7254** MB、
六个 100 s 分桶 p50 广角 45.0–46.0 / 窄角 28.8–29.4 **全程不漂**、`throttled` 与跑前同一批**历史**位（**无新增降频/欠压**）、错误列表空（无流停顿）。

## 5. 上限结论与推荐生产点

- **裸上限（设备+链路）**：`hw_only` **431.6 fps**，**`streaming` 320–329 fps**。320×1.23 MB ≈ 400 MB/s ⇒ **上限被 PCIe x1 压着，不是被 26 TOPS 压着**（换 `ultra_performance` 无收益）。
- **端到端上限**：Python 缩放的形状顶在 **~52 聚合**（瓶颈逐点：`make_image` 6.2 MB 映射 7.5–9 ms、PIL resize 14.2/21.1 ms、画布 2.5/10.1 ms；**单核无一钉死** 47.8/42.3/40.6/44.0）；
  **换 ISP 推理流之后没有"上限"可言**——60 聚合就是满速（两路各 30），Hailo 才用到 50 %，剩余天花板回到 320 fps 那条线。
- **推荐生产点**：**两路 `main 1920×1080`（预览）+ `lores 640×360`（推理）、队列 depth=2 新帧赢、batch-1 单设备**。
  实测 10 分钟：60.04 聚合、**零丢帧**、广角 p99 **55.7 ms**、窄角 p99 **39.8 ms**、CPU 15 %。
- **不要两台都要 1080p 再在主机缩放**：那一格实测各 ~26 fps、广角 p99 **129 ms**、CPU 43 %。
- **热/供电约束（真实存在）**：满载出现过 `throttled=0x50000`（bit16 曾欠压 + bit18 曾触发软温度限）；长跑本身没新增事件，但**满载余量不大**，值得查 5V 电源。
- **延迟口径**：`capture_timestamp_ns = SensorTimestamp = 曝光开始`，所以表里的 e2e **含曝光积分**（当时 AE 顶到 **33.0 ms**）⇒ 对运动目标**偏保守约 16 ms**，且光照变亮会等量下降；EOF 不经 `get_metadata()` 暴露，"读出+ISP 交付"那段**明说没量**。

## 6. IMX500 片上 NN：只作辅助，量化如下（默认 OFF 是对的）

对照 = 同一推理形状（窄角 ISP lores→Hailo，60 s），唯一区别是站点整套栈（含 visiond 的片上 RPK）在不在跑：

| | 主路交付 | e2e p50/p95/p99 | 单核峰值 | 辅助路自己的节拍 |
|---|---|---|---|---|
| 辅助 **OFF**（栈停） | **29.85** | 28.8 / 30.0 / **30.4** | **7.06 %** | — |
| 辅助 **ON**（visiond 跑片上 NN + webd + controld） | **29.85** | 28.6 / 32.1 / **34.0** | **33.52 %** | 站点自报 `camera_fps = 26.0124`（实测）、`vision_dropped = 0` |

⇒ **主流水线交付一点没掉（29.85 = 29.85）；代价是尾延迟 +2~4 ms，以及整机单核 7 %→33.5 %（这是整栈的代价，不只 NN）。
辅助路自己只能跑到 26.01 fps——片上 RPK 在广角上吃掉约 4 fps 的传感器节拍。**
**默认 OFF、留一个开关**：合理。辅助要开也该由 visiond 在**同一进程内**用它的 `CnnOutputTensor` 元数据出标志位，
**今天实测的架构约束**：visiond 握广角时，另一个进程**可以**打开窄角（P6 两跑都成功），
但**片上 NN 只有握着广角的那个进程能喂** ⇒ 辅助与主流水线要么同进程，要么第二进程永远拿不到那片 `CnnOutputTensor`。

## 7. 真机证据（原始文件）

| 证据 | 位置 |
|---|---|
| 环境盘点 124 行 | 我家 `logs/drag/phase1_inventory.txt`（脚本 `Firmware/scripts/collect_station_inventory.sh`） |
| Phase 2 双摄基线 4 组 | `logs/drag/phase2_[ABCD]_*.json` |
| 裸吞吐两次 | 报告 `DUAL_HAILO_PHASE5A_RAW_2026-09-29.md`（含 `--help` 摘要与 throttle 位） |
| 端到端矩阵 6 格 | `logs/drag/pl_P*.json` |
| 阶段拆分受控对照 A/A′/B | `logs/drag/is_[AB]*.json`（含 `stream_roles` 核验与 metadata 样本） |
| 两次 600 s 长跑（含漂移分桶） | `logs/drag/pl_SOAK_wide1080_narrow480.json`、`logs/drag/is_SOAK2_isp_lores.json` |
| 辅助 ON/OFF | `logs/drag/is_P6*.json` + `/api/state` 摘录 |
| 报告正文 | `Firmware/docs/ADR-001/reports/DUAL_HAILO_PHASE1/PHASE2/PHASE4_5B/PHASE5A_RAW`、`ISP_INFERENCE_STREAM_2026-09-29.md` |
| 生产形状 + 契约测试 | `Firmware/perception/detection_frame.py`、`perception/tests/test_detection_frame.py`（7 passed） |

### 没验到的、别当成验过的

1. **最终候选长跑里两路检出都是 0**——那 10 分钟没人在这台机器前。"两路各自产生过检出"在基线长跑（广角 85 / 窄角 55 帧）与矩阵第 5 格里证过，但**在 B 配置上还欠一次有人入镜的短窗**。
2. **"Python 完全不碰像素"的形状没测**：需要 hailort 的 GStreamer element，这台机零 GStreamer、无 sudo。B 是"ISP 做缩放 + Python 一次 691 KB 拷贝"。
3. **>30 fps 的一切**：这台机的模式表里没有。
4. `perception/tests/test_pipeline.py` 有 2 条红（`model_inference_ms` 记为 unmeasured）**在我动工之前就红**（移开我的两个新文件后同样红）；`vision/tests/test_ipc_publisher.py` 的重连那条也是既有波动。

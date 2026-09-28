# 推理流的像素热路径：能不能留在原生侧（Pi ISP），Python 只做编排

回答架构师 scoped 任务的那一句问题：**"camera→Hailo 的像素热路径能不能留在原生侧，Python 只做编排和吃元数据？"
——能。这台机器上**不需要 GStreamer、不需要 C++**，靠 libcamera 的第二条（lores）流就做到了。**

工具：`Firmware/tools/bench_hailo_pipeline.py`（`--selftest` 5/5；`--inference-stream main|lores` 切换推理源，
其余一切相同：同一 HEF、`--queue-depth 2`、newest-wins、阈值 0.5、60 s、同一 Hailo 配置）。
原始 JSON：我家 `logs/drag/is_*.json`。跑在 rpi-turret，栈停着，跑前确认无他人握相机。

---

## 1. 现有热路径里到底哪一步贵（逐步计时，不是"Python 慢"一句话）

基线路径每帧经过：`capture_request()` → `request.make_image("main")` → PIL `resize`（BILINEAR）→
PIL `new`+`paste` → `np.array(copy=True)` → `np.ascontiguousarray` → `infer.infer()` → 解码。
**没有 OpenCV**（这台机器根本没装 cv2）；**没有 cvtColor / 没有 RGB↔BGR**——ISP 直接出 RGB888。

双路 1080p、推理取主图（cell A2，60 s，p50 / p95，单位 ms）：

| 操作 | 广角 imx500 | 窄角 imx477 | 说明 |
|---|---|---|---|
| `capture_request` 等待 | 8.02 / 15.25 | 7.91 / 15.04 | 拉一帧 |
| **`make_image("main")` 把整帧映射进 Python** | **7.52 / 15.46** | **9.00 / 15.71** | 主图 **framesize 6,220,800 B**（API 自己报的，不是我按 W×H×3 算的） |
| **PIL `resize` 1080p→640×360** | **14.23 / 21.14** | **13.85 / 20.88** | **单步最贵的就是它** |
| canvas+paste+`np.array` | 2.71 / 10.10 | 2.45 / 10.18 | 1.23 MB |
| （letterbox 合计） | 21.88 / 24.67 | 16.81 / 24.76 | resize + 上面两步 |
| `np.ascontiguousarray` | 0.002 / 0.007 | 0.002 / 0.007 | 已经是连续的，几乎免费 |
| `infer.infer()` | 7.47 / 8.30 | 7.43 / 8.27 | 设备侧 |
| 后处理 | 0.12 / 0.23 | 0.05 / 0.09 | |
| **Python 每帧搬运** | **7.45 MB** | **7.45 MB** | 6.22 MB 映射 + 1.23 MB 画布 |

⇒ **贵的不是"Python"，是三件具体的事：把 6.2 MB 整帧搬进 Python（7.5–9 ms）、PIL 双线性缩放（14 ms p50 / 21 ms p95）、
再把 1.2 MB 拷进画布（2.5 ms p50 / 10 ms p95）。** 两路各做一遍，就吃满了一个 33 ms 的帧周期 ⇒ 交付掉到 ~26 fps/路。

**单核口径（架构师点名要的）**：`cpu0 47.83% / cpu1 42.30% / cpu2 40.60% / cpu3 43.97%`，总 43.64%。
⇒ **不是某个核被单线程钉死**，而是总工作量本身太大（外加 GIL 下 5 个线程互相抢）——所以"再开一个进程"未必救得了，**少搬字节才是解**。

## 2. Pi ISP 能不能同时出"大图预览 + 小图推理"？能，而且是官方 role，运行时核验过

`create_video_configuration(main=1920×1080 RGB888, lores=640×360 RGB888)` 被栈接受，
并且 **`stream_configuration('lores')` 亲口回报**：`size [640,360] format RGB888 stride 1920 framesize 691,200`；
同一回报里 `main` 仍是 `1920×1080 RGB888 framesize 6,220,800`。
两台相机同时各开一组（cell B）：**IMX500 与 IMX477 都成功**。
几何没被压扁：16:9 主图 → **640×360**（16:9）→ 上下补边到 640×640；宽角那台若用 4:3 主图，`lores_size_for()` 会给 640×480。
（工具在 lores role 缺失或尺寸不符时会**拒跑**，不会把基线路径挂个 "lores" 标签糊过去。）

## 3. 像素留在原生侧之后，Python 还剩什么

cell B 每帧：`make_array("lores")`（**映射 691 KB**）0.89 / 0.99 ms（p50/p95）→
一次 `pad_into()` 拷进**预先分配好、补边已写好**的 640×640 画布 **0.169 / 0.221 ms** → `contiguous` 0.002 → `infer` 8.4。
**Python 每帧搬运 1.92 MB**（0.69 映射 + 1.23 画布），主机总工作量 ~1 ms/帧。
预览那 6.2 MB 主图**根本不进 Python**（本轮甚至没有取用它；将来给 PIP 时走 webd/编码侧，不占推理热路径）。

## 4. 受控对比（只变推理源这一件事）

| pipeline | 主预览流 | 推理源 | 聚合 FPS | 每路 FPS | Python 侧预处理 p50/p95 | e2e p50/p95/p99（广角 / 窄角） | CPU 总 | **单核峰值** | Hailo 忙 | 丢帧 |
|---|---|---|---|---|---|---|---|---|---|---|
| **A 基线** | 1920×1080 ×2 | 主图 → PIL → NumPy | 51.95 | 25.85 / 26.10 | letterbox 21.9/24.7 · 24.8 | 96.7/119.7/**132.7** · 86.2/106.3/**119.5** | 43.64 % | 47.83 % | 39.1 % | 0 满 / 41 被顶 |
| **B ISP lores** | 1920×1080 ×2（**保留**） | ISP 640×360 → 一次拷贝 | **59.87** | **29.93 / 29.93** | 1.06/1.21 | **45.1/52.3/57.0** · **30.1/39.0/40.8** | **14.24 %** | **15.80 %** | 50.0 % | **0 / 0** |

**B 相对 A：聚合 +15%、广角端到端 p99 从 132.7 → 57.0 ms（−57%）、窄角 119.5 → 40.8 ms（−66%）、CPU 少 2/3、丢帧归零、
队列峰值从 2 降到 1（没有堆积）。Hailo 占用 39% → 50%——因为它终于被喂满了。**

## 5. 预览与推理不必同源

两路都是"主图 1080p + lores 640×360"，**同时**跑，30+30 fps 满速、0 丢帧、温度反而下降（50.15 → 47.95 °C）。
⇒ 生产里"高分辨率给预览/PIP、小流给检测器"这个形状，在这台机器上**现在就能成立**。

## 6. C++ 还需不需要：**不需要**

判据是架构师给的那条：只有当主机侧像素工作仍然显著、且 profiling 明确指向 Python/API 开销时才写 C++。
现在主机侧每帧 ~1 ms、单核峰值 15.8%、聚合 59.87 fps、0 丢帧、Hailo 忙 50% ⇒ **限制已回到设备/链路本身**
（裸上限 320 fps streaming，见 Phase 5-A）。**Python 只做编排 + 元数据，不再是热路径**。
唯一还留在 Python 的像素动作是**一次 691 KB 拷进画布**；真要消掉它，办法是换 padding 位置（Hailo 侧/硬件），
不是换语言。

## 7. 延迟口径（不再把分位数相加）

`e2e` 是**一次实测区间**，不由阶段值相加；阶段分位数各自独立，本来也不能相加。

`capture_timestamp_ns` 到底是什么时刻——**`SensorTimestamp`，libcamera 语义 = 该帧"曝光开始(SOF)"，单调时钟域**。
本轮实测到的同一帧元数据里有：`ExposureTime`（**33,044 µs / 33,013 µs——AE 已经打到帧周期上限**）、`FrameDuration`、
`FrameWallClock`、`ScalerCrop`、`SensorTemperature`，imx500 另有 `CnnOutputTensor` / `CnnInputTensorInfo` / `CnnKpiInfo`
（片上 NN 自己的输出，Phase 6 要用）。元数据里**没有** `Timestamp`（buffer 的 EOF 时间不经 `get_metadata()` 暴露），
所以我**不声称**能分解"读出+ISP 交付"那一段。

诚实口径：**我的 e2e = 曝光开始 → 推理完成**，因此它**包含**曝光积分（这里 33 ms）、ISP、交付、预处理、排队、推理。
SOF 之前只有光子飞行（可忽略），**没有藏着没量的传感器/ISP 段**。
反过来还有个偏保守的方向：运动目标的"那一瞬间"实际是曝光中心（SOF + ~16.5 ms），
⇒ **对运动目标，我报的 e2e 高估约 16 ms。**
另外：33 ms 曝光说明现场偏暗（AE 顶格）；**亮一点这套 e2e 会等量下降**——本表是"最暗情况"下的数。

## 8. 长跑

* **基线（广角 1080p 主图进 Python + 窄角 640×480）600 s：已完成。**
  聚合 59.92 推理/秒（两路各自 29.90 / 30.02），600 s 共抓 35,960 帧、推理 35,952，
  **丢帧：队列满 0、被新帧顶掉 6 帧（0.017%）**，队列峰值 2 = 设定值，
  `MemAvailable` 7242 → 7294 MB（**不降**），温度 49.05 → 52.9 °C，跑后 `throttled=0x50000`（**与跑前同一历史位，无新增降频/欠压**），
  **两路在同一 10 分钟窗内都产生过检出：广角 85 帧、窄角 55 帧**（但没人刻意入镜，"检得准"不在此范围），
  无流停顿/重启（错误列表为空）。e2e：广角 83.3/97.9/101.1，窄角 22.3/24.2/27.5。
  分桶漂移：见 JSON `end_to_end_buckets`。
* **最终候选（两路 1080p 主图 + ISP lores 推理）600 s：跑完补数。**

## 复现

```bash
R=/home/eamars/workspace/OpenAutoTurret/run/releases/<release>
H=/home/eamars/workspace/OpenAutoTurret/run/hailo-probe/yolov8n.hef
# A 基线
OTA_RUN_DIR=/tmp/ota-stack-1000 $R/run/station-venv/bin/python $R/Firmware/tools/bench_hailo_pipeline.py \
  --hef $H --seconds 60 --inference-stream main \
  --spec 'imx500=1920x1080@30,imx477=1920x1080@30' --out /tmp/is_A.json
# B ISP 推理流（其余参数完全相同）
... --inference-stream lores --out /tmp/is_B.json
```

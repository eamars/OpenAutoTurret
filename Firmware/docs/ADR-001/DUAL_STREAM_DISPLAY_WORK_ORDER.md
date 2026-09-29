# 双流显示（选 (b)：visiond 是两颗物理相机的唯一持有者）— 施工单

架构师定的形状：`visiond → {IMX500 wide, IMX477 detail} → main+lores → Hailo；main → preview → webd → HUD 主画面 + PIP`。
**webd 不碰 `/dev/media*`**；`camera` 参数只认 `wide` / `detail`；两路都由 **durable identity / by-path** 绑定，**绝不用 0/1 索引**。
**JPEG 只是 transport，不是契约**——换 unix socket / 共享内存时相机归属不搬家。

## 已落地（今天）

| 件 | 位置 | 状态 |
|---|---|---|
| **named-stream 清单**（visiond↔webd 的边界契约） | `Firmware/perception/stream_manifest.py` | `--selftest` 4/4；6 条 pytest；键/体不一致会拒、半份 JSON 读成"没有"、没测过的帧率是 `None` 不是 0 |

它的 `StreamDescriptor` 就是那份契约：`role` / `camera_id` / `identity_source` / `durable` / `path` / **`transport`** / `width,height` / **`delivered_fps`（实测）** / `dropped`。

## 剩下三段（每段都可独立部署 + 独立验收）

**Slice 2 · visiond 发布清单（单路也能发）** — `perception/visiond.py`
`run_capture` 里 identity 已经打出来了（`_run_camera` 附近，第 537 行那句 `visiond: camera identity ...`）；
把它 + `tap_path` + `PreviewTap`/`JpegPreviewWorker` 的实测计数写成一条 `wide` 描述，落到 `$OTA_RUN_DIR/video_streams.json`
（新环境变量 `OTA_VISION_STREAM_MANIFEST`，缺省即不发布 ⇒ 老部署不受影响）。
**验收**：跑起来后 `cat $OTA_RUN_DIR/video_streams.json` 里 `wide.identity_source == "by-path"`、`durable == true`、`delivered_fps` 是实测值。

**Slice 3 · 第二颗相机进 visiond**（真正的 (b)）— `perception/visiond.py` + `camera_worker.py` + `configs/*.json`
配置新增**按 by-path 绑定**的第二路（示意，最终以 `perception/config.py` 的加载器为准）：
```json
"streams": { "wide":   { "by_path": "/dev/v4l/by-path/platform-...-capture-video0", "preview": {...} },
             "detail": { "by_path": "/dev/v4l/by-path/platform-...-capture-video1", "preview": {...} } }
```
每路各自一个 `PreviewTap` + 一个 `JpegPreviewWorker`（§39：**深度 1、latest-only，绝不在采集线程里编码**），
清单里两条；**没插窄角时 `detail` 直接缺席**（不是 `delivered_fps: 0`）。
Hailo 侧沿用今天量过的 B 路：两路 `main 1080p + lores 640×360 → depth=2/latest-wins`。

**Slice 4 · webd 与 HUD** — `web/webd/video.py`、`app.py`、`hud.py`
复用现有 source abstraction（`video.py` 已经是"默认 OFF、懒开、独立线程、报实测到达帧率"的形状），
**加一个 `VisiondStreamSource`**：读清单、按 `role` 取；`/api/video?camera=wide|detail`，**不传 `camera` 等价 `wide`**（老 HUD 不断）。
顺手修架构师点的那条 bug：**`/api/state` 的 `camera_id` / `camera_identity_source` 现在是空串**
（`web/webd/protocol.py:204` 有槽位、`perception/visiond.py:501` 只在**收工报告**里写它），
由清单的 `wide` 条目填上 ⇒ HTTP 面报的是感知进程**实际绑定**的那颗。
HUD：**PIP 左上角、默认隐藏**，显示后可 wide/detail 互换，**两路各自显示自己的实测 delivered FPS**。

## 真机验收（架构师列的 7 条，一条都不许口头通过）

| # | 验什么 | 怎么算过 |
|---|---|---|
| 1 | wide 单独预览 | `/api/video?camera=wide` 出帧，且清单里 wide 的 delivered FPS 是实测（≈30） |
| 2 | detail 单独预览 | 同上 `camera=detail`；两块的 `camera_id` **必须不同** |
| 3 | wide 主画面 + detail PIP | 两路同时在跑，`/api/video` 两条各自计数在涨 |
| 4 | 两路反向切换 | 主/PIP 互换后 FPS 与身份**跟着画面换**（换错会立刻看得出来） |
| 5 | PIP 默认隐藏 | 首次加载 DOM 里 PIP 不请求第二路（`/api/video?camera=detail` 请求数为 0） |
| 6 | visiond 持两摄时 Hailo 双路仍 ≈30+30 | 复用 `tools/bench_hailo_pipeline.py` 同参数复跑，聚合 ≥59、丢帧 0、p99 与 `is_SOAK2` 同量级 |
| 7 | **webd 重启不丢相机归属** | 只重启 webd：visiond **不**重开 `/dev/media*`（比对 visiond 进程 `/proc/<pid>/fd` 里的 media fd 集合不变、`video_streams.json` 的 `camera_id` 不变），预览 5 s 内自己回来 |

第 7 条是 (b) 的意义所在：**webd 崩了、升级了、被重启，都不能让相机重新被人抢一次。**

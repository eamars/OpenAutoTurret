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

---

## Slice 3C 的真面目（09-29 下午核过代码，不是我推测的）

`PerceptionPipeline.process_frame(...)` **没有 `camera_id` 参数** ⇒ 今天的 pipeline 天生单相机:
第二颗传感器的帧直接喂进去,会被折进第一颗的跟踪状态里.这不是"加个循环"能解决的,
**它就是 `visiond.py:527` 与 `telemetry.hpp:314` 两处注释都在指的那次 dual-worker cut**.

**这一轮先交不需要真机、也不需要架构决定的那块**:`perception/inference_gate.py`
——一颗加速卡、两路相机的**推理仲裁器**.三条固定性质,都有测试(`selftest` 6 条 + pytest 7 条,零硬件):

| 性质 | 为什么是它 |
|---|---|
| **有界、latest-wins、按相机分别计数** | 生产数字是 depth 2(实测).积压不是历史,是对"现在"的谎;`dropped_full` 与 `failures` **分开计**,因为它们是两种病 |
| **轮转,不是到达序** | 广角每次都早一微秒提交就能把窄角饿死.**公平是性质,所以有测试**,`fairness = min/max` 直接发布 |
| **一设备一线程;失败会喊** | 第一条异常带名字进 stderr(B45),之后只计数.设备一直失败应该让流降级,不该让守护进程退休 |

**那次 cut 还欠的三步**(按依赖顺序):
1. `process_frame` 接受 `camera_id`(或由 pipeline 持有 per-camera 状态),否则第二路的框会串进第一路的轨迹;
2. **per-camera adapter**:广角今天用的是 IMX500 片上 RPK(站点实测 `camera_fps ≈ 26.01`),而 IMX477 没有片上 NN ⇒ **它必须走 Hailo**,所以是一个 pipeline 里两个 adapter,不是一个 adapter 吃两路;
3. 轨迹/选择的合并:`perception/tracking/camera_registry.py` 已经在"按 durable id 分轨迹"上了,缺的是把两路的 `TrackSet` 汇进 `controld` 的那一层(§20 的线格式要加身份,`telemetry.hpp` 的注释已经预感到这件事).

**上线时要量的两个数**(唯一需要站点的部分):两路各自的**实测** delivered FPS 仍 ≈30/30,
以及仲裁器发布的 `fairness`(应 ≥0.9)。其余全部可用 `MockAdapter` 在无设备上验证。

---

## UI 待办（09-29 傍晚，compact 之前落在这里）

**主人规则：web UI 没更新 = 没改.** 所以下面每一条的验收都写成"屏幕上看得见什么",不是"提交过没有".

### 已核实（从 `http://192.168.2.103:8080` 取，不是从 git 推）
- webd 跑在新 release `402b516feff4.J4kmfn/Firmware`；根路径 HUD 里 `id="pip"` 与 `function imuLabel` 都在.
- `/api/video?camera=detail` 起流 ok、一帧 30,307 字节；未知角色 400 并列出 `wide, detail`；
  `/api/video/state?camera=detail` 带实测 `delivered_fps`.
- `/api/state` 的 `imu`：`present:true / rate 215.7 / world_elevation_deg:null / basis:"…no mount calibration…"`.

### 缺陷 A（可见）**右上角 `IMU ABSENT` 是写死的**
`web/webd/hud.py:1129` → `chip("IMU","amber","ABSENT")` 是**无条件常量**.四态 `imuLabel()`(hud.py:384) 只喂了
DIAG 抽屉那一行.** ⇒ 改成用 `imuLabel(t.imu)` 算出来.

### 缺陷 B（根因）**叠加层只在一个出口**
`web/webd/app.py:267` 的叠加层（`imu` 合并、`video_streams`、`camera_id`）**只作用于 `/api/state`**；
喂 HUD 的是 `/ws` 广播（`app.py:90`、`app.py:103`），它把 controld 载荷**原样**发出 ⇒ 页面看不见 `video_streams`.
** ⇒ 叠加层收进**产生遥测帧的那一处**（`/ws` 与 `/api/state` 共用同一产物）.一条出口，两个消费者.

### 已写好、**故意未部署**的那笔（`git` 里：见提交 "PIP button"）
PIP 按钮 `#pipopen`（关着就在角上可见）；实测帧率走 HUD **已有**的那条轮询（`window.otaPipTick`，
`delivered_fps` 缺失时显示 `rate n/m`，**不显示 0**）；删掉一条永不生效的兄弟选择器规则（`~` 要求 DOM 顺序，我写反了）。

### 待主人定点（他给了截图，别再覆盖元素）
图上已确认的占据区：左上 `MANUAL/HOLD/Person`；右上芯片行；视频内左中 `HOLD TO AIM` 十字；视频内右缘 pitch 标尺；
视频内右下 `PITCH -0.1°`；左下 `FIELD OF REGARD`＋`SAFE ENVELOPE`；底中 `Manual/Hold`、`Auto`；右下停靠栏
`TARGETS MODE MANUAL DIAG MENU`；最底状态条.**"右下角"是三层叠着的地方，不能用.**
- **A（我推荐）**：顶部右侧黑边带（芯片行下、yaw 刻度右，约 x900–1250 / y100–340）——压黑边，不盖画面不碰控件.
- **B**：底部黑带（`Auto` 与停靠栏之间，约 x640–1000 / y1150–1300）——不压画面，但挤.

### 部署与验收（定完角一次做完）
1. `./Firmware/tools/station_address.sh deploy -- --prebuilt --activate --ready-timeout 600`
   （**`--prebuilt` 不能漏**：漏了远端会在 Pi 上原生编 → 退出码 2）
2. 起来之后**把模式放回 MANUAL**（主人有意 park；`/api/command` 的载荷形状是 `{"command": …}`）.
3. 屏幕上要看见：芯片 `FRESH 216HZ/A3`（或未配置 / 无样本 / `STALE <ms>`）；PIP 按钮在指定角上；点开才有请求；
   PIP 上显示**这一路自己的**实测帧率；单摄部署下这些东西一个都不出现.

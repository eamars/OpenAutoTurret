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

### 主人判决（09-29 晚，三张不同窗口比例的截图之后）——**取代我提的 A/B**

> **「PIP 和主摄像头的左上角对齐，不放进黑色区域——因为不同分辨率下黑色条带的位置不一样.**
> **唯一的方法就是：画面只重叠画面，不重叠网页元素。」**

这条把"锚在哪"从**页面**改成了**画面本身**，而且给了一个可检验的不变量：
**PIP 只与 `<video>` 的元素盒相交，永不与任何页面控件相交。**黑边（letterbox）是渲染副产物，
它的位置随视口比例变——09-29 那三张图（1678×567、707×847、1504×850）里黑边分别在左右、上下、几乎没有，
所以**任何以黑边为基准的坐标都会在别的比例覆盖元素**。

**落法**：PIP 是视频容器的子元素，`position:absolute` 相对**视频元素盒**定位（左上角对齐）；
左上角唯一可能撞上的是页面上层的 `MANUAL/HOLD/Person` 状态块（它锚在页面上，宽屏时落在黑边、
近方形时落在画面上），所以**下移量用 `getBoundingClientRect()` 量出来**，而不是写死像素：
`top = max(8, block.bottom - video.top + 8)`，`left = 8`，`ResizeObserver` 触发重算。
**测出来的间隙，不是猜出来的常数。**

### 待主人定点（已由上面判决取代；下面是我当时读图记录的占据区，仍然有效——用来核对'不重叠'）
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

### 部署后的实况（09-29 晚，release `77bfb0d77511.YY0CWa`，`77bfb0d`）

从**页面实际吃的那条 `/ws`** 抓的一帧（不是 `/api/state`）：
`video_streams=[('detail',8.97,True),('wide',8.98,True)]`；
`imu={present:True, fresh:True, rate_hz:215.2, world_elevation_valid:False}`；`camera_id cam-baa28c2a by-path`.
页面源码里 `pictureBox`×2、`placePip`×6、`ResizeObserver`×1、`#pipopen`×1、`indexOf("FRESH")`×1.

**两条留给主人的判断**：① 芯片文字用了 `imuLabel` 的整串（`FRESH 215HZ|RATE …`），信息全但**偏长**，要不要精简版；
② 画面左上角与 `#mode-block` 撞上时下移让开——**近方形窗口会明显往下掉一格**，不合适就改让法。
**还欠一条测试**：`/ws` 载荷带叠加层这件事现在只有线上实证，没有回归测试钉住。

### 朝向：配置到位了，**效果没到**（已闭合，见下一节）（09-29 夜，release `03b6a8e44247.8B5lnT`）

我从站上各取一帧（`/api/video?camera=wide|detail&limit=1` 是 **multipart 流**——第一次我把整条流当图片存，
`file` 说是 data，**是我的量法错了，不是站上的帧坏**），同一时刻同一场景对照：

| 角色 | 证据 | 判定 |
|---|---|---|
| wide | `SECRET LAB` 三角**朝上**、窗帘在**上**、桌面在**下**、`TITAN` 可读 | **正**（`camera_install.yaml` 的 `rotate_180` 生效） |
| detail | 同一个 logo 三角**朝下**、亮帘在**下** | **仍倒**——`secondary.orientation` 值进配置了、visiond 也开了流，**transform 没生效** |

**下一手**：查 `camera.py` 里 `open_picamera2_sensor` 那条分支——两处 `Transform(hflip,vflip)` 之一点了，
要么顺序不对（configure 之后设），要么那条分支只算了没用。**判据不是代码读起来对，是 detail 那一帧的 logo 朝上。**

### 朝向：闭合（09-29 夜，release `0840a0df8c1a.bHzl8f`，`0840a0d`）

**根因**：`VisionConfig.from_dict` 见到 `vision` 段就把根换成它，所有小节都在**里面**找；我把
`secondary` 追加在了 `vision` **旁边**。于是配置从未被读——模型名由启动器环境补上、几何恰好等于默认、
朝向落回 `none`，**一个症状都没有**。两处小改：

1. 段挪进 `vision`；
2. 写在 `vision` **旁边**的小节现在**自己报错**（`config: sections ['secondary'] sit beside 'vision'
   and are ignored; move them inside 'vision'`）。"键在文件里但没人读"是最难查的一类失败，因为
   对文档的每一次目视都显得正确。

**生效链**：文档 → `VisionConfig` 解析出 `orientation='rotate_180'` → visiond 启动行打印
`secondary stream imx477 … orientation='rotate_180'` → 传感器 transform（`create_preview_configuration`
那条与主摄相同的已验证路径）→ **抓回的一帧里亮帘在上、显示器从上往下挂、线缆垂在下面**。
判据是**像素**，不是 diff。

---

## 生产切 Hailo · 实现顺序（09-29 深夜立，goal `goal-e600f9c0`）

主人的话：**「目标是把生产环境切换到 Hailo，不依赖 IMX500。这些工作不需要实际校准，
实际校准也不可以成为不部署的理由。Web 需要同步更新——我看不到的就没有更新。别 workaround。」**

**读到的形状（不是猜的）**：帧的入口是 `perception/model/adapter.py:89 ModelAdapter.infer(image, metadata, *, frame_sequence, …)`
——**没有 `camera_id`**，这就是 dual-worker 的切口；`HailoYoloAdapter`（`model/hailo_yolo.py:22`）已实现 `open/infer/close/describe`；
`build_adapter`（`model/adapter.py:297`）按 `model.adapter` 选类；profile `hailo_yolov8n` 已在
`perception_v1.json`（`adapter=hailo, camera_model=imx477, 640x480@15, orientation=none`）。

| 步 | 做什么 | 完成判据（**都落在他看得见的东西上**） |
|---|---|---|
| **H1** | 后端自述上线：`adapter.describe()` 进 visiond 发布 → webd 叠加层 → `/api/state`+`/ws` 的 `inference` 块（后端名、模型、每路输入分辨率、推理 fps、端到端延迟、丢帧）；HUD 芯片行加一枚后端芯片，DIAG 加行 | 他刷页面就看见 `NN HAILO`（切回 IMX500 时看见 `NN IMX500`），**不点 DEV 也能看见** |
| **H2** | `open_picamera2_sensor` 支持 **main + lores** 双腿（main 1920×1080 给人看，lores 640×360 给 Hailo），`CameraOwner` 交 lores 进推理、main 进 tap | 清单/线上报出**推理输入 640×360**；主画面仍是 1080p；Phase 5 的结论落地：**主机缩放从 14.5 ms/帧 降到 ~2 ms 量级** |
| **H3** | 生产 profile 切 `hailo_yolov8n`（广角：`camera_model: imx500` 当**纯传感器**用 + 安装朝向照 `camera_install.yaml`），**片上 NN 关掉** | **拔掉 Hailo = 没有感知**（吵，带原因），**不是**静默降级回 IMX500；`tracks` 全部来自 Hailo |
| **H4** | dual-worker cut：`infer(..., camera_id=…)`、per-camera adapter/几何、`InferenceArbitner` 喂两个 worker、两路 `TrackSet` 在 `tracking/camera_registry.py` 汇流；窄角 IMX477 的 lores 也进来了 | 站上实测两路各 ~30 fps、**fairness ≥ 0.9**、HUD 上两条流各自的身份/分辨率/延迟都在 |

**不许的**：并行跑两套后端贴标签；主机缩放当默认（它是显式后备）；用"校准没做"当不部署的理由；
把 `describe()` 里没测过的数字填成常数（没测到就发 `null`，HUD 显示 `n/m`）。

### H2 进行中（round 2）：lores 腿的补丁**没落地**，现场是干净的

`camera.py::open_picamera2_sensor` 带返回注解（`-> Tuple[...]`），我用 `def …\(([^)]*)\):` 找签名 ⇒ 正则不匹配 ⇒ 断言先炸、**文件从未写盘**（所以没有半套改动，perception 仍是基线 4 既存失败 / 435 passed）。
下一手：按字符串锚点插参数（锚 `buffer_count` 那段参数文本，别用正则配平括号）；info 字典要加的两行插在 `"stream_size": (width, height), "task": "object_detection",` 之后（已用 `cat -A` 核对过原文）；`ModelConfig` 加 `camera_lores_width/height`（`_as_int` 点名式校验），visiond 里 `adapter.configure_stream(*(info.get("lores_size") or stream))`，`CameraOwner` 需要按 `info["inference_input"]` 取 lores 帧（**这一处还没读过，动手前先读**）。

### H2 完成（round 5）：一条请求出两条腿，推理吃小的那条

`open_picamera2_sensor(..., lores_size=…)` 现在向 ISP 同时要求 main 与 lores；`CapturedFrame` 多带
`inference_image` / `inference_size`（**两条腿取自同一个 request**——两次请求就是两个光照时刻）；
`visiond` 用 `adapter.configure_stream(*(lores or stream))` 把推理配到**它真正会被喂的那条腿**上，
`process_frame` 拿 `frame.inference_image or frame.image`。profile 侧新增 `camera_lores_width/height`，
**为 0 就是单腿站**（今天的全部 profile 都是 0，所以这一步不改生产行为——它是 H3 的前提，不是偷跑）。
芯片上显示的是 `describe()["stream"]`，也就是推理真正吃的那条腿的实测尺寸，不是模型名义输入。

**测试**（`tests/test_lores_leg.py`，4 passed、**零 skip**，在没有 picamera2 的机器上靠注入桩模块跑）：
两条腿各取一次且都释放、`inference_image` 是小腿 / `image` 是大腿、单腿站行为不变、
`create_preview_configuration` 的 kwargs 里 `lores.size == (640,360)` 且 `info["inference_input"]=="lores"`。
**H3 才见真章**：把生产 profile 切 `hailo_yolov8n` 并填 `camera_lores_width/height`，芯片应变绿
`NN HAILO 640x360`。

### H3 已上线（release `6e11098ab330.uL2Wd3`），但有一个明确的未完项

**成了的部分（现读）**：`adapter=hailo`、`stream=[640,360]`、`inferences=1955`、`failures=0`、
`model_inference_ms=7.9`（与 Phase 5 的 7.0–8.2 ms 相符）、`hailo_device_id=0001:01:00.0`、
`device_architecture=HAILO8`、artifact SHA 与 profile 声明一致；广角 main 仍 1920×1080；
`detections_raw = detections_emitted = 5175` ⇒ **类别过滤无罪**。

**未完项：`tracks=0`、`track_list_age_ms=None`（controld 自启动没收到过一次 TrackSet）。**
根因已定位，且是本轮我自己种的：**解码端不减 letterbox pad**。HEF 的 on-device NMS 给的是
**含 pad 的张量坐标**；探针用 pad=80（480 腿）时，不减只是位置偏移、框仍在画面内；
640×360 腿 pad=140 ⇒ 纵向偏移 21.9%、尺度错 1.78× ⇒ 框落到画面之外 ⇒ 跟踪永不确认。
**正解（不是 workaround）**：pad 必须**双向走完整回路**——编码时算出来，解码时按同一条映射还原
（`leg_y = tensor_y − pad`，再除以腿高；x 除以 640）。因此 `pad` 不能是 `infer()` 的局部变量，
要成为解码映射的一部分。**先别动跟踪门限**：在坐标正确之前，任何"把 `new_track` 调低"都是拿参数盖 bug。

### 屏上没有框的第二环（09-30 凌晨现读）：TrackSet 声明的是推理腿，不是发布画面

pad 修完之后（release `eef257b16afe`）仍然 `tracks=[]`、`track_list_age_ms=-1`。逐环节现读：

| 环节 | 读数 |
|---|---|
| 网络 | `detections_raw = detections_emitted`（每帧都有框）、`detections_pad_dropped = 0` |
| 跟踪 | `run/perception/track_set.json`：`detections_in 4068`、`tracks_created 18`、**`tracks_confirmed 11`**、`tracks 2` ⇒ **跟踪器在工作** |
| 发布 | 同一份文件顶层 **`stream_width/stream_height = 640 × 360`** |
| controld | `vision: … age 2 ms`（视觉在收）、`tracking=off`（HOLD 下不消费）、**`track_list_age_ms = -1`**（`last_set_receive_ns_` 从未赋值 ⇒ **一份 TrackSet 都没进环**） |

**结论**：交换被几何校验挡在环外——TrackSet 声明了几何，而 controld 按**它对外发布的那幅 1920×1080** 核对。**正解**：TrackSet 必须声明**发布画面的几何**；推理腿的几何是**推理侧的事实**，已经由 `inference.stream` 单独发布，不该冒充画面几何。
归一化坐标可以从腿直接搬到画面上，**前提是 lores 腿是同一光学的缩放而非裁剪**（16:9 对 16:9）；**这个前提由“框是否落在他身上”当场证伪/证实**，不是靠我说。

**动手位置**：`dset.stream_width/stream_height` 来自 `configure_stream(腿)`（`perception/pipeline.py:469` 把它们写进发布）；
visiond 同时知道 `stream`（main）与 `lores`，所以由它把**发布几何**交给发布路径。

## 换后端之后的端到端验法（09-30 凌晨跑通，逐环节都有读数才算通）

任何一次改动推理后端或采集腿之后，照这五步读。**每一步都要看见数字**；缺一环就别宣布成功——
今天两次"以为通了"都因为只看了其中一环。

| # | 看哪 | 通过判据 | 今天抓到的 bug |
|---|---|---|---|
| 1 | `/api/state` 的 `inference` | `adapter` 是你以为的那个、`stream` 是推理真正吃的腿、`failures` 不涨 | `frame.or` 让 numpy 求布尔 ⇒ visiond 6 s 退出 |
| 2 | 同块 `detections_raw` / `detections_emitted` / `detections_pad_dropped` | raw>0 且 emitted>0；raw≠emitted 就说明被类别/label map 吃了 | 以为"没人"，其实每帧 2.6 个框 |
| 3 | `run/perception/track_set.json` 的 `counters.tracks_confirmed` | >0 ⇒ 跟踪在工作；`low_score_associations` 大 ⇒ 才是门限问题 | pad 未还原 ⇒ 框落到画面外 |
| 4 | 同文件顶层 `stream_width/stream_height` | **等于对外发布的那幅画面**（广角 1920×1080），不是推理腿 | 声明成 640×360 ⇒ controld 环外丢弃 |
| 5 | `/api/state` 的 `tracks` 与页面上的框 | 有人 ⇒ 至少一条，且**框贴在人身上**（这证明 lores 是缩放不是裁剪） | 前四环都错时无从发现 |

`camera_fps≈30` 与 `model_inference_ms≈8` 是旁证（Phase 5 量过的区间），不是判据。
**门限永远最后调**：在 1–4 环没绿之前动 `new_track` 就是拿参数盖 bug。

### H4 起手（round 12）：`camera_id` 穿过整条契约，`merge_track_sets` 有了拒绝条件

第二段（dual-worker cut）的第一步不是加线程，是**让"这是谁的像素"这句话一路传到底**。今天的
`infer()` 没有 `camera_id`，两路喂同一个 Hailo 时一个 adapter 说不清自己刚吃掉的是哪条腿的帧——
于是计数器、letterbox pad、几何全都是混的。这一轮把这些做完，**还没有**加第二路：

| 改动 | 落在哪 | 拒绝条件 |
|---|---|---|
| `infer(..., camera_id=)` | `adapter.py` 基类 + hailo + imx500 + mock | adapter 绑过摄像头又收到另一路的帧 ⇒ `ModelRejected`「a frame routed to the wrong worker is a routing bug」，**不计入 `failures`**（路由错是接线错，不是网络失败，记错列就看不见） |
| 一个 adapter 只服务一个摄像头 | `bind_camera()` | 二次绑到别的 id ⇒ `ConfigError`「cannot also serve」 |
| `process_frame(..., camera_id=)` | `pipeline.py` 两个生产调用点都盖上（真传感器用 `_ident.id`，合成路径用 `mock://synthetic` 导出的 id） | — |
| 发布文档与每条 track 都带 `camera_id` | `protocol/track_set.py`（`stamp_camera()`）、`tracking/track.py` | — |
| 汇流 | `tracking/camera_registry.py::merge_track_sets()` | 没盖 camera_id / 同一路来两份 / 声明几何不一致 / uuid 撞车 ⇒ 各自点名拒绝 |

**为什么不在这轮就汇**：`merge_track_sets` 在生产里还没有产生者——窄角那颗仍只有预览腿（`detail_stream.py`
开头那句"故意不做推理"依然成立）。先契约后线程，是为了让第二路接上时**错会当场叫**，而不是两路混出一堆
谁也说不清的计数器。

**下一步（H5，还没做）**：窄角开 640×360 lores 腿 → 第二个 adapter + 第二个 pipeline + 第二个
`CameraWorker` → 生产里真正调用 `merge_track_sets` → 量两路各自 fps 与 **fairness ≥ 0.9**。
controld 侧的 per-track 归属要 native wire v3（头里没有摄像头字段），那是汇流之后的一步。

#### H4 上线实况（round 12，release `0c3a917b7c62.JnxFSk`，`0c3a917`）

从站上 `/api/state` 与 `run/perception/track_set.json` 现读，不是从 git 推：

```
后端 = hailo | camera_id = cam-baa28c2a | failures = 0 | emitted = 8617 | fresh = True
wire camera_id = cam-baa28c2a | tracks = [('Person #10', 'cam-baa28c2a')] | 声明 = 1920x1080
```

也就是说**每条 track 现在都带自己的摄像头归属**，第二条腿接上来的时候不需要再改发布格式。

**本地与站上 suites**：perception **4 既存失败 / 452 passed**（比基线多 2 条绿：新加的 pipeline 两条）、
webd **279 passed / 1 skipped**、doc-tree 绿、arch_lint 无新红、`camera_registry` 自检 **21/21**（多出 11 条
都是 merge 的拒绝条件）。站上 prebuilt suite **66 binaries 0 failed**，readiness 到 `AUTO_TRACK / camera_fps=30.0158`。

**这一轮我自己造的险（写进部署卡了）**：清理 release 目录时拿 `run/active_release` 当指针——**那个文件不存在**。
`cat` 出来是空，于是循环把两份目录（上一版合格的 `7fd9a1919cb1.FAggEo` 与刚激活的 `0c3a917b7c62.mYPvxo`）都删了，
栈从**已删除的目录**里继续服务（`readlink /proc/<pid>/cwd` 里挂着 `(deleted)`）。修法不是猜指针，是**问进程**：
`pgrep -af visiond` 的 argv 里就写着它在哪个 release。同一 revision 重新部署一次即恢复，现在 `releases` 里恰好一份
`0c3a917b7c62.JnxFSk`。教训归到 `docs/operations/deploy.md` 的已知故障清单里。

### H5 之前必须先定的一条：汇流要按新鲜度收，不按"最近一次成功"收

`merge_track_sets` 拿到的是**两份文档**，而两路各自的 worker 快慢不同（广角 30 fps，窄角今天只有 9 fps 的
预览）。如果汇流照单全收"每路最近一次的 TrackSet"，一条 1.5 秒前的窄角 track 会被当成现在有人在——**而
控制环会朝它转**。这不是精度问题，是文档在说一句它没测过的事。

所以 H5 的汇流带一条闸：**超过 `merge_max_age_ns` 的贡献不参与合并**，并且被丢掉的那一路要写进合并文档
（`stale_sources: [camera_id]`）——丢掉是要被看见的，不是悄悄少了一路。阈值取自配置（AGENTS 规矩①），
不写在函数体里。**没测过的延迟不许当健康**，这一条与 §40 "unmeasured 不等于 0" 同源。

### H5（round 13）：窄角开自己的 640×360 腿，第二条 pipeline 在同一个 Hailo 上跑

**代码事实**（等站上读数才算数，这一节只写"做了什么"）：

- `secondary` 节进文档：`imx477 / 1280x720 main / 640x360 lores / depth 2 / rotate_180`——**缩放仍然落在
  ISP 侧**，主机不 resize；`orientation` 与广角同一套机制（两颗都是倒装的，理由写在 config 注释里）。
- `DetailFrame` 带 `inference_image`/`inference_size`：两条腿来自**同一次 capture request**，都 copy。
- 第二个 `HailoYoloAdapter`（`bind_camera(detail_id)`）+ 第二个 `PerceptionPipeline`（独立 EventLog）
  + `CameraWorker`（`camera_worker.py` 里本来就有的生命周期），从 `SecondaryCameraStream.latest()` 取
  **最新一帧**（depth 2 / latest-wins）。
- **单发布器**：广角循环仍是唯一发布者，经 `MergedTrackSetView` 发**合并文档**；`_publish_wire` 仍只发
  广角那一份——controld 的头里至今没有摄像头字段（v3 才有），把合并集发给它会让窄角的框指挥炮管。
  这句话在启动日志里也说了，不是只在文档里。
- 汇流画布 = 发布的画面（广角的 1920×1080）；窄角 TrackSet 也声明它，所以合并文档只声明一个几何。
- `describe()` 现在每路各自带 `stream`（腿尺寸）、`declared`、`inference_fps`（自测，窗口 ≥1 s，
  **没测够就不出现这个键**，不写 0）；`inference_health.json` 多 `cameras`/`detail_stream`/`merge` 三块。

### H5 的真机结论：**一个 Hailo 设备不允许第二个 context**（`32a765d` 上线即倒，`c0649f7` 恢复）

主人那句「你改错咯」是对的，现场和他看到的一样：`/api/video/start` 返 500、预览全停。死因不是我推的，
是 HailoRT 自己在 `vision.log` 里喊的：

```
[HailoRT] [error] CHECK_SUCCESS failed with status=HAILO_DEVICE_IN_USE(73)
visiond: model refused (§9.3): HailoRT could not open .../yolov8n.hef: error 73 (HAILO_DEVICE_IN_USE)
```

**读法**：`HailoYoloAdapter.open()` 拥有设备，所以"一个 adapter 一个摄像头"在 Hailo 上等于"一个摄像头一个
HailoRT context"——而这块 Hailo-8 只许一个 context。于是第二个 adapter 一 `open()` 就把整个 visiond 带走
（EXIT_MODEL）。**它没有静默退回单摄**，这一点是 ① 要的行为，我认它死得吵；错在我把"per-camera adapter"
直接等同于"per-camera device"，而 device 这一层根本还没拆出来。

**当下的处置是配置，不是代码**：`secondary.lores_width/height = 0`（`_why` 里带着这条实测理由）。窄角回到
预览-only，广角一切照旧——恢复后实读 `后端 = hailo | inference_fps = 29.91 | failures = 0`，
`/api/video/start` 返 `ok: True`，两路预览在流（detail 7.99 / wide 6.99），`MANUAL`，单 release。
**"关掉"是被发布出来的**：健康文档里 `cameras` 键**不存在**（不是 `0 Hz`），所以页面上少一路是看得见的状态。

**H6 的正解（下一轮）**：把 device 从 adapter 里拆出来——**一个 context、一个 runner、两个摄像头共用**；
per-camera 的仍是几何、pad、计数器与 `camera_id`。共用 runner 会把两路串行化，所以 `measure_dual_feed.py`
先量出公平性与各自 fps，再决定要不要上 multi-instance vstream。**没有读数不改架构**。

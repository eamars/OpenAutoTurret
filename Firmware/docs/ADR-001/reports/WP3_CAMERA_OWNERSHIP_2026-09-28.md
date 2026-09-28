# WP3 · 相机所有权与身份（进行中）

## 稳定 ID：现场验证（这台站真有的路径）

`/dev/v4l/by-path/` 在站上是两枚：`platform-1000880000.pisp_be-video-index0` 与 `-index1`
（PiSP 后端的两个节点，同一台物理 IMX500），另有 `/dev/video0/1/10…`。
`perception/camera_id.py` 对它们的实际输出见本轮对话记录：**两个节点映射到同一个 ID**
（同一台相机就应该是同一个身份，节点不是相机），且 `durable=True`；
而 `/dev/video0` 只能给出 `source=index, durable=False`——**这就是为什么今天之前
"哪台相机"这个问题在代码里没有答案**。

未做（NOT_RUN）：ID 接入 visiond 与 `webd/protocol.py`（下一刀）；A3 mock 故障注入；T2 帧率。

现场实测（2026-09-28 深夜，站上直接跑 `perception/camera_id.py`）：

```
platform-1000880000.pisp_be-video-index0   cam-f45ad188  by-path  durable=True
platform-1000880000.pisp_be-video-index1   cam-f45ad188  by-path  durable=True
video0                                     cam-eab9807c  index    durable=False
```

同一台物理相机的两个节点 → **同一个 ID**（节点不是相机）；裸编号 → 另一个 ID 且明说不耐久。
ID `cam-f45ad188` 从此是这台站 wide 相机的身份，换 USB 口/换枚举顺序都不会变，
而 `/dev/videoN` 会——下一刀接进 visiond 与 protocol 之后，HUD 上应同时看到
`camera_id` 与 `camera_identity_source`。

## 接线落地（本轮）与它的诚实边界

`visiond` 的上行载荷现在带 `camera_id` + `camera_identity_source`，`webd/protocol.py` 已声明
（中继带类型，不声明就"存在但到不了页面"）；webd 套件 **269 passed / 1 skipped** 仍绿。
边界要说清：**平台今天只把 `camera_num`（内核编号）交给我们**，所以线上会读到的多半是
`source=index, durable=False` —— 不是失败，是**页面第一次诚实地写着"这个身份不耐久"**。
升级动作是配置层面的：把 `/dev/v4l/by-path/platform-1000880000.pisp_be-video-index0` 交给
`open_picamera2`，`camera_id` 就变成已在现场验证过的 `cam-f45ad188 / by-path`。
下一刀之后要做的是 backpressure 显式策略与丢弃计数（A3 的"队列有界"需要可数的东西）。

## 部署后现场读到的结果（不粉饰）

`--prebuilt` 落地（65 binaries / 0 failed、冒烟 `phase=homing fault=''`）之后 `/api/state`：

    {'camera_id': '', 'camera_identity_source': '', 'phase': 'hold', 'vision_frames': 6520}

**两个键存在（中继不再吞它们）但值为空** ⇒ 链上还有第三处关卡：`webd/app.py` 合并视觉上行时
用的是**显式键清单**，我没把新键加进去（`vision_frames` 能到页面就是走那张清单）。
这是同一族陷阱的第三种打扮：协议声明了、发送方发了，中间还有一层没放行。
**但我上一轮把那一层认错成"webd 合并白名单"**：`vision_frames` 在 `webd` 里只有声明与显示两处
（`protocol.py`、`dashboard.py`），也就是说感知字段是 **controld 从视觉 IPC 收到上报后，由它自己的
遥测序列化（`control/src/web/web_server.hpp`）发出去的**——真正的关卡在那儿，不在 webd。
所以下一刀要动的是 controld 那条上报通路（视觉上报 → controld 遥测 → web_server.hpp 的 JSON），
而不是给 webd 加白名单；改完期望值仍然是 `cam-f45ad188 / by-path`。下一刀：把两键加进 `webd/app.py` 的合并清单，
再把 `by-path` 节点配进 `open_picamera2`（那之后期望值＝`cam-f45ad188 / by-path`）。

## 下一刀的施工清单（追证人字段 `vision_frames` 得到的五个落点，别重新调查一遍）

| 落点 | 位置 | 要做什么 |
|---|---|---|
| 感知上报解析 | `control/src/control/control_loop.cpp:2559`（`snap.perception_session_uuid = session_text;` 同段） | 从同一份 JSON 里读 `camera_id` / `camera_identity_source` |
| 快照赋值 | 同文件 `:2885`（`snap.vision_frames = vs.frames;` 同段） | 一并赋值 |
| 遥测结构 | `control/src/telemetry/telemetry.hpp:174` 附近 | 加两个字段，**默认空串**（空＝"未上报"，不是"没相机"） |
| Web 序列化 | `control/src/web/web_server.hpp:173` 附近 | 紧跟 `vision_frames` 输出两键 |
| 日志行 | `control/src/main.cpp:630` | 顺手带上，现场一眼能看到 |

验收（写死，不接受口头）：`/api/state` 里 `camera_id` 非空；配好 `by-path` 节点之后必须等于
**`cam-f45ad188`** 且 `camera_identity_source="by-path"`；webd 套件与 C++ 套件全绿；
`--prebuilt` 部署，站上零编译。

## 施工前的发现：那张五步图前提错了（已改道）

五个落点是量出来的没错，但**前提是"感知上报是 JSON 字典"——错了**：`snap.perception_*` 来自
`selection_.last_set().observation`（`session`/`generation`/`native`，v3 **带类型线格式**），
`vision_frames` 来自 `VisionLink::Stats`。要往 controld 遥测里加 `camera_id`，就得给 **v3 线格式加字段**
（Python 写方 + C++ 读方 + 版本/兼容语义），不是加一个键那么简单。

改道（本轮）：**身份先落在 owner 自报这一层**——`visiond` 启动即打印
`camera identity <id> source=<by-path|index> durable=<…>`，每次开都留一行身份记录；
上行载荷里的两个键保留为 best-effort。controld 那一份**等 dual-worker 那刀必然 bump 线格式时一起带**，
那时五步图仍然有效，只是它属于那一刀。

## 现场未通过：身份行没出现在 vision.log（本轮，未验证）

站上 release `1091b03a4dd8` 的 `perception/visiond.py` **确实含有**我加的身份打印（远端 grep 到），
`vision.log` 里 `visiond: camera 0 stream 1920x1080 …` 在（第 16 行），
但**它前面没有 `camera identity …` 那一行**。 ⇒ 有两种解释，未分辨之前我不说"已上线"：

1. 运行时 import 的 `perception` **不是这份 release 源码**（station-venv 里可能装了包，遮蔽了 release 目录）
   —— 若成立，则第 15 轮 `camera_id` 为空的真正原因**根本不是中继**，而是**我改的代码没在跑**；
2. 打印被 stdout 缓冲/重排（可能性低：同一 print 流里的后续行都在）。

分辨探针已备：用 station-venv 的 python 打印 `perception.__file__` 看它解析到哪个路径。
在分辨出来之前，WP3 的"身份已上线"降级为 **NOT_VERIFIED**；已验证的仍只有
`camera_id.py` 的 4/4 自检与真路径映射（`cam-f45ad188 / by-path`）。

## 分辨完成（本轮）：身份行进了没人接的 stdout

不是包遮蔽，也不是分支没执行：`visiond` 的所有日志行都以 **`file=sys.stderr`** 结尾，
启动器把 **stderr** 收进 `vision.log`；我那句 `print` 走 **stdout**，等于打进黑洞——
所以"代码明显在跑（帧号 8430）、日志里却没有它"。修法：加 `file=sys.stderr`。
教训写死：**在这个仓库里，守护进程的可观测输出 = stderr；stdout 的 print 不会被采集。**

## 耐久身份上线（第 29 轮）——**但我预告的期望值错了**

部署后站上打印：`visiond: camera identity cam-baa28c2a source=by-path durable=True`
⇒ 实质目标达成：**身份现在来自端口名、可跨重编号**（不再是 `index/durable=False`）。
**但我上一轮写死的期望值 `cam-f45ad188` 没出现。** 不偷偷改期望值：原因是同一个节点在
`/dev/v4l/by-path` 下可能有多个名字，`resolve_durable_id` 取 `sorted()` 的第一个，
而我当初手工验证用的是我自己从 `ls` 里挑的那一个名字。
⇒ 该验的性质（durable、端口身份、两节点同一相机）都已成立；**"到底在哈希哪个名字"是我欠的下一个核对**
（一行打印即可），如果它落在一个含枚举顺序的后缀上，耐久声明就得再收一档。

另记：**WP2 ③（CAN1 断、CAN0 有效）需要 `ip link set can1 down`＝sudo**，我这环境无 sudo，
得主人方便时配合一次，或由他给一条免密规则。

## 核对完成（第 30 轮）：耐久身份成立，期望值是我挑错了设备

站上实跑：`realpath(/dev/video0) = /dev/video0`，`/dev/v4l/by-path` 下**唯一**命中它的名字是
`platform-1f00128000.csi-video-index0` → `cam-baa28c2a`，与 visiond 打印**完全一致**；
候选唯一（无"挑哪个"的歧义），名字里除每接口计数器（已被剥掉）外无枚举顺序残留。
⇒ **`source=by-path, durable=True` 的声明成立。**
我上一轮的期望值 `cam-f45ad188` 来自 `platform-1000880000.pisp_be-*`——那是 **PiSP 后端节点**，
而 owner 打开的是 **CSI 前端节点**：我手工挑错了设备，身份语义本身没错（身份＝owner 实际打开的那个节点所在端口）。
双节点若将来同时被打开，各自有各自的身份，这正是我们想要的（节点≠相机时，身份也该分开）。


---

## 增量台账（2026-09-28 晚，站离线刷机中：全部本地验证）

| 增量 | 内容 | 本地验收 | 状态 |
|---|---|---|---|
| 1 | `perception/camera_worker.py`：每相机一个 worker（自有线程、自有 latest tap、代次标注、失败隔离）＋ `WorkerSupervisor` | `python -m perception.camera_worker` **9/9**（死亡隔离／挂起不拖人／洪泛有界且计数／重启换代／旧代次被拒） | **完成**（`6ccd94a`） |
| 2 | `perception/tracking/track_manager.py`：按 `camera_id` 归置的 pipeline 实例（跟踪状态今日长在 `PerceptionPipeline` 实例里，仓库无 `*Tracker` 类），一个相机的跟踪器抛错不得波及兄弟 | 待做：同形状自检 | 未开始 |
| 3 | `visiond.py` 真正改用 worker＋manager（现在仍是 `pipeline.py:255` 那一个线程）；预览 latest tap 按相机归置 | 待做：`visiond --selftest` 增加双 worker 断言 | 未开始 |
| 4 | 资格报告 A3/T2 | **NOT_RUN：站离线（主人刷 GM6020 固件），且 08 的 A3/T2 需真相机帧率** | 押后 |

`LatestTap` 与 `pipeline.PreviewTap` 的分工已写进 `camera_worker.py`：预览那份带 fps 限速与预览计数
（§39：预览不上关键路径）；跟踪这份带代次、不限速——**把限速套到跟踪路径会悄悄丢跟踪帧**。

### 发现 F-WP3-1：本地套件先前有 276 个测试**从未被收集**（不是"17 个红"）

`perception/pipeline.py:78`、`perception/replay/evaluator.py:41`、`perception/tests/test_track_manager.py:20`
都从 `perception.tracking.track_manager` 导入，而**该包在仓库里不存在**（真货在 `vision/track_manager.py`）。
缺包导致多个测试文件采集期失败，连带**静默丢弃 276 个测试**（`--co` 计数：建包前 750，建包后 1026）。
因此本报告上方"34→17 failed"的基线是**低报的**；建包后的真实画面是 **122 failed / 880 passed / 1026 collected**，
其中绝大多数失败是**一直存在、只是不可见**。

推论：`--co` 计数应作为套件的常规断言之一（收集的测试数下降就是红），否则"绿"只反映被收集的那部分。

### 增量②的真正内容（下一刀）

不是"我发明一个 TrackManager"。`pipeline.py:359` 的调用处就是规格：
`TrackManager(config, session_uuid=..., event_log=..., diagnostics=...)`，带 `.aliases` 与 `.set_selected_uuid`。
真货 `vision/track_manager.py` 有 `TrackManager/Track/TrackManagerConfig` 但**不满足该签名**（28 个
`test_track_manager` 在 0.1s 内即失败＝API 不合）。两条路待 V3 文档裁定：
(a) 把 `vision/track_manager.py` 移植成满足调用处的 `perception/tracking/track_manager.py`；
(b) 若 V3-2（`12fe12b`：轨道在 visiond 内形成）已让 `pipeline.py` 的 TrackManager 用法成为遗留，则删该导入。
**先定方向再动手**，不两边都做。

我那个"每相机一个跟踪器"的东西已改名为本来的名字：`perception/tracking/camera_registry.py` /
`CameraTrackRegistry`（自检 10/10）。

### 撤回 F-WP3-1（我错了，错因写在这）

上面那条"276 个测试从未被收集 / 17 红是低报的绿"**因果不成立**，撤回。实情：

- `perception/tracking/` 与真文件 `track_manager.py`（758 行）**一直存在且被跟踪**；
- 我用 `cat >` 建自己的文件时**把真文件覆盖了**，又 `git mv` 把覆盖后的版本改名成 `camera_registry.py`；
  于是出现"122 failed"——那是**我造成的 API 不合**，不是把隐藏失败暴露出来；
- 我从 `e85c3ce` 恢复的版本与被我覆盖前**逐字相同**（`git diff e5ad6d3 HEAD -- .../track_manager.py` 为空），
  所以真文件无损；
- 基线本来、现在也还是：**1026 collected / 17 failed / 985 passed / 24 skipped**。

教训落成动作：在既有目录里**新建同名文件之前先查它是否已被跟踪**（`git ls-tree`/`git status`），
`cat >` 是破坏性写入；`mkdir -p` 静默存在不报错，会让人以为目录是自己新建的。

### 增量②重述

真 `TrackManager` 一直可用，所以增量②**不是移植、也不是删导入**，而是：`camera_registry.py`
（每相机一个跟踪器，自检 10/10）**尚未接线**，因此它现在**不构成任何主张**。接线属于增量③
（`visiond` 双 worker），届时用 `test_visiond` 作闸门——它是绿的。

### 增量③（本轮）：synthetic 路径改跑在 CameraWorker 上——**已接线，未被行为验证**

`_run_synthetic` 的帧循环从"在守护自己的栈上直接 for 循环"改为跑进 `CameraWorker`＋`WorkerSupervisor`，
每相机的 `state/frames/dropped/generation/error` 打进 stderr 摘要（本站的日志是从 stderr 采的）。

- 已证：**不破坏既有**——`test_visiond` 25 passed，全量 **17 failed / 985 passed / 1026 collected** 与改前一致。
- **未证**：新那行 `visiond: worker …` **本地没有任何东西能跑出来**。原因是出厂配置
  `perception/configs/perception_v1.json` **没有 mock profile**（`--profile mock` 直接被拒），
  而本容器无 `picamera2`，所以 `isinstance(adapter, MockAdapter)` 那条分支到不了。

### 发现 F-WP3-2：§55.18 的离线验收**按出厂配置跑不起来**

`_run_synthetic` 的注释声称"无传感器、同一条代码路径（recorder/publisher/preview/timings）"，
但唯一出厂配置里没有能选出 `MockAdapter` 的 profile ⇒ 在没有相机、也没有 picamera2 的机器上，
这条"离线验收路径"**不可达**。要么配置里补一个 mock profile，要么这条路径等第二台相机才第一次被执行。
**倾向补配置**（数字进配置，不往代码里塞默认），属于 WP5 清单收口的邻域。

中途我自己踩的第二个坑也记一笔：先写了 `config.camera.device`，而 `CameraConfig` 只有几何字段
（`width/height/frame_rate/preserve_aspect_ratio`），**属性访问直接抛**，`or` 兜不住——16 个测试当场红给我看。

### F-WP3-2 关闭：出厂配置补了 `_mock` 模型项，离线路径可达了

`vision.models._mock`（`adapter: mock`，速率与站点一致以免时序窗口表现得像在 PC 上）。
名字带下划线是故意的：它不是候选模型，是**离线验收用的选择项**。理由写在项内的 `_what` 里——
"离线"必须是操作者**能选**的东西，不能是"模型没装好时恰好会发生的事"。

验证（本地，无相机、无 picamera2）：

```
$ .venv/bin/python -m perception.visiond --config perception/configs/perception_v1.json \
    --profile _mock --max-frames 6
visiond: synthetic 6 frames, 0 with tracks, 0 document pairs written
visiond: worker cam-385fcd67 state=stopped frames=6 dropped=5 generation=1
```

读法：`state=stopped`＝干净收尾（不是 dead）；`generation=1`＝未被重启过；
**`dropped=5` 是正确行为**——latest-tap 只有一格且这条路径上**尚无消费者**，故除首帧外每帧覆盖上一帧。
这个数字现在就是"第二路消费者还没接"的现场证据，而不是一个看起来像 bug 的计数。

套件：**17 failed / 985 passed / 1026 collected**（与改前逐项一致）。

### WP3 剩余（下一轮）

1. **把上面那条人工验证变成断言**：`test_visiond` 里加一例跑 `--profile _mock`（须 `quiet=False`，
   因为那行受 `--quiet` 抑制），断言出现 `state=stopped` 且 `frames == max_frames`。
   没有它，这行输出随时可以被人悄悄删掉而没人红。
2. **A3/T2 仍是 NOT_RUN**：站离线（主人刷 GM6020 固件），且需要真相机帧率。
3. 双相机（wide/detail）真接入属硬件到位后的事；`CameraTrackRegistry` 与 worker 已就位但**只有单路消费者**。

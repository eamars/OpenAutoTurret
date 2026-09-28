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
这是同一族陷阱的第三种打扮：协议声明了、发送方发了，**合并层没放行**——今天下午协议层是这一族，
`be_cmd` 那次是没声明，这次是合并白名单。下一刀：把两键加进 `webd/app.py` 的合并清单，
再把 `by-path` 节点配进 `open_picamera2`（那之后期望值＝`cam-f45ad188 / by-path`）。

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

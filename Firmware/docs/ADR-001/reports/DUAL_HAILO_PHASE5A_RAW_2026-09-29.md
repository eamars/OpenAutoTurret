# Phase 5-A · 裸 Hailo 吞吐（官方工具，不接相机）

`hailortcli benchmark`（**子命令与参数是读它自己的 `--help` 得到的**，没有猜；`--power-mode` 是运行时开关，没动 boot 配置）。
模型 `yolov8n.hef`（manifest 钉住，SHA-256 已核对），输入 **640×640×3 UINT8**，**batch 1**，输入为随机数据（没给 `--input-files`）。

| 跑法 | 聚合 FPS（hw_only） | 聚合 FPS（streaming） | HW 延迟 |
|---|---|---|---|
| 默认 15 s/模式，`performance` | **431.556** | **329.241** | **3.354 ms** |
| `--time-to-run 45 --power-mode ultra_performance` | **431.452** | 320.097 | 3.253 ms |

**读数怎么念：**
- **`hw_only` = 431 fps** 是 NPU 本身的上限（不含主机送数据的开销）。
- **`streaming` = 320–329 fps** 才是**这台机器真正能用**的上限：它含每次把 1.23 MB 输入搬进、结果搬出。
  ⇒ 320 fps × 1.23 MB ≈ **~400 MB/s 有效**，和 **Gen3 x1**（Phase 1：`current_link_width=1`）的量级吻合。
  **也就是说裸上限被 PCIe x1 压着，而不是被 26 TOPS 压着。**（要突破得换 x4 载板，属硬件改动，只报不改。）
- `ultra_performance` 没换来吞吐（431 不变、streaming 反而略低）⇒ **瓶颈不在核心供电档，在链路/主机。**

**对照需求：**双摄 30+30 fps = 60 次/秒，只占 streaming 上限的 **~19%**。
⇒ **设备侧有大把余量（约 5×）**；Phase 5-B 的天花板**大概率在主机侧**（缩放、色彩转换、Python、内存拷贝），这也是为什么矩阵要按"每路交付 + p99"来测，而不是问 Hailo 能不能。

## 顺手量到的一条真约束（对 10 分钟长跑很关键）

跑完之后：`vcgencmd measure_temp` = **54.9 °C**，`vcgencmd get_throttled` = **`0x50000`**。
按位念：**bit16 曾发生欠压** + **bit18 曾发生降频**（bit0/2/3 为 0 ⇒ **此刻没在降频**）。
而 Phase 1 空载时同一 boot 读到的是 `0x0`——**这些事件发生在今天这几次负载里**（Hailo 满载 + 双摄 + 4 核）。

⇒ 两条要报的：**① 主机的热余量已经很紧**（软温度限触发过，虽然 NPU 不吃这个亏）；
**② `0x50000` 的欠压位提示 5V 供电在满载时可能是 Marginal 的**（Hailo HAT + 两颗相机 + 4 核）。
Phase 5-B 的 10 分钟长跑**必须把这两条算进去**：架构师写的稳定判据里就有"无热降频"。

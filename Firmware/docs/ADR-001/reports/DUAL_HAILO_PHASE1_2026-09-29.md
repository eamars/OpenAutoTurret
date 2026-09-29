# Phase 1 · 这台机器到底是什么（双摄 → 单 Hailo 验证的第一阶段）

跑在 **rpi-turret 本机**（`scripts/collect_station_inventory.sh`，只读；全文快照：我家 `logs/drag/phase1_inventory.txt`，124 行）。
本地时刻 2026-09-29 上午。

## 1. 软硬件清单（实测，不是推断）

| 项 | 实测值 |
|---|---|
| 板 | **Raspberry Pi 5 Model B Rev 1.1**，aarch64，4 核，RAM 8062 MB（可用 7093 MB） |
| OS / kernel | Debian GNU/Linux 13 (trixie) / **6.18.39+rpt-rpi-2712** |
| 相机栈 | **rpicam-apps v1.13.0**（egl/qt/drm/libav 都在）、**libcamera v0.7.2+rpt20260817**、**Picamera2 0.3.37**（站点 venv）、numpy 2.2.4 |
| IMX500 相关包 | `imx500-all 1.13.0-1`、`imx500-firmware 0.FF23+3`、`imx500-models 1:1.0.0-1`、`imx500-tools` |
| Hailo | **Board Name: Hailo-8**、**Device Architecture: HAILO8**（与 manifest 声明一致）、PCIe **`0001:01:00.0`**、`hailortcli scan` 找到 1 台 |
| HailoRT | **4.23.0**（`hailort`、`hailort-pcie-driver`、`python3-hailort 4.23.0-1`；`hailo_platform.__version__ = 4.23.0`） |
| hailo-apps | **没装**：`hailoapp` / `hailomux` / `hailo_perf` / `hailo_camera_tool` 全部 `无` |
| GStreamer | 当时判定「整机没有」：`gst-launch-1.0`/`gst-inspect-1.0` 确实无，**但那条 `ldconfig -p \| grep -c gstreamer = 0` 是假的**——见下方「更正」 |
| HEF 编译器 | **无 `hailortc`** ⇒ 不能在机上重编 HEF / 改 batch |
| PCIe 实际链路 | `current_link_speed = **8.0 GT/s (Gen3)**`、`current_link_width = **1**`、`max_link_width = 4` ⇒ **跑在 Gen3 x1**（Pi 5 的 FPC 排线只给一条 lane；`max_link_speed` 也是 8.0，说明已是本机协商到的最高代） |
| 温控 | `get_throttled = 0x0`（无降频）、空载 `measure_temp 50.5 °C` / `thermal_zone0 48.50 °C` |
| 未装/缺口 | venv 的 `importlib.metadata` 查不到 PIL（探针仍能 `make_image` ⇒ PIL 来自系统 site-packages，**venv 带 system-site**，记着，别当成"没有"） |

`hailortcli --help` 亲自看过（不猜子命令）：可用 `run`、**`run2`**、`scan`、**`benchmark`**、`measure-power`。
⇒ **Phase 3 / Phase 5-A 的"裸吞吐"有官方工具**（`hailortcli benchmark`），不必自造。

## 2. 两台传感器：看得到、按身份认、原生模式是多少

`rpicam-hello --list-cameras`（**顺序只是当下的枚举结果，不作身份**；身份按 i2c 节点/路径）：

```
/sys/bus/i2c/devices/10-001a = imx500      /sys/bus/i2c/devices/11-001a = imx477
0 : imx500  [4056x3040 10-bit] (/base/axi/.../i2c@88000/imx500@1a)
    'SBGGR10_CSI2P': 2028x1520 @30.00, 4056x3040 @30.00
1 : imx477  [4056x3040 12-bit RGGB] (/base/axi/.../i2c@80000/imx477@1a)
    'SRGGB10_CSI2P': 1332x990 @30, 2028x1080 @30, 2028x1520 @30, 4056x2160 @30, 4056x3040 @30
    'SRGGB12_CSI2P': 1332x990 @30, 2028x1080 @30
```

**两条直接改掉 Phase 5 矩阵的实测事实：**

1. **这台机器上没有任何 >30 fps 的传感器模式。** 两颗的全部模式都只报 **30.00 fps**。
   ⇒ 架构师建议的 Test 3「IMX477 2028×1080 ~50 fps」**在本 stack 下不成立**（不是我没找到，是 sensor mode 表里没有）。
   广角也没有 1920×1080 原生模式——它是 2028×1520 / 4056×3040（**1080p 是 ISP 缩放出来的，不是传感器模式**）。
2. 两摄都在 CSI/MIPI 上（两个 PiSP pipeline，分别注册到 media3 / media4），**Phase 2 的前提成立**。

## 2.9 更正（同日，主人授权 sudo 之后重测）——**我 Phase 1 的"GStreamer 整机没有"是错的**

我那条"证据"长这样：`ldconfig -p | grep -c gstreamer` → **0**。真相是**我的 ssh 非登录 shell 的 PATH 里没有 `ldconfig`**
（它在 `/sbin/ldconfig`），命令本身 `command not found`，**`grep -c` 对着空输入忠实地数出 0 行**。
`dpkg -l | grep -c gstreamer` 当场数是 **11**：`libgstreamer1.0-0`、`-plugins-base/-good/-bad/-libav` 全都在
（被 rpicam-apps 的视频编码支持带进来的），**`hailort` 4.23.0 还自带 GStreamer 插件 `libgsthailo.so`**：
`hailonet`、`synchailonet`、`hailodevicestats` 三个元素**本来就在机器上**。缺的只是 CLI（`gstreamer1.0-tools`）。

主人授权后已装（**都是补丁级/同版本配套，没碰内核、没碰 PCIe 驱动、没重启**）：

| 包 | 结果 |
|---|---|
| `gstreamer1.0-tools` + `-plugins-base/-good/-bad` 升级到同系列 patch | `gst-inspect-1.0` 报 **264 插件 / 1372 元素** |
| `gstreamer1.0-libcamera`（版本 **0.7.2+rpt20260817-1，与已装 libcamera 完全一致**） | **`libcamerasrc` 有了**（可按 `camera-name` 选相机） |

⇒ **架构师要的"原生像素路径"在这台机器上是可用的**：`libcamerasrc → (ISP 缩放) → videoconvert → hailonet`，不需要 `hailo-all`。
**没装 `hailo-all`，是故意的**：它会把 **HailoRT 4.23.0 → 5.1.1**、**23 包升级 + 216 新装**，其中含 **`hailort-pcie-driver`**
⇒ 换的是正在跑的那块设备的内核驱动，**大概率要重启才干净**，而主人明确说"只要不重启就行"。要看详情：`apt-get -s install hailo-all`。
`hailoapp`/`hailomux`（TAPPAS 那套多源 mux）仍然没有——那是 5.x/TAPPAS 的东西，装它就要动 `hailo-all`。

**教训落在我家的账上**：`grep -c` 不区分"没有匹配"和"上游命令根本没跑成"。**一条会失败的上游命令配一个把空当 0 的计数，就是伪造了一个测量。**

## 3. 「优先用 Hailo 现成的多源 / GStreamer 基础设施」——**一半是错的：元素在，mux 不在**

架构师写的是 *prefer adapting Hailo's current multisource pipeline / GStreamer infrastructure rather
than creating an unnecessary custom inference scheduler*。实测的可用性是：

| 想要的路 | 这台机器上 | 判 |
|---|---|---|
| Hailo GStreamer 插件（`hailonet`） | **其实一直装着**（`hailort` 4.23.0 自带 `libgsthailo.so`：`hailonet` / `synchailonet` / `hailodevicestats`）——见「更正」 | **可用**（缺的只是 `gst-inspect`/`gst-launch` 这些 CLI 工具） |
| `hailoapp` / `hailomux`（官方多源 mux） | 全部不存在 | **不可用** |
| Python `HailoMultiStreamPipeline` | `ImportError`（python3-hailort 4.23.0 不导出） | **不可用** |
| 多流 HEF（batch>1 / multi-stream） | 只有 model-zoo 的 **batch-1** `yolov8n.hef`，且**机上无编译器** | **不可用**（除非拿到别处编好再传） |
| `hailortcli benchmark` / `run2` | **在** | ⇒ **裸吞吐与多网络用官方 CLI** |
| `hailo_platform.InferVStreams`（官方 Dataflow 层） | **在** | ⇒ **端到端用官方 Dataflow API，不自造调度器** |

**所以 Phase 4 的形状是：官方 Dataflow（`InferVStreams`）+ 每相机一条有界/泄漏队列 + camera_id 在 Python 侧随帧携带。**
"自造调度器"的批评在这里的边界是清楚的：**batch-1 的设备本来就只能交替喂**，我用的不是私有调度框架，
是官方 write/infer 调用加上"每路一个有界队列"的胶水——而且**这条路没有替代者可装**（无 sudo）。
这条我按"报告 + 继续做"处理，不停下来等 apt。

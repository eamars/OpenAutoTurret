# Pico W USB ↔ PIO UART 桥接器

用于把 RoboMaster Assistant 的 USB 虚拟串口连接到 GM6020 PWM/UART 口。
RP2040 PIO 负责 8N1 时序；USB CDC 的波特率设置会同步到 PIO。
支持 9600–1000000 baud，包括 Assistant 使用的 115200 和 921600。
不需要 DTR/RTS，也不向通信数据中插入日志。Wi-Fi 未启用。

## 接线

先完成 **GP0 与 GP1 短接、电机未连接** 的回环测试，再拔掉短接线接电机。

| Pico W | 物理脚 | GM6020 |
| --- | --- | --- |
| GP0，TX | 1 | 白色 PWM/RX |
| GP1，RX | 2 | 灰色 TX |
| GND | 3 | 黑色 GND |

Pico 用 USB 供电；电机 XT30 单独供 24V。**24V 不得连接 Pico 的任何引脚。**
不连接电机与 Pico 的电源正极。Pico GPIO 为 3.3V 电平。

按住 BOOTSEL 接 USB，把 `pico_uart_bridge.uf2` 复制到确认属于这块空闲板的
`RPI-RP2` 盘。它重启后枚举为 `Pico W PIO UART Bridge`，Windows 分配新的 COM 号。
本地开发 VID/PID 为 `CAFE:4001`，序列号来自板载 flash 唯一 ID。
这是开发用身份，不能作为商业 USB VID/PID 分配使用。

关闭其他串口程序，然后打开 RoboMaster Assistant v2.7。桥接器只原样转发字节，
不自动升级电机、不改变电机参数、不自行发出设备查询或运动指令。
升级前按站点运行手册处理原控制器的占用。

## 实机回环探测

必须移除电机接线，并短接 GP0/GP1。以下测试会发送随机二进制数据，**不能对电机运行**。
脚本会验证目标 COM 的 USB VID/PID，防止碰到现有 COM3 或其他串口。

在 Windows PowerShell 5.1 中运行（将 COM 号换为新桥接器实际端口）：

```powershell
powershell.exe -NoProfile -ExecutionPolicy Bypass -File Firmware/tools/pico_uart_bridge/probe_loopback.ps1 -Port COM6 -LoopbackConfirmed -Quick
powershell.exe -NoProfile -ExecutionPolicy Bypass -File Firmware/tools/pico_uart_bridge/probe_loopback.ps1 -Port COM6 -LoopbackConfirmed
```

第一条测试两个速率和 DTR 高/低。第二条在 921600 下每阶段传输 1 MiB，
115200 下传输 32 KiB，并在同一次打开串口期间切换速率；逐字节比较结果。
所有阶段必须 PASS。回环证明数据通路和切换行为，不能独立测量绝对波特率，
也不能替代 GM6020 识别与升级实测。

## 构建

本次使用 Pico SDK 2.1.1（`ee68c78d0afae2b69c03ae1a72bf5cc267a2d94c`）、
Arm GNU Toolchain 14.2.Rel1、Pico W / RP2040、Release，系统时钟使用 SDK 默认值。
SDK 取自用户现有 OpenTrickler 项目的 `library/pico-sdk`，只读使用。

Linux / WSL，工作目录为仓库根目录：

```bash
export PICO_SDK_PATH=/path/to/pico-sdk
export PICO_TOOLCHAIN_PATH=/path/to/arm-gnu-toolchain-14.2.rel1-x86_64-arm-none-eabi
python3 -m venv run/pico-tools/venv  # 仅首次创建；不需要额外 Python 包
cmake -S Firmware/tools/pico_uart_bridge -B run/pico-bridge-build \
  -DPICO_BOARD=pico_w -DCMAKE_BUILD_TYPE=Release -DPICO_NO_PICOTOOL=1 \
  -DPython3_EXECUTABLE="$PWD/run/pico-tools/venv/bin/python"
cmake --build run/pico-bridge-build -j8
run/pico-tools/venv/bin/python Firmware/tools/pico_uart_bridge/bin_to_uf2.py \
  run/pico-bridge-build/pico_uart_bridge.bin run/pico-bridge-build/pico_uart_bridge.uf2
```

SDK 2.1.1 的 pioasm 在较新主机 GCC 上可能需要强制包含 `cstdint`。
独立构建、安装主机工具后给固件 CMake 增加 `-Dpioasm_DIR=.../lib/cmake/pioasm`，
无需改 SDK：

```bash
cmake -S "$PICO_SDK_PATH/tools/pioasm" -B run/pico-tools/pioasm-build \
  -DCMAKE_CXX_FLAGS="-include cstdint" -DCMAKE_INSTALL_PREFIX="$PWD/run/pico-tools/pioasm-install"
cmake --build run/pico-tools/pioasm-build -j8
cmake --install run/pico-tools/pioasm-build
```

## 行为和边界

- PIO 使用 16.8 分数分频器，每位 8 个周期；按最近分频值取整。
- 核心 0 处理 TinyUSB，核心 1 持续服务 PIO；每个方向有 16 KiB 软件缓冲。
- 波特率改变时暂停新的 USB→UART 数据入队，等待旧 TX 队列排空后重配 PIO。
  主机应在两个报文之间切换速率，不应在仍有旧速率响应时切换。
- 仅支持 8 数据位、无校验、1 停止位；不支持的格式或速率会停止转发新 TX 数据。
- 没有硬件流控，主机长时间停止读取仍可能使 UART→USB 缓冲溢出。
  RAM 中的 `bridge_rx_overruns` / `bridge_framing_events` 计数器可由调试器检查。
- DTR 低不会复位、屏蔽收发或改变 GPIO；没有 1200-baud 自动 BOOTSEL 功能。
- 固件更新仍通过 BOOTSEL。不要把回环成功表述为电机固件升级已经验证。

PIO 时序参考 Raspberry Pi 官方
[uart_tx](https://github.com/raspberrypi/pico-examples/blob/master/pio/uart_tx/uart_tx.pio) 和
[uart_rx](https://github.com/raspberrypi/pico-examples/blob/master/pio/uart_rx/uart_rx.pio)。
GM6020 接线依据仓库内官方用户手册的“Using RoboMaster Assistant”章节。

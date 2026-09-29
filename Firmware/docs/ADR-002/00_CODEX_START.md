# ADR-002 · 给 Codex 的启动指令

你正在现有 OpenAutoTurret 工作区实施ADR-002：GM6020 yaw与CyberGear pitch闭环/摩擦/平滑运动。**不是再次做电压→电流迁移，不是全系统重构。**

先读取当前AGENTS、`Firmware/docs/STATION_OPERATIONS.md`、本包`ADR-002.md`和`docs/01_SOURCE_AUDIT.md`。新文档归档到`Firmware/docs/ADR-002/`。

## 第一次行动

在现有工作区执行只读核对：
```bash
git status --short
git branch --show-current
git rev-parse HEAD
git diff --stat
```
不要clone/全量下载另一个项目副本；不要pull覆盖工作区；保留用户未提交修改。把本地HEAD填入来源manifest，逐项核对所读符号是否仍与ADR一致；分支已变时按差异调整，不盲套文本补丁。

重点核对：
`Firmware/config/{mixed_hardware,turret_mixed}.yaml`、`gm6020_velocity.hpp`、`mixed_can_motor_backend.{hpp,cpp}`、`can_motor_backend.cpp`、`speed_servo.hpp`、`control_loop.{hpp,cpp}`、`homing_controller.cpp`、`payload_profile.{hpp,cpp}`。路径以当前工作区实际文件为准，来源表给出本次阅读路径。

## 必须保留的决定

Yaw：current/0x1FE；host速度PI输出A+有界摩擦补偿；内部电流环不乱调。
Pitch：优先RunMode2原生速度闭环；host仅位置P+速度前馈；LimitCur硬上限5A。
主机200Hz不先加速；实际反馈率独立测量；不把pitch约50Hz反馈请求误作内部环频率。

普通卡滞/短暂饱和/历史CAN错误不自动HOLD或全轴掉电。先修guard零电流与正常控制竞争、tick假装stall次数、累计counter永久故障和20ms迟到过度升级。真正失控/过期反馈/驱动危险仍及时采取保护；优先支撑承重轴，不等损坏发生后才停。

0A≠零速度≠位置保持≠禁能。最大**已合格**电流可用，不追求最小电流；不能把GM±3A协议满量程或1.62A额定直接批准为持续低速/堵转工况。保留初始0.8A与pitch5A边界直到对应资格有证据。温度unknown不能按0°C处理。

## 本轮实施

按PR1→PR2→PR3→PR4推进，见`docs/05_CODEX_PLAN_AND_ACCEPTANCE.md`。先只完成PR1并运行相关native测试，报告再继续；不要求用户为每个纯软件小修改重复确认，但硬件运动/部署须遵守实际授权。

PR1重点是把真实有效配置、raw RX/最终TX、运行状态和输出仲裁做对。PR2基于实测校准补偿；PR3先实测原生pitch速度环；PR4绑定真实hardware/mode/payload。未校准参数null/disabled，不猜一套production PID。

复用现有trace、reference manager、profile存储和launcher；不新建调参服务，不修改视觉模型、目标选择或UI框架。pitch current/MIT是有条件实验，不是强制工作。

## 测试和报告

宿主机native x86_64跑通用测试、宿主机交叉编译arm64；硬件试验只在Pi且只用既有单一CAN控制路径。不要在Pi编译，不运行并行CAN脚本，不自行`--activate`。每次gain/模式应用报告实际值和读回，通用yaw no-op setter不得报成功。

本包自带工具可以先在包目录运行：
```bash
python -m unittest discover -s tests -v
python tools/control_math.py
python tools/trace_metrics.py fixtures/synthetic_trace.csv --output reports/synthetic_metrics.json
```
它们只证明离线工具的行为，不证明机构稳定。最终报告逐项列实际测试与NOT_RUN；包括同工况前后指标、当前有效参数、source/deployed commit、未解决风险和可执行回滚办法。

# 来源与证据范围

日期2026-09-29。基线为可变分支`ADR-001`，完整HEAD未取得；必须由Codex在实际工作区填入`manifest.json`。没有clone、repo ZIP、全量下载或连接Pi。本包没有仓库源码副本。行号仅帮助定位，分支变化后应按符号核对。

[S]为直接阅读的项目源码/规则，[R]为项目作者报告/整理资料；后者不是本包亲自实测。[V]为厂商原始资料。正文的新公式、实验建议、阈值和修复方案是设计判断，不是假装来源已实现。

## S01
[AGENTS.md](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/AGENTS.md)

工作区/构建/现场入口规则。

## S02
[Firmware/config/mixed_hardware.yaml](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/config/mixed_hardware.yaml)

两轴模式、current增益与上限；约L1–69。

## S03
[Firmware/config/turret_mixed.yaml](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/config/turret_mixed.yaml)

200Hz、分轴cap、homing、payload、服务模式；约L31–275。

## S04
[Firmware/control/src/can/gm6020_velocity.hpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/can/gm6020_velocity.hpp)

现PI、50ms滤波、20ms invalid；约L16–79。

## S05
[Firmware/control/src/control/mixed_can_motor_backend.cpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/control/mixed_can_motor_backend.cpp)

guard、CAN健康、snapshot和命令路径；重点L361–562、命令尾部。

## S06
[Firmware/control/src/control/mixed_can_motor_backend.hpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/control/mixed_can_motor_backend.hpp)

guard枚举、streak=3、typed zero；约L25–87。

## S07
[Firmware/control/src/control/can_motor_backend.cpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/control/can_motor_backend.cpp)

模式切换/读回、SpdRef、keepalive、gain setter；约L300–802。

## S08
[Firmware/control/src/control/control_loop.cpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/control/control_loop.cpp)

watchdog、fresh sample、服务servo、payload信任/限幅；重点L729–822、服务段、L3031–3077。

## S09
[Firmware/control/src/can/cybergear_protocol.hpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/can/cybergear_protocol.hpp)

RunMode/IqRef/SpdRef/LimitCur/Iqf与gain地址；约L103–185。

## S10
[Firmware/control/src/calibration/homing_controller.cpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/calibration/homing_controller.cpp)

接近speed、退让position、jitter诊断；约L46–181。

## S11
[Firmware/control/src/control/safety_supervisor.hpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/control/safety_supervisor.hpp)

现有安全动作与反馈/温度处理。

## S12
[Firmware/control/src/can/gm6020_protocol.hpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/can/gm6020_protocol.hpp)

电流缩放、协议帧、1.62持续上限常量。

## S13
[Firmware/config/payload_profiles/conservative.yaml](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/config/payload_profiles/conservative.yaml)

旧双CyberGear UID与2026-09-02历史成绩；全72行。

## S14
[Firmware/control/src/main.cpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/main.cpp)

实际payload加载与默认trusted调用；约L420–439，另查看入口/运行配置段。

## S15
[Firmware/control/src/payload/payload_profile.hpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/payload/payload_profile.hpp)

schema1、informational gains、原子保存接口；约L23–84。

## S16
[Firmware/control/src/control/control_loop.hpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/control/control_loop.hpp)

100ms遥测判稳估计、payload默认commissioned=true；约L295–315、L489–518。

## S17
[Firmware/control/src/control/speed_servo.hpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/control/speed_servo.hpp)

位置P+FF、quiet迟滞、error cap、a/j；全43行。

## S18
[Firmware/control/src/calibration/contact_detector.hpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/calibration/contact_detector.hpp)

接触与jitter判据；全149行。

## S19
[Firmware/control/src/config/mixed_hardware_profile.cpp](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/config/mixed_hardware_profile.cpp)

current字段解析/单位/限流/帧mode匹配；约L163–275。

## S20
[Firmware/docs/STATION_OPERATIONS.md](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/docs/STATION_OPERATIONS.md)

部署/运行规则与现场记录；选择性章节，文档状态需与最新报告区别。

## S21
[Firmware/docs/README.md](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/docs/README.md)

新文档归ADR-002目录。

## R01
[Firmware/docs/ADR-001/reports/CURRENT_MODE_MIGRATION_2026-09-28.md](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/docs/ADR-001/reports/CURRENT_MODE_MIGRATION_2026-09-28.md)

append-only迁移报告；优先后部2026-09-29 01:0x以及L172–219；实机成绩为作者报告。

## R02
[Firmware/docs/ADR-001/sources/GM6020_DRIVE_CONFIG_2026-09-28.md](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/docs/ADR-001/sources/GM6020_DRIVE_CONFIG_2026-09-28.md)

Assistant Current Ring和PWM/CAN区别；更正段优先。

## R03
[Firmware/docs/references/gm6020/GM6020_AI_Reference.md](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/docs/references/gm6020/GM6020_AI_Reference.md)

手册索引；其中早期current未知已被新报告部分取代。

## R04
[Firmware/docs/references/cybergear/CyberGear_AI_Reference.md](https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/docs/references/cybergear/CyberGear_AI_Reference.md)

仓库整理的手册参考；查阅协议/寄存器/模式相关节，不声称独立审核全部原PDF。

## V01
[DJI / RoboMaster GM6020 User Guide v1.4, 2023.10](https://rm-static.djicdn.com/tem/17348/RM%20GM6020%20%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%EF%BC%88%E8%8B%B1%EF%BC%8920231103.pdf)。已直接读取官方PDF，并检查印刷页7、8、10、11的截图。页7：0x1FE/0x2FE电流表与±16384↔±3A；页8：1kHz反馈、13bit位置与rpm；页10：工作范围/实验环境；页11：额定1.62A、连续堵转0.90A及名义常数。未随包复制PDF。

## 未完成的源码阅读
本包没有完整阅读所有controller/loader/transport/tests，也没有完整审计payload_profile.cpp。具体实现时应沿所读调用路径补看必要函数；不能把“当前调用未检查”夸大成全仓库证明从未检查。CyberGear资料是该项目整理的参考，本次未独立读取其原始中文PDF，因此不以其中社区/存储表推翻本机已验证CAN模式。

## 时间线处理
旧N1交接书的yaw电压、±90°虚拟运动界限不作为当前事实。当前配置是current且position_envelope=none。迁移报告早期“backend未实现”被后部实现记录取代；同一报告对PWM影响的猜测不能覆盖厂商明确的CAN/PWM模式区别。最新所读迁移节表示站台停止/未重启，本包未验证实时状态，不能声称当前正在运行或新固件已实机合格。

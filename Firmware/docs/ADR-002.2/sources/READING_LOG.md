# 来源、审计范围与事实等级

日期2026-09-30。文档中的[Dn]或无来源的方法为本包设计，不是既有源码能力或实机结果。

## 用户与已提供文件

- [U1] 本串用户提供的Codex执行行为报告，记录时间2026-09-30 00:12 NZDT。没有访问其C:\workspace路径或原始trace。
- [U2] 用户在本串对ADR-002.2的讨论：模型/实测反馈/验证，独立程序，基于数学，不人工选参，可适应payload/摩擦变化。
- [U3] 本轮用户最终要求：无“首版/终版”拆分；1/2资产硬件未变可复用；3a独立程序和3bproduction都通过；生成开发包。
- [P1] OpenAutoTurret_ADR-002_Development.md，2026-09-29。Files读取控制设计部分；不把它作为新源码证明。
- [P2] OpenAutoTurret_ADR-002.1_Development.md，2026-09-30。Files读取正式决策：确认16+8搜索、保护/回执、既有模式和范围。
- [H1] 用户提供的《OpenAutoTurret 当前架构与架构师决策交接书》，2026-09-27，最近激活8901808。只用作历史硬件/IMU位置与未完成标定线索；其yaw电压/旧运行状态不能覆盖后来current配置。引用位置：原文件第38–48、54–67、83行。

## 少量公开源码/项目资料（非固定commit完整审计）

- [R1] ADR-001 README、AGENTS、文档索引；另读取main README识别其“状态是历史记录”的提醒。
  - https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/README.md
  - https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/AGENTS.md
  - https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/docs/README.md
  - https://raw.githubusercontent.com/eamars/OpenAutoTurret/main/README.md
- [R2] https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/config/mixed_hardware.yaml
  - yaw current配置及现有限流；pitch配置字段为position。配置注释中的额定/保护推断未经本次独立认证，不作为突破热边界的依据。
- [R3] https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/control/can_motor_backend.cpp
  - 阅读mode切换、position/speed、读回及负载漂移相关段，不声称所有函数都经审计。
- [R4] https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/docs/references/cybergear/CyberGear_AI_Reference.md
  - 项目整理的手册转录；只使用其标注OFFICIAL-MANUAL的mode/runtime字段和限制含义作设计依据，不以社区补充为当前能力。不是本次独立审阅原厂PDF，不把表范围当机构授权。
- [R5] https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/control/mixed_can_motor_backend.hpp
- [R6] https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/control/src/can/gm6020_velocity.hpp
  - 只用于定位现有主机速度环与混合后端，不将旧公开内容冒充报告里的bad742d。

读取未克隆整库、未下载模型或所有源码。API取得ADR-001完整HEAD的尝试未成功；容器直接请求也遇DNS不可达。`base_commit=null`，由Codex在实际工作区记录并核对。没有新证据证明bad742d与此公开分支相同。

## 数学/工具一手来源

- [M1] MIT Underactuated Robotics, System Identification：结构化机械系统辨识、可辨识组合参数、预测验证。
  https://underactuated.mit.edu/sysid.html
- [M2] SciPy官方lsq_linear文档：带边界线性最小二乘。
  https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.lsq_linear.html
- [M3] SciPy官方least_squares文档：有界非线性最小二乘与鲁棒loss。
  https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html
- [M4] SciPy官方cont2discrete文档：离散化方法。
  https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.cont2discrete.html

引用这些来源不代表它们背书本包阈值、控制器公式适用于未测机构，或证明实现已稳定。本包公式为明确假设下的推导；全套物理验收仍需执行。

## 本包实际工作

生成设计文档、schema、离线回归/PI计算/复用与双门验证参考、合成数据和单测；结果见reports。未读取当前本地源码树、编译C++、连接Pi、部署/启动电机、测量温度或重新分析Codex原始数据。

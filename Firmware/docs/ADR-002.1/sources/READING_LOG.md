# 来源与核验边界

## [U1] 用户直接提供的执行报告

题名：ADR-002 执行行为报告：人工调参与停止交接。
记录时间：2026-09-30 00:12 NZDT。
本轮用户消息完整给出；本包据此审查执行过程。没有重新读取其声称位于 C:\workspace\OpenAutoTurret\run\adr002\ 的原始证据，未把该路径当成本工具环境可访问目录。

## [P1] 上一版生成文档

OpenAutoTurret_ADR-002_Development.md，2026-09-29。
本次通过 Files 读取决策与完整调试章节；确认非武器化云台范围、分轴模式、保护原则、开放式候选调试表达及目标值。它是先前设计，不是当前源码证明。

## [P2] 上一版 Codex 启动说明

OpenAutoTurret_ADR-002/00_CODEX_START.md，2026-09-29。
本次通过 Files 读取完整文件；其已写明 host 构建、实际参数读回、PR 顺序和不自动部署等要求。

## [R1] 选择性公开读取

读取日期：2026-09-30（本会话）。只读 README、文档索引和 AGENTS；未克隆/拉取整库。

- https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/README.md
- https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/AGENTS.md
- https://raw.githubusercontent.com/eamars/OpenAutoTurret/ADR-001/Firmware/docs/README.md
- https://raw.githubusercontent.com/eamars/OpenAutoTurret/main/README.md

README 自身说明部分状态是历史记录。AGENTS 使用 physical camera station 表述。它们不证明本地 bad742d 或未提交 PR3 的实际实现。

## [R2] 未取得的版本

报告提交：bad742dddba6a95d3d055e43998adfd0506b0a6e。
对该提交以下 raw 文件的读取均返回 HTTP 404：

- Firmware/control/src/can/gm6020_velocity.hpp
- Firmware/control/src/control/mixed_can_motor_backend.hpp
- Firmware/config/mixed_hardware.yaml
- Firmware/docs/ADR-002/00_CODEX_START.md

因此本次 source_audit_of_reported_commit=false。404 不证明本地提交不存在，也不证明一定没有推送；只说明本次公开路径无法取得。全部新控制策略是本包设计，不冒充对该版本源码的新发现。

## 未进行的操作

未连接 Pi；未访问本地 C 盘；未启动运动；未修改固件/保护；未编译 C++；未测试真实电机；未取温度或执行热平衡试验。开发包离线验证使用完全合成数据。

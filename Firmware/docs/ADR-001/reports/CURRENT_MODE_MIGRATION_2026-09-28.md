# GM6020 yaw：电压 → 力矩电流 迁移（进行中）

任务来源：主人转来 ChatGPT 的任务书（2026-09-28 晚）。本文是**进度与施工图**，不是完成报告。
今晚的动机：yaw 带滑环、阻力随角度不均匀，电压模式给不出需要的**力矩**；电流模式由驱动器内部闭环，
才是这个问题的正道。**但电流模式的前置条件我们至今没验证**（见"未验证"）。

## 已完成（本地可证，已跑）

**协议层** `control/src/can/gm6020_protocol.hpp`：

- `current_raw_uncapped(double amps)` —— 纯换算 `raw = round(A / (3.0/16384))`，非有限值抛、越 ±16384 抛。
- `current_raw_from_amps(amps, limit_a)` —— **安培单位下先钳位、再编码**；`limit_a` 必须有限且 >0，
  且 **> 1.62 A（DJI 连续额定）直接拒绝**：软件不许自己把常态上限抬上去。
- `current_frame(motor_id, amps, limit_a)` —— **专用的电流帧构造器**（不挪用 `voltage_frame`）：
  `id=0x1FE`、标准帧、`dlc=8`、ID 1 占 `DATA[0:1]` 大端有符号、`DATA[2..7]` 保持 0；**只对 ID 1 资格化**，
  `motor_id != 1` 抛（第二台是假设，不是事实）。
- 常量：`kRawFullScale=16384`、`kAmpsFullScale=3.0`、`kMaxContinuousA=1.62`、`kAmpsPerRaw`。

**测试** `control/tests/test_gm6020_transport.cpp` 新增 6 例（`ctest -R gm6020` 绿；全量 **77/77**）：
0x1FE/标准/DLC8/未用槽位为 0；`0.8 A → 4369 → 0x1111`；`-0.5 A → 0xF555`（二补）；
端点 `0/±3 A → 0/±16384`；**钳位先于编码**（2.0 A @ 0.8 A 上限 == 0.8 A）；
NaN / 上限 0 / 上限负 / 上限 3.0 / ID 2 全部抛（失败关闭）。

## 未做（下一场照此施工，顺序即依赖顺序）

1. **配置** `mixed_hardware_profile.{hpp,cpp}`：`ControlMode` 加 `Current`；解析/校验
   `control_mode: current` 必须配 `command_frame_id: 0x1FE`、`feedback_frame_id: 0x205`，
   且**电压/电流命令 ID 不得混用**；要求**显式外部配置确认位**，建议
   `current_ring_verified: true`（缺省即失败关闭，日志必须打印"未确认 ⇒ 电流模式未启用"）。
   新增主机侧电流上限（安培、有限、>0），**初始标定值 0.8 A**（不是协议满量程 3 A）。
   `turret_mixed.yaml` 里 yaw 的 `limit_cur_a: 0.0` 语义改为**主机命令钳位**（旧注释"GM6020 无软件
   CurrentLimit 寄存器"作废），并写清单位。
2. **backend** `mixed_can_motor_backend.cpp`：yaw 速度环输出语义从"raw 电压计数"改为**明确标单位的
   电流努力量**（`kYawOutputCeiling/kYawVelocityKp/kYawVelocityKi` 是电压单位的产物，
   **不得原样挪用**冒充电流增益；改名 + 独立可配 + 标注"待实物标定"）。
   保留全部现有语义：位置环、加速度成形、连续角/会话参考、反馈新鲜度、CAN 健康、速度守卫、
   无进展守卫、温度守卫、心跳。**每条归零路径改发电流 0**（启动 burst、`send_yaw_zero_locked()`、
   trip/fault、`close()`、正常关机、探针清理），并把日志措辞从 "zero voltage requested" 改成
   **"zero current requested"**，同时保留 **"disable state unavailable"**（不得写成 de-energized）。
3. **探针/启动器**：`--yaw-voltage` 不得"顺手"当电流解释；加 `--yaw-current-a`；旧电压探针若保留，
   标注 legacy 并在 profile 选 current 时拒绝误用。`probe_yaw_motion` 记录**命令电流**（不是
   `voltage_raw`），行程/速度边界、CAN 健康、反馈新鲜、心跳、末尾静止核验、**单一 CAN 拥有者**全部保留。
4. **回归测试**：编码器解码/解卷不变；`0x205` 反馈解析不变（**`current_raw` 的工程单位换算仍未验证，
   不得换算成安培**）；CyberGear pitch 不变；mixed 总线所有权不变。

## 未验证（不许被"代码写了"冒充）

- **固件 ≥ v1.0.11.2** 与 **Current Ring 已使能**：本仓库**从不读设备固件版本**（`gm6020*` 无版本查询），
  也没人记录过。**这两项必须由操作者确认并落进配置**，否则电流模式不启用。
- 因此"电流模式能解决今晚的哼唧"仍是**假设**；今晚已证的只有：旧 9000 电压上限推不动、抬到 15000 能动。
- `0x205` 反馈 `current_raw` 的单位/刻度：仓库现按"未验证"处理，本次迁移不引入换算。

## 硬件标定边界（顺序不可颠倒）

固件确认 → Assistant 开 Current Ring → 配置记录确认位 → 停 controld（保证单一 CAN 拥有者）→
先只做**接收**验 `0x205` 与总线健康 → `0x1FE` 打一次极小 bounded 电流 / 零 / 反向 / 零 →
核对编码器方向、响应、零电流静止、无 CAN 错误 → 才跑 bounded 行程标定测试并整定电流模式 PI。
**首轮远低于 0.8 A**，不用 ±3 A。

建议的探针调用形式（**待实现，不是已可用命令**）：
`Firmware/tools/probe_yaw_motion --yaw-current-a 0.2 --travel-deg 30`（其余边界沿用现有默认）。

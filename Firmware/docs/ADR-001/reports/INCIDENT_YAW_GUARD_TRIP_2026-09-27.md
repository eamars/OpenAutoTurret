# 事故验证报告：yaw GM6020 温度守卫跳闸（2026-09-27）

状态：站点锁存 `phase=fault`，安全（yaw 零电压、pitch 保持、CAN 双总线 ErrorActive）。
本报告是日期化测量记录，不是平行事实来源；事发后读数以站内为准。

## 1. 一句话结论

yaw（GM6020）在 MANUAL/HOLD 静止状态下，状态帧温度原始字节爬到 45，
撞上 `mixed_can_motor_backend.cpp` 里硬编码的 `kYawTemperatureRawCeiling = 45`（`>=` 触发），
独立守卫线程跳闸 → 控制环锁存 fault。其余全部守卫条件当时均为绿。
**不是松动、不是 CAN、不是编码器、不是控制环超时。**

## 2. 时间线（本地时区，站端日志）

| 本地 | 事件 |
|---|---|
| ~18:04 | 栈随 Pi 重启上线，18:04:45 完成归零；此后 AUTO_ROAM→AUTO_TRACK→取消→MANUAL jog（主人在场操作） |
| 18:0x–21:06 | 长时间 MANUAL/HOLD，yaw 持续带电保持（voltage 模式），累计约 3 小时 |
| 20:20–21:06 | 零星 `SLOW CYCLE 8–9 ms (phase=hold, action=DERATE)`，与本故障无因果证据 |
| **21:06:14.411** | `GM6020 guard trip`（触发字段全列，见 §3） |
| 21:06:14.413 | `black-box scene preserved (id 3668)`；`control fault: independent motor watchdog: control deadline, feedback, or motor health` |
| 21:06 → 21:23+ | `phase=fault` 持续 ≥17 分钟不自恢复；心跳里 `temp_yaw=nan` |

## 3. 触发条件矩阵（trip 日志行实录）

```
feedback_safe=true reference_valid=true received=true encoder_valid=true
feedback_age_ms=0.683 can_up=true can_state=0(ErrorActive) rxerr=0 txfail=0
both_buses_healthy=true measured_speed_deg_s=0.000 temp_raw=45
requested_speed_deg_s=0.000 no_progress_ms=0 heartbeat_seen=true heartbeat_age_ms=3
```

对照 `mixed_can_motor_backend.cpp:439-446` 的九个析取项，逐项排除后只剩
`temperature_raw >= kYawTemperatureRawCeiling`（常量 45，定义在 :36）。

## 4. 源码级发现（这比跳闸本身更要紧）

1. **同一物理量存在两个真相源**：控制环层的过温是配置驱动
   `motor_overtemp_c: 75.0`（`turret_mixed.yaml:176`，§38），而 mixed backend 的 yaw 守卫
   用硬编码 `45`（raw 单位）。配置说 75，代码拿 45 拦人——本工作区架构规矩第 1 条的反例。
2. **单位未建立**：`gm6020_protocol.hpp:4` 自己写明 "Current/temperature remain raw:
   the guide does not establish feedback scaling"。守卫拿一个单位未确认的 uint8 去比一个
   无出处注释的 45。若该字节是 °C（DBUS 手册口径），阈值等于 45 °C，比 §38 严 30 度；
   若不是 °C，则两边都不知道自己在拦什么。
3. **致命量不在遥测里**：守卫能读到 `temp_raw`，但 mixed backend 的 yaw 快照只填
   `temperature_raw_valid`，不填 `temperature_known/temp_c`（mixed_can_motor_backend.cpp:348），
   控制环于是发 `temp_yaw=nan`（control_loop.cpp:621）。**HUD 上看不到 yaw 在升温，
   直到它掀桌。**
4. **无迟滞、无自动恢复、fault 串吞掉原因**：`>=45` 一跳即锁存；trip 后只发零电压帧；
   复位只存在于 backend 的 open/close（:137/:219/:256）——运行期没有任何 clear 路径。
   telemetry 的 fault 串只写"watchdog: deadline, feedback, or health"，八种病共用一句话，
   细节只在日志里（黑匣子场景 3668 已留存，导出路径待查）。

## 5. 现场状态与影响

- 事发姿态 yaw +0.6167 rad（+35.3°）、pitch −0.7826 rad（−44.8°），跳闸后未再移动。
- yaw 零电压（按包内契约：零电压 ≠ disable ≠ 断能证明）；yaw 竖轴无重力风险。
- 跳闸时站点在 MANUAL/HOLD，**无人在跟踪、无人在 HUD**；无财产与人身事件。
- 本次没有降低任何已有硬保护来换取成功日志；守卫按其（存疑的）设计正确行动。

## 6. 恢复选项（等主人签字，均不在本报告内执行）

| 选项 | 动作 | 运动 | 备注 |
|---|---|---|---|
| A | launcher 重启栈 | **不确定** | 18:04 那次开机 45 秒后跑了归零；开机是否自动归零需先从 launcher 流程证实，别当成"纯软重启" |
| B | `start_homing` | 是 | 站内既定恢复路：`recovery_before_homing` 自动进 Recovering，清 trip、验证驱动、温度须 <75 °C（已停发电 20+ 分钟，大概率已满足）、随后归零行程 |
| C | 不动 | 否 | fault 锁存本身安全，可留到他亲眼看 |

任一运动选项执行前：验膛验匣（"没弹"是条件不是状态）＋ 回答"目标问题"＋ 清场。

## 7. 纳入开发计划的条目（建议）

**进 WP1（事件契约与时间戳语义，正是本包的题眼）**：
1. 守卫跳闸必须产出结构化事件：`{axis, condition_enum, temp_raw, 全守卫矩阵, 单调钟+墙钟}`，
   telemetry 的 fault 字段携带 condition_enum，不许八种病共用一句话。
2. yaw 温度接通到 telemetry（`temp_c`/`temperature_known`），HUD 可见。
3. 黑匣子场景（本例 id 3668）给一条只读导出路径，随事件留指针。

**进 WP2 或紧随其后的守卫治理片（要现场热数据，不许拍脑袋放宽）**：
4. `kYawTemperatureRawCeiling` 改为配置驱动＋单位建立（用 IR 温度计对读数两三个工作点，
   确认 raw 字节是否 °C），与 §38 的 `motor_overtemp_c` 合并成一个真相源；
   阈值数值本身的调整必须等热数据与 GM6020 绕组额定，本阶段**不降**。
5. 加迟滞（如跳 45 / 复 40）与"温度回落后可重新 arm"的运行期路径，替代只能重启。
6. yaw 长时间带电保持的发热是根因：把"空闲 yaw 断能/降保持"列入运动策略议题（与载荷包线同批讨论）。

## 8. 尚缺证据

- GM6020 温度字节单位（raw 是否 °C）——现场 IR 对照可补。
- 跳闸时刻之后温度回落曲线——遥测没接线，拿不到（第 7.2 条的理由）。
- 黑匣子场景 3668 的落盘位置与格式——待查 webd/controld。
- launcher 重启是否自动触发归零——读 `run_application.sh` 流程可答，未读。
- Pi 端无 can-utils，未做 CAN 帧直读复核（本容器同缺，装在途）。

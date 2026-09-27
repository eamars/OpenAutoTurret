# 01 · 基线与不可混淆的参数

## 1. 证据边界

[U] 基线是 2026-09-27、`codex/hardware-adaptation`；最近实际激活版本为 `8901808`。运行中发生 `get_throttled=0x50005`，已受控停止，尚无修复后合格负载复测。[U L3–L5；R1]

本包通过网页按文件读取文档和配置；没有 clone、下载仓库压缩包或拉取所有源码，没有读取 Pi 私有 `run/` 数据，没有连接站台。GitHub 分支 tip 的完整 SHA 未独立取得，故不能把本包称作对 `8901808` 全部源代码的审计。Codex 必须在本地补录 HEAD 与解析后配置。

## 2. 当前系统事实

| 项目 | 当前资料支持的事实 | 不支持的推断 |
|---|---|---|
| 控制 | C++ controld，目标 200 Hz；单一运动 owner | 200 Hz 新视觉或硬实时保证 |
| yaw | GM6020直驱、can0、1 Mbps经典CAN、ID1；反馈0x205、命令0x1FF | 与pitch相同的限流/禁能语义 |
| pitch | CyberGear直驱、can1、ID0x7F；UID7216313130333105；报告固件1.2.1.5；5 A上限 | 5 A读回已证明瞬态峰值或机械端挡承受能力 |
| yaw范围 | 会话虚拟±90°、内缩10°，工作±80° | 绝对方位、物理home、已验收整圈 |
| pitch范围 | 归零后有效限位；有机械端挡 | 配置 expected_travel ±70°就是本次实测端点 |
| 视觉 | 正常IMX500 SSD约26 Hz；每次visiond一个camera/model profile | 两摄融合已实现 |
| 窄角实验 | IMX477 + 25mm F1.4（操作者报告）；640×480/15Hz→640×640 Hailo YOLOv8n | 全幅名义FOV就是该流的有效FOV |
| IMU | pitch组件BNO085，约50Hz；相对tare与观察日志 | base姿态、绝对朝向或已启用稳定控制 |
| UI | 约15Hz状态、低优先级约10Hz预览 | 浏览器画面是控制时间真值 |

来源：[U L15–L85；R1–R3、R6–R8]。原始协议细节继续以仓库电机参考和现场固件为准，本包不重写驱动协议。

## 3. 参数审计表

以下是**文件中出现的值**，不是最终作用于电机的保证。必须让配置解析器输出来源与有效值，确认模式/轴限幅的真实优先级。[R3]

| 配置 | 文件值 | 本阶段处置 |
|---|---|---|
| control.loop_hz | 200 | 保留；记录计算耗时和调度lateness两类指标 |
| safety.feedback_max_age_ms | 100 | 保留硬新鲜度门槛；性能目标另设，不借优化提高硬门槛 |
| safety.deadline_max_us | 2000 | 查清这是计算耗时还是周期；不能当成5ms循环周期 |
| safety.motor_overtemp_c | 75 | 保留运行配置；commissioning可采用更低阶段温度，二者分开 |
| homing.speed_kp / speed_ki | 4 / 0.05 | 是历史保留值，不是新驱动辨识结果 |
| homing.motion_checks_abort | false | 不作为新验收通过条件；区分hard guard与shadow指标后修复 |
| coarse / fine / backoff | 5°/s、3°/s、5° | 记录分阶段表现，不以改快掩盖抖动 |
| contact_dwell / repeatability | 1500ms / 0.5° | 触端阈值的候选基线，需新机构证据 |
| homing.contact.stall_velocity_threshold | 0.5rad/s≈28.65°/s | 远高于3–5°/s接近速度；不能独立证明“停住” |
| homing.contact.v_move_threshold | 0.04rad/s≈2.29°/s | 保留单位；不要和上行阈值混为一谈 |
| homing.contact.torque_safety_nm | 10 | 注释引用旧机构；不能当作当前端挡的合格扭矩上限 |
| tracking.motor_response_ms | 120 | 待替换的估计，不能与完整实测延迟重复相加 |
| aim_point | box_fraction，(0.50,0.22) | 标记 body_upper，不标记 head_detected |
| v3.service_speed_kp/ki | 4 / 0.05 | 审计host与CyberGear内部环是否重复积分 |
| auto selection dwell | 500ms | 冷启动策略延迟；不重复用于已选目标每帧更新 |
| acquire / coast / lost_hold / roam | 50 / 250 / 500 / 1000ms | 保留baseline，之后按分段时延和实测制动重新评估 |
| motion.modes.auto_track target | 20°/s、30°/s²、100°/s³ | 和maximum、axis限幅存在作用层问题；不得据此保证最终20°/s |
| axes.yaw max | 10°/s、15°/s²、60°/s³ | 与v3模式配置不同，先查解析与应用路径 |
| motors.yaw.direction_sign | -1，注释为placeholder | 方向试验与最终sign必须显式验证 |
| payload.auto_verify | false | 不能把载荷验证描述为已有能力 |

## 4. 模型与标定的三处陷阱

正常launcher采用 `person_detect_available`，但 JSON 的独立默认profile为 `person_detect`，后者是另一条YOLO11n配置。benchmark必须记录**实际启动参数和resolved profile**，不能只读JSON默认字段。[R5]

Hailo manifest声明输出阈值0.5，而tracker配置含0.15/0.25/0.30。这些更低tracker阈值未必能接收到被设备NMS截断的框。先核查HEF可调阈值及实际返回分布，再决定调硬件后处理或tracker阈值；不能声称BYTE低分关联已有效。[R5–R6]

`camera_install.yaml`为 `rotate_180`，Hailo profile中 `camera_orientation=none`。这可能是有意的逐相机安装差异，不直接判为bug；必须记录每相机变换链、模型输入与预览方向，验证只应用一次。[R5、R7]

IMX500旧1920×1080内参和IMX477名义14.33°×10.77°只作规划输入，不给新双摄控制授权。[U L52]

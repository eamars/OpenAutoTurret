# 01 · 选择性源码审计

本表不是完整仓库审计；数字只描述所读 `ADR-001` 内容，现场部署和分支 SHA 尚未核对。来源编号见 [SOURCES](../sources/SOURCES.md)。相对路径均从仓库根目录起。新增判断不能当实机测量。

## A. 当前控制/反馈路径

| 项目 | 源码/资料中的事实 | ADR-002 含义 |
|---|---|---|
| yaw drive | GM6020，can0，ID1，current，0x1FE，反馈0x205 | 保留电流模式，不再写电压迁移 [S02,S12] |
| yaw controller | Kp=1.0，Ki=0.6，host ceiling=0.8 A；已有条件积分抗饱和 | 增益未经本次载荷整定，不是固定0.8 A输出 [S02,S04] |
| yaw feedback | 厂商1 kHz，仓库探针记录约1 kHz | 接收频率与有效主机速度带宽不同 [V01,R02] |
| host | loop_hz=200，名义5ms | 保留，先测周期尾延迟 [S03] |
| pitch drive | 声明position；服务`service_speed_control=true`，backend运行speed=2 | “声明模式”和“当前寄存器模式”分开报告 [S02,S03,S07,S08] |
| pitch feedback | SpdRef变化时发命令；否则 age>15ms、ping interval>=20ms | 静态/恒速约50Hz请求，变化时可更多；不能当内部环频率 [S07] |
| pitch gains | 服务/归零都kp4、ki0.05，写0x701F/0x7020 | 分轴、分用途整定；不把Ki当Kd [S03,S07,S09] |
| pitch authority | LimitCur最高5 A；setup需读回后使能 | 不是可以自动上调到厂商协议23A [S02,S07,S09] |
| estimates | yaw PI速度滤波50ms；ControlLoop判稳/遥测速度100ms | 不是简单串联150ms；两条估计给不同消费者，必须标识来源 [S04,S08,S16] |
| homing | pitch接近speed，退让position；yaw只建会话相对参考 | 用户描述“yaw归零波动”要按时间线区分pitch归零期间yaw漂移、ready/parking、服务运动 [S08,S10] |
| payload | auto_verify=false，但main仍加载conservative；启动set_payload_profile默认commissioned=true | 不得宣称关闭自动检查就不会使用旧档案 [S03,S08,S13,S14,S15] |

## B. 必须先解决的源码问题

### F01：无进展 guard 与主机正常输出竞争

`mixed_can_motor_backend.cpp::yaw_guard_loop`：需求>=5°/s、持续约1.5s没有至少3编码器计数进展时，no_progress为真。`yaw_stall_streak_` 在 guard 的每次约5ms循环增加，不是完成一次独立“起动→失败”事件才增加。第三次以后进入 GuardResponse::Hold，随后每轮 `send_yaw_zero_locked()`；另一线程仍可在正常 `command_velocity()` 中写电流。[S05,S06]

**源码确认**：存在两个逻辑输出决定；互斥锁只序列化发送，不能保证策略一致。
**待实测**：实际输出/零输出比例、是否解释用户当次起动失败。
**修正**：普通性能约束统一交给控制周期；guard只对真正危险实施优先级接管并禁止随后普通输出。无进展按单次移动意图/恢复事件计数，不能按循环次数假装“失败三次”。

`Hold` 分支将零电流称为 dynamic braking，缺少相应闭环制动证据。0 A不等于速度0或位置不动，更不等于禁能。[S05,S06,V01]

### F02：低速 PI 自身可能需要数秒建立起动力矩

现有速度PI已有积分，公式为 `u = Kp*e + integral`。若机构暂时完全不动，参考瞬时取常值，忽略上游斜坡/滤波，起动需要电流 `Ibreak`，则：

`time = max(0, (Ibreak - Kp*v_ref)/(Ki*v_ref))`（正方向、Ki>0、上限足够）。

用**举例值** Ibreak=0.40 A：5°/s 时 P项约0.0873 A、积分约0.0524 A/s，需要约5.97s；10°/s约2.15s。0.40 A来自某姿态正负脉冲已能移动的仓库记录，**不是本机构已识别的最大静摩擦阈值**；这只是说明为何纯调速度误差会慢，不是实机延迟预测。[S02,S04,R01；计算见tools/control_math.py]

与 F01 的1.5s窗口相比较，可以形成“尚未建立足够起动电流，guard便开始零输出”的合理根因假设。应先抓实际wire输出确认，而不是仅提高Kp或延长超时。

### F03：调度小延迟被放大为掉电

`VelocityLoop::step` 在dt>20ms、无效输入等条件将valid永久置false，直到显式reset。命令路径见invalid后会trip；`ControlLoop::step`看到backend watchdog fault会 `deenergize_all()`。[S04,S05,S08]

应区分：一次新鲜反馈仍可用的调度迟到、旧反馈真正超时、时间倒退/非有限值。迟到周期不继续用大dt猛积分，也不默认让两轴掉电；将状态重建、冻结积分、参考追平/受控减速纳入现有恢复路径。不得把100ms反馈失效时的持流当成闭环。

### F04：历史 CAN 计数永久污染健康判断

`buses_healthy()`要求两总线累计`rx_error_frames==0 && tx_failed==0`，一次历史错误可能令恢复后的总线仍被判失败。guard自身把部分counter问题当warning，命令路径却可能因此trip；单次send失败也可能立即trip。[S05]

改成当前link/CAN state、新鲜反馈、最近成功TX与**增量/时间窗错误**。不通过清掉原始计数掩盖故障。can1坏了不能在独立can0健康判断里伪装成yaw本轴丢控制；系统是否两轴受控停由上层统一决定，不能暗中释放健康pitch。

### F05：速度估计的时间口径和量化

yaw PI以调用时间差分最新q，不以独立RX样本时间计算；50ms低通虽压噪但带来滞后。另一条100ms滤波用于判稳/遥测，不应拿它的速度直接证明电机慢100ms。[S04,S08,S16]

GM编码器360/8192=0.043945°/count；在5ms差分中一个计数对应8.789°/s，rpm字段1rpm=6°/s。这解释了低速不能直接用raw rpm或单样本速度调高增益。使用同一unwrap坐标中的新鲜RX样本、有限滑动回归/差分窗，保留旧估计作A/B而非一次全换。[V01；纯计算]

### F06：参考本身较慢，且追赶没有速度余量

当前manual target acceleration15°/s²，jerk60°/s³；即使电机理想，也不可能瞬间达到20°/s。AUTO_TRACK target和maximum均20°/s、30°/s²，已经落后后再提高位置P，最终速度仍可能被钳在同一个20°/s。[S03]

SpeedServo还有位置误差±2°截断、Kp配置4、静止迟滞，以及a/j整形；混合backend又有60°/s²速度斜坡。应测每一层参考，去掉重复“规划斜坡”，保留最后一道硬能力限幅。不得一律取消jerk来制造“更快”的观感。[S05,S17]

### F07：pitch不是缺少电流闭环；诊断需要真实Iq

在RunMode=2，电调已闭速度环；主机不能再直接写IqRef当作“速度模式摩擦前馈”。需要的力矩由内部速度误差/积分产生。增大LimitCur只解除限幅，并不保证更早出力。[S07,S09]

采集：SpdRef、q/RX、raw velocity、选定速度估计、Iqf(0x701A)、LimitCur、RunMode、温度、VBUS(0x701C)。type2的torque是电机反馈估计，不是独立扭矩传感器；不要无标定地反推真实载荷。

### F08：归零的假接触与模式切换必须分段看

源码接触判据包含位置进展、速度、effort、持续时间及“曾运动或高effort”。旧注释中的0.4Nm/摩擦判断、速度噪声和增益背景来自旧机构，不能移植为当前交叉滚子机构的真值。`motion_checks_abort:false`与`mode_displacement_check:false`并不证明波动无害。[S03,S10,S18]

`jitter_suffix()`还会把jitter统一解释成“insufficient torque authority, raise limit_cur”。这是诊断过度推断，应该改为“检测到起伏，原因待区分”，记录是否实际到达5A限流。已是5A时不能让agent按错误提示继续加电流。[S10]

退让切position是现有有历史原因的设计，不盲删。模式切换目前有stop/disable窗口，悬臂载荷可能移动；正常服务/短暂停应保持speed mode。只有在相同模式退让通过新的重复性和接触卸载试验后，才考虑简化归零切换。[S07,S10]

### F09：安培/转矩/温度并未全部有可信工程单位

yaw snapshot把温度设为unknown、保留raw；temp raw guard当前为0，不能凭100ms反馈健康推断温度安全。电流命令单位已明确，反馈raw电流虽然脉冲报告与命令接近，仍须独立记录缩放版本/验证状态。未验证时用raw，不伪填Nm或°C。[S02,S05,R01]

GM手册原页列±3A命令、1.62A额定、0.90A连续堵转及0.741Nm/A名义常数；名义常数可用于初步预算，不能代替现场力矩/热资格。手册工况图与表的量纲/工况不应用单一比例强行“调和”。[V01]

### F10：payload旧档案不是此次校准

`conservative.yaml`包含两枚旧CyberGear UID和2026-09-02成绩；当前yaw是GM6020、pitch UID也不同。main照常load并通过默认commissioned=true标Ok；`motion_profile()`继续采用其中v/a/j cap。`auto_verify:false`只关自动检查，不关闭档案载入。[S13,S14,S08,S03]

旧gain字段在header中明确是CyberGear informational；daemon不把它们在线写电调。新agent不能把修改这些字段当作修改yaw实际Kp/Ki，也不能把旧的valid=true当混合机构合格。[S15]

PR1须记录实际加载/匹配状态；PR4须绑定实际硬件、模式和payload标识。与载荷不匹配时保留当前已批准的commissioning能力，不自动相信旧性能，也不因此故意掉电。

### F11：generic调参接口在yaw是no-op

Mixed backend的通用`set_current_limit(yaw)`、`set_speed_loop_gains(yaw)`不用于GM的当前PI；pitch对应的运行时gain setter为fire-and-forget，另有停机读回恢复路径。[S05,S07]

最小实施方案：本阶段采用**配置版本+受控会话**整定yaw参数，不开发未经需要的全功能热更新API；若为调试增加setter，必须typed、返回成功/拒绝、读回有效配置，不能silent no-op。pitch寄存器写后读回，调参过程中仍不能阻塞200Hz线程。

## C. 没有取得的证据

未取得固定commit SHA、实时运行版本/参数、现场CAN完整轨迹、实际内环频率、摩擦曲线、重心/惯量、温度缩放/热平衡、最大速度制动数据。仓库报告中的测试结果均属其作者报告，本包没有重复执行。源码检查能定位问题与实验次序，不能凭空给出最终硬件Kp/Ki。

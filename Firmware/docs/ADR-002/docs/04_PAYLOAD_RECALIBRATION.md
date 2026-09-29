# 04 · 载荷变化后的重校准指令

## 1. 什么变化触发什么校准

| 变化 | 需要重新确认 | 不应自动做的事 |
|---|---|---|
| 加/减相机、镜头、配重、线束或改重心 | 两轴起动/运行摩擦、惯量/重力影响、速度/位置响应、热与制动 | 清除机械零点、伪造旧成绩仍valid |
| 轴承、预紧、连接件、传动安装变化 | 上述全部，另加同轴度/摩擦随角度、机械几何/限位校验 | 只按重量比例缩放PID |
| 电机固件/模式/编码器关系变化 | 协议、单位、限流与动态校准、停止语义；必要时重新归零 | 直接复用不同模式的gain |
| 仅软件滤波、环路gain变化 | 响应/稳定性/停止/热任务覆盖 | 用“载荷没变”跳过回归 |
| 仅业务UI/视觉改动、机构未变 | 只做涉及控制时序/负载的回归 | 无理由重新撞端点归零 |

Yaw承载整个pitch组件，pitch payload变化也可能改变yaw惯量；不能只校准pitch。重心变化即使总质量不变仍可能显著改变pitch静态支撑。

## 2. 最小档案方案

复用`payload::PayloadProfileStore`原子保存和`.prev`机制。建议升级schema/扩展每轴typed control字段；实现时检查所有parser/serializer、启动载入及runtime选择。不要另建一个无人实际加载的“校准数据库”。

保留旧schema读取用于历史查看/受限fallback，不允许旧双CyberGear成绩自动qualify GM6020。现有`gain_kp/ki/kd`是旧CyberGear信息字段，不隐式重解释。

新档案至少包含：

- 身份：profile_id、schema、base_commit、config revision、payload标识、结构revision、质量/重心（未知可null）、操作者批准范围。
- 设备：yaw型号/ID/模式/驱动固件记录、pitch UID/固件/RunMode、当前单位与缩放验证；GM没有可读UID就明确null，并用安装asset ID+人工确认，不能编一个UID。
- 控制：yaw host速度Kp/Ki、估计窗/算法、正反起动和运行补偿、持续/起动电流资格；pitch已读回SpdKp/SpdKi与5A边界；两轴positionP及v/a/j能力分别保存。
- 几何与坐标：pitch有效端点/校准ID、yaw session零来源、相对角/物理角映射是否可跨会话。动态档案与零点分版本。
- 证据：每个试验的配置hash、时间、原始trace路径/hash、指标、温度/电源、PASS/FAIL/NOT_RUN，以及适用温度/姿态/速度范围。

[`payload_record.example.json`](../templates/payload_record.example.json)是**证据记录草案**，不是当前YAML配置。未知值null，status=NOT_RUN；不能通过改`qualified=true`绕过测量。

## 3. 默认与实际生效参数

硬件身份和模式在mixed_hardware；运动规则在turret_mixed；测得的动态参数/资格写typed payload。PR1先打印“数值+来源”，PR4明确解析优先级：安全硬边界永不被payload抬高；已匹配合格payload可提供动态参数和能力边界；未匹配则使用明确的commissioning profile，状态unqualified，不套旧gain。

配置中同一量只保留一个实际权威源。不要把旧yaml和新payload都写上Kp而不说明谁胜出。schema升级时可迁移当前0.8/Kp1/Ki0.6作为**未整定默认**，不能顺手标成qualified。

## 4. Agent运行顺序

1. 读取AGENTS、STATION_OPERATIONS、本ADR和当前有效profile。核对硬件/固件/模式/结构及payload变化，保留旧档案，不覆盖。
2. 判断几何校准是否仍有效：更换mass但编码器/端挡未变，不主动清零；混合profile当前启动保留校准机制尚有限，必须沿现有启动规则执行，不擅自绕过mandatory homing。
3. 使用操作者批准的单轴commissioning会话，禁自动巡航/视觉驱动，只由controld发CAN；另一轴主动保持。没有运动授权时仅准备离线代码/报告。
4. 执行调试协议：静止噪声/时序 → yaw正反breakaway与匀速 → pitch多姿态原生速度 → 位置/反向/归零（必要时）→ 制动/保持 → 热态/双轴。
5. 得到方向性摩擦与响应数据后整定，而非按重量比例盲乘Kp/Ki。只在已批准电流内测试；需要更高边界时列明证据和申请范围，不自动改硬限制。
6. 生成新profile候选及对比报告，机器检查缺项、身份/模式匹配、单位和有效范围；所有required test PASS才提出晋升。未知没有默认0。
7. 经操作者批准，原子切换到新档案，读回实际应用参数，复跑简短正反/停止冒烟；失败自动回到上一个已批准档案，记录原因。热资格不够则仅允许已验证范围，不能声称完整最大性能。

### 方向与重力的简单分离（仅作为辨识近似）

同一个pitch角度、相近低速的正反运动，记录带符号电流。若正反摩擦近似对称，可用`(i_positive+i_negative)/2`估计重力/固定偏置电流，`(i_positive-i_negative)/2`估计摩擦电流；不对称时不能照此强行分解，应保留两方向独立数据。多个角度可拟合简单sin/cos重力项，但在原生speed mode中这些结果先用于能力/热资格与诊断，不通过偷偷写IqRef注入。Yaw加速度前馈也只有在得到有效电流/加速度数据后才拟合，默认不启用。

## 5. 可直接复制给日后的 agent

> 按 `Firmware/docs/ADR-002/docs/04_PAYLOAD_RECALIBRATION.md` 对当前payload重新校准yaw和pitch。先从实际工作区记录HEAD、有效配置、实际电调模式、pitch UID/固件和机构/payload变化。只使用现有launcher与controld单一CAN控制路径，不运行并行CAN脚本、不改电机电流内环、不默认切MIT、不启用视觉自动运动。保留旧profile和几何校准；只在确有几何/参考失效时重新归零，不以重新清零代替动力学校准。
>
> 测两轴正反起动电流/延迟、低速运行、反向、位置响应、保持与制动；pitch在多个负载力矩姿态测试，yaw随pitch姿态覆盖。用实际温度、电源与代表性工作循环验证允许电流，优化稳定性而非最小电流。普通可恢复问题继续闭环或受控限制，不无故HOLD/FAULT/释放承重轴；真实控制丢失或危险仍及时处理。
>
> 一次只改变一个参数组，候选范围有界，记录实际寄存器/有效参数与trace。优先调yaw host电流输出速度PI和摩擦补偿、pitch驱动内置速度PI；不要把通用yaw no-op setter或旧payload gain字段误当实际生效。生成新typed profile、旧新指标对比、未完成项和回滚记录。未经已明确的现场运动/部署批准，只完成离线准备，不自行`--activate`。不能因测试只跑过合成样例就声明校准成功。

## 6. 档案何时不再有效

控制模式/单位变化、电机身份变化、机械预紧/传动改变、payload不匹配、温度/姿态超出测过范围，均使对应动力学资格失效。失效不必让仍能安全控制的电机掉电：保持承重，退回明确的受限commissioning能力并请求重校准。高频性能偶发退化应记录并复验，不每次颤动都执行全套校准。

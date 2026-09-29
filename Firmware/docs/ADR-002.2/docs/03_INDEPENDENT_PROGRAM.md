# 03 · 独立程序与生产共用核心

以下名称是**拟新增的逻辑目标**，不是声称仓库已有同名API。落地时可遵循现有目录风格，但接口语义与职责不得变化。

## 进程与模块

```
工作站 calibrate.py：模型拟合、信号设计、离线仿真、参数计算、证据检查
                         │一次监督会话/批量命令，不逐组LLM决策
                         ▼
Pi commissiond：独立采集/辨识信号/3a执行、日志与保护
                         │
                 共享 axis_control_core
                         │
                 共享 typed motor/sensor I/O
                         │
                        电机

Pi controld：真实production输入/reference/modes/调度 → 同一core/I/O → 电机
```

两个实时程序不能同时持有电机输出。分析程序可与只读生产监测并存，不能在生产正常业务运动中无预告注入辨识信号。

### shared axis_control_core

输入：统一时钟样本、编码器/gyro观测、q/v/a参考、已验证参数快照、输出包络、控制dt。
输出：请求电流、限幅后电流、完整状态/诊断。核心包括观察器、前馈、速度PI、位置P、起动逻辑、抗饱和与无扰切换；无文件I/O、网络、业务模式或隐式配置刷新。

bench与production使用相同ARM64构建目标、编译定义和core依赖hash。不同可执行文件hash不同正常；core API/编译flags不同不能只靠源码hash声称一致。

### commissiond

辨识段绕过测试轴上层位置/速度闭环，只保留内置电流环及必要的测试保护。执行器实际输出、包络干预全部记录。保护中断或电流限幅的数据保留，可用于符合条件的实际输入分析，但不能标为原始未限幅响应。

pitch重力导致开环不能在批准范围内保持时，使用**明确记录的稳定承重反馈＋外生辨识输入**；数据标为closed_loop_identification，交输出误差求解。此分支是合同规定，不是agent临场改成production调参。支撑夹具改变动力学时不得把夹持状态模型用于自由运动。

3a进入共享core闭环，与辨识模式分开。另一轴也由当前唯一owner负责支撑，不能留下production半个控制器同时发CAN。

### production适配

production只调用共享核心，移除同一运动轴的旧重复PI/补偿/guard输出。保留正常reference/业务模式/保护链、配置加载、归零和停止接口；每一级发布实际参考与限制原因。禁止通过让台架程序藏在production进程里而绕过业务链来满足3b。

## 输出所有权与程序交接

launcher获得站台互斥、取消旧owner会话、要求旧进程确认关闭电机输出，再授予新owner epoch。所有输出路径校验当前owner/epoch；遗留排队命令不可跨epoch执行。OS进程互斥与应用token联合验证；flock不是对恶意root进程的完全防篡改保证。

pitch会在停止/换模式时失去支撑的事实不能由文档消除。现有夹具/机械支撑与已验证交接步骤是启动当前模式所需条件；没有安全交接能力就BLOCKED，不允许只加一条日志继续。不能把0A等同刹车，也不能用GM没有的disable反馈确认停止。

退出、新owner建立失败、分析进程失联、用户stop必须沿经过验证的当前模式停车策略执行，保护不能依赖工作站SSH仍连着。新的独立程序必须纳入launcher监督，禁止随手kill全部进程或无人知情常驻。

## 控制模式与底层验证

yaw保留current命令；pitch新增verified current能力，复用项目低层编码。进入mode前停稳/支撑，验证mode/Iq引用/反馈/超时行为。电流帽是命令边界，实际电流监督是另一项；内部保护、软件cap与实际峰值不能混同。

正常服务不同时运行原生速度环和host速度PI。归零专用旧mode仍可使用，但进入/退出需验证中性参考、积分/负载支撑、encoder映射及control owner；否则3b失败。

## 运行时协议

逻辑操作：`describe → snapshot → prepare_profile → apply_profile → verify_profile → run_case → finish_case`。

每个profile包括完整参数、模型/观察器/标定/包络身份。core tick边界提交主机数值；电调需要写入的模式/字段全量读回后才提交revision。错误不能吞掉；禁止把成功TX当成功配置。

`run_case`要求expected_effective_hash、owner_epoch、case_id、采集ready回执、冻结的test_spec_hash。任意不符RUN次数为0。

算法值更新只生成JSON并热加载。PID/表/滤波/估计噪声/状态阈值可枚举；保护范围、实际硬件能力是只读约束。修改算法时新core hash，必须重新验证，不允许用“热参数”包装新代码行为。

## 测试控制与性能

200Hz控制循环不得等待文件或Python；数据进无阻塞有界环并异步写盘，溢出记为数据无效。离线求解默认工作站运行；Pi只执行有界实时case和必要在线残差统计。一次上传case集合和方法身份，批量完成；不是每次SSH读文本再决定下一步。

正常处理可恢复性能问题，不升级全轴FAULT。若保护因真实危险触发，其动作和相应axis停止证据必须保留，不能关闭保护继续优化。

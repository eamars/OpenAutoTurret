# 06 · 实施结合点、验收与agent权限

阶段1/2前置条件和完成标准由[架构师覆盖指令](07_STAGE1_OFFLINE_OVERRIDE.md)替代。
本文件实机能力与现场输入约束适用于阶段2及后续，不阻断完整离线数学实现。

## 1. 对既有实现的最小复用

公开读取的关键路径：[R2] mixed_hardware.yaml、[R3] can_motor_backend.cpp、[R5] mixed_can_motor_backend.hpp、[R6] gm6020_velocity.hpp。它们只是定位入口；本包未取得本地bad742d或dirty PR3的源码。

从既有轴I/O、统一launcher、trace、profile存储提取可复用部件。把共享控制计算抽成库，production与commissiond链接。保留已验证的停止/输出仲裁修复，先核对报告中的park_power_probe真实状态。不得复制整个controld改名commissiond，也不得重写所有CAN协议。

既有yaw速度增益名称单位含糊时提供兼容映射；正式schema写A·s/rad及A/rad。所有数值runtime，不靠再次改header试参数。

## 2. 必需产物清单

- 可构建的独立commissiond、共享axis_control_core、真实production adapter；x86通用测试、ARM64交叉构建。
- 单一calibrate入口：读取资产→探测适用性→定向采集→模型拟合→离线控制器计算→3a→3b→证据检查→晋升/明确拒绝。
- 完整encoder/IMU/电流时间与坐标标定、数据质量检查；可重复的信号选择和变化检测。
- ModelSpec、PlantFamily/Snapshot、runtime registry、controller profile、复用判定、双环境证书与schema。
- requirements.json每一条的真实代码/测试/实机证据映射；不得只交一张全PASS表。

无上面的任意必需功能就不是完整实现。参考Python模块的功能边界另列，不能用它替代真实adapter或生产链。

## 3. 失败原因与唯一分支

| 原因 | 规定行为 |
|---|---|
| DATA_INVALID：覆盖/时钟/字段/回执错误 | 同case最多重试1次；再失败停止，修采集不换PID |
| MEASUREMENT_LIMITED：噪声/采样不足 | 输出具体不能解析的频段/指标，修测量；禁止放宽质量线 |
| INSUFFICIENT_EXCITATION | 信息量选择器补测，最多2轮×3case，失败输出缺失方向/参数 |
| OPERATING_POINT_CHANGED | 新O分段，用已有资产更新模型；原过渡记录保留 |
| MODEL_INADEQUATE | 指出留出残差/共振/耦合超出模型；停止晋升，不让agent临时加入新模型结构 |
| ENVELOPE_LIMITED | 给出需求与批准电流/速度/热/停止约束冲突；不扩边界、不以慢到不动通过 |
| INTEGRATION_MISMATCH | 比较core/observer/reference/限幅/时序/有效参数；修生产接入，不能在生产另手调一份gain |
| HARD_ABORT | 既有保护接管并停止校准；没有自动重复动作 |

软件缺陷修复可能需要新构建，结束当前campaign、保留证据、生成新hash并重验受影响项。不是为参数取值而编译，也不是悄悄把改动带入活动实验。

## 4. 程序必须约束agent

锁定method/metrics/test-spec/包络/软件；不锁定待识别theta。agent无手动候选输入权限，production接收的参数候选必须来自完整计算报告及匹配hash。只允许操作员明确批准的新方法版本修改规则，agent不能改阈值再自签。

hash提供一致性而非完整安全授权。使用现有会话授权固定批准规则；同进程拥有全部root权限时不能宣称完全防篡改。所有动作必须能关联到真实operator/session授权。

主agent与子agent都不得在活动测试中集成PR/改共享树；可以只读分析已关闭日志，但其意见不能驱动下一组参数。统计与后续动作由确定算法生成。

## 5. 防止假完成的测试

必须自动复现：写入拒绝仍run、requested≠actual、旧证书配新参数、只3a就晋升、shadow冒充3b、不同核心/观察器、同标签不同载荷、没有timestamp的新鲜值、gyro坐标错、数据泄漏、编码器量化假抖动、几乎不动假平稳、zero-I被当保持、旧profile不适用仍回退、同一CAN多个owner。

必须以合成模型验证a/b辨识与控制器计算的已知真值，以录制数据回放验证C++和分析的单位/时间一致，以实机验证模式/电流/停止/3a/3b。三者不能互相代替。

## 6. 现场输入不是待决设计

必须由现场读取/操作员许可提供：确切commit和dirty diff、当前设备UID/firmware/模式、机械范围/许可、实际电流/温度/供电边界、IMU安装标定、可逆payload/摩擦作业、生产授权窗口。缺失输出BLOCKED，不虚构数值。

未知现场输入并不把算法选择交回agent：本包已经规定模型、拟合、选择器、控制计算、更新分支、指标、3a/3b和晋升。agent只补事实、实现和运行规定程序。

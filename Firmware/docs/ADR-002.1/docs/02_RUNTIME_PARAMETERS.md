# 02 · 运行时参数与应用契约

## 1. 最小实现

继续使用 controld 及现有 UDS。逻辑操作命名为 `describe / snapshot / prepare / apply / verify / run / finish`；这是**拟议契约**，不是声称当前存在同名 CLI。

每个参数条目包含：

`name, type, unit, default, actual_value, supported_range, allowed_test_range, mode, mutability, apply_condition, source_binding, readback_source, encoding, restart_required, reason`

`mutability` 只能为 `experiment_writable / fixed_in_campaign / protected_read_only / unsupported`。unsupported 的原因和已知替代观测必须可见。

## 2. 应覆盖的实际控制项

下表是一次源码核对的完整清单，不要求为了凑字段创造不存在的功能：

| 组 | registry 覆盖项 | 本轮搜索状态 |
|---|---|---|
| yaw 速度环 | Kp、Ki、实际已有积分限幅/抗饱和参数 | 只搜索 Kp/Ki；策略固定 |
| yaw 补偿 | 正负方向起动/运行幅度、启用、淡入淡出、已有 breakaway 状态/确认参数 | 只搜索 4 个幅度；其余固定 |
| 速度估计 | 已有算法、RX 窗、滤波时间、死区、确认窗口 | 暴露但本轮不搜索；避免再次混杂 |
| 输出 | 已有输出 slew、参考限速/加速度/jerk、位置外环 P、前馈 | 暴露但本轮固定；不加不存在的 D 环 |
| pitch | 原生速度 Kp/Ki、已有反馈请求周期、位置外环/服务与归零参数 | 只搜索速度 Kp/Ki；模式不变 |
| 实验 | 每段时长、重复数、轨迹定义、质量阈值、样本有效性阈值 | manifest 冻结，不编进 C++ |
| 限制与状态 | 真正生效电流帽、设备保护、端点、租约、控制周期、总线身份 | 可读；优化器不可扩大/旁路 |
| 电调不支持项 | 固件内部参数或未实现的可写寄存器 | 明确 unsupported，禁止 no-op 成功 |

需要“等静止再改”不等于需要业务 HOLD。以 runner WAIT_AT_REST 表示实验暂停，让原控制器保持负载；不要在参数交换时盲目清积分或撤能。

## 3. 原子提交的准确含义

1. 读取完整 runtime snapshot，得到 revision R、硬件模式和参数编码定义。
2. prepare 一次提交完整参数集，不允许只应用其中一部分并继续。
3. 验证类型、单位、有限数、支持范围、批准试验范围和模式。bool 不得当数值接受。
4. 将候选值按实际表示精度规范化，计算 expected_effective_hash。hash 不能使用模糊 float 字符串或未量化请求值。
5. 等待已有的可安全交换状态；到期未到达则中止，不临时降低静止阈值。
6. 主机参数在控制 tick 边界切换。电调寄存器按既有协议写入并**全部读回**；期间不能宣称外部已经提交成功。
7. 完整验证后发布 revision R+1，包含实际值和 applied tick。写入不一致则恢复 R 并验证；恢复失败中止并使用已有保护/停止链。
8. RUN 只接受期望 revision/hash，与 snapshot 和回执同时相符。重复 apply 使用幂等 request_id，不产生第二次积分状态转换。

电调寄存器读回延迟与编码器反馈延迟分别记录；TX 成功不是参数已生效。软件覆盖值不得在下一 tick 被配置刷新线程写回默认值；trace 必须能验证全程一致。

## 4. 消除补偿开关混杂

enabled/disabled 不得暗中改变积分重置、输出 slew 或运动意图处理。固定一个正常控制路径；“补偿为零”与“补偿非零”的唯一区别应是该项数值。保留必要的无扰状态交换，但其规则对全部候选相同。

先用相同虚拟反馈输入做零幅度路径等价单测，再做固定基线。硬件试验不得把多项语义改变称为单变量摩擦实验。

## 5. Trace 必需字段

至少包括：campaign/trial/candidate ID、source/binary/metrics hash、requested/effective 参数 hash、revision、控制 tick、RX 原始时间/序号、去回绕编码器位置、在线估速、原始速度、最终内环参考 `be_cmd`、上游 `vref`、PI 误差/积分、补偿分量、限幅前输出、最终输出、限幅原因、实际电流、设备状态、主机时间、phase、非测试轴位置、有效温度/单位/有效性。

IMU 在 runner 开始前建立连续归档，记录 generation/tare/时钟。默认仅辅助证据；安装未标定时不用 sensor X 替代机构 yaw，不让它悄悄成为主要 jitter 判据。缺少辅助 IMU 要显式报告，不能假装有关联证据。

每份原始流从 trial 前基线覆盖到结束后保持，不用最后 11 秒的滚动文件代替先前所有动作。各流哈希与时间范围自动写入结果。

## 6. 覆盖验收

一次性核对现有 config、controller 和 trial setter，把真实可调字段与 registry 对齐；测试断言新增可调字段必须有条目/原因。100% 是**真实可调项覆盖率**，不是保证厂商所有内部寄存器都能写。

验收过程中记录二进制 SHA：改变每个 experiment_writable 参数后读回、再恢复，整个过程二进制不变、无编译/部署动作。只读保护写请求必须被服务端拒绝，不能仅靠 agent 自律。

## 7. 哈希不是授权

SHA-256 只检验一致性。不得把“请求自己带来的 plan_hash”直接当批准依据。controller 会话在独立的既有授权步骤固定 approved_manifest_hash，随后 RUN/APPLY 都与该值比较；活动会话中不能重新批准、修改 writable 分类或扩大试验域。结束会话也不构成自动批准下一会话。

runner 对批准 manifest 只读。agent 可修改开发树，但一旦源/二进制/度量与批准快照不同，原 campaign 就不能继续；不能先改 manifest 再计算新 hash 来规避冻结。对拥有整套主机写权限的恶意进程，单靠这份文档和哈希不能提供完整安全隔离，不能宣称绝对防篡改。

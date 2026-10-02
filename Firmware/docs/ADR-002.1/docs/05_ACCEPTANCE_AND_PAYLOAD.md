# 05 · 最小交付、行为验收与换载荷流程

## 1. 两个实现切片，不并行调试多个 PR

A：保存工作区、查明停止回归、registry/事务/trace。B：冻结 planner + 本地 runner + scorer + profile 候选输出。活动 campaign 期间禁止改源码。PR3/PR4 与本轮无关的剩余功能留在独立工作区/后续切片，不在实验进行时集成。

本轮不新建 UI、数据库、独立守护服务、第二条 CAN 输出链、复杂辨识模型或完整参数平台。

## 2. 必须由机器验证的行为

| 故障/操作 | 必须结果 |
|---|---|
| 参数请求被拒绝 | 后续 RUN 次数=0；campaign 中止 |
| 请求 Kp=2、实际 Kp=1 | hash/revision/value 校验失败；不运行；文件夹名称无效 |
| 一部分 pitch 寄存器写成、另一部分失败 | 无提交回执；恢复全量旧值并读回；不执行测试 |
| 试验中参数版本变化 | 本组无效并终止 campaign |
| 主 agent 要追加一个临时候选 | 不在 lock 内，runner 拒绝 |
| 主 agent 要放宽阈值/换评分脚本 | lock/hash 不符，runner 拒绝 |
| 参数只改数值 | 二进制 SHA 不变；编译/部署次数=0 |
| 静止或缓慢爬行、jitter 很低 | FAIL_QUALITY，不得晋级 |
| 日志末尾覆盖不到运动 | INVALID_DATA，不生成漂亮但虚假的统计 |
| 一次采集无效后再无效 | 只重试同一个 case 一次；随后中止 |
| 安全只读参数写请求 | 服务端拒绝，不是仅警告 agent |
| 普通性能不合格 | 受控结束本组；不是无差别撤掉承重轴扭矩 |
| 活动实验中源码改变/重编译 | campaign 被锁阻止或结束；不得混用结果 |
| 种子、锁文件、输入相同 | 候选与机器决定完全可复现 |

## 3. 分清三种“完成”

**基础设施完成**：真实固件上证明参数热更新、事务阻断、冻结规则、完整采集与自动运行；停止/故障回归通过。

**本次范围整定完成**：冻结的工况矩阵通过，报告支持哪个方向/姿态/速度/载荷。未测范围不得默认为合格。

**生产与热资格完成**：另需原 ADR-002 的必要位置、归零、停止、双轴和热工况证据与既有批准流程。

不能用本包 Python 单测、旧 release 81/81 或“jog 没 fault”替代上述任何实机结论。

## 4. 未来 payload 变化的固定操作

1. 比较 payload/机构/电机模式/传感器/标定身份。质量、重心、配重、线束、轴承预紧变化均使相应动态资格失效；不无故擦除仍有效的几何零点。
2. 自动复制**方法版本**（registry、planner、metrics、阈值和停止规则），不复制“已合格”标记。
3. 从真实 runtime snapshot 和批准域按 docs/03 公式生成并冻结新 campaign。两轴分别运行同一流程；不按重量比例乘 PID，不让 agent 手填摩擦。
4. 若旧配置在新载荷上不具备试验资格，只能做已批准的测量/预检；不能把旧资格自动继承。域不足则输出 BLOCKED/NO_FEASIBLE_CANDIDATE，不擅自扩大电流或增益域。
5. 保存 candidate profile，绑定 payload、机构、模式、几何和 source/binary/metrics hash，并记录通过/未测范围。生产晋升使用现有授权和原子 profile 存储；未授权仅输出候选。

## 5. 最终交接必须出现的证据

`runtime_parameter_inventory.json`、`campaign.lock.json`、`trial_results.jsonl`、完整原始 trace 索引/哈希、`selection.json`、`qualification_scope.json`、`code_and_test_status.json`。

selection 可为 NO_FEASIBLE_CANDIDATE，这是诚实的固定算法结果；不允许在最后一次循环外由 agent 私自“选一组看起来还行的”。

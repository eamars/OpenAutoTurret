# 09 · 故障树、降级与回退

## 1. 顶事件：发生非预期运动或错误跟随

| 分支 | 主要诱因 | 预防/检测 | 降级与残余限制 |
|---|---|---|---|
| 供电/主机 | 欠压、OOM、控制进程丢失 | 分级负载、健康事件、build与运动隔离 | 软件不能在掉电后保证中性输出；停止且禁止无人值守 |
| 命令所有权 | 第二controller、独立camera旁路、不同run目录绕锁 | 保持站台锁与worker所有权；launcher监督 | 拒绝启动第二owner |
| 几何 | 旧K、错crop、重复180°、未知baseline | calibration identity、round-trip与动态验证 | 相机仅shadow |
| 时间 | 用publish重盖capture、跨clock、过期frame | 原始时间、epoch、age与seq | 拒绝帧；既有loss流程 |
| 身份 | UUID碰撞、显式选人被auto替换、重复tag | global membership、selection latch/generation、conflict | ambiguous/重选，不猜 |
| 控制 | 双积分、饱和、模式跳变、预测重复 | per-axis辨识、anti-windup、限幅/残差 | 单轴功能回退、降低授权包络而非抬高硬限 |
| 归零 | 静摩擦误触端、噪声误速度、未abort | phase数据、hard/shadow分离、重复性 | 未就绪，不重试到“看似成功”为止 |
| 关停 | readiness拒绝stop、park故障、禁能语义错误 | stop幂等锁存、正常park/fault stop分开、分轴证据 | stop_incomplete，现场处置；不伪造断能 |
| IMU | stale tare、安装不明、重启复用旧状态 | generation/tare/time校验；先shadow | observe-only，编码器仍是主参考 |

## 2. 降级优先级

模型/预览过载：关第二预览→关face ROI→降低detail处理采样→停detail，保持wide与control独立。身份不确定：不只是降画质，必须撤销该目标观测的可用性。power/control/CAN硬故障：不是普通性能降级，按控制器stop/fault流程处理。

## 3. 回退规则

- 视觉增强失败，可回wide-only，但必须保留已选目标策略与准确的source/age语义。
- 协议变化必须回退配套的producer/consumer；不得只退visiond让旧包被新parser误读。
- 标定失效不回退到“能打开的旧文件”；应停止对应自动视觉功能。
- 电机回退只选支持GM6020 yaw + CyberGear pitch的mixed release；不能启动旧双CyberGear版本。
- 即使回到曾运行的8901808，也不能把欠压/stop/homing未关闭的问题视为已解决；回退不是自动运动许可。
- 未有合格回退版本时保持停止，保留故障证据和工作区，不删除日志恢复“干净状态”。

## 4. 发布最小证据

revision/config/model/calibration/install IDs；实际运行测试清单与结果；resolved limits/capabilities；供电与停止报告；已启用feature阶段；未运行故障测试；stop输出能力限制；明确回退目标。

**不得声称本包证明：** 完整yaw整圈安全、最高速度制动、GM6020断能、pitch无机械下落、跨会话人脸身份、IMU绝对航向或所有长时间故障恢复。

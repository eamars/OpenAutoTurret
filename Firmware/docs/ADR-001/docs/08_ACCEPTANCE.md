# 08 · 可复现测试与量化门槛

## 1. 三类数值不可混用

**当前配置约束**：pitch 5A上限、feedback 100ms等来自现有配置/交接书。**本包工程目标**：下面的性能和质量数值，待首批本机数据校核。**实际测量**：本包没有新增硬件测量，结果字段应为null/NOT_RUN。

门槛只授权对应功能/条件；单摄通过不授权双摄，camera-only不授权运动，三次成功stop不证明故障停止或断能。

## 2. 验收矩阵

| ID / 级别 | 方法与最小样本 | 初始目标 | 失败处置 |
|---|---|---|---|
| A0 离线 | resolved profile/axis/mode/unit测试 | 每个最终参数可追溯；无混淆单位/默认profile | 不变更运动行为 |
| A1 离线 | timestamp/epoch/seq/calibration/selection反例 | stale、future、旧generation、未知geometry均被拒绝或shadow | 不升级native授权 |
| A2 离线 | Hailo pad/resize/crop/orientation round-trip | 合成像素逆变换误差≤1e-6px；空框/错shape拒绝 | 禁该profile几何 |
| A3 离线 | mock worker退出/卡死、backpressure、重启 | wide独立持续，队列有界，无旧owner并发 | 保持wide-only |
| P0 无运动 | idle5min/每单摄10min/双摄Hailo30min | 当前throttle低4位=0；无新增历史故障；无OOM/重启/CAN错误 | 停止/保留证据 |
| T0 无运动/受控 | 每profile≥1000完整trace | p95曝光参考→controller≤80ms，p99≤120ms；publish→RX p95≤5ms | 先减非关键负载 |
| T1 受控 | 同时记录200Hz循环与CAN反馈 | 计算耗时p99<2ms、周期/lateness另报；yaw反馈age p99≤10ms、pitch≤40ms | 分析，不抬高100ms硬门槛 |
| T2 无运动 | 各camera持续30min | wide有效更新≥20Hz且保留约26Hz目标；detail≥14Hz的15Hz baseline；无持续积压 | detail回退 |
| C0 标定 | 25–40训练姿态+≥10独立保留姿态/mode | RMS≤0.7px；保留集p95≤1.5px | 只显示/记录 |
| C1 标定/受控 | 多距离/方向/姿态的跨镜留出集 | LOS映射残差p95≤0.5°，报告视差与时间误差 | 不交接detail |
| M0 受控 | pitch归零10次，原始phase数据齐全 | 无false-contact/硬越界忽略；repeatability≤0.5°；慢速段p95速度误差≤max(1°/s,20%command) | homing未就绪 |
| M1 受控 | 每轴每方向≥10有界step（含不同幅度） | 超调≤max(0.5°,5%step)；轨迹结束1s内稳定±0.5°/500ms | 单轴回退，重辨识 |
| M2 受控 | 已资格化低/中速度移动停止，各方向/姿态覆盖 | 100%留在已验证停止包络，无虚假disable/断能确认 | 该速度/载荷不授权 |
| S0 受控 | 30次正常moving stop覆盖track/roam/manual；另列fault cases | 30/30完成其定义的轴证据；无readiness拒绝停止请求 | 保持stop未资格化 |
| V0 有标签 | ≥300 person实例、≥100空景负例；独立保留集 | precision≥0.95、recall≥0.90（IoU≥0.5），逐关键分箱 | 该profile只shadow |
| V1 双摄 | ≥100个独立交接episode | 错误目标替换=0观察事件；交接成功≥95%；双方证据就绪后交接p95≤300ms | wide-only/ambiguous |
| I0 标记 | ≥100遮挡/交叉/复用episode，单独冲突集 | 错误替换=0观察事件；有效码重获p95≤1s；冲突100%拒绝确定身份 | 暂停/重选 |
| F0 可选脸 | 正侧背/重叠/低照有标签分箱 | 可见脸precision≥0.98、recall≥0.90；中心p95≤脸框宽10%；无邻人错挂观察事件 | body锚点 |
| U0 UI | 录制可控frame序列/双摄/重启测试 | overlay frame_id一致；来源/年龄/unknown状态无伪造 | 不展示错误叠加 |
| IM0 shadow | 多姿态/运动/重启/gap记录 | mounting/time验证后再报残差；reset使旧tare失效；融合候选不能让p95误差变差>10% | IMU observe-only |
| R0 恢复 | 3次受控重启+camera/IMU软件故障注入 | 双CAN正确UP/1Mbps；不复用旧epoch；不自动绕过资格 | 停止，非无限重启 |

P0的history-only既不能当当前故障，也不能证明窗口内没有再次短暂故障；bits已sticky时测试报告必须说明观测能力。主机soft monitor不是硬件断能安全装置。

T0按所有收到的完整样本统计，不仅统计“被控制器接受的快帧”，否则掩盖慢帧拒绝。建议selected观测通用最大age初值150ms、detail接管更严100ms，均与motor feedback100ms分开；expired结果不更新选择/控制。控制coast是明确的预测/减速状态，不伪造新观测。

## 3. 边界案例必须覆盖

- 相机index交换但hardware ID不变；generation改变后旧frame迟到；同一frame重复发布。
- UTC/NTP变化不影响age；BOOTTIME映射失效使clock_epoch变化；禁止跨boot拼接统计。
- IMX500 rotate_180与IMX477 none，各模型letterbox/crop反变换和预览round-trip。
- cancel→select相同人产生新generation；explicit track丢失后sole candidate不同；两相机局部UUID碰巧相同。
- Hailo停止回传、全空检测、仅高分框、模型hash变更、JPEG消费者阻塞。
- 反馈陈旧、CAN1断开而CAN0有效、yaw零请求但无disable语义、park中反馈丢失、重复stop不能重置deadline。
- IMU reset/recovery、旧tare、时间缺口、pitch运动被误当base运动。

## 4. 样本量与统计诚实性

零错误的100次独立episode仅是工程入门证据，不意味着真实错误率为零；用近似rule-of-three时95%上界约3/N，但它假设独立同分布，连续帧或重复同一路径通常不满足。报告分母、分箱、失败、ambiguous与排除条件，不能只报成功次数。

p99必须同时报告n和缺失数；短60帧探针只能证明有限连通性/时延，不能证明尾时延或person/head质量。[U L81]

## 5. 基准运行清单

同一场景比较 baseline、dual_shadow、detail_authoritative、可选face开关、IMU shadow开关。固定camera mode/exposure策略/model SHA/config/load，记录环境光与距离变化。每次只改变一个主要因素；异常段保留而非从统计中静默删除。

包内 `tools/offline_checks.py summarize` 仅演示有事件缺失/跨clock epoch时如何避免算假延迟，不含完整accuracy/IDF1/角度评估器。Codex应把WP1和WP5真实日志适配器接入同类统计，不声称本包工具已解析仓库CSV。

## 6. 结果报告模板

使用 `examples/qualification_report_template.json`。每条结果包含 `status=NOT_RUN/PASS/FAIL/BLOCKED`、版本/模型/标定身份、样本数、actual metrics、目标、失败证据、excluded_cases、剩余限制。未知字段用null；schema合法本身绝不产生PASS。

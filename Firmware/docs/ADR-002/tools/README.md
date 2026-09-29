# 离线工具（不连接硬件）

Python 3.10+，只用标准库。它们是开发包自检/日志统计辅助，**不是固件补丁或硬件调参器**。

```bash
python -m unittest discover -s tests -v
python tools/control_math.py
python tools/trace_metrics.py fixtures/synthetic_trace.csv --output reports/synthetic_metrics.json
```

`control_math.py`给出：理想静止轴的PI电流积累示例、RX时间窗速度斜率、最终限幅后的反算抗积分饱和数学步骤。没有实现breakaway状态机、CAN发送或运行安全策略，不能直接塞进电机循环。

`trace_metrics.py`读取CSV，按每轴连续trial/phase片段统计时间加权RMS、采样间隔/反馈年龄分位数、限幅时间比例、动作标签变化和温度线性斜率。缺少数据返回null；单样本不能给时间加权RMS。负feedback age保留并计数，不偷偷改成0。跨generation RX时间倒退需先分文件。

## CSV最小列

必需：`t_ns,rx_ns,axis,trial_id,phase,drive_mode,q_ref_rad,v_ref_rad_s,q_rad,v_est_rad_s,clamped,guard_action`。

可选：`u_applied_a,iq_a,temperature_c,bus_voltage_v`。

时间为monotonic整数ns，同一轴控制时间严格递增；yaw/pitch可以同时间交错记录。rx_ns可重复，代表还没有新反馈。原始数据的frame seq、raw current、actual mode、estimator ID等保存在完整原生trace中；导出子集时不能改变单位或时间。若cycle_start先于刚收到的帧，工具会保留负age并告警计数，由分析者核对，不当作硬件时间倒退。

`phase`使用独立、连续的start/steady/hold/brake等标签；非连续同名片段不会被工具跨间隔合并。各轴速度统计分开；不要混合加速和steady段宣称匀速误差。

pitch speed/position mode的`u_applied_a`必须空白；它的主机命令是速度/位置，不是0A。`iq_a`只填已验证缩放的实测电流；未知raw值不要塞进去。温度同理，GM raw尚未定标时留空。

默认位移阈值为三个GM计数；分析pitch请用已测噪声阈值覆盖`--motion-threshold-rad`。返回的是第一次越过位移阈值时间，不是持续运动确认，也不是机械真实起动时刻。工具不自动算复杂轨迹的10–90%/settling或热平衡；这些仍需依据完整标注轨迹分析。

**200Hz控制日志不能证明总CAN反馈1kHz。** `observed_distinct_feedback_hz_NOT_total_CAN_hz`只代表日志看见的独立反馈样本。完整CAN频率应以传输层帧计数和同一时间窗测量。

`fixtures/synthetic_trace.csv`为合成数据；所有输出都不能用来qualify载荷。测试报告中的PASS仅指本包离线工具测试。

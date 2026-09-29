# ADR-002.1 · 自动实验与 agent 执行契约

日期：2026-09-30（Pacific/Auckland）。状态：**设计包；参考工具仅离线运行；尚未集成固件或完成实机整定。**

本包沿用 ADR-002 的**非武器化相机／传感器云台**范围。它不授权部署、启动运动或关闭设备保护。只规范可复现的实验、参数热更新、证据与 agent 权限；不新增电机控制模式。

## 本次裁决

1. 不接受“关闭所有安全阀门”。性能/生产资格判据与设备保护分离：前者可在隔离台架会话中只记录或淘汰候选；急停、设备电流/温度边界、反馈失效、物理越界和命令失联保护不得旁路。不得把软限位一概归为性能判据。
2. 所有**本次控制实现中可调的参数**进入统一运行时 registry。参数取值变化不得重编译；不支持写入的电调内部参数明确列为只读/不支持，不能伪造成功。
3. 禁止主 agent 或子 agent 逐轮挑参数。候选生成、排列、评分、精测、重试和结束由冻结的脚本决定；LLM 不进入“测一组—读文字—想下一组”的循环。
4. 固件开发与参数实验分时执行。活动 campaign 中源代码、二进制、度量实现、阈值、参数域均不可变。
5. 不保证在试验前知道最终物理参数。**固定的是得到答案的算法与规则，不是编造未测的增益和摩擦值。**

## 阅读顺序

- `00_CODEX_START.md`：可直接交给 Codex 的行为合同。
- `ADR-002.1.md`：正式决策、与 ADR-002 的覆盖关系。
- `docs/01_REPORT_REVIEW.md`：报告事实、旧方案缺口、证据限制。
- `docs/02_RUNTIME_PARAMETERS.md`：热更新、应用事务和数据契约。
- `docs/03_CAMPAIGN.md`：冻结的自动实验顺序与搜索规则。
- `docs/04_METRICS.md`：jitter、跟随、停止和数据有效性。
- `docs/05_ACCEPTANCE_AND_PAYLOAD.md`：交付门槛与换载荷后的复用。

`manifests/` 中数值是设计目标或离线示例，不是已批准的物理安全包络。`tools/adr0021.py` 提供标准库实现的离线候选生成、评分、回执阻断和演示；**没有 SSH、SocketCAN、UDS、串口或电机发送功能**。实机 adapter 仍由 Codex 在原有单一控制路径中实现。

## 已交付：参数清单

`manifests/parameter_inventory.json` 由 controld 自己生成（`controld <config> --dump-parameter-registry <path>`，经
`Firmware/tools/dump_parameter_inventory.py` 补上二进制 SHA-256 与源码 revision），**不是手抄表**：31 条条目 + 18 条排除规则。
## 已交付：事务式参数应用（prepare→apply→readback→revision）

controld 多了一组命令，**由控制环持有门**：`param_prepare <kp:ki:rx:bp:bn:rp:rn:slew>` 只暂存一个候选并回 `request_id` 与 `expected_hash`（不落硬件）；
`param_apply <request_id>` 写入后**读回**，读回与暂存值一致才推进 revision；`param_restore` 是 `restoring` 状态的出口；`param_snapshot` 说明当前哪个 revision 是已验证的。
**只要交换没走完，运动命令一律被拒**（`response_probe` / `manual_jog_*` / `manual_step` / `run_test_motion` / `start_tracking` / `start_payload_verification`），
拒绝理由带状态与最后一次已验证的 revision+hash——这就是「kp2 目录里其实是 Kp=1」那一类编排错误的机械化拦截。
`yaw_control_trial` 现在走同一条实现（旧脚本不改也能用）。驱动**当面拒绝**、或**答应却存了别的值**（静默截断），都会被抓住：
`control/tests/test_control_loop.cpp` 的 `ARefusedApplyIsNotARevisionAndASilentClampHoldsMotion` 用一台会撒谎的模拟电机复现整件事。
两条轴现在共用同一个 revision 与同一扇门，但**读回的来源不同，清单里也这么写**：yaw 是 `host_echo`（GM6020 没有增益寄存器），
pitch 是**寄存器读回**、异步完成——`pitch_control_trial` 先读当前寄存器（读不到就拒绝开始一次无法验证的交换），写入后由控制环的 poll 收尾；
**交换未完成跨 step 边界期间运动命令同样被拒**，ack 里带 `register_readback_ms=`（驱动器回答寄存器读用了多久，与反馈延迟是两件事）。
`param_restore` 按事务里持有的**参数名**决定该恢复哪条轴——靠一个 flag 会把恢复送到错误的植物上。

`Firmware/tools/tests/test_parameter_inventory.py` 会拒绝任何"新出现的可调字段既没有条目也没有排除原因"，
也会拒绝与当前二进制不一致的旧文档。清单里 `yaw.friction.enabled` 被标成 `fixed_in_campaign`，理由写在条目本身：
现在 trial 命令用"任一幅度 > 0"反推这个开关，所以开/关不是单变量对照（§4）。

## 已交付：§6 验收在真站上通过（19/19）

`reports/runtime_snapshot_2026-09-29.json`：对 **9 条 experiment_writable** 逐条改动→读回→恢复（含基线共 19 次交换），
**全部 applied-and-read-back**；revision 每次交换推进一次；结束时 `state=idle` 且 `applied_hash == expected_hash`。
**交换集内二进制 SHA 完全不变**——零编译、零部署、零重启，这就是第一片存在的全部理由。
只读保护是**服务端形态**的拒绝：`yaw.host_current_limit_a` 战役前后都 0.8 A，服务端的清单自报 `protected_read_only`（写路径不存在）。
温度是**如实缺**而不是编的：live 帧里没有温度键（列过键），`temp_raw`+单位+有效性在 trace 记录里，所以快照写的是来源而不是借个数字凑齐。
两处**待主人定**：① `manual_commissioning` 只能由 launcher 环境变量武装 ⇒ 正常栈里调参面不可达；② boot `slew=0` 而 prepare 只收 `(0,10]` ⇒ 出厂默认值无法原样 apply 回去。

## 进行中：冻结的 planner（campaign.lock.json）

`Firmware/tools/adr0021_plan.py` 把设计**一次算死**写进 `campaign.lock.json`：维度名必须来自清单的 `experiment_writable`（保护字段被拒时连条款一起报，D7），
coarse 固定 16（要改必须写 `coarse_count_reason`）、refine ≤8 且必须带 gate、confirm ≥2、stop 必须写 max_trials 与 no_improvement_rounds、payload 必须绑定。
锁文件把 `design_sha256` + 清单 SHA-256 + 二进制 SHA + source_rev 绑在一起；`--check` 会拒绝事后被改过的设计与已经漂移的清单/二进制。
样板见 `manifests/campaign.example.json`（levels 是现网 boot 值的乘子）。**runner 跑起来不决定任何一轮**——D6。

## 离线验证

```bash
python -m unittest discover -s tests -v
python tools/adr0021.py demo --output reports/demo
```

`reports/offline_validation.json` 记录本包实际执行结果；它不证明站台测试、温度资格或固件 C++ 测试通过。

## 基线与查阅范围

主要依据用户粘贴的 2026-09-30 00:12 NZDT 执行报告及上一版 ADR-002 文件。报告给出的 `bad742dddba6a95d3d055e43998adfd0506b0a6e` 四个选择性公开文件请求均返回 404；本次**没有核实该提交的源码或本地脏树**。未克隆仓库，未访问 Pi 或用户本地 `run/`。公开 README/AGENTS 只作为文档入口，不替代本地当前状态。详见 `sources/READING_LOG.md`。

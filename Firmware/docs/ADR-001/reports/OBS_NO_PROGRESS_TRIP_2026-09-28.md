# 观察记录：AUTO_ROAM 换向点上的 `no_progress` 跳闸（2026-09-28）

状态：站点安全（跳闸后 `phase=fault`，yaw 零请求、pitch 保持）。
本条是**观察记录**，不是结案：两种解释都还活着，我把能区分它们的那一步写清楚了。

## 1. 事实（一次，可复核）

栈于本地 02:31:46 由 `run_application.sh start` 起来（正常启动＝AUTO_ROAM）。
**02:33:41.399 跳闸**，正好在起栈后 1 分 55 秒：

```
GM6020 guard trip: feedback_safe=true reference_valid=true received=true encoder_valid=true
feedback_age_ms=0.068 can_up=true can_state=0 rxerr=0 txfail=0 both_buses_healthy=true
measured_speed_deg_s=0.879 temp_raw=31 requested_speed_deg_s=10.000
no_progress_ms=1502 heartbeat_seen=true heartbeat_age_ms=2
→ control fault: independent motor watchdog trip: no_progress
```

跳闸**前**的 1 Hz 遥测（同一文件，紧挨着）：

```
02:33:39.566  phase=hold  q_yaw=-1.3346 rad
02:33:40.577  phase=hold  q_yaw=-1.3353 rad
```

关键三点：
- **`phase=hold` 却挂着 10 °/s 的 yaw 速度请求**；
- 那一秒 yaw 实际位移 **0.04°**（≈0.04 °/s），而矩阵里的 0.879 °/s 是衰减中的瞬时值；
- **`q_yaw = -1.3353 rad = -76.5°`，而软限位是 ±80°**——它当时停在扫掠扇区的边界附近。

`no_progress` 的定义：命令速度 ≥ `kNoProgressCommandRadS` 且编码器在窗口内没走够 3 个计数；
GM6020 编码器 8192 计数/圈 ⇒ 3 计数 ≈ **0.132°**。**它在 1.5 秒里连 0.132° 都没走到。**

## 2. 两种解释（我不在凌晨四点挑一个来改安全守卫）

| | 假设 | 支持它的证据 | 反对它的证据 |
|---|---|---|---|
| **A** | **陈旧命令**：进入 hold 时 `yaw_requested_velocity_rad_s_` 没被清掉，守卫把一次已经不作数的请求当成还在飞的命令 ⇒ **误跳** | 遥测明写 `phase=hold` 而 `requested_speed=10` | 若请求是残值，`progress_at` 也应该早就被"低速复位"分支刷新掉（`|req| < 阈值` 时刷新窗口）——而它没有，说明守卫看到的是**≥阈值**的请求 |
| **B** | **边界命令**：roam 在扇区换向点朝限位方向要到 10 °/s，而机构已经顶在包络边上动不了 ⇒ **守卫报的是真事**，但守卫的"期望"是错的：**它假定每个 ≥阈值的命令都能产生运动** | yaw 在 -76.5°（软限位 -80°）；换向点会有短暂驻留；1.5 s 窗口对"到边再反向"这种动作太紧 | 若真是顶到边，`reference_manager`/roam 应当被包络挡住，不该发出这个命令——那真正的错在上游而不是守卫 |

**A 与 B 的处置不一样，而且相反**：A 要清命令/改守卫武装条件；B 要改 roam 的边界行为，**守卫反而不该动**。所以我不"顺手修一个让它不响"。

## 3. 区分它的那一步（下一次复现就做，成本一次扫掠）

抓 `no_progress_ms` 起算前 2 秒的 **200 Hz 逐周期**：`phase / 命令模式 / requested_speed / q_yaw`。
- 若 `phase` 在窗口内一直是 hold 且速度请求是**阶跃残值** ⇒ **A**；
- 若窗口内 `q_ref` 明确朝 `-80°` 方向推进而 `q_yaw` 卡住 ⇒ **B**。
现有 1 Hz 遥测**不足以**分开这两条——**这就是我不下结论的原因**。
`tools/can_temp_peek`＋遥测能拿到逐周期数据；WP1b 的事件 recorder 正是要把这件事变成一条日志就够。

## 3½. 复现观察（已跑，03:12–03:27 本地）

停机归因排练把站点起停了 4 次，合计约 **15 分钟**的 AUTO_ROAM/AUTO_TRACK 真实运行：
**`no_progress` 一次都没复现**，四轮全部干净停车（`STOPPED, pitch disable confirmed`）。

⇒ 对本案的影响：**不是"一到扇区边就必然跳"的确定性故障**——两种假设都还活着，但 B（边界必然触发）被削弱。
⇒ 对我自己的提醒：**别把"没复发"读成"没问题"**。现场继续带着这条跑；下一次复发时，
`wp1b` 之前的替代品是我自己盯 1 Hz 遥测＋`tools/can_temp_peek`，逐周期那条仍然欠着。

## 4. 今夜明确不做的事

- **不降低这条硬守卫来换一条"成功"日志**（`00_CODEX_START.md`：不要为取得日志降低已有硬保护）。
- 不在没证据时选 A 或 B 去改代码。
- 复发观察交给现场：新 release（自审修复版）已经/即将在 AUTO_ROAM 下再跑一轮，**同一位置再跳＝可复现，跳在别处＝另有因**。

## 5. 与 WP 的关系

守卫"命令可动性"这条属于 **WP6**（单轴辨识、无扰模式切换、停止包络）与 **WP1c**（守卫治理：每个条件要能说清自己在防什么）。
本条为两者提供第一条真实反例：**现在这条守卫会把"到边"读成"坏了"。**

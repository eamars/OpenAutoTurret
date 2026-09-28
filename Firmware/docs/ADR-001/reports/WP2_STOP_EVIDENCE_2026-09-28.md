# WP2｜停止证据分轴（2026-09-28 08:4x，release `c9b4ffb1282e`）

**契约原话（`docs/04_CONTRACTS.md:110` §8）** 要求发布 `stop_id`/reason/`requested_at`/stage/每轴
`feedback_age`/`stationary_observed`/`last_neutral_request_at`/`pitch_disable_requested|confirmed`/
`yaw_zero_requested`/`yaw_disable_confirmed=null`/`power_isolated_confirmed=null`/
`completion_quality`/`missing_evidence`，并钉死两句：
**字段必须区分 requested/observed/confirmed/unsupported**；
**`stationary` 只证明那段窗口里没检测到运动，不证明断扭矩**；
**不得把两轴布尔简单 AND 后标绿"安全断电"**。

## 落成的样子

`control/src/control/stop_evidence.hpp` → `$RUN/traces/stop-evidence.ndjson`（追加，best-effort：
**磁盘满不许变成一种新的停不下来**）。`tests/test_stop_evidence.cpp` 5 条。

**类型层面就禁掉了那个错**：一轴不是 bool，是**每个主张一个枚举**
（`Absent/Requested/Observed/Confirmed/Unsupported`）——`pitch && yaw` 这种写法**在这里表达不出来**，
不是"劝你别写"。

**两次记录、同一个 `stop_id`**：请求一次（`stage=requested`）、完成一次（`stage=parked`）。
理由：**只让成功路径写工件，"没有记录"就同时意味着"什么都没发生"和"它中途死了"**——
而后者才是有人要查的那种。

## 现场第一条（真停一次，`run_application.sh stop`）

| stage | quality | 关键行 |
|---|---|---|
| `requested` | **unverified** | `missing=[pitch/yaw:no_stationary_observation, pitch/yaw:feedback_age_unknown]`；`age_ms=null`（**不是 0**） |
| `parked` | **verified** | pitch `dis_conf=confirmed`、stationary **557.9 ms**、age **4.573 ms**；yaw stationary **502.2 ms**（＝`park.dwell_ms`）、age **0.265 ms**、`dis_req/dis_conf=unsupported`；`power_isolated=unsupported`；`missing=[]` |

`stop_id=stop-60378202284708`；轮转把它带走了（`logs-history/20260927T194650Z-launcher1517752/traces/`）——
**"每轮的物证跟着每轮走"这条在真机上成立**，且**不用改 launcher**（`traces/` 整目录被归档）。

`stationary_observed` **沿用了本分支本来就要求的 dwell/位姿核验**（位置在容差内稳住一段窗口），
不是我自己新发明的速度阈；窗口随记录发布，**主张的强度就写在脸上**。

## 还没做（照实记）

1. **`fail_parking()` 不写工件** ⇒ 失败的停只有 `requested` 行；目前"请求了没有完成"＝失败信号，
   够用但不体面。补法：失败点也追加一条 `stage` 为失败名的记录，`missing_evidence` 会自己长出来。
2. **`stop_id` 与 `shutdown.cause` 没串起来**：谁按的（WP1a）与证明了什么（本次）分在两个文件。
   串起来要 cause 传进 controld（web 命令加一字段即可），下一片可做。
3. **`main.cpp` 的 `STOP FAILED` 路径不产出轴证据**——那条路径本来就没有轴级凭据（phase 已是 fault），
   记录缺席是诚实的；但值得在 runbook 里明说（已写进 STATION_OPERATIONS 的"未覆盖"段）。

## 深夜增补：关机会落一行，但 join 还没成（现场证据）

`--prebuilt` 部署（rc=0，站上 65 测试 0 失败）之后现场读到：

    {"kind":"shutdown","stop_id":"stop-none","parked":false,"phase":"fault",
     "cause":"safety interrupted homing: stale or missing motor feedback"}

**好消息**：结构化关机记录真的落盘了，`json.loads` 一行就解析过；而且它没粉饰——`parked=false`、
`phase=fault`。
**没成的**：`stop_id=stop-none` 是**诚实值**——这个进程不是被 stop/park 停的，它死在**归零被安全层
打断**（反馈陈旧）。于是问题从一件变成两件：① 归零途中反馈变陈旧就直接判 fault，正是 `08 §3` 点名的
边界（"反馈陈旧"、"park 中反馈丢失"），也和主人"能跑 > hold > fault、看不新鲜≠失控"那条同族；
② 停机与归零各走各的路径，`stop_id` 只覆盖前者，所以一个进程可以带着完整的一生结束而文件里只有末行。
⇒ 下一刀明确：把"请求停止"与"有资格宣称 parked"拆开（readiness 只能否定后者，不能拒绝停止本身）；
`control_loop.cpp` 里 mixed 分支现在有四处会**拒绝停**：需要 pitch 归零 + yaw 会话基准（:249）、
GM6020 温度/故障策略（:255）、反馈不新鲜、以及 `yaw_speed > 25 °/s`（太快反而不许停）。
`07` 的 Done 条写的是"stale 时可请求停止但不盲 park"，`08/S0` 要求"无 readiness 拒绝停止请求"——
现在这四条都和它对不上。这是行为缺陷，不是缺测试。

## 否证 · 免归零不能复用 RetainedHoming（2026-09-28 深夜，第 25 轮）

主人的裁决（相机测试不需每次归零；电机持续通电时免归零，代价是必须证明中间没断电、encoder 还记得）
**不能靠现成的 `RetainedHoming` 落地**：`control_loop.cpp:581` 里 `restore_retained_homing()` 遇到
`backend_->supports_continuous_yaw()` **直接 return false**，因为它要求两轴归零 + 软/硬限位结构
（`q_soft_max_rad`/`q_hard_max_rad`）——那是有限行程双轴的模型；本站 yaw 是 GM6020 连续旋转无 endstop。
`main.cpp:257` 的 `!mixed_mode` 门只是表象。

新立条目（下一场做，按 07 的格式补进 WP2/WP3 的 Done 条）：

> **WP2-N1 免归零凭证**：`zero_source ∈ {retained, homing}`，`retained` 需三条同时成立——
> ① 上次会话零点已落盘（pitch 绝对角 + yaw 会话计数）；② 驱动器通电连续性可证（同 boot 且记录新鲜）；
> ③ 落盘值与当前 encoder 一致（不一致＝中途断过电，立即退回归零）。
> 验收：热重启一次 ⇒ 日志 `zero_source=retained` 且**无归零动作**；拔驱动器电再上电 ⇒ `zero_source=homing`。

## WP2 ② 已落码（站离线，未部署）：重复 stop 幂等

`control_loop.cpp` mixed 分支在重置 park 状态前**没有守卫**，所以 park 途中第二次 STOP 会重算
`mixed_park_deadline_ns_`（＝反复按停止 ⇒ park 永不超时）**并重铸 `mixed_stop_id_`**
（＝第一次请求的证据链被丢在新身份之外）。现改为：`mixed_stop_park_` 为真 ⇒ 接受请求、直接返回，
**deadline / park 状态 / stop_id 三者都不动**。

诚实边界：容器里 **77/77 只证明没编译回归**——仓库里没有构造 `ControlLoop` 的测试，所以这条**没有单测钉住**。
站回来后的验收（写死）：park 途中连发两次 `stop_motion`/`request_shutdown`，
读 `stop-evidence.ndjson` 期望 **同一 `stop_id`、`deadline` 不被后推**；
若第二次请求换出了新 id，这条就还是 FAIL。

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

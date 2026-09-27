# WP1a 自审：跳闸原因这件事我哪里可能错（2026-09-28）

**背景**：09-28 02:30 他说「WP1a 不用我看了，我信你」。复审权移给我，**不等于免检**——
"这段我做对了"这句话要有分量，就得我自己先把它问倒。所以我这份不是"我检查过了"，
是**逐条列出我可能错在哪、哪几条已经变成代码、哪几条还欠证据**。
被审对象：`e446966`（watchdog trips arrive with their cause）。

## 1. 我找出的两处真错（都已改，见 §2）

### ① `reference_invalid` 会冒充真凶（错误归因，已修）

我原来那条 if 链里第二个候选是 `!yaw_reference_valid_` → 令牌 `reference_invalid`。
但守卫的 `should_stop` **里根本没有这一项**（它管的是反馈/CAN/总线/速度/温度/无进展/心跳）。
⇒ 只要 reference 恰好失效而真凶是别的，我就把**旁观者报成凶手**：
CAN 掉线与 reference 失效同时存在时，日志与 fault 串都会说 `reference_invalid`，
而让人去查的那条路是错的。**"多给一个原因"在这里不是更保守，是更危险。**

改法：`reference_valid` 从候选里拿掉，降级成矩阵里的上下文位（`ref_valid=0/1`）；
候选集合与 `should_stop` 的每一项一一对应，这条**用测试钉住**（见 §3）。

### ② 详情的读法不安全（数据竞争，已修）

原设计：守卫在 `yaw_mutex_` 下写 `yaw_trip_detail_`，然后发布 `yaw_trip_`；
读者**不拿锁**，只 `yaw_trip_.load() ? yaw_trip_detail_ : {}`。
发布方向没问题（写详情→置标志），**但清理方向不安全**：`close()` 里
`yaw_trip_.store(false); yaw_trip_detail_ = {};` **不带锁**，而控制环可能在
看到标志为真之后、复制这 124 字节的过程中被并发改写——按内存模型这是 data race（UB），
在 aarch64 上表现为"可能读到半个原因"。

改法：详情自己一把小锁 `yaw_trip_detail_mutex_`；写、读、清理三处都拿它。
嵌套顺序固定 `yaw_mutex_ → yaw_trip_detail_mutex_`，**没有反向路径**。
**健康周期的代价：一次原子读，无锁**（标志为假直接返回），200 Hz 路径不受影响。

## 2. 改动清单

| 文件 | 改动 |
|---|---|
| `control/src/control/motor_backend.hpp` | 新增 `TripInputs` + 纯函数 `select_trip_condition` / `format_trip_detail`（守卫的候选集合变成可测的东西，不再只在硬件跑起来才看得见） |
| `control/src/control/mixed_can_motor_backend.{hpp,cpp}` | 守卫改用选择器；详情读写清理共用 `yaw_trip_detail_mutex_`；矩阵加 `ref_valid`，`can_up` 改为语义更直的 `can_down` |
| `control/tests/test_watchdog_trip_events.cpp` | +4 条（见 §3），并把"最满的一行"改成走新格式化函数 |

## 3. 测试（这次改动的判据）

1. `AStateThatCannotLatchIsNeverNamedAsTheCause`：只有 `reference_valid=false`、其它都不触发 ⇒ `unknown`（不许编凶手）。
2. `ABystanderDoesNotShadowTheConditionThatFired`：`reference_valid=false` **且** `can_down=true` ⇒ `can_down`（这条就是我原来会答错的那个形状）。
3. `SeveralConditionsReportTheFirstInGuardOrder`：多因同触发 ⇒ 按守卫顺序取第一个，且顺序是契约不是意外。
4. `DetailBuffersHoldTheFullestHonestLine`：最满的一行 88 字符进 96 字节缓冲，**逐字钉死**——将来谁加字段撑爆了，这条会响。

## 4. 现场顺带白捡的一条证据（WP1a 在真实停机里说话）

部署 `480b779` 时旧栈停机，controld 打的是：

```
STOP FAILED: pitch STOP and yaw zero requested (phase=fault, fault='independent motor watchdog trip: no_progress')
```

**要是没有 WP1a，这里只会是 `control deadline, feedback, or motor health` 那十个原因共用的一句话。**
（这条同时登记在 `INCIDENT_STACK_STOP_2026-09-28.md` §7 的现场流水里；
`no_progress` 本身是新出现的跳闸形态，另案追查。）

## 5. 我还不服的地方（留给下一片，不装完）

- **`should_stop` 里 `health.state != ErrorActive` 才算坏**（ErrorActive 是"坏"的名字却当"必须为"用）：这是既有契约，我这一片不动它，但**我没有证据说明为什么**。→ WP1c 取 EAI/官方状态帧证据时一并问清。
- **`select_trip_condition` 与守卫顺序的一致性靠人盯**：候选集合我做了纯函数，但"顺序 == 守卫求值顺序"目前没有机器保证。真要做严，得让守卫的 `should_stop` 由同一份输入表算出来。→ 记进 WP1c（守卫治理本来就要重写这一段）。
- **详情只有 5 个字段**，全量矩阵仍在日志里。若事件消费者需要全量，那是协议问题（WP1b 的事件契约）而不是这里塞字符串。
- **TSAN/竞争复跑**：NOT_RUN。本仓库测试没有 sanitizer 目标；我这次是把并发写路径消掉（清理带锁），不是"跑过 TSAN 证明没问题"。这条不假装。

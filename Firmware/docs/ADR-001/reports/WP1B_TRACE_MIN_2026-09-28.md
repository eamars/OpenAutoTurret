# WP1b 第一片：逐周期痕迹补上"相位"和"热字节"（2026-09-28 凌晨）

**这片不等于 WP1b 完成。** WP1b 的合同还有 FrameIdentity、时钟映射、有界记录器落盘、
lineage、合成回放（§2/§5 of `docs/04_CONTRACTS.md`）——那些仍在欠。
这片只把**已经存在但缺关键字段的逐周期痕迹**补齐到"能分开两种相反解释"的程度。

## 1. 为什么先补这个（动机是有案的，不是顺手）

`OBS_NO_PROGRESS_TRIP_2026-09-28.md` 记了一次真实跳闸：`phase=hold`、还挂着 10 °/s 请求、
那一秒 yaw 只走 0.04°、位置 −76.5°（软限位 ±80°）。我当时写下的结论是：

> **1 Hz 遥测分不开两种解释，而这两种解释的修法相反。**
> A＝陈旧速度请求（该改守卫的武装条件）；B＝换向点朝限位要到运动（守卫没冤枉，该改 roam）。

仓库里其实**早有逐周期痕迹**，字段有 `q / ref / vref / cmd / effort / rx / vest / safety / period`。
**但我当时把它的深度说错了，这里更正**：能经 socket 被读到的那条 `trace_` 环只有 **256 条 = 1.28 s**
（`telemetry.hpp` 里写死的裸 `256`；我先前按另一条仅供内部的 `control_log_`（4096 ≈ 20 s）说话，
那条不回答 socket）。**1.28 秒根本不叫证据**——跳闸之后循环还在写，一秒半就把自己冲干净了。
这一条是 04:5x 我第一次真去读它之后才发现的：**没读过的东西，连自己的假设都保不住。**
**唯独没有"这条是在哪个 phase 写的"**。而没有相位，`cmd≠0 且 q 不动` 这两件事
在 A、B 下长得一模一样。所以缺的不是"更多数据"，是**一个位**。

## 2. 改了什么（加法，没动任何控制路径）

| 位置 | 改动 |
|---|---|
| `telemetry/telemetry.hpp` | `ControlLogRecord` 新增 `Phase phase`、`int temp_raw[kAxisCount]`（−1＝该轴没有这个字节） |
| `control/control_loop.cpp` | 写记录时填 `phase_` 与每轴热字节；**只在原有那次 push 里填，没加第二次遍历** |
| `web/web_server.hpp` | `read_control_trace` 每行多两个键：`"phase":"hold"`、`"temp_raw":[pitch,yaw]` |
| `control/phase.hpp`（新） | `enum class Phase` 与 `phase_name()` 从 `control_loop.hpp` **原样搬来** |

新头文件的理由很无聊也不许省：`telemetry.hpp` 不能 include `control_loop.hpp`（方向反了），
而**我拒绝手工维护第二套相位词汇**——两处各自维护，早晚会有一处漏掉 `Recovering`。

`temp_raw` 的 −1 是有语义的：pitch 的 CyberGear 没有裸热字节，"没有"比 `0` 或 `25` 诚实。

## 3. 验收

| 类别 | 命令 | 结果 |
|---|---|---|
| 单元 | `./build/control/test_telemetry --gtest_filter="*ControlTraceRow*"` | **PASS**（钉住：环里读回 `phase`/`temp_raw`，**含 −1 不被吃掉**） |
| 全站台 | `ctest -E retained_homing`（容器） | **76/76** |
| 现场 | 部署后拉一次 `read_control_trace` | **NOT_RUN**：全栈里**没有任何调用者**（webd 没有 trace 路由，`read_control_trace` 只能靠人连 WebSocket）。字段确实在真站的环里，但我**不拿"编译进去了"冒充"我看到了"** ⇒ 见 §4.1 |
| mock/replay | 不涉及（本片不新增观测语义，只补上下文位） | N/A |

## 4. WP1b 还欠什么（写清楚，别让下一位以为做完了）

0. **跳闸瞬间的冻结已做**（commit `3a273d9`）：`fault()` 与 park-failure 落点各取深环最新 **1024 条 ≈ 5.1 s**，
   `read_control_trace` 用它作答并带 `"frozen":true,"frozen_t_ns":…`；**先落定者赢**——后来的 park 失败
   不许拿残骸覆盖守卫自己那份窗口。冻结处**允许阻塞**（一次、有界），而每周期发布仍是 try-locks 丢样本：
   **每秒丢一个样本和为一台已经决定停机的机器阻塞一次，是两个不同的交易。**
1. **谁在跳闸那一刻把它抄下来**：现在 `read_control_trace` 是"有人来问才有"。
   要的是一个**有界、非阻塞、故障触发**的落盘（`$OTA_RUN_DIR/traces/…ndjson`，
   与 `logs-history` 同一个归档策略），并且**行内 ns 用十进制字符串**（`04_CONTRACTS.md` §2 末：
   64 位 ns/seq 不许在 JS 里变 Number）。
2. **lineage**：一行 trace 要能指回当轮的 session_id / release / config 摘要。
   `stack.info` 已有 release 与 config 路径，缺一个把三者钉成一行的 id。
3. **时钟语义**：控制域继续 `CLOCK_MONOTONIC`；BOOTTIME 偏移、跳变检测、`clock_epoch` 提升——**未做**。
4. **FrameIdentity / sensor_timestamp_semantics**：视觉侧那一大摊（§2 表）**一行未动**；
   本片只碰控制域，所以不敢写"WP1b 完成"。
5. **合成回放**：`synthetic_trace_summary.json` 是 WP0 的形态验证，不是逐周期回放。

## 5. 现场回填

### 已做到（04:5x，release `9b03fede4a70`，真站）

写了调用者：`Firmware/tools/pull_control_trace.py`（纯 stdlib，直接读 controld 的 SEQPACKET unix socket）。
第一次真去读，就**读出来一个洞**：那份回答**不是合法 JSON**——yaw 没有扭矩量，`effort` 是 NaN，
而这条手写序列化线吐出的是 `std::to_string` 的 `nan`。
（同一个文件里 telemetry 那条线**早就有** `json_finite_or_null` 和 `UnknownYawTorqueIsValidJsonNull` 这条测试；
**教训在屋里，500 行外手写的第二条线没继承**。）修好之后现场原文：

```
rows=256 span=1.289s phases={'hold': 256} yaw_temp_raw=27..27
newest: {"t":45663870996110,"ack":2,...,"effort":[0.0357056,null],
         "phase":"hold","temp_raw":[-1,27],...}
```

⇒ `phase` 与 `temp_raw` 确实在真站上跨线；`effort` 的 yaw 是 **null**（"没有这个量"），
不是 0、不是 nan——**三个数字各说各的事**。`--selftest` 7/7，钉住 NDJSON 里 ns 与 64 位 seq 走十进制字符串。

### 仍然 NOT_RUN 的一条（写清楚，别混）

**`"frozen":true` 的端到端证据还没拿到**：单元里冻住了、跨线也带标记（`test_web_server` 钉住），
但**今夜真站没发生一次跳闸**，所以现场那行 `FROZEN_AT=…` 我**没亲眼看到**。
下次任何 `fault()`/park-failure 落定时跑一次 `pull_control_trace.py`，摘要行就会以 `FROZEN_AT=` 开头；
看到之前，这条就记 NOT_RUN。

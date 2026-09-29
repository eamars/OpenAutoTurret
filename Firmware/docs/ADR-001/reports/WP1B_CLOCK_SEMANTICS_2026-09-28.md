# WP1b｜时间戳语义（2026-09-28 清晨，先量再定）

**这一片不是设计题，是测量题**：先搞清"我们在用哪把钟"，再决定每个工件必须自带什么。
可跑的东西：`tools/clock_probe.py`（纯标准库，站上和容器里各跑一遍）。

## 1. 量到的事实（一手，2026-09-28 07:2x）

| 机器 | 读数 |
|---|---|
| 站（Pi 5，`eamars@192.168.2.100`） | `MONOTONIC=55354.647348`、`BOOTTIME=55354.647362`、`REALTIME=1790533381.711` |
| **BOOTTIME − MONOTONIC** | **0.0000149 s（15 µs，纯读差）** ⇒ **这台机器从未 suspend**（差值累计的就是睡过去的时间） |
| REALTIME − MONOTONIC | 1 790 478 027.1 s（= 那一刻的换算锚点；对上墙上钟 2026-09-27T18:23:01Z） |
| `/proc/uptime` | 55354.64 ⇒ 与 BOOTTIME 一致（**控制器 1 Hz 状态行里的 `t=` 就是这一族**） |
| `/sys/power/suspend_stats` | **不存在** ⇒ 没有第二条路能反驳"从未 suspend" |
| `boot_id` | `ed65e7b6-4e32-43f7-afe5-41fb532eb200`（这一次开机的身份） |
| `timedatectl show -p NTPSynchronized` | 读不到值（属性名/服务差异）⇒ **时钟同步状态是下一条要落实的**（见 §4） |

**结论**：控制路径（`common/time.hpp` → `CLOCK_MONOTONIC`）**在这台机器上与 BOOTTIME 实际等价**，
所以 suspend 不是今天的风险；但"等价"是**这台机器现在的事实**，不是语义——语义要靠工件自己声明。

## 2. 我昨天自己埋的坑（这一片的真正内容）

跳闸冻结文件（`$RUN/traces/trip-*.ndjson`，昨夜 WP1b 交付）头部只有 `frozen_t_ns`：
**没有钟名、没有开机身份、没有墙上钟锚点**。它只在一件事上安全——**留在正在跑的那台机器上**。
而它注定不会：今晨实测两台归档轮次里各躺着一枚（`logs-history/…/traces/trip-*.ndjson`，375 KB），
**把它们放在时间轴上的唯一线索，是它们被归档进的那个目录的名字**。
**依赖一个目录名，等于没依赖。**

## 3. 定下的语义（写进代码与测试）

> **任何活得比一次开机久的东西，必须自带四样**：钟名、本次开机身份、冻结那一刻的墙上钟、
> 以及"单调钟 → 墙上钟"的偏移。缺一样，读的人就只能猜。

头部现在长这样（`telemetry.hpp`，同一瞬间测得）：

```json
{"kind":"trip_trace","rows":1024,"frozen_t_ns":"…","clock":"CLOCK_MONOTONIC",
 "boot_id":"ed65e7b6-…","wall_t_ns":"179053…","mono_to_wall_ns":"1790478…"}
```

- ns 一律**十进制字符串**（沿用 §2 的规矩：墙上钟 ns 超 2^53，JSON number 会掉低位）；
- 测试读的是 **daemon 真写出来的文件**，并要求 `wall_t_ns` 是个像样的墙上钟——
  **`0`（"没人设过"的形状）必须在这里失败**，而不是三周后有人复盘时问"云台到底是几点停的"。
- **顺手一条自我记账**：此前**没有任何测试断言过头部**。今早那个"少一个收尾引号"和这次"少一个锚点"
  是同一条教训的两次：**我只断言了行，放过了头。**

## 4. 还欠的（按能验的程度排）

1. ~~live 那条路没有锚点~~ **已补（`e7c8cee`）**：帧头现在也带
   `clock`/`boot_id`/`mono_to_wall_ns`，`frozen_t_ns` 同时改成十进制字符串（对齐 §2；
   它一直是 number，ns 级会掉低位——`pull_control_trace.py` 的自检本来就按字符串写）。
   **现场结账分两条**：
   - **帧头（live）= 当场可验**，不需要跳闸：拉一次 `read_control_trace` 就能看到 ⇒ 本轮现场验。
   - **文件头（freeze）= 单元已验、现场待下一次跳闸**：`$RUN/traces/` 现在是空的，
     我不拿单元格的功劳算现场的。命令：`python3 <release>/Firmware/tools/pull_control_trace.py`。
2. **NTP/同步状态没有落实**：今早 `timedatectl show -p NTPSynchronized` 取不到值。
   墙上钟锚点若机器自己没对时，只是"这台机器以为的墙上钟"。⇒ 启动时把同步状态一并写进行，
   或在 runbook 里写明"锚点是本机时钟，跨机比对要另加纪律"。
3. **启动时打印 `BOOTTIME − MONOTONIC`**（三行）：将来谁让这台 Pi 睡了觉，日志里第一眼就能看见。
4. **`t=` 状态行的单位**在 runbook 里没写死是哪把钟（我这次是**推断** BOOTTIME 与 `/proc/uptime` 一致）
   ⇒ 与 1) 一起补一句"哪把钟 + 锚点在哪"。

## 5. 现场结账（07:4x，release `e7c8cee749b6`）

在站上拉一次 `read_control_trace`（不需要跳闸就能验 live 那条路）：

```
帧头： {'type':'control_trace','axes':['pitch','yaw'],'frozen':False,'frozen_t_ns':'0',
        'clock':'CLOCK_MONOTONIC','boot_id':'ed65e7b6-4e32-43f7-afe5-41fb532eb200',
        'mono_to_wall_ns':'1790478027063700826'}
用锚点换算最新一行的 t  → 2026-09-27T18:46:32.678Z，与本机墙上钟差 **0.056 秒**
boot_id 与 /proc/sys/kernel/random/boot_id **一致**
```

⇒ **验收通过的标准不是"键存在"，是"拿着锚点能把行放到墙上钟上"**：差 56 ms。
这 56 ms 是量移（两次 syscall 在锁外）＋ 序列化＋socket 传输的和，
**在 200 Hz（5 ms 一周期）的尺度上是 ~11 个周期**——谁要拿痕迹去对相机帧，得知道这个量级；
要更准就得把锚点测在**快照那一刻**（锁内），那是一次 syscall 换 11 个周期，现在不值。

**仍待现场**：文件头（跳闸才写）。`$RUN/traces/` 现在为空；下一次跳闸后
`head -1` 那个 ndjson 就该看到同样的四个键。命令与判据在 §4-1。

## 6. 合成回放（`tools/offline_checks.py`，本轮补上）

ADR-001 README 第 33 行一直写着 `python tools/offline_checks.py summarize examples/synthetic_trace.ndjson`——
**这条命令此前指向一个不存在的文件**。这一支把它变成真的，顺带把昨天那次"文件不是合法 JSON"焊成规矩：

> **离线检查器的每一条线都必须真解析。** `find(键名)` 型断言就是放过那次事故的元凶。

- `make-example` 写一段**合成**痕迹（形状取自今早 06:59 那次 `no_progress`：`AUTO_ROAM/search`、
  参考点在走、`cmd=[0,0]`、`effort` 一路 `null`、`temp_raw=[-1,27]`）；
  例子头部用的是**那一次真实的锚点数**，所以 `summarize` 把它放到墙上钟正好是 **17:59:38Z＝06:59:38 NZDT**。
  **为什么必须合成**：仓库规矩不许把现场抓取物提交进库——所以能提交的只有合成件。
- `check`：逐行解析、头部四要素齐不齐、ns 是不是字符串（number 就拒——超 2^53 会掉低位）、
  `t`/`ack` 单调、`temp_raw` 不许拿 0 当缺席，还能 `--source .` **拿 C++ 源码里的词表**核对
  文件里的 `phase`/`mode`/`track`——**词表漂移当场红**。
- `--selftest`：**6/6**，六个坏件全被拒，其中包括**现场真发生过的那次**（少一个收尾引号）。

**现场一侧的两个副产品**：
1. 拿今早 06:59 那枚**真**文件跑 `check`：它因"缺 clock/boot_id/wall_t_ns"被拒——**正确**，
   那文件写于加锚点之前（头部只有 `kind/rows/frozen_t_ns`），**来源规矩从此有机可查**。
2. 同一枚真文件 1024 行**全部解析成功**，行内是 `"track":"search","phase":"hold"`
   ⇒ **今早那个收尾引号的修复，在真机写的文件上验过了**（不是只在单元里绿）。

## 7. 更正与对齐（写完后才发现我读错了 ADR 的那行引用）

**我上一版说错了一句**：我说 README 的
`python tools/offline_checks.py summarize examples/synthetic_trace.ndjson` "指向一个不存在的文件"。
**错**——那条命令是**相对 `docs/ADR-001/`** 的，包内 `tools/offline_checks.py` 与
`examples/synthetic_trace.ndjson` **一直都在**，还被 `SHA256SUMS.txt` 校着。
**我把包内路径当成了仓库根路径。**（今日第四处"我以为"，仍然写在这里不抹。）

而且包内那个 `synthetic_trace.ndjson` 与我的**不是一个东西**：它是**观测谱系**痕迹
（`ota.n1.draft.trace-event/1`：capture→inference→publish→controller_receive），
它自己的 `summarize` 文档字符串明说 **"requires proposed event names; it is not an adapter for
existing station CSV"**——**"消费真格式"这件事确实还没人做**。⇒ 我这份的定位是**真格式适配器**，
为避免两处同名互相冒充，**改名**：`tools/trip_trace_checks.py` ＋ `examples/trip_trace_sample.ndjson`。

**对齐契约（`docs/04_CONTRACTS.md:22,32`、`docs/08_ACCEPTANCE.md:42`）**：契约早就定义了词——
`clock_epoch / clock_mapping_id`＝**时钟转换的有效期与映射身份**；要求是
"控制域继续 `CLOCK_MONOTONIC`；读 BOOTTIME/MONOTONIC 建 **offset 并带误差界**；
运行前后查漂移与跳变；**运行期间禁止 suspend**；跳变/换 boot/映射失效 ⇒ **提升 clock_epoch 并撤销旧观测**；
**禁止跨 boot 拼接统计**"。逐条对：

| 契约要求 | 我现在做到哪 |
|---|---|
| 控制域继续 `CLOCK_MONOTONIC` | ✅ 本来如此，且工件现在**声明**了钟名 |
| 建立 offset | ✅ `mono_to_wall_ns`（冻结/取窗那一刻测）＋ 现场实测残差 **0.056 s** |
| **带误差界** | ❌ 我只报了一次单点，**没声明界** ⇒ 待做：offset 界随头部发布 |
| 运行前后查漂移与跳变 | ❌ 单次读数 ⇒ 待做：启动＋周期性复测，超界即判定跳变 |
| 提升 `clock_epoch` 并撤销旧观测 | ❌ 我发的是 `boot_id`（覆盖"换 boot"这一种），**没有 epoch** ⇒ 待做 |
| 禁止跨 boot 拼接统计 | ✅ **可被查了**：`boot_id` 在头部；`trip_trace_checks.py` 拿到两枚不同 boot 的文件就该拒绝合并（这条下一步加） |
| 运行期间禁止 suspend | ✅ 可证：今日 BOOTTIME−MONOTONIC = 15 µs；**界**要长期测才谈得上 |

⇒ **WP1b 时间戳语义的"未做"从整块变成了三条**：误差界、周期复测/跳变判定、`clock_epoch` 发布与旧观测撤销。

## 8. 三条剩余项关掉（`f6527fa6fe54`，08:2x 现场）

| 契约要求 | 现在 |
|---|---|
| offset **带误差界** | ✅ `mono_to_wall_err_ns`：mono→wall→mono 三次读、取最紧一对、半跨度算界。**现场实测 29 ns（0.029 ms）** |
| 运行前后查漂移/跳变 | ✅ 每次发布工件都复测；两次真实样本相隔约 40 分钟：`…700826` → `…700891`，**差 65 ns**（跳变判定阈 10 ms，差三个数量级）⇒ 这台机器的映射稳；界与阈都不是装饰 |
| 提升 `clock_epoch` 撤销旧观测 | ✅ 发布头带 `clock_epoch`（本机现在 =1，从未跳变）；跳变走 `clock_epoch_after()`——**抽成纯函数按算术测**（本服务不调 `settimeofday`，今晚也不 sudo）；旧行留在旧 epoch，不悄悄重定基 |
| 禁止跨 boot 拼接统计 | ✅ `trip_trace_checks.py` 一次传两枚：跨 boot / 跨 epoch 直接拒；自检 **9/9** |

**现场那一帧**（`read_control_trace`，release `f6527fa6fe54`）：

```
clock: CLOCK_MONOTONIC  epoch: 1  界: 29 ns  boot: ed65e7b6
偏移: 1790478027063700891
```

**一处测试写法上的收获**：我给帧断言写死了 `"clock_epoch":1`，红了一次——因为 web 那条测试
是**手搓的 TraceWindow**，它从没见过映射，于是老老实实写 `epoch 0 / boot "unknown" / 界 0`。
**断言错的不是代码，是断言的对象**：帧测试该管"键在不在、缺席是不是写成缺席"，
"值对不对"归 telemetry 那个真写了文件的测试。⇒ 改完 76/76。

**这一片还剩**：文件头锚点与 epoch 的**现场证据**（要等下一次真跳闸写文件；`$RUN/traces/` 现在空着）
——判据：`head -1` 那枚 ndjson 应看到 `clock/boot_id/wall_t_ns/mono_to_wall_ns/mono_to_wall_err_ns/clock_epoch` 六样齐。

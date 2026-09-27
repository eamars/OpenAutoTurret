# 拆掉 yaw 的虚拟限位：影响面与设计（2026-09-28，主人交办）

**这不是 ADR-001 的活**，所以不放 ADR 目录。主人原话：**"涉及的有点广，希望你仔细考虑。"**
这份是先于代码的分析；代码按 §6 的顺序做，每步都有可跑验收。

---

## 1. 先说清"现在的限位是什么"

| 事实 | 出处 |
|---|---|
| yaw 是 GM6020，`topology: continuous`、`control_mode: voltage`，**没有机械端stop、永不对着挡块归零** | `config/mixed_hardware.yaml`、`GM6020_AI_Reference.md` |
| yaw 的 ±80° **不是测出来的**，是**会话内软件扇区**：站文件 `expected_travel_deg: ±90` ＋ `soft_margin_deg: 10`（配置里自己写着 **"Provisional"**） | `config/turret_mixed.yaml:47-53` |
| 扇区在代码里只有**一个汇合点**：`ControlLoop::runtime_limits(Yaw)`（其余 9 个调用点都从它取） | `control_loop.cpp:3367-3385` |
| **位置环在我们这侧闭合**：发给电机的是电压/速度帧（`0x1FF`），**不发绝对角** ⇒ "自由转"不存在"电机自己选近路"的歧义 | `mixed_can_motor_backend.cpp:707/727/742` |
| 16 位反馈角已被 `gm6020::UnwrappedEncoder` 解成**会话内连续角**（`int64_t counts_`、Δ 绕回消歧、速率合理性、80 ms 观测窗上限） | `can/gm6020_protocol.hpp:54-86` |

⇒ **结论：拆扇区在机制上可执行**（不是"电机能不能"的问题）。**真正的广，在"无界"这个状态对所有消费者的语义**——见 §2。

---

## 2. 核心设计决定：**"无界"必须是第三种状态，不能复用 `valid=false`**

`AxisLimits` 现在只有两态：`valid`（测到了）/ `!valid`（还没测到）。`!valid` 的语义是**坏**，不是**无界**，而各消费者拿到 `!valid` 时的行为是**"动不了"**：

| 若把"无界"塞成 `valid=false` | 后果 |
|---|---|
| `in_soft()` 返回 false（`safety_envelope.hpp:52`） | **手动 step 每条都被拒**、参考点夹紧全部失效（`control_loop.cpp:4330`） |
| `distance_to_soft()` 返回 **0.0**（`:59`） | 遥测 `soft_limit_distance_yaw_rad` 报成**"贴着边界"**（**拿 0 冒充未知**——正是本仓库明令禁止的那类假象） |
| 制动治理按到界距离限速 | **yaw 永远不许动**（距离 0 ⇒ 上限 0） |
| `soft_limits_valid`（readiness 门）| 站台**永远不就绪** |

⇒ 所以 `AxisLimits` 加一个明确声明：`enum class Envelope : uint8_t { Measured, Virtual, Unbounded }`
（`Measured`＝归零测到；`Virtual`＝今天的会话扇区；`Unbounded`＝**政策声明该轴无包线**）。
**`Unbounded` 是立场，不是缺数据**——文件里写明，UI 里看得见，测试里钉得住。

各语义在 `Unbounded` 下的定义（**逐条可测**）：
`in_soft()=true`、`in_hard()=true`（不可能越界）、`distance_to_soft()=无`（不是 0）、
`max_speed_at()=v_max`（不受边界调制）、Layer-2/3 停止可行性**天然成立**、
`soft_limit_distance_*_rad`（遥测快照里那两个）⇒ **`null`**（与 `temp_raw=-1` 同一套"缺席要说缺席"的纪律）。
> **这一句最初写错过**：我写的是"遥测**与 trace 的同名字段**"。收口时拿真跳闸文件核了一遍——
> 冻结窗口的行只有 `t/ack/mode/track/phase/temp_raw/q/ref/vref/cmd/effort/vest/rx/safety/period_us/goal`，
> **痕迹里从来没有距离这一列**（`ControlLogRecord::soft_limit_distance[]` 这个字段存在，但写文件不发它）。
> 所以：遥测那格确实要发 `null`（已发、已现场验）；trace 那格**没有需要新语义的东西**，
> 代价是事后复盘查不到"当时离边界多远"——**这是一个被命名的缺口，不是一句已完成的承诺**。

## 3. 九个调用点逐个交代（这是"广"的实情）

| # | 位置 | 现在拿扇区做什么 | `Unbounded` 下改成 | 我怎么验 |
|---|---|---|---|---|
| 1 | `runtime_limits()` | 合成扇区（唯一汇合点） | 按配置声明返回 `Unbounded` | 单元：站文件写 0 ⇒ 状态是 Unbounded 而非 !valid |
| 2 | `safe_envelope()` 2918 | 给 roam 规划器两条 yaw 边 | **见 §4**（roam 区域与包线解耦） | 单元＋现场扫掠 |
| 3 | roam 配置 3008 | 扇区内缩后当扫掠边界；已有 `roam_region_named`/`roam_yaw_min/max_deg` 可再收窄 | **roam 用"命名区域"（相对进入点）**，与轴包线无关 | 单元：无包线时 roam 仍是有界扫掠 |
| 4 | 搜索扫掠 505 | `ready ± search_span` 与扇区取交 | 无包线 ⇒ 只按 `search_span`（本来就相对 ready） | 单元 |
| 5 | `command_state_.q_min/max` 2845 | 给 dashboard 显示可命令区间 | 发 `null`（＋`envelope:"unbounded"` 及其**来源**） | webd 测试：UI 不许显示 ±0 |
| 6 | supervisor 698/993/2626 | 硬界越界 ⇒ BRAKE | yaw 永不越界；**其余九项守卫一项不动** | 单元：注入无包线不产生 DERATE/BRAKE |
| 7 | `manual_step` 4329 | 目标出软界/进制动余量 ⇒ 拒 | yaw 不设限（pitch 照旧拒） | 单元＋现场：跨 ±180° 迈一步 |
| 8 | 遥测快照 3309 | 距离（0＝贴界） | `null` ＋ `yaw_envelope:"none"`（**已发已验**：`/api/state` 三个字段 `None`）；**trace 行不带这一列**（真跳闸文件的键清单为证）⇒ 事后复盘查不到，缺口见 §2 注 |
| 9 | `soft_limits_valid`（readiness） | 两轴都要 valid | **改语义为"每个轴的包线状态都已声明"**（Measured/Virtual/Unbounded 都算声明） | 现场：部署后 readiness 必须照旧点亮 |

## 4. roam 怎么办（我认为最容易做错的一处）

`AUTO_ROAM` 的文档承诺是**"一次确定性、有界扫掠"**（`roam_planner.hpp` 头注）：
"任何看着它的人都能说出下一个位置"。**轴变成无界，不能顺手把承诺改成"一直转圈"。**
所以：
- **包线（能不能去）** 与 **扫掠区域（今天去哪儿）** 从此分开：
  扇区拆掉 ⇒ yaw 包线 `Unbounded`；**扫掠区域仍必须有界**，用**进入 AUTO_ROAM 那一刻的位姿为中心的相对区域**
  （`±roam_yaw_span_deg`，与已有的 `roam_region_named` 同一套机制），**不累积、不越跑越远**；
- 无包线时**"往边界走"这个状态机分支不再存在** ⇒ 扫掠的两端由相对区域给，掉头逻辑不变；
- 跟踪（AUTO_TRACK）参考点：无包线 ⇒ 不夹紧，但**每周期仍受 `max_velocity 10 °/s` 与制动模型约束**（速度上限与扇区无关，仍从配置取）。

## 5. 风险清单（我不假装没有）

1. **滑环与线缆**：拆扇区的正当理由就是滑环（`commissioning` 记录：主人确认"有滑环、yaw 无端stop"）。
   但滑环是否**无限**扭转次数，现场没有一手依据 ⇒ 我把这条列为**需要他确认的一项**（不阻塞拆，阻塞"从此无人管地一直转"）。
2. **会话角的连续性依赖观测节奏**：`UnwrappedEncoder` 在 **>80 ms 无反馈**时判无效（历史上出现过 65.8 ms 调度缝）。
   扇区时代跨不过缝也无所谓；无界之后**长时间丢帧 ⇒ 会话角失效**，此时**必须按已有守卫走**（编码器失效＝不许再动），
   这条要在测试里钉住：**无包线绝不放宽时效守卫**。
3. **无界 ⇒ 没有"到边"这个天然终点**：所以 §4 的"扫掠区域必须自己声明"是硬要求，不是选择题。
4. **那桩 `no_progress` 案的证据**：假设 B（朝扇区边顶）**在拆掉扇区后不可观测**。
   我评估**证据损失很小**：① 15 分钟真实 AUTO_ROAM/AUTO_TRACK 未复现；② 我 05:2x 的手动 jog 显示
   **参考点在边界内侧就被夹紧**（顶不到边，`cmd` 全 ≈0）——**B 的机制本身被削弱**，天平往 A（陈旧速度请求）偏。
   而且**扇区在配置里仍可表达** ⇒ 将来要做对照实验，改一行站文件就能装回去。
5. **UI 与文档**：dashboard 的"resolved limits 及其来源"必须显示 `unbounded`，不许显示 ±0 或 ±90 的旧数——
   这条我列为验收项，不是"以后再说"。

## 6. 施工顺序（每步都能验，不许一次翻完）

1. **加状态、不改行为**：`Envelope` 三态进 `AxisLimits`；`runtime_limits` 在配置**没声明**扇区时才给 `Unbounded`；
   站文件仍写 ±90/10 ⇒ **真站行为逐位不变**（回归靠现有 76 项＋新单元）。
2. **语义补齐**：§2 那张表（`in_soft`/`distance_to_soft`/`max_speed_at`/遥测 `null`/readiness 新语义）
   ＋ 每条一个单元；webd 侧一行测试。
3. **roam/搜索/step 与包线解耦**（§4）＋ 单元（无包线时扫掠仍有界、step 不被拒、pitch 照旧受管）。
4. **拆**：站文件 yaw 的扇区声明改成"无界"（**写明理由与撤回的代价**）；部署后**现场四验**：
   ① 手动 step 跨 ±180°，会话角不失真（trace 原文）；② AUTO_ROAM 在新扫掠区域跑通一轮，两端掉头；
   ③ 中途 `stop` 仍旧 `STOPPED (pitch disable confirmed)`；④ `soft_limit_distance_yaw_rad=null` 且 readiness 点亮。
5. **文档**：`STATION_OPERATIONS.md` 增加"yaw 无包线运行"一节（怎么撤：改回一行站文件；撤了会失去什么）。

## 7. 要他确认（不阻塞，但别怪我没问）

- **滑环允许无限扭转吗？允许连续转几圈？**（这决定 AUTO_ROAM 是否允许"跨圈"的扫掠中心漂移，以及要不要一个**扭转累计计数器**当作观测。）
- 拆完之后 AUTO_ROAM 的**扫掠区域**默认多大（我先按现有 `search_span` 同量级，可配）。
- 要不要顺手把 **yaw 扇区从"站文件里的行程带"里搬出来**，独立成 `yaw.software_sector_deg`（我现在用"缺省/0＝无界"表达；搬一次名更清楚，但那是第三个提交的事）。

---

## 8. 落地时被打脸的三处（写完 §1–§7 之后才知道的，按规矩不许抹掉）

1. **"拿 0/0 当无界哨兵"是错的，而且是被校验器拦下来的。**
   我最初把哨兵定成 `expected_travel_deg: {min: 0, max: 0}`，结果 `turret_config.cpp:383`
   直接拒绝 `min >= max`——**这条挑剔是对的，不是我碍事**：零宽行程带正好是这次要消灭的那种歧义
   （"没人测过"和"声明为零"长得一模一样）。⇒ 改成显式键 **`position_envelope: none`**，
   行程带保留它另一份工作（**命名 roam 区域在它内部校验**）——这也是"没有墙还敢命名区域"成立的原因。
2. **§2 我写重了两处。** `max_speed_at()` 与 `stop_feasible()` 对 `!valid` **本来就返回"不受约束"**
   （不是我以为的"限速归零"），`reference_manager` 的拒绝也有 `valid` 前缀守卫。
   真正会瘫痪的只有 **`in_soft()`**（step/故障判定）与 **`distance_to_soft()` 的 0**（遥测说谎）。
   ⇒ 第四态照样要，但影响面比我吓自己的那份小——**"我列的清单要先怀疑一遍"**，今晚第二次撞上同型的错。
3. **`fetch()` 对缺键返回可用节点**，我用 `if (fetch(...))` 判存在 ⇒ 每个不带新键的文件都被判成"写了个怪值"。
   现用 `anode[key].IsDefined()`。**教训：仓库的 helper 不等于我认识的 helper，先看用法再写。**

## 9. 撤与不撤，我现在的话

拆的**理由**是物理的（连续轴＋滑环＋无端stop＋那个数自己写着 Provisional），
拆的**代价**是"到边"这个天然终点没了——于是 AUTO_ROAM 的有界性从"墙给的"变成"文件声明的"
（§4），这是我唯一真正改变行为语义的地方，也是我给它单独写测试（`NoOuterYawWallDoesNotExcuseAnEndlessSweep`）的原因。
**撤回 = 删掉 `position_envelope: none` 一行**，其余全部保留（第四态、roam 区域、遥测 `null` 都不白做：
它们描述的是"包线是什么"，不是"包线有多大"）。

---

## 10. 现场结果（09-28 06:3x–06:5x，release `9027791` → 修复版）

**先说要紧的：它转过了 ±180°，而且是拿手推过去的。**

| 时刻(NZDT) | 我看到什么 | 它证明了什么 |
|---|---|---|
| 06:34:01 | 启动日志 `[warning] continuous yaw declared WITHOUT a position envelope` | 声明在日志里说出口，不靠"少了个限制"去猜 |
| 06:34 | 部署的就绪轮询点名等 `soft_limits_valid` ⇒ 点亮 | **无包线不等于不可用**（readiness 的新语义在真机上成立） |
| 06:36–06:37 | 手动 jog `yaw+`：`q_yaw` **−0.3° → +274.1°**，单调、跨 +180° **无跳变**、无 fault、≈10.6 °/s | 会话连续角跨编码器缝不失真；**同一根探针昨晚只能到 +74.3°**（扇区挡的）——这是拆之前/之后的天然对照 |
| 06:40:46 | 我交回 AUTO_ROAM，`ROAM_RECOVERY interrupted_sweep dir=−1`，`roam_target_yaw=+41.6°`，`roam_pattern=BOUNDED_SWEEP` | **没有墙，扫掠区域照样存在且有两条边**（§4 的设计成立） |
| 06:41:01 | `GM6020 guard trip … speed_over_ceiling` → `supervisor: BRAKE` → `phase=fault` | **见下面那条坏消息** |
| 06:41 | `$RUN/traces/trip-52834529264141.ndjson`：375 KB / 1024 行 / 5.173 s / header 带 `frozen_t_ns` | **WP1b 事件记录器的现场端到端证据到手**（整晚欠的 `"frozen":true` 那条，就此销账） |

### 坏消息（一次我自己开门放进来的跳闸，以及三个互相矛盾的数）

长接近（164° → 41.6°）把 yaw 驱动到超过后端自带的 **25 °/s** 硬闸 ⇒ `speed_over_ceiling` 跳闸。
追下去发现**三个数各说各话，谁都没错在一起**：

| 谁 | 依据 | 值 |
|---|---|---|
| 站文件 `axes.yaw.max_velocity_deg_s` | 操作者写的声明 | **10 °/s** |
| roam 意图的速度上限 `motion_speed(AutoRoam)` | **两轴取 max** ⇒ 拿到的是 **pitch 的 30 °/s** | 30 °/s |
| 后端独立守卫 | 硬编码常量 | 25 °/s |

**这道缝本来就在，我的改动只是把它照出来**：扇区还在的时候，长扫掠根本不存在，靠近边界时逐轴治理器也会自动降速——**"没人超速"一直是包线在偷偷代劳的**。包线一拆，代劳没了，守卫立刻接管。
⇒ **修法 = 把"求值"收到声明底下**（roam 扫的是 yaw，就按 yaw 自己的轮廓限速；`l.roam_v_max = motion_profile(yaw, AutoRoam).target.speed`），
**不修 = 不动守卫**：把 25 抬到 30 就叫"把门柱往后搬换一条绿日志"，而那个上限是他的参数。
剩下的缝（守卫的 25 是硬编码、`motion_speed()` 的跨轴 max 用在 manual/track 上也同样可疑）**留给他拍板**，见文末。

### 我自己写的东西里被抓出来的两个缺陷（都是读了现场文件才看见的）

1. **跳闸文件不是合法 JSON**：`"track":"search,"phase"`——少一个收尾引号。
   我的测试当时**通过**了，因为它只查"键在不在"。⇒ 测试改成钉**相邻关系＋引号配平**。
   **记这条的理由**：`find("\"track\":\"")` 这种断言是装饰品，它恰好放过了真实发生的坏法。
2. **遥测的形状：当场没跟上，第二天补上了**——原来 `q_soft_min_yaw_rad/q_soft_max_yaw_rad`
   在无包线时报 **0/0**，`soft_limit_distance_yaw_rad` 报 **−1**（我自己的哨兵）。readiness 与
   判断都对了，但**dashboard 照 0/0 会画出一个零宽包线**，而且 `null < 0.05` 在 JS 里是真的
   ⇒ "没有边界"会被点成"贴着边界"。现在：三个字段发 `null`，加一个 `yaw_envelope:"none"` 的词。
   **补的时候我又错一次，而且是他在真机上看见的**：快照问的是 `declared()`——而 `declared()`
   对 `Unbounded` **按设计就是真**（"没有边界"本身就是一个立场），于是无界 yaw 报回 `sector`，
   那个 −1 也当数字发了出去。**"声明过"与"有边界"是两个问题**，线上要问的是后者（`unbounded()`）。
   另有教训一条：我那个单元测试是**手设字段**跑的，所以生产端问错谓词它看不见——
   **对结构体字段的断言不等于对填它那道缝的断言。**

### 现在的立场

`position_envelope: none` **保留**（它做到了要做的事，而且守卫一次都没被放松）。
两件事等主人：① **25 °/s 那个硬编码守卫**要不要读站文件（我倾向读，但那是安全参数）；
② 无包线之后 AUTO_ROAM 的**扫掠区域**默认取 `search_span`（现在 ±37.1°，日志里可见），要不要单独给一个键。

### 补：我把"没有再跳闸"错当成"修好了"，被抓第三次（07:0x）

按 yaw 轮廓限速之后，`speed_over_ceiling` 确实不再出现——**但我没有只看这一条**。读状态读到的是：

```
operating_mode AUTO_ROAM   phase hold   q_yaw +41.6°   ← 12 秒不动
intent_has_joint_target True  intent_q_yaw −41.6°      ← 意图明明在对面那条边
roam_progress 0.0008                                   ← 0.08%，不涨
```

**根因是我自己写的那一行**：我拿 `motion_profile(yaw, AUTO_ROAM).target.speed` 当上限，
而站文件**没给 yaw 写 AUTO_ROAM 的 target** ⇒ 上限 = 0 ⇒ **扫掠被一个"数字"瘫痪了**。
守卫没响，是因为**根本没动**——**"没跳闸"和"在干活"是两件事**；今晚第三次撞上同一句教训
（前两次：`reference_invalid` 多报、ready 检查查错端点）。
⇒ 上限改取**该轴声明的最大速度**（`maximum.speed`，解析器保证 > 0），并且 **0 一律视作"没有这个上限"而不是"限速为零"**。

### §6 第 4 步「现场四验」的结账（07:0x，release `168d048`）

| 验收项 | 结果 | 凭据 |
|---|---|---|
| 手动跨 ±180°，会话角不失真 | **PASS** | jog −0.3° → **+274.1°**，单调、无 fault |
| 无包线仍点亮 readiness | **PASS** | 部署的就绪轮询**点名等** `soft_limits_valid` 并通过 |
| 扫掠在新区域真的动起来（跨 ≥30°） | **PASS** | AUTO_ROAM 期间 yaw **移动 38.4°**（32.2 → 70.6），无跳闸——修零上限之前是 **0°** |
| 跳停/停止证据不变 | **PASS** | 部署重启留 `cause=operator_stop`；06:41 那次跳闸留 `BRAKE`＋black-box scene 1 |
| **扫掠在两端都掉头**（一轮完整来回） | **NOT_RUN** | 单腿跑通即被 vision 的 `target held for 50 ms` 自动交接进 AUTO_TRACK；要一个**无目标窗口**才验得到 |
| 遥测把"无包线"说成 `null` | **未做（跨三层）** | 见 §10 与 TODO 第 3 条：`protocol.py` 字段是 `float = 0.0` |


---

## 后补（09-28 中午，主人确认与改动）

- **滑环无限制**（自由移动、不计圈数）⇒ 本文所有以"线缆会绕"为名的保守自动作废；无限制 yaw
  剩下的唯一论据是我们自己的会话角与其它软件判据。
- **travel tape 回来了，而且改成滑尺**：光标定在中央不动，刻度在它下面滑（战斗机/直升机 HUD 的
  读法），两端渐隐而不是硬切。窗口宽度取**该轴自己的 effective FOV**（与安全包线多边形同一对数），
  不再用我随手取的常数；FOV 没报时用 1/4 行程兜底，`windowSource` 字段如实说明用的哪一种。
  值框下面那两行说明文字（"JOINT TRAVEL, NOT HEADING" / "0 = TRAVEL MIDPOINT"）按主人要求删掉。
- 上限从"测量超速就断电"改成"对请求夹紧"；manual 两轴速度对齐。理由与全部断电入口见
  `ADR-001/reports/AUDIT_POWER_REMOVAL_2026-09-28.md`。

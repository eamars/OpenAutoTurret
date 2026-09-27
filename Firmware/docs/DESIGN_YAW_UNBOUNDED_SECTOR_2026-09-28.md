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
`soft_limit_distance_*_rad` 与 trace 的同名字段 ⇒ **`null`**（与 `temp_raw=-1` 同一套"缺席要说缺席"的纪律）。

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
| 8 | 遥测/trace 3309 | 距离（0＝贴界） | `null` | 现场拉一行原文 |
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

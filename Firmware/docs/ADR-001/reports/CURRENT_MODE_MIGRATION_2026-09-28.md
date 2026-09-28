# GM6020 yaw：电压 → 力矩电流 迁移（进行中）

任务来源：主人转来 ChatGPT 的任务书（2026-09-28 晚）。本文是**进度与施工图**，不是完成报告。
今晚的动机：yaw 带滑环、阻力随角度不均匀，电压模式给不出需要的**力矩**；电流模式由驱动器内部闭环，
才是这个问题的正道。**但电流模式的前置条件我们至今没验证**（见"未验证"）。

## 已完成（本地可证，已跑）

**协议层** `control/src/can/gm6020_protocol.hpp`：

- `current_raw_uncapped(double amps)` —— 纯换算 `raw = round(A / (3.0/16384))`，非有限值抛、越 ±16384 抛。
- `current_raw_from_amps(amps, limit_a)` —— **安培单位下先钳位、再编码**；`limit_a` 必须有限且 >0，
  且 **> 1.62 A（DJI 连续额定）直接拒绝**：软件不许自己把常态上限抬上去。
- `current_frame(motor_id, amps, limit_a)` —— **专用的电流帧构造器**（不挪用 `voltage_frame`）：
  `id=0x1FE`、标准帧、`dlc=8`、ID 1 占 `DATA[0:1]` 大端有符号、`DATA[2..7]` 保持 0；**只对 ID 1 资格化**，
  `motor_id != 1` 抛（第二台是假设，不是事实）。
- 常量：`kRawFullScale=16384`、`kAmpsFullScale=3.0`、`kMaxContinuousA=1.62`、`kAmpsPerRaw`。

**测试** `control/tests/test_gm6020_transport.cpp` 新增 6 例（`ctest -R gm6020` 绿；全量 **77/77**）：
0x1FE/标准/DLC8/未用槽位为 0；`0.8 A → 4369 → 0x1111`；`-0.5 A → 0xF555`（二补）；
端点 `0/±3 A → 0/±16384`；**钳位先于编码**（2.0 A @ 0.8 A 上限 == 0.8 A）；
NaN / 上限 0 / 上限负 / 上限 3.0 / ID 2 全部抛（失败关闭）。

## 未做（下一场照此施工，顺序即依赖顺序）

1. **配置** `mixed_hardware_profile.{hpp,cpp}`：`ControlMode` 加 `Current`；解析/校验
   `control_mode: current` 必须配 `command_frame_id: 0x1FE`、`feedback_frame_id: 0x205`，
   且**电压/电流命令 ID 不得混用**；要求**显式外部配置确认位**，建议
   `current_ring_verified: true`（缺省即失败关闭，日志必须打印"未确认 ⇒ 电流模式未启用"）。
   新增主机侧电流上限（安培、有限、>0），**初始标定值 0.8 A**（不是协议满量程 3 A）。
   `turret_mixed.yaml` 里 yaw 的 `limit_cur_a: 0.0` 语义改为**主机命令钳位**（旧注释"GM6020 无软件
   CurrentLimit 寄存器"作废），并写清单位。
2. **backend** `mixed_can_motor_backend.cpp`：yaw 速度环输出语义从"raw 电压计数"改为**明确标单位的
   电流努力量**（`kYawOutputCeiling/kYawVelocityKp/kYawVelocityKi` 是电压单位的产物，
   **不得原样挪用**冒充电流增益；改名 + 独立可配 + 标注"待实物标定"）。
   保留全部现有语义：位置环、加速度成形、连续角/会话参考、反馈新鲜度、CAN 健康、速度守卫、
   无进展守卫、温度守卫、心跳。**每条归零路径改发电流 0**（启动 burst、`send_yaw_zero_locked()`、
   trip/fault、`close()`、正常关机、探针清理），并把日志措辞从 "zero voltage requested" 改成
   **"zero current requested"**，同时保留 **"disable state unavailable"**（不得写成 de-energized）。
3. **探针/启动器**：`--yaw-voltage` 不得"顺手"当电流解释；加 `--yaw-current-a`；旧电压探针若保留，
   标注 legacy 并在 profile 选 current 时拒绝误用。`probe_yaw_motion` 记录**命令电流**（不是
   `voltage_raw`），行程/速度边界、CAN 健康、反馈新鲜、心跳、末尾静止核验、**单一 CAN 拥有者**全部保留。
4. **回归测试**：编码器解码/解卷不变；`0x205` 反馈解析不变（**`current_raw` 的工程单位换算仍未验证，
   不得换算成安培**）；CyberGear pitch 不变；mixed 总线所有权不变。

## 未验证（不许被"代码写了"冒充）

- **固件 ≥ v1.0.11.2** 与 **Current Ring 已使能**：本仓库**从不读设备固件版本**（`gm6020*` 无版本查询），
  也没人记录过。**这两项必须由操作者确认并落进配置**，否则电流模式不启用。
- 因此"电流模式能解决今晚的哼唧"仍是**假设**；今晚已证的只有：旧 9000 电压上限推不动、抬到 15000 能动。
- `0x205` 反馈 `current_raw` 的单位/刻度：仓库现按"未验证"处理，本次迁移不引入换算。

## 硬件标定边界（顺序不可颠倒）

固件确认 → Assistant 开 Current Ring → 配置记录确认位 → 停 controld（保证单一 CAN 拥有者）→
先只做**接收**验 `0x205` 与总线健康 → `0x1FE` 打一次极小 bounded 电流 / 零 / 反向 / 零 →
核对编码器方向、响应、零电流静止、无 CAN 错误 → 才跑 bounded 行程标定测试并整定电流模式 PI。
**首轮远低于 0.8 A**，不用 ±3 A。

建议的探针调用形式（**待实现，不是已可用命令**）：
`Firmware/tools/probe_yaw_motion --yaw-current-a 0.2 --travel-deg 30`（其余边界沿用现有默认）。

## 配置层补丁契约（本轮实地读到行号，下一刀照抄即可）

| 站 | 位置 | 要做什么 |
|---|---|---|
| 1 | `control/src/config/mixed_hardware_profile.hpp:12` | `enum class ControlMode { Voltage, Position, Speed }` → 加 `Current` |
| 2 | 同文件结构体内（`control_mode` 字段旁，约 :25） | 加 `bool current_ring_verified = false;` 与 `double host_current_limit_a = 0.0;`（**缺省即失败关闭**） |
| 3 | `mixed_hardware_profile.cpp:186` 的**已知键白名单** | 加 `"current_ring_verified"`、`"host_current_limit_a"`——**漏这一步会被"未知键"拒绝**，是整刀最容易漏的站 |
| 4 | `cpp` 的 **yaw 分支**（约 :150-170，pitch 分支的模式映射在 :194-197 可作形状参考） | 接受 `"current"`；选 current 时校验：`command_frame_id == 0x1FE`（电压/电流 ID 不得混用）、`current_ring_verified == true`、`0 < host_current_limit_a <= 1.62` |
| 5 | 校验失败的信息 | 要**点名缺哪一项**（红要带原因）：例如 `axes.yaw: current mode requires current_ring_verified: true (firmware >= v1.0.11.2, RoboMaster Assistant v2.7+)` |

**注意别顺手做的事**：YAML 里生产仍是 `control_mode: voltage`——**固件与 Current Ring 未验证前不切**，
所以本次改动**不改变任何现有行为**（新键只在选了 current 时才生效），这也让它可以安全地先合进去。

**验收形状**（与 WP3/WP4/WP5 同一套路）：profile 测试里加"选 current 但缺确认位 ⇒ 被拒且信息点名该项"、
"上限 1.63 ⇒ 被拒"、"`command_frame_id: 0x1FF` 配 current ⇒ 被拒"三条，**每条都得能红**。

## 本轮实地做掉与查到的

**已改（惰性、已验证）**：`mixed_hardware_profile.hpp:12` 的 `ControlMode` 加 `Current`；结构体加两个
**失败关闭**字段 `current_ring_verified = false`、`host_current_limit_a = 0.0`。
构建干净、**77/77**。**没有产生任何 unhandled-switch 警告** ⇒ 全仓库不存在对 `ControlMode` 的穷尽 `switch`，
所以编译器不会替我列清单——**站点得自己找**，这就是一定要读代码、不能靠加枚举蒙混的原因。

**我错了一次并当场纠**：我一度怀疑 `control_mode` 除解析器外无人读（"配置里的模式是装饰品"）。
grep 证伪：`control/src/control/mixed_can_motor_backend.cpp:76-82` 就读它。

**Layer 3 的靶心（精确到行）**：`mixed_can_motor_backend.cpp:76-82` 是一道**准入合取**——
`protocol==Gm6020 && bus=="yaw" && motor_id==1 && topology==Continuous &&
control_mode==Voltage && feedback_frame_id==0x205 && command_frame_id==0x1ff`，
否则 `err = "mixed profile yaw must be GM6020 ID 1 with continuous voltage control"`。

⇒ 电流模式要进来，改的就是这一处：允许 `Current` 时把 `command_frame_id` 要求换成 **`0x1fe`**，
并**在此处**追加 `current_ring_verified` 与 `0 < host_current_limit_a <= 1.62` 两项检查；
错误信息也要一分为二（电压说电压、电流说**缺的是哪一项**），否则"没开 Current Ring"会伪装成
"你的 profile 形状不对"。

### 解析器站点 3＋4 已试、已回退（本轮，带证据）

试了就把红跑出来：把 `"current_ring_verified"`/`"host_current_limit_a"` 加进
`check_keys(axes["yaw"], ...)` 的列表之后，**`test_mixed_station_config` 立刻红**——
它用一份缺 `guard_temp_raw_ceiling` 的 YAML 断言"第一条错误点名该项"，而新键**也变成了必填**，
于是第一条错误改点名新键。**这说明 `check_keys` 那份列表同时承担两件事：拒绝未知键 **和** 要求全部存在。**
⇒ 出厂 `turret_mixed.yaml` 里没有这两个新键，若照那样合进去，**整份站配置会变成"缺键"**。

正确做法（下一刀）：新键必须**可选**——要么给 `check_keys` 一份"允许但可缺省"的第二列表，
要么在读取处用显式默认值 + 单独的未知键豁免。**不能靠把键塞进必填列表蒙过去**，那是把兼容性弄坏。

顺带两个自我纠正：
- 我一度把"构建失败"读成"ctest 77/77 所以没事"——**ctest 在旧二进制上照样全绿**，
  这条陷阱我自己上一轮刚点名，这一轮就踩到了；已改成用 `grep -c "error:"` 判构建。
- 插入点也错了第一次：分支引用 `yaw_axis` 却插在 `auto& yaw_axis = ...` **之前**。

### 更正：上一轮我对那次红的**因果解释是错的**

我写的是"`check_keys` 那份列表同时承担未知键拒绝**与**必填要求，所以新键变必填 ⇒ 出厂 YAML 会缺键"。
读了 `mixed_hardware_profile.cpp:19-38` 才知道不成立：**`check_keys` 只遍历 YAML 里出现的键、
只报 `has unknown key`，它不要求任何键存在。**

所以那次 `test_mixed_station_config` 的红**原因未定**（`NOT_VERIFIED`），现场证据留在这里：

```
Value of: loaded.ok     Actual: false   Expected: true
Expected: (std::string::npos) != (missing.errors[0].find("guard_temp_raw_ceiling"))
```

两个断言同时出现，说明**至少有一个本应加载成功的 fixture 变成不 ok**，并且**缺键用例的第一条错误**
不再点名 `guard_temp_raw_ceiling`。**首要嫌疑是站点 4**——我把 `check_axis_string(...,"voltage",...)`
换成 `string_value(...)` 加分支，并删掉了紧随其后的硬编码赋值；嫌疑点是 `string_value` 对缺失/类型
的处理与原检查不同，或我删除赋值的那条正则**误删了他处**。**不再凭想象补因果**——下一刀从
"只上站点 3（白名单）"与"只上站点 4"分别单独跑一次开始，两步分离就能定位。

这也是今晚反复出现的那个模式：**看到红就急着给解释**。红先复现、再二分，因果要跑出来。

### 二分结果：罪魁是站点 3；我第 50 轮的判断成立，第 51 轮的"更正"作废

**做法**：只上站点 3（白名单加两键）→ 构建 0 error、**ctest 1 红**，错误原文
`axes.yaw.current_ring_verified is required | axes.yaw.host_current_limit_a is required`。
⇒ 不需要站点 4 参与就能复现，**因果由二分给出，不再靠猜**。

**机制**（读全了，`mixed_hardware_profile.cpp:19-48`）：`check_keys` 先拒绝未知键，
**再遍历 `allowed` 要求每个键存在**（第 41-46 行，报 `X is required`）。
所以那份列表**确实同时承担两件事**——这正是我第 50 轮写的；第 51 轮我**只读了 19-38 行**
就宣布"它不要求存在"，于是把对的判成错的。**教训：读函数要读到 `return`，别按半截实现下结论。**

**修法（本轮已落地，构建 0 error、77/77）**：`check_keys` 增加第 5 个参数
`optional = {}`——**默认空 ⇒ 既有调用点一字未改**；未知键判定把 optional 也算已知；
存在性检查仍只看 `allowed`。yaw 处把 `current_ring_verified` / `host_current_limit_a` 放进 optional。
⇒ 新键合法、可缺省，**出厂 `turret_mixed.yaml` 不会因为这次改动变成"缺键"**。

**还剩**：站点 4（`control_mode: current` 的解析）——它与这次红**无关**，可以单独做。

### 三条用例为什么还没写：**测试面不存在**（本轮查清，不是拖延）

- `grep 'validate_profile' control/tests/*.cpp` **零命中**——不是没人想起来，是**调不到**：
  `mixed_can_motor_backend.hpp:140` 起是 `private:`，**`validate_profile(...) const` 在 :149**，
  而调用它的路径要打开 CAN 套接字 ⇒ 在容器里只能等到真机。
- 于是"门无人守"是**结构问题**，修法按优先级：
  1. **把 yaw 的准入规则抽成配置层自由函数**（例如 `config::mixed::validate_yaw_axis(profile, err)`），
     backend 与测试都调它——**规则回到解析器旁边**，成为"清单级不变量"，测试面自然出现；
  2. 或把 `validate_profile` 提为 `public static`（最小改动，但把一个实现细节为测试外露）；
  3. `friend`（最不推荐：测试与类的私有布局焊死）。
  **我选 1**：它同时解决"这道门只能真机验"的问题，而 2 只是让红能出现。
- **本轮实测到的可复用件**（下一刀写测试直接用，别再造 fixture）：
  `control/tests/test_mixed_station_config.cpp` 里有 `firmware_root()`、`why(loaded)`、
  `config::mixed::load_mixed_hardware_profile(path)`，混合硬件清单文件是
  **`Firmware/config/mixed_hardware.yaml`**（不是 `turret_mixed.yaml`——后者是运行时总配置）；
  `:95-109` 已在断言"出厂拓扑钉死为 Voltage / 0x205 / 0x1FF"，**这正是我要防被误删的那条**。

## 22:3x 复测（主人刷完固件、站新址 **192.168.2.103**、`controld=0` 我是唯一 CAN 生产者）

同一套双向对照（`/tmp/probe_current.py`，反馈 0x205 每秒 1001 帧）：

| 窗口 | 帧数 | 角度 Δ | 转速 | 反馈电流 raw（min,max） | 均值 | 带宽 |
|---|---|---|---|---|---|---|
| 静置基线 1.0 s | 1001 | 0 | 0 | +126, +179 | **+147.0** | 53 |
| **+0.40 A** 1.2 s | 1201 | 0 | 0..−1 | +119, +175 | **+147.7** | 56 |
| 归零 0.6 s | 601 | −1 | 0..−1 | +130, +179 | +149.8 | 49 |
| **−0.40 A** 1.2 s | 1201 | 0 | 0..−1 | +126, +179 | **+148.9** | 53 |
| 归零 0.6 s | 601 | 0 | 0 | +126, +179 | +150.0 | 53 |

**判定：电流帧仍被忽略。** ±0.4 A 三个窗的反馈电流**统计上同一条**（均值差 <3 raw、带宽同量级），
角度与转速不动。**判据用的是符号反转**——单向巧合不算，正负两向都不动才算真不动。

**两条我已排除、不再当解释挂着：**
1. **字节布局错**：`0x1FF` 电压帧按本仓参考文档同为**四槽位**（bytes 0-1/2-3/4-5/6-7 = ID 1-4，无模式字节），
   而我今晚**正是用这一布局把轴推动过** ⇒ 对称布局的 `0x1FE` 不是假阴性。
2. **帧没上总线**：`ip -details link show can0` = **ERROR-ACTIVE，berr-counter tx 0 rx 0**。
   无应答帧会立刻产生 ACK 错误并推高错误计数 ⇒ **我发的 600+ 帧被驱动 ACK 了**，它收下但不执行。

**剩下的唯一解释（待主人确认）**：按 DJI 指南，**电流环不会因为"发了像电流的帧"就生效**——
需要固件 **≥ v1.0.11.2** *并且* 在 **RoboMaster Assistant ≥ v2.7 里把 "Current Ring On/Off Switch" 打开**。
主人今晚报的是"已升级到最新版、支持电流环"，**这是否包含了那个开关，我不知道**（记为待问，不当已答）。

**另一件顺带事实**：`dmesg` 显示这台 Pi **约 8 分钟前重启过** ⇒ 换 IP（.100→.103）是重启换了 DHCP 租约，
**部署脚本的 `--connect-address` 必须跟着改**，已写进 skill。

## 00:2x 定案：**驱动只吃电流帧，不吃裸电压帧**（站已停，未重启；controld=0，我是唯一发送者）

同一轮里两类帧交错发送，各自做符号反转（`/tmp/probe_both.py`）：

| 帧 | 角度 Δ（字） | 驱动回报电流均值 |
|---|---|---|
| 静置不发 | 0 | 0.0 |
| **电流 +0.40 A** | **+306**（≈13.4°） | **+2160**（命令 +2185，差 1%） |
| 电流 0 | −5 | +4.5 |
| **电压 +8000** | **0** | **0.0** |
| **电压 −8000** | **0** | +0.3 |
| **电流 −0.40 A** | **−280**（≈−12.4°） | **−2159** |
| 静置不发 | +8 | −515（收尾瞬态） |

**判据：正负都动且方向随符号反转 ⇒ 电流帧生效；电压帧两个方向皆 Δ0、反馈电流与静置同一条 ⇒ 不生效。**

**完整因果链（每一环都有独立证据）**：
助手写进电调的参数使驱动不再接受裸 `0x1FF` → controld 仍发 `0x1FF`（**实测 198 帧/秒，负载 0..15000，
持续咬在 13.4k**，即我们配的天花板）→ 驱动回 0 A / 0 rpm → HUD 里"yaw 推不动"。
⇒ **机械与接线无罪**（卡死的形状应是电流顶起而角度不动；此处电流为零）。
⇒ **机制未定**：`PWM模式=位置模式` 被写入是最像的候选，但"为什么 8000 与 −8000 都完全无位移"
   我没有解释，**不编**。新固件里 0x1FF 的语义/量程是否已变（v1.4 文档写 ±25000、旧文档 ±30000）同样待查。

**修它的路线就是本包本身**：yaw 切 `control_mode: current`。配置层已就绪并有 3 条测试守；
**缺 backend 的安培语义（layer 3）与探针 `--yaw-current-a`（layer 4）**。
⇒ **今晚起这条不再是"以后要做的优化"，而是现网 yaw 动不了的唯一病因。**

**我这边被推翻的三条（都记着，别抹）**：
① 我把他手推的 jog 当成跟踪成绩；② 我把 `TARGET_UNREACHABLE` 解释成载荷帽追不上；
③ 我先说"你的设置无罪"，实际驱动侧参数正是病因——**只有"你手没拆坏"这一条从头到尾成立。**

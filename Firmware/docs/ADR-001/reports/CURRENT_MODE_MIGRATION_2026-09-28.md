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

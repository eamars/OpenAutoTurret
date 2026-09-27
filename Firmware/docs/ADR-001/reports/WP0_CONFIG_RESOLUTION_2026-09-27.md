# WP0 · 启动配置解析报告（2026-09-27）

**本切片解决的问题**：按包要求补录 HEAD、工作区状态、实际激活 release 与解析后的
profile/config；查清 `axes` × `motion.modes` 覆盖规则、yaw 符号、pitch 激活驱动模式、
两层 PI 是否叠加；报告当前真实 suite 的执行结果。
**运行时改动：零。** 本报告与新测试之外未动任何生产文件；未做 `--activate`、未启动、
未停止、未向电机发过任何新命令。站台全程保持 18:05 起既有的 MANUAL/HOLD。

时间基准：本地 Pacific/Auckland（NZDT，UTC+13）；括号内 UTC。

---

## 1. 仓库基线（本地）

| 项 | 值 |
|---|---|
| clone | `git@github.com:eamars/OpenAutoTurret.git`，本地 `repos/OpenAutoTurret` |
| 分支 | `ADR-001`（自 `main` 切出） |
| HEAD | `49bc169aa53662cb0ac1a592f349f3d81838add5`（Merge PR #1 codex/hardware-adaptation） |
| 工作区 | 本切片前 clean |
| 本机工具链 | g++ 14.2 在；Python 依赖（requirements-station + pytest/matplotlib/pyserial）已装 ⇒ **Python suite 已跑**（§7）；cmake / GTest / yaml-cpp / spdlog 系统包**装包在途** ⇒ C++ suite 记录时 NOT_RUN（§7） |

## 2. 站台实际激活 release 与出处身份（现场，全部只读取证）

| 项 | 值 | 身份 |
|---|---|---|
| 进程 | `controld <release>/Firmware/config/turret_mixed.yaml`（argv[1] 实测） | 当前运行 |
| release 目录 | `run/releases/f9aa2dc44a14.3TaYdS`（`deploy_station.py` 命名：12-hex 前缀 + 随机段） | 当前运行 |
| release 前缀出处 | `git cat-file f9aa2dc44a14` 在 Pi checkout 中 **unknown revision** | **UNVERIFIED**——不能据此 release 目录名宣称其对应某 commit |
| Pi 私有 checkout | HEAD `6a47f1d`（2026-09-20，"Add v2 CAD"），dirty；其 `Firmware/config/` 内**没有** `turret_mixed.yaml` | 与分支 tip 不同代 |
| 交接书记载 | "最近实际激活 `8901808`，欠压后停止" | **已过时**：站台 09-27 18:03（05:03Z）受控重启并完成全流程（§5） |
| 运行中 config sha256 | `turret_mixed.yaml = 237b68249363…`、`mixed_hardware.yaml = b84723cd9c4f…` | 内容哈希，解析基准 |

**三方身份不一致的结论**：分支 HEAD（`49bc169`）≠ Pi checkout（`6a47f1d`）≠ release 可证出处（无）≠ 交接口径（`8901808`）。包警告"不要假定 HEAD 就是激活版本"成立且比预想更强。

运行 config 与 HEAD 的 `turret_mixed.yaml` 归一化后**仅两行差异**：

- `motors.yaw.direction_sign`：live `-1`（注 "live yaw+ turned camera left"）vs HEAD `+1`（注 "GM raw+ and yaw+ aim camera left"）；
- `motors.pitch.direction_sign` 的注释措辞。

两值在 mixed 路径**均不被消费**（§4.2），故该差异是**文档身份漂移**，不是行为漂移。

## 3. 解析链：谁在启动时读了什么

```
run_application.sh（MODE=hardware）
  └─ DEFAULT_CONTROL_CONFIG=config/turret_mixed.yaml（OTA_CONTROL_CONFIG 可覆盖；本次现场未设）
      └─ controld argv[1] → load_turret_config()           # turret_config.cpp
           ├─ hardware_profile: config/mixed_hardware.yaml → load_mixed_hardware_profile()
           │    └─ validate_profile()：必须 can0/spi0.0 + can1/spi1.0 @1Mbps；
           │       yaw=GM6020/ID1/continuous/voltage（feedback 0x205, command 0x1FF）；
           │       pitch=CyberGear/ID127/bounded/position|speed；expected UID 校验；
           │       current_limit_a ≤ 5 —— 违反即拒绝启动
           ├─ payload.active_profile=conservative → payload_profiles/conservative.yaml
           └─ 缺键 ⇒ 内置保守默认 + warning（现场启动日志逐条打印，本次 17 条，
              例：tracking.search_span_deg→45.0、v3.auto_track.*→0）
```

**现场启动日志摘录**（`/tmp/ota-stack-1000/controller.log`，18:03:53–18:05:23）：
`boot OK: pitch uid=0x7216313130333105`（与 expected_unique_id_hex 逐位一致）；
`payload profile: loaded 'conservative' (v_max pitch=20.1 deg/s, yaw=20.1 deg/s)`；
每模式每轴打印 `target/maximum … (before axis/payload/intent/boundary caps)`——即解析器
自己声明还有四层运行时封顶（§4.1）；pitch 归零 18:04:45 完成；AUTO_ROAM→AUTO_TRACK→
（1000ms 丢失）→AUTO_ROAM→ 18:05:16 `STOP_MOTION … controlled hold in MANUAL`（人工
覆盖，§27 语义），18:05:23 一次 `MANUAL jog yaw+`。当前 `phase=hold / MANUAL / safety=ALLOW`。

## 4. 四个重点问题

### 4.1 `axes` × `motion.modes` 覆盖规则（单位：文件 deg/s、deg/s²、deg/s³；内部 rad）

解析（`turret_config.cpp::parse_motion`）：`motion.modes.<mode>` 生成两轴同值 profile；
`motion.modes.<mode>.axes.<axis>` 为**整块** per-axis 覆盖（`maximum`+`target` 必须成对，
生产文件**未使用**该覆盖）；顶层 `axes.<axis>.max_*` 单独存入 `motion.axis_maximum`。
静态校验：`target ≤ maximum ≤` 服务包络 20/30/120（`motion_profile.hpp` 常量）。

运行时（`control_loop.cpp::motion_profile()` → `resolve_motion()`）：

```
effective.maximum = mode.maximum ∩ axis_maximum ∩ payload ∩ derate   （逐分量 min）
effective.target  = mode.target  ∩ effective.maximum                   （derate 只缩 propulsion）
```

`/api/state` 的 `motion_profile.{configured,effective,limit_reason}` 是该交集的现场输出。
**今日 MANUAL/HOLD 实测**：yaw effective target = **10°/s**（被 `axes.yaw.max_velocity` 从 20 截下），
pitch effective target = 20°/s（`axes.pitch=30` 不截；payload conservative 0.35 rad/s≈20.1°/s 不严格小于 20，不截）。

**AUTO_TRACK 解析值**（静态推导，与上式一致；运行时快照将在 motion 窗口复核）：

| 轴 | mode 请求 target | axis_maximum | effective target |
|---|---|---|---|
| yaw | 20 / 30 / 100 | 10 / 15 / 60 | **10 / 15 / 60** |
| pitch | 20 / 30 / 100 | 30 / 60 / 300 | **20 / 30 / 100** |

⇒ 基线疑虑成立：**auto_track 名义 20°/s 对 yaw 永远兑现不了**（10°/s），pitch 名义 20°/s 受
保守 payload 与"载荷下全速制动未验证"双重限制——包络上限，不是保证。

### 4.2 yaw 符号

`motors.*.direction_sign`：解析器强校验 ±1（`turret_config.cpp:351`），但 **mixed 路径无消费者**
（grep 全 control/src：唯一读者是 `logical_coordinates.hpp` 的 `MotorModel`，其符号由归零端点
`setup_model_from_endpoints` 或 `homing_plan` 赋 1）。连续 yaw 的逻辑位置 =
`yaw_encoder_.relative_rad() - yaw_origin_rad_`（`mixed_can_motor_backend.cpp:277`），**无符号乘法**：
raw+ ≡ 逻辑+，固定。live 文件里的 `-1` 是 legacy 字段残留。
**"yaw+ 相机朝左"这一物理断言两份文件口径一致但未现场复核**——需一次受控 jog（本切片不做运动）。

### 4.3 pitch 激活驱动模式

- 声明：`mixed_hardware.yaml control_mode: position`（validate_profile 允许 position|speed）。
- 运行时真相是**生命周期状态机**：`v3.service_speed_control=true` ⇒ 服务运行于 **RunMode=2 速度模式**
  （`transition_mode` 写 RunMode 后逐寄存器读回验证；MIT 与 IqRef 写被 TX 闸门直接拒绝，
  `cybergear_system.cpp:343-349`；`LimitCur ≤5A` 在最后的公共 TX 边界再拦一道）。
- 现场 RunMode 寄存器**直接读回 = NOT_RUN**（只读探针需与 controld 共存的窗口，属 WP2 现场项）；
  间接证据：状态机路径 + 反馈帧 mode=2 + 今日归零/服务全程。
- `mixed_hardware.yaml control_mode` 与 `turret_mixed.yaml v3.service_speed_control` 是**两个文件
  描述同一事实的两张皮**（声明 position、运行 speed），报告如实登记，本切片不改写。

### 4.4 两层 PI 是否叠加

**不叠积分。** 级联关系：

- **宿主层**（`speed_servo.hpp`）：位置 P + 速度前馈 + jerk/加速度整形，**明确无积分器**；
  P 增益 = `v3.position_servo_kp`（4.0，代码钳 [2,6]，`control_loop.cpp:2047`）。
- **驱动层**（CyberGear 内部速度环 PI）：增益 = `SpdKp(0x701F)/SpdKi(0x7020)`，由
  `v3.service_speed_kp/ki`（4.0/0.05）经 `transition_mode` 写入并读回；`adopt_running_mode`
  在开机收编时**只比对不写入**，不符即拒启（fail-closed）。

⇒ 全链路唯一积分器在驱动器内；`service_speed_*` 名字里的 "speed" 指驱动速度环，不是宿主 PI。

**但同一组寄存器有三个写方**（这是本次查出的真实隐患）：

| 写方 | 键 | 现值 | 时机 | 现状态 |
|---|---|---|---|---|
| 归零收尾 | `homing.speed_kp/ki` | 4.0 / 0.05 | homing 完成时 `transition_mode` 写 | **活跃**（今日 18:04 已写） |
| 服务收编 | `v3.service_speed_kp/ki` | 4.0 / 0.05 | 开机 adopt 校验；park 转换写 | **活跃** |
| 载荷自检 | `payload.check_spd_kp/ki` | 5.0 / 0.02 | `set_speed_loop_gains` | **休眠**（`auto_verify=false`） |

今日三对值两两相同或休眠，行为无冲突；**一旦有人只改其中一对**，生效值取决于最后一次转换
（homing 后持 homing 值；下次开机 adopt 若值≠service 值会**拒启**——是响的，不是静默的）。
建议后续 WP 在遥测里带当前驱动增益回读，本切片不改行为。

## 5. capability 清单（身份 = 现在 / 历史 / 待验收）

| 能力 | 现场值 | 身份 |
|---|---|---|
| 控制环 | 200Hz 名义；实测 cycle≈5.06ms、deadline misses=0、CAN err=0 | 现在 |
| 归零状态 | 今日 18:04:45 pitch 全程归零完成 + 会话 yaw 参考建立 | 现在（新鲜） |
| 安装标定 | `installation_calibrated=false`，source=identity | 待验收 |
| payload profile | conservative；捕获注记 **2026-09-02 yousee PHY、hardware 0x7b43…04/0x7811…0a（≠现 pitch UID 0x7216…05）**；`max_verified_speed=5°/s`、`brake: valid=false` | **历史身份、现在生效**——基线"载荷验证不得描述为已有能力"成立：它是旧总线的记录 |
| 载荷自检 | `auto_verify=false` | 休眠 |
| IMU | BNO085 observer ready（generation=0），观察/记录 | shadow，无控制权重 |
| 视觉 | 运行 `--profile person_detect_available`（launcher 传参）；JSON 默认 `"profile": "person_detect"` 是**另一条 YOLO11n 配置** | 解析身份=命令行；基线警告现场复现确认 |
| 停止证据 | 单一 `STOP_MOTION` 事件（18:05:16），无 per-axis 字段 | WP2 对象 |

## 6. 配置键 × 处置（对照 `01_BASELINE.md §3` 参数审计表补"消费点"列的要点）

- `safety.deadline_max_us=2000`：**单周期计算耗时门槛**（"cycle longer than this counts as a miss"），不是 5ms 周期本身。审计问题的答案。
- `homing.contact.stall_velocity_threshold=0.5rad/s` 与 `v_move_threshold=0.04rad/s`：分工见文件内注释（位置进展才是判据），消费点 `station_wiring.cpp::make_homing_plan`。
- `tracking.motor_response_ms=120`、`aim_point.box_fraction(0.50,0.22)`：加载即用；aim_point 是框内分数（body_upper 语义），非检出头部。
- `motion.modes` 在场时旧键冲突即报错（`tracking.*_speed`、`v3.service_max_speed` 等），无静默混用——"单一权威"由解析器强制。

## 7. 测试执行记录

| 套件 | 命令 | 结果 |
|---|---|---|
| ADR-001 包自带 | `python -m unittest discover -s tests`（ADR-001/） | **Ran 107 tests — OK**（仅证包工具与合成样例） |
| 仓库 Python suite（perception / tools / web） | `pytest perception/tests tools/tests web/webd/tests` | **744 passed, 15 failed, 3 errors, 1 skipped, 29 subtests**（清单见下；未分诊——环境缺件与真缺陷混在一起，WP1 起逐项定性，失败≠本切片引入：本切片零生产改动） |
| 仓库 C++ suite（62 个 GTest + probe ctest） | cmake + GTest + yaml-cpp + spdlog | **NOT_RUN（记录时）**：容器缺系统依赖，装包已在途；到位后补跑并回填本节。试编译（本地解包前缀）已验证 configure 全过、`ota_core` 编译 100%，只差链接路径 |
| 本切片新增 `test_mixed_station_config.cpp` | 同上 | **NOT_RUN**（随 C++ suite 一并补跑；语义数值已对照现场遥测与启动日志双重核对） |
| 现场只读探针（RunMode 回读、方向 jog、制动包络） | 需 launcher 许可的运动/探针窗口 | **NOT_RUN**：WP0 不含运动授权 |

Python suite 18 项红/错（2026-09-27 20:1x 本地记录，未分诊）：
`perception/test_model.py::TestFactory`（manifest 与已装 artefact 一致性 ×2，疑与 HEF 缺件相关）、
`perception/test_pipeline.py::TestPipelineFrame` ×2、`tools/test_install_station.py::CheckTest::test_a_consistent_install_passes`、
`tools/test_v3_acceptance.py` ×7（验收工具全组）、`web/webd` dashboard/telemetry 文档映射 ×3、
`web/webd/test_section_20_ledger.py` setup ERROR ×3。

## 8. 尚缺证据

1. release `f9aa2dc44a14` 的 commit 出处（需操作者机器或部署日志佐证）；
2. pitch 驱动 RunMode/SpdKp/SpdKi 的现场**寄存器直读**；
3. yaw 物理方向的现场 jog 确认（"yaw+ 相机朝左"目前是文件断言）；
4. 载荷下全速制动包络（conservative 档案 brake=invalid）；
5. 仓库 C++ suite 的当前通过清单（Python suite 已录，§7；C++ 待系统依赖到位后回填本节）。

## 9. 回退办法

本切片 = 本报告 + 一个新测试文件（glob 自动注册，无 CMakeLists 改动、无生产代码/配置改动）。
`git revert <WP0 commit>` 即完全回退；站台不受任何影响（未部署、未重启、未发命令）。

# 05 · Codex任务、回归与验收

原则：四个必做PR，每个能够独立说明改动/测试/回滚。以下是任务边界，不要求创建同名抽象类或重写整套控制器。

## PR1 — 基线、单一输出和可恢复错误（先做）

涉及现有`mixed_can_motor_backend.{hpp,cpp}`、`gm6020_velocity.hpp`、`control_loop.cpp`和最少测试/遥测代码。

- 固定HEAD、记录有效配置/actual mode/实际payload来源。拒绝将旧双CyberGear档案视为新机构测量结果。
- 增加每轴最终命令/原因与真实TX证据。复用现有窗口，不新建服务。
- 消除no_progress guard与普通控制交替电流；将三次tick改为真正episode含义，性能事件不触发默认掉电。
- 统一当前CAN健康/错误增量；按轴保留本轴健康信息；修正仍新鲜反馈下单次20ms迟到的恢复。
- 独立watchdog保留；紧急接管压住旧普通输出。检查`deenergize_all`调用原因，健康承重轴优先受控支撑，不笼统删所有停机。

**回归**：tick!=episode、mock两writer竞争、emergency后的旧命令、历史CAN非零但恢复、持续BUS_OFF/反馈过期、一次TX失败与持续TX失败、dt迟到/倒退/NaN分别处理、yaw异常不无故释放健康pitch、明确急停仍生效。

**回滚**：保留修复前代码作为审计对照；如PR失败不自动将已知输出竞争重新部署到带载站台。使用上一个已经物理批准的版本/配置或保持停机待修，记录current-mode兼容性。

## PR2 — Yaw局部闭环与摩擦补偿

涉及`gm6020_velocity.hpp`、mixed profile解析、必要encoder estimator；保持200Hz与0x1FE。

- typed单位、最终输出抗饱和、补偿切换无扰；有限三态摩擦补偿，未校准参数默认禁用。
- 起动用明确移动意图；正常quiet hold不触发boost、不清除必要积分；反向先制动后切方向。
- fresh-RX速度估计A/B；最终窗由数据选，不强制多滤波器并存。
- 有限起动失败不反复冲击；性能标记与业务模式解耦。
- 首批保持0.8A/pitch5A，不把提升电流作为通过离线测试的前提。资格完成后可开放更高已批准yaw能力。

**回归**：正反对称/不对称补偿、量化/重复RX/回绕、站立正常不boost、方向迟滞、breakaway结束I接管、限流/限斜率后的antiwindup、参数NaN/无穷/负限值拒绝、热cap降低仍保留安全制动能力、越界不允许boost。

**实机**：仅PR1通过后按调试协议测量；给出前后同工况起动延迟、匀速误差、过冲、RMS/温度，不只贴“电机能动”。

## PR3 — Pitch原生速度环、保持和归零

涉及`can_motor_backend.cpp`、服务/归零配置和必要的homing诊断，默认不改模式架构。

- 日志同时记录configured mode / actualRunMode / type2 enabled state。
- 异步Iqf/VBUS/gain读回；检查SpdRef持续性、reference被其它路径覆盖、50Hz反馈作用。
- 服务/归零gain分别命名、保存；位置外环与内部速度环分别整定。
- 归零jitter诊断不自动断言需要加电流；false contact和模式切换漂移单独回归。
- 保持目标固定，不在每周期重新pin到实测位置；常规pause不disable/re-enable。

**回归**：5A限制及setup顺序、寄存器Ki正确、speed与position命令一致、ping不重启运动参考、静止保持不下沉、接近/接触/退让各phase不互相覆盖目标、摩擦平台不误接触、真实端挡仍停止、增益写回失败不假报告成功。

**实机**：两方向多个pitch角度恒速/位置试验；有监督归零重复，当前speed→position退让先保留。只有明确证据才改同模式退让。

## PR4 — Typed payload与重校准

涉及现有`payload/payload_profile.{hpp,cpp}`、startup/runtime选择、配置解析与文档。

- hardware/mode/payload匹配后才能qualified；旧schema档案保留历史可读，但不自动信任旧gain/成绩。
- v/a/j与dynamic gains分轴、单位清楚；安全硬上限独立。支持测量结果null和NOT_RUN。
- 文件原子保存、`.prev`回滚、应用后有效参数报告；复用现有命令，不打造独立自动调参守护进程。
- 明确`auto_verify`不影响载入的语义；不通过重启把unqualified runtime选择升级成qualified。

**回归**：旧UID/不同电机模式拒资格、同质量不同payload标识、GM无UID不伪造、缺热数据不满额qualify、未知schema拒绝、断电写文件恢复、启动/runtime信任一致、profile切换不抬高硬边界、no-op调参不报成功。

## 条件实验 C1 — Pitch current mode

不是必做PR，不阻塞核心交付。PR3后给出“原生速度环为什么仍不够”的同工况A/B证据，再决定是否实现RunMode3与host速度PI/重力摩擦前馈。不默认MIT，不假设LimitCur对current模式有效；5A命令约束、失联保护、重力支撑与有载制动证据不足就保留speed。

## 测试层级与实际命令

本包仅执行Python离线工具测试，不包含仓库C++编译或Pi结果。

Codex在实际仓库中按AGENTS/构建文件取得正确build目录后运行对应x86_64单元测试和集成测试；若按当前AGENTS排除retained_homing，报告排除原因并在后续单独跑相关回归，不把排除的测试算PASS。不要编造不存在的CMake preset或launcher flags。

通用测试在宿主机native执行；arm64二进制在宿主机编译；需要硬件的测试在Pi执行。平台/固件/timeout区别写进报告。

## 必需结果表（新建reports/implementation_status.md）

| 能力 | 源码/离线 | 单轴物理 | 热/双轴 | 当前授权范围 |
|---|---|---|---|---|
| 输出仲裁/恢复 | NOT_RUN | NOT_RUN | NOT_RUN | 未部署 |
| Yaw电流速度环 | NOT_RUN | NOT_RUN | NOT_RUN | 仅当前已批准范围 |
| Yaw摩擦补偿 | NOT_RUN | NOT_RUN | NOT_RUN | 默认关闭直至校准 |
| Pitch原生速度整定 | NOT_RUN | NOT_RUN | NOT_RUN | 保留现有模式/硬限制 |
| Pitch归零 | NOT_RUN | NOT_RUN | NOT_RUN | 不从旧机构继承成绩 |
| 新payload资格 | NOT_RUN | NOT_RUN | NOT_RUN | 不自动启用 |

每次提交必须列出：实际变更、原因、真实测试命令/结果、未测试项、有效参数diff、source/deployed commit、下一步及回滚。不得为了“清空未完成项”把物理测试改为可选后写全部通过。

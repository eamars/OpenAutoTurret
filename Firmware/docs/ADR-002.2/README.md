# ADR-002.2 · 可复用系统辨识与双环境自动整定

**决策日期：2026-09-30 · 一套交付范围 · 一个完成标准**

**阶段1/2已由[架构师覆盖指令](docs/07_STAGE1_OFFLINE_OVERRIDE.md)修订：阶段1是完整纯数学软件与合成验证，阶段2才核验实机能力、实测标定并自动辨识/注入参数。** 当前本地工作禁止访问设备，到阶段2入口停止。全ADR仍须3a/3b实机双验收。

本包保存完整开发合同和参考工具；阶段1实际软件位于[commissioning](../../commissioning/README.md)与共享C++核心，已接入本地构建和显式离线回放。物理部署/验收另记；没有连接 Pi、发送 CAN、运行实机或取得新载荷数据。

阶段1结果与阶段2入口见[本地验证报告](reports/STAGE1_HANDOFF.md)及[机器证据](reports/STAGE1_LOCAL_VALIDATION.json)。

## 唯一流程

**1 建模 → 2 获取实际系统反馈 → 3a 独立调参验证程序实机通过 → 3b production software 实机通过。**

阶段1、2形成可复用的软件、标定、原始响应、模型族与参数资产。硬件身份相同、坐标与时序有效、数据覆盖当前工况时直接复用；payload、预紧/摩擦和温度变化使用同一套方法更新相应参数版本，不重开发、不重编译、不进行 PID 实机盲搜。新参数仍必须经过3a和3b。

不以“首版”“MVP”“后续完善”免除任何必需能力；允许按依赖提交代码，不允许以一个中间提交宣布 ADR 完成。明确的未知物理条件记为 BLOCKED，未完成的实机证据记为 NOT_RUN，不能写成将来可选工作。

## 阅读入口

| 文件 | 用途 |
|---|---|
| `00_CODEX_START.md` | agent 行为合同、接手步骤和唯一完成标准 |
| `ADR-002.2.md` | 正式决策及对002/002.1的替代关系 |
| `docs/01_STAGES_AND_REUSE.md` | 三阶段、硬件/工况身份、缓存复用与失效 |
| `docs/02_MODEL_AND_SYNTHESIS.md` | 电流等效模型、辨识、前馈、反馈计算与不确定性 |
| `docs/03_INDEPENDENT_PROGRAM.md` | 独立程序、共用控制核心、资源所有权和接口 |
| `docs/04_DATA_AND_ADAPTATION.md` | 实际反馈、IMU、实验生成、工况变化与自动更新 |
| `docs/05_DUAL_VALIDATION.md` | 必需的3a/3b、质量指标、变化矩阵与正式晋升 |
| `docs/06_IMPLEMENTATION_CONTRACT.md` | 实施边界、源代码结合点、错误分支和交付清单 |
| `docs/07_STAGE1_OFFLINE_OVERRIDE.md` | 优先覆盖阶段1/2职责、前置条件和完成标准 |
| `contracts/` | JSON数据契约、验收规则、全量需求矩阵 |
| `reference/`、`tests/` | 不访问硬件的数学/身份/双验收参考与测试 |
| `sources/READING_LOG.md` | 来源、选择性代码阅读和未核实范围 |
| `reports/OFFLINE_VALIDATION.json` | 本包实际运行结果，不是固件或实机资格 |

## 范围

仅非武器化相机/传感器云台的电机与测量工程。不得将本包用于武器瞄准、发射或伤害目标的优化。没有视觉识别、目标选择、射击逻辑、UI重构或新云服务。

本包选择**两轴电流接口＋同一实现的主机运动控制核心**作为正常运动的完整目标。它是针对“可辨识、可计算、独立验证”的明确架构变更，不是宣称当前 pitch 已经运行在电流模式。pitch 的电流模式能力、限制、承重与停止行为必须取得证据；失败即阻断，不能由agent偷换成另一条未规定路线。[R2,R3,P1]

## 离线运行

```bash
python -m unittest discover -s tests -v
python -m reference.demo --output reports/demo
```

上述命令只运行历史参考工具；完整阶段1的构建、参数接口与复现命令见[实现说明](../../commissioning/README.md)。参考测试或离线回放通过均不能代替正常设备输出链接入和3a/3b实机资格。

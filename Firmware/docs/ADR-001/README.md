# OpenAutoTurret · N1 下一阶段开发包

**日期：2026-09-27｜供 Codex 实施参考｜仅相机/传感器云台。**

## 本包的目标

在现有 mixed-drive、单一控制所有者与三模式架构上增量开发：先建立可信的时序、配置解析与停止证据，再修复 pitch 归零和控制过冲；同时完成双摄 shadow 路径，最终实现广角找人、窄角构图、显式指定目标的连续跟踪。

这是**架构与离线开发包，不是可以直接部署的固件补丁，也不是现场运动许可**。本包自带的 Python 工具只处理合成或已记录的数据，不连接 SSH、CAN、相机、I2C 或 Hailo。本文没有完成任何 Pi 现场复测。

## 推荐阅读顺序

1. `00_CODEX_START.md`：直接给 Codex 的任务入口及首个切片。
2. `docs/01_BASELINE.md` → `02_ARCHITECTURE.md` → `03_DECISIONS_0_9.md`。
3. 按实施内容读取 `04_CONTRACTS.md`、`05_CONTROL.md`、`06_VISION.md`。
4. `07_WORK_PACKAGES.md` 和 `08_ACCEPTANCE.md` 定义任务与完成标准；`09_ROLLBACK.md` 规定降级。
5. `10_SOURCES.md` 记录证据、外部参考和核验限制。

`schemas/` 是**拟议的日志/语义 schema**，不是仓库现有 native socket 的 ABI。`examples/n1_policy_proposal.json` 是待实现的设计输入，不得复制覆盖 `turret_mixed.yaml`。

## 首个可交付切片

Codex 先做 **WP0–WP2：基线与配置解析、统一事件时序、停止证据及归零诊断**。这些工作不必等待新相机模型或现场供电修复。WP3 的双摄工作可在离线 mock 下并行，但不要在同一 PR 里同时改变电机增益、相机主源和默认模式。

供电未通过期间：可编码、回放、编译和单元测试；不得自行激活正常自动运动。实际相机探针也必须遵守 launcher 所有权及现场电源检查。

## 本包直接可运行的检查

Python 3.10+，在本目录执行（核心工具仅使用标准库）：

```bash
python -m unittest discover -s tests -v
python tools/offline_checks.py summarize examples/synthetic_trace.ndjson
python tools/offline_checks.py check-observation examples/selected_observation.json examples/validation_context.json
```

输出仅证明**本包参考工具与合成样例**，不证明仓库兼容、模型精度、控制稳定或硬件安全。实际测试结果见 `reports/PACKAGE_VALIDATION.md`。

## 范围与证据标记

- **[事实 U]**：用户提供的完整交接书，保存在 `sources/USER_HANDOFF.md`。
- **[事实 R]**：指定 GitHub 分支上选择性核对的文档/配置。
- **[参考 E]**：为时钟、相机同步、标定和候选模型查阅的官方资料。
- **[设计 D]**：本包给出的选型、协议草案、目标数值、测试计划。不是实测结果。

所有未有现场证据的目标值均为 D；未知值必须保留 `null`/`unqualified`，不能以零、默认值或“测试通过”替代。

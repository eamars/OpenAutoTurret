# ADR-003 · 相机快速跟拍：目标预测与双层前馈／反馈

日期：2026-09-30。目标目录：`Firmware/docs/ADR-003/`。

**状态：架构／开发方案，尚未实现 ADR-003，未部署、未运行任何实机。**

用途已由用户确认为相机取景、快速拍摄移动目标。本包仅设计相机视线跟随、取景稳定和相关测量，不包含发射、弹道、武器对准、身份识别或自动快门功能。

## 核心决策

保留一个目标状态估计器、一个参考轨迹生成器、一个 ADR-002.2 电机控制核心。目标运动前馈决定相机应该怎样运动；电机模型前馈决定执行该轨迹需要多少驱动。两者不能相互替代。

利用现有基于曝光时刻姿态的 LOS 转换、CV Kalman、`track_reference` 和正常参考链做必要扩展，不重写整套跟踪系统。

一套完整交付：**阶段1纯数学／仿真 → 阶段2实测参数自动注入 → 阶段3a独立程序实机通过 → 阶段3b生产软件实机通过。** 参数、标定、数据按适用性复用；没有缩减版完成状态。

## 阅读入口

| 文件 | 内容 |
|---|---|
| [00_CODEX_START.md](00_CODEX_START.md) | Codex 的执行边界、实施顺序和禁止事项 |
| [ADR-003.md](ADR-003.md) | 正式架构决策、两层控制及替代方案 |
| [docs/01_SOURCE_AUDIT.md](docs/01_SOURCE_AUDIT.md) | 当前分支的选择性控制链检查，区分现状与设计 |
| [docs/02_MATH_AND_INTERFACES.md](docs/02_MATH_AND_INTERFACES.md) | 方程、坐标、时间、接口、参数计算和降级 |
| [docs/03_STAGES_AND_ACCEPTANCE.md](docs/03_STAGES_AND_ACCEPTANCE.md) | 三阶段、14个场景、摄影指标、双验证与复用 |
| [contracts/requirements.json](contracts/requirements.json) | 18项架构决定及14项场景的追踪清单 |
| [contracts/photography_spec.template.json](contracts/photography_spec.template.json) | 真实摄影要求的输入模板；未知值不是0或默认合格 |
| [sources/SOURCES.md](sources/SOURCES.md) | 研究稿对应、代码来源和核验限制 |

## 阅读与事实边界

依据用户提供的 ADR-003 研究稿及 `codex/adr002-control` 分支必要文件。未克隆、未拉取整库、未获取模型权重。公开分支完整 HEAD 未固定，`base_commit=null`；读取的是移动分支的选择性快照，不是本地工作区或已部署二进制审计。

分支的阶段2报告记录过基线采集，但同时说明 ADR-002.2 的3a／3b尚未完成。此处不把目标架构当作已经合格的生产控制器。ADR-003阶段1可以立即独立开展；实机接入依赖真实适用的 ADR-002.2 产物与授权。

包内只有文档和声明性契约，没有电机控制实现、执行脚本或硬件命令。交付检查只检查文档、JSON、交叉引用和打包，不声称控制算法测试或实机测试通过。

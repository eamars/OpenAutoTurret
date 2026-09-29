# OpenAutoTurret · ADR-002 开发包
## Yaw / Pitch 闭环、静摩擦补偿、平滑运动与载荷重校准

日期：2026-09-29。状态：**供 Codex 实施的设计决策；不是已经集成、部署或实机验收的固件。**
基线：GitHub `ADR-001` 分支的选择性源码阅读；完整 commit SHA 未取得，必须在实际工作区补录。

先读 [00_CODEX_START.md](00_CODEX_START.md)，再读 [ADR-002.md](ADR-002.md)。

| 文件 | 用途 |
|---|---|
| [源码审计](docs/01_SOURCE_AUDIT.md) | 当前真实控制路径、参数、缺陷候选与证据等级 |
| [控制设计](docs/02_CONTROL_DESIGN.md) | 分轴内外环、单一输出、静摩擦、抗饱和、继续运行策略 |
| [调试协议](docs/03_TUNING_PROTOCOL.md) | 从局部速度测试到阶跃、热态、归零、双轴验证 |
| [载荷重校准](docs/04_PAYLOAD_RECALIBRATION.md) | 日后可直接交给 agent 的流程和档案绑定规则 |
| [任务与验收](docs/05_CODEX_PLAN_AND_ACCEPTANCE.md) | 四个必做 PR、一个有条件实验、验收与回滚 |
| [来源](sources/SOURCES.md) | 所读文件、官方手册页码及未取得的证据 |
| [离线工具](tools/README.md) | CSV 统计、简化 PI 计算；不包含任何硬件访问 |
| [载荷记录模板](templates/payload_record.example.json) | 校准证据模板，不是现有 daemon 可直接加载的配置 |

最小实施范围：修正输出仲裁/误停 → yaw 静摩擦和速度环 → pitch 原生速度环与归零 → 载荷档案。
不重写 vision、UI、调度架构；不默认加入 IMU 融合、MIT 控制、MPC、在线自学习或自动云端调参。

离线验证（Python 3.10+，仅标准库）：
```bash
python -m unittest discover -s tests -v
python tools/control_math.py
python tools/trace_metrics.py fixtures/synthetic_trace.csv --output reports/synthetic_metrics.json
```
这些命令不联网、不打开 CAN、不发电机命令。样例轨迹完全合成，不能作为整定或实机合格证据。

导入仓库时将本包文档放到 `Firmware/docs/ADR-002/`；实际实现只修改必要的现有源文件。
原始运行轨迹留在站台 run/ 目录，不提交仓库。源码、厂商 PDF、模型权重均未随包复制。

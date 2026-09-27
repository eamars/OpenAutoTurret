# 给 Codex 的实施指令

请依据本包推进 OpenAutoTurret 的 N1 阶段，保持现有工程与操作方式，不重建整个系统。

## 先做这些

读取当前 checkout 的 `AGENTS.md`、`Firmware/docs/STATION_OPERATIONS.md`、本包 `docs/01_BASELINE.md` 和 `docs/07_WORK_PACKAGES.md`。记录 `git rev-parse HEAD`、工作区状态、当前 profile 与实际 release；不要假定分支 HEAD 就是最近激活的 `8901808`。

用本地文件搜索定位本包列出的符号和协议；只阅读本任务需要的文件。本包没有核对全部 C++/Python 实现，路径与新增字段是定位线索，不是现有 ABI 的保证。不要另行下载整库、权重库、HEF、CAD 或历史运行捕获来“补齐上下文”。

首个切片执行 **WP0 → WP1 → WP2**：

- 输出启动配置解析报告：来源、最终值、单位、作用层、当前/历史/待验收身份。特别查清 axes 与 motion.modes 的覆盖规则，yaw 符号、pitch active drive mode、两层 PI 是否叠加。
- 增加有界、非阻塞的事件记录与时间戳语义；不改增益、不改自动选择 dwell、不换生产主模型。
- 将停止证据拆成 per-axis 独立字段，并修复“反馈不够新鲜导致连停止请求也被拒绝”的路径；先加故障回归，再改运行逻辑。
- 增加 pitch homing 分阶段诊断与 shadow guard；未知机械端点和最大制动能力仍需现场数据，不用推测数值填空。

完成一个切片就提交一个可独立审阅的变更，报告实际运行的测试、未运行项目和下一步。没有硬件数据时提交采集与回放工具，不把任务停在“需要用户再提供信息”；也不要伪造测量结果或用 mock pass 替代现场验证。

## 不可破坏的约束

`controld` 是唯一电机命令所有者。保留 `MANUAL / AUTO_TRACK / AUTO_ROAM` 三种正常模式；Hold 是现有人工覆盖语义，故障/关停状态与相机仲裁子状态不是新增用户模式。visiond 只发观测和选择结果，webd 不发 CAN。

保留统一 launcher、站台锁、非特权运行、独立 release 目录、现有项目 venv。不要 force-kill 控制器，不绕过 launcher 开第二个相机或电机 owner，不以不同 `OTA_RUN_DIR` 绕锁，不覆盖 Pi 未提交工作区。

pitch `LimitCur ≤ 5 A`，reset/模式切换后按驱动契约验证再 enable；不得自适应加大限流。5 A 是上限，不是每时刻必须输出的电流。yaw 的零电压请求不是 disable，也不是断能证明。

不要照搬旧双 CyberGear 的增益、标定、行程、制动或物理验收。本阶段不改为 torque/MIT 控制，不引入 ROS、数据库、消息中间件或通用插件框架。

## 并行与迁移

WP3–WP5 可在 WP1 语义稳定后用 mock/replay 并行。实际双摄/Hailo负载与运动启用依赖各自现场门槛，而不是“所有软件任务必须等供电修好”。UI 只增量显示本阶段必要信息，不重做既有 HUD。

本包 JSON schema 不是现有 native packet。先审计既有 serializer/parser 与 capability/version，再做同 release 的两端变更；不直接把 JSON 写进未知二进制 socket。

## 本次不得自行做的操作

不因接到本包便执行 `--activate` 或正常无参数启动。当前交接书记录的站台为欠压后停止。现场运动仅在运行手册要求的电源、反馈、机构清场等前提通过后，使用受控 commissioning 流程。不要降低已有硬保护以取得一条“成功”日志。

## 每个变更的交付格式

提交摘要包含：本次解决的问题、修改路径、协议/配置变化、运行命令与测试结果、尚缺证据、回退办法。测量写日期化验证报告；架构写现有 architecture 文档；运行流程写 `STATION_OPERATIONS.md`。不要把本包整套资料平行复制成另一套永久“事实来源”。

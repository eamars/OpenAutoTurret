# 05 · 选择性源码检查与结合点

读取日期2026-10-02。仅指定移动分支的必要文件及厂商资料；未clone、未取得完整HEAD、未查看本地dirty tree/物理资格。用户明确002.2/003完成，本表不是重新评判其完成度。行号会随分支改变，以符号为准。

| 来源 | 本次看见的内容 | ADR-004最小动作 |
|---|---|---|
| S01 control_loop.cpp::roam_config | 固定pitch参考，`level_scan_available=false`；命名扫区为请求而非扩大限位 | 接入GravityFrame可用性和水平路径；替换本路径的固定pitch验证，不改目标跟踪数学 |
| S02 mode/roam_planner.hpp | 已有扫描、反向、区域和内缩；pitch_ref固定 | 复用扫描相位/模式策略，增加联合路径类型，不新增业务FSM |
| S03 web/webd/hud.py | 固定中心角标、两段共线横线、reticleCantDeg=0、正角屏幕顺时针；已有stale逻辑 | 绑定权威水平提示，保留样式/中心/缺口，添加分类文本和失效隐藏 |
| S04 tools/imu_bno085.c | accel/gyro/rv/game_rv；原始values和另列relative_xyzw；请求20ms间隔；host tare而非本包所需绝对base姿态 | 复用raw game_rv；不新增I2C owner、不用relative替代gravity；实际率和时延仍取有效数据 |
| S11 imu_trace_ingest.cpp | 异步trace读取、game原始快照、generation/freshness | 在既有消费者增加必要字段，不新开读文件/融合服务 |
| S05 geometry/turret_kinematics.hpp | B前/左/名义上、C右/下/前，Rz*Ry*R_PC、光线转换 | 直接复用，安装标定/真实单位来源由已完成002.2确认 |
| S06 geometry/los_joint_solver.hpp | 实际外参、两pitch分支、角等价/范围选择 | 复用点逆解，补路径连通性、极值、奇异和连续branch验证 |
| S07 control/safety_envelope.hpp | 区分Measured/Virtual/Unbounded，原有停止距离助手 | 复用运行时界限及已接受停止能力；speed-only公式不能当含初始加速度/滞后的完整证明 |
| S08 config/turret_mixed.yaml | yaw position_envelope=none，pitch soft_margin_deg=5，存在命名roam区域 | 不恢复旧±90硬限；soft内缩与新增工作留距分账；配置值不冒充当前运行读回 |
| S09 config/camera_install.yaml | 显示方向支持rotate180及镜像，本次文件为rotate180 | active transform只应用一次，不因倒置再转一次 |
| S10 web/webd/app.py | 主页面来自hud，现有IMU诊断0.5s缓存；视频和状态是不同通道 | GravityFrame作为control权威状态转发；不靠该缓存做动态cant、不承诺MJPEG曝光同步 |
| S12 control/reference_manager.hpp | 来源裁决/LOS到joint，自己不发CAN | 加gravity scan intent/geometry；仍只一个最终reference owner |
| S13 control/tracking_reference.hpp | 原有跟踪参考生成入口 | 保持ADR003 q/v/a所有权；只扩展路径映射，避免平行参考输出 |
| S14 AGENTS.md | 文档索引、既有站台规程、统一入口 | 遵从本地实际操作规程，不自动运行部署/电机命令 |

### 需要本地核对而不是让架构师猜的内容

实际共享核心的类/构建目标；002.2停止API是否已包含加速度/延迟；R_PS/R_PC及时间标定是否已接受；实际IMU report质量/率；当前有效限位与额外留距；当前profile是否已覆盖倒置/倾斜；ADR003最终reference owner和相机显示time/transform实现。

若本地已实现本表缺口，直接映射复用，不按旧文件重复补一次。差异应记录，不允许将“核对差异”扩展成整库审计或重做旧ADR。

# 来源与核验边界

## 设计与用户基线

U：当前用户的六项ADR-004要求；已确认相机用途及ADR-002.2/003完成。该完成状态为用户报告，未独立重做其物理测试。

P1：本对话ADR-002.2及其阶段边界覆盖指令，2026-09-30：阶段1完全纯数学、参数自动注入、双实机验证。

P2：OpenAutoTurret_ADR-003_Development.md，2026-09-30：一个reference owner、基座LOS、同一q/v/a、两层分工；本次读取开头架构部分。旧文件“尚未完成”措辞是历史状态，不是当前结论。

P3：用户Library中的open_auto_turret_bno085_imu_expansion_v1_1.md，2026-09-01：pitch-stage IMU、重力回推base、无磁北水平面等历史设计。只参考上述思路，不继承其自动tare/无需安装标定等旧提议，也不将其当当前实现。

## 选择性项目来源（移动分支，无固定commit）

### S01
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/control_loop.cpp

### S02
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/mode/roam_planner.hpp

### S03
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/web/webd/hud.py

### S04
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/tools/imu_bno085.c

### S05
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/geometry/turret_kinematics.hpp

### S06
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/geometry/los_joint_solver.hpp

### S07
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/safety_envelope.hpp

### S08
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/config/turret_mixed.yaml

### S09
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/config/camera_install.yaml

### S10
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/web/webd/app.py

### S11
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/imu_trace_ingest.cpp

### S12
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/reference_manager.hpp

### S13
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/tracking_reference.hpp

### S14
https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/AGENTS.md

## 厂商资料

### V01 · CEVA BNO08X Datasheet
https://www.ceva-ip.com/wp-content/uploads/BNO080_085-Datasheet.pdf
已读取并截图核对game rotation vector相关页（PDF索引30）：重力roll/pitch，gyro/accel，无磁北yaw锁定。不将请求采样频率或融合状态等级当实测精度。

### V02 · CEVA SH-2 Reference Manual
https://www.ceva-ip.com/wp-content/uploads/SH-2-Reference-Manual.pdf
已读取并截图核对Gravity报告页（PDF索引67）：报告0x06，device frame、m/s²/Q8。当前方案复用raw game_rv，不要求为ADR004额外启用该report。

## 本包自己的数学与决定
所有路径链式求导、参考所有权、保留距离组合、投影矩阵与开发范围为本包推导/设计。2°工作留距、1°/2°平扫要求、2s窗口等是明确设计要求，不是用户实测、厂商保证或已验收成绩。

## 未执行
未clone/整库下载；没有固定分支HEAD；没有连接Pi、CAN/I2C、编译C++或改生产固件；没有取得当前本地profile/标定/限位读回，也没有物理平扫、倒置或HUD实时测试。原始源码/PDF不随包复制。

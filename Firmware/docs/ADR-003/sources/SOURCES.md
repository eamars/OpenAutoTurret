# 来源、研究稿对应与核验限制

读取日期：2026-09-30。证据分层：**R＝用户提供要求；S＝本次选择性公开源码／项目文档；E＝外部官方定义；D＝本包设计。** D中的方案、公式、预算与方法不应伪装成R的原话或S中已经实现的能力。

## 用户材料

**R0**：本轮附件 `Pasted text(20260930-084708).txt`，以“Please design ADR-003...”开头的23节研究稿。会话检索引用为`turn16file0`。它是需求／架构研究，不是现场数据或已经证明的控制器。原附件按原始字节保存在本包 `sources/USER_RESEARCH.txt`，供Codex对照；未修改其内容。

**R1**：用户明确用途“相机取景用，需要快速拍摄移动目标”。阶段纪律继承本轮此前明确的ADR-002.2覆盖要求：阶段1纯数学、阶段2反馈注入、1／2按适用性复用、3a和3b均实机通过、单一完整交付、禁止agent手调及无关过度开发。

## 研究稿到本设计的对应

| 研究稿 | 处理 |
|---|---|
| §§1–2、6：两级FF／FB | 原则保留；采用已有二阶参考生成结构作为Level1的确切公式 |
| §§3–4：坐标与一致轨迹 | 保留；补充同一曝光时刻姿态、关节Jacobian及同一积分状态 |
| §5：电机PD／积分示例 | 不以示例覆盖ADR-002.2；保留已接受的位置P／速度PI及anti-windup |
| §§7–9：静止、噪声、估计边界 | 保留；一个CV估计器与连续置信权重，无静止专用控制器 |
| §§10–15：时序、多速率、停止、反向、缺速、丢失 | 全部列为强制行为；失联与速度无效分别处理 |
| §§16–18：002.2关系与阶段 | 保留；依赖的物理伺服未完成不阻塞纯数学阶段 |
| §19：14场景与两种FF对照 | 全部保留；不伪造最终接受数值，建立摄影要求输入 |
| §§20–22：可观测性、降级、18项决定 | 全部覆盖，使用最小现有日志及运行时接口 |
| §23：仅设计交付、实现文件边界 | 本包只有开发方案／声明性契约，不实现ADR-003 |

**明确架构选择而非研究稿原结论：** CV而不增加目标加速度状态；参考误差／伺服误差分配；连续时间Q定义；光学时间V2尾部；自动R/q/λ计算规则；摄影画面指标。研究稿允许对这些项作决定，本包没有把示例方程当强制替换源码。

## 选择性公开文件

所有S文件来自移动分支 `codex/adr002-control`。读取以必要类／函数／文档状态为限；没有将整个仓库下载到工作区。

| ID | 文件／入口 |
|---|---|
| S01 | [ADR-002.2决策](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/docs/ADR-002.2/ADR-002.2.md) |
| S02 | [阶段2准备／基线进度报告](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/docs/ADR-002.2/reports/STAGE2_READINESS.md) |
| S03 | [ControlLoop实现](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/control_loop.cpp) |
| S04 | [TrackingController](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/tracking_controller.hpp) |
| S05 | [TargetEstimator接口](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/tracking/target_estimator.hpp) |
| S06 | [TargetEstimator实现](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/tracking/target_estimator.cpp) |
| S07 | [track_reference](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/tracking_reference.hpp) |
| S08 | [ReferenceLimiter](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/reference_limiter.hpp) |
| S09 | [ReferenceManager](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/reference_manager.hpp) |
| S10 | [当前混合机构配置](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/config/turret_mixed.yaml) |
| S11 | [VisionIngest](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/vision/vision_ingest.cpp) |
| S12 | [native perception wire](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/tracks/perception_wire.hpp) |
| S13 | [perception pipeline](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/perception/pipeline.py) |
| S14 | [camera owner](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/perception/camera.py) |
| S15 | [SpeedServo](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/speed_servo.hpp) |
| S16 | [MotorStateHistory](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/common/motor_state_history.hpp) |
| S17 | [控制时钟](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/common/time.hpp) |
| S18 | [AutoTrackController政策](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/tracking/auto_track_controller.hpp) |
| S19 | [ControlLoop接口／依赖](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/control_loop.hpp) |
| S20 | [MotionIntent](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/control/motion_intent.hpp) |
| S21 | [文档命名／目录约定](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/docs/README.md) |
| S22 | [旧TargetMeasurement契约](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/tracking/target_measurement.hpp) |
| S23 | [LOS joint solver](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/control/src/geometry/los_joint_solver.hpp) |
| S24 | [已入库阶段1离线覆盖指令](https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/adr002-control/Firmware/docs/ADR-002.2/docs/07_STAGE1_OFFLINE_OVERRIDE.md) |

## 外部官方资料

**E01**：[libcamera官方controls定义](https://raw.githubusercontent.com/libcamera-org/libcamera/master/src/libcamera/control_ids_core.yaml)。用于核对SensorTimestamp、曝光时间和时钟定义。读取到的上游定义不能替代已安装版本／pipeline的实际核验。

**E02**：[OpenCV相机标定与重建说明](https://docs.opencv.org/4.13.0/d9/d0c/group__calib3d.html)。用于投影／畸变／内外参及缩放语义，不作为当前相机已经标定的证据。

## 失败和未取得的内容

GitHub目录页和commit API未能取得可用的完整HEAD；`tracking_controller.cpp`路径返回404，而实际控制逻辑位于已读取的header。未把请求失败推断为仓库缺失功能。

`base_commit=null`。所读文件可能处于移动分支不同更新时刻，本包仅提供必要改动面和设计；Codex须以实际工作区记录HEAD和差异。项目报告里的checkout／release SHA不是本次读取的统一源码基线。

未访问Pi、未运行控制程序、未发送CAN、未修改源代码或配置、未重跑作者报告的测试、未读取其本地raw运行目录。未下载整库、厂商PDF或模型权重。包内校验和只用于本包文件一致性，不是源代码快照哈希。

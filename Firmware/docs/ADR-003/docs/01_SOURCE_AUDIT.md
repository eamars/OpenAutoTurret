# 01 · 当前控制链的选择性检查

读取对象：`eamars/OpenAutoTurret` 的 `codex/adr002-control` 分支，2026-09-30。未固定完整HEAD，未检查本地脏树或运行设备。下面仅描述读取到的范围；代码注释中的历史测量不作为本次实机证据。

## 1. 实际调用链与已有能力

| 当前入口／符号 | 读取到的事实 | ADR-003处理 |
|---|---|---|
| `perception/camera.py::CameraOwner.next_frame` | 保留SensorTimestamp与metadata receive；缺sensor stamp不会用now冒充 | 保留。增加明确光学时间／控制时钟映射，不重写采集框架 [S14] |
| `perception/pipeline.py` | 已分段记录处理时间；已有latest-only预览及异步诊断快照 | 保留，只补缺失时间证据。不能把“增加latest-only”当作全新收益 [S13] |
| `tracks/perception_wire.hpp`、`vision_ingest.cpp` | native观测已带session、UUID、generation与质量；已有SEQPACKET接收 | 在原codec上一次版本化增加光学时间字段，不新做transport [S11,S12] |
| `TrackingController::update_snapshots/set_measurement` | 按新鲜的各轴RX时刻维护历史，使用观测时刻姿态将像素转基座LOS | 这是已有自身运动补偿基础，不从零建立，不再额外加encoder rate [S04,S16] |
| `TargetEstimator` | 已有每轴θ／ω Kalman、协方差、创新门控及预测时域 | 保留结构，限定修改Q时间语义、自动拟合和机动时的观测处理 [S05,S06] |
| `joint_motion_rates` | 已将可信LOS速度映射到joint motion；当前为小时间前推求解 | 合并到单一状态／几何契约，明确导数与分支；不得说现有代码完全没有速度FF [S04] |
| `ReferenceManager::resolve` | 负责intent→joint位置、速度上限及不可达判断 | 扩展携带同一目标运动状态；不在此重复滤波或积分 [S09,S23] |
| `track_reference` | 已有二阶位置／速度跟随与jerk约束；omega内有经验夹限 | 扩展为唯一q/v/a出口，参数来自计算；不另建新追踪控制器 [S07] |
| `ControlLoop::step` | 自动跟踪会读取joint motion rate进入生成器；后续服务速度路径有位置误差修正和再次限制 | 迁移到002.2规定的core时去除本路径重复职责，不能直接加另一层环 [S03,S15] |
| 控制遥测q_ref_rate/accel | 当前存在基于先前q_ref的平滑差分显示值 | 只保留显示；真实控制参考直接来自生成器状态，不从遥测回取 [S03] |

## 2. 已发现的设计约束，不等于实机根因证明

**A. “还没有预测”不是当前事实。** 当前基座LOS估计、预测与速度前馈已经存在。因此本ADR是明确数据语义和控制归属、补齐一致性及数据求解，而不是全套重建。

**B. 预测时域仍含配置提前量。** 当前配置有`motor_response_ms:120.0`，控制类组合观测时域与控制／电机提前量。这是需要分解验证的设置，不能据此断言120 ms就是纯时延，也不能盲目增加。[S04,S10]

**C. 原生消息与旧58字节消息不是同一能力。** native输入已带较丰富的选择身份；不能以旧结构字段少为由替换整条native管线。[S11,S12,S22]

**D. 两层整形需要按职责核对。** 当前track_reference和SpeedServo都修改动态状态；ADR-002.2目标核心也有自己的物理控制职责。本ADR只确定一个最终reference owner，不直接删除所有电机保护或所有速度限制。[S03,S07,S15]

**E. 时间单位一致不代表时钟语义一致。** 控制使用CLOCK_MONOTONIC；当前camera封装直接取得SensorTimestamp。官方libcamera定义SensorTimestamp为CLOCK_BOOTTIME的第一行曝光时间。必须检查已安装版本及映射；这并不证明当前运行必然存在非零时钟偏差。[S14,S17,E01]

**F. 当前参数不是完整的新机构资格。** 所读分支的ADR-002.2总文档仍注明3a／3b未完成；更详细的阶段2报告已记录基线采集进展。两者更新粒度不同，不能用旧摘要否认后续采集，也不能把基线等同运动资格。[S01,S02]

**G. 当前构图点不是独立头部检测。** 配置的box_fraction只是现有摄影anchor策略。跟踪优化保持同一anchor source，不在此增加人脸／头部检测器。[S10]

## 3. 最小改动面

| 文件／现有模块 | 允许的必要改动 | 不做 |
|---|---|---|
| `tracking/target_estimator.{hpp,cpp}` | 明确时间模型、协方差及噪声参数语义；自动拟合结果加载；机动／速度不可信处理 | CA/IMM/神经网络、另写一份Python估计器作正式控制 |
| `control/tracking_controller.hpp` | 分离纯状态输出、预测查询与模式授权；单一TargetState | 保留另一套能独立授予运动的内部FSM并与AutoTrack争用 |
| `control/reference_manager.hpp`、`motion_intent.hpp` | 扩展已选目标的运动／时间契约 | 第二个参考裁决器、绕过limits |
| `control/tracking_reference.hpp`、`reference_limiter.hpp` | 唯一q/v/a返回与一致积分，热参数边界显式化 | 新通用规划库、独立位置／速度／加速度三条轨迹 |
| `control/control_loop.cpp` | 接入同一状态和参考样本；局部移除重复lead/显示求导依赖 | 重写整个主循环、顺便修所有历史命名 |
| 002.2 control core／backend | 仅消费统一ReferenceSample所需的最小接口衔接 | 新PI、新摩擦表、新电流模式、新电机整定实验 |
| `perception/camera.py`、pipeline timing、native codec | 增加必要光学时间／revision与已有trace字段 | 双摄同步、模型更换、全新消息总线 |
| 既有独立验证宿主、测试与CMake | 链接同一个tracking_core，新增本ADR场景 | 第二个常驻daemon、云服务、专用数据库 |

这里列出的路径是所读分支中的结合点。若本地已有完成的等价能力，复用并举证，不重新实现；若函数移动，只沿其调用链定位。不得以本设计包的逻辑类名推断仓库已有同名函数。

## 4. 当前不足以断言的内容

没有取得当前完整HEAD、运行二进制、真实完整视觉／电机trace、实际相机曝光时序标定、摄影目标速度范围、可接受像素／拖影预算，或002.2双验证证书。本次无法给出“已提升多少毫秒／多少倍”的结论。

公开报告中的日期、提交与设备状态均为其作者记录，不是本次访问设备得到的事实。审计只建立开发改动边界，不替代阶段2与3。

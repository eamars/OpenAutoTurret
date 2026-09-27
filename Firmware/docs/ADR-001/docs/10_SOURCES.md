# 10 · 证据目录与核验限制

## 1. 本次读取范围

读取用户附带的完整架构交接书，以及指定分支上少量文档/配置的相关段落。没有clone、仓库归档下载、递归获取代码、HEF/权重下载或Pi运行数据获取。未修改GitHub/站台。

原始附件复制在 `sources/USER_HANDOFF.md`；U的L编号指聊天文件工具呈现的逻辑行号。GitHub源的URL使用用户指定分支，读取日期2026-09-27；branch tip SHA没有独立取得。`8901808`只表示交接书报告的最近激活版本，不冒充本次网页内容固定的完整commit。

## 2. 用户来源

U：`OpenAutoTurret 当前架构与架构师决策交接书`（用户附件）。关键定位：L3–L5基线/停止；L15–L34进程；L38–L52硬件/标定；L58–L75闭环与图像语义；L79–L85集成；L93–L104问题0–9与其他风险；L108–L112交付要求。

## 3. 指定GitHub分支文档和配置

基址：`https://raw.githubusercontent.com/eamars/OpenAutoTurret/codex/hardware-adaptation/`

| ID | 路径 | 用途 |
|---|---|---|
| R1 | Firmware/docs/STATION_OPERATIONS.md | 当前停止状态、所有权、部署/stop与受控commissioning合同 |
| R2 | Firmware/config/mixed_hardware.yaml | 两轴协议、总线、ID、UID与5A cap |
| R3 | Firmware/config/turret_mixed.yaml | 文件层配置、模式/轴限制、homing与tracking参数 |
| R4 | Firmware/docs/AI_HAT_PERCEPTION_PLAN.md | 既有视觉计划、非生物识别范围、selected目标连续性与分阶段集成 |
| R5 | Firmware/perception/configs/perception_v1.json | 实際profile、阈值、model字段与selection/tracker设置 |
| R6 | Firmware/config/hailo_yolov8n_manifest.json | HEF SHA/架构/runtime、pad、NMS与许可声明 |
| R7 | Firmware/config/camera_install.yaml | rotate_180安装变换 |
| R8 | Firmware/docs/HARDWARE_ADAPTATION_PLAN.md | mixed协议与异步RX/控制设计的相关段落 |
| R9 | AGENTS.md | 仓库操作约束 |
| R10 | Firmware/docs/README.md | 文档地图及历史/当前证据划分 |

未逐行审计所有实现。文档中指向的 `control_loop.cpp / can_motor_backend.cpp / tracking/` 等交给Codex本地针对任务阅读。本包提出的native扩展、thread布局、配置resolver和stop状态必须对照实际源码落地，不以本包字段假装现存接口。

## 4. 补充官方参考（非本机实测）

E1 · Raspberry Pi 27W USB-C供电规格，5.1V/5A档：
`https://www.raspberrypi.com/products/27w-power-supply/`

E2 · Raspberry Pi camera software，多摄与软件同步：同名义fps，异型号存在额外偏差；不是本机同步验收。
`https://www.raspberrypi.com/documentation/computers/camera_software.html#synchronise-cameras`

E3 · libcamera当前SensorTimestamp定义：CLOCK_BOOTTIME、首行曝光。与本机版本/实现是否相同尚需核对。
`https://raw.githubusercontent.com/libcamera-org/libcamera/master/src/libcamera/control_ids_core.yaml`

E4 · OpenCV ChArUco/ArUco标定方法与API参考，不能据此假定本机装有该版本。
`https://docs.opencv.org/4.13.0/da/d13/tutorial_aruco_calibration.html`

E5 · AprilRobotics AprilTag官方库及family说明；本机未安装/未性能验证。
`https://github.com/AprilRobotics/apriltag`

E6 · OpenCV Zoo YuNet官方模型说明和目录许可；不同ONNX形状/版本需与本机匹配，尚无本机测量。
`https://raw.githubusercontent.com/opencv/opencv_zoo/main/models/face_detection_yunet/README.md`
`https://raw.githubusercontent.com/opencv/opencv_zoo/main/models/face_detection_yunet/LICENSE`

## 5. 设计与未知项

过程/线程选择、异步仲裁、JSON语义、所有新benchmark目标、测试样本数、默认feature阶段、tag/YuNet选型均为D设计提案。实际电源接法、load惯量、per-mode有效K/D、双摄外参、stop最坏包络、工作点增益、全链尾延迟、精度与IMU动态质量仍未知。

当前值、建议值、实测值分别保存在不同字段。每个未完成现场项保留NOT_RUN/BLOCKED；不要把“文档齐全”写成“物理验收通过”。

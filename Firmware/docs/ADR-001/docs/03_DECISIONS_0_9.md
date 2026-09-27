# 03 · 交接书第0–9题的逐项决策

本章的阈值为**拟议验收目标 D**，不证明当前站台已达到；详细试验定义见08。不能只为通过验收而放宽目标，调整必须带测量依据和版本记录。

## 0 · P0 供电与运行资格

**事实/根因假设：** 负载下0x50005和受控停止已观察；并发build失联的原因未证实。候选原因包括电源5V输出档不足、线缆/接头/HAT路径压降、瞬态负载与热限制，需区分，不能单凭“换大瓦数电源”结案。[U L93；R1]

**必要测量：** 记录供电型号/PD档、实际接法、负载时板端电压、当前/历史throttled位、CPU温度/频率、内存、OOM/内核/Hailo/CSI/CAN错误与单调时间。测量接电由操作者完成，软件不操作未确认的电源切断装置。

**候选方案/选型：** 选择可验证5V档与线缆、供电/电机路径审查，加低频health monitor和分级负载。Pi官方27W供电的5.1V/5A是参考条件，不能证明本机HAT布线或瞬态裕量。[E1] 不采用降CPU频率掩盖欠压，也不以软件watchdog冒充独立断能。

**契约：** `power_sample{raw_bits,current_bits,history_bits,sample_time,age,temperature,frequency}`；采样失败为unknown。启动qualifier与既有安全状态机消费健康结果，控制线程不运行vcgencmd。history-only不永久锁死，当前故障立即撤销后续运动资格，已发生的新历史位同样使本轮复测失败。

**验收：** idle5min、两单摄各10min、双摄+Hailo30min记录；无当前bit0–3、无新增bit16–19、无重启/OOM/总线错误。历史位预先已置位时，说明软件不能排除采样间短瞬态，必要时重新建立受控boot基线和外部电压记录。后续运动负载单独复测，不由静态双摄pass替代。

**降级/回退：** 站台保持停止；允许离线开发。修复不具备证据时不激活AUTO。所有阶段保留失败日志。

## 1 · pitch归零不平滑

**根因假设：** 反馈原始速度噪声；接收时间量化引起的差分尖峰；内部速度环与host PI叠加；位置/速度模式切换跳变；静摩擦、结构振动、接触判据混淆。现有资料不能判定哪个占主导。[U L94]

**必要测量：** 分阶段记录目标位置/速度、encoder位置/采样间隔/窗口速度、原始速度、Iq、LimitCur读回及年龄、控制输出、积分状态、限幅命中、模式切换、接触候选、guard理由与IMU。运动开始前至最终停止连续采集。

**候选方案/选型：** 先用按真实反馈时间的窗口速度估计与无扰模式切换，不先重写触端算法或切MIT。pitch归零速度段保留驱动内部速度环；若查明host也积分同一误差，则选一处积分，禁止双重积分。先通过replay区分真实超速与估计器噪声，再在受控运动下验证。

**契约：** `HomingPhaseEvent`、`VelocityEstimate{window_ns,sample_count,valid,age}`、hard_guard与shadow_guard独立reason code；raw speed只保留为诊断，不单独认定抖动。速度/位移/时间硬约束不得用滤波延迟掩盖。

**验收：** 10次受控完整归零无false-contact、无hard-guard越界被忽略，端点重复性≤0.5°；远离接触区的p95速度误差目标≤max(1°/s,20%接近速度)，不出现持续反向振荡；命令不连续跳变超过既有加速度/jerk约束。现有`motion_checks_abort:false`不能作为通过状态。

**回退：** 保持未就绪，停止本次归零并输出阶段/数据缺口。保留5A上限，不通过反复enable/加流冲过机构卡点。

## 2 · 反馈速度不足

**根因假设：** 总延迟可能被帧周期、曝光、排队、选择dwell、pitch轮询和机械响应主导，而不是Hailo推理。[U L95]

**必要测量：** 按04统一时钟记录曝光参考、host取帧、推理、关联、选择、发布、controller接收/受理、CAN排队与反馈响应；记录每级分位数、样本数、丢弃原因与完整case。

**候选方案/选型：** 优先latest-only、消除同步等待和重复dwell、改善pitch非阻塞反馈调度，再评估15→20/25/30Hz。保持control200Hz；不先盲目提高主循环频率。

**契约：** Event schema + frame/decision/command关联；原始sensor时钟和接收时钟分开；失序/超期检测。CAN enqueue不命名为物理TX完成，encoder首次响应是独立受控试验指标。

**验收：** 拟议p95曝光参考→controller接收≤80ms、p99≤120ms；publish→controller接收p95≤5ms；wide有效观测≥20Hz且目标维持约26Hz，detail先≥14Hz并以20Hz为后续优化目标。反馈年龄性能目标yaw p99≤10ms、pitch p99≤40ms；100ms硬失效规则不变。分辨冷启动选择延迟与已选目标反应延迟。

**回退：** 先减预览/可选模型负载，必要时禁用detail；不得用重复同一帧冒充帧率或用publish时间刷新旧观测。

## 3 · 过冲

**根因假设：** 响应120ms假设不匹配、积分饱和、命令斜率/环路重复、预测重复补偿、视觉构图锚点切换、非新鲜反馈均可能贡献。[U L96]

**必要测量：** 分yaw/pitch、正反向、载荷与姿态测有界阶跃、斜坡、停止、近边界、视觉目标换向/遮挡，记录峰值误差、稳定时间、饱和驻留和有效延迟。

**候选方案/选型：** 采用轴独立辨识与增益；yaw保留host电压伺服，pitch优先保留本机驱动位置能力并审计service_speed_control实际分支。单处积分、饱和反馈anti-windup、无扰切源、共用reference limiter。不从资料捏造新Kp/Ki。

**契约：** 每轴`ActuatorCapabilities`、`EffectiveLimits`、饱和/积分/command-mode遥测；参考速度为目标运动前馈留修正余量，但不自动修改物理速度上限。辨识参数绑定installation/config/load ID。

**验收：** 已观察过的±15°pitch/约30°yaw包络内先做单轴受控测试；峰值超调目标≤max(0.5°,阶跃幅值5%)，轨迹结束后1s内进入±0.5°且保持500ms。不同于“从命令开始1s内必须走完”。边界不越界，测得停止包络覆盖后续授权速度。

**回退：** 退回未改gain的mixed版本并保持必要资格门槛；不回退旧双CyberGear固件，不用更宽软件边界掩盖过冲。

## 4 · 双摄同步追踪

**根因假设：** 两帧首发差25.1ms不代表同步质量；26/15Hz、独立曝光与机构运动导致时间不齐、视差和身份混淆。[U L97]

**必要测量：** sensor/clock语义、曝光参数、持续Δt分布、运动模糊、双摄负载、相同目标的跨镜几何残差和误关联事件。

**候选方案/选型：** 选双worker+异步时间对齐+单主观测仲裁；暂不要求硬同步。官方软件sync作为后续同名义fps实验，不把frame-start同步当成曝光中点/rolling-shutter整幅同步。[E2]

**契约：** per-camera frame identity/generation、local与global track、source membership、共同时刻姿态与ray origin。对齐有协方差和质量说明；本阶段不强行做立体深度。

**验收：** 连续双摄30min不重复owner、不阻塞wide；已选目标交接测试至少100个独立episode，错误替换=0，正确交接成功率≥95%；报告样本量、失败与不确定率，零观察错误不等于零真实错误概率。

**回退：** detail失效回同一wide目标；宽窄都不明确则停止追随/既有loss处理。不能因窄角推理卡住停止wide更新。

## 5 · YOLO集成

**根因假设：** HEF可运行不等于person精度可用；类别索引、RGB顺序、letterbox反变换、设备NMS截断都可能错误。[U L98；R5–R6]

**必要测量：** 有标签真实人物、空景负例、遮挡/低照、实际score分布、设备输出阈值、camera→model→camera像素round-trip、持续运行功耗和延迟。

**候选方案/选型：** N1固定现有IMX477+Hailo YOLOv8n person HEF作为detail baseline；不顺便迁移新的YOLO大版本。head/face是独立任务，当前COCO person输出不能冒称head。

**契约：** 单一model manifest，保存artifact SHA、architecture、runtime、input、preprocess、labels、NMS、license声明和模型资格。两个现有manifest位置应由一份canonical事实生成/验证，禁止重复人工维护。

**验收：** SHA/架构/输出shape/类别映射/坐标变换测试全部通过；部署域person precision目标≥0.95、recall≥0.90（IoU≥0.5，逐场景报告，不只总平均）。至少300个标注person实例、100个负例帧作为首轮工程样本。

**回退：** hash不符或Hailo失败使detail不可用，不silent换未资格化模型；保留IMX500基线。HEF仍在忽略的artifact路径，按版本分发，不提交运行捕获。

## 6 · YOLO在哪个摄像头

**根因假设：** 全局搜索与窄角细节需要不同覆盖；25mm信息不足以决定远距头部质量。[U L99]

**必要测量：** 两路实际生产模式FOV、每米目标像素、2/5/10m等可达距离的检测/模糊/曝光与CPU/Hailo预算。距离仅是试验分箱，不是当前保证。

**候选方案/选型：** 选择IMX500端侧SSD持续wide搜索，IMX477+Hailo person作为detail。保留现有物理分工，不让昂贵Hailo广角推理成为首阶段依赖。只有wide召回确实不足，才开展Hailo-wide模型对照。

**契约：** `camera_role != motor_authority`；wide/detail配置只声明观测角色。授权由同一controller支持的qualification+arbiter结果决定。

**验收：** detail加入后wide有效fps/年龄不恶化超过10%基线；窄角无有效证据时仍能用wide持续覆盖。只有detail几何与质量都通过才进入authoritative。

**回退：** wide-only；不会因窄角视场小而无限扫描追逐假目标。

## 7 · FOV与几何标定

**根因假设：** sensor mode裁切、focus、安装旋转、双相机baseline与光轴差异会让名义FOV和真实LOS不一致。[U L100]

**必要测量：** 每个mode的K/D、crop/scale/orientation、相机间与相机到轴外参，跨姿态与跨距离保留集，时间参考与rolling shutter误差。

**候选方案/选型：** 用打印尺寸经过实测的ChArUco平板、多姿态采集和独立保留集；建立mode/installation绑定的calibration manifest。[E4] 不用一个FOV常数覆盖所有stream，不用单平面homography覆盖不同距离的人。

**契约：** `CalibrationIdentity`包含硬件身份、sensor mode、crop、尺寸、变换链、镜头/对焦标记、K/D、外参及适用时间语义；更换任一关联条件就失效。

**验收：** 单摄重投影RMS目标≤0.7px且独立保留集p95≤1.5px；双摄实际可见域LOS映射p95≤0.5°。若窄角小目标需要更严精度，按像素到角度预算收紧；不能只靠训练集拟合误差通过。

**回退：** 该相机只显示/记录，不给运动观测；保留旧标定作历史证据但不移植资格。

## 8 · 跟踪特定标记的人

**根因假设：** UUID是轨迹身份而非真实个人身份；衣着可混淆，长遮挡不能靠猜测。[U L101]

**必要测量：** 用户点选延续、多人交叉、相似衣着、标签遮挡/反光/复用、双镜同ID冲突，记录错误跟随episode而非仅检测score。

**候选方案/选型：** 首选明确可见的AprilTag+会话enrollment，点选为无需标签的降级；颜色仅弱关联证据，不做持久ReID。官方AprilTag库是候选读码器；family与版本固定，优先其推荐tagStandard41h12，已有依赖只支持其他family时记录选型差异。[E5]

**契约：** `target_session_id`、selector policy、tag family+id、局部track成员、selection generation、conflict/revoked/expired。标签被绑定到人只在空间关联唯一时成立；标签不是不可复制的身份证明。

**验收：** 标签在声明距离/光照域的重获p95≤1s；100次遮挡/交叉episode中错误替换=0，冲突测试全部返回ambiguous。尺寸门槛由实际码面像素实验决定，不承诺任意远距离可读。

**回退：** 继续短时同track关联或暂停追随/人工重选；不以另一个sole candidate自动顶替。退选/删除立即撤销旧generation并删除会话缓存。

## 9 · 脸位置与指定人

**根因假设：** body-box 22%不是脸中心；脸位置检测与“这个人是谁”是不同问题。[U L102]

**必要测量：** 正/侧/背面、多脸重叠、脸尺寸、模糊/低照、person-face关联与锚点跳变；不要只测静态正脸。

**候选方案/选型：** N1加入可关闭的**YuNet CPU selected-ROI脸定位实验**，先适配本机OpenCV版本再固定具体ONNX与SHA；不在本轮强制升级OpenCV/Hailo。官方目录提供不同形状/版本模型，不能直接跟随main默认模型。[E6] 它是脸检测而非全头检测，背头无脸时明确退body_upper/torso。持久人脸库/生物识别不在当前项目AI计划范围，本包不扩展；“指定人”由第8题的会话目标与标记实现。[R4]

**契约：** `anchor.kind=face_box/head_box/body_upper/torso/marker_center`、anchor confidence与person/association score分离，父person track、timestamp、失效时间独立。face confidence不充当identity confidence。

**验收：** 声明可见脸域内precision≥0.98、recall≥0.90，错挂邻人脸=0个观察事件；脸中心误差p95≤脸框宽10%，源切换参考满足原limit。可选ROI模型不得让wide延迟回归>10%；旧脸结果不能反复更新时间维持“新鲜”。

**回退：** 无脸/脸歧义→有效身体锚点或无观测，不伪造head_detected；模型失败可完全关闭，不影响person检测和手工选人。

## 决策关闭的含义

0–9的**架构选择**在本包中已明确；供电、端点、速度/制动、模型质量、双摄/IMU标定等**现场证据**仍未关闭。Codex应分别维护decision status与qualification status，不把“已有设计”写成“功能已验收”。

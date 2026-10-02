# 03 · reticle、时间语义与最小接口

## 1. 保留现有 reticle 几何

中心框/四角标固定在已有 optical-axis 映射位置，不能默认显示窗口 50% 就是光轴。左右线段属于穿过该中心的同一条直线，中间留既有缺口；中心框不旋转、不抬高/降低，不新增纵向十字臂。[S03]

重力提示不修改跟踪瞄点、相机外参或图像。无需新增操作者可调的“roll控制”，保留既有开发测试入口仅用于自动测试；生产值由遥测驱动。标定残余可归现有相机标定，不再加用户手调 cant 去掩盖错误符号。

## 2. 水平面到屏幕线

由**当前实际关节姿态**得到：`n_C=transpose(R_BC(q_actual))*up_B`。

针孔/已矫正图像的无限远水平线：

`l_I = inverse(K).T * n_C`，满足 `l_I^T [u,v,1]=0`。

若 A 为图像到显示面的有效平移/裁切/缩放/镜像/180°安装变换，则：

`l_D = inverse(A).T * l_I`。

实际生产已包含的 rotate_180、镜像不能再应用一次；只使用 active camera 的唯一 transform manifest。[S09] 不能取 raw IMU roll 直接乘 -1 解决显示。fx≠fy、letterbox、CSS非等比缩放也必须按映射处理。

方向向量 `t_D=(l_D.y,-l_D.x)`。屏幕 y 向下，`atan2(t_y,t_x)` 的正角为顺时针，与现有 `reticleCantDeg` 一致。线方向是 modulo π，显示时选择靠近上一有效方向的等价角，避免 +89°到-89°跳变；不要把角度硬截在±5°，倾斜较大仍须真实显示。

reticle 只取线的**方向**，平移到固定中心 `(x0,y0)`，即使用 `[a,b,-a*x0-b*y0]`。它是居中的水平倾斜提示，不是仰拍时真正地平线在画面中的位置。不得因拟合地平线而移动中心框。正倒置另显一个状态标签；一条无箭头直线不能区分180°。

未矫正镜头的畸变不能由单个矩阵 A 表示。复用现有投影/畸变函数，在 reticle 中心附近取水平方向的局部投影切线；不新增全帧去畸变管线，不声称原始广角画面中的地平线必为直线。小中心线段适用的方向误差进入同一验证。

视轴接近 ±up 时，`sqrt(n_C.x²+n_C.y²)` 接近0，水平线方向不可观测。使用水平线方向不确定性与有效投影阈值拒绝显示；默认相当于距离竖直 <2° 或传播方向误差超预算时隐藏横线，中心框保留。无效/stale/gravity epoch改变时亦隐藏，不回0°装成水平。

## 3. 复用遥测，不通过网页重新融合

新增一个控制侧权威 `GravityFrame`；webd 转发，不从其现有0.5 s缓存的 IMU诊断自行计算另一个 up_B。[S10]

逻辑结构（是拟议接口，落地允许复用现有命名）：

```text
GravityFrame {
  epoch, imu_generation, calibration_id, time_map_id,
  estimate_ns, sample_ns, published_ns, valid, reason,
  up_B[3], gravity_sigma_rad, mounting_class, tilt_rad
}
LevelSweepContext {
  gravity_epoch, geometry_id, plant_scope_id, limit_revision,
  reference_frame, requested_heading_span, effective_heading_arc,
  branch_id, phase, phase_rate, phase_acceleration,
  actual_min_clearance[2], extra_clearance[2], limiting_reason
}
HorizonCue {
  gravity_epoch, camera_id, camera_geometry_id, display_transform_id,
  pose_ns, time_basis="CONTROL_POSE", valid, reason,
  line_normal_image[2], mounting_class
}
```

无效值为 null/明确状态，不是全0向量。时间为已有 control CLOCK_MONOTONIC；浏览器不把这个值与 Date.now() 直接相减。age 沿既有服务器计算/时钟映射和本地消息到达后的增量，epoch匹配后才显示。

C++几何层输出原图中的水平线法向，web/HUD仅负责有效显示变换和共线渲染，不承担基座/IMU融合。已有遥测 codec 一次版本化/字段扩展，复用现有两语言 golden tests；不新增通信协议框架。比例/单位读回规则沿用 ADR-002.2。

## 4. 显示时间选择已经固定

本 ADR 的 reticle 表示**当前遥测姿态下的水平倾斜**，time_basis=CONTROL_POSE。沿用已有 WebSocket 状态发布率；与普通控制反馈解耦，不提高电机频率或重写 MJPEG。

浏览器 `<img>` 的 MJPEG 显示没有可信 frame_id/exposure 配对时，不能声称横线已与屏幕上那一帧完全对时。界面在需要处标“姿态水平”，不是“图像地平线锁定”。动态显示验收分开报告几何误差、姿态遥测年龄、视频年龄差及其造成的可见角差。

ADR-003 已有的曝光/姿态索引用于**离线验收**配对并解释画面差异；本 ADR 不增加帧同步视频协议。该选择不是待补终版功能；用户当前要求是水平面指示，而非录制图像的精确去旋转。若后续提出逐曝光叠加，那是另一个明确需求。

gravity资格另外绑定持续到达的IMU支持样本；canonical估计时间不能伪装成新的样本时间。控制姿态 cue 初始 freshness上限200 ms，且消息和姿态各自年龄都检查；沿用旧 UI更严格失联规则。控制侧每次原遥测发布时用最新实际 q 重算 n_C，不能只用2秒静止重力窗口时的相机角度。底座固定，up_B可锁定但 R_BC 必须随电机变化。

为抑制闪烁，允许既有显示插值在 modulo π 上按时间插值；不再加一个隐藏重低通去“美化”延迟。超期立即无效，不能继续预测过期姿态直到视觉看起来顺滑。

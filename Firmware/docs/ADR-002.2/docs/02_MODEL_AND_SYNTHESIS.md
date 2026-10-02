<!-- doc-tree-check: ignore -->
> **Historical (2026-10-02).** The model-first code, tools and operation cards this document links to were removed when ADR-002.x closed; they remain in git history before the cleanup commit. Current path: [automatic servo commissioning](../../operations/servo-commissioning.md), closing report [SERVO_COMMISSIONING_2026-10-02](../reports/SERVO_COMMISSIONING_2026-10-02.md).

# 02 · 建模、系统辨识与数学控制器计算

> **2026-10-01最新优先修订：** [辨识修复与前瞻验证](08_IDENTIFICATION_REPAIR.md)依据[架构师review](../architect_review_01/ADR-002.2-independent-review.md)，覆盖下文仅a/b/h移动模型、统一时延、短窗预测及阶段1完成的冲突规则。采用预声明执行器/机械/摩擦/测量族、整run训练/选择/最终验证、完整状态连续轨迹及闭环估计检查；只有支持域内预测合格模型冻结后才能合成，失败模型不得先做有界实机反馈探针。下文理想PI公式仅为经验证的局部特例，不代替非线性增量动力学与A/B/C验证。

以下均为设计方法[D]，不是实测参数。源于一般机械辨识方法的内容见[M1]；本包推导和数值选择是拟实施的工程约定。

Executable parent mathematics now lives in
[assembly_dynamics.py](../../../commissioning/assembly_dynamics.py): supplied two-axis
geometry defines coupled M/C/G, with cable/friction loads and actuator mapping
supplied explicitly. Independent mechanics probes do not establish station
parameters or native/production coupling. Review 02's
[recovery amendment](09_ESTIMATOR_RECOVERY.md) remains authoritative for estimation,
configuration support, synthesis and qualification.

## 1. 输入、坐标和模型的边界

输出轴角q(rad)、速度ω(rad/s)、加速度α(rad/s²)、控制q轴电流i(A)。电流模型不等于电源DC电流模型；不得用滑环供电额定值替代电机Iq约束。保留请求、限幅、最后成功TX和实际电流反馈四个不同量。

两轴分别辨识、按另一轴姿态和工况建立有限模型族，双轴同时运动验证耦合。工作范围之外不外推。惯量用电流等效系数，不强行报告未经测量的Nm或kg·m²：

\[
i_j=a_j(z,c)\dot\omega_j+b_j(z,c)\omega_j+h_j(q_j,d_j,z,c)+\epsilon_j,
\]

z为另一轴姿态，c为工况，d为运动方向；a>0，b≥0。h表示**位置/方向性总负载电流**，可包含重力、干摩擦和其他重复偏置。没有额外证据不把h的均值叫重力、差值叫纯摩擦，也不把静止保持电流直接当breakaway幅度。[M1]

h采用分方向连续分段线性表：有限区间5个等距节点；只有整圈角度映射与运动许可成立时yaw使用8个周期节点。z采用已批准范围的低/中/高三个姿态层；没有覆盖的格点不能补0。插值在有效区间内进行，重复性不足扩大不确定性而不是加密表遮盖问题。

**可辨识性约束：** h已包含bias，不额外同时拟合任意常数重力列和同一自由度的表常数；避免设计矩阵秩亏。实际坐标零必须跨会话可恢复；否则位置表需session映射确认后才能复用。

## 2. 静摩擦是独立辨识项

对每个有效姿态/位置/方向，记录起动的**总电流有向区间**[I_not_moving,I_sustained_motion]，以及已运动时h。达到电流/位移/时间边界仍未起动是删失观测，不是“已测得breakaway=上限”。起动后必须持续跟随；3个计数不能代替连续运动证据。

起动所需增量由总起动电流相对于当前h、前馈和反馈已有输出计算，不能把整份起动总电流再次叠加到已含h的指令上。状态固定为REST/START/MOVE；反转先制动再转换方向，起动结束无扰衔接。首次实现必须包含此完整逻辑，不以未来状态机补齐。

## 3. 数据与拟合

### 3.1 有约束积分回归：初始化和信息量检查

运动状态不变、同一工况窗口[t0,t1]内：

\[
\int i\,dt=a(\omega_1-\omega_0)+b(q_1-q_0)+\sum_k\beta_k\int\phi_k(q,d,z)dt.
\]

用实际时间积分，位置经去重/去回绕，速度来自经标定的测量/观察器。窗口不跨起动、换向、接触或工况切换。列归一化后SVD检查秩、条件数（默认上限10⁶）；再用有界最小二乘，a正、b非负，h系数无强制符号。不能用直接求逆正规方程。[M2]

回归有噪声自变量/闭环偏差风险，**不是最终无偏真值**。参考代码只实现该初始化的常系数特例，不冒充完整机械模型。

### 3.2 输出误差拟合：最终模型

以初始化为起点，用端到端输入（成功TX电流序列）驱动模型，联合拟合角度和已标定gyro。以静止与重复试验估得的噪声标准差白化残差，使用有界trust-region least_squares、Huber损失，有限评估预算2000；最终解须满足物理约束并优于初始化的留出预测，不盲信优化器success。[M3]

电调电流环/通讯/观测延迟单独记录。iqf为滤波反馈，不能把它当无限带宽的瞬时真实Iq。只在有效测量带宽内解释机械等效参数；无法分离的执行器/遥测动态保留在端到端模型中，不以反滤波制造虚假高频信息。

观测模型含实际因果滤波和采样。物理拟合阶段可以用已声明的离线平滑，但交付控制器的验证必须用实际实时因果观察器。若已知稳定闭环用于承重或获取数据，必须将其控制律和外部激励显式记录并进行输出误差拟合；不能把请求参考当实际输入，也不能宣称完全开环。[M1]

### 3.3 不确定性与留出

按整段run做128次固定种子block bootstrap，保留a/b/h/时延联合相关性；样本不足不声称置信区间可靠。带空间表的模型可合并重复工况先验，但不把变动前后的数据无条件混成一个静止系统。

用独立run检验自由运行轨迹、一步预测、残差时间结构、角度/方向分布。bootstrap/有限仿真只是工程鲁棒性检查，不等于对任意模型形式误差的严格稳定性证明。

## 4. 前馈

\[
i_{ff}=\hat a\alpha_{ref}+\hat b\omega_{ref}+\hat h(q,d,z,c).
\]

h使用当前可靠q，惯量项使用经过轨迹整形的α_ref。方向由参考运动意图加迟滞决定，不随噪声速度每tick翻转。起动区间产生有界短时增量而非恒定永久boost；当前工况不能识别时禁止查询任意“最近节点”外推。

不追求最小电流。前馈预计所需驱动，反馈补偿误差；总请求统一进入最终限幅/限斜率，避免前馈绕过限流。

## 5. 反馈的计算，不是实机搜索

局部模型a·ωdot+b·ω=u，速度PI：u=Kp e+I，Idot=Ki e。理想特征多项式：

\[
a s^2+(b+K_p)s+K_i.
\]

以ζ=1固定阻尼作为解析起点：

\[
K_p=2\omega_n a-b,\qquad K_i=\omega_n^2a.
\]

如果Kp<0，不静默clamp成0。该点对所选控制结构不可行；由离线计算选择满足约束的带宽。PI的闭环零点、参考前馈和实际离散实现使实际过冲不能只由ζ推断。

### 固定离线求解规则

1. 频率有效域由实际控制采样fs、辨识有效带宽fid及端到端p99时延τ决定：`wn_max=min(2πfs/20, 2πfid/5, 0.35/τ)`；τ=0只在确有零延迟模型时略去该项，未知不是0。该式是保守搜索上界设计，不是稳定性定理。
2. 固定下界`wn_min=wn_max/100`；在[wn_min,wn_max]内以256个对数间隔**离线模型点**计算上述PI和 `Kpos=wn/5`，检查实际C++离散核心仿真、留出轨迹与128个不确定性模型。无实机运行与此网格绑定。
3. 只接纳所有必需性能指标、离散极点、全部交越点的≥50°相位裕量/≥6dB增益裕量，以及输出/停止边界通过的点。不是取单个有利交越；非最小相位/多交越需完整Nyquist/离散根验证。[M4用于离散化，不代替这些设计验证]
4. 在通过点中选择满足响应要求且最大可用带宽的点；同带宽按最差跟踪/jitter评分，再按数值ID打破平局。不按最小电流排名，不保证全局最优。
5. 无可行点输出ENVELOPE_LIMITED或MODEL_INADEQUATE及失败约束；不调用实机PID网格搜索。只有模型预测已满足指标，才允许提交一个新的实机验证候选。

这一离线网格是计算工具，不是002.1那种对设备逐组试增益。允许128模型×256点数值计算，不允许128×256次设备动作。

### 必须与production一致的离散定义

积分使用梯形积分：`I += Ki*dt*(e+e_prev)/2 + Kaw*dt*(u_applied-u_requested)`，Kaw=wn；所有项单位SI。u_applied是统一输出仲裁后的可信命令，失败TX不能算成功施加。输出历史、抗饱和、无扰换参、dt异常处理与重启规则实现于共享核心。

`I_new = u_previous - Kp_new*e - i_ff_new`，在批准积分/输出域内进行无扰初始化；不能盲清积分让pitch下沉。积分边界是既有物理包络与模型需求的显式结果，不是agent手挑的隐藏新参数。

位置环无新增积分；既有reference轨迹的速度/加速度/jerk范围在实验与生产证书中绑定。参数套用到不同整形器后必须重新3b，不能称同一控制结果。

## 6. IMU与观察器必须完整交付

编码器提供输出轴位置，gyro提供机体角速度，加速度计提供specific force而非轴角加速度。测量模型固定：

`gyro = H(q) * [yaw_rate,pitch_rate]^T + bias + noise`

`accel = R^T*(a_origin + alpha×r + omega×(omega×r) - gravity) + bias + noise`

联合单轴运动求IMU安装旋转和H(q)，记录杆臂；未知杆臂时不能把线加速度当精确角加速度。角加速度由gyro/状态估计获得，积分辨识避免对量化角度做二阶差分。验证数据内同时提供gyro角速度与accelerometer振动/冲击分析；不能把accelerometer重复积分当长期稳定速度。

实时观察器采用每轴q/ω的因果状态估计，异步消费真实新RX，gyro转换为轴率后与encoder联合更新；噪声从静止/重复数据得出，过程噪声由离线创新似然在固定正值域中计算，禁止agent手调滤波窗。安装/时钟/gyro失效时用已验证的encoder-only支路；没有该支路证书则退出相应资格，不能声称融合仍正常。

控制主周期保持本机核实的200Hz，不据此宣称IMU或所有电流反馈均200Hz。旧交接书记录BNO085约50Hz且安装标定未完成，必须实测当前状态。[H1]

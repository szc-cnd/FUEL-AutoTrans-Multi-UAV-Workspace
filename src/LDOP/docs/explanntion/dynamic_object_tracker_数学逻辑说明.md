# DynamicObjectTracker 与多运动模型 Kalman 工具数学逻辑说明

本文把当前跟踪模块、运动模型和多模型 Kalman 工具，从代码流程改写成数学形式。它描述的是当前实现，而不是理想化设计：KF/EKF 组件已经按 LDOT 补齐 `CA2D / CA3D / CV3D / CTRA`，tracker 新轨迹先以 `UNKNOWN + CV3D` 初始化，达到分类条件后按类别切换到对应模型。

对应代码主要位于：

1. `include/ldop/dynamic_object_tracker.h`
2. `src/dynamic_object_tracker.cpp`
3. `include/ldop/motion_model.h`
4. `src/motion_model.cpp`
5. `include/ldop/multi_model_kalman_filter.h`
6. `src/multi_model_kalman_filter.cpp`

## 0. 代码执行路线

读 tracker 时建议先把代码分成“三层”，否则很容易在关联、生命周期和 Kalman 数学之间来回跳：

1. `DynamicObjectTracker::processDynamicTracks()` 是单帧主流程，决定本帧对每条轨迹做 `predict / update / create / coast / delete` 中的哪一种。
2. `dynamic_object_tracker.cpp` 匿名命名空间里的函数负责局部工具逻辑，例如 cost matrix、Hungarian、fallback 关联、建轨抑制和 ROS 点类型转换。
3. `KalmanFilterBase` 及其 `LinearKalmanFilter` / `ExtendedKalmanFilter` 子类只负责单条轨迹内部的 `setDt()`、`predict()`、`update()`、`initialize()`，不关心其他轨迹，也不关心 ROS 发布。

`processDynamicTracks()` 的代码顺序就是一帧跟踪顺序：

```text
detections + tracks_
  -> predictTracks(): 所有旧轨迹先预测到当前 header.stamp
  -> buildCostMatrix(): detection-track 两两算马氏距离，必要时追加 fallback 候选
  -> selectAssociations(): Hungarian 全局匹配，得到一对一 matches
  -> updateMatchedTrack(): 匹配成功的轨迹先做分类证据和模型切换，再做 Kalman update，并刷新 size/history
  -> shouldSuppressTrackSpawn() / createTrack(): 未匹配 detection 决定是否建新 ID
  -> coastTrack(): 未匹配旧轨迹进入短期漏检保留状态
  -> deleteExpiredTracks(): 删除超过漏检阈值的轨迹
  -> buildOutput() + buildTrackMarkers(): 生成对外消息和 RViz marker
```

最容易混淆的边界是：`TrackState::bbox.center` 是对外和 marker 使用的当前轨迹中心，但每次预测或更新后都会被 `filter->position()` 覆盖；`bbox.size` 不进 Kalman，只在匹配成功时按 detection 的 AABB 尺寸做指数平滑。

## 1. 模块边界

当前第二阶段跟踪链路可以写成：

$$
\mathcal{D}_k
\xrightarrow{\text{DynamicObjectTracker}}
\mathcal{T}_k
\xrightarrow{\text{buildOutput}}
\mathcal{O}_k
$$

其中：

1. $\mathcal{D}_k=\{d_j\}_{j=1}^{M_k}$ 是第 $k$ 帧的动态目标检测集合，由聚类模块输出。
2. $\mathcal{T}_k=\{T_i\}_{i=1}^{N_k}$ 是 tracker 内部维护的轨迹集合。
3. $\mathcal{O}_k$ 是对外发布的 `ldop/DynamicObjectArray`。

职责边界是：

1. `DynamicObjectTracker` 维护轨迹列表、数据关联、建轨、漏检 coast、删除、输出消息和 marker。
2. `TrackState` 是单条轨迹的运行状态，内部独占一个 `KalmanFilterBase` 实例。
3. `KalmanFilterBase` 只负责单条轨迹的状态估计，不接触 ROS I/O，不知道其他轨迹，也不做数据关联。
4. `MotionModel` 是独立的运动模型接口，当前实现包括 `CAMotionModel`、`CVMotionModel` 和 `CTRAMotionModel`。

换成数学对象：

$$
T_i =
\left(
\text{id}_i,\;
x_i,\;
P_i,\;
B_i,\;
\text{age}_i,\;
\text{hits}_i,\;
\text{miss}_i,\;
\tau_i,\;
\mathcal{H}_i,\;
\kappa_i,\;
q_i
\right)
$$

其中：

1. $\text{id}_i$ 是稳定轨迹 ID。
2. $x_i$ 是 Kalman 状态向量。
3. $P_i$ 是状态协方差。
4. $B_i$ 是轨迹当前 bbox，中心由滤波器位置覆盖，尺寸来自检测框平滑。
5. $\text{age}_i$ 是轨迹经历的总帧数。
6. $\text{hits}_i$ 是成功关联 detection 的次数。
7. $\text{miss}_i$ 是连续漏检帧数。
8. $\tau_i$ 是该轨迹上次处理时间戳。
9. $\mathcal{H}_i$ 是用于 marker 和调试的历史样本序列。
10. $\kappa_i$ 是当前确认类别，取值为 `Unknown/Human/Vehicle/Uav/Other`。
11. $q_i$ 是 `Human/Vehicle/UAV/Other` 四类证据分数。

## 2. 输入检测量

第 $k$ 帧每个 detection 记为：

$$
d_j = (\text{detection\_id}_j,\; B_j,\; n_j,\; t_k)
$$

其中 bbox 为：

$$
B_j = (c_j,\; s_j,\; \psi_j)
$$

当前实现中：

1. $c_j=(x_j,y_j,z_j)^T\in\mathbb{R}^3$ 是聚类阶段计算的点簇质心。
2. $s_j=(l_j,w_j,h_j)^T\in\mathbb{R}^3$ 是同一连通块的 AABB 尺寸。
3. $n_j$ 是 detection 中的点数，只用于临时合并/分离保护。
4. $\psi_j=0$，当前没有引入有向框。

跟踪滤波只把中心 $c_j$ 当作 3D 位置观测：

$$
z_j = c_j
$$

尺寸 $s_j$ 不进入 Kalman 状态，只在轨迹 bbox 中做指数平滑。

## 3. 时间步长处理

对已有轨迹 $T_i$，当前帧时间为 $t_k$，轨迹上次时间为 $\tau_i$。代码先计算预测步长：

$$
\Delta t_i =
\begin{cases}
\Delta t_{\text{default}}, & t_k=0\ \text{or}\ \tau_i=0 \\
t_k-\tau_i, & t_k-\tau_i>0 \\
\Delta t_{\text{default}}, & t_k-\tau_i\le 0
\end{cases}
$$

然后做上限裁剪：

$$
\Delta t_i \leftarrow \min(\Delta t_i,\; \Delta t_{\max})
$$

对应参数：

1. `tracking_default_dt`：$\Delta t_{\text{default}}$
2. `tracking_max_dt`：$\Delta t_{\max}$

这个裁剪很重要：如果点云时间戳跳变或长时间暂停，滤波器不会一次性外推一个很大的距离，而是最多按 $\Delta t_{\max}$ 做一步预测。

## 4. 多运动模型状态定义

类别到运动模型的映射为：

$$
\text{Human}\rightarrow\text{CA2D},\quad
\text{Vehicle}\rightarrow\text{CTRA},\quad
\text{UAV}\rightarrow\text{CA3D},\quad
\text{Other}\rightarrow\text{CV3D}
$$

`Unknown` 不触发模型切换，新轨迹默认使用 `CV3D`。这和 LDOT 的“新目标先按 else/CV 初始化”一致，但对外消息在分类前发布 `CLASS_UNKNOWN`，避免把内部默认模型误解成已经确认是 `Other`。

### 4.1 CA3D 状态定义

当前 `MotionModelType::CA3D` 的状态是 9 维：

$$
x =
\begin{bmatrix}
p \\
v \\
a
\end{bmatrix}
=
\begin{bmatrix}
p_x & p_y & p_z & v_x & v_y & v_z & a_x & a_y & a_z
\end{bmatrix}^T
$$

其中：

1. $p\in\mathbb{R}^3$ 是目标中心位置。
2. $v\in\mathbb{R}^3$ 是目标中心速度。
3. $a\in\mathbb{R}^3$ 是目标中心加速度。

观测只测位置：

$$
z =
\begin{bmatrix}
z_x & z_y & z_z
\end{bmatrix}^T
$$

观测矩阵为：

$$
H =
\begin{bmatrix}
I_3 & 0_3 & 0_3
\end{bmatrix}
$$

所以预测状态到观测空间的映射是：

$$
\hat{z}=Hx=p
$$

## 5. CA3D 状态转移矩阵

constant-acceleration 离散模型假设在一个时间步 $\Delta t$ 内加速度近似保持常量：

$$
p_k = p_{k-1} + v_{k-1}\Delta t + \frac{1}{2}a_{k-1}\Delta t^2
$$

$$
v_k = v_{k-1} + a_{k-1}\Delta t
$$

$$
a_k = a_{k-1}
$$

写成矩阵：

$$
x_k^- = F(\Delta t)x_{k-1}^+
$$

其中：

$$
F(\Delta t)=
\begin{bmatrix}
I_3 & \Delta t I_3 & \frac{1}{2}\Delta t^2 I_3 \\
0_3 & I_3 & \Delta t I_3 \\
0_3 & 0_3 & I_3
\end{bmatrix}
$$

代码中 `transition_` 的非零块就是：

$$
F_{p,v}=\Delta t I_3,\quad
F_{p,a}=\frac{1}{2}\Delta t^2 I_3,\quad
F_{v,a}=\Delta t I_3
$$

## 6. 过程噪声 Q

当前 CA3D 模型使用白 jerk 驱动的离散过程噪声。设 jerk 噪声方差为：

$$
q = \sigma_j^2
$$

对应参数：

$$
\sigma_j = \texttt{tracking\_ca\_jerk\_sigma}
$$

单轴状态为：

$$
x^{(1D)}=[p,\ v,\ a]^T
$$

单轴过程噪声模板是：

$$
Q_{\text{axis}}(\Delta t)=q
\begin{bmatrix}
\frac{\Delta t^5}{20} & \frac{\Delta t^4}{8} & \frac{\Delta t^3}{6} \\
\frac{\Delta t^4}{8} & \frac{\Delta t^3}{3} & \frac{\Delta t^2}{2} \\
\frac{\Delta t^3}{6} & \frac{\Delta t^2}{2} & \Delta t
\end{bmatrix}
$$

因为完整状态按 $[p_x,p_y,p_z,v_x,v_y,v_z,a_x,a_y,a_z]$ 分块排列，所以完整基础过程噪声可以写成：

$$
Q_0(\Delta t)=q
\begin{bmatrix}
\frac{\Delta t^5}{20}I_3 & \frac{\Delta t^4}{8}I_3 & \frac{\Delta t^3}{6}I_3 \\
\frac{\Delta t^4}{8}I_3 & \frac{\Delta t^3}{3}I_3 & \frac{\Delta t^2}{2}I_3 \\
\frac{\Delta t^3}{6}I_3 & \frac{\Delta t^2}{2}I_3 & \Delta t I_3
\end{bmatrix}
$$

这个矩阵表达的是：

1. jerk 噪声会积分到加速度、速度和位置。
2. 时间步长越大，位置方差会按 $\Delta t^5$ 量级增长。
3. 三个坐标轴之间暂不建模相关性，因此非同轴的交叉项为 0。

## 7. 测量噪声 R

位置观测噪声方差为：

$$
r=\texttt{tracking\_ca\_position\_noise}
$$

基础测量噪声矩阵是：

$$
R_0=rI_3
$$

含义是：

1. detection center 的 $x/y/z$ 三轴测量方差相同。
2. 三轴测量误差暂不建模相关性。
3. bbox 尺寸误差不在 $R$ 中，因为尺寸不作为 Kalman 观测。

## 8. 新轨迹初始化

创建新轨迹时，`TrackState` 使用 `MotionModelType::CV3D` 创建滤波器，类别为 `Unknown`。CV3D 状态为：

$$
x_0 =
\begin{bmatrix}
c_j\\0
\end{bmatrix}
$$

其中 $c_j$ 是首帧 detection 质心，速度从 0 开始。初始化同时保存：

$$
\kappa_i=\text{Unknown},\quad q_i=[0,0,0,0]^T,\quad s_i^{max}=s_j,\quad n_i^{ref}=n_j
$$

CV3D 初始协方差来自 `kalman_filter/cv_model/init_cov`。CA2D、CA3D 和 CTRA 的初始协方差分别来自 `kalman_filter/ca_model/human/init_cov`、`kalman_filter/ca_model/uav/init_cov` 和 `kalman_filter/ctra_model/init_cov`。

为了说明 CA3D 的协方差结构，若后续类别确认为 UAV 并切到 CA3D，初始协方差可写成分块对角矩阵：

$$
P_0 =
\begin{bmatrix}
c_p I_3 & 0 & 0 \\
0 & c_v I_3 & 0 \\
0 & 0 & c_a I_3
\end{bmatrix}
$$

对应参数：

1. $c_p=\texttt{tracking\_ca\_initial\_position\_cov}$
2. $c_v=\texttt{tracking\_ca\_initial\_velocity\_cov}$
3. $c_a=\texttt{tracking\_ca\_initial\_acceleration\_cov}$

`createTrack()` 调用：

$$
\texttt{filter.initialize}(c_j)
$$

当前 `initialize()` 不是一次 Kalman update，而是直接把首帧 detection 中心写入当前模型的位置状态：

$$
x_0 =
\begin{bmatrix}
c_j\\0\\0
\end{bmatrix}
$$

其中位置为点簇质心，速度和加速度仍从 0 开始。这样做的原因是避免“零状态 + 首帧 update”在较大测量噪声下把新轨迹中心拉向原点或旧参考点。初始协方差仍然由配置控制。

$$
c_p=0.1,\quad c_v=1.0,\quad c_a=10.0
$$

也就是说，新轨迹的位置一开始就等于观测中心，但速度/加速度的不确定性仍然保留给后续帧逐步估计。

## 9. Kalman 预测

对每条已有轨迹，在数据关联前先预测到当前帧：

$$
x_i^- = F_i x_i^+
$$

$$
P_i^- = F_i P_i^+ F_i^T + Q_i
$$

其中：

1. 对 `CA2D/CA3D/CV3D` 线性模型，$F_i=F(\Delta t_i)$，状态预测直接使用 $F_i x_i^+$。
2. 对 `CTRA` EKF，状态预测使用非线性函数 $x_i^- = f_i(x_i^+)$，协方差传播仍使用局部线性化矩阵：

$$
F_i=\left.\frac{\partial f_i}{\partial x}\right|_{x_i^+}
$$

也就是说，CTRA 的转移雅可比在预测前状态 $x_i^+$ 处计算，再用 `stateTransition()` 得到 $x_i^-$。这样协方差沿当前状态的运动切线传播，不会把 yaw/yaw_rate 先推进后再线性化。

3. $Q_i$ 是当前滤波器持有的过程噪声矩阵 `process_noise_`；启用自适应噪声时，它来自上一轮 update 后保留下来的自适应估计。

在当前 tracker 调用路径中，每帧 `predictTracks()` 都会先调用：

$$
\texttt{setDt}(\Delta t_i)
$$

而 `setDt()` 只负责把当前步长写入运动模型，不在这里重置 Q/R。真正进入预测公式的过程噪声矩阵由滤波器当前持有的 `process_noise_` 决定；当关闭自适应噪声时，`predict()` 会显式读取当前 $\Delta t$ 下的基础 $Q_0(\Delta t)$。因此当前预测阶段的有效形式可以理解为：

$$
P_i^- = F_i P_i^+ F_i^T + Q_i^{adaptive}
$$

其中 $Q_i^{adaptive}$ 由后续观测更新阶段基于 innovation 统计估计，并保留到下一次 predict 使用。

## 10. 数据关联代价

预测完成后，对每个 detection-track 组合计算 3D 位置马氏距离平方。

设 detection 中心为：

$$
z_j=c_j
$$

轨迹预测位置为：

$$
\hat{p}_i=H x_i^-
$$

位置 innovation 为：

$$
\nu_{j,i}=z_j-\hat{p}_i
$$

取预测协方差的位置块：

$$
P_{p,i}=P_i^-(0:2,0:2)
$$

代码会先强制位置协方差对称，并加入很小的正则项：

$$
\tilde{P}_{p,i}
=
\frac{1}{2}(P_{p,i}+P_{p,i}^T)
+\epsilon I_3
$$

其中：

$$
\epsilon=10^{-6}
$$

关联代价为：

$$
c_{j,i}=\nu_{j,i}^T \tilde{P}_{p,i}^{-1}\nu_{j,i}
$$

这个量是马氏距离平方。它不是普通欧氏距离，而是会被轨迹当前的位置不确定性调节：

1. 如果轨迹位置协方差小，同样的中心偏差会得到更大的代价。
2. 如果轨迹已经 coast，位置协方差通常会变大，同样的中心偏差会更容易通过 gate。

## 11. 卡方 gate

因为观测维度是 3，代码用 3 自由度卡方分布把置信度转换为 gate：

$$
\gamma =
F^{-1}_{\chi^2_3}(\alpha)
$$

其中：

1. $\alpha=\texttt{tracking\_association\_gate\_confidence}$。
2. $F^{-1}_{\chi^2_3}$ 是 3 自由度卡方分布分位数函数。

普通轨迹 gate：

$$
c_{j,i}\le\gamma
$$

处于漏检 coast 状态的轨迹，即 $\text{miss}_i>0$，使用放宽 gate：

$$
c_{j,i}\le \gamma\cdot \rho_{\text{coast}}
$$

其中：

$$
\rho_{\text{coast}}=\texttt{tracking\_coasting\_gate\_relax\_factor}
$$

如果组合不满足 gate，cost matrix 中写入 `invalid_cost`，后续 Hungarian 匹配不会把它作为真实合法匹配返回。

## 12. Hungarian 全局匹配

设当前帧 detection 数量为 $M$，已有轨迹数量为 $N$。先构造原始代价矩阵：

$$
C\in\mathbb{R}^{M\times N}
$$

其中：

$$
C_{j,i} =
\begin{cases}
c_{j,i}, & c_{j,i}\ \text{通过 gate} \\
C_{\text{invalid}}, & \text{否则}
\end{cases}
$$

代码再把它扩展为方阵，维度为：

$$
D=M+N
$$

扩展方阵的作用是让 Hungarian 算法可以同时表达：

1. detection 匹配真实 track。
2. detection 没有匹配任何 track。
3. track 没有匹配任何 detection。

dummy 匹配代价为：

$$
C_{\text{unmatched}}=\frac{1}{2}C_{\text{invalid}}
$$

Hungarian 算法求解：

$$
\pi^*=\arg\min_{\pi}\sum_{r=1}^{D}\tilde{C}_{r,\pi(r)}
$$

其中 $\pi$ 是一个一对一分配。最后代码只保留满足下面条件的真实匹配：

$$
0\le j<M,\quad 0\le i<N,\quad C_{j,i}<C_{\text{invalid}}
$$

所以 dummy 匹配、未通过 gate 的组合都会在输出 matches 时被过滤。

## 13. 匹配成功后的分类、模型切换和轨迹更新

如果 detection $d_j$ 匹配轨迹 $T_i$，先更新分类保护和类别证据。当前 detection center 是点簇质心，不是 AABB 几何中心。

### 13.1 临时合并/分离保护

tracker 保存历史稳定尺寸 $s_i^{max}$ 和参考点数 $n_i^{ref}$。当前帧计算：

$$
r_s=\max_a\frac{s_{j,a}}{\max(s_{i,a}^{max},\epsilon)}
$$

$$
r_n=\frac{n_j}{\max(n_i^{ref},1)}
$$

如果 $r_s$ 或 $r_n$ 突然大于 $1+\rho$，先视为临时合并；如果小于 $1-\rho$，先视为临时分离。临时状态下分类使用 $s_i^{max}$，并冻结类别证据分数。只有变化持续超过 `classification_size_change_confirm_frames` 后，才接受新的稳定尺寸和参考点数。

### 13.2 类别证据积分

轨迹命中数达到 `classification_start_frame` 后，可靠匹配帧执行 bbox/质心分类，得到当前观测类别 $\hat{\kappa}$。类别分数更新为：

$$
q_i \leftarrow \lambda q_i
$$

$$
q_{i,\hat{\kappa}} \leftarrow q_{i,\hat{\kappa}} + \eta
$$

其中：

1. $\lambda=\texttt{classification\_score\_decay}$。
2. $\eta=\texttt{classification\_score\_increment}$。

当最高分满足：

$$
\max(q_i)\ge \texttt{classification\_confirm\_score}
$$

且领先第二名至少：

$$
\texttt{classification\_switch\_margin}
$$

才确认类别。coasting 帧、临时合并帧和疑似遮挡残片帧不更新 $q_i$；短时遮挡不会把缺观测当成反证。

### 13.3 模型切换

如果确认类别对应的模型和当前模型不同，tracker 在本帧 Kalman update 前切换 filter。状态迁移只保留稳定的公共物理量：位置和世界坐标系速度。对 CTRA，若水平速度足够大，用速度方向生成 yaw；否则从旧 CTRA 状态保留 yaw 或使用 0。

切换后调用 `initializeState()` 写入新模型完整状态，并重置为新模型的初始协方差。这样不直接搬运旧协方差，避免不同状态维度之间产生虚假的相关性。

### 13.4 Kalman update

模型确认或保持后，再用 detection center 做 Kalman update。

预测观测：

$$
\hat{z}=Hx_i^-
$$

innovation：

$$
y=z_j-\hat{z}
$$

innovation covariance：

$$
S=H P_i^- H^T + R_i
$$

Kalman gain：

$$
K=P_i^-H^TS^{-1}
$$

状态更新：

$$
x_i^+=x_i^-+Ky
$$

协方差使用 Joseph 形式：

$$
P_i^+
=(I-KH)P_i^-(I-KH)^T + KR_iK^T
$$

Joseph 形式比简单的 $(I-KH)P$ 更稳，更容易保持协方差半正定和对称。

然后更新 bbox：

1. bbox 中心由滤波后位置覆盖：

$$
B_i.center \leftarrow p_i^+
$$

2. bbox 尺寸做指数平滑：

$$
s_i^+ =
\beta s_j + (1-\beta)s_i^-
$$

其中：

$$
\beta=\texttt{tracking\_box\_size\_smoothing\_alpha}
$$

尺寸不进入 Kalman 状态，原因是点云稀疏、遮挡和视角变化会让 AABB 尺寸比中心更抖；当前实现只平滑它，不用它反向影响运动状态。

最后更新生命周期计数：

$$
\text{age}_i\leftarrow \text{age}_i+1
$$

$$
\text{hits}_i\leftarrow \text{hits}_i+1
$$

$$
\text{miss}_i\leftarrow 0
$$

并把 `last_stamp` 改为当前帧时间。

## 14. 自适应测量噪声与过程噪声

每次 Kalman update 完成后，代码用本次 innovation 更新自适应噪声估计。设本次 innovation 为：

$$
y=[y_x,y_y,y_z]^T
$$

维护最近 $W$ 个完整 innovation 向量：

$$
\mathcal{Y}_k=\{y_{k-W+1},\dots,y_k\}
$$

其中：

$$
W=\texttt{tracking\_adaptive\_window\_size}
$$

样本协方差：

$$
C_{\gamma,k}=\frac{1}{|\mathcal{Y}_k|}\sum_{y\in\mathcal{Y}_k}yy^T
$$

先估计测量噪声：

$$
\hat{R}_k=C_{\gamma,k}-HP_k^-H^T
$$

然后只取对角项，并做非负截断、EMA 和下限约束：

$$
R_k(i,i)=
\max\left(
R_{\min}(i,i),
\alpha_R\max(0,\hat{R}_k(i,i)) + (1-\alpha_R)R_{k-1}(i,i)
\right)
$$

再用更新后的 $R_k$ 和临时增益 $K_k^{temp}$ 估计过程噪声：

$$
\hat{Q}_k=
K_k^{temp}
\left(
C_{\gamma,k}-H(P_k^- - Q_{k-1})H^T-R_k
\right)
\left(K_k^{temp}\right)^T
$$

同样只更新对角项：

$$
Q_k(i,i)=
\max\left(
Q_{\min}(i,i),
\alpha_Q\max(0,\hat{Q}_k(i,i)) + (1-\alpha_Q)Q_{k-1}(i,i)
\right)
$$

对应参数：

1. $\alpha_Q=\texttt{kalman\_filter/adaptive\_alpha}$
2. $\alpha_R=\texttt{kalman\_filter/adaptive\_r\_alpha}$
3. $Q_{\min}=Q_0\cdot\texttt{kalman\_filter/adaptive\_min\_noise\_ratio}$
4. $R_{\min}=R_0\cdot\texttt{kalman\_filter/adaptive\_min\_noise\_ratio}$

如果 `kalman_filter/adaptive_window_size <= 1`，则自适应噪声关闭：

$$
Q_k=Q_0,\quad R_k=R_0
$$

实现细节再强调一次：自适应 `R_k` 会立即作用在当前这次 update；自适应 `Q_k` 会保留到下一次 predict 使用。当前实现和 LDOT 一样，`setDt()` 不会把已经估计出来的 Q 重新按基础 $Q_0(\Delta t)$ 映射成“统一缩放比”。

## 15. 协方差 clamp

每次 predict 和 update 后都会执行 `clampCovariance()`。

第一步，强制对称：

$$
P\leftarrow \frac{1}{2}(P+P^T)
$$

第二步，对每个状态维度的对角线做非负约束：

$$
P_{ll}\leftarrow \max(P_{ll},0)
$$

第三步，对位置、速度、加速度三类对角线分别应用上限：

$$
L_l =
\begin{cases}
L_p, & l\in p_x,p_y,p_z \\
L_v, & l\in v_x,v_y,v_z \\
L_a, & l\in a_x,a_y,a_z
\end{cases}
$$

其中：

1. $L_p=\texttt{tracking\_max\_pos\_cov}$
2. $L_v=\texttt{tracking\_max\_vel\_cov}$
3. $L_a=\texttt{tracking\_max\_acc\_cov}$

如果某个对角线超过上限：

$$
P_{ll}>L_l
$$

则记录缩放系数：

$$
s_l=\sqrt{\frac{L_l}{\max(P_{ll},10^{-9})}}
$$

否则：

$$
s_l=1
$$

最后统一左右乘：

$$
P\leftarrow DPD
$$

其中：

$$
D=\operatorname{diag}(s_1,\dots,s_9)
$$

这样做不是只截断对角线，而是同步缩放相关项，尽量保留协方差块内部的相对相关结构。

## 16. 未匹配 detection 的建轨抑制

Hungarian 匹配后，未匹配 detection 不一定立即创建新轨迹。代码会先检查它是否仍然靠近已有轨迹，以减少目标中心抖动、bbox 抖动或短时遮挡造成的裂轨。

对未匹配 detection $d_j$ 和已有轨迹 $T_i$，先计算三维中心距离：

$$
d_{3d}(j,i)
=
\sqrt{
(c_{j,x}-c_{i,x})^2+
(c_{j,y}-c_{i,y})^2+
(c_{j,z}-c_{i,z})^2
}
$$

如果：

$$
d_{3d}(j,i)\le d_{\text{sup}}
$$

则抑制建轨。其中：

$$
d_{\text{sup}}=\texttt{tracking\_spawn\_suppression\_distance}
$$

代码还计算水平 AABB IoU。设两个水平矩形面积分别为 $A_j,A_i$，交集面积为 $A_{\cap}$，并集面积为：

$$
A_{\cup}=A_j+A_i-A_{\cap}
$$

水平 IoU：

$$
\operatorname{IoU}_{xy}(j,i)=\frac{A_{\cap}}{A_{\cup}}
$$

如果：

$$
\operatorname{IoU}_{xy}(j,i)\ge \eta_{\text{sup}}
$$

也抑制建轨。其中：

$$
\eta_{\text{sup}}=\texttt{tracking\_spawn\_suppression\_iou\_threshold}
$$

两个判据任意一个成立，就不会为该 detection 创建新 ID：

$$
\text{suppressed}
=
\left(d_{3d}\le d_{\text{sup}}\right)
\lor
\left(\operatorname{IoU}_{xy}\ge \eta_{\text{sup}}\right)
$$

如果两个参数都配置为 0，则对应抑制判据关闭。

## 17. 未匹配 detection 创建新轨迹

如果 detection 未匹配，且没有被建轨抑制挡住，则创建新轨迹：

$$
\text{id}_{new}=\texttt{next\_track\_id}
$$

然后：

$$
\texttt{next\_track\_id}\leftarrow \texttt{next\_track\_id}+1
$$

新轨迹初始化步骤是：

1. 构造 `TrackState`。
2. 通过 `createKalmanFilter()` 创建内部 `KalmanFilterBase` 实例。
3. 调用 `initialize(detection.bbox.center)`，直接把首帧 detection center 写入位置状态。
4. bbox 取 detection bbox。
5. bbox center 再用滤波器位置覆盖。
6. 设置生命周期：

$$
\text{age}=1,\quad \text{hits}=1,\quad \text{miss}=0
$$

7. 写入一条 matched history sample。

## 18. 未匹配 track 的 coast

对没有匹配任何 detection 的轨迹，代码不会立即删除，而是进入 coast 状态。

注意：coast 之前，本帧开头已经对所有 track 执行过一次 predict。因此 coast 阶段不再额外预测，只更新生命周期和历史：

$$
\text{age}_i\leftarrow \text{age}_i+1
$$

$$
\text{miss}_i\leftarrow \text{miss}_i+1
$$

$$
\tau_i\leftarrow t_k
$$

$$
B_i.center\leftarrow Hx_i^-
$$

并追加一条：

$$
\text{matched}=false
$$

的历史样本。

coast 的直观含义是：

1. 轨迹短期没被观测到，但仍按运动模型保持存在。
2. 下一帧数据关联时，$\text{miss}_i>0$ 会触发更宽的 gate。
3. 如果后续重新匹配成功，`missed_frames` 会清零。

## 19. 轨迹删除

轨迹删除规则是：

$$
\text{delete}(T_i)
\iff
\text{miss}_i > M_{\max}
$$

其中：

$$
M_{\max}=\texttt{tracking\_max\_missed\_frames}
$$

注意这里是严格大于，不是大于等于。因此如果参数为 5，轨迹会在连续漏检计数变成 6 时被删除。

## 20. 对外发布条件

`buildOutput()` 会先按轨迹 ID 排序，然后只发布命中次数达到阈值的轨迹：

$$
\text{publish}(T_i)
\iff
\text{hits}_i\ge H_{\min}
$$

其中：

$$
H_{\min}=\texttt{tracking\_min\_hits\_to\_publish}
$$

输出消息中每个目标为：

$$
o_i =
\left(
\text{id}_i,\;
B_i.size,\;
x_i,\;
P_i,\;
c_i,\;
m_i
\right)
$$

其中 $x_i$ 是当前运动模型的原生状态，$P_i$ 是同一模型状态下的协方差，$c_i$ 是目标类别，$m_i$ 是运动模型类型。当前所有模型都把位置放在状态前三维：

$$
p_i=x_i(0:2)
$$

速度不再作为独立消息字段发布，而是由下游按 $m_i$ 解释：

$$
\begin{aligned}
\text{CA2D}:&\quad v_i=(x_i(3),x_i(4),0)\\
\text{CA3D/CV3D}:&\quad v_i=(x_i(3),x_i(4),x_i(5))\\
\text{CTRA}:&\quad v_i=(x_i(3)\cos x_i(5),x_i(3)\sin x_i(5),0)
\end{aligned}
$$

`model_covariance` 按当前模型状态维度做 row-major 展开。因此 CA2D、CA3D、CV3D 和 CTRA 的协方差长度不同；下游必须结合 `motion_model_type` 判断矩阵维度，不能把它当成固定 6x6 公共协方差。

类别字段 $c_i$ 来自 tracker 内部稳定后的分类状态；未确认前为 `CLASS_UNKNOWN`。

当前输出不包含：

1. acceleration
2. prediction horizon
3. track confidence

另外，`buildOutput()` 不检查 `missed_frames == 0`。所以只要轨迹还没被删除，且历史命中次数达到阈值，coasting 轨迹也可能继续出现在 `/ldop/dynamic_objects` 中。

## 21. 轨迹历史与 marker

每次 matched 或 coast 后都会追加历史样本：

$$
h_k=(t_k,\;c_k,\;m_k,\;x_k,\;P_k,\;\text{matched}_k)
$$

其中 $x_k/P_k$ 与当前 `DynamicObject.msg` 的 `model_state/model_covariance` 语义一致，表示模型原生状态和协方差。这样 history 能直接服务后续预测模块；代价是消费者需要按 `motion_model_type` 解释状态布局。

历史长度上限为：

$$
|\mathcal{H}_i|\le H_{\text{history}}
$$

其中：

$$
H_{\text{history}}=\texttt{tracking\_history\_size}
$$

如果超过上限，删除最旧样本：

$$
\mathcal{H}_i
\leftarrow
\text{last}_{H_{\text{history}}}(\mathcal{H}_i)
$$

marker 输出包括：

1. `DELETEALL`：每帧先清空旧 marker。
2. `dynamic_track_history`：历史轨迹线，至少 2 个点才发布。
3. `dynamic_track_head`：当前轨迹头部球体。
4. `dynamic_track_label`：目标上方文字，显示轨迹 ID、类别和速度模长。

如果轨迹处于 coast 状态，即：

$$
\text{miss}_i>0
$$

marker 透明度会降低，用于可视化区分“当前帧未匹配到观测”的轨迹。

## 22. 一帧完整流程

把 `processDynamicTracks()` 写成数学化伪代码：

```text
输入：当前帧 header、detections D_k
状态：上一帧后保留的 tracks T_{k-1}

1. 对每条已有轨迹 T_i：
   1.1 根据 header.stamp 和 track.last_stamp 计算 dt_i
   1.2 setDt(dt_i)
   1.3 predict: 线性模型用 x_i^- = F_i x_i^+；CTRA EKF 用 x_i^- = f_i(x_i^+)，且 F_i 在 x_i^+ 处线性化；P_i^- = F_i P_i^+ F_i^T + Q_i
   1.4 bbox.center = H x_i^-

2. 对每个 detection-track 组合：
   2.1 计算 innovation nu_{j,i} = z_j - H x_i^-
   2.2 计算马氏距离平方 c_{j,i}
   2.3 用卡方 gate 过滤非法组合

3. 用 Hungarian 算法求全局最小匹配。

4. 对匹配成功的 detection-track：
   4.1 Kalman update
   4.2 平滑 bbox size
   4.3 hits += 1, age += 1, missed_frames = 0
   4.4 追加 matched history

5. 对未匹配 detection：
   5.1 若贴近已有轨迹或水平 IoU 过高，则抑制建轨
   5.2 否则创建新 TrackState 和新 filter

6. 对未匹配 track：
   6.1 age += 1
   6.2 missed_frames += 1
   6.3 追加 unmatched/coast history

7. 删除 missed_frames > max_missed_frames 的轨迹。

8. 构建输出：
   8.1 hits 不足阈值的轨迹不发布
   8.2 发布 id、size、model_state、model_covariance、object_class、motion_model_type
   8.3 构建 history/head markers
```

## 23. 参数到数学量的对应关系

| 参数 | 数学符号 | 含义 |
| --- | --- | --- |
| `tracking_default_dt` | $\Delta t_{\text{default}}$ | 时间戳无效或非递增时的回退步长 |
| `tracking_max_dt` | $\Delta t_{\max}$ | 单帧预测步长上限 |
| `tracking_association_gate_confidence` | $\alpha$ | 3D 卡方 gate 置信度 |
| `tracking_coasting_gate_relax_factor` | $\rho_{\text{coast}}$ | coast 轨迹 gate 放宽倍数 |
| `tracking_box_size_smoothing_alpha` | $\beta$ | bbox 尺寸 EMA 平滑系数 |
| `tracking_spawn_suppression_distance` | $d_{\text{sup}}$ | 马氏距离 gate 失败后的第一层回退匹配阈值：distance 先过，才继续看 IoU；同时也是最终未匹配 detection 的建轨抑制阈值 |
| `tracking_spawn_suppression_iou_threshold` | $\eta_{\text{sup}}$ | 马氏距离 gate 失败后的第二层回退匹配阈值：只有 distance 已经过阈值时，才检查 IoU；同时也是最终未匹配 detection 的建轨抑制阈值 |
| `tracking_max_missed_frames` | $M_{\max}$ | 删除轨迹前允许的连续漏检帧数 |
| `tracking_min_hits_to_publish` | $H_{\min}$ | 对外发布前需要的命中次数 |
| `tracking_history_size` | $H_{\text{history}}$ | 每条轨迹保留的历史样本数；每个样本保存模型原生状态和协方差 |
| `kalman_filter/adaptive_window_size` | $W$ | innovation 向量统计窗口 |
| `kalman_filter/adaptive_alpha` | $\alpha_Q$ | 自适应 Q 缩放 EMA 系数 |
| `kalman_filter/adaptive_r_alpha` | $\alpha_R$ | 自适应 R 缩放 EMA 系数 |
| `kalman_filter/adaptive_min_noise_ratio` | $\rho_{\min}$ | 自适应噪声缩放下限 |
| `kalman_filter/max_pos_cov` | $L_p$ | 位置协方差对角线上限 |
| `kalman_filter/max_vel_cov` | $L_v$ | 速度协方差对角线上限 |
| `kalman_filter/max_acc_cov` | $L_a$ | 加速度协方差对角线上限 |
| `kalman_filter/ca_model/human/jerk_sigma` | $\sigma_{j,h}$ | Human CA2D 模型 jerk 噪声标准差 |
| `kalman_filter/ca_model/human/init_cov` | $P_{0,h}$ | Human CA2D 新轨迹初始协方差对角线 |
| `kalman_filter/ca_model/human/meas_noise` | $R_h$ | Human CA2D 三轴位置观测噪声方差 |
| `kalman_filter/ca_model/human/z_process_noise` | $q_{z,h}$ | Human CA2D 的 z 位置过程噪声 |
| `kalman_filter/ca_model/uav/jerk_sigma` | $\sigma_{j,u}$ | UAV CA3D 模型 jerk 噪声标准差 |
| `kalman_filter/ca_model/uav/init_cov` | $P_{0,u}$ | UAV CA3D 新轨迹初始协方差对角线 |
| `kalman_filter/ca_model/uav/meas_noise` | $R_u$ | UAV CA3D 三轴位置观测噪声方差 |
| `kalman_filter/cv_model/acc_sigma` | $\sigma_a$ | Other CV3D 模型加速度噪声标准差 |
| `kalman_filter/cv_model/init_cov` | $P_{0,o}$ | Other CV3D 新轨迹初始协方差对角线 |
| `kalman_filter/cv_model/meas_noise` | $R_o$ | Other CV3D 三轴位置观测噪声方差 |
| `kalman_filter/ctra_model/init_cov` | $P_{0,v}$ | Vehicle CTRA 新轨迹初始协方差对角线 |
| `kalman_filter/ctra_model/process_noise` | $Q_v$ | Vehicle CTRA 过程噪声对角线 |
| `kalman_filter/ctra_model/meas_noise` | $R_v$ | Vehicle CTRA 三轴位置观测噪声方差 |
| `classification_start_frame` | $H_{\text{cls}}$ | 开始给类别证据积分的最小命中数 |
| `classification_score_decay` | $\lambda$ | 类别证据分数衰减系数 |
| `classification_score_increment` | $\eta$ | 当前观测类别的证据增量 |
| `classification_confirm_score` | $q_{\min}$ | 确认类别所需的最高分阈值 |
| `classification_switch_margin` | $m_{\text{cls}}$ | 最高类别分数领先第二名的最小 margin |
| `classification_size_change_ratio` | $\rho_s$ | bbox 尺寸临时合并/分离比例阈值 |
| `classification_point_count_change_ratio` | $\rho_n$ | 点数临时合并/分离比例阈值 |
| `classification_size_change_confirm_frames` | $K_s$ | 接受尺寸/点数持续变化所需帧数 |

## 24. 当前实现的关键语义

1. 当前“多模型”不是 IMM。代码没有多个模型概率、模型混合、模型转移矩阵或多模型融合。
2. KF/EKF 层已经实现 `CA2D / CA3D / CV3D / CTRA`；当前 tracker 新轨迹默认 `UNKNOWN + CV3D`，类别证据确认后再切到 `CA2D/CTRA/CA3D/CV3D`。
3. 跟踪关联默认使用位置马氏距离平方；只有马氏距离被 gate 拒绝时，才会退回到“先 distance、后 IoU”的两层阈值做次优匹配。
4. bbox IoU 当前不参与主匹配 cost，只作为 distance 已通过后的第二层回退条件，以及最终未匹配 detection 的建轨抑制条件。
5. bbox size 不进入 Kalman 状态，只做指数平滑。
6. 新轨迹会直接用首帧 detection center（点簇质心）初始化滤波器位置状态，速度与加速度仍从零起步。
7. coasting 轨迹冻结类别证据，不因为缺少观测而降低类别分数或切回 `Unknown`。
8. coasting 轨迹会继续发布，前提是它已经满足 hits 阈值且尚未超过漏检删除阈值。
9. `max_missed_frames` 的删除条件是 `missed_frames > max_missed_frames`。
10. 自适应 R 会影响当前与后续观测更新；自适应 Q 会持续影响后续预测，并在当前 `update()` 内同步修正本次使用的 `P_{pred}`。

## 25. 一句话总结

当前跟踪模块可以概括为：

$$
\text{点簇质心观测}
\xrightarrow{\text{当前类别模型 predict}}
\text{预测轨迹}
\xrightarrow{\chi^2\ \text{gate}+\text{Hungarian}}
\text{全局关联}
\xrightarrow{\text{分类证据/必要时切模型}}
\text{matched update}
\xrightarrow{\text{create/coast/delete/output}}
\text{稳定 ID、类别、平滑 bbox、模型状态与不确定性}
$$

其中 `DynamicObjectTracker` 负责“哪条 detection 属于哪条轨迹，以及轨迹是否存在”，`KalmanFilterBase` / `LinearKalmanFilter` / `ExtendedKalmanFilter` 负责“单条轨迹在数学上如何预测和校正运动状态”。

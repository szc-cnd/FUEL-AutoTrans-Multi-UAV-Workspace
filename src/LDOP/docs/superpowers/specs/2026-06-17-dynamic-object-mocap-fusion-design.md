# 动态目标动捕高频融合层设计

## 背景

LDOP 当前动态目标链路是 10Hz 点云输入经 UFOMap 动静态分割、动态点聚类后，把 `DynamicObjectDetection` 直接送入 tracker，再由 tracker 输出动态目标、轨迹 marker 和 predictor 输入。这个链路的目标中心来自 LiDAR 动态点聚类质心，频率受点云和里程计约束，约为 10Hz。下文里“雷达聚类中心”“LiDAR 聚类中心”都指这个点云质心，不是 AABB 几何中心。

本设计引入外部动捕系统提供的高频 `geometry_msgs/PoseStamped`。每个动捕位姿 topic 对应一个目标，topic 本身就是外部目标源标识；消息不携带额外目标 ID，也不携带 covariance。这里需要区分原始 mocap rigid-body pose 和进入融合语义后的目标 body pose：先把动捕 world 坐标系转换到 LDOP 处理坐标系，再用单位局部旋转和竖向平移先验把刚体的 mocap rigid-body frame 转换到目标机体/body frame。在这个两层转换之后，目标 body pose 原点才近似是目标几何中心。本文统一以 `SE(3)` 作为 frame 转换的规范表达，position / quaternion 展开只作为当前简化假设下的实现提示。点云质心会随可见点分布、遮挡和分割残片变化，不能作为固定外参定义对象；第一版只把它当作几何中心的低频近似观测。动捕数据在录包使用前已经做滤波，因此 LDOP 内不再叠加一层动捕滤波。当前阶段面向已录制数据集的离线处理和回放验证，不把在线实时闭环作为第一版约束。

第一版目标是先实现动捕辅助的融合观测层：用 LiDAR 聚类建立低频目标存在性、bbox 和几何中心近似锚点，用动捕相对运动在两帧聚类之间传播约 50Hz 的融合位姿；同时把融合 pose knot 重采样到均匀 pose grid，并用三次 B 样条生成约 200Hz 的虚拟 IMU / motion intent，供后续跟踪和预测阶段接入。旧 tracker/predictor 暂不接入，主流程到融合发布后结束。

## 范围

本次实现包含：

- 订阅多个动捕 `PoseStamped` topic。
- 用 `SE(3)` 链 `T_LM * T_MR * T_RB` 把动捕 pose 转到 LDOP 处理 frame；第一版的全局外参只启用参数平移，局部 rigid-body frame 到目标 body frame 使用单位旋转和 `-0.5 * bbox_size.z` 竖向平移先验。
- 在 LiDAR 聚类帧到来时，关联 detection 和动捕目标，把点云质心作为目标几何中心的近似观测来更新 `T_RB_est` 平移锚点。
- 在高频动捕帧到来时，用当前 mocap pose 和 `T_RB_est` 平移项计算 `fused_pose`。
- 用离线固定延迟三次 B 样条从融合位姿求导。
- 发布 mocap knot 级融合目标 pose 消息、200Hz 虚拟 IMU 消息和一个融合动态目标 marker topic。
- 在 `verbose=true` 时附加调试日志和 offset line marker。

本次实现不包含：

- 旧 tracker/predictor 接入融合观测。
- UAV 简化 ECL EKF 的状态估计实现；本阶段只提供它后续接入所需的 fused pose 和 virtual IMU / motion intent 输入。
- 人和车结合朝向的 EKF 状态估计实现；本阶段只明确这类下游 EKF 可以使用的观测与惯性式运动量语义。
- 基于 covariance 的雷达/动捕信息矩阵融合。
- mocap-only 新目标生命周期。目标必须先由雷达聚类关联后才发布融合结果。

## 总体架构

新增 `DynamicObjectPoseFusion` 模块，作为独立高频融合观测层。LDOP 主流程第一版调整为：

```text
点云 + odom 10Hz
  -> UfomapMapper
  -> DynamicObjectClusterer
  -> DynamicObjectPoseFusion 更新雷达锚点和聚类特征
  -> 发布融合结果和融合 marker
  -> 本帧结束

N 个 mocap PoseStamped topic 高频
  -> DynamicObjectPoseFusion 缓存动捕 pose
  -> 有雷达锚点后生成 mocap knot 级 fused pose observation
  -> 发布融合 pose 结果和融合 marker
  -> 写入样条源窗口

DynamicObjectPoseFusion 连续时间运动输出
  -> 从 fused pose knot 源窗口重采样均匀 pose grid
  -> 用三次 B 样条从 pose grid 生成 200Hz virtual IMU / motion intent
  -> 发布给后续 UAV 简化 ECL EKF、车/人朝向 EKF 和轨迹预测的运动输入
```

旧 tracker、predictor、track marker、prediction marker 在第一版主流程中暂停调用。`Ldop` 构造阶段可以继续初始化 `DynamicObjectTracker` 和 `DynamicObjectPredictor`，这样后续恢复接入时不用重新梳理参数加载；但 `processFrame()` / 处理 loop 不调用 `processDynamicTracks()` 和 `predict()`，`publishFrame()` 也不发布旧 tracker / predictor 输出或空消息。这样避免旧 KF 把高频动捕传播结果当作 50Hz 雷达命中，也避免把后续 EKF 模型升级和融合层混在同一阶段。

融合模块默认常开，不设置 `fusion_enable`。调试行为统一由现有 `verbose` 控制，不新增 `fusion_debug_offset_line` 之类单独调试开关。

## 线程与发布边界

`ldop_node` 使用 `ros::AsyncSpinner(0)`，点云、里程计和多个 mocap callback 可能并发执行；同时现有点云重处理在 LDOP 自己的后台线程中完成。因此 fusion 模块内部必须用互斥锁保护每个 mocap source 的 pose 缓存、anchor、offset 和 B 样条窗口。

为了避免发布时长时间占锁，模块接口按“更新状态 -> 生成输出快照 -> 释放锁 -> 发布”的顺序组织。点云处理线程只调用 fusion 的雷达锚点更新入口；mocap callback 只调用高频 pose 更新入口。两个入口共享同一套状态，但不直接互相发布半成品。

雷达帧关联不能只看每个 mocap source 的 latest pose。每个 source 维护短时间 pose deque，覆盖 `fusion_mocap_sync_tolerance` 和点云处理延迟。雷达锚点更新入口按雷达帧 `header.stamp` 在对应 deque 中找最近 mocap pose；超过同步容差的 source 不参与该帧关联。deque 只保存经过四元数归一化、时间戳严格递增检查后的 pose，避免把异常输入带入关联和样条窗口。

## 数据模型

内部新增两类输出结构，避免继续借用 `DynamicObjectDetection::bbox.center` 表达融合位姿，也避免把 50Hz 融合 pose 观测和 200Hz 虚拟 IMU 控制输入混在一个消息语义里。

```text
FusedDynamicObjectObservation
  stamp
  mocap_source_index
  fused_pose
  bbox_size
  point_count
  offset_body
  lidar_anchor_age
  lidar_updated_in_current_frame
```

字段语义：

- `stamp` 是融合 pose 观测的物理时刻，来自对应 mocap pose knot 的原始时间戳。第一版 pose 观测按 mocap knot 级发布，通常约 50Hz。
- `mocap_source_index` 是第一版稳定目标 ID。雷达聚类 ID 在遮挡后重现时可能变化，因此融合输出不沿用雷达 detection ID。
- `fused_pose` 是核心输出，表示 `stamp` 时刻目标近似几何中心语义下的融合 pose。position 由 mocap pose 和 `offset_body` 得到，orientation 来自动捕刚体姿态。
- `bbox_size` 来自最近一次雷达聚类，两个聚类帧之间保持不变。
- `point_count` 来自最近一次雷达聚类，保留点云簇规模诊断和后续分类/粘连判断需要的聚类特征。
- `offset_body` 表示 `T_RB_est` 的平移项 `t_RB`，严格说是目标 body 原点在原始 mocap rigid-body frame `R` 下的坐标；它不是对已经转换好的 `fused_pose` 再额外叠加的 residual / bias。当前 `R_mocap_object = I`，rigid-body frame 和 body frame 轴向相同，因此这个偏移与 body-frame 表达数值相同；字段名沿用 `offset_body` 是为了表达它服务于目标 body pose，而不是引入另一个世界系 bias。当前数据的局部转换先验来自 `offset_body_prior = [0, 0, -0.5 * bbox_size.z]^T` 和单位旋转；LiDAR 质心只作为低频近似观测慢速修正这个局部变换估计。
- `lidar_updated_in_current_frame` 用于后续 tracker 接入时避免 50Hz 重复累计 10Hz 雷达证据。

对外新增 `FusedDynamicObject.msg` 和 `FusedDynamicObjectArray.msg`，而不是改用旧 `DynamicObject.msg` 表达未跟踪目标。消息字段直接匹配上述观测结构：

```text
FusedDynamicObject.msg
  std_msgs/Header header
  uint32 mocap_source_index
  geometry_msgs/Pose fused_pose
  geometry_msgs/Vector3 bbox_size
  uint32 point_count
  geometry_msgs/Vector3 offset_body
  float64 lidar_anchor_age
  bool lidar_updated_in_current_frame

FusedDynamicObjectArray.msg
  std_msgs/Header header
  FusedDynamicObject[] objects
```

`FusedDynamicObject.header.stamp` 是该目标自己的物理 pose 时间。`FusedDynamicObjectArray` 使用事件批语义，不强行等待所有 mocap source 同步成同一帧：mocap callback 通常发布只包含当前 source 的一条 observation，雷达帧锚点更新后可以发布本帧被刷新或重新初始化的 source observation。array 的 `header.frame_id` 是 LDOP 处理 frame，`header.stamp` 取本批次中最新的 object stamp，仅用于 RViz / rosbag 检索；消费者必须以每个 object 自己的 `header.stamp` 作为融合观测时间。

bbox yaw、旧 bbox center、雷达 detection ID 不进入融合消息；box 的位置和朝向统一由融合后的 `fused_pose` 表达。LiDAR 聚类质心只作为内部 offset 观测使用，第一版假设它是目标几何中心的低频近似观测，不把它固化成对外消息字段。

200Hz 虚拟 IMU 另用独立结构表达：

```text
FusedDynamicObjectVirtualImu
  stamp
  available_stamp
  mocap_source_index
  fused_pose_at_eval
  virtual_linear_velocity
  virtual_angular_velocity
  virtual_linear_acceleration
  virtual_specific_force
```

字段语义：

- `stamp` 是虚拟 IMU 的物理评估时刻 `t_eval`，必须用于后续 EKF/预测的时间积分。这里不能写结果可用时刻，否则积分 `dt` 会被固定延迟污染。
- `available_stamp` 是离线固定延迟下该虚拟 IMU 样本真正可生成时的窗口最新原始 fused pose knot 时间戳，可用于评估“延迟输出和最新真值”的误差。
- `fused_pose_at_eval` 是同一 `t_eval` 上的样条 pose，主要用于诊断、可视化和下游需要位姿参考时使用。
- `virtual_linear_velocity` 是 world frame 下的 `p_dot(t)`。
- `virtual_linear_acceleration` 是 world frame 下的 `p_ddot(t)`。
- `virtual_angular_velocity` 是 body frame 下的 `omega_body(t)`。
- `virtual_specific_force` 是 body frame 下的 `R(t)^T * (p_ddot(t) - g)`，语义对齐 `virtual_imu_validator_ros` 的虚拟加速度计读数。

对外新增 `FusedDynamicObjectVirtualImu.msg` 和 `FusedDynamicObjectVirtualImuArray.msg`：

```text
FusedDynamicObjectVirtualImu.msg
  std_msgs/Header header
  time available_stamp
  uint32 mocap_source_index
  geometry_msgs/Pose fused_pose_at_eval
  geometry_msgs/Vector3 virtual_linear_velocity
  geometry_msgs/Vector3 virtual_angular_velocity
  geometry_msgs/Vector3 virtual_linear_acceleration
  geometry_msgs/Vector3 virtual_specific_force

FusedDynamicObjectVirtualImuArray.msg
  std_msgs/Header header
  FusedDynamicObjectVirtualImu[] samples
```

virtual IMU array 也是事件批语义。一次 mocap knot 到来后，如果该 source 的样条窗口已经就绪，模块可以一次生成该 source 尚未发布的一批 200Hz `t_eval` 样本；这些样本可以放在同一个 array 里。每个 sample 的 `header.stamp` 才是积分使用的物理时刻，array header 只取本批最后一个 sample 的 `header.stamp`。虚拟 IMU 样本只在样条窗口满足条件时发布；窗口未就绪、严重丢帧或 offset 跳变重置窗口期间，不发布伪 200Hz 样本，也不补发由无效窗口推出来的历史样本。

## 固定外参

本文所有 frame 变换统一使用 `SE(3)`。约定 `T_AB` 表示把 B frame 下的点变换到 A frame：

```text
T_AB =
  [ R_AB, t_AB
    0,    1    ]

p_A = T_AB * p_B
```

其中：

- `L` 表示 LDOP 里程计/雷达处理 frame。
- `M` 表示 mocap world frame。
- `R` 表示原始 mocap rigid-body frame。
- `B` 表示目标机体/body frame，也就是融合语义里的近似几何中心 frame。

动捕到 LDOP 的 frame 处理分成两个层次：

1. 全局 world frame 对齐：把动捕 world 坐标系下的 pose 转到 LDOP 里程计/雷达处理 frame。
2. 局部刚体 frame 对齐：把 mocap rigid-body frame 转成目标机体/body frame，使进入 fusion 的 pose 已经使用目标 FLU body frame，并且 pose 原点近似目标几何中心。

当前动捕 world frame 与 LDOP 里程计/雷达处理 frame 的坐标轴已经对齐，第一版只需要给 `T_LM` 配置平移项。这个平移项的方向按 `p_L = p_M + t_LM` 定义，也就是 mocap world 原点 `M` 在 LDOP frame `L` 下的坐标；这样可以避免把“从 mocap 原点到 LDOP 原点”的口语描述误解成相反符号。参数配置全局平移外参：

```text
t_ldop_mocap_world
```

因此第一版全局外参写成：

```text
T_LM =
  [ I, t_ldop_mocap_world
    0, 1                  ]
```

原始 mocap rigid-body pose 是 `T_MR(t)`。全局对齐后的刚体 pose 是：

```text
T_LR(t) = T_LM * T_MR(t)
```

在当前 `R_LM = I` 的假设下，展开到 position / orientation 为：

```text
p_rigid_ldop(t) = p_rigid_mocap_world(t) + t_ldop_mocap_world
q_rigid_ldop(t) = q_rigid_mocap_world(t)
```

这个约定和 `virtual_imu_validator_ros` 的姿态语义保持一致：四元数表示刚体 frame 到 world/reference 的旋转。这里不额外旋转全局姿态；如果后续发现动捕 world 与 LDOP frame 存在轴向旋转，只需要把 `T_LM` 从纯平移升级为完整 `SE(3)` 外参或 `tf2` lookup，不改变后续组合顺序。

多个 mocap topic 默认处在同一个 mocap world 下，第一版只支持一组全局平移外参。

局部刚体 frame 对齐不在 LDOP 第一版里暴露成 per-object 参数。当前数据的目标局部坐标系方向已经按前-左-上（FLU）组织，因此 mocap rigid-body frame 到目标 body frame 的局部旋转等价于单位旋转：

```text
R_mocap_object = I
```

动捕刚体原点在目标顶部附近，目标几何中心位于其下方。第一版使用当前 LiDAR detection 的 bbox 高度给出局部平移先验：

```text
offset_body_prior(i) = [0, 0, -0.5 * bbox_size_i.z]^T
```

其中 `bbox_size_i.z` 来自当前候选 LiDAR detection；目标已经关联后，则使用该 mocap source 最近一次成功关联的 `latest_bbox_size.z`。在 `SE(3)` 记号里，`T_RB` 的平移项是 B 原点在 R frame 下的坐标；当前 `R_mocap_object = I`，所以它和 body frame 下的 `offset_body_prior` 数值一致。若以后允许非单位局部旋转，这个字段仍应按 `T_RB` 平移项理解，而不能直接当作 body frame 下的新向量使用。局部转换先验写成：

```text
T_RB_prior(i) =
  [ R_mocap_object, offset_body_prior(i)
    0,              1                    ]
```

因此进入融合语义的目标 body pose 先验为：

```text
T_LB_prior(t, i) = T_LM * T_MR(t) * T_RB_prior(i)
                 = T_LR(t) * T_RB_prior(i)
```

在当前 `R_LM = I`、`R_mocap_object = I` 的假设下，展开为：

```text
p_body_prior(t) = p_rigid_ldop(t) + R_rigid_ldop(t) * offset_body_prior(i)
q_body_prior(t) = q_rigid_ldop(t)
```

实现不把 `T_LB_prior` 当作长期状态保存。每个新 mocap pose 都重新按 `T_LB = T_LR * T_RB_est` 组合，其中 `T_RB_est` 是 rigid-body frame 到目标 body frame 的局部变换估计。首次关联前用 `T_RB_prior` 初始化 `T_RB_est`；关联成功后，用 LiDAR 质心观测对 `T_RB_est` 的平移项做小步 EMA 修正。这里修正的是局部变换估计，不是在已经转换好的 `T_LB_prior` 或 `fused_pose` 上额外叠加一个偏差。对无人机来说，只要这个 FLU body frame 与后续简化 ECL EKF 使用的机体系一致，就不需要额外姿态矫正；对车和人也直接复用同一约定。若以后局部旋转不再是单位阵，或 mocap 刚体原点不再是顶部附近，再把完整 `T_RB` 固定变换作为新的数据集预处理步骤或 LDOP 后续参数单独加入，而不是在本版里预留 per-object calibration 参数。

## 雷达关联与 offset 锚点

雷达聚类帧到来时，fusion 模块拿到 10Hz detections，并取每个 mocap topic 在该雷达时间附近的 pose。时间同步使用最近邻缓存，超出 `fusion_mocap_sync_tolerance` 的 mocap pose 不参与本帧雷达关联。

关联代价使用 LiDAR 聚类质心与当前预测融合位置的距离。对每个候选 mocap source，先按 `T_LR_j(t_k) = T_LM * T_MR_j(t_k)` 得到雷达时刻附近的 LDOP-frame rigid-body pose，再用当前 `T_RB` 估计预测 body pose：首次关联前使用 `T_RB_prior(i)`，已有锚点后使用 `T_RB_est`。第一版明确假设：`DynamicObjectDetection::bbox.center` 是点云质心，它不是严格几何中心，但在当前数据和目标尺度下可以作为目标近似几何中心的低频观测。这个假设只影响雷达锚点更新，不改变局部 rigid-body -> body 转换的固定语义；转换后的目标 body pose 仍表示目标近似几何中心，避免把外参绑到会随遮挡和可见点分布变化的质心上。

首次还没有融合预测时，使用候选 LiDAR detection 的 bbox 高度构造局部平移先验。由于 `offset_body` 存储的是 `T_RB` 的平移项，也就是目标近似几何中心相对原始 mocap rigid-body pose 原点、并在 rigid-body frame `R` 下表达的位置；当前 `R_mocap_object = I`，它才与 body-frame 表达数值相同。目标局部 z 轴向上，动捕刚体原点在顶部附近时默认先验为：

```text
offset_body_prior(i) = [0, 0, -0.5 * bbox_size_i.z]^T
```

这个先验只用于首次关联和初始化前的预测；关联成功后仍以 LiDAR 聚类质心反算的 `T_RB_est` 平移观测为准。因为质心不是固定外参目标，后续 `T_RB_est` 平移项使用小步 EMA 和门控慢速修正，只吸收小的系统偏差或数据集坐标处理残差，不追随每一帧点云质心抖动。

如果首次关联时需要预测候选 detection `i` 与 mocap source `j` 的融合中心，先用该候选 detection 的 `T_RB_prior(i)` 计算：

```text
T_predicted_fused(i, j, t_k) = T_LR_j(t_k) * T_RB_prior(i)
p_predicted_fused_{i,j} = translation(T_predicted_fused(i, j, t_k))
```

在当前 `R_mocap_object = I` 的假设下，展开为：

```text
p_predicted_fused_{i,j} = p_rigid_ldop_j + R_rigid_ldop_j * offset_body_prior(i)
```

已有有效锚点的 source 则使用当前估计的 `T_RB_est = [I, offset_body; 0, 1]` 做同样的预测。

```text
cost(i, j) = || p_lidar_i - p_predicted_fused_{i,j} ||
```

第一版目标数量假设不超过 10 个，常见情况不超过 5 个。关联使用小规模无依赖最优匹配：对通过距离门控的 pair 做 bitmask DP 或等价穷举，最小化总距离，只接受小于 `fusion_association_max_distance` 的 pair。这样保留 Hungarian 的全局最优语义，但不新增外部依赖。

关联成功后，不做伪 covariance 加权，而是更新 `T_RB_est` 的平移项。LiDAR 只给出 LDOP frame 下的中心点观测 `p_lidar^L(t_k)`，因此平移观测等价于把这个点用 `T_LR(t_k)^{-1}` 变回当前 rigid-body frame；在当前 `R_mocap_object = I` 时，这个 rigid-body frame 偏移和字段名里的 body offset 数值相同：

```text
p_lidar^R(t_k) = T_LR(t_k)^-1 * p_lidar^L(t_k)
t_RB_observed = p_lidar^R(t_k)
t_RB_observed = R_rigid_ldop(t_k)^T * (p_lidar(t_k) - p_rigid_ldop(t_k))
```

首次关联：

```text
offset_body = t_RB_observed
```

后续关联：

```text
offset_body <- (1 - alpha) * offset_body + alpha * t_RB_observed
```

雷达在这里的职责是确认目标存在、更新 bbox size 和 point_count，并把点云质心作为几何中心近似观测来慢速校正 `T_RB_est` 的平移项，也就是“原始 mocap rigid-body pose 原点到目标几何中心”的局部刚体坐标偏移。动捕负责高频传播和朝向。

为避免错配、聚类粘连或残片污染 offset，更新前使用三层门控：

- `fusion_association_max_distance`：LiDAR 聚类质心到预测融合中心的最大距离。
- `fusion_offset_observation_max_norm`：观测 offset 的最大物理长度。
- `fusion_offset_update_max_delta`：单次 offset 观测相对当前 offset 的最大变化。

门控失败时不更新 offset，也不刷新该目标的雷达锚点。`verbose=true` 时记录拒绝原因。

## 高频传播

每个 mocap topic 维护一个融合锚点：

```text
anchor_stamp
anchor_mocap_pose_ldop
anchor_fused_pose
offset_body
latest_bbox_size
latest_point_count
```

高频 mocap pose 到来时，先用 `T_LR(t) = T_LM * T_MR(t)` 把原始 rigid-body pose 转到 LDOP frame。若该 topic 已有有效雷达锚点，则用当前 `T_LR(t)` 和 `T_RB_est` 的平移项直接计算融合位姿；在本版局部旋转为单位阵时，它才与 body-frame offset 数值相同。第一版把下面的 `SE(3)` 关系作为唯一规范公式：

```text
T_RB_est =
  [ I, offset_body
    0, 1           ]

T_LB_fused(t) = T_LR(t) * T_RB_est
```

在当前 `R_LM = I`、`R_RB_est = I` 的假设下，展开到 position / orientation：

```text
p_fused(t) = p_rigid_ldop(t) + R_rigid_ldop(t) * offset_body
q_fused(t) = q_rigid_ldop(t)
```

`anchor_mocap_pose_ldop` 和 `anchor_fused_pose` 只记录最近一次 LiDAR 锚点更新时的状态，服务于目标是否已有锚点、锚点年龄和重初始化判断；实现中不要再用 `anchor_fused_pose * inverse(anchor_mocap_pose_ldop) * T_LR(t)` 这类左乘增量公式替代上面的 `T_LR(t) * T_RB_est` 公式。原因是 `T_RB_est` 的平移项定义在当前 rigid-body frame 下，必须随当前 `R_rigid_ldop(t)` 旋转；左乘 anchor 增量容易把 offset 固定在旧锚点姿态下，导致目标转向时中心点传播错误。

当 offset 在雷达帧被更新时，使用同一个规范公式立刻重算当前 `fused_pose`，并刷新 `anchor_fused_pose` 和 `anchor_mocap_pose_ldop`。常规的 10Hz offset EMA 小步校正不清空 B 样条运动窗口，因为 mocap 约 50Hz 时两帧雷达之间通常只有 5 个真实融合 pose knot；若每次雷达校正都清空窗口，默认 8 个均匀 pose-grid knot 的求导条件会长期无法满足。这里的原始 fused pose knot 是样条源窗口里的样本，不是样条建好之后插值得到的 200Hz 虚拟评估点；样条真正求解前还会从这些原始 knot 重采样出均匀 pose-grid knot。只有首次关联、雷达锚点重初始化、目标 source 重绑定，或同一时刻 offset 校正导致 `fused_pose` 位置跳变超过 `fusion_bspline_reset_translation_delta` 时，才清空对应 mocap source 的 B 样条运动窗口，并从断点后的连续融合 pose 重新累计。这样把“真实连续运动”和“低频校正跳变”分开处理，避免把明显不连续的校正量解释成速度或加速度。

如果雷达短时缺失，已有锚点可继续按 mocap 高频 pose 输出融合结果，bbox size 和 point_count 沿用最近一次聚类结果。如果 `lidar_anchor_age > fusion_lidar_anchor_timeout`，停止发布该目标，避免动捕在没有雷达确认时无限续航。

## 三次 B 样条求导

融合位姿传播和虚拟运动量求解分成两条链路：

```text
fused pose 观测链路:
  LiDAR 锚点 + 当前 mocap pose + T_RB_est 平移项 -> mocap knot 级 fused_pose，通常约 50Hz

virtual IMU 链路:
  最近一段 fused pose knot -> 均匀 pose grid -> 三次 B 样条 -> 200Hz velocity / omega / acceleration / specific force
```

动捕 nominal 频率约 50Hz。50Hz fused pose 观测层的作用类似 VINS-Fusion 里低频外部观测被高频惯性传播后的状态观测：LiDAR 聚类提供目标存在性、bbox 和低频中心锚点，mocap 提供两帧雷达之间的刚体运动。由于 mocap 输入本身已经是 pose，第一版不必先把它转换成 IMU 再预积分来得到 50Hz pose；直接用 `T_LB_fused = T_LR * T_RB_est` 生成融合 pose，更少一层积分误差，也更容易和 LiDAR offset 对齐。

200Hz virtual IMU 层的作用不同：它不是为了修正 50Hz fused pose，而是给后续无人机简化 ECL EKF、车/人带朝向 EKF 和轨迹预测提供统一的惯性式运动输入。这个 200Hz 信号来自连续时间样条求导，不包含真实 200Hz 传感器才能观测到的高频振动或冲击；下游使用时应把它当作“由 50Hz pose 轨迹生成的平滑控制/运动意图”，噪声和可信度不能按真实 IMU 的独立 200Hz 测量来设。

当前阶段使用录制数据集离线处理，因此 virtual IMU 允许非因果固定延迟窗口，默认使用 2 帧未来样本。这里的未来样本表示离线样条求导可以等待当前评估时刻之后的均匀 pose-grid knot 到齐，再生成该评估时刻的虚拟 IMU 样本。虚拟 IMU 消息的 `header.stamp` 必须使用物理评估时刻 `t_eval`，固定延迟下的结果可用时刻写入 `available_stamp`。

`virtual_imu_validator_ros` 的虚拟 IMU 生成流程不是要求输入 200Hz 真值 pose。它先把 GT 通过线性位置插值和姿态 SLERP 采样到 `pose_rate_hz` 的均匀 pose 网格（默认 20Hz），再构建连续时间样条，最后在 `virtual_imu_rate_hz` 的时间轴上评估样条导数（默认 200Hz）。这一步下采样是验证器为了模拟低频外部 pose 输入而做的处理；LDOP 融合层不把动捕降到 20Hz，但样条内部仍按 validator 的数学契约先形成均匀 pose 网格。第一版用原始 fused pose knot 作为插值源，按 `fusion_mocap_nominal_rate` 生成样条输入网格；位置线性插值，姿态 SLERP，网格步长 `h = 1 / fusion_mocap_nominal_rate`。

LDOP 第一版也按这个思路在离线模式下输出虚拟评估网格：原始融合 pose knot 仍按 mocap 到达时刻发布，样条输入则是由这些 knot 插值得到的均匀 pose-grid knot，输出按 `fusion_virtual_imu_rate_hz` 的均匀时间轴生成，默认 200Hz。若后续只想输出和 mocap 相同的频率，可以把 `fusion_virtual_imu_rate_hz` 设成 `fusion_mocap_nominal_rate`，但第一版文档按服务下游 EKF/预测的 200Hz 虚拟 IMU 网格描述。

因此需要区分三类采样点：原始 fused pose knot、样条输入 pose-grid knot、输出虚拟评估点。插值型 B 样条先用均匀 pose-grid knot 解控制点并形成连续轨迹，之后才可以在 200Hz 或其他频率上评估 `fused_pose_at_eval`、`p_dot`、`p_ddot` 和 `omega_body`。200Hz 虚拟评估点不能反过来当作新的输入 knot 来满足 `fusion_bspline_min_samples`，也不能修复 offset 跳变或 mocap 严重丢帧造成的轨迹断点。

每个 mocap source 维护独立的 `last_published_virtual_imu_eval_stamp`。新的原始 fused pose knot 写入样条源窗口后，模块先检查原始 knot 的时间戳严格递增和严重丢帧门控，再把已经被足够未来原始 knot 覆盖的时间段重采样成均匀 pose-grid knot。如果 pose-grid 窗口满足 `fusion_bspline_min_samples` 和 `fusion_bspline_future_samples`，就计算当前窗口中“已经拥有足够未来 pose-grid knot 支撑”的最晚评估时刻。模块只在 `(last_published_virtual_imu_eval_stamp, latest_confirmed_eval_stamp]` 内按 `fusion_virtual_imu_rate_hz` 的均匀网格生成样本，并在发布后推进 `last_published_virtual_imu_eval_stamp`。这样 rosbag 离线回放时可以由一个 mocap callback 触发一小批历史 `t_eval` 输出，但不会重复发布同一个 `t_eval`，也不会因为窗口刚恢复就补发跨过断点的无效样本。每个 sample 的 `available_stamp` 写触发这批样本时窗口最新原始 fused pose knot 的时间戳。

位置求导沿用 `virtual_imu_validator_math_tutorial.md` 中的位置插值型三次 B 样条。三次 B 样条在单次求值时使用 4 个相邻控制点；但插值型实现需要先用多个 pose 样本解控制点线性方程。`virtual_imu_validator_ros` 的 `InterpCubicBSpline3D::build()` 至少需要 5 个 pose，验证节点主流程为了形成稳定有效区间要求下采样 pose 不少于 8 个。LDOP 第一版按 validator 主流程对齐：滚动窗口默认至少保留 8 个均匀 pose-grid knot，只在避开样条边界的有效区间内求导；默认 2 帧未来 pose-grid 样本到齐后，评估最新可确认的虚拟评估网格时刻。

控制点求解也按 validator 的插值型实现组织：边界控制点固定为端点位置 `C_0 = p_0`、`C_{N-1} = p_{N-1}`，内部点使用

```text
(C_{i-1} + 4 C_i + C_{i+1}) / 6 = p_i,  i = 1 ... N-2
```

组成线性方程求解所有控制点。给定控制点后，再在目标时间段内用 4 个相邻控制点和三次 B 样条基函数解析计算：

```text
p(t), p_dot(t), p_ddot(t)
```

其中 `p_dot(t)` 和 `p_ddot(t)` 分别来自基函数对时间的一阶、二阶导数；实现时按教程中的 `1/h` 和 `1/h^2` 链式法则缩放，避免用相邻帧有限差分替代样条导数。

姿态求导沿用 `virtual_imu_validator_math_tutorial.md` 中的 SO(3) 累积 B 样条。这里要注意：validator 的位置样条是插值型，姿态样条不是先解线性方程强制穿过全部姿态点，而是把输入姿态样本直接作为 SO(3) 累积样条的控制姿态。这样做的目标是得到平滑可求导的旋转轨迹，并与 validator 当前实现保持一致。实现使用 4 个相邻姿态、相邻姿态的 `SO(3)` 对数映射、累积样条系数、指数映射和右雅可比，求 body 系角速度：

```text
omega_body(t) = vee(R(t)^T * R_dot(t))
```

令局部段上的 4 个姿态控制点为 `R0, R1, R2, R3`，相邻相对旋转为 `Omega1 = Log(R0^T R1)`、`Omega2 = Log(R1^T R2)`、`Omega3 = Log(R2^T R3)`。累积样条写成 `R(t) = R0 * A1 * A2 * A3`，其中 `Ak = Exp(phi_k^)`，`phi_k = lambda_k(u) * Omega_k`，`phi_k_dot = lambda_k'(u) * Omega_k / h`。在这个定义下：

```text
omega_body =
  (A2 * A3)^T * Jr(phi1) * phi1_dot
  + A3^T * Jr(phi2) * phi2_dot
  + Jr(phi3) * phi3_dot
```

这里的姿态样条只用于从融合姿态生成平滑角速度；不额外引入真实 IMU 残差优化。

每个 mocap source 在原始 fused pose knot 写入样条源窗口前都要按 `virtual_imu_validator_ros` 的做法对四元数归一化，并维护 `q` / `-q` 的符号连续性：如果当前四元数与上一有效四元数点积小于 0，则翻转当前四元数系数。重采样生成均匀 pose-grid knot 时也沿用这个连续符号约定，再做 SLERP。这样 SO(3) 对数映射不会因为等价四元数符号跳变而生成虚假的大角速度。实现时注意 ROS `geometry_msgs/Quaternion` 字段顺序是 `x,y,z,w`，而 `Eigen::Quaterniond` 构造函数常用顺序是 `w,x,y,z`，这里必须显式映射，不能按内存顺序直接拷贝。

样条公式按均匀时间间隔推导。LDOP 第一版保留原始 mocap/fused pose knot 用于对外发布和重采样源，但样条求导只在均匀 pose-grid knot 上进行：网格起点对齐到 `h = 1 / fusion_mocap_nominal_rate`，网格内每个 pose 由相邻原始 fused pose knot 插值得到。若原始 knot 时间戳倒退、重复，或相邻间隔偏离 nominal dt 超过 `fusion_mocap_dt_jitter_tolerance` 导致无法可信插值，50Hz fused pose 仍可在真实 mocap pose knot 上继续发布，但 200Hz virtual IMU 暂停发布，并清空该 mocap source 的 B 样条运动窗口，从断点后的新连续片段重新累计。这样与 validator 的“先均匀 pose grid，再连续时间样条，再高频求导”一致，避免把非均匀真实时间戳直接代入均匀 B 样条公式。

virtual IMU 输出：

- `virtual_linear_velocity`：世界系 `p_dot(t)`。
- `virtual_linear_acceleration`：世界系 `p_ddot(t)`。
- `virtual_angular_velocity`：body 系 `omega_body(t)`。
- `virtual_specific_force`：body 系 `R(t)^T * (p_ddot(t) - g)`，与 `virtual_imu_validator_ros` 的虚拟加速度计读数一致。

`virtual_linear_acceleration` 和 `virtual_specific_force` 是两个不同物理量：前者是 LDOP/world frame 下的运动学线加速度，后者是 validator 语义下 IMU/body frame 的比力。实现和消息字段必须分开，避免后续 tracker 或 EKF 重力项双算或漏算。

虚拟 IMU 的 `header.stamp` 使用内部 B 样条实际求值的 `t_eval`。这样后续 EKF/预测模块可以按正确物理时间积分；若要评估“固定延迟输出和最新真值之间的误差”，使用 `available_stamp` 找到生成该样本时的最新真值。

当样本不足默认 8 个均匀 pose-grid knot、未来样本不足，或时间间隔抖动门控失败时，不能生成 200Hz virtual IMU 样本；此时 50Hz fused pose topic 仍可在真实 mocap pose knot 到来时发布传播得到的 `fused_pose`，但 virtual IMU topic 暂停发布。等样条窗口重新满足条件后，再恢复按 `fusion_virtual_imu_rate_hz` 的均匀网格输出。

本节只复用 `virtual_imu_validator_ros` 的连续时间位姿求导主体，不包含该验证器里的 RMSE、相关系数、真实 IMU 低通对比、bias 扣除或预积分一致性验证流程。

## 可视化

第一版保持一个融合动态目标可视化 topic，语义是：

```text
当前 fused_pose + 最近一次雷达聚类特征
```

box 的位置和朝向来自 `fused_pose`，box 的尺寸来自最近一次雷达聚类 `bbox_size`。这里的可视化 box 是“融合几何中心 pose + 最近 LiDAR AABB 尺寸”的近似表达，不是重新估计出的严格 oriented bbox；它的目的只是让 RViz 中目标随 mocap 高频移动，同时保留最近一次点云聚类的尺度信息。两帧聚类之间，box size 不变，但 box 位姿按高频动捕传播移动。下一帧聚类到来后，再刷新 size 和 offset。

`verbose=false` 时只显示融合 box。`verbose=true` 时额外显示 offset line：从当前 mocap pose 原点连到 `fused_pose.position`。这条线用于观察动捕三轴位置和雷达聚类中心之间的差值是否稳定，也就是 `offset_body` 是否被错配或聚类粘连带偏。

## 参数

参数默认集中放在对应 `*Params` 结构体，`loadParameters()` 使用三参数 `pnh_.param()` 读取。

建议参数：

```yaml
fusion_mocap_topics:
  - /mocap/target_0/pose
  - /mocap/target_1/pose
fusion_mocap_to_ldop_translation: [0.0, 0.0, 0.0]
fusion_mocap_sync_tolerance: 0.03
fusion_association_max_distance: 1.0
fusion_offset_alpha: 0.2
fusion_offset_observation_max_norm: 2.0
fusion_offset_update_max_delta: 0.3
fusion_lidar_anchor_timeout: 0.5
fusion_bspline_min_samples: 8
fusion_bspline_future_samples: 2
fusion_bspline_reset_translation_delta: 0.2
fusion_mocap_nominal_rate: 50.0
fusion_virtual_imu_rate_hz: 200.0
fusion_mocap_dt_jitter_tolerance: 0.005
fusion_gravity_z: -9.80665
fusion_objects_topic: /ldop/fused_dynamic_objects
fusion_virtual_imu_topic: /ldop/fused_dynamic_object_virtual_imu
fusion_marker_topic: /ldop/fused_dynamic_object_markers
```

建议约束：

| 参数 | 类型 | 默认值 | 约束与语义 |
| --- | --- | --- | --- |
| `fusion_mocap_topics` | `string[]` | 示例 topic | 非空；数组顺序就是 `mocap_source_index`，录包和配置必须保持稳定。 |
| `fusion_mocap_to_ldop_translation` | `double[3]` | `[0, 0, 0]` | 必须正好 3 个元素，表示 `T_LM` 的平移项，也就是 mocap world 原点在 LDOP frame 下的坐标。 |
| `fusion_mocap_sync_tolerance` | `double` | `0.03` | 大于 0；雷达帧关联时允许的最近邻 mocap 时间差。 |
| `fusion_association_max_distance` | `double` | `1.0` | 大于 0；LiDAR 质心到预测融合中心的距离门控。 |
| `fusion_offset_alpha` | `double` | `0.2` | 建议 `[0, 1]`；越大越快追随 LiDAR 质心观测，越小越保持局部 frame 转换先验。 |
| `fusion_offset_observation_max_norm` | `double` | `2.0` | 大于 0；拒绝物理长度明显不合理的 offset 观测。 |
| `fusion_offset_update_max_delta` | `double` | `0.3` | 大于 0；拒绝单帧突变，避免错配或聚类粘连污染 offset。 |
| `fusion_lidar_anchor_timeout` | `double` | `0.5` | 大于 0；超过该时长没有 LiDAR 锚点刷新就停止发布该 source。 |
| `fusion_bspline_min_samples` | `int` | `8` | 至少 5，默认按 validator 主流程保留 8 个均匀 pose-grid knot。 |
| `fusion_bspline_future_samples` | `int` | `2` | 大于等于 0 且小于 `fusion_bspline_min_samples`；控制固定延迟窗口中的未来 pose-grid knot 数量。 |
| `fusion_bspline_reset_translation_delta` | `double` | `0.2` | 大于 0；offset 校正造成融合位置跳变超过该值时重启样条窗口。 |
| `fusion_mocap_nominal_rate` | `double` | `50.0` | 大于 0；用于原始 fused pose knot 的严重丢帧门控，并作为样条输入 pose grid 的重采样频率，不改变 fused pose 对外发布频率。 |
| `fusion_virtual_imu_rate_hz` | `double` | `200.0` | 大于 0；virtual IMU 输出评估网格频率。 |
| `fusion_mocap_dt_jitter_tolerance` | `double` | `0.005` | 大于等于 0；原始 fused pose knot 间隔偏离 nominal dt 超过该值时重启样条窗口。 |
| `fusion_gravity_z` | `double` | `-9.80665` | LDOP/world frame 下重力 z 分量，用于 `R^T * (p_ddot - g)`。 |

不设置 `fusion_enable`。融合层在第一版常开。

不设置 `fusion_debug_offset_line`。offset line 和详细日志由现有 `verbose` 统一控制。

## 异常处理

- 没有雷达锚点：不发布该 mocap topic 的融合目标，等待第一次雷达关联。
- mocap pose 与雷达帧时间差过大：该 topic 不参与本帧关联。
- `fusion_mocap_topics` 为空或平移外参数组不是 3 个元素：初始化失败并明确报错，因为这类配置错误会让 source identity 或坐标系语义不可信。
- 雷达短时缺失：继续用 mocap + offset 高频传播，bbox size 和 point_count 沿用旧聚类结果。
- 雷达锚点超时：停止发布该目标。
- offset 观测超限或突变：拒绝本次 offset 更新，保留旧 offset。
- B 样条均匀 pose-grid 样本不足或未来样本不足：50Hz fused pose 继续发布，200Hz virtual IMU 暂停发布。
- 时间间隔抖动过大或严重丢帧：50Hz fused pose 继续发布，200Hz virtual IMU 暂停发布，并清空对应 mocap source 的 B 样条运动窗口。
- 动捕四元数异常：丢弃该 pose 样本，不更新缓存。
- 动捕时间戳倒退或重复：丢弃该 pose 样本，不更新缓存。

## 验证策略

单元测试：

- 固定 `offset_body` 时，`T_LB_fused = T_LR * T_RB_est` 的组合顺序是否正确，mocap 平移/旋转后 `fused_pose` 是否正确跟随。
- 首次雷达关联是否初始化 offset。
- 首次关联预测是否在 FLU 约定下使用 `T_predicted_fused = T_LR * T_RB_prior`，其中 `R_mocap_object = I`、`offset_body_prior = [0, 0, -0.5 * bbox_size.z]^T`，且不额外引入每目标旋转外参。
- LiDAR 聚类质心是否只作为几何中心近似观测进入 offset 更新，不进入对外融合消息字段。
- 后续雷达关联是否按 `fusion_offset_alpha` 平滑 offset。
- `fusion_offset_observation_max_norm` 和 `fusion_offset_update_max_delta` 是否能拒绝异常更新。
- 常规 offset EMA 更新是否保留 B 样条运动窗口，首次关联、重绑定或超过 `fusion_bspline_reset_translation_delta` 的位置跳变是否清空窗口。
- 雷达锚点超时后是否停止发布。
- fused pose topic 的 `header.stamp` 是否等于 mocap pose knot 原始时间戳。
- `FusedDynamicObjectArray` 事件批是否允许不同 source 不同步，且消费者可以从每个 object 自己的 `header.stamp` 读到真实观测时间。
- virtual IMU topic 的 `header.stamp` 是否等于内部样条评估时刻，`available_stamp` 是否等于生成该样本时的窗口最新原始 fused pose knot 时间戳。
- virtual IMU 是否只发布 `(last_published_virtual_imu_eval_stamp, latest_confirmed_eval_stamp]` 内的网格样本，不重复发布、不跨断点补发。
- 样条窗口未满足条件时是否暂停 virtual IMU 输出，窗口恢复后是否先按 `fusion_mocap_nominal_rate` 重采样 pose grid，再切回 `fusion_virtual_imu_rate_hz` 均匀评估网格。
- 时间间隔抖动超过 `fusion_mocap_dt_jitter_tolerance` 时是否暂停 virtual IMU 输出并重启样条窗口。
- 四元数符号翻转输入是否仍生成连续角速度。
- `virtual_specific_force` 是否按 `R^T * (p_ddot - g)` 使用 `fusion_gravity_z` 计算。
- `processFrame()` / 处理 loop 是否只执行 mapper、clusterer 和 fusion，不调用 `DynamicObjectTracker::processDynamicTracks()` 或 `DynamicObjectPredictor::predict()`。

集成验证：

- 播放 rosbag，检查 `/ldop/fused_dynamic_objects` 频率接近 mocap pose knot 频率，`/ldop/fused_dynamic_object_virtual_imu` 频率接近 `fusion_virtual_imu_rate_hz`。
- RViz 中融合 box 尺寸在两帧聚类之间保持不变，但位姿高频移动。
- `verbose=true` 时 offset line 长度和方向应随目标刚体坐标系稳定，而不是随机跳变。
- `verbose=false` 时不显示 offset line，也不打印融合调试细节。
- 旧 `/ldop/dynamic_objects`、track marker 和 prediction topics 在第一版主流程中不再随每个点云帧发布新消息；检查 rosbag 或 `rostopic hz` 时应看到 fusion 输出成为动态目标主输出。

构建验证：

- 新增或修改 `msg/*.msg` 后运行 `catkin build ldop --no-status`。
- 若新增 fusion 单元测试，运行对应测试；若改动消息和主流程，至少运行 `catkin run_tests ldop --no-status` 并用 `catkin_test_results /home/st/ldop_ws/build/ldop` 查看结果。

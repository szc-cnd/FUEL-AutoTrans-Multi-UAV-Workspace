# DynamicObjectTracker 执行流程说明

本文专门按代码中的函数调用顺序说明跟踪模块的执行流程。它不重复展开完整 Kalman 数学公式；数学细节见 `dynamic_object_tracker_数学逻辑说明.md`。这里重点回答一个读代码时更实际的问题：从节点初始化开始，数据怎样一步步进入 tracker，tracker 又怎样维护 `tracks_`、稳定 ID、输出消息和 RViz marker。

对应代码主要位于：

1. `src/ldop.cpp`
2. `include/ldop/dynamic_object_tracker.h`
3. `src/dynamic_object_tracker.cpp`
4. `include/ldop/motion_model.h`
5. `src/motion_model.cpp`
6. `include/ldop/multi_model_kalman_filter.h`
7. `src/multi_model_kalman_filter.cpp`

## 1. 总体调用链

从 ROS 节点运行到跟踪输出，可以先记住这条主线：

```text
Ldop::Ldop()
  -> 创建 DynamicObjectTracker
  -> DynamicObjectTracker::DynamicObjectTracker()
  -> DynamicObjectTracker::loadParameters()
  -> sanitizeTrackerParams()

每帧点云处理：
Ldop::processingLoop()
  -> Ldop::processFrame()
  -> UfomapMapper::processInputCloud()
  -> DynamicObjectClusterer::processDynamicObjects()
  -> DynamicObjectTracker::processDynamicTracks()
  -> Ldop::publishFrame()
```

tracker 模块真正接收的是聚类模块输出的 `std::vector<DynamicObjectDetection>`。也就是说，tracker 不直接看原始点云，也不直接访问 UFOMap。它只关心每个 detection 的：

1. `detection.bbox.center`：当前目标观测位置，来自点簇质心。
2. `detection.bbox.size`：当前目标 AABB 尺寸，只用于轨迹 bbox 尺寸平滑和可视化。
3. `detection.point_count`：当前检测中的点数，只用于临时合并/分离保护，不对外发布。
4. `detection.stamp` / `header.stamp`：当前帧时间，tracker 实际用 `header.stamp` 计算预测步长。

## 2. LDOP 初始化阶段

### 2.1 `Ldop::Ldop()`

`Ldop::Ldop()` 是整个 LDOP 主类的构造函数，跟踪模块在这里被创建。

代码顺序是：

```text
Ldop::Ldop()
  -> loadParameters()
  -> setupRosInterfaces()
  -> ufomap_mapper_ = make_unique<UfomapMapper>(...)
  -> dynamic_object_clusterer_ = make_unique<DynamicObjectClusterer>(...)
  -> dynamic_object_tracker_ = make_unique<DynamicObjectTracker>(pnh_, verbose_)
  -> startWorkers()
```

这里要注意两个边界：

1. `Ldop` 负责 ROS I/O、线程和模块串接。
2. `DynamicObjectTracker` 只在被构造时读取自己的跟踪参数，之后每帧只通过 `processDynamicTracks()` 接收 detection。

### 2.2 `DynamicObjectTracker::DynamicObjectTracker()`

tracker 构造函数本身很短：

```text
DynamicObjectTracker::DynamicObjectTracker(pnh, verbose)
  -> 初始化 pnh_ / config_ / params_ / tracks_ / next_track_id_ / verbose_
  -> loadParameters()
  -> verbose 时打印 gate、history、miss、publish 阈值
```

此时 `tracks_` 还是空的，`next_track_id_` 从 0 开始。真正的轨迹不会在构造阶段创建，而是在后续某一帧出现未匹配 detection 时由 `createTrack()` 创建。

## 3. 参数读取与合法化

### 3.1 `loadParameters()`

`DynamicObjectTracker::loadParameters()` 只做两件事：

```text
loadParameters()
  -> 从 ROS 参数服务器读取 tracking_* 参数到 params_
  -> config_ = sanitizeTrackerParams(params_)
```

`params_` 是原始参数快照，保留用户配置语义；`config_` 是运行时真正使用的合法化配置。后续热路径基本都读 `config_`，不再重复处理非法值。

读取的参数可以按用途分组理解：

1. 生命周期参数：`tracking_history_size`、`tracking_max_missed_frames`、`tracking_min_hits_to_publish`。
2. 数据关联参数：`tracking_association_gate_confidence`、`tracking_coasting_gate_relax_factor`、`tracking_spawn_suppression_distance`、`tracking_spawn_suppression_iou_threshold`。
3. bbox 平滑参数：`tracking_box_size_smoothing_alpha`。
4. 时间步长参数：`tracking_default_dt`、`tracking_max_dt`。
5. 类别分类参数：`classification_*` 和 `classify_*`，用于控制开始分类帧数、类别证据积分、临时合并/分离保护和尺寸/质心阈值。
6. Kalman 噪声和协方差参数：只从 `kalman_filter/...` 分组读取，避免旧顶层参数和当前分组参数同时表达同一套滤波语义。

### 3.2 `sanitizeTrackerParams()`

`sanitizeTrackerParams()` 是匿名命名空间里的工具函数，它把 `DynamicObjectTrackerParams` 转成 `DynamicObjectTrackerConfig`。

主要转换顺序是：

```text
sanitizeTrackerParams(params)
  -> 裁剪 history / missed / min_hits 等计数参数
  -> association_gate_confidence 转成 3D 卡方 gate 阈值
  -> 合法化 coasting gate 放宽倍数
  -> 裁剪 bbox size 平滑系数到 [0, 1]
  -> 合法化 fallback / spawn suppression 的 distance 和 IoU 阈值
  -> 合法化 max_dt
  -> 填充 filter_config
```

这里有几个容易读错的点：

1. `tracking_association_gate_confidence` 不是直接当距离阈值用，而是先通过 `chiSquare3DThresholdFromConfidence()` 转成 3 自由度马氏距离平方阈值。
2. 新轨迹默认使用 `CV3D`，对外类别先发布 `UNKNOWN`；达到 `classification_start_frame` 后才开始给类别证据积分。
3. `classification_interval_sec`、`classification_stable_confirmations` 和 `classify_xy_distance_threshold` 不进入当前实现，类别稳定由分数和 margin 判断。
4. `kalman_filter/.../meas_noise` 当前直接作为三轴位置观测噪声方差使用，不再额外平方。
5. `spawn_suppression_distance` 和 `spawn_suppression_iou_threshold` 有两个用途：马氏距离 gate 失败后的回退关联，以及最终未匹配 detection 的建轨抑制。

## 4. 每帧输入怎样走到 tracker

### 4.1 `Ldop::processingLoop()`

后台线程从输入队列中取一帧点云/odom 快照：

```text
processingLoop()
  -> 等待 input_queue_
  -> 取出最旧 InputFrame
  -> processed = processFrame(frame)
  -> publishFrame(processed)
```

tracker 不参与队列管理。队列、互斥锁、条件变量都属于 `Ldop` 主流程。

### 4.2 `Ldop::processFrame()`

跟踪模块在 `processFrame()` 的后半段被调用：

```text
processFrame(frame)
  -> ufomap_mapper_->processInputCloud(...)
  -> dynamic_object_clusterer_->processDynamicObjects(...)
  -> dynamic_object_tracker_->processDynamicTracks(processed.dynamic_cloud_msg.header,
                                                   cluster_result.detections)
  -> 保存 dynamic_objects_msg
  -> 保存 dynamic_track_markers_msg
```

这里传给 tracker 的 header 来自 `processed.dynamic_cloud_msg.header`。所以 tracker 使用的是动态点云输出的时间戳和坐标系，而不是自己重新拼 header。

## 5. 单帧跟踪主入口

### 5.1 `processDynamicTracks()` 的总流程

`DynamicObjectTracker::processDynamicTracks()` 是跟踪模块的唯一模块级主流程入口。它接收当前帧 header 和 detections，返回：

1. `dynamic_objects_msg`：对外发布的稳定动态目标。
2. `dynamic_track_markers_msg`：RViz 轨迹历史线和轨迹头。
3. `timing`：跟踪模块内部阶段耗时。

代码顺序是：

```text
processDynamicTracks(header, detections)
  -> predictTracks(header.stamp)
  -> buildCostMatrix(detections, tracks_, ...)
  -> selectAssociations(original_cost, invalid_cost)
  -> 对 matches 调 updateMatchedTrack()
       -> prepareMatchedTrackForUpdate()
          -> 临时合并/分离保护
          -> 可靠观测更新类别证据
          -> 确认类别变化时切换模型
  -> 对未匹配 detection 做 shouldSuppressTrackSpawn() / createTrack()
  -> 对未匹配旧 track 调 coastTrack()
  -> deleteExpiredTracks()
  -> buildOutput(header)
  -> buildTrackMarkers(header)
  -> verbose 时打印 timing 和计数
  -> return result
```

这段代码里的几个临时变量是阅读主线：

1. `matched_detections`：长度等于当前帧 detection 数量，记录哪些 detection 已经匹配到旧轨迹。
2. `matched_tracks`：长度等于进入本帧时的 `tracks_.size()`，记录哪些旧轨迹已经匹配到 detection。
3. `original_cost`：detection 行、track 列的关联代价矩阵。
4. `matches`：Hungarian 之后保留下来的真实 detection-track 匹配。
5. `suppressed_track_spawns`：被建轨抑制挡住的未匹配 detection 数量，只用于日志。

## 6. 第一步：预测旧轨迹

### 6.1 `predictTracks()`

进入数据关联前，所有已有轨迹都会先预测到当前帧时间：

```text
predictTracks(stamp)
  for track in tracks_:
    dt = computeDeltaSeconds(stamp, track.last_stamp, default_dt, max_dt)
    track.filter->setDt(dt)
    track.filter->predict()
    updateBBoxCenterFromFilter(track)
```

这一步只处理旧轨迹。如果当前是第一帧，`tracks_` 为空，这个循环什么都不做。

关键语义：

1. `computeDeltaSeconds()` 处理时间戳异常：当前时间或上一帧时间为 0、时间差非正数时，回退到 `tracking_default_dt`。
2. `dt` 最后会被 `tracking_max_dt` 截断，避免长时间暂停后一次预测跳太远。
3. `setDt(dt)` 只把当前时间步长交给运动模型，不执行预测，也不在这里重置 Q/R。
4. `predict()` 才真正执行状态外推。
5. `updateBBoxCenterFromFilter()` 把 `track.bbox.center` 覆盖成预测后的滤波器位置。

### 6.2 `KalmanFilterBase::setDt()`

`setDt()` 在滤波器内部只调用运动模型的 `setDt()`：

```text
KalmanFilterBase::setDt(dt)
  -> model_->setDt(dt)
```

这和 LDOT 对齐：`setDt()` 的职责只是保存本帧预测要用的时间步长。真正的矩阵读取发生在更明确的位置：

1. `predict()` 调用 `model_->getTransitionF(state_)` 得到当前 dt 下的 F。
2. 关闭自适应 Q/R 时，`predict()` 调用 `model_->getProcessNoiseQ()` 得到当前 dt 下的基础 Q。
3. `initialize()` 调用 `model_->getInitState()`、`getInitCovP()`、`getProcessNoiseQ()`、`getMeasNoiseR()` 初始化状态、协方差和基础噪声。
4. R 不依赖 dt，只作为基础测量噪声、自适应 R 初值和下限来源。

### 6.3 `KalmanFilterBase::predict()`

`predict()` 只使用当前运动模型，不引入 detection：

```text
predict()
  -> F = model_->getTransitionF(state_)
  -> 如果 adaptive_window_size <= 1，process_noise_ = model_->getProcessNoiseQ()
  -> state_ = F * state_
  -> covariance_ = F * covariance_ * F^T + process_noise_
  -> clampCovariance()
```

上面的伪代码适合 `LinearKalmanFilter`。`ExtendedKalmanFilter` 对 CTRA 采用同一协方差传播形式，但转移雅可比会先在预测前状态处计算，再调用 `stateTransition()` 推进状态；这样 CTRA 的 yaw/yaw_rate 不会在协方差传播里被等效多推进一步。

预测结束后，tracker 把轨迹 bbox 中心同步为 `filter->position()`。因此后面的关联、建轨抑制和 marker 使用的 `track.bbox.center` 已经是当前帧预测位置。

## 7. 第二步：构造关联代价矩阵

### 7.1 `buildCostMatrix()`

预测完成后，代码对每个 detection-track 组合计算代价：

```text
buildCostMatrix(detections, tracks, config, gate, relax, invalid)
  -> 初始化 matrix[detection_count][track_count] = invalid_cost
  -> 遍历每个 detection
  -> 遍历每条 track
  -> computeMahalanobisCost(detection_position, track)
  -> 根据 track 是否 coast 决定 relaxed_gate
  -> 如果马氏距离通过 gate，写入 cost
  -> 否则尝试 computeFallbackAssociationCost()
```

如果当前没有旧轨迹，`tracks_.size() == 0`，矩阵列数为 0。后续 `selectAssociations()` 会返回空匹配，所有 detection 都会进入“未匹配 detection”分支。

如果当前没有 detection，矩阵行数为 0。后续没有匹配，所有旧轨迹都会进入 coast 分支。

### 7.2 `computeMahalanobisCost()`

该函数只负责一个 detection 和一条 track 的主关联代价：

```text
computeMahalanobisCost(detection_position, track)
  -> innovation = detection_position - track.filter->position()
  -> 取 filter covariance 的位置 3x3 块
  -> 强制协方差对称
  -> 加 1e-6 正则项
  -> cost = innovation^T * inverse(position_covariance) * innovation
```

这里的 `detection_position` 是点簇质心，`track.filter->position()` 是本帧预测后的轨迹中心。代价越小，说明 detection 越符合该轨迹当前预测和不确定性。

### 7.3 coasting gate 放宽

`buildCostMatrix()` 会根据 `track.missed_frames` 调整 gate：

```text
if track.missed_frames > 0:
  relaxed_gate = association_gate_threshold * coasting_gate_relax_factor
else:
  relaxed_gate = association_gate_threshold
```

含义是：漏检中的轨迹已经有一段时间没被观测到，预测不确定性和实际偏差都可能更大，所以允许更宽的重关联窗口。

### 7.4 `computeFallbackAssociationCost()`

只有马氏距离 gate 失败后，才会进入回退关联：

```text
computeFallbackAssociationCost(detection, track, config, rejected_gate_cost, invalid)
  -> 如果 distance 和 IoU 回退都关闭，返回 invalid
  -> 如果开启 distance：
       center_distance 必须 <= spawn_suppression_distance
       distance_penalty = center_distance / spawn_suppression_distance
  -> 如果开启 IoU：
       iou 必须 >= spawn_suppression_iou_threshold
       iou_penalty = 1 - iou
  -> fallback_cost = rejected_gate_cost + 1 + penalty
```

这个 cost 被故意设成高于 gate 内马氏距离匹配，所以正常情况下仍优先选择马氏距离匹配。回退关联主要用来处理目标原地旋转、小半径旋转或检测中心短时跳动导致马氏距离失败的情况。

## 8. 第三步：Hungarian 全局匹配

### 8.1 `selectAssociations()`

`selectAssociations()` 把原始矩阵扩展成方阵，然后用 Hungarian 算法做一对一全局最小匹配：

```text
selectAssociations(original, invalid_cost)
  -> 如果矩阵为空，返回 {}
  -> detection_count = original.size()
  -> track_count = original.front().size()
  -> dimension = detection_count + track_count
  -> 构造 dimension x dimension 方阵 cost
  -> 真实 detection-track 区域填 original cost
  -> dummy 区域填 unmatched_penalty
  -> 运行 Hungarian 势能更新和增广
  -> 转回 assigned_column_for_row
  -> 只保留真实 detection-track 且 raw_cost < invalid_cost 的匹配
```

dummy 行/列的作用是显式表达“这个 detection 不匹配任何 track”或“这个 track 不匹配任何 detection”。最后返回的 `matches` 只包含真实 detection-track 对。

### 8.2 匹配结果怎样回到主流程

`matches` 中每个元素是 `AssociationCandidate`：

```text
AssociationCandidate {
  detection_index
  track_index
  raw_cost
}
```

主流程会用它做两件事：

1. 把 `matched_detections[detection_index]` 置为 `true`。
2. 把 `matched_tracks[track_index]` 置为 `true`。

然后调用 `updateMatchedTrack(tracks_[track_index], detections[detection_index], header.stamp)`。

## 9. 第四步：更新匹配成功的轨迹

### 9.1 `updateMatchedTrack()`

匹配成功后，轨迹进入 update 分支：

```text
updateMatchedTrack(track, detection, stamp)
  -> matched_hits = track.hits + 1
  -> prepareMatchedTrackForUpdate(track, detection, matched_hits)
  -> track.filter->update(detection.bbox.center)
  -> 保存 previous_size
  -> track.bbox = detection.bbox
  -> 对稳定后的 bbox.size 做 EMA 平滑
  -> updateBBoxCenterFromFilter(track)
  -> age += 1
  -> hits += 1
  -> missed_frames = 0
  -> last_stamp = stamp
  -> appendHistorySample(track, stamp, true, max_history_size)
```

两个点特别重要：

1. 分类和模型切换发生在 Kalman update 之前，保证当前帧 detection 被新模型吸收。
2. Kalman update 只接收 `detection.bbox.center`，即 3D 位置观测；bbox size 不进入滤波器状态。
3. `track.bbox = detection.bbox` 后，`bbox.center` 又会被 `updateBBoxCenterFromFilter()` 覆盖成滤波器位置；所以最终轨迹中心来自滤波器，尺寸来自稳定尺寸平滑。

### 9.1.1 `prepareMatchedTrackForUpdate()`

这一步是 LDOT 分类/切模型逻辑在 LDOP 中的落点：

```text
prepareMatchedTrackForUpdate(track, detection, matched_hits)
  -> updateClassificationSizeState()
  -> 如果 hits 不足 classification_start_frame，返回稳定尺寸
  -> 如果当前帧疑似临时合并/分离，冻结类别证据
  -> classifyObservation()
  -> classification_scores 做 decay + increment
  -> 分数和 margin 达标后确认类别
  -> 类别变化时 switchTrackModel()
```

`classification_scores` 只在可靠匹配帧更新。coasting、临时粘连和遮挡残片都不会降低分数；缺少观测只表示没有新证据，不表示类别被推翻。

### 9.2 `KalmanFilterBase::update()`

滤波器的 `update()` 只负责单条轨迹的状态校正：

```text
update(measurement)
  -> 检查 measurement 是否有限
  -> innovation = measurement - H * state_
  -> updateAdaptiveNoise(innovation, predicted_covariance)
  -> S = H * P * H^T + R
  -> K = P * H^T * S^-1
  -> state_ += K * innovation
  -> Joseph 形式更新 covariance_
  -> clampCovariance()
```

tracker 不直接改 `state_` 或 `covariance_`，只通过 `position()`、`velocity()`、`covariance()` 读取滤波器结果。

### 9.3 `appendHistorySample()`

匹配成功后会追加一条历史样本：

```text
appendHistorySample(track, stamp, matched=true, max_history_size)
  -> sample.stamp = stamp
  -> sample.object_class = track.object_class
  -> sample.motion_model_type = track.model_type
  -> sample.model_state = track.filter->state()
  -> sample.model_covariance = track.filter->covariance()
  -> sample.matched = true
  -> push_back(sample)
  -> 如果 history_size 为 0，清空 history
  -> 如果超过上限，删除最旧样本
```

历史样本既用于 RViz 轨迹线，也可以作为后续调试轨迹状态和第三阶段预测输入的依据。当前历史保存模型原生状态和协方差，而不是固定公共状态；读取方必须结合 `motion_model_type` 解释状态布局。

## 10. 第五步：处理未匹配 detection

匹配成功的 detection 已经被 `matched_detections` 标记。主流程随后遍历所有 detection：

```text
for detection_index in detections:
  if !matched_detections[detection_index]:
    if shouldSuppressTrackSpawn(detection, tracks_, config_):
      suppressed_track_spawns += 1
      continue
    createTrack(detection, header.stamp)
```

也就是说，未匹配 detection 不一定创建新 ID，它会先经过建轨抑制。

### 10.1 `shouldSuppressTrackSpawn()`

建轨抑制用于避免同一目标因为中心抖动或 bbox 抖动被裂成多个 ID：

```text
shouldSuppressTrackSpawn(detection, tracks, config)
  -> 如果 distance 和 IoU 抑制都关闭，返回 false
  -> 遍历所有已有 tracks
  -> 如果 IoU >= spawn_suppression_iou_threshold，返回 true
  -> 如果 center_distance <= spawn_suppression_distance，返回 true
  -> 否则返回 false
```

注意：这里和 fallback 关联不同。fallback 关联是给 detection-track 一个次优匹配候选；建轨抑制是最终仍未匹配时，决定“要不要创建新轨迹”。两者复用同一组 distance / IoU 阈值，但发生在不同阶段。

### 10.2 `createTrack()`

如果未匹配 detection 没有被抑制，就创建新轨迹：

```text
createTrack(detection, stamp)
  -> TrackState track(next_track_id_++, CV3D, filter_config)
  -> track.filter->initialize(detection.bbox.center)
  -> track.bbox = detection.bbox
  -> object_class = Unknown
  -> max_observed_size = detection.bbox.size
  -> last_point_count = detection.point_count
  -> updateBBoxCenterFromFilter(track)
  -> age = 1
  -> hits = 1
  -> missed_frames = 0
  -> last_stamp = stamp
  -> appendHistorySample(track, stamp, true, max_history_size)
  -> tracks_.push_back(std::move(track))
```

`next_track_id_++` 是稳定 ID 的来源。它只递增，不因为旧轨迹删除而复用旧 ID。

### 10.3 `TrackState` 构造函数

`TrackState` 构造函数只创建单条轨迹的滤波器：

```text
TrackState::TrackState(track_id, model_type, filter_config)
  -> id = track_id
  -> model_type = track_model_type
  -> filter = createKalmanFilter(model_type, filter_config)
  -> 如果 filter == nullptr，抛异常
```

这一步之后，滤波器对象已经存在，但新轨迹的位置状态还没有设置成 detection。真正把首帧 detection 写入位置状态的是后面的 `track.filter->initialize(...)`。

### 10.4 `createKalmanFilter()`

当前 factory 已补齐 LDOT 对应的模型入口：

```text
createKalmanFilter(model_type, config)
  -> CA2D: CAMotionModel(use_3d=false) + LinearKalmanFilter
  -> CA3D: CAMotionModel(use_3d=true) + LinearKalmanFilter
  -> CV3D: CVMotionModel + LinearKalmanFilter
  -> CTRA: CTRAMotionModel + ExtendedKalmanFilter
```

这仍然不是 IMM：没有多模型概率、模型混合或模型转移矩阵。当前 tracker 新轨迹默认创建 `MotionModelType::CV3D`；确认类别后，`switchTrackModel()` 会在 matched-track update 前切到 `CA2D/CTRA/CA3D/CV3D`。

### 10.5 `KalmanFilterBase` 构造与 `initialize()`

`createKalmanFilter()` 会先创建运动模型，再创建对应的 KF/EKF：

```text
createKalmanFilter(CV3D, config)
  -> model = CVMotionModel(config.cv)
  -> model->setDt(config.default_dt)
  -> filter = LinearKalmanFilter(model)
  -> filter->setAdaptiveParams(...)
  -> filter->setCovLimitParams(...)
```

随后 `createTrack()` 会再次调用：

```text
track.filter->initialize(detection.bbox.center)
```

第二次初始化才是新轨迹真正进入业务语义的初始化。它会：

```text
initialize(measurement)
  -> 检查 measurement 是否有限
  -> state_ = model_->getInitState(measurement)
  -> covariance_ = model_->getInitCovP()
  -> process_noise_ = model_->getProcessNoiseQ()
  -> measurement_noise_ = model_->getMeasNoiseR()
  -> 清空 innovation_buffer_
  -> clampCovariance()
```

这意味着新轨迹首帧位置直接等于 detection center，速度和加速度从 0 开始估计。

## 11. 第六步：处理未匹配旧轨迹

主流程接着遍历进入本帧时已有的 tracks：

```text
for track_index in matched_tracks:
  if !matched_tracks[track_index]:
    coastTrack(tracks_[track_index], header.stamp)
```

`matched_tracks` 的长度是在 `createTrack()` 之前确定的，所以本帧新建的轨迹不会马上进入 coast 分支。

### 11.1 `coastTrack()`

未匹配旧轨迹进入 coast 状态：

```text
coastTrack(track, stamp)
  -> age += 1
  -> missed_frames += 1
  -> last_stamp = stamp
  -> updateBBoxCenterFromFilter(track)
  -> appendHistorySample(track, stamp, false, max_history_size)
```

coast 前，本帧开头已经执行过 `predictTracks()`。所以 `coastTrack()` 不再调用 `predict()`，它只是记录“这一帧没有匹配到观测，但轨迹仍暂时保留”。

coast 状态会影响下一帧：

1. `missed_frames > 0` 时，`buildCostMatrix()` 会使用更宽的 gate。
2. marker 透明度会降低。
3. 如果后续匹配成功，`updateMatchedTrack()` 会把 `missed_frames` 清零。

## 12. 第七步：删除过期轨迹

### 12.1 `deleteExpiredTracks()`

所有 update/create/coast 分支处理完后，主流程调用：

```text
deleteExpiredTracks()
  -> 删除 missed_frames > max_coast_frames 的 track
```

注意这里是严格大于，不是大于等于。如果 `tracking_max_missed_frames` 合法化后是 5，那么连续漏检计数到 6 才删除。

删除发生在构建输出之前，所以过期轨迹不会再出现在本帧 `dynamic_objects_msg` 或 marker 中。

## 13. 第八步：构建对外输出

### 13.1 `buildOutput()`

`buildOutput()` 生成 `/ldop/dynamic_objects` 对应的消息：

```text
buildOutput(header)
  -> object_array.header = header
  -> 把 tracks_ 指针收集到 ordered_tracks
  -> 按 track id 升序排序
  -> 遍历 ordered_tracks
  -> hits < min_hits_to_publish 的轨迹跳过
  -> 填 object.id / size / model_state / model_covariance / object_class / motion_model_type
  -> push_back(object)
```

输出字段来源：

1. `object.id` 来自 `track.id`。
2. `object.size` 来自 `track.bbox.size`，匹配成功时做过 EMA 平滑。
3. `object.model_state` 来自 `track.filter->state()`；所有模型前三维固定为滤波后的目标中心。
4. `object.model_covariance` 来自 `track.filter->covariance()`，按模型原生状态维度做 row-major 展开。
5. `object.object_class` 来自 tracker 内部确认类别；分类前是 `CLASS_UNKNOWN`，确认后是 `CLASS_HUMAN / CLASS_VEHICLE / CLASS_UAV / CLASS_OTHER`。
6. `object.motion_model_type` 来自 `track.model_type`，下游应按该字段解释 `model_state` 和 `model_covariance`。

当前 `DynamicObject.msg` 不单独发布 `center`、`velocity` 或固定 6x6 公共协方差。常规中心位置直接读取 `model_state[0..2]`；速度需要按 `motion_model_type` 从模型状态转换，例如 CTRA 的世界系水平速度为 `v*cos(yaw), v*sin(yaw), 0`。

`buildOutput()` 不检查 `missed_frames == 0`。因此已经满足 `hits` 阈值且尚未删除的 coasting 轨迹，也可能继续对外发布。

### 13.2 `buildTrackMarkers()`

`buildTrackMarkers()` 生成 `/ldop/dynamic_track_markers` 对应的 RViz marker：

```text
buildTrackMarkers(header)
  -> push DELETEALL marker
  -> 遍历 tracks_
  -> coasting = track.missed_frames > 0
  -> makeTrackColor(coasting)
  -> 构建 dynamic_track_history LINE_STRIP
  -> history 点数 >= 2 才发布 history marker
  -> 构建 dynamic_track_head SPHERE
  -> 构建 dynamic_track_label TEXT_VIEW_FACING
```

marker 语义：

1. `dynamic_track_history` 是历史轨迹线，来自 `track.history`。
2. `dynamic_track_head` 是当前轨迹头部球体，位置是 `track.bbox.center`。
3. `dynamic_track_label` 是目标上方文字，显示稳定 `track.id`、当前确认类别和速度模长。
4. coasting 轨迹 alpha 更低，用来表示当前帧没有匹配到 detection。
5. `LINE_STRIP` 少于 2 个点时不发布，避免 RViz 报错。

## 14. 最后一段：返回并发布

`processDynamicTracks()` 把输出装进 `DynamicObjectTrackerFrameResult`：

```text
result.dynamic_objects_msg = buildOutput(header)
result.dynamic_track_markers_msg = buildTrackMarkers(header)
result.timing.process_dynamic_tracks_ms = ...
return result
```

然后回到 `Ldop::processFrame()`：

```text
processed.dynamic_objects_msg = tracking_result.dynamic_objects_msg
processed.dynamic_track_markers_msg = tracking_result.dynamic_track_markers_msg
```

最后 `Ldop::publishFrame()` 发布：

```text
dynamic_objects_pub_.publish(frame.dynamic_objects_msg)
dynamic_track_markers_pub_.publish(frame.dynamic_track_markers_msg)
```

到这里，一帧的跟踪处理才真正对外可见。

## 15. 三种典型帧的执行路径

### 15.1 第一帧检测到目标

假设 `tracks_` 为空，当前帧有 detections：

```text
processDynamicTracks()
  -> predictTracks(): 无旧轨迹，跳过
  -> buildCostMatrix(): 0 列矩阵
  -> selectAssociations(): 无 matches
  -> updateMatchedTrack(): 不执行
  -> 每个 detection 进入未匹配分支
  -> shouldSuppressTrackSpawn(): 没有旧轨迹，通常 false
  -> createTrack(): 创建 id=0、id=1...
  -> coastTrack(): matched_tracks 原本为空，不执行
  -> deleteExpiredTracks()
  -> buildOutput(): hits 是否达到 min_hits_to_publish 决定是否发布
  -> buildTrackMarkers(): 新轨迹头部球体和文字会发布，history 不足 2 点时不发布线
```

### 15.2 后续帧正常匹配

假设已有轨迹，当前 detection 能匹配：

```text
processDynamicTracks()
  -> predictTracks(): 旧轨迹先按 dt 外推
  -> buildCostMatrix(): 马氏距离通过 gate
  -> selectAssociations(): 得到 detection-track match
  -> updateMatchedTrack(): Kalman update、size 平滑、hits++、missed=0
  -> 未匹配 detection 分支跳过已匹配 detection
  -> 未匹配 track 分支跳过已匹配 track
  -> deleteExpiredTracks()
  -> buildOutput()
  -> buildTrackMarkers()
```

这是最标准的稳定跟踪路径。

### 15.3 目标短暂漏检

假设已有轨迹，但当前帧没有对应 detection：

```text
processDynamicTracks()
  -> predictTracks(): 旧轨迹先预测到当前帧
  -> buildCostMatrix(): 没有可用匹配
  -> selectAssociations(): 无 matches
  -> updateMatchedTrack(): 不执行
  -> createTrack(): 若没有未匹配 detection，也不执行
  -> coastTrack(): 旧轨迹 missed_frames++
  -> deleteExpiredTracks(): 超过 max_coast_frames 才删除
  -> buildOutput(): 若 hits 足够且未删除，coasting 轨迹仍可发布
  -> buildTrackMarkers(): 头部、历史线和文字透明度降低
```

这条路径解释了为什么目标短暂消失时，轨迹不会马上断掉。

## 16. 读代码时的断点顺序

如果要在 IDE 里按执行流程调试，建议按下面顺序打断点：

```text
Ldop::processFrame()
DynamicObjectTracker::processDynamicTracks()
DynamicObjectTracker::predictTracks()
buildCostMatrix()
computeMahalanobisCost()
computeFallbackAssociationCost()
selectAssociations()
DynamicObjectTracker::updateMatchedTrack()
DynamicObjectTracker::createTrack()
DynamicObjectTracker::coastTrack()
DynamicObjectTracker::deleteExpiredTracks()
DynamicObjectTracker::buildOutput()
DynamicObjectTracker::buildTrackMarkers()
```

如果问题是“为什么没有生成新 ID”，优先看：

1. `matches` 是否已经匹配到了旧轨迹。
2. `shouldSuppressTrackSpawn()` 是否返回 true。
3. `createTrack()` 是否被调用。

如果问题是“为什么 ID 断了”，优先看：

1. `computeMahalanobisCost()` 的 cost。
2. `relaxed_gate` 是否足够。
3. `computeFallbackAssociationCost()` 是否被 distance 或 IoU 拒绝。
4. `missed_frames` 是否超过 `max_coast_frames`。

如果问题是“为什么 RViz 还有轨迹但输出没有目标”，优先看：

1. `track.hits` 是否小于 `min_hits_to_publish`。
2. `buildOutput()` 是否跳过了该轨迹。
3. `buildTrackMarkers()` 是否仍在发布 head marker。

## 17. 一句话流程总结

跟踪模块的主流程可以概括为：

```text
构造时读取并合法化参数
  -> 每帧先预测旧轨迹
  -> 用 detection 质心和预测轨迹做全局关联
  -> 匹配成功则 update
  -> 未匹配 detection 可能 create
  -> 未匹配旧轨迹进入 coast
  -> 超过漏检阈值则 delete
  -> hits 达标的轨迹输出 dynamic_objects
  -> 所有未删除轨迹输出 RViz track markers
```

`DynamicObjectTracker` 负责“轨迹生命周期和数据关联”，`KalmanFilterBase` / `LinearKalmanFilter` / `ExtendedKalmanFilter` 负责“单条轨迹的位置、速度、加速度或 CTRA 状态估计”。把这两个职责分开看，代码执行顺序会清楚很多。

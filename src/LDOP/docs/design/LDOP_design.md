## LDOP 整体功能定位
- `LDOP` 的长期目标是成为无人机自主系统中的动态环境感知与规划地图增强模块。
- 外部输入默认只有两类：
  - LiDAR 点云
  - 无人机位姿/里程计
- 当前实现路线基于：
  - DUFOMap 的动静态判别方法
  - UFOMap 底层三维体素地图
  - ROS 话题输出与 RViz 可视化
- 设计时应始终区分两类环境表达：
  - 静态层：用于构建干净、稳定、可膨胀、可查询的静态地图
  - 动态层：用于检测、跟踪、预测动态障碍物，并形成未来风险
- 不要把动态目标长期污染进静态地图。动态信息应作为独立风险层或融合层输出。

## 四阶段路线图

### 第一阶段：动静态点云分割与静态地图构建
- 当前状态：
  - 已完成。当前 `LDOP` 已经发布当前帧静态点云、当前帧动态点云，并通过 `/ldop/static_map_markers` 输出静态体素地图 RViz 可视化。
  - 已为第四阶段打下基础查询能力：`UfomapMapper` 能返回显式 `Unknown/Free/Occupied/OutOfMap` 状态，支持按 query depth 的局部 occupied 查询和最近 occupied 查询；这些是 planner-facing 接口的底层能力，不等同于第四阶段 service 已完成。
- 目标：
  - 基于 DUFOMap `seenFree()` / void-region 思路判别每帧点云中的静态点和动态点。
  - 基于 UFOMap 底层地图构建可观察的静态体素地图。
- 主要输入：
  - 原始点云
  - 无人机位姿/里程计
- 主要处理：
  - 点云插入 UFOMap
  - 基于 `seenFree(point)` 做动静态判别
  - 对内部使用方提供只读 UFOMap 查询，包括显式状态查询、局部 occupied 查询和最近 occupied 查询
  - 输出当前帧静态点云
  - 输出当前帧动态点云
  - 遍历 UFOMap 静态占用体素并生成可视化
- 必须输出：
  - `/ldop/static_cloud`：当前帧静态点云
  - `/ldop/dynamic_cloud`：当前帧动态点云，用于观察动态残差
  - `/ldop/static_map_markers` 或等价话题：静态体素地图 RViz 可视化
- 验收重点：
  - 动态物体不应明显污染 `/ldop/static_cloud`
  - `/ldop/dynamic_cloud` 不再是空壳
  - RViz 中能看到 UFOMap 静态体素地图

### 第二阶段：动态点云聚类、检测与跟踪
- 当前状态：
  - 已完成。当前 `LDOP` 已经把动态点云聚类为 detection，经 tracker 输出稳定 `DynamicObjectArray`，并提供当前帧检测框和跨帧轨迹两组 RViz 可视化。
- 目标：
  - 将第一阶段输出的动态点云从“动态点集合”提升为“动态目标集合”。
  - 对动态目标进行跨帧关联，形成稳定目标 ID、对象类别、模型原生状态及协方差。
- 主要输入：
  - `/ldop/dynamic_cloud`
  - 无人机位姿/里程计
  - 可选的 UFOMap label / clustering 结果
- 主要处理：
  - 动态点云聚类
  - 小簇过滤
  - 包围盒或尺寸估计
  - 目标中心估计
  - 基于 bbox 尺寸、质心高度和轨迹历史进行 human / vehicle / UAV / other 分类
  - 跨帧数据关联
  - 根据确认类别在 CA2D / CTRA / CA3D / CV3D 间切换运动模型
  - 维护目标模型原生状态和协方差；预测侧按 `motion_model_type` 解释状态布局
- 必须输出：
  - `/ldop/dynamic_objects`：包含目标 id、尺寸、模型原生状态 `model_state`、模型原生协方差 `model_covariance`、`motion_model_type` 和 `object_class`；目标中心固定在 `model_state[0..2]`，速度由 `motion_model_type` 对应的状态布局解释
  - 可选 `/ldop/dynamic_clusters`：带聚类结果的动态点云
  - 可选 `/ldop/dynamic_object_markers`：当前帧目标框 RViz 可视化
  - `/ldop/dynamic_track_markers`：轨迹历史线、头部球和目标上方 ID/类别/速度文字
- 验收重点：
  - 同一动态目标在连续帧中尽量保持稳定 ID
  - 目标尺寸、模型原生状态及协方差随时间连续
  - 新轨迹分类前保持 `UNKNOWN`，类别证据稳定后再发布 human / vehicle / UAV / other
  - 聚类错误不应反向破坏第一阶段静态地图输出

### 第三阶段：动态目标短时轨迹预测
- 当前状态：
  - 已完成。轨迹预测内部的 P1 轨迹外推、P2 航向/意图 GMM 多分支、P3 交互感知预测、P4 predictor 内部在线闭环反馈均已落地。
  - 已通过 `PredictionMapQuery` 复用第一阶段 UFOMap 查询基础：P3 地图交互可以读取显式地图状态、按 `prediction_map_query_depth` 调整查询深度，并查询最近 occupied 证据。该能力用于预测交互和 P4 复用基础，不代表第四阶段规划地图接口已经完成。
- 目标：
  - 对第二阶段稳定跟踪到的动态目标进行 1-3 秒短时预测。
  - 为规划器提供未来动态障碍物位置和风险演化。
- 主要输入：
  - 已跟踪目标的状态历史，按 `motion_model_type` 解释模型原生状态及协方差
  - 无人机当前位姿/时间戳
- 主要处理：
  - 基于当前运动模型的轨迹外推
  - 航向/意图 GMM 多分支预测
  - 目标间交互、静态地图约束和 free corridor 方向先验
  - 通过只读地图 adapter 使用显式 occupancy state、coarse/fine query depth 和最近 occupied 证据，保持 predictor 不拥有或修改 UFOMap
  - 分支概率重权重，并把局部交互修正转成时间平滑控制后重新 rollout
  - 基于上一轮预测 residual / NIS / likelihood 的同语义分支概率反馈和未来过程噪声尺度校准
  - 预测时间采样
  - 预测置信度或风险半径估计
- 必须输出：
  - `/ldop/dynamic_object_predictions`：发布未来时间采样、模型原生预测状态、协方差和 GMM 分支概率，不复用当前跟踪对象消息承载预测数组
  - `/ldop/dynamic_prediction_markers`：预测轨迹 RViz 可视化
  - 后续可选动态风险点云或风险体素
- 验收重点：
  - 预测时间范围默认覆盖 1-3 秒
  - 预测结果与目标当前运动趋势一致
  - 短时预测稳定，不因单帧噪声剧烈跳变
  - 下一帧同一目标的同语义分支概率和未来 `Q` 尺度能随上一轮预测误差校准

### 第四阶段：规划可用的膨胀地图或 ESDF / 风险地图
- 当前状态：
  - 尚未实现 planner-facing ROS service/msg、planning map 或 risk map。
  - UFOMap 应用层借鉴点中的 1、3、4 已作为前三阶段可复用基础落地：显式地图状态、query depth 调节和最近 occupied 查询。
  - 借鉴点 2、5、6 仍归入第四阶段：`BoundingVolume-lite` 几何查询抽象、swept-volume 辅助线段检查、完整/局部地图发布后置增强。
- 目标：
  - 将静态地图和动态预测融合为无人机运动规划可直接使用的安全表达。
  - 地图越强大越好，但必须保持静态层和动态层语义清晰。
- 主要输入：
  - 第一阶段静态地图或静态点云
  - 第三阶段动态目标预测
  - 无人机位姿/规划范围
- 主要处理：
  - 将已落地的底层 occupancy state 和 query depth 包装成稳定 planner-facing 查询语义
  - 批量点、box 或局部范围查询
  - 线段/轨迹边碰撞检查
  - 静态障碍物膨胀
  - unknown 策略和 `INFLATED_OCCUPIED` 状态定义
  - 静态 ESDF 或等价距离场构建
  - 动态障碍物未来位置膨胀
  - 动态风险随预测时间衰减或叠加
  - 静态安全层与动态风险层融合
- 推荐输出：
  - 静态膨胀地图
  - 静态 ESDF 地图
  - 动态障碍物风险地图
  - 融合后的 planning map / risk map
  - 对应 RViz 可视化 marker 或点云
- 设计约束：
  - 静态地图应主要来自第一阶段的静态输出，不应直接使用未过滤原始点云。
  - 动态风险应来自第三阶段预测，不应永久写入静态地图。
  - 若继续复用 `rog_map`，`LDOP` 应提供干净静态输入和动态风险输出；若在 LDOP 内部构建规划地图，也应保留静态层与动态层的可分离表示。

## 阶段承接关系
- 第一阶段是全部后续能力的基础：没有可靠动静态分割，就没有干净静态地图，也没有可信动态目标输入。
- 第二阶段只消费第一阶段的动态点云，不应改变第一阶段静态地图的核心判别语义。
- 第三阶段只在第二阶段稳定跟踪的目标上做预测，不应直接对未聚类动态点云做规划级预测。
- 第四阶段融合第一阶段静态地图和第三阶段动态预测，输出规划可用的安全地图或风险地图。
- 每一阶段都应保留可视化输出，方便单独验证，不要等到最终规划接入后才调试。

## 全局数据与接口约定
- 所有核心输出都必须带正确 `header.stamp` 和 `header.frame_id`。
- 输入点云 `xyz` 与 `odom.pose.pose.position` 必须已经由上游转换到同一个世界坐标系下；这是 LDOP 的输入限制，不是内部猜测或兜底假设。
- LDOP 内部不执行 TF 转换，也不通过 `world_frame` 参数重写坐标系标签；静态点云、动态点云、静态地图 marker、动态目标和轨迹 marker 都沿用输入点云的 `header.frame_id`。
- 如果后续引入 TF 转换，优先放在数据集 adapter 或 LIO/预处理节点中；若必须进入 LDOP，需要显式写清楚转换来源、失败策略和时间同步策略。
- 点云和位姿时间戳不一致时，应记录时间偏差；后续跟踪和预测阶段不能忽略时间间隔。
- 每新增一个稳定能力，都应同步暴露 ROS 话题、参数和最小 RViz 可视化方式。
- 输出话题名应通过参数配置，默认话题名保持稳定，避免下游模块反复改配置。

## 当前优先级
- 第一阶段闭环已经完成：
  - `/ldop/static_cloud`：当前帧静态点云
  - `/ldop/dynamic_cloud`：当前帧动态点云
  - `/ldop/static_map_markers`：静态体素地图 RViz 可视化
- 第二阶段检测跟踪链路已经完成：
  - `/ldop/dynamic_objects`：tracker 输出的稳定动态目标集合
  - `/ldop/dynamic_object_markers`：当前帧检测框
  - `/ldop/dynamic_track_markers`：跨帧轨迹历史、头部点和目标标签
- 第三阶段动态目标短时轨迹预测已经完成：
  - 已完成预测内部 P1/P2/P3/P4：轨迹外推、航向/意图 GMM 多分支、交互感知概率重权重和平滑控制重新 rollout、predictor 内部在线闭环反馈。
  - 已完成 UFOMap 应用层 1、3、4 的跨阶段基础：显式地图状态、可配置 query depth 和最近 occupied 查询；这些能力已经服务 P3 地图交互，并留给第四阶段 planner-facing 接口复用。
  - 离线多 horizon evaluator 仍可后续补充，用于量化 `ADE/FDE/NLL/ECE/Brier/coverage` 等指标，但不再作为第三阶段完成门槛。
- 第三阶段预测只消费第二阶段稳定目标输出，不应直接对未聚类动态点云做规划级预测；第四阶段规划地图再融合第一阶段静态地图和第三阶段动态预测。
- 下一步第四阶段应优先落地 planner-facing 的 2、5、6：`BoundingVolume-lite` 查询抽象、swept-volume 辅助线段检查、完整/局部地图发布后置增强；静态膨胀、unknown 策略和批量点/范围查询仍是 P4-A/P4-B 的接口内容，但不作为单独 UFOMap 借鉴点编号。

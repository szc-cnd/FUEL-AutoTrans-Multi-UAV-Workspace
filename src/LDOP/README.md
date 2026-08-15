# LDOP 设计总结

## 1. 为什么要做 LDOP

`LDOP` 的长期目标，是成为整个自主系统中的“动态环境感知模块”。

它未来应该承担的职责是：

- 读取原始 LiDAR 点云和里程计
- 区分静态信息和动态信息
- 跟踪动态目标并预测它们的运动
- 给 `rog_map` 提供更干净的静态点云输入
- 给规划器或后续融合层提供动态风险信息

这意味着 `LDOP` 不应该去替代 `rog_map`。

更准确地说，`LDOP` 应该成为：

- 让 `rog_map` 的静态地图更干净的模块
- 让规划器具备动态障碍感知能力的模块

## 2. 和论文思路的对应关系

论文《Flying through cluttered and dynamic environments with LiDAR》的整体流程，与 `LDOP` 未来的发展方向高度一致：

1. 定位 / LIO
2. 动态环境感知
3. 地图构建
4. 规划与控制

这篇论文对 `LDOP` 最重要的启发，是它把环境表示拆成了两层：

- 静态层
- 动态层

论文中的关键思想是：

- 静态层只使用静态点来构建
- 动态层使用动态目标未来的预测位置来构建
- 规划时同时考虑静态层和动态层
- SFC 生成时只依赖静态环境

这正好支持我们当前的架构选择：

- `LDOP` 负责动态感知与预测
- `rog_map` 负责静态占据、膨胀和 ESDF
- 规划器或后续融合层负责把静态风险和动态风险结合起来

## 3. 推荐的模块边界

### 当前仓库里 LDOP 已经负责什么

- 读取原始点云和里程计
- 静态点 / 动态点分离
- 发布当前帧静态点云 `/ldop/static_cloud`
- 发布当前帧动态点云 `/ldop/dynamic_cloud`
- 发布由 tracker 维护的 `DynamicObjectArray`
- 发布独立动态目标短时预测 `/ldop/dynamic_object_predictions`
- 发布预测轨迹 RViz 可视化 `/ldop/dynamic_prediction_markers`
- 维护 UFOMap，并提供 `seenFree`、节点查询、候选查询与 label 读写等基础接口
- 为后续静态体素地图可视化提供地图状态基础

### 未来可以继续扩展什么

- 离线多 horizon 轨迹预测评估工具
- 动态风险图或规划接口
- 更完整的回归测试和参数校验

### rog_map 应该负责什么

- 静态占据地图更新
- 障碍膨胀
- ESDF
- 静态碰撞查询
- 静态地图相关可视化

### 规划器或融合层应该负责什么

- 从 `rog_map` 查询静态安全信息
- 从 `LDOP` 查询动态风险信息
- 把两者组合成最终的规划代价或约束

## 4. 当前阶段状态

阶段划分以 `docs/design/LDOP_design.md` 的“四阶段路线图”为准。

按这个定义，当前 `LDOP` 的第一阶段“动静态点云分割与静态地图构建”已经完成：

1. `/ldop/static_cloud` 发布当前帧静态点云
2. `/ldop/dynamic_cloud` 发布当前帧动态点云
3. `UfomapMapper` 维护 UFOMap，并基于 `seenFree(point)` 完成当前帧动静态判别
4. `/ldop/static_map_markers` 输出当前帧静态体素 RViz 可视化；UFOMap 持久化地图仍由内部模块维护

第二阶段“动态点云聚类、检测与跟踪”也已经完成：

1. `DynamicObjectClusterer` 将当前帧动态点云提升为 detection 集合，并通过 `/ldop/dynamic_object_markers` 输出检测框可视化
2. `DynamicObjectTracker` 维护跨帧数据关联、稳定目标 ID、目标尺寸、模型原生状态和协方差
3. tracker 已输出对象类别 `object_class` 和运动模型 `motion_model_type`
4. `/ldop/dynamic_track_markers` 输出轨迹历史线、头部点和目标标签

因此，当前 `LDOP` 已经完成第三阶段：动态目标短时轨迹预测。后续主线可转入第四阶段规划风险层，或先补充离线多 horizon 预测评估工具。

需要注意的是，`/ldop/dynamic_objects` 当前已经由 tracker 输出稳定目标状态，不再是早期的单框占位输出；它仍然不是第三阶段预测接口，预测轨迹走独立话题。

第三阶段内部的轨迹预测能力按 P1-P4 递进理解：

- P1 轨迹外推已完成。
- P2 航向/意图 GMM 多分支预测已完成。
- P3 交互感知预测已完成：目标间交互、静态地图约束、free corridor 方向先验会影响分支概率，并通过时间平滑控制重新 rollout 中心轨迹。
- P4 predictor 内部在线闭环反馈已完成：上一轮预测残差、NIS 和 likelihood 会反馈到下一帧同语义分支概率先验和未来过程噪声尺度。

所以当前应把“轨迹预测 P1-P4 已完成”理解成“LDOP 第三阶段动态目标短时轨迹预测已经完整完成”。离线 evaluator 和 planner-facing 风险地图是后续工具或第四阶段工作，不再作为第三阶段完成门槛。

## 5. 当前 LDOP 的运行结构

当前 `LDOP` 的骨架已经按四层结构拆开了：

- ROS 回调层
- 处理线程
- 发布线程
- `AsyncSpinner`

### 当前数据流

1. 点云回调快速接收最新点云
2. 里程计回调维护最新位姿信息
3. 处理线程按先进先出顺序消费队列中的最旧一帧；只有在队列触顶时才会丢掉更旧输入
4. 发布线程发布处理结果

这种结构很适合作为低时延动态感知的基础，因为它避免了把重计算直接塞进 ROS 回调里。

## 6. 当前已经对外发布的接口

### 已有输出

- `/ldop/static_cloud`
- `/ldop/dynamic_cloud`
- `/ldop/static_map_markers`

### 当前动态对象接口

- `/ldop/dynamic_objects`

消息类型为：

- `ldop/DynamicObjectArray`

配套消息类型为：

- `ldop/DynamicObject`

当前 `ldop/DynamicObject` 保留已跟踪目标的模型原生当前状态：

- `id`
- `size`
- `model_state`
- `model_covariance`
- `object_class`
- `motion_model_type`

其中 `model_state` 前三维固定为目标中心 `x/y/z`；速度需要按 `motion_model_type` 解释：

- `CA2D`：`[x,y,z,vx,vy,ax,ay]`
- `CA3D`：`[x,y,z,vx,vy,vz,ax,ay,az]`
- `CV3D`：`[x,y,z,vx,vy,vz]`
- `CTRA`：`[x,y,z,v,a,yaw,yaw_rate]`

短时间范围内的未来预测位置通过单独的预测话题输出，不再塞进当前跟踪对象消息。

当前实现是检测和跟踪接口：

- `DynamicObjectClusterer` 输出帧内 detection，并继续发布 `/ldop/dynamic_object_markers`
- `DynamicObjectTracker` 输出稳定 `id`、平滑后的 `size`、模型原生状态/协方差、当前运动模型和确认类别
- 跟踪轨迹通过 `/ldop/dynamic_track_markers` 可视化

也就是说，`/ldop/dynamic_objects` 已进入检测跟踪阶段，但还不是预测接口。

### 当前预测接口

- `/ldop/dynamic_object_predictions`
- `/ldop/dynamic_prediction_markers`

`/ldop/dynamic_object_predictions` 输出按目标组织的 branch-level trajectory-level GMM。每个分支包含未来采样点、模型原生状态、模型原生协方差、分支概率和行为族信息。

当前预测链路已经包含轨迹外推、航向/意图多分支、交互感知重权重/重新 rollout，以及 predictor 内部在线闭环反馈。

## 7. LDOP 应该怎样和 rog_map 连接

这里也沿用 `docs/design/LDOP_design.md` 的四阶段路线。

### 第一阶段：动静态点云分割与静态地图构建

当前已经完成。做法是：

- `LDOP` 输入：原始点云 + 里程计
- `LDOP` 输出：`/ldop/static_cloud`、`/ldop/dynamic_cloud`、`/ldop/static_map_markers`
- `rog_map` 如果需要更干净的静态输入，可以把输入点云话题改成 `/ldop/static_cloud`

这一步解决的是基础静态层问题：动态目标不容易在静态地图中留下 ghosting 残影。

### 第二阶段：动态点云聚类、检测与跟踪

当前第二阶段已经完成，把 `/ldop/dynamic_cloud` 从“动态点集合”提升为“动态目标集合”，主链路包括：

- 动态点云聚类
- 目标包围盒或尺寸估计
- 跨帧目标关联
- 模型原生状态与协方差维护，状态布局按 `motion_model_type` 解释
- 目标类别分类
- 类别驱动的 CA2D / CTRA / CA3D / CV3D 运动模型切换
- 稳定输出 `/ldop/dynamic_objects`

这一阶段仍然不应反向改变第一阶段的 `/ldop/static_cloud`、`/ldop/dynamic_cloud` 和静态体素地图语义。

### 第三阶段：动态目标短时轨迹预测

在第二阶段目标跟踪稳定后，新增一组预测输出话题，发布未来状态和预测轨迹。

这时规划器才有比较可靠的未来动态障碍物位置可用。

当前第三阶段已经完成：基础预测话题、GMM 多分支、交互感知预测和 predictor 内部在线闭环反馈均已接入。离线多 horizon evaluator 可以后续补充，但不再阻塞第三阶段收口。

### 第四阶段：规划可用的膨胀地图或 ESDF / 风险地图

最后再把两类信息组合给规划器：

- `rog_map` 或等价模块提供静态安全信息
- `LDOP` 提供动态目标和预测风险

如果后面系统复杂度上来，可以单独做融合层或规划地图接口，但静态层和动态层的语义要继续保持分离。

## 8. 推荐的后续开发顺序

### 第一步：进入第四阶段前的动态风险表达

第一阶段、第二阶段和第三阶段已经完成。下一步应在现有短时轨迹预测基础上，确定规划侧需要消费的动态风险表达。

当前可作为动态风险输入的目标与预测输出是：

- 聚类并跟踪后的稳定目标 ID
- 目标尺寸和对象类别
- 当前运动模型和模型原生状态/协方差
- branch-level trajectory-level GMM 预测分支
- 分支概率、未来模型原生状态/协方差和 `influence_weight`

### 第二步：补充离线预测评估工具

离线 evaluator 不再作为第三阶段完成条件，但仍建议用于量化预测质量和后续论文/实验对比。

建议输出是：

- 多 horizon `ADE/FDE/minADE/minFDE`
- `NLL`、coverage、ECE/Brier 或 NIS consistency
- rosbag 场景统计和压力场景报告

### 第三步：进入第四阶段

让规划器同时接入两类信息：

- `rog_map` 提供静态安全信息
- `LDOP` 提供动态风险信息

## 9. 测试与最终验证

日常开发优先使用轻量级检查：

```bash
cd /home/st/ldop_ws
catkin build ldop --no-status
catkin run_tests ldop --no-status
catkin_test_results /home/st/ldop_ws/build/ldop
```

完整 Gazebo + PX4 + FAST-LIO + LDOP 联合仿真只作为最后验证检查启用，不作为每次改动的默认测试入口：

```bash
cd /home/st/ldop_ws
./src/LDOP/launch/gazebo_px4_LDOP.sh
```

该脚本会依次启动：

1. `roslaunch ldop px4.launch interactive:=false`
2. `roslaunch external_pos_fusion fusion_mapping_mid360.launch`
3. `roslaunch ldop run_ldop.launch`

使用它时主要验证：

- `/ldop/static_cloud` 在 RViz 中保持稳定、不过度吸收动态物体残影
- `/ldop/dynamic_cloud` 能反映当前帧动态点云
- `/ldop/dynamic_objects` 能保持稳定目标 ID、类别、运动模型、模型原生状态和协方差
- `/ldop/dynamic_object_markers` 与 `/ldop/dynamic_track_markers` 分别反映当前帧检测框和跨帧轨迹
- LDOP、PX4、Gazebo、外部位姿融合链路能完整跑通并可正常退出

## 10. 一句话记忆法

以后如果要快速回忆整个设计，可以记住这句话：

`LDOP` 负责告诉系统“什么东西在动、它将往哪里去”；`rog_map` 负责告诉系统“哪些静态空间是安全的”。

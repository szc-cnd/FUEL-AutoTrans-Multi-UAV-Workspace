# 地面滤波与跨帧动态检测 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不新增依赖的前提下，为 LDOP 增加地面平面内点滤波、预热状态、跨帧运动候选和动态地图积分保护。

**Architecture:** 继续以 `UfomapMapper::processInputCloud()` 作为预处理和地图边界。输入点先经过有限值、距离、2.5 米天花板和倾斜地面平面过滤；随后同时运行现有 `seenFree` 分类与上一帧邻域运动检测，合并为当前帧动态掩码。只有掩码之外的点写入 UFOMap，最终动态候选继续交给现有 clusterer/tracker。

**Tech Stack:** ROS Noetic, C++20, UFOMap, `sensor_msgs::PointCloud2`, 现有 LDOP clusterer/tracker；不新增 PCL 或其他依赖。

## Global Constraints

- 本地仓库为 `E:/比赛功能包/LDOP`，远程源码为 `/home/oem/ldop_ws/src/LDOP`。
- 每次源代码或配置修改必须同步本地与远程。
- Git 提交备注使用中文。
- 用户明确不要求自动化测试；以远程编译和手动启动观察作为验证。
- 保留 `ufomap_input_max_z=2.5` 且代码采用严格 `point.z < input_max_z`。

---

### Task 1: 增加运行参数和状态字段

**Files:**
- Modify: `include/ldop/ufomap_mapper.h`
- Modify: `config/ldop.yaml`

**Interfaces:**
- `UfomapMapperParams`/`UfomapMapperConfig` 提供地面估计、单侧地面截止、预热、跨帧匹配和搜索半径参数。
- `UfomapMapper` 保存已处理帧数、锁定的地面平面参数、地面样本和上一帧预处理点。

- [ ] 在头文件中加入地面参数：`ground_filter_enabled`、`ground_estimation_frames`、`ground_estimation_radius`、`ground_estimation_bin_size`、`ground_estimation_min_points`、候选带宽、内点阈值和最大坡度。
- [ ] 加入动态预热和跨帧参数：`warmup_frames`、`temporal_motion_enabled`、`temporal_match_distance`、`temporal_search_radius`。
- [ ] 加入 `dynamic_points` 和当前帧索引集合所需的分类结果字段，并保留现有 ROS 消息字段不变。
- [ ] 在 YAML 中写入默认值和中文说明，保留 `ufomap_input_max_z: 2.5`。

### Task 2: 实现倾斜地面平面估计和滤波

**Files:**
- Modify: `src/ufomap_mapper.cpp`

**Interfaces:**
- 在 `UfomapMapper` 内增加地面候选估计、样本锁定和点过滤的私有逻辑。
- 地面判定只保留 `z > a*x + b*y + c` 的点，不使用固定世界高度带或对称高度容差。

- [ ] 在有限值点和距离过滤之后，从传感器附近、传感器下方候选点的 Z 直方图中选取数量足够的地面候选带。
- [ ] 使用鲁棒平面拟合和连续样本中位数锁定 `a、b、c`，锁定后不被目标点短时变化覆盖。
- [ ] 对点应用 `point.z <= a*x + b*y + c` 的单侧地面过滤；估计未就绪时保留点并输出节流日志。
- [ ] 在 `clearMap()` 中同时清空地面状态、帧快照和帧计数。

### Task 3: 增加跨帧运动候选并保护静态地图

**Files:**
- Modify: `include/ldop/ufomap_mapper.h`
- Modify: `src/ufomap_mapper.cpp`

**Interfaces:**
- `detectTemporalMotion()` 返回当前输入点索引；当前 `seenFree` 候选与这些索引合并后形成最终动态掩码。
- `processInputCloud()` 将最终静态点积分到 UFOMap，并用最终掩码重建当前帧静态/动态点云和 clusterer 输入。

- [ ] 用空间哈希网格查询上一帧点：当前点在匹配半径内无对应点、但在搜索半径内存在上一帧点时标记为运动候选。
- [ ] 对跨帧候选查询 UFOMap：已被 `seenFree` 判定或已有稳定命中体素的点不走补充路径，避免固定表面采样差异造成误检。
- [ ] 预热期间更新快照但清空对外动态候选；预热结束后启用跨帧候选，继续让现有最小点数、聚类和跟踪确认生效。
- [ ] 只积分非动态掩码点，避免动态候选污染 UFOMap。
- [ ] 按最终掩码重建 `static_cloud_msg`、`dynamic_cloud_msg`、`static_points` 和 `dynamic_cluster_points`，确保 RViz 当前帧不会把运动候选继续显示为静态。
- [ ] 在 verbose 日志中输出地面平面/就绪状态、预热状态和跨帧候选点数。

### Task 4: 编译、同步和运行核验

**Files:**
- Verify: `config/ldop.yaml`
- Verify: `include/ldop/ufomap_mapper.h`
- Verify: `src/ufomap_mapper.cpp`

- [ ] 使用 `git diff --check` 检查本地修改。
- [ ] 将所有修改文件复制到 `/home/oem/ldop_ws/src/LDOP` 并核对 SHA256。
- [ ] 远程执行 `source /home/oem/ldop_ws/ldop_env.sh && cd /home/oem/ldop_ws && catkin build ldop --no-status`。
- [ ] 重启 LDOP 后核对 ROS 参数和 `/ldop/dynamic_objects`、`/ldop/dynamic_cloud` 频率。
- [ ] 用中文提交备注提交本次修改，并确认本地工作树干净。

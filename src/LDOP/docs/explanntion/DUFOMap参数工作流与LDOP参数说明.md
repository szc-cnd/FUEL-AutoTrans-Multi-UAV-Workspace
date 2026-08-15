# DUFOMap 官方实现、参数工作流与 LDOP 当前实现说明

这份文档合并了原来的两份说明：

- `DUFOMap官方实现与LDOP当前实现说明.md`
- `DUFOMap参数工作流与LDOP参数说明.md`

目标不是逐个背参数名，而是按 `DUFOMap` / `UFOMap` 的实际工作流程来理解：

1. 点云怎样进入地图
2. 射线怎样把空间写成 hit / miss / seenFree
3. `LDOP` 怎样基于 `seenFree(point)` 把当前帧分成静态和动态

这样理解后，后面调参时可以先判断自己是在调“输入预处理”“积分方式”“free 空间生成方式”，还是在调“当前帧输出口径”。

## 0. 合并后的核心结论

`LDOP` 现在和官方 `DUFOMap` 的核心一致点在于：都用 `UFOMap` 做逐帧增量建图，用 `seenFree(point)` 作为动态残影清理依据。区别不在“有没有用 DUFOMap 思路”，而在处理模式不同：官方参考实现更偏离线，`LDOP` 当前实现更偏在线。

可以把当前逻辑只记成两条流：

### 0.1 地图更新流

当前帧点云每帧都送进 `UFOMap`，持续更新占据、free、seen-free 等状态。  
这部分是主链路，也是 `seenFree()` 判断的基础。

### 0.2 当前帧分类与可视化流

当前输入帧会先用更新前的 `UFOMap` 状态按 `seenFree(point)` 拆成静态和动态点云；
随后再把当前帧积分进地图。静态地图可视化同样从更新前快照导出，允许比地图更新迟一帧，不再额外维护历史点缓存。

### 0.3 和官方实现的主要区别

官方 `dufomap.cpp` 更像离线清图工具：先把整段序列全部积分进地图，再统一遍历累计点云，用 `!seenFree(p)` 保留静态点。  
`LDOP` 当前更像在线分割系统：每帧先用已有地图对当前帧做分类，再把当前帧写入 `UFOMap`，不再保留额外的累计点云旁路。

### 0.4 几个最容易混淆的点

- `seenFree(point)` 的含义：这个位置后来已经被观测为空闲，所以旧点更像动态残影。
- `propagate` 的作用：控制本帧地图修改后，状态是否立刻向八叉树上层传播。在线流程通常更适合 `true`。
- `min_range`：当前项目里主要用于和上游 SLAM 的近点过滤口径保持一致，继续传给 `UFOMap` 即可。
- `max_range`：适合同时约束 `UFOMap` 的工作半径和 `LDOP` 当前帧输出的判别半径。
- 官方 `cluster()`：只是对动态候选体素做辅助连通域聚类，不等于完整的检测、跟踪或预测模块，也不是点级动静态判别的核心。

### 0.5 一句话版本

官方 `DUFOMap` 参考程序是“离线累计后统一清图”，当前 `LDOP` 是“在线逐帧分割”；两者共享的真正核心，都是 `UFOMap` 增量更新加 `seenFree` 判别。

## 1. 先看官方 DUFOMap 的主流程

官方 `DUFOMap` 当前主程序在 `src/dufomap.cpp`，配置文件是 `assets/config.toml`。

可以把它简化成下面 6 步：

### 1.1 读取配置并创建地图

- 读取 `important.resolution`
- 读取 `map.levels`
- 用这两个参数构造 `ufo::Map`

对应含义：

- `resolution` 决定叶子体素尺寸
- `levels` 决定八叉树层级规模

### 1.2 逐帧读入点云和视点

每帧会读出：

- 当前帧点云 `cloud`
- 传感器视点 `viewpoint.translation`

这里的 `viewpoint.translation` 就是后面射线积分的起点。

### 1.3 先做近点过滤

官方会先用 `min_range` 把传感器附近的点滤掉，避免自车点、原点附近脏点进入地图。

这一步发生在积分之前，所以会影响后面所有 hit / miss / seenFree 结果。

### 1.4 把当前帧点云积分进 UFOMap

核心调用就是：

```cpp
ufo::insertPointCloud(map, cloud, viewpoint.translation, config.integration, config.propagate);
```

这一句决定了：

- 哪些点被当作命中 `hit`
- 哪些射线区段被写成 free / miss
- 哪些位置后续会变成 `seenFree`

`important` 组和 `integration` 组的大多数参数都在这一步生效。

### 1.5 可选传播与聚类

- 如果 `propagate=false`，官方会在全部帧处理完后再统一 `map.propagateModified()`
- 如果启用 clustering，会对 `seenFree && hits>=1 && label==0` 的叶子节点做聚类

注意：

- 聚类不是点级动静态判别的核心
- 判别核心仍然是 `seenFree(point)`

### 1.6 用 `seenFree(point)` 过滤累计点云

官方最后遍历累计点云：

```cpp
if (!map.seenFree(p)) {
  cloud_static.push_back(p);
}
```

含义很直接：

- `seenFree(p) == true`：这个位置后来被观测为空闲，更像动态残影
- `seenFree(p) == false`：这个点没有被后续 free 观测“推翻”，更像静态

## 2. LDOP 当前实现和官方 DUFOMap 的差异

当前 `LDOP` 的 `UfomapMapper` 沿用了同一条核心思路，但工作模式已经从“离线累计后统一过滤”变成了“在线逐帧用已有地图分类，再更新地图”。

主流程可以简化成下面 5 步：

### 2.1 读取 ROS 点云并提取有限点

- 从 `PointCloud2` 里提取有限 `xyz`
- 非有限点直接丢弃

### 2.2 用当前 odom 位置作为 `sensor_origin`

- `odom.pose.pose.position` 被直接当作射线起点
- 输入点云 `xyz` 和 odom pose 必须已经在同一个世界系；这是 LDOP 的输入限制，LDOP 内部不做 TF 转换

### 2.3 当前帧按更新前地图立即分类

和官方“最后统一过滤累计点云”不同，当前 `LDOP` 在每帧处理时直接对当前帧做：

- `seenFree(point) == true` -> `/ldop/dynamic_cloud`
- `seenFree(point) == false` -> `/ldop/static_cloud`

这里查询的是当前帧插入前的地图状态，避免同一帧射线刚写出的 `seenFree` 反过来污染本帧分类。

### 2.4 当前帧积分进 UFOMap

分类与静态体素可视化快照完成后，再调用 `ufo::insertPointCloud(...)` 更新地图。因此大多数 `ufomap_*` 参数本质上仍然是 UFOMap 积分参数，只是它们的效果从下一帧分类开始体现得最直接。

### 2.5 当前实现不再维护额外历史点缓存

当前 `LDOP` 只保留 UFOMap 地图本身，以及基于当前输入帧生成的 `/ldop/static_cloud` 和 `/ldop/dynamic_cloud`。

所以当前 `LDOP` 里有三类参数：

1. 从 DUFOMap / UFOMap 直接继承的积分参数
2. 在线流程新增的“当前帧输出口径”参数
3. 工程层面的性能 / 内存参数

第三阶段轨迹预测还新增了一个只读地图查询参数：`prediction_map_query_depth`。它不是 DUFOMap 积分参数，也不会改变 UFOMap 中 hit / miss / seenFree 的写入结果；它只控制 `PredictionMapQuery` 读取静态地图时采用叶子还是更粗深度，用于在 P3 地图交互和后续 P4 planner-facing 查询之间复用同一套 coarse/fine 查询基础。

## 3. 官方 DUFOMap 参数和 LDOP 参数的对应关系

| 官方 DUFOMap | 当前 LDOP | 说明 |
| --- | --- | --- |
| `important.resolution` | `ufomap_resolution` | 叶子体素尺寸 |
| `map.levels` | `ufomap_depth_levels` | 八叉树层级规模 |
| `integration.min_range` | `ufomap_min_range` | 近点过滤 / 最小积分距离 |
| `integration.max_range` | `ufomap_max_range` | 最大积分距离；在 LDOP 里还影响当前帧分类输出半径 |
| `integration.hit_depth` | `ufomap_insert_hit_depth` | 命中更新深度 |
| `integration.miss_depth` | `ufomap_insert_miss_depth` | free / miss 更新深度 |
| `integration.ray_casting_depth` | `ufomap_ray_casting_depth` | 射线投射深度 |
| `important.inflate_unknown` | `ufomap_inflate_unknown` | 论文里的 `d_p` |
| `integration.inflate_unknown_compensation` | `ufomap_inflate_unknown_compensation` | 是否补偿未知区域膨胀 |
| `integration.ray_passthrough_hits` | `ufomap_ray_passthrough_hits` | 射线是否穿过命中点 |
| `important.inflate_hits_dist` | `ufomap_inflate_hits_dist` | 论文里的 `d_s` |
| `integration.down_sampling_method` | `ufomap_down_sampling_method` | 积分前降采样策略 |
| `integration.simple_ray_casting` | `ufomap_simple_ray_casting` | 简化射线投射开关 |
| `integration.simple_ray_casting_factor` | `ufomap_simple_ray_casting_factor` | 简化射线步长系数 |
| `integration.parallel` | `ufomap_parallel` | 是否并行积分 |
| `integration.num_threads` | `ufomap_num_threads` | 线程数 |
| `integration.only_valid` | `ufomap_insert_only_valid` | 是否只对有效射线段积分 |
| `integration.sliding_window_size` | `ufomap_sliding_window_size` | 最近 N 帧 hits / misses 窗口；`>1` 时合并多帧观测生成 `seenFree` |
| `integration.early_stop_distance` | 暂无 | 官方 / UFOMap 参数；当前 LDOP 未接线，且当前底层主积分路径未实际使用 |
| `integration.propagate` | `ufomap_propagate` | 每帧后是否立即传播修改 |

## 4. 按工作流程理解每个参数

下面按“数据流经过哪里，这个参数就在哪里讲”的顺序说明。

### 4.1 地图初始化阶段

这一阶段决定地图的基本空间表达能力。

#### `ufomap_resolution`

作用：

- 叶子体素边长，单位米
- 这是最核心的参数之一

调大时通常会：

- 地图更粗
- 小目标和细边缘更容易被吞并
- 对位姿抖动、测距噪声更宽容
- 内存和积分开销通常下降

调小时通常会：

- 地图更细
- 更容易保留结构细节
- 也更容易把轻微配准误差放大成 `seenFree` 误检

经验理解：

- 它本质上决定了“你允许系统把多大的几何差异视为同一个体素”

#### `ufomap_depth_levels`

作用：

- 八叉树层级规模
- 也约束 `hit_depth`、`miss_depth`、`ray_casting_depth` 的合法范围

对当前项目的直接意义：

- 它更像地图空间规模和层级能力的上限设置
- 一般不是第一优先级调参项

什么时候需要关心：

- 你想扩更大空间范围
- 或者希望更细地控制不同深度上的更新行为

### 4.2 输入预处理阶段

这一阶段决定“哪些点有资格进入积分”。

#### `ufomap_min_range`

作用：

- 过滤离 `sensor_origin` 太近的点
- 常用于剔除自车点、原点附近噪声、某些 SLAM 算法夹带的 pose 点

调大时通常会：

- 更激进地去掉近点
- 近距离障碍物和机体附近结构也更容易丢失

调小时通常会：

- 保留更多近点
- 也更容易把自车残影、近场脏点写进地图

#### `ufomap_down_sampling_method`

作用：

- 点云进入积分前的降采样策略

可选值：

- `none`
- `center`
- `centroid`
- `uniform`

调参理解：

- `center` 更偏稳定、可解释，常用于调试
- `uniform` 更偏速度和分布均匀
- `none` 保留全部点，但积分负担最大

如果你在调试误检：

- 优先用 `center`
- 这样更容易把误差归因到射线和位姿，而不是采样随机性

### 4.3 射线积分阶段

这是最关键的阶段，因为 `seenFree` 的来源就在这里。

#### `ufomap_insert_hit_depth`

作用：

- 命中点 `hit` 写入的深度

直观理解：

- 控制“障碍物命中”被记录在哪一层体素上

常见情况：

- 保持 `0`，沿用默认叶层语义，通常够用

#### `ufomap_insert_miss_depth`

作用：

- free / miss 写入深度

直观理解：

- 控制“射线经过但没命中”的空间以哪一层体素被写成 free

调参影响：

- 这个参数和 `ray_casting_depth` 一起，决定 free 空间写入的尺度

#### `ufomap_ray_casting_depth`

作用：

- 射线投射时使用的深度

调参影响：

- 会影响 free carving 的空间尺度
- 进而影响后面哪些点会被 `seenFree(point)` 判成动态

什么时候调：

- 只有当你明确在做不同深度更新策略实验时再碰
- 否则一般保持 `0`

#### `ufomap_max_range`

作用：

- 限制积分时的最大射线长度
- `-1` 表示不限制；其他非负值需要大于 `ufomap_min_range`

在当前 `LDOP` 中还有一个额外作用：

- 当前帧分类前，还会用它再过滤一次输出点，只分类工作半径内的点

所以在 LDOP 里它同时决定两件事：

1. 多远的点会参与地图积分
2. 多远的点会参与当前帧动静态输出

调大时通常会：

- 纳入更远点
- 远距离噪声、稀疏点、配准误差更容易影响 `seenFree`

调小时通常会：

- 当前帧输出更保守
- 远处误检常常下降
- 但远处动态也更可能直接不参与判别

#### `ufomap_insert_only_valid`

作用：

- 是否只对有效射线区段做积分

直观理解：

- 它控制“超过有效范围或不满足条件的射线段”是否参与 free carving

调参影响：

- 对远距离噪声和范围边界附近的误判比较敏感

#### `ufomap_sliding_window_size`

作用：

- 控制 UFOMap 是否缓存最近 N 帧 `hits` / `misses`
- 当值 `>1` 时，底层会合并最近 N 帧观测后再计算 miss / `seenFree`
- 当值为 `0` 或 `1` 时，等价于关闭多帧窗口

对当前 `LDOP` 的意义：

- 当前帧分类公式仍然是 `seenFree(point)`，但 `seenFree` 的生成证据可以来自最近多帧
- 对点云稀疏、单帧 26 邻域观测不完整的情况，可能让 free / void 判定更稳定
- 对动态物体刚进入或刚离开的场景，窗口内旧证据会保留一段时间，因此可能带来短时滞后

调大时通常会：

- 降低单帧偶然缺点导致的分类抖动
- 增加内存和积分阶段合并开销
- 让动态残留或误判证据在窗口期内更不容易立刻消失

当前默认：

- `ufomap_sliding_window_size: 0`
- 这表示保持原来的单帧积分行为，不引入多帧窗口

调试建议：

- 如果当前问题是“单帧观测不足、`seenFree` 不稳定”，可以从 `3` 开始做 A/B
- 如果当前问题是“动态残留消不掉”，不要一开始就调大它，否则可能让残留更慢消失

#### `early_stop_distance`

作用：

- 这是官方 / UFOMap 侧的积分参数，不是当前 LDOP 的 ROS 参数
- 设计语义是让射线在接近 hit 终点前提前停止，避免把命中点附近的一小段空间误写成 free

对当前 `LDOP` 的实际状态：

- 当前 `LDOP` 没有 `ufomap_early_stop_distance`
- 只把这个名字写进 `ldop.yaml` 不会被读取
- 当前使用的 UFOMap `dufomap` 分支里，主积分路径虽然有 `early_stop_distance` 字段，但没有在 proper ray casting 主调用里实际传下去

所以它现在更适合记成：

- “可以作为后续底层 UFOMap 实验方向”
- 不是当前 LDOP 可直接调的参数

### 4.4 DUFOMap 论文相关的两组关键容错参数

这组参数最接近论文里的 `d_p` 和 `d_s`，本质是在做“对误差的容忍建模”。

#### `ufomap_inflate_unknown`

作用：

- 论文里的 `d_p`
- 在未知区域邻域上做膨胀，容忍位姿误差或观测稀疏带来的边界不稳定

调大时通常会：

- 系统更不容易把轻微错位直接判成 free
- 误检动态可能下降
- 但真实动态和细结构也可能被“保守化”

调小时通常会：

- 系统更敏感
- 更容易出现边缘闪烁式动态误检

#### `ufomap_inflate_unknown_compensation`

作用：

- 是否对 `inflate_unknown` 带来的末端影响做补偿

直观理解：

- 是 `d_p` 的配套细化开关

建议：

- 如果你在按 DUFOMap 默认思路靠拢，通常和 `inflate_unknown` 配套开启

#### `ufomap_inflate_hits_dist`

作用：

- 论文里的 `d_s`
- 沿射线方向扩展命中区域，容忍测距噪声

调大时通常会：

- 更不容易因为一点点距离波动就被后续 free 观测推翻
- 静态结构更稳
- 但动态目标残影也可能更容易留在静态里

调小时通常会：

- 对真实动态更敏感
- 也更容易让静态边缘被误判成动态

### 4.5 射线是否“穿透命中点”

#### `ufomap_ray_passthrough_hits`

作用：

- 控制射线遇到命中点后，是否继续向后穿过命中位置

常见推荐：

- DUFOMap 默认更偏向 `false`

为什么：

- 不让射线轻易穿过命中点，通常更符合“命中点后面不可随便写 free”的保守原则
- 对减少静态结构背后的错误 free carving 往往更有利

### 4.6 射线投射算法选择

#### `ufomap_simple_ray_casting`

作用：

- `true` 使用简化射线投射
- `false` 使用更完整的 proper ray casting

调参理解：

- `true` 往往更快
- `false` 往往更稳、更适合调试和对齐官方默认行为

如果你正在追查误检：

- 这是非常值得优先做 A/B 的参数

#### `ufomap_simple_ray_casting_factor`

作用：

- 简化射线投射的步长系数

影响：

- 只在 `ufomap_simple_ray_casting=true` 时有意义
- 系数越大，步长越激进，速度更高，但近似误差风险也更大

调试建议：

- 如果要对齐官方默认思路，先回到 `simple_ray_casting=false`
- 如果必须用简化射线，再从 `1.0` 起步

### 4.7 并行与传播阶段

这组参数主要影响性能、时序和地图状态何时可见。

#### `ufomap_parallel`

作用：

- 是否并行执行积分

影响：

- 主要是性能
- 理论上不应改变最终语义，但调试时为了减少不必要变量，常先关掉做基线实验

#### `ufomap_num_threads`

作用：

- 并行积分的线程数
- `0` 表示把线程数选择交给 UFOMap 的默认策略

影响：

- 主要是性能调度
- 不直接改变判别逻辑

#### `ufomap_propagate`

作用：

- 每帧插入后是否立即传播修改

在官方 DUFOMap 中：

- 更偏离线；可以处理完整段数据后再统一传播

在当前 LDOP 中：

- 因为下一帧分类会查询上一帧插入后的 `seenFree(point)` 状态，所以通常更适合保持 `true`

这是一个非常关键的语义差异：

- 在 LDOP 里，它不是单纯性能开关
- 它还决定“后续帧分类时，地图状态是否已经更新到可查询状态”

### 4.8 当前帧输出与静态体素可视化阶段

当前实现里，当前帧分类结果会直接发布到 `/ldop/static_cloud` 和 `/ldop/dynamic_cloud`。
静态地图可视化通过 `/ldop/static_map_markers` 输出，使用的是当前帧积分前的地图快照，因此允许比最新地图状态迟一帧；这里不再额外维护历史点缓存窗口。

## 5. 哪些参数最值得优先调

如果目标是“让当前帧动静态分割先稳定下来”，优先级建议如下。

### 第一优先级：最影响 `seenFree` 误检的参数

- `ufomap_resolution`
- `ufomap_max_range`
- `ufomap_inflate_unknown`
- `ufomap_inflate_hits_dist`
- `ufomap_ray_passthrough_hits`
- `ufomap_simple_ray_casting`
- `ufomap_sliding_window_size`
- `ufomap_down_sampling_method`

这几项最直接影响：

- free 空间怎么被 carving
- 静态结构边缘会不会被后续 free 观测推翻
- 多帧观测是否会共同影响 `seenFree`
- 远距离噪声会不会进入判别链

### 第二优先级：输入口径与近场稳定性

- `ufomap_min_range`
- `ufomap_insert_only_valid`

这两项更像“哪些点和哪些射线有资格参与积分”。

### 第三优先级：一般不先动的参数

- `ufomap_depth_levels`
- `ufomap_insert_hit_depth`
- `ufomap_insert_miss_depth`
- `ufomap_ray_casting_depth`
- `ufomap_parallel`
- `ufomap_num_threads`

它们不是没用，而是通常不是第一批造成“效果很差”的主因。

## 6. 如果想先靠近官方 DUFOMap 的默认思路

可以把下面这组当作“官方思路基线”来理解：

```yaml
ufomap_resolution: 0.1
ufomap_inflate_unknown: 1
ufomap_inflate_unknown_compensation: true
ufomap_ray_passthrough_hits: false
ufomap_inflate_hits_dist: 0.2
ufomap_down_sampling_method: center
ufomap_min_range: 0.2
ufomap_max_range: -1  # 表示不限制
ufomap_insert_only_valid: false
ufomap_insert_hit_depth: 0
ufomap_insert_miss_depth: 0
ufomap_ray_casting_depth: 0
ufomap_simple_ray_casting: false
ufomap_simple_ray_casting_factor: 1.0
ufomap_sliding_window_size: 0
ufomap_parallel: true
ufomap_num_threads: 0
```

但要注意，当前 `LDOP` 和官方仍然有三个本质差异：

1. `LDOP` 是在线逐帧分类，官方更偏离线累计后统一过滤
2. `LDOP` 当前帧输出还会额外受 `ufomap_max_range` 影响
3. `LDOP` 的 odom 和点云必须已经在同一个世界系里，否则参数再像官方也会误检

所以“照抄官方默认参数”不一定直接得到最好结果，但这组参数很适合作为调试起点。

## 7. 对当前仓库最实用的调参建议

如果你现在的目标是先把 `/ldop/static_cloud` 和 `/ldop/dynamic_cloud` 稳住，建议按这个顺序做：

### 7.1 先确认不是输入问题

先确认：

- `/cloud_registered.header.frame_id` 是否就是上游统一后的世界系
- `/Odometry` 的位置是否就是点云射线起点
- 点云和 odom 是否已经对齐到同一世界系

如果这里有抖动，后面任何 `seenFree` 相关参数都会被动放大误差。

### 7.2 先从“官方思路基线”开始

优先回到更保守、更可解释的一组：

- `ufomap_down_sampling_method: center`
- `ufomap_simple_ray_casting: false`
- `ufomap_simple_ray_casting_factor: 1.0`
- `ufomap_inflate_unknown: 1`
- `ufomap_inflate_unknown_compensation: true`
- `ufomap_inflate_hits_dist: 0.2`
- `ufomap_ray_passthrough_hits: false`
- `ufomap_sliding_window_size: 0`

这里先把 `ufomap_sliding_window_size` 放在 `0`，是为了先得到最容易归因的单帧积分基线；如果后面确认主要问题是单帧观测不足，再单独把它调到 `3` 或更高做对比。

### 7.3 再单变量调 `resolution`、`max_range` 和窗口大小

这些参数最容易让结果发生肉眼可见变化：

- `resolution` 主要影响“对抖动的容忍度”和“结构细节”
- `max_range` 主要影响“远处点是否进入积分和当前帧输出”
- `ufomap_sliding_window_size` 主要影响“`seenFree` 是否允许合并最近多帧证据”

### 7.4 最后才碰深度和并行参数

等前面稳定后，再决定要不要调整：

- `ufomap_insert_hit_depth`
- `ufomap_insert_miss_depth`
- `ufomap_ray_casting_depth`
- `ufomap_parallel`
- `ufomap_num_threads`

## 8. 相关代码位置

当前仓库里和本文最相关的位置：

- `config/ldop.yaml`
- `include/ldop/ufomap_mapper.h`
- `src/ufomap_mapper.cpp`
- `include/ldop/prediction_interaction_context.h`
- `src/prediction_interaction_context.cpp`
- `docs/design/LDOP_design.md`
- `docs/design/第四阶段规划地图接口设计草案.md`

外部参考：

- DUFOMap 官方仓库：`https://github.com/KTH-RPL/dufomap`
- 官方配置：`https://github.com/KTH-RPL/dufomap/blob/main/assets/config.toml`
- 官方主流程：`https://github.com/KTH-RPL/dufomap/blob/main/src/dufomap.cpp`
- DUFOMap 论文：`https://mit-spark.github.io/Longterm-Perception-WS/2024_IROS/assets/proceedings/DUFOMap/paper.pdf`

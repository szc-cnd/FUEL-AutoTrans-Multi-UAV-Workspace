# 入口/门识别模块说明

2026-07-10：把前置“识别门/通道入口”步骤从调参记录整理成独立模块说明，明确它在比赛流程里的职责、输入输出、限制条件和调试口径。

## 模块定位

这个模块对应代码里的 `corridor_search_manager`，启动入口在：

`plan_manage/launch/exp/run_swarm_indoor1_fuel_exploration.launch`



1. 起飞后先在起飞区前方识别“能进入搜索区的通道入口”。
2. 在 RViz 上把入口标出来，并持续发布入口坐标，方便第二架无人机复用。
3. 给当前无人机发布一个穿过入口的 through-point，让无人机先进入窄道/搜索区。
4. 确认已经穿过入口后，再解锁 FUEL exploration，让 frontier 只在通道内部承担建图/补图角色。

## 为什么不用实时点云当主判据

当前环境下不能把“实时雷达点云看见什么”当成门识别的核心依据，主要原因是：

1. `ego_start` 交接后，即使终端已经打印：

   ```text
   Hover done. Planning unlocked.
   Control handed over to controller. Offboard manager STOP publishing.
   ```

   控制器仍可能继续给较大的上升油门，导致飞机实际高度偏高。

2. 飞机飞高后，雷达对下方低矮结构、门框底部、通道入口边界的观测会明显变差。

3. 当前仿真雷达本身有缺陷，点云存在断裂、漏扫、天花板/噪声点投影干扰等问题。

4. 比赛任务更关心“尽快进入搜索区找二维码/颜色/温度异常/终点二维码并降落”，不是完整三维学术建图。

因此这个模块的设计原则是：实时点云只作为建图来源之一，真正的门判断尽量看“累计后的占据地图结构”，也就是 RViz 里更接近人眼看到的全局/累计地图形状。

## 输入

模块主要使用这些输入：

1. `/Odometry`：FAST-LIO 输出的里程计，用于当前位置、高度和参考朝向。

2. `/sdf_map/occupancy_all`：FUEL/Diff 地图侧累计占据点云，是当前门识别的主输入。

3. `/sdf_map/unknown`：未知区域信息，当前主要用于辅助调试和后续扩展。

4. `/start_after_hover`：起飞悬停完成后的开始信号。未 ready 前不允许进入主动搜索。

5. RViz 目标触发话题：这里只作为“允许开始搜索”的触发，不代表让飞机飞到 RViz 点击点。

## 输出

模块发布这些关键输出：

1. `/corridor_search/entry_pose`

   这是“入口边界坐标”，给 RViz 标记和第二架无人机参考。它应该落在门/通道入口本身，而不是门后的飞行目标点。

2. `/corridor_search/pre_entry_goal`

   这是当前无人机要跟踪的前置目标，一般是门后的 through-point，用来让飞机穿过入口。

3. `/planning/pos_cmd`

   在前置阶段直接给控制链路发布位置命令，避免 FUEL 还没进入通道就开始刷起飞区 frontier。

4. `/corridor_search/exploration_trigger`

   当检测到已经穿过入口后，触发 `exploration_manager`，此时才进入 FUEL 探索/建图阶段。

5. `/corridor_search/debug_goal`

   RViz Marker 输出，包括 `door_center`、`door_through`、`door_frame`、`door_label`、`entry_reference` 等，用于确认到底识别了哪个入口。

6. `/corridor_search/accumulated_occ_map`

   模块内部过滤后的累计地图，用于调试门识别输入。看门识别问题时优先看这个话题，不要只看原始实时点云。

## 核心概念

### door_point

`door_point` 是入口边界点，也就是截图里圈出来的“起飞区和通道连接处”。它用于：

1. RViz 标记入口。
2. 发布 `/corridor_search/entry_pose`。
3. 给第二架无人机作为入口参考坐标。
4. 判断当前无人机是否已经跨过入口。

### goal_point / through-point

`goal_point` 是门后的穿越点，用于当前无人机控制。它通常会比 `door_point` 更靠通道内部一点，作用是让无人机真正穿过去，而不是停在门口。

这两个点不能混用：`entry_pose` 应该发 `door_point`，控制目标应该发 `goal_point`。

## 当前识别流程

1. 等待 hover ready 和 RViz 触发。

2. 订阅 `/sdf_map/occupancy_all`，在模块内部按体素累计稳定地图。

3. 对累计地图做过滤：

   - `map_accum_min_hits`：低命中体素剔除。
   - `map_accum_min_neighbors`：孤立噪声点剔除。
   - `map_accum_min_vertical_bins`：缺少竖向连续性的点剔除。
   - `map_accum_max_use_z`：过滤天花板/高处点，避免顶面投影成假墙或假门。

4. 把过滤后的累计地图投影到无人机前方局部二维栅格。

5. 在局部栅格里找入口候选，优先考虑这些比赛场景特征：

   - 连续墙体突然出现可通行缺口。
   - 单边连续墙体旁边出现大缺口，缺口后方继续有通道结构。
   - 两根竖直柱/墙边形成 0.4m 到 1.5m 左右的可通行门架。
   - 起飞区边界和上方窄道之间的连接口，即使另一侧点云不完整，也允许靠“单边连续墙 + 缺口 + 后方延伸结构”推断为入口。

6. 候选通过路径安全检查后，生成：

   - `door_point`：入口边界。
   - `goal_point`：入口后方 through-point。
   - `door_frame`：RViz 中的门框线。

7. 候选连续确认多次后锁定入口，进入 `APPROACH_ENTRY`。

8. 无人机跨过 `door_point` 后发布 `/corridor_search/exploration_trigger`，FUEL 才开始探索。

## 为什么截图里的位置应该被当成入口

截图中圈出的地方满足比赛取巧逻辑里的入口条件：

1. 它位于起飞区和上方搜索区的边界。

2. 下方大房间的墙/边界在这里中断，形成明显缺口。

3. 缺口上方有持续延伸的墙体/点云结构，说明后面不是纯空地，而是通道或搜索区域。

4. 入口宽度满足无人机可通过，不是普通噪声点之间的小缝。

5. 这个入口比起房间内部零散 frontier 更符合比赛目标：20s 内进入搜索区域，而不是在起飞区补图。

所以当前模块的正确行为应该是：先把这里标成 `door`/`entry_reference`，再发布门后的 through-point，使无人机穿过该入口，然后才交给 FUEL。

## 调试判断标准

看日志时优先确认这些字段：

1. 是否出现：

   ```text
   door detected center=(...)
   entry locked at (...)
   ```

2. `door detected center` 是否落在入口边界。

3. `through=(...)` 是否在入口后方，而不是墙内。

4. `entry_reference` marker 是否显示在入口边界，而不是 through-point。

5. `/corridor_search/accumulated_occ_map` 是否已经滤掉大部分天花板点和孤立噪声点。

6. 如果日志一直是 `best=0`，说明地图候选没通过几何/路径过滤，不应该继续调 FUEL frontier。

## 常用参数

这些参数在 `run_swarm_indoor1_fuel_exploration.launch` 的 `corridor_search_manager` 节点里：

1. `map_accum_voxel_res`：累计地图体素分辨率。

2. `map_accum_min_hits`：体素最少命中次数，过低容易受噪声影响，过高会漏掉稀疏门框。

3. `map_accum_min_neighbors`：邻域过滤强度，过高会删掉真实细门柱。

4. `map_accum_min_vertical_bins`：竖向连续性过滤，过高会删掉低矮/断裂墙体。

5. `map_accum_max_use_z`：找门输入的最大高度，用于滤除天花板点。

6. `door_width_min` / `door_width_max`：允许的入口宽度。

7. `door_pass_dist`：through-point 放在入口后方多远。

8. `door_path_clearance`：当前位置到 through-point 的中心走廊避障余量。

9. `portal_min_exit_support_rows`：门后方延伸结构要求，过大容易“明明有门但不确认”。

10. `door_confirm_cycles`：连续确认次数，过低会受噪声误锁，过高会迟迟不标门。

## 当前模块边界

这个模块只负责“进入搜索区前”的入口识别和穿越，不负责二维码/颜色/温度识别。后续任务接口应该在 FUEL 进入搜索区后接入：

1. 二维码识别。
2. 颜色识别。
3. 温度异常点识别。
4. 终点二维码识别。
5. 终点降落触发。

## 后续改进方向

如果当前启发式仍然识别不到截图里的入口，下一步应该继续沿着“累计地图几何”做，而不是回到实时点云打分：

1. 对 `/corridor_search/accumulated_occ_map` 做起飞区外轮廓提取。

2. 找起飞区边界上的大缺口，而不是只找两侧完整门框。

3. 缺口后方若存在连续墙体或长条占据结构，就直接提高为入口候选。

4. 给入口候选增加比赛先验：优先选择从起飞区通向 `search_region_min_y` 以上区域的缺口。

5. 入口锁定后强制保持标记，不因后续临时噪声或高度变化取消 RViz 标记。

# 当前帧绿色静态体素显示设计

## 目标

保留旧版 RViz 绿色体素方块的视觉效果，同时解决移动人员离开后静态显示仍残留的问题。

## 根因

旧的 `/ldop/static_map_markers` 是从持续累积的 UFOMap 遍历得到的 `CUBE_LIST`。UFOMap 需要保留历史观测用于 `seenFree`、动态检测和后续规划查询，因此其中曾经出现过的占据体素不会因为目标离开而自动消失。将该话题直接显示在 RViz 中，会把地图历史误认为当前帧障碍物。

## 方案

1. UFOMap 的积分、分类、查询和持久化状态保持不变。
2. 每帧分类完成后，收集本帧被判定为静态的点。
3. 使用 UFOMap 的叶子编码将这些点去重，并转换为当前帧的绿色 `CUBE_LIST`。
4. MarkerArray 每帧先发送 `DELETEALL`，再发送本帧体素；因此点数减少或目标离开时，RViz 中的旧方块会立即清除。
5. RViz 启用 `/ldop/static_map_markers`，关闭重复的 `/ldop/static_cloud` 静态点云；动态点云和动态目标 Marker 保持启用。

## 数据流

```text
当前帧点云
  ├─ 分类：static_cloud / dynamic_cloud
  ├─ 积分：持久化 UFOMap（检测与规划使用）
  └─ static_cloud ──> 当前帧体素 MarkerArray ──> RViz 绿色方块
```

## 验证方式

按用户要求不新增或运行自动化测试。修改后执行 LDOP 编译并检查远程运行时话题；用户通过启动脚本观察绿色静态体素、动态点云、检测框，以及人员离开后的清除效果。

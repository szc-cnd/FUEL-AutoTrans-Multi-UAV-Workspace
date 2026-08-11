# UAV0 比赛统一启动包

`uav0_detection_landing_stack.launch` 把 `match_ws/src` 中的 RealSense D435、
颜色标签、二维码、热成像融合、目标远程上报和精确降落包集中到一个入口。
默认启动 D435、三类检测和 `target_reporting`，不默认打开下视相机、精确降落
或独立 RViz。

## 仿真/检测显示

默认由统一入口启动 D435：

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch uav0_competition_bringup uav0_detection_landing_stack.launch \
  enable_realsense:=true
```

颜色标签和二维码共用这一套 D435，不能重复启动第二个 RealSense 节点。
如果 D435 已经由其他终端启动，才使用：

```bash
roslaunch uav0_competition_bringup uav0_detection_landing_stack.launch \
  enable_realsense:=false
```

规划器已经在自己的 RViz 中启动 `target_rviz_marker`，因此通常不要把
`enable_target_rviz` 设为 `true`。独立查看检测结果时才使用：

```bash
roslaunch uav0_competition_bringup uav0_detection_landing_stack.launch \
  enable_target_rviz:=true
```

## 精确降落

连接下视相机并确认标定文件后，显式打开：

```bash
roslaunch uav0_competition_bringup uav0_detection_landing_stack.launch \
  enable_down_camera:=true \
  enable_precision_landing:=true
```

该入口不会自动解锁、切换 `OFFBOARD` 或触发降落；仍需按现场安全流程发布
上升沿 `/need_to_land`。仿真或未连接下视相机时保持这两个开关为 `false`。

## 话题关系

三个检测包发布 `/UAV0/...` 候选和调试话题；`target_reporting` 完成坐标转换、
候选/确认跟踪、TCP 远程上报，并发布 `/UAV0/target_reporting/observation`。
规划器 RViz 的标记适配器再将观测转成 `/UAV0/target_reporting/markers`。
候选结果目前只用于上报和 RViz 显示，不参与规划决策。

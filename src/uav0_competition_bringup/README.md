# UAV0 比赛统一启动包

`uav0_detection_landing_stack.launch` 把 `match_ws/src` 中的 RealSense D435、
颜色标签、二维码、热成像融合、相机外参 TF、目标远程上报和精确降落包集中到一个入口。
默认启动 D435、三类检测和 `target_reporting`，不默认打开下视相机、精确降落
或独立 RViz。D435 原始点云默认同步发布，但 RViz 只显示检测目标附近的过滤点云。
RViz 同时显示由过滤点云计算出的检测目标三维线框包围盒。

## 仿真/检测显示

根据热成像相机是否接入，选择下面一种启动方式。方案 A 适用于当前未接热成像
相机的电脑；方案 B 适用于 D435 和热成像相机都已接入的电脑。

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
# 方案 A：启动 D435，关闭热成像
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_realsense:=true \
  enable_thermal:=false \
  realsense_enable_pointcloud:=true \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true \
  target_reporting_mission_id:=onboard_test_$(date +%Y%m%d)
```

```bash
# 方案 B：D435 和热成像都启动
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_realsense:=true \
  enable_thermal:=true \
  realsense_enable_pointcloud:=true \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true \
  target_reporting_mission_id:=onboard_test_$(date +%Y%m%d)
```

入口还会自动启动 `camera_body_tf.launch` 中的相机静态外参发布节点，读取：

```text
$HOME/handeye_calibration/body_camera_03.yaml
```

当前入口默认发布 `UAV0/body -> camera_link` 外参，与 FAST-LIO 的 `UAV0/body`
坐标系对齐。若实际 FAST-LIO 仍发布无前缀的 `body`，可追加
`camera_body_tf_parent_frame:=body`。

当前 FAST-LIO 已自行广播 `UAV0/camera_init -> UAV0/body`，因此统一入口默认设置
`enable_camera_body_odom_tf:=false`，不会再启动 `fastlio_odometry_tf` 重复发布同一
变换。只有更换为“不发布 TF、仅发布 Odometry”的定位源时，才设置
`enable_camera_body_odom_tf:=true`，并用 `camera_body_tf_odom_topic` 指定其里程计话题。

脚本不修改 `ROS_LOG_DIR`，因此和规划器一样使用 ROS 默认日志目录：

```text
~/.ros/log/<本次运行ID>/
```

最近一次运行也可以通过以下路径查看：

```text
~/.ros/log/latest/
```

颜色标签和二维码共用这一套 D435，不能重复启动第二个 RealSense 节点。
`/camera/depth/color/points` 是 D435 原始点云输入，
`/UAV0/target_reporting/detected_object_cloud` 是只保留检测目标附近点的 RViz 输出。
`/UAV0/target_reporting/detected_object_boxes` 是按目标类型着色的三维线框包围盒输出。
若需降低 D435 负载，
可传入 `realsense_enable_pointcloud:=false` 关闭点云输出。
过滤半径默认是目标中心周围 0.25 m。
如果 D435 已经由其他终端启动，才使用：

```bash
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_realsense:=false \
  enable_thermal:=false \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true
```

规划器已经在自己的 RViz 中启动 `target_rviz_marker`，因此通常不要把
`enable_target_rviz` 设为 `true`。独立查看检测结果时才使用：

```bash
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_thermal:=false \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true \
  enable_target_rviz:=true
```

## UAV0 精确降落

比赛六分屏脚本的第 5 屏会自动查找 Generic USB 下视相机，并同时启动 UAV0
精确降落节点。手动使用统一入口时追加以下参数：

```bash
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_realsense:=true \
  enable_thermal:=false \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true \
  enable_down_camera:=true \
  enable_precision_landing:=true \
  landing_vehicle_ns:=UAV0
```

该入口不会自动解锁、切换 `OFFBOARD` 或触发降落；仍需按现场安全流程发布
上升沿 `/UAV0/need_to_land`。下视相机、状态、调试图和 MAVROS 接口也分别使用
`/UAV0/down_camera/...`、`/UAV0/landing/...` 与 `/UAV0/mavros/...`。

## 话题关系

三个检测包发布 `/UAV0/...` 候选和调试话题；`target_reporting` 完成坐标转换、
候选/确认跟踪、TCP 远程上报，并发布 `/UAV0/target_reporting/observation`。
规划器 RViz 的标记适配器再将观测转成 `/UAV0/target_reporting/markers`。
候选结果目前只用于机载端 RViz 显示，不通过 TCP 发送到 Windows；Windows 端只接收确认目标
及其最终证据图片。确认图片由 `target_reporting` 根据检测几何统一绘制二维码四角、颜色标签
或热源矩形框；候选和确认结果都不参与规划决策。
任务编号默认按当天生成，也可以通过 `target_reporting_mission_id` 显式指定。

D435 原始点云 `/camera/depth/color/points` 只作为显示过滤器输入；RViz 实际显示
`/UAV0/target_reporting/detected_object_cloud`，只包含当前候选/确认目标附近的点，
不参与检测判定、避障或规划决策。若 D435 已在其他终端启动，需确保原始点云话题已发布。
包围盒话题为 `/UAV0/target_reporting/detected_object_boxes`，同样只用于 RViz 显示。

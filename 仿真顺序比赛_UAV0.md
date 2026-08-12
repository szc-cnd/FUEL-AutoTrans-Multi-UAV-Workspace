# UAV0 比赛启动流程（精简版）

本流程对应 `~/match_ws` 的 UAV0 仿真/实机验证。启动顺序为：MAVROS、MID360、FAST-LIO、位姿回传、UAV0 统一检测与上报、FUEL 规划器和原控制器。五个功能包以及统一启动包都位于 `match_ws/src`，不再依赖 `~/db_ws`。检测结果只进入远程上报和本流程的 RViz 显示，不改变规划器逻辑。

传感器和节点不会自动解锁，也不会自动切换 `OFFBOARD`。确认数据正常后，再按现场飞行流程操作。

## 前六步一键启动（Terminator 六分屏）

如果希望把第 1～6 步集中到一个 Terminator 窗口中，可只执行下面的入口；不需要
再手动重复执行第 1～6 节中的命令：

```bash
cd ~/match_ws
bash shfiles/start_uav0_first_six_terminator.sh
```

窗口布局为两行三列：

```text
上排：1 MAVROS       | 2 MID360       | 3 FAST-LIO
下排：4 视觉位姿回传 | 5 检测/TF/上报 | 6 FUEL 规划器/RViz
```

六个分屏会同时打开，但每个分屏会等待自己的前置话题：MID360 等待 ROS master，
FAST-LIO 等待 `/UAV0/livox/lidar` 和 `/UAV0/livox/imu`，其余分屏等待
`/UAV0/fast_lio/Odometry`；因此不需要人工按照时间估计启动间隔。每个分屏会把
启动失败或等待超时直接显示在自己的终端中，并在命令退出后保持窗口打开。

默认使用方案 A（启动 D435、关闭热成像）。已接入热成像相机时使用：

```bash
bash shfiles/start_uav0_first_six_terminator.sh --thermal
```

如果 FAST-LIO 实际发布的是无前缀话题 `/Odometry`，使用：

```bash
bash shfiles/start_uav0_first_six_terminator.sh --odom-topic /Odometry
```

停止时可关闭本入口打开的 Terminator 窗口（不会替 ROS 节点执行强制停止）：

```bash
bash shfiles/start_uav0_first_six_terminator.sh stop
```

该入口包含第 5 步的 `enable_target_reporting:=true`，因此机载端上报客户端会一同
启动；Windows 接收服务器仍需在远程端单独启动。第 7 步控制器不在六分屏内，也不会
自动解锁、切换 `OFFBOARD` 或起飞。各 ROS 节点仍写入默认的 `~/.ros/log/`，六分屏
自身的启动错误记录在 `/tmp/uav0_first_six_terminator_<用户ID>.log`。

## 1. 启动 UAV0 MAVROS

终端 1：

```bash
cd ~/match_ws
sh shfiles/run.sh
```

终端 2 检查连接：

```bash
rostopic echo /UAV0/mavros/state
```

确认：

```text
connected: True
```

## 2. 启动 UAV0 MID360

```bash
cd ~/match_ws
source devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch \
  vehicle_ns:=UAV0 \
  msg_frame_id:=UAV0/livox_frame \
  publish_freq:=30.0
```

主要话题：

```text
/UAV0/livox/lidar
/UAV0/livox/imu
```

## 3. 启动 UAV0 FAST-LIO

```bash
cd ~/match_ws
source devel/setup.bash
roslaunch fast_lio mapping_mid360.launch \
  vehicle_ns:=UAV0 \
  rviz:=false
```

FAST-LIO 会根据 `vehicle_ns:=UAV0` 自动生成输入话题、输出话题和坐标系。RViz 不在 FAST-LIO 终端启动，而由规划器终端启动。

## 4. 回传 UAV0 视觉位姿

```bash
cd ~/match_ws/src/cxr_ego_ctrl/src
source ~/match_ws/devel/setup.bash
python3 laser_mid360.py iris 0 fastlio off
```

输入：`/UAV0/fast_lio/Odometry`。

输出：`/UAV0/mavros/vision_pose/pose`。

## 5. 启动 UAV0 检测和上报统一入口

统一入口是 `uav0_competition_bringup`，源码在：

```text
~/match_ws/src/uav0_competition_bringup
```

根据热成像相机是否接入，选择以下一种统一启动方式。两种方式都会启动 D435、
D435 原始点云、颜色标签、二维码、相机外参 TF 和目标上报；方案 B 另外启动热成像检测。
相机外参 TF 使用 `/home/oem/handeye_calibration/body_camera_03.yaml`，不需要
再单独执行 `camera_body_tf.launch` 或 `target_reporting.launch`。
统一入口默认将该外参发布为 `UAV0/body -> camera_link`，与当前 FAST-LIO 的
`UAV0/body` 坐标系对齐；若实际 FAST-LIO 使用无前缀 `body`，可追加
`camera_body_tf_parent_frame:=body`。
当前 FAST-LIO 已自行发布 `UAV0/camera_init -> UAV0/body`，统一入口默认关闭
`fastlio_odometry_tf`，避免两个节点重复广播同一条 TF。

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
# 方案 A：启动 D435，关闭热成像（当前电脑使用此方案）
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_realsense:=true \
  enable_thermal:=false \
  realsense_enable_pointcloud:=true \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true \
  target_reporting_mission_id:=onboard_test_$(date +%Y%m%d)
```

```bash
# 方案 B：D435 和热成像都启动（已接入热成像相机时使用）
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_realsense:=true \
  enable_thermal:=true \
  realsense_enable_pointcloud:=true \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true \
  target_reporting_mission_id:=onboard_test_$(date +%Y%m%d)
```

只有以后更换为“不发布 TF、仅发布 Odometry”的定位源时，才在命令中追加
`enable_camera_body_odom_tf:=true` 和对应的
`camera_body_tf_odom_topic:=<里程计话题>`；当前 FAST-LIO 不要打开该开关。

该脚本不修改 `ROS_LOG_DIR`，因此和规划器一样使用 ROS 默认日志目录：

```text
~/.ros/log/<本次运行ID>/
```

最近一次运行也可以通过 `~/.ros/log/latest/` 查看。未接热成像相机时使用
方案 A，避免热成像设备打开失败；接入并确认设备正常后再使用方案 B。

过滤点云默认保留目标中心周围 0.25 m 内的点；只需调试显示范围时，可在规划器
启动文件的 `target_rviz_marker` 节点中调整 `object_cloud_radius`。

D435 原始点云话题为 `/camera/depth/color/points`，RViz 实际只显示过滤后的
`/UAV0/target_reporting/detected_object_cloud`。如需降低相机负载，可在统一启动
命令中将 `realsense_enable_pointcloud:=true` 改为 `false`。
检测目标还会以三维线框包围盒显示在 `/UAV0/target_reporting/detected_object_boxes`；
包围盒只用于 RViz 显示，不参与避障和规划。

颜色标签和二维码共用统一入口启动的这一套 D435，不要再单独启动第二个
RealSense 节点。如果 D435 已经在其他终端运行，才改用：

```bash
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_realsense:=false \
  enable_thermal:=false \
  enable_camera_body_tf:=true \
  enable_target_reporting:=true
```

入口默认启动：

- `color_tag_detector`：颜色标签检测；
- `qr_detector`：普通二维码检测；
- `uvc_ubuntu`：热成像检测和 D435 深度融合；
- D435 PointCloud2：原始输入 `/camera/depth/color/points`，过滤输出
  `/UAV0/target_reporting/detected_object_cloud`，只显示检测目标附近点云；
- 检测目标三维线框包围盒：`/UAV0/target_reporting/detected_object_boxes`，颜色标签、二维码、热源分别使用橙色、绿色、品红色；
- `camera_body_tf`：默认只发布 `UAV0/body -> camera_link` 相机静态外参，
  不重复发布 FAST-LIO 已提供的世界到机体 TF；
- `target_reporting`：候选显示、检测器稳定门控、坐标转换、空间去重和远程 TCP 上报；
  上报层不再额外累计 3 帧，只有检测器自身 `confirmable=true` 才登记确认目标。

`enable_target_reporting:=true` 是远程数据传输开关，必须保持为 `true`；Windows
接收服务器仍需按下面的远程端步骤单独启动。

没有热成像硬件的仿真环境可加 `enable_thermal:=false`。同一套 D435 或 UVC
设备不要在其他终端重复启动。

检查检测观测和 RViz 标记话题：

```bash
rostopic echo /UAV0/target_reporting/observation
rostopic echo /UAV0/target_reporting/markers
rostopic hz /UAV0/target_reporting/detected_object_cloud
rostopic echo /UAV0/target_reporting/detected_object_boxes
rostopic hz /UAV0/color_tag_detector/debug_image
rostopic hz /UAV0/vision/qr_debug_image
rostopic hz /UAV0/thermal/debug_image
```

### 当前实际的数据传输流程（match_ws 机载端）

下面是当前使用的两端启动方式。Windows 接收端仍使用现有目录，机载端统一切换
到 `match_ws`。一键入口会同时启动相机外参 TF、检测节点和 `target_reporting`，
不要再单独启动 `camera_body_tf.launch` 或第二个 `target_reporter` 节点。

#### 远程 Windows 端

Windows 端不需要 ROS。在 PowerShell 中执行：

```powershell
cd "C:\Users\Jayus\Documents\飞行器比赛"
$env:PYTHONPATH = (Resolve-Path ".\target_reporting\src").Path
$missionId = "onboard_test_{0:yyyyMMdd}" -f (Get-Date)

python .\target_reporting\scripts\target_report_server.py `
  --host 0.0.0.0 `
  --port 5000 `
  --image-port 5001 `
  --output .\received_target_reports `
  --mission-id $missionId
```

接收文件会保存到：

```text
C:\Users\Jayus\Documents\飞行器比赛\received_target_reports\$missionId\
```

#### 机载端（192.168.31.163）

先按前面的步骤启动 FAST-LIO 和位姿回传，然后回到第 5 节选择方案 A 或方案 B，
执行对应的一键命令一次即可。该入口会同时启动相机外参 TF、检测和目标上报，
使用标定文件：

```text
/home/oem/handeye_calibration/body_camera_03.yaml
```

不需要再单独启动 `camera_body_tf.launch` 或 `target_reporting.launch`。机载端
配置文件为 `/home/oem/match_ws/src/target_reporting/config/target_reporting.yaml`。

配置文件应保持以下关键参数：

```yaml
remote_host: "192.168.31.147"  # Windows 远程端 IP
remote_port: 5000
image_port: 5001
mission_id: "auto"  # 自动使用当天的 onboard_test_YYYYMMDD
```

`192.168.31.163` 是机载端地址，不能填到 `remote_host`；Windows 接收服务器
未启动时，检测、规划和 RViz 仍可运行，上报客户端会自动重试。候选结果只在机载端
用于 RViz，不发送到 Windows；Windows 只接收检测器确认目标和带框证据图片。

三个检测器的状态字段统一为 `candidate`、`stable`、`confirmable`、
`stable_count`、`stable_window` 和 `reason`。原始候选可以立即在 RViz 中看到，
但不会因此远程上报；颜色标签需在 12 帧窗口内至少 8 次匹配，二维码需通过
真实性/深度校验并连续 5 帧稳定，热源需在 3 帧窗口内至少 2 次且像素跳变合格。

### 精确降落（当前暂不纳入流程）

当前比赛流程不启动下视相机和精确降落，`enable_down_camera` 与
`enable_precision_landing` 均保持默认值 `false`。以后确认下视相机设备路径和
标定文件有效后，再在统一入口中追加以下参数：

```bash
rosrun uav0_competition_bringup start_uav0_detection_landing_stack.sh \
  enable_realsense:=true \
  enable_thermal:=false \
  enable_camera_body_tf:=true \
  enable_down_camera:=true \
  enable_precision_landing:=true
```

该配置会启动下视相机和降落控制节点，但不会自动解锁、切换 `OFFBOARD` 或发布
降落触发；仍需按现场安全流程发布上升沿 `/need_to_land`。不要在统一入口已经
运行后再次单独启动同名的 `precision_landing` 节点。

## 6. 启动 UAV0 规划器

终端 6：

```bash
cd ~/match_ws
source devel/setup.bash
roslaunch "$(rospack find diff_planner)/launch/exp/run_swarm_indoor1_fuel_exploration.launch"
```

该入口启动 UAV0 的 FUEL 规划器和 RViz。RViz 的 `Detection Results` 分组会显示
`/UAV0/target_reporting/markers`，候选使用半透明、确认使用不透明，但颜色按目标类型保持一致：
颜色标签橙色、二维码绿色、热源品红色；颜色、二维码和热成像
调试图像以及只包含检测目标附近点的
`/UAV0/target_reporting/detected_object_cloud` 也已配置在同一个 RViz 中。
三维线框包围盒话题 `/UAV0/target_reporting/detected_object_boxes` 也已配置在
`Detection Results` 分组中；包围盒由目标附近点云估计，只用于可视化。
RViz 不单独显示 D435 深度图像，深度数据仅作为点云过滤和三维包围盒估计的输入。
这里启动的 `target_rviz_marker` 只负责显示，不会给 FUEL 发布检测目标或观察位姿。

如果只想运行规划而不启动检测显示，可加：

```bash
roslaunch "$(rospack find diff_planner)/launch/exp/run_swarm_indoor1_fuel_exploration.launch" \
  enable_detection_rviz:=false
```

## 7. 启动 UAV0 控制器

终端 7：

```bash
cd ~/match_ws/src/control/src
source ~/match_ws/devel/setup.bash
rosrun exploration_control cxr_egoctrl_v1 \
  __name:=UAV0_controller \
  _vehicle_ns:=/UAV0 \
  _planner_enable_height:=0.5
```

规划器和控制器分开启动，规划器输出的位置指令由 UAV0 原控制器接收。

## 8. 启动后检查

```bash
rostopic echo /UAV0/mavros/state
rostopic echo /UAV0/fast_lio/Odometry
rostopic echo /UAV0/mavros/vision_pose/pose
rostopic echo /UAV0/planning/pos_cmd
rostopic echo /UAV0/target_reporting/markers
```

确认上述话题持续有数据后，再进入飞行模式操作。`dual_ego_start.py` 是双机同步起飞工具，不属于 UAV0 单机启动的必要步骤。

## 9. 停止顺序

1. 退出 UAV0 控制器和规划器。
2. 停止统一入口（包含三个检测、`target_reporting`，以及按开关启动的降落节点）。
3. 确认无人机已退出自动控制、落地并上锁。
4. 停止位姿回传、FAST-LIO 和 MID360。
5. 最后停止 MAVROS。

不要在无人机仍处于解锁或 `OFFBOARD` 状态时直接关闭控制器或 MAVROS。

如果需要一次停止当前用户启动的全部 ROS1 节点、`roslaunch`、`roscore`、RViz 等
ROS 相关进程，可在确认无人机已经落地、退出 `OFFBOARD` 并上锁后执行：

```bash
cd ~/match_ws
bash shfiles/stop_all_ros.sh
```

该脚本先执行 `rosnode kill -a`，再清理残留启动进程，并自动对仍未退出的进程发送
`SIGKILL`；默认同时关闭本仓库六分屏入口打开的 Terminator 窗口，但不会执行降落或上锁。

执行前只查看候选进程、不发送信号：

```bash
bash shfiles/stop_all_ros.sh --dry-run
```

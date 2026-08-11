# UAV0 比赛启动流程（精简版）

本流程对应 `~/match_ws` 的 UAV0 仿真/实机验证。启动顺序为：MAVROS、MID360、FAST-LIO、位姿回传、UAV0 统一检测与上报、FUEL 规划器和原控制器。五个功能包以及统一启动包都位于 `match_ws/src`，不再依赖 `~/db_ws`。检测结果只进入远程上报和本流程的 RViz 显示，不改变规划器逻辑。

传感器和节点不会自动解锁，也不会自动切换 `OFFBOARD`。确认数据正常后，再按现场飞行流程操作。

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

## 5. 启动 UAV0 检测、上报和降落统一入口

统一入口是 `uav0_competition_bringup`，源码在：

```text
~/match_ws/src/uav0_competition_bringup
```

统一入口会同时启动 D435、三个检测节点和目标上报：

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch uav0_competition_bringup uav0_detection_landing_stack.launch \
  enable_realsense:=true
```

颜色标签和二维码共用统一入口启动的这一套 D435，不要再单独启动第二个
RealSense 节点。如果 D435 已经在其他终端运行，才改用：

```bash
roslaunch uav0_competition_bringup uav0_detection_landing_stack.launch \
  enable_realsense:=false
```

入口默认启动：

- `color_tag_detector`：颜色标签检测；
- `qr_detector`：普通二维码检测；
- `uvc_ubuntu`：热成像检测和 D435 深度融合；
- `target_reporting`：候选/确认跟踪、坐标转换、远程 TCP 上报。

没有热成像硬件的仿真环境可加 `enable_thermal:=false`。同一套 D435 或 UVC
设备不要在其他终端重复启动。

检查检测观测和 RViz 标记话题：

```bash
rostopic echo /UAV0/target_reporting/observation
rostopic echo /UAV0/target_reporting/markers
rostopic hz /UAV0/color_tag_detector/debug_image
rostopic hz /UAV0/vision/qr_debug_image
rostopic hz /UAV0/thermal/debug_image
```

### 远程 Windows 端

Windows 端不需要 ROS，只运行 `target_reporting` 中的 TCP 服务端。把
`match_ws/src/target_reporting` 复制到 Windows 后，在 PowerShell 中执行：

```powershell
cd C:\match_ws\target_reporting
$env:PYTHONPATH = "$PWD\src"
py -3 .\scripts\target_report_server.py `
  --host 0.0.0.0 --port 5000 --image-port 5001 `
  --output C:\target_reports --mission-id competition_current
```

机载端的 `src/target_reporting/config/target_reporting.yaml` 中将
`remote_host` 改成 Windows 的局域网 IP。Windows 服务端未启动时，检测、规划
和 RViz 仍可正常运行；上报客户端会重试，已确认记录会在网络恢复后补发。

### 精确降落（需要下视相机时）

精确降落节点默认关闭。仿真或未接下视相机时保持默认值；实机确认下视相机和
标定文件有效后，在启动统一入口时同时打开：

```bash
roslaunch uav0_competition_bringup uav0_detection_landing_stack.launch \
  enable_realsense:=true \
  enable_down_camera:=true \
  enable_precision_landing:=true
```

这只启动下视相机和降落控制节点，不会自动解锁、切换 `OFFBOARD` 或发布降落
触发；仍需按现场安全流程发布上升沿 `/need_to_land`。不要在统一入口已经运行
后再次单独启动同名的 `precision_landing` 节点。

## 6. 启动 UAV0 规划器

终端 6：

```bash
cd ~/match_ws
source devel/setup.bash
roslaunch "$(rospack find diff_planner)/launch/exp/run_swarm_indoor1_fuel_exploration.launch"
```

该入口启动 UAV0 的 FUEL 规划器和 RViz。RViz 的 `Detection Results` 分组会显示 `/UAV0/target_reporting/markers`，黄色为候选、绿色为确认；颜色、二维码和热成像调试图像也已配置在同一个 RViz 中。这里启动的 `target_rviz_marker` 只负责显示，不会给 FUEL 发布检测目标或观察位姿。

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

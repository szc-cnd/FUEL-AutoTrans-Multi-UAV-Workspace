# UAV0 比赛启动流程（精简版）

本流程对应 `~/match_ws` 的 UAV0 实机。保留原有启动顺序：MAVROS、MID360、FAST-LIO、位姿回传、检测上报、FUEL 规划器和原控制器分开启动。检测结果只进入远程上报和本流程的 RViz 显示，不改变规划器逻辑。

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

## 5. 启动 UAV0 检测结果上报

如果三个检测节点已经启动，在规划器前启动 `target_reporting`。它负责接收候选/确认结果、完成已有坐标转换和远程上报；规划器启动时会自动启动 RViz 显示适配节点。

```bash
cd ~/db_ws
source devel/setup.bash
roslaunch target_reporting target_reporting.launch
```

检查检测观测和 RViz 标记话题：

```bash
rostopic echo /UAV0/target_reporting/observation
rostopic echo /UAV0/target_reporting/markers
```

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
2. 停止 `target_reporting`。
3. 确认无人机已退出自动控制、落地并上锁。
4. 停止位姿回传、FAST-LIO 和 MID360。
5. 最后停止 MAVROS。

不要在无人机仍处于解锁或 `OFFBOARD` 状态时直接关闭控制器或 MAVROS。

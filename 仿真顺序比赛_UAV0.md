# UAV0 比赛启动流程（精简版）

本流程对应 `~/match_ws` 的 UAV0 实机。保留原有启动顺序：MAVROS、MID360、FAST-LIO、位姿回传、FUEL 规划器和原控制器分开启动。

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

## 5. 启动 UAV0 规划器

终端 5：

```bash
cd ~/match_ws
source devel/setup.bash
roslaunch "$(rospack find diff_planner)/launch/exp/run_swarm_indoor1_fuel_exploration.launch"
```

该入口启动 UAV0 的 FUEL 规划器和 RViz。不要再单独启动 `run_detector_fuel_uav0.launch`，避免重复启动规划相关节点。

## 6. 启动 UAV0 控制器

终端 6：

```bash
cd ~/match_ws/src/control/src
source ~/match_ws/devel/setup.bash
rosrun exploration_control cxr_egoctrl_v1 \
  __name:=UAV0_controller \
  _vehicle_ns:=/UAV0 \
  _planner_enable_height:=0.5
```

规划器和控制器分开启动，规划器输出的位置指令由 UAV0 原控制器接收。

## 7. 启动后检查

```bash
rostopic echo /UAV0/mavros/state
rostopic echo /UAV0/fast_lio/Odometry
rostopic echo /UAV0/mavros/vision_pose/pose
rostopic echo /UAV0/planning/pos_cmd
```

确认上述话题持续有数据后，再进入飞行模式操作。`dual_ego_start.py` 是双机同步起飞工具，不属于 UAV0 单机启动的必要步骤。

## 8. 停止顺序

1. 退出 UAV0 控制器和规划器。
2. 确认无人机已退出自动控制、落地并上锁。
3. 停止位姿回传、FAST-LIO 和 MID360。
4. 最后停止 MAVROS。

不要在无人机仍处于解锁或 `OFFBOARD` 状态时直接关闭控制器或 MAVROS。

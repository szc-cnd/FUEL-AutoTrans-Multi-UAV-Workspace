# UAV1 比赛启动流程（精简版）

本流程对应 `~/match_ws` 的 UAV1 实机。保留原有启动顺序：MAVROS、MID360、FAST-LIO、位姿回传、精确降落、Diff-Planner、轨迹桥接和 AutoTrans 控制器分开启动。

传感器、规划器和控制器不会自动解锁，也不会自动切换 `OFFBOARD`。确认数据链路正常后，再按现场飞行流程操作。

## 1. 启动 UAV1 MAVROS

```bash
cd ~/match_ws
sh shfiles/run_uav1_sensor_stack.sh mavros
```

该脚本启动 `/UAV1/mavros`，并请求 IMU、姿态、里程计和 ESC 状态频率。

检查：

```bash
rostopic echo /UAV1/mavros/state
```

确认：

```text
connected: True
```

## 2. 启动 UAV1 MID360

```bash
cd ~/match_ws
sh shfiles/run_uav1_sensor_stack.sh mid360
```

主要话题：

```text
/UAV1/livox/lidar
/UAV1/livox/imu
```

## 3. 启动 UAV1 FAST-LIO

```bash
cd ~/match_ws
sh shfiles/run_uav1_sensor_stack.sh fastlio
```

该命令不启动 RViz。主要输出：

```text
/UAV1/fast_lio/Odometry
/UAV1/fast_lio/cloud_registered
```

## 4. 回传 UAV1 视觉位姿

```bash
cd ~/match_ws/src/cxr_ego_ctrl/src
source ~/match_ws/devel/setup.bash
python3 laser_mid360.py iris 1 fastlio off
```

输入：`/UAV1/fast_lio/Odometry`。

输出：`/UAV1/mavros/vision_pose/pose`。

## 5. 启动 UAV1 下视相机和精确降落

新开终端执行：

```bash
cd ~/match_ws
sh shfiles/run_uav1_sensor_stack.sh landing
```

脚本自动查找 Generic USB 下视相机，启动 `/UAV1/down_camera` 和
`/UAV1/precision_landing_node`。它只监听 `/UAV1/need_to_land`，启动时不会自动
解锁、切换 `OFFBOARD` 或触发降落。状态和调试话题位于 `/UAV1/landing/...`，
飞控输入输出只连接 `/UAV1/mavros/...`，不会连接 UAV0。

检查：

```bash
rostopic hz /UAV1/down_camera/image_raw
rostopic echo /UAV1/landing/state
```

## 6. 单独启动 UAV1 Diff-Planner 和 RViz

终端 5：

```bash
cd ~/match_ws
source devel/setup.bash
roslaunch autotrans_reference_bridge uav1_diff_autotrans.launch \
  enable_planner:=true \
  enable_controller:=false \
  enable_rviz:=true
```

该终端只启动 UAV1 Diff-Planner 和 RViz，不启动 AutoTrans 控制器。RViz 使用原 UAV0 规划器的显示配置，目标点发布到：

```text
/UAV1/planning/goal
```

## 7. 单独启动 UAV1 桥接和 AutoTrans 控制器

终端 6：

```bash
cd ~/match_ws
source devel/setup.bash
roslaunch autotrans_reference_bridge uav1_diff_autotrans.launch \
  enable_planner:=false \
  enable_controller:=true \
  enable_rviz:=false
```

轨迹链路：

```text
/drone_1_planning/trajectory
    -> autotrans_reference_bridge
    -> /drone_1_planning/autotrans_trajectory
    -> AutoTrans MPC
    -> /UAV1/mavros/setpoint_raw/attitude
```

不要再同时运行旧的 `run_uav1_autotrans.sh`，避免重复启动桥接节点或控制器。

## 8. 启动后检查

```bash
rostopic echo /UAV1/mavros/state
rostopic echo /UAV1/fast_lio/Odometry
rostopic echo /UAV1/mavros/vision_pose/pose
rostopic echo /drone_1_planning/trajectory
rostopic echo /drone_1_planning/autotrans_trajectory
rostopic echo /UAV1/mavros/setpoint_raw/attitude
```

未确认 MAVROS、FAST-LIO、视觉位姿、轨迹桥接和 setpoint 均正常前，不进入 `OFFBOARD`。

`dual_ego_start.py` 是原双机同步起飞和控制交接工具，不属于 UAV1 AutoTrans 的必要启动步骤。

## 8. 停止顺序

1. 退出 AutoTrans 控制器、轨迹桥接和 Diff-Planner。
2. 确认无人机已退出自动控制、落地并上锁。
3. 停止位姿回传、FAST-LIO 和 MID360。
4. 最后停止 MAVROS。

不要在无人机仍处于解锁或 `OFFBOARD` 状态时直接关闭控制器或 MAVROS。




UAV1 六分屏一键启动脚本
cd ~/match_ws
bash shfiles/start_uav1_six_terminator.sh

rostopic echo /UAV1/fast_lio/Odom_high_freq 

## 4.5 启动 UAV1 LDOP

LDOP 订阅 UAV1 FAST-LIO 输出：`/UAV1/fast_lio/cloud_registered` 和
`/UAV1/fast_lio/Odom_high_freq`，发布到 `/UAV1/ldop/...`。FAST-LIO 启动并有数据后，
另开终端执行：

```bash
cd ~/match_ws
bash shfiles/run_uav1_ldop.sh
```

该 launch 会启动 `/UAV1/ldop_node` 和 `/UAV1/ldop_rviz`。它不会改变
`/UAV1/fast_lio/...`、MAVROS 或 Diff-Planner 的话题。

如果已经通过六分屏启动了 Diff-Planner/RViz，使用下面的命令避免再开一个 RViz：

```bash
cd ~/match_ws
bash shfiles/run_uav1_ldop.sh --no-rviz
```
bash shfiles/stop_all_ros.sh 

# UAV1 比赛启动流程（精简版）

本流程对应 `~/match_ws` 的 UAV1 实机。保留原有启动顺序：MAVROS、MID360、FAST-LIO、位姿回传、精确降落、Diff-Planner、轨迹桥接和 AutoTrans 控制器分开启动。

传感器、规划器和控制器不会自动解锁，也不会自动切换 `OFFBOARD`。确认数据链路正常后，再按现场飞行流程操作。

## 1. 启动 UAV1 MAVROS

cd ~/match_ws
sh shfiles/run_uav1_sensor_stack.sh mavros

rostopic echo /UAV1/mavros/state


## 2. 启动 UAV1 MID360


cd ~/match_ws
sh shfiles/run_uav1_sensor_stack.sh mid360




## 3. 启动 UAV1 FAST-LIO


cd ~/match_ws
sh shfiles/run_uav1_sensor_stack.sh fastlio

rostopic echo /UAV1/fast_lio/Odom_high_freq 


cd ~/match_ws
bash shfiles/run_uav1_ldop.sh
roslaunch ldop run_ldop.launch

## 4. 回传 UAV1 视觉位姿


cd ~/match_ws/src/cxr_ego_ctrl/src
source ~/match_ws/devel/setup.bash
python3 laser_mid360.py iris 1 fastlio off



cd ~/match_ws/src/cxr_ego_ctrl/src
source ~/match_ws/devel/setup.bash
rosrun cxr_ego_ctrl laser_mid360_high_freq.py


rostopic hz /UAV1/fast_lio/Odom_high_freq
rostopic hz /UAV1/mavros/vision_pose/pose

rostopic echo /UAV1/fast_lio/Odom_high_freq
rostopic echo /UAV1/mavros/vision_pose/pose
rostopic echo /UAV1/livox/imu_mps2




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


cd ~/match_ws
source devel/setup.bash
roslaunch autotrans_reference_bridge uav1_diff_autotrans.launch \
  enable_planner:=true \
  enable_controller:=false \
  enable_rviz:=true

测试：
cd ~/match_ws
source devel/setup.bash
roslaunch diff_planner run_swarm.launch


该终端只启动 UAV1 Diff-Planner 和 RViz，不启动 AutoTrans 控制器。RViz 使用原 UAV0 规划器的显示配置，目标点发布到：




## 7. 启动 UAV1 简单控制器

终端 6：

```bash
cd ~/match_ws
source devel/setup.bash
roslaunch exploration_control simple_controller.launch \
  vehicle_ns:=UAV1 \
  node_name:=UAV1_controller
```

简单控制器发布 `/UAV1/control/position_setpoint`，降落入口中的仲裁器唯一转发到
`/UAV1/mavros/setpoint_raw/local`。收到降落触发且精确降落通过预检查后，仲裁器
锁存降落控制权，简单控制器不再影响飞控。若以后改回 AutoTrans，现有 AutoTrans
入口也已改为向 `/UAV1/control/attitude_setpoint` 发布并经过同一仲裁器。


## 6. 单独启动 UAV1 桥接和 AutoTrans 控制器

终端 6：

cd ~/match_ws
source devel/setup.bash
roslaunch autotrans_reference_bridge uav1_diff_autotrans.launch \
  enable_planner:=false

## 8. 启动后检查

rostopic echo /UAV1/control/attitude_setpoint
rostopic echo /UAV1/mavros/state
rostopic echo /UAV1/fast_lio/Odom_
rostopic echo /UAV1/fast_lio/Odometry
rostopic echo /UAV1/mavros/vision_pose/pose
rostopic echo /drone_1_planning/trajectory
rostopic echo /drone_1_planning/autotrans_trajectory
rostopic echo /UAV1/control/position_setpoint
rostopic echo /UAV1/mavros/setpoint_raw/local
rostopic echo /UAV1/landing/control_owner




`dual_ego_start.py` 是原双机同步起飞和控制交接工具，不属于 UAV1 AutoTrans 的必要启动步骤。

# UAV0 搜索降落独立测试

本流程用于绕过 FUEL 和通道任务，单独测试：

```text
原地起飞到 0.60 m 并悬停
→ 等待人工“已出通道”信号
→ 前视 D435 原地扫描
→ 前视发现则飞向粗定位；否则升至世界系 z=2.00 m
→ Diff 执行 3 m × 4 m 下视蛇形搜索
→ 下视 ArUco 稳定确认
→ 飞到平台上方 2.00 m
→ 视觉精降
→ AUTO.LAND
```

独立入口不启动 FUEL、通道搜索或 LDOP。测试前应把无人机放在模拟“已经飞出通道约
0.6 m”的位置，并让机头朝向真实出通道方向；收到人工信号时，程序会把当时位置和
偏航角锁存为搜索起点与搜索方向。

## 安全规则

- 第一次必须拆桨运行默认隔离模式。
- 一键脚本和独立 launch 都不会自动解锁、切换 `OFFBOARD` 或发送出通道信号。
- 只有显式使用 `--flight` 或 `enable_flight_control:=true` 才会连接 MAVROS 控制输出。
- 出通道切换会锁存，无法通过发布 `false` 撤销。重新测试必须人工接管、落地上锁并
  重启独立 launch。
- 不要同时启动比赛七分屏入口或完整 FUEL 规划器，否则会出现同名节点和重复控制源。

## 一键启动

### 默认隔离测试，不起飞

```bash
cd ~/match_ws
bash shfiles/start_uav0_search_landing_test.sh
```

脚本会打开六个 Terminator 分屏：

```text
1 MAVROS | 2 MID360 | 3 FAST-LIO
4 位姿回传 | 5 搜索降落与 RViz | 6 状态与人工触发
```

每次启动都会把六个分屏的完整终端输出保存到独立目录：

```text
~/search_landing_logs/YYYYMMDD_HHMMSS/
```

DIFF 的规划、地图和崩溃前诊断位于 `search.log`，测试后可直接保留该目录用于复盘。

默认不启动简单控制器，并把位置设定点、姿态设定点和模式服务全部接到
`/UAV0/search_landing_test/blocked_*`，因此只能观察识别、搜索航点和 Diff 轨迹。

### 实飞测试

```bash
cd ~/match_ws
bash shfiles/start_uav0_search_landing_test.sh --flight
```

实飞模式启动简单控制器。它会先持续发布 `0.60 m` 原地起飞/悬停设定点，但仍需操作者
按照现场安全流程手动解锁并切换 `OFFBOARD`。

确认无人机已经在 `0.60 m` 稳定悬停后，另开终端发送一次“已出通道”信号：

```bash
source /opt/ros/noetic/setup.bash
source ~/match_ws/devel/setup.bash

rostopic pub -1 /UAV0/mission/task_status std_msgs/String \
  "data: 'SEARCH_OUTSIDE_LANDING'"
```

该信号会解开前视/下视 ArUco 阶段门控、激活搜索管理器，并把规划命令所有者锁存为
Diff。重新测试必须人工接管、落地上锁并重启独立测试节点。

前视原地扫描期间控制器继续保持当前位置，搜索管理器只发送偏航指令，不向 Diff
发送与当前位置重合的零长度目标。前视发现平台或扫描完成后，才向 Diff 下发第一个
有实际位移的目标。

需要临时修改悬停高度时使用：

```bash
bash shfiles/start_uav0_search_landing_test.sh --flight --hover-height 0.60
```

脚本只允许 `0.50～1.00 m` 的等待高度。关闭 RViz：

```bash
bash shfiles/start_uav0_search_landing_test.sh --flight --no-rviz
```

无人机落地、上锁并退出 `OFFBOARD` 后，可关闭本脚本打开的窗口：

```bash
bash shfiles/start_uav0_search_landing_test.sh stop
```

## 分别手动启动

下面是六个终端需要执行的启动代码。启动代码集中列出，查询指令统一放在后面。

### 终端 1：MAVROS

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

sh shfiles/run.sh
```

### 终端 2：MID360

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch livox_ros_driver2 msg_MID360.launch \
  vehicle_ns:=UAV0 \
  msg_frame_id:=UAV0/livox_frame \
  publish_freq:=30.0
```

### 终端 3：FAST-LIO

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch fast_lio mapping_mid360.launch \
  vehicle_ns:=UAV0 \
  odom_topic:=/UAV0/fast_lio/Odometry \
  rviz:=false
```

### 终端 4：FAST-LIO 位姿回传 PX4

```bash
cd ~/match_ws/src/cxr_ego_ctrl/src
source /opt/ros/noetic/setup.bash
source ~/match_ws/devel/setup.bash

python3 laser_mid360.py iris 0 fastlio off \
  _odom_topic:=/UAV0/fast_lio/Odometry
```

### 终端 5：搜索降落独立入口

第一次拆桨检查使用飞控隔离模式：

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch uav0_competition_bringup uav0_search_landing_test.launch \
  enable_flight_control:=false
```

无桨检查通过后，实飞使用：

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash

roslaunch uav0_competition_bringup uav0_search_landing_test.launch \
  enable_flight_control:=true \
  hover_height:=0.60
```

### 终端 6：人工触发和状态查询

这个终端先只加载环境，不立即触发：

```bash
source /opt/ros/noetic/setup.bash
source ~/match_ws/devel/setup.bash
```

## 启动后的查询指令

### 1. 检查飞控和定位

```bash
rostopic echo -n 1 /UAV0/mavros/state
rostopic hz /UAV0/fast_lio/Odometry
rostopic hz /UAV0/fast_lio/cloud_registered
rostopic hz /UAV0/mavros/vision_pose/pose
```

第一条确认 MAVROS 已连接；后三条分别确认 FAST-LIO 位姿、注册点云和 PX4 位姿回传。
持续查询频率的命令看到稳定输出后，按 `Ctrl+C` 再执行下一条。

### 2. 检查两台相机和两路调试图

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/aligned_depth_to_color/image_raw
rostopic hz /UAV0/down_camera/image_raw
rostopic hz /UAV0/landing/front/debug_image
rostopic hz /UAV0/landing/combined_debug_image
```

依次确认 D435 彩色图、D435 对齐深度图、下视原图、前视粗搜索图和统一下视调试图
正常发布。统一下视图在精降触发前显示搜索标注，触发后自动切换为精降标注。也可以
分别使用 `rqt_image_view <话题名>` 查看画面。

### 3. 检查启动后尚未触发

```bash
rostopic echo -n 1 /landing_diff_search_manager/state
rostopic echo -n 1 /planner_command_arbiter/owner
```

人工触发前应分别看到：

```text
WAIT_EXIT_SWITCH
FUEL
```

这里的 `FUEL` 只是仲裁器的初始名称；独立入口没有启动 FUEL，也不会产生 FUEL 控制
指令。收到人工信号后，它才锁存为 `DIFF`。

### 4. 实飞前检查 0.60 m 悬停控制输出

```bash
rostopic hz /UAV0/control/position_setpoint
rostopic echo /UAV0/fast_lio/Odometry
```

第一条应接近 `50 Hz`。手动解锁并切换 `OFFBOARD` 后，通过第二条确认世界系 `z` 已
稳定在约 `0.60 m`，同时确认 XY 没有持续漂移。

## 触发后的查询指令

```bash
rostopic echo /landing_diff_search_manager/state
rostopic echo /planner_command_arbiter/owner
rostopic echo /drone_0_planning/status
rostopic echo /UAV0/landing_diff/subgoal
rostopic echo /UAV0/landing/front/status
rostopic echo /UAV0/landing/front_aruco_hint
rostopic echo /UAV0/landing/search/status
rostopic echo /UAV0/mission/detection/final_aruco
rostopic echo /UAV0/mission/landing_request
rostopic echo /UAV0/landing/control_owner
```

这些话题依次用于查看搜索状态、规划所有者、Diff 规划结果、当前 Diff 航点、前视检测、
前视粗定位、下视稳定检测、最终平台世界坐标、降落请求和最终控制权。正常生成轨迹时
`/drone_0_planning/status` 应显示 `TRAJECTORY_PUBLISHED`；无效或过近目标会被安全拒绝，
不会再导致 DIFF 节点退出。

典型状态变化为：

```text
WAIT_EXIT_SWITCH
→ FRONT_ARUCO_INITIAL_WAIT
→ FRONT_ARUCO_YAW_SCAN_LEFT / RIGHT / RETURN
→ FRONT_ARUCO_HINT_DIFF_APPROACH（前视发现）
  或 FRONT_ARUCO_YAW_SCAN_COMPLETE_START_DOWN_SWEEP（前视未发现）
→ ARUCO_LOCKED_DIFF_APPROACH
→ DIFF_APPROACH_REACHED_LANDING_REQUESTED
→ 精降 ACQUIRE → ALIGN → DESCEND_HIGH
→ FIXED_XY_DESCENT → REQUEST_AUTO_LAND → DONE
```

其中检测高度到 `≤1.50 m` 后立即进入 `FIXED_XY_DESCENT`，不再等待水平误差达到 0.08 m 并稳定 0.5 秒；固定当前融合定位 X/Y 后以 `0.10 m/s` 下降，到估计离地高度 `≤0.30 m` 再请求 `AUTO.LAND`。

## RViz 中应看到的内容

- UAV0 机体模型和 FAST-LIO 历史路径。
- FAST-LIO 注册点云和 Diff 占据地图。
- Diff 当前规划轨迹、当前目标点和 `landing_search_subgoal`。
- 前视 ArUco 带标注图像，以及自动切换搜索/精降标注的统一下视图像。

搜索管理器每次只发送一个航点，因此 RViz 不会一次画出完整蛇形路线，而是随着当前
航点到达逐段更新。

## 异常处理与停止

出现定位跳变、轨迹穿墙、图像冻结、错误 ArUco 锁定或控制权异常时，立即通过遥控器
退出 `OFFBOARD`。不要先关闭唯一的设定点发布节点。

查看最近一次测试日志：

```bash
latest_search_log=$(find ~/search_landing_logs -mindepth 1 -maxdepth 1 \
  -type d | sort | tail -n 1)
echo "${latest_search_log}"
ls -lh "${latest_search_log}"
grep -RniE "error|warn|failed|abort|assert" "${latest_search_log}"
```

确认无人机已经人工接管、落地、上锁并退出 `OFFBOARD` 后，再按以下顺序停止：

1. 搜索降落独立入口。
2. 位姿回传。
3. FAST-LIO。
4. MID360。
5. MAVROS。

需要清理全部 ROS 进程时，必须在落地上锁后执行：

```bash
cd ~/match_ws
bash shfiles/stop_all_ros.sh
```

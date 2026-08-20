# 前视与下视 ArUco 无飞行观察

该模式只用于在地面分别观察前视 D435 和下视 USB 相机的 ArUco 检测画面。它不启动
MAVROS、Diff-Planner、控制仲裁器或精确降落控制器，不会解锁、切换 OFFBOARD 或起飞。

## 启动

建议先取下桨叶，然后执行：

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch uav0_competition_bringup uav0_dual_aruco_observer.launch
```

启动后自动打开两个窗口：

- 前视：`/UAV0/aruco_observer/front/debug_image`
- 下视：`/UAV0/aruco_observer/down/debug_image`

不需要发布 `SEARCH_OUTSIDE_LANDING`，观察入口已经仅对自身检测节点关闭任务阶段门控。
正式比赛入口的阶段门控没有改变。

## 里程计说明

没有启动 FAST-LIO 时，两个窗口仍能显示 ArUco 边框和 ID，但状态会提示等待同步里程计，
不会产生可靠的平台世界坐标。如果 FAST-LIO 已经运行，默认读取：

```text
/UAV0/fast_lio/Odometry
```

使用其他里程计话题时：

```bash
roslaunch uav0_competition_bringup uav0_dual_aruco_observer.launch \
  odometry_topic:=/实际里程计话题
```

## 手动打开窗口

如果启动时不希望自动打开窗口：

```bash
roslaunch uav0_competition_bringup uav0_dual_aruco_observer.launch \
  open_views:=false
```

再分别执行：

```bash
rqt_image_view /UAV0/aruco_observer/front/debug_image
rqt_image_view /UAV0/aruco_observer/down/debug_image
```

## 检查话题

```bash
rostopic hz /camera/color/image_raw
rostopic hz /camera/aligned_depth_to_color/image_raw
rostopic hz /UAV0/down_camera/image_raw
rostopic hz /UAV0/aruco_observer/front/debug_image
rostopic hz /UAV0/aruco_observer/down/debug_image
```

检测状态：

```bash
rostopic echo /UAV0/aruco_observer/front/status
rostopic echo /UAV0/aruco_observer/down/status
```

按 `Ctrl+C` 即可停止全部观察节点。

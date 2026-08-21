# UAV1 下视相机外参标定流程

本文档用于重新标定 UAV1 的 `UAV1/body <- UAV1/down_camera_optical_frame`
外参。标定输入为 UAV1 FAST-LIO 里程计和下视相机棋盘格观测，最终结果供搜索降落与
精确降落使用。

本流程只负责采集和验收外参，不会自动覆盖飞行配置。未经质量检查的 YAML 禁止写入
`landing_search.yaml` 或 `precision_landing.yaml`。

## 1. 标定原则

1. 拆除螺旋桨，飞控保持锁定，禁止启动电机。
2. 棋盘格和周围环境全程固定，只移动整架 UAV1。
3. 相机、IMU、Mid360 和机架必须作为一个刚性整体移动，不能单独转动相机。
4. 不连续采用每一帧；每个稳定停留姿态最终只保留一个代表点。
5. 标定必须同时包含 X、Y、Z 平移以及横滚、俯仰、航向变化。
6. 录包前等待 FAST-LIO 完全稳定，定位漂移时禁止开始标定。

使用的棋盘格规格固定为：

- 内角点：`11 × 8`
- 单个方格边长：`0.040 m`
- 棋盘格应平整固定，不能翘曲、反光或被遮挡

## 2. 场地布置

1. 用胶带将棋盘格四角固定在地面，标定期间不能移动棋盘格。
2. 选择具有墙角、桌腿等固定几何特征的室内环境，避免空旷、玻璃和大量运动人员。
3. 保持光照均匀，避免棋盘格上出现强反光和大面积阴影。
4. 操作人员站在 UAV 后方，不能遮挡 Mid360，也不能踩到棋盘格。
5. 推荐保持相机到棋盘格的距离为 `0.8–1.2 m`。
6. 画面内必须看到完整棋盘格，棋盘格四周最好保留至少 `20–30 px` 余量。

## 3. 前台启动节点

标定需要 UAV0 提供 ROS Master，然后在 UAV1 分别启动 Mid360、FAST-LIO 和下视相机。
所有进程都在前台运行，需要停止时在对应终端按 `Ctrl+C`。

### 3.1 UAV0：启动 ROS Master

标定时推荐只启动 ROS Master，不需要启动 UAV0 的规划、控制和检测节点。在 UAV0
终端执行：

```bash
cd /home/asus/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source shfiles/configure_dual_uav_ros_network.sh
configure_dual_uav_ros_network uav0
roscore
```

如果 UAV0 的 ROS Master 已经运行，则不需要再次执行 `roscore`，也不能启动第二个
ROS Master。本流程不需要启动 UAV0 的飞行、规划、检测或控制节点。

在 UAV0 确认 Master 正常：

```bash
echo "$ROS_MASTER_URI"
rosnode list
```

`ROS_MASTER_URI` 应为 `http://192.168.31.21:11311`。

### 3.2 UAV1 终端 1：启动 Mid360

```bash
cd /home/oem/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source shfiles/configure_dual_uav_ros_network.sh
configure_dual_uav_ros_network uav1
sh shfiles/run_uav1_sensor_stack.sh mid360
```

### 3.3 UAV1 终端 2：启动 FAST-LIO

```bash
cd /home/oem/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source shfiles/configure_dual_uav_ros_network.sh
configure_dual_uav_ros_network uav1
sh shfiles/run_uav1_sensor_stack.sh fastlio
```

### 3.4 UAV1 终端 3：启动下视相机

```bash
cd /home/oem/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source shfiles/configure_dual_uav_ros_network.sh
configure_dual_uav_ros_network uav1
sh shfiles/run_uav1_sensor_stack.sh landing
```

该命令在本标定流程中只用于提供 UAV1 下视相机图像和内参。不要启动搜索、精降或电机
控制测试。

### 3.5 UAV1 终端 4：将标定图像限制为 10 Hz

```bash
cd /home/oem/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source shfiles/configure_dual_uav_ros_network.sh
configure_dual_uav_ros_network uav1

rosrun topic_tools throttle messages \
  /UAV1/down_camera/image_raw 10.0 \
  /UAV1/calibration/down_camera/image_raw
```

### 3.6 UAV1 终端 5：查看下视画面

```bash
cd /home/oem/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source shfiles/configure_dual_uav_ros_network.sh
configure_dual_uav_ros_network uav1

rqt_image_view /UAV1/calibration/down_camera/image_raw
```

检查画面时必须满足：

- 完整棋盘格始终位于画面内；
- 内角点清晰，没有明显运动模糊；
- 棋盘格不贴住图像边缘；
- 相机支架和镜头没有松动。

## 4. 等待并检查 FAST-LIO

节点启动后至少等待 `2–3 min`，不要立即录包。执行：

```bash
rostopic hz /UAV1/fast_lio/Odometry
rostopic hz /UAV1/calibration/down_camera/image_raw
```

让 UAV1 静止约 `10 s`，间隔数秒执行两次：

```bash
rostopic echo -n1 /UAV1/fast_lio/Odometry/pose/pose/position
```

两次位置不应持续漂移。随后将整架 UAV 缓慢移动一小段再放回原位，确认 FAST-LIO 位姿
能大致回到起点。出现以下情况之一时禁止录包：

- 位置持续漂移或突然跳变；
- 静止时姿态不断旋转；
- 点云定位丢失或里程计频率异常；
- 操作人员的手或身体大面积遮挡 Mid360。

## 5. 开始录包

确认图像和 FAST-LIO 都稳定后，在第 6 个终端执行：

```bash
cd /home/oem/match_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
source shfiles/configure_dual_uav_ros_network.sh
configure_dual_uav_ros_network uav1

mkdir -p /home/oem/handeye_calibration

rosbag record --lz4 \
  -O /home/oem/handeye_calibration/uav1_down_camera_handeye_04.bag \
  /UAV1/calibration/down_camera/image_raw \
  /UAV1/down_camera/camera_info \
  /UAV1/fast_lio/Odometry \
  /tf_static
```

若重新录制，必须换用新的序号，例如 `_05.bag`，不要覆盖已有录包。

## 6. 单个姿态的采集节奏

每个姿态严格按照以下节奏操作：

1. 用约 `2–3 s` 缓慢移动到新姿态。
2. 从画面确认完整棋盘格仍然可见。
3. 停止移动后等待约 `1 s`，让手部晃动消失。
4. 保持整架 UAV 完全静止 `3 s`。
5. 缓慢移动到下一个姿态。

不要在停留期间反复微调位置。若棋盘格没有完整进入画面，应重新调整并重新停稳一次，
而不是将调整过程当作有效姿态。

## 7. 推荐的移动顺序

角度和距离不要求精确到给定数值，重点是姿态差异明显、停留稳定且完整棋盘格可见。
建议采集 `20–25` 个有效停留姿态。

### 7.1 A 组：水平平移与高度变化

机体基本保持水平：

1. 棋盘格正上方，高度约 `1.0 m`；
2. 正上方，高度约 `0.8 m`；
3. 正上方，高度约 `1.2 m`；
4. 向机体前方平移约 `20 cm`；
5. 向机体后方平移约 `20 cm`；
6. 向机体左侧平移约 `20 cm`；
7. 向机体右侧平移约 `20 cm`；
8. 向左前方平移约 `15–20 cm`；
9. 向右后方平移约 `15–20 cm`。

X、Y、Z 三个方向的总位置范围最好都达到 `20–30 cm`。不能只在原地旋转，也不能像
上一包一样只产生约 `9 cm` 的位置变化。

### 7.2 B 组：横滚变化

回到棋盘格附近上方，绕机体前后轴倾斜：

10. 向左横滚约 `10–15°`；
11. 向右横滚约 `10–15°`；
12. 向左横滚约 `20–25°`；
13. 向右横滚约 `20–25°`。

倾斜时可以适当平移整架 UAV，使棋盘格保持在画面内。

### 7.3 C 组：俯仰变化

14. 机头下压约 `10–15°`；
15. 机头上抬约 `10–15°`；
16. 机头下压约 `20–25°`；
17. 机头上抬约 `20–25°`。

### 7.4 D 组：航向变化

18. 航向向左旋转约 `30°`；
19. 航向向右旋转约 `30°`；
20. 航向向左旋转约 `50–60°`；
21. 航向向右旋转约 `50–60°`。

只改变航向不能充分约束外参，必须同时完成前面的横滚和俯仰姿态。

### 7.5 E 组：组合姿态

22. 左滚约 `15°`、机头下压约 `10°`、左偏航约 `20°`；
23. 右滚约 `15°`、机头下压约 `10°`、右偏航约 `20°`；
24. 左滚约 `15°`、机头上抬约 `10°`、右偏航约 `20°`；
25. 右滚约 `15°`、机头上抬约 `10°`、左偏航约 `20°`。

如果某个大角度姿态无法看到完整棋盘格，应减小角度或适当升高 UAV，不能采用棋盘格
残缺的画面。

## 8. 结束录制与录包检查

完成全部姿态后，再保持最后一个姿态静止 `3 s`，然后在录包终端按一次 `Ctrl+C`，
等待 rosbag 完成索引写入。执行：

```bash
rosbag info /home/oem/handeye_calibration/uav1_down_camera_handeye_04.bag
```

录包必须包含：

```text
/UAV1/calibration/down_camera/image_raw
/UAV1/down_camera/camera_info
/UAV1/fast_lio/Odometry
```

确认文件时长、消息数量和文件大小正常后，再依次停止图像查看、限频、下视相机、
FAST-LIO 和 Mid360 终端。

## 9. 数据筛选规则

求解时不能把所有检测帧直接送入标定器，必须执行以下筛选：

1. 删除棋盘格不完整、模糊或重投影误差过大的帧；
2. 根据图像时刻前后的 FAST-LIO 位姿变化删除运动帧；
3. 将同一个连续静止姿态合并为一个停留段；
4. 每个停留段只选择一个清晰度和重投影误差最好的代表帧；
5. 删除与上一个保留点平移和旋转都过于接近的重复姿态；
6. 使用不同手眼算法交叉验证，并进行离群点和逐点删除复算；
7. 最终至少保留 `15` 个不同且稳定的有效姿态。

## 10. 外参验收标准

满足以下条件后，标定结果才算通过：

- 稳定且不同的有效姿态不少于 `15` 个；
- 棋盘格重投影 RMS 尽量不超过 `0.5 px`；
- 固定棋盘格世界坐标平移 RMS 不超过 `2–3 cm`；
- 固定棋盘格世界坐标最大平移残差不超过 `5 cm`；
- 固定棋盘格旋转 RMS 不超过 `1°`；
- 最大旋转残差不超过 `2°`；
- Park、Horaud、Andreff 等方法得到的平移差尽量不超过 `2–3 cm`；
- 删除任意一个姿态重新求解后，平移和旋转不能明显跳变；
- 标定平移应与实际测量的机体原点到相机光心安装距离相符。

任何一项明显不满足，都不能把结果用于精降。

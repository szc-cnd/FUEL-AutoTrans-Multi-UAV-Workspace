# 目标坐标与证据图远程上报

该包统一接收颜色标签、二维码和热源检测结果，通过 ROS TF 转换到 FAST-LIO 的 `camera_init` 世界坐标系，并将其作为比赛语义上的 `channel` 坐标。检测包的首个有效候选会立即生成实时观测并发送到远程端；候选在空间上连续命中 `confirm_hits`（默认 3）次后才生成最终确认记录和带标注的 JPEG。当前阶段候选结果只用于远程上报和 RViz 显示，不接入规划器。

## 数据流

```text
三个检测包的原始候选/相机坐标/检测图
  -> target_reporter_node.py
  -> camera optical frame -> body -> camera_init
  -> 首帧候选实时观测、空间确认、JSONL和JPEG
  -> /UAV0/target_reporting/markers（RViz显示）
  -> TCP 5000(实时观测、最终JSON/ACK) + TCP 5001(JPEG/图片ACK)
  -> target_report_server.py
```

FAST-LIO 在整场任务中不得重启或重置。UAV0 统一启动配置使用
`UAV0/camera_init` 作为 ROS 内部定位帧；上报 JSON 中使用业务名称
`channel`（当前数值坐标与比赛 `world` 坐标一致）。

## 输入话题

默认使用三个检测包的“原始候选”话题，先用于实时上报和 RViz 显示：

```text
/UAV0/color_tag_detector/candidate_point_camera  geometry_msgs/PointStamped
/UAV0/color_tag_detector/candidate_text           std_msgs/String(JSON)
/UAV0/vision/qr_candidate_pose_camera            geometry_msgs/PoseStamped
/UAV0/vision/qr_candidate_detected               std_msgs/String(JSON)
/UAV0/thermal/target_candidate_camera_point      geometry_msgs/PointStamped
/UAV0/thermal/target_candidate_detected          std_msgs/Bool
```

旧版检测节点可将 `use_detector_candidates` 设为 `false`，回退到下面的稳定结果话题：

```text
/UAV0/color_tag_detector/target_point_camera   geometry_msgs/PointStamped
/UAV0/color_tag_detector/result_text           std_msgs/String(JSON)
/UAV0/color_tag_detector/debug_image           sensor_msgs/Image

/UAV0/vision/qr_pose_camera                    geometry_msgs/PoseStamped
/UAV0/vision/qr_detected                       std_msgs/String(JSON)
/UAV0/vision/qr_debug_image                    sensor_msgs/Image

/UAV0/thermal/target_camera_point              geometry_msgs/PointStamped
/UAV0/thermal/fusion_valid                     std_msgs/Bool
/UAV0/thermal/debug_image                      sensor_msgs/Image
```

所有空间消息必须使用真实相机光学坐标 `frame_id`、源图像时间戳和米制坐标。TF树至少需要连通：

```text
UAV0/camera_init -> UAV0/body -> camera_link -> camera_color_optical_frame
UAV0/camera_init -> UAV0/body -> camera_link -> camera_depth_optical_frame
```

其中 `camera_link` 到彩色/深度光学坐标由 RealSense 发布，`UAV0/body` 到
`camera_link` 使用手眼标定外参。D435 原始点云 `/camera/depth/color/points` 使用
`camera_depth_optical_frame`；显示适配器再输出只包含检测目标附近点的
`/UAV0/target_reporting/detected_object_cloud`，默认保留目标中心半径 0.25 m 内的点。
显示适配器还根据这些目标附近点发布三维轴对齐线框包围盒；包围盒只用于 RViz，
不参与检测判定、避障或规划决策。

检查命令：

```bash
rosrun tf tf_echo UAV0/camera_init camera_color_optical_frame
rostopic echo /UAV0/target_reporting/observation
```

RViz 标记输出：

```text
/UAV0/target_reporting/markers  visualization_msgs/MarkerArray
/UAV0/target_reporting/detected_object_cloud  sensor_msgs/PointCloud2
/UAV0/target_reporting/detected_object_boxes  visualization_msgs/MarkerArray
```

标记使用 `world` 坐标系；当前比赛配置中 `world` 与 `channel` 的数值坐标一致。黄色球和文字表示候选，绿色球和文字表示已确认目标。
包围盒使用 D435 点云坐标系发布并由 RViz TF 转到固定坐标系：颜色标签为橙色、二维码为绿色、热源为品红色。
包围盒是目标附近点云的三维轴对齐包围盒；点云稀疏时使用以检测位置为中心的最小尺寸盒，
因此它不是二维图像像素级分割轮廓。

## 局域网配置

两台电脑连接同一个 Wi-Fi。建议固定地址：

```text
机载端：192.168.10.20
远程端：192.168.10.100
TCP JSON端口：5000
TCP 图片端口：5001
```

在远程端防火墙中允许Python或ROS访问TCP 5000、5001。机载端修改 `config/target_reporting.yaml` 中的 `remote_host`。

## 启动

远程端先启动：

```bash
rosrun target_reporting target_report_server.py \
  --host 0.0.0.0 --port 5000 --image-port 5001 \
  --output ~/received_target_reports
```

机载端在三个检测节点和 FAST-LIO 正常后启动：

```bash
roslaunch target_reporting target_reporting.launch
```

另开终端启动检测结果 RViz：

```bash
roslaunch target_reporting target_rviz.launch
```

该启动文件只启动显示适配节点和 RViz，不启动规划器，也不会改变飞行逻辑。若只想启动适配节点而手动打开 RViz，可传入 `open_rviz:=false`。

也可在远程端使用：

```bash
roslaunch target_reporting remote_server.launch
```

## 记录目录

机载端默认写入：

```text
~/target_reports/mission_YYYYMMDD_HHMMSS/
  reports.jsonl
  images/annotated/*.jpg
```

远程端默认写入指定的 `--output` 目录，结构相同。候选观测和确认观测都会发布到 `/UAV0/target_reporting/observation` 并通过 TCP 5000 实时发送到远程端，但候选不写入最终 JSONL，也不要求 ACK。只有确认目标生成最终记录和证据图片；确认目标不会再产生飞行目标。

`mission_id` 默认是 `competition_current`。同一场任务中机载端和远程端应保持该值不变；节点或电脑重启后会重新打开同一目录，根据 `event_acks.jsonl` 和 `image_acks.jsonl` 只补发尚未确认的最终JSON和图片。开始新一场正式任务前，将机载端配置和远程端 `--mission-id` 同时改成新的唯一名称，例如 `final_20260729_01`，避免把不同场次混在一起。

同一目标类型可以存在多个空间候选。相距超过 `dedup_distance_m` 的后续真目标会获得新的 `target_id`，不会被先前稳定误检阻挡。ACK重传保持相同 `seq`，远程端只保存一次。

## 真机测试清单

1. 使用 `rostopic echo` 确认三类相机坐标都带正确时间戳、光学帧名和米制数值。
2. 使用 `tf_echo` 确认三个相机帧均可转换到 `camera_init`。
3. 在已知位置放置目标，核对 RViz 标记、实时观察话题和远程端世界坐标一致。
4. 确认目标后检查机载与远程端均生成 JSONL 和可打开的标注JPEG。
5. 断开 Wi-Fi，确认检测和飞行不停止；恢复后确认待发结果自动补发。
6. 在最终消息尚未收到ACK时重启机载上报节点，确认同一 `mission_id` 下未确认JSON和图片会从磁盘重新加载并补发。
7. 重复发送相同 `seq`，确认远程 JSONL 只增加一条。
8. 先制造一个稳定误检，再在距离大于0.3米处放置真目标，确认远程端出现第二个 `target_id`。
9. 比赛任务期间不要重启FAST-LIO；若发生重启，应重新开始本场记录，不能混合两个世界原点。

## 本机关键测试

```bash
python target_reporting/test/test_core.py -v
python target_reporting/test/test_transport.py -v
python target_reporting/test/test_end_to_end.py -v
python -m compileall -q target_reporting
```

## Checkerboard hand-eye calibration

The calibration tool uses this aircraft's checkerboard: 11x8 inner corners
and 0.040 m square size. It estimates the fixed `body -> camera_link` transform
from FAST-LIO `/Odometry` and the RealSense color image without changing any
detector or flight-control node.

Collect a bag while the board is fixed and the complete rigid camera, IMU, and
LiDAR assembly is moved slowly through 10-20 varied poses:

```bash
mkdir -p ~/handeye_calibration
rosbag record -O ~/handeye_calibration/checkerboard.bag \
  /Odometry /camera/color/image_raw /camera/color/camera_info /tf_static
```

Run the solver:

```bash
source /opt/ros/noetic/setup.bash
source ~/db_ws/devel/setup.bash
rosrun target_reporting checkerboard_handeye_calibrate.py \
  --bag ~/handeye_calibration/checkerboard.bag \
  --output ~/handeye_calibration/body_camera.yaml \
  --pattern-cols 11 --pattern-rows 8 --square-size 0.040
```

The bag must contain the RealSense `/tf_static` chain from `camera_link` to
`camera_color_optical_frame`. Inspect `samples_used`, reprojection error, and
target-pose spread before using the result.

After a valid YAML is produced, publish both transforms:

```bash
roslaunch target_reporting camera_body_tf.launch \
  calibration_file:=/home/asus/handeye_calibration/body_camera.yaml
rosrun tf tf_echo camera_init body
rosrun tf tf_echo camera_init camera_color_optical_frame
```

Do not start `camera_body_tf.launch` before a real calibration YAML exists;
there is deliberately no identity-transform fallback.

OpenCV图像标注和ROS TF需要在安装了ROS Noetic、`python3-opencv` 与相机驱动的Ubuntu机载环境中实测。

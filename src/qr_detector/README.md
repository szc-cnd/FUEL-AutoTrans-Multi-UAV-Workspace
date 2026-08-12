# qr_detector

这是一个用于 ROS Noetic 的普通二维码检测基线包，面向 Intel
RealSense D435 的彩色图像与对齐深度图。

本包用于无人机比赛通道内作业目标点中的普通 QR Code 检测，不是降落用
ArUco 码检测器。

## 功能

- 订阅 RealSense 彩色图像、对齐深度图和相机内参。
- 使用 OpenCV `cv2.QRCodeDetector` 检测普通 QR Code。
- 检测到二维码角点后，即使解码内容为空，也认为发现了二维码目标。
- 根据四个角点计算二维码中心像素坐标。
- 在中心点附近取深度窗口中位数，避免只依赖单个深度像素。
- 使用相机内参反投影，输出二维码中心在相机坐标系下的位置。
- 发布 JSON 检测状态、`PoseStamped` 位姿和调试图像。
- 支持连续多帧确认、短时间丢失保持和 EMA 位置滤波。
- 支持保存检测失败帧，便于后续采集数据训练 YOLO 兜底模型。
- 有效候选会发布到机载候选话题供 target_reporting/RViz 观察，但默认不发送到远程端；只有连续确认后的结果才进入稳定上报链路。

## 编译

将 `qr_detector` 放到 `catkin_ws/src` 下，然后编译：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x src/qr_detector/scripts/qr_detector_node.py
```

## 启动

```bash
roslaunch qr_detector qr_detector.launch
```

## 订阅话题

- `/camera/color/image_raw`
- `/camera/aligned_depth_to_color/image_raw`
- `/camera/color/camera_info`

## 发布话题

- `/UAV0/vision/qr_pose_camera`
  - 类型：`geometry_msgs/PoseStamped`
  - 含义：二维码中心点相对于相机坐标系的位置。
- `/UAV0/vision/qr_detected`
  - 类型：`std_msgs/String`
  - 内容：JSON 字符串，包括 `detected`、`data`、`points`、
    `center_u`、`center_v`、`x`、`y`、`z`、`method` 等字段。
- `/UAV0/vision/qr_debug_image`
  - 类型：`sensor_msgs/Image`
  - 内容：带二维码边框、中心点、识别内容和相机坐标的调试图像。

## 参数

参数文件位于 `config/qr_detector.yaml`。

- `image_topic`：彩色图像话题。
- `depth_topic`：对齐到彩色图的深度图话题。
- `camera_info_topic`：彩色相机内参话题。
- `depth_window_size`：中心点附近取深度中位数的窗口大小，默认 `11`。
- `min_area`：二维码四边形的最小像素面积，当前为 `100`。
- `min_side_length`：二维码最短边像素阈值，当前为 `12`。
- `max_side_ratio`：最长边与最短边比例上限，当前为 `8`。
- `max_angle_cos`：角点直角约束阈值，当前为 `0.90`。
- `qr_eps_x`、`qr_eps_y`：OpenCV QR 角点扫描容差，当前均为 `0.25`。
- `preprocess_mode`：预处理方式，当前为 `gray`。
- `upscale_factor`：检测前图像放大倍数，当前为 `1.5`。
- `confirm_frames`：连续检测到多少帧后确认 `detected=true`，当前为 `3`。
- `draw_raw_candidates`：是否在二维码调试图像中绘制单帧原始候选，当前为 `false`；RViz 候选标记仍由 target_reporting 单独显示。
- `lost_hold_time`：短时间丢失后保留上一帧结果的时间，默认 `0.3` 秒。
- `ema_alpha`：相机坐标 `x,y,z` 的 EMA 滤波系数。
- `save_failed_frame`：是否保存检测失败帧，默认 `false`。
- `failed_frame_dir`：检测失败帧保存目录，默认 `~/qr_failed_frames`。
- `failed_frame_interval`：检测失败帧保存间隔，默认 `1.0` 秒。

## 输出坐标

当前节点只输出二维码中心点在相机坐标系下的位置：

```text
Z = depth
X = (center_u - cx) * Z / fx
Y = (center_v - cy) * Z / fy
```

后续如果需要输出通道坐标系下的位置，可以在此结果基础上接入无人机定位和
相机外参变换。

## 后续扩展

当前版本只使用 OpenCV `QRCodeDetector`。如果比赛现场光照、运动模糊或角度
导致 OpenCV 检测不稳定，可以在节点中的 QR 检测入口后面加入 YOLO 兜底分支，
复用现有的深度取值、相机反投影、滤波和发布逻辑。

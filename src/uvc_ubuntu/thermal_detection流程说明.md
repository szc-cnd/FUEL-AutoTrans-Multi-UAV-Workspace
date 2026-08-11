# 热成像异常温度源检测流程说明

## 当前取流参数

```text
streamType = 10
UVC 请求尺寸 = 384 x 288
offset = 0
len = 221184
格式 = YUYV / YUV422
Python reshape = (288, 384, 2)
gray = yuv[:, :, 0]
```

当前比赛检测只使用 YUV-only 实时图像，不解析温度矩阵，不使用全屏测温组合流。

## 取流流程

```text
热成像相机
  ↓
uvc_demo 打开相机
  ↓
streamType=10，仅 YUV
  ↓
请求 384 x 288 YUYV
  ↓
offset=0，输出完整一帧 221184 字节
  ↓
Python 循环读满一帧
```

## 检测流程

```text
YUYV 原始帧
  ↓
取 Y 通道得到灰度图
  ↓
ROI 裁剪
  ↓
GaussianBlur 仅用于检测 ROI
  ↓
threshold = mean + k * std
  ↓
提取高亮区域
  ↓
轮廓面积过滤
  ↓
选择平均灰度最高的候选亮斑
  ↓
输出中心像素 cx, cy
```

## 图像显示

debug 图基于原始灰度图画框：

```text
蓝色框：ROI
绿色框：当前帧亮斑
红色十字：当前帧中心点
```

显示放大使用：

```python
cv2.INTER_NEAREST
```

避免低分辨率热图被默认插值抹糊。

## 默认参数

```text
shift_x = 0
shift_y = 0
uvc_offset_fix = 0
roi_margin_x = 30
roi_margin_y = 20
k = 2.0
min_area = 20
max_area = 5000
```

## 单机运行

```bash
cd ~/uvc_ubuntu
make clean
make
sudo pkill -9 uvc_demo
sudo -E python3 thermal_detect.py --shift-x 0 --roi-margin-x 30 --uvc-offset-fix 0
```

期望日志：

```text
req_width=384,req_height=288,size=221184,len=221184,offset=0,offset_fix=0
```

## ROS 运行

```bash
roscore
```

```bash
cd ~/uvc_ubuntu
sudo -E bash -c 'source /opt/ros/noetic/setup.bash && cd /home/cat/uvc_ubuntu && python3 -u thermal_ros_node.py'
```

查看：

```bash
rostopic echo /UAV0/thermal/target_detected
rostopic echo /UAV0/thermal/target_pixel
rqt_image_view /UAV0/thermal/debug_image
```


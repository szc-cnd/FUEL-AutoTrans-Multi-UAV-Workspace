# uvc_ubuntu ROS 功能包使用说明

当前目录已经同时支持两种方式：

- 标准 ROS Noetic 功能包：使用 `catkin_make`、`rosrun`、`roslaunch`。
- 原有独立工程：继续使用根目录 `Makefile` 和 `python3 thermal_detect.py`。

热成像参数固定为 `streamType=10`、`384x288`、YUYV、`offset=0`，单帧长度为 `221184` 字节。

## 一、放入 catkin 工作空间

将完整目录复制到新电脑：

```bash
mkdir -p ~/catkin_ws/src
cp -a /你的复制来源/uvc_ubuntu ~/catkin_ws/src/
cd ~/catkin_ws/src/uvc_ubuntu
```

最终应存在：

```text
~/catkin_ws/src/uvc_ubuntu/package.xml
~/catkin_ws/src/uvc_ubuntu/CMakeLists.txt
~/catkin_ws/src/uvc_ubuntu/thermal_ros_node.py
```

## 二、安装依赖

```bash
sudo apt update
sudo apt install build-essential cmake pkg-config libusb-1.0-0-dev \
  python3-numpy python3-opencv python3-yaml \
  ros-noetic-ros-base ros-noetic-cv-bridge ros-noetic-image-transport
```

D435 融合还需要：

```bash
sudo apt install ros-noetic-realsense2-camera
```

## 三、内置 libuvc

当前 ROS 包已经包含完整的 `libuvc` 源码，不再依赖旧电脑的 `/home/cat/libuvc` 软链接，也不需要预先执行 `sudo make install`。

复制完成后检查关键文件：

```bash
test -f ~/catkin_ws/src/uvc_ubuntu/libuvc/include/libuvc/libuvc_internal.h
echo $?
```

输出 `0` 表示内置源码完整。`catkin_make` 会通过包内 `CMakeLists.txt` 自动构建静态 `libuvc`；根目录的旧兼容命令 `make` 也会自动在 `libuvc/build` 中完成构建。

## 四、catkin 编译

```bash
cd ~/catkin_ws
source /opt/ros/noetic/setup.bash
catkin_make
source ~/catkin_ws/devel/setup.bash
rospack find uvc_ubuntu
```

最后一条命令应输出：

```text
/home/你的用户名/catkin_ws/src/uvc_ubuntu
```

可以把工作空间环境加入 `~/.bashrc`：

```bash
echo 'source /opt/ros/noetic/setup.bash' >> ~/.bashrc
echo 'source ~/catkin_ws/devel/setup.bash' >> ~/.bashrc
```

## 五、运行热成像检测

终端 1：

```bash
roscore
```

终端 2，推荐使用 launch：

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch uvc_ubuntu thermal_detector.launch
```

也可以直接使用 rosrun：

```bash
source ~/catkin_ws/devel/setup.bash
rosrun uvc_ubuntu thermal_ros_node.py
```

查看输出：

```bash
rostopic echo /UAV0/thermal/target_detected
rostopic echo /UAV0/thermal/target_pixel
LIBGL_ALWAYS_SOFTWARE=1 rqt_image_view /UAV0/thermal/debug_image
```

如果普通用户没有 USB 权限，需要配置相机 udev 规则，或者临时在保留 ROS 环境的 root shell 中启动。不要同时运行两个 `uvc_demo`，否则会出现相机占用错误。

## 六、运行 D435 融合

先启动 RealSense 相机节点，再启动热成像检测和融合组合：

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

另一个终端：

```bash
source ~/catkin_ws/devel/setup.bash
roslaunch uvc_ubuntu thermal_d435_system.launch
```

如果热成像相机与 D435 的相对位置、角度、分辨率或裁剪方式发生变化，需要重新标定 `thermal_to_d435_homography`。

## 七、保留的独立运行方式

原来的编译和单机检测方式仍然保留：

```bash
cd ~/catkin_ws/src/uvc_ubuntu
make clean
make
sudo -E python3 thermal_detect.py \
  --shift-x 0 --roi-margin-x 30 --uvc-offset-fix 0 --display-scale 3.0
```

无框原始画面：

```bash
sudo -E python3 thermal_raw_view.py --display-scale 3.0
```

## 八、运行结果检查

底层日志应包含：

```text
req_width=384,req_height=288,size=221184,len=221184,offset=0,offset_fix=0
```

ROS 应发布：

```text
/UAV0/thermal/image_raw
/UAV0/thermal/debug_image
/UAV0/thermal/target_detected
/UAV0/thermal/target_pixel
```

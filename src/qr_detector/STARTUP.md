# QR Detector 启动流程

当前二维码检测只使用 Terminator 启动脚本。

## 一键启动

```bash
bash ~/scripts/start_qr_detector.sh
```

脚本只打开一个 Terminator 窗口，并创建 3 个 pane：

- ROS Master
- RealSense D435/D455，相机和彩色对齐深度均为 `640x480 @ 30Hz`
- QR Detector 二维码检测节点

ROS Master、相机和 QR 节点都已经运行时，脚本会输出跳过信息，不会重复启动或弹出新窗口。

## 查看结果

```bash
rqt_image_view /UAV0/vision/qr_debug_image
rostopic echo /UAV0/vision/qr_detected
rostopic echo /UAV0/vision/qr_pose_camera
rostopic hz /UAV0/vision/qr_detected
rostopic hz /camera/color/image_raw
```

## 单独启动

```bash
bash ~/scripts/start_qr_detector.sh --roscore
bash ~/scripts/start_qr_detector.sh --camera
bash ~/scripts/start_qr_detector.sh --detector
```

## 修改二维码参数

```bash
gedit /home/oem/match_ws/src/qr_detector/config/qr_detector.yaml
```

修改后重启 QR Detector pane：

```bash
bash /home/oem/match_ws/src/qr_detector/scripts/start_qr_detector.sh --detector
```

Terminator 布局文件为：

```text
/home/oem/match_ws/src/qr_detector/scripts/terminator_qr_detector.conf
```

当前参数为灰度预处理、`upscale_factor=1.5`、`qr_eps_x/y=0.15`、最短边
`16px`、边长比例上限 `4`、连续 `5` 帧真实性和深度校验，并关闭调试图像中的
单帧原始候选框。角点候选仍通过 `/UAV0/vision/qr_candidate_pose_camera` 和
`/UAV0/vision/qr_candidate_detected` 供机载 RViz 显示；只有内部解码校验和
深度一致性通过后，才会累计 `target_reporting` 的确认次数。候选不会发送到
远程端。

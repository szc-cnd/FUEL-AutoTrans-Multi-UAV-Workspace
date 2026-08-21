# Precision landing operator guide

> 比赛入口 `landing_stack.launch` 只启动下视相机和精确降落节点，不负责
> 解锁、起飞或悬停。`landing_test.launch` 是旧的独立测试入口，现已默认关闭
> 自动开始和解锁权限，不得用于正式比赛启动流程。

比赛入口同时启动 `landing_setpoint_arbiter`。简单控制器应发布到
`/UAVx/control/position_setpoint`，不能直接发布 MAVROS；正常飞行时仲裁器转发
控制器指令。收到 `/UAVx/need_to_land=true` 后，降落节点通过全部预检查并发出
第一帧保持指令时，仲裁器锁存 `LANDING` 所有权并永久阻断控制器输出，直至相关
节点重启。可通过 `/UAVx/landing/control_owner` 查看 `CONTROLLER`、
`LANDING_PENDING` 或 `LANDING`。

## 出通道后的平台搜索与自动交接

`precision_landing.launch` 默认启动下视 `landing_search_node`；比赛统一入口还会启用
前视 D435 粗定位节点。完整链路为：

```text
SEARCH_CORRIDOR -> CROSS_EXIT -> SEARCH_OUTSIDE_LANDING
-> 出口原地前视：静看 1 s，缓慢扫描 -30° 到 +30° 并回正
-> 前视跨扫描累计两个不同 ID；不足两个时升至 2 m 做 Diff 下视蛇形补齐
-> 协调器按出口距离分配：UAV0 远平台、UAV1 近平台
-> UAV0 到远平台上方等待下视指定 ID 复核 -> APPROACH_LANDING
-> 规划到平台上方 2.00 m -> /UAVx/mission/landing_request
-> /UAVx/need_to_land -> 精确降落
-> /UAV0/landing/success=true 后才释放 UAV1 前往近平台并重复下视复核、精降
```

搜索节点只在 `/UAVx/mission/task_status` 进入门外搜索阶段后接受 ArUco，避免通道内
误检提前结束任务。它将相机测量通过
[`config/landing_search.yaml`](config/landing_search.yaml) 中的下视相机手眼外参和同步
FAST-LIO 里程计转换到世界系，并完成相机内五帧锁定与世界系八帧稳定过滤。稳定结果
发布到 `/UAVx/mission/detection/final_aruco`。Diff 搜索管理器生成高于平台 `2.00 m`
的接近目标，到达后才申请交接给降落代码。

前视节点读取 D435 彩色图、对齐深度、`body_camera_03.yaml` 静态外参以及与图像同步的
FAST-LIO 里程计。每个 ID 分别经过连续帧、深度一致性和世界坐标稳定过滤，并跨完整
偏航扫描累计到 `/UAV0/landing/front/candidates`。双机模式不再响应单个
`front_aruco_hint`；两个不同 ID 由协调器合并去重，下视坐标优先覆盖同 ID 的前视粗
坐标。协调器只把远平台粗目标发布到 `/UAV0/landing/assigned_target`，它不能写入
`final_aruco`、`landing_request` 或 `/need_to_land`。UAV0 到达粗目标后若下视一直未
复核指定 ID，则保持悬停，不下降也不改去另一平台。

通道内及常规 FUEL 探索的水平速度上限保持 `0.20 m/s`。进入门外搜索后，降落专用
Diff 的速度上限为 `0.30 m/s`、加速度上限为 `0.60 m/s²`；发布降落请求后不再使用
搜索阶段速度，由视觉精降节点独立控制下降。

规划器到达后，搜索节点仍会复核目标和里程计新鲜度、水平误差以及接近高度；只有全部
通过才发布 `/UAVx/need_to_land=true`。该触发是锁存的，之后由
`landing_setpoint_arbiter` 将控制权交给 `precision_landing_node`，不会交给简单控制器。
关键诊断话题为：

```bash
rostopic echo /UAV0/landing/search/status
rostopic echo /UAV0/landing/search/target_world
rostopic echo /UAV0/landing/front/status
rostopic echo /UAV0/landing/front_aruco_hint
rostopic echo /UAV0/mission/task_status
rostopic echo /UAV0/landing/control_owner
```

搜索节点和最终精降节点都使用“下视相机到 FAST-LIO 机体原点”的同一组
Park 手眼标定结果。`landing_search.yaml` 以四元数存储，`precision_landing.yaml`
以旋转矩阵存储；两者的平移均为
`[0.073831, -0.039020, -0.133468] m`，不再使用旧的 `[0.07, 0, 0]` 粗测偏移。

This package controls a PX4 vehicle through MAVROS only after a rising
`/need_to_land` trigger, valid camera calibration, fresh vehicle data, and an
armed `OFFBOARD` state. Treat it as flight-critical software: begin every
session without propellers and keep an RC takeover mode available.

## Build and configure

From the Catkin workspace root, install declared ROS dependencies and build:

```bash
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

The deployed default is
[`config/precision_landing.yaml`](config/precision_landing.yaml). Its private
parameters are loaded by the launch file. `marker.dictionary` is deliberately
fixed to `DICT_4X4_250`; the node rejects any other value. The competition
marker's measured black-square side length is `0.60 m` and must match
`marker.size_m`. Do not use an uncalibrated camera:
`safety.require_camera_info` is mandatory and the node
refuses to start if it is false. Invalid or stale camera information prevents
descent.

## Calibrate the downward-facing camera

1. Print a checkerboard and measure its square side. Capture diverse views
   (tilt, distance, and every part of the frame) while the camera is mounted
   exactly as it will fly.
2. With the camera driver running, calibrate it with the actual board's
   `11x8` inner corners and `0.040 m` square side:

   ```bash
   rosrun camera_calibration cameracalibrator.py --size 11x8 --square 0.040 \
     image:=/usb_cam/image_raw camera:=/usb_cam
   ```

3. Save the result in the camera driver's calibration URL/configuration, then
   restart the driver. Confirm a non-zero intrinsic matrix is published:

   ```bash
   rostopic echo -n 1 /usb_cam/camera_info
   ```

   The message must have non-zero `width`, `height`, `K[0]`, `K[2]`, `K[4]`,
   `K[5]`, and `K[8]`, plus finite distortion values. A visible image without
   this message is not a calibrated input and must not be used for descent.

4. Verify the body transform. The shipped rotation and translation are the
   2026-08-15 Park hand-eye result from
   `/home/asus/handeye_calibration/body_down_camera_park_03_stable.yaml`, not
   an idealized axis mapping. It maps image-right to aircraft-right, image-top
   to aircraft-forward, and camera-positive Z to body-down while preserving
   the measured small mounting-angle errors. If command directions are wrong,
   stop testing and re-check the mounted camera and calibration; do not replace
   the calibrated matrix with an ideal matrix merely to change a sign.

The matrix is only a documented default, not proof of the physical
installation. Verify the real mounting, cable orientation, and both command
axes with propellers removed before every mounting change. Do not fly if
either sign is wrong.

`marker.max_tilt_deg` rejects pose estimates whose marker plane is tilted too
far from the camera viewing ray. The shipped `60 deg` limit is a permissive
starting point; validate it from recorded images and tighten it for the actual
camera, marker, and approach geometry.

## Detector-only and direction check

Launch without sending a landing trigger; this is detector-only operation—the
node processes and publishes diagnostics but sends no setpoints before the
trigger.

```bash
roslaunch precision_landing precision_landing.launch
rostopic echo /landing/locked_id
rostopic echo /landing/debug_image
```

The image and camera-info inputs can be changed at launch without editing the
YAML:

```bash
roslaunch precision_landing precision_landing.launch \
  image_topic:=/downward/image_raw camera_info_topic:=/downward/camera_info
```

With props removed, arm/offboard only when permitted by the test setup, move
the marker toward the top of the image, and inspect `/landing/error_xy` and
the raw local setpoint. It must request forward body motion. If it does not,
do not fly: fix the camera extrinsic as described above and repeat this test.

## Landing run and monitoring

Start the full node as above, establish a stable hover in `OFFBOARD`, and
publish one rising trigger only after the preflight checklist is satisfied:

```bash
rostopic pub -1 /need_to_land std_msgs/Bool "data: true"
```

Watch the controller state, locked marker ID, horizontal error, and PX4 mode:

```bash
rostopic echo /landing/state
rostopic echo /landing/locked_id
rostopic echo /landing/error_xy
rostopic echo /mavros/state
```

The normal sequence is acquire, align, and visual descent above `1.50 m`.
At or below `1.50 m`, after horizontal error remains at or below `0.08 m`
for `0.50 s`, the node records the fused local X/Y and estimated ground
height. It then holds full XYZ position targets: X/Y and yaw remain fixed
while the Z target moves down at `0.10 m/s`; ArUco visibility is no longer
required during this phase. The node stays in `OFFBOARD` until an estimated
height of `0.30 m` or lower, then requests PX4 to switch to `AUTO.LAND` for
the final descent and motor stop.

Freshness is checked twice for every Image, CameraInfo, PoseStamped, and
MAVROS State sample: callback receive age must remain within its configured
timeout and the message header age must remain within the same timeout. Zero
header stamps are rejected because their source age is unknowable. Header
stamps more than `safety.max_header_future_sec` (default `0.05 s`) ahead of
ROS time are rejected; smaller future skew is tolerated for synchronized
camera/PX4 hosts, while receive-age checking remains mandatory. Synchronize
all onboard clocks and do not enlarge this tolerance to mask a clock fault.

Return control immediately with the RC flight-mode switch, or explicitly
change PX4 out of `OFFBOARD`, for example:

```bash
rosservice call /mavros/set_mode "base_mode: 0
custom_mode: 'POSCTL'"
```

## Real-vehicle landing-only test

`landing_test.launch` starts the precision-landing node plus a dedicated
takeoff/handoff node. It does not start MAVROS or the downward camera. Its
default is fail-safe: `allow_arming=false` and `auto_start=false`, so merely
launching it cannot arm or take off. A real-vehicle automatic test requires
the operator to explicitly enable both arguments after completing all checks.

Use an open test area, remove all unrelated setpoint publishers, confirm an RC
mode-takeover path, place the 0.60 m marker below the takeoff point, then start
the already-configured MAVROS and `/usb_cam` camera before running:

```bash
roslaunch precision_landing landing_test.launch \
  allow_arming:=true auto_start:=true
```

Watch the test controller and landing controller in separate terminals:

```bash
rostopic echo /landing_test/status
rostopic echo /landing/state
rostopic echo /mavros/state
```

With both explicit gates enabled, no separate start service call is required.
After the 5 s countdown the test node captures the current local XY/yaw,
streams a position setpoint for 2 s, requests `OFFBOARD`, arms, climbs to the
configured `2.05 m` relative height, and
holds for 2 s. It then publishes `/need_to_land=true` while continuing the
hold setpoint. Once `precision_landing_node` reaches `ACQUIRE` and begins
publishing its own setpoints, the test node stops its position setpoints and
leaves landing control to `precision_landing_node`.

For a non-arming inspection launch, explicitly override both gates:

```bash
roslaunch precision_landing landing_test.launch \
  allow_arming:=false auto_start:=false
```

Expected test status sequence:

```text
WAITING -> PRESTREAM -> REQUEST_OFFBOARD -> REQUEST_ARM
-> TAKEOFF -> HOVER -> HANDOFF -> COMPLETE
```

Before handoff, an operator-requested abort is available:

```bash
rosservice call /landing_test/abort
```

Abort clears the landing trigger and enters `FAILSAFE_HOLD`; it intentionally
continues the last position setpoint. Use the RC to switch out of `OFFBOARD`
before stopping the node. Never kill the node while it is the only active
OFFBOARD setpoint source.

## Staged field-test checklist

Run every stage at a safe location with an observer and an RC takeover path.
Do not advance after an unexpected command direction, state transition, or
mode change.

1. Props removed: validate topics, camera calibration, marker lock, state
   transitions, command direction, and manual mode takeover.
2. Restrained/tethered hover: verify `OFFBOARD` ownership and zero descent
   until the trigger and alignment gates are satisfied.
3. XY-only: keep altitude fixed and confirm centering responses in both axes.
4. Visual-descent band: confirm the `0.25 m/s` limit above `1.50 m`.
5. Handoff height: at or below `1.50 m`, confirm visual correction continues
   until the error remains at or below `0.08 m` for `0.50 s`.
6. Fixed-XYZ descent: confirm the recorded fused X/Y stays fixed while the Z
   target descends at `0.10 m/s`, including when ArUco leaves the image.
7. Low-height cutoff: confirm the node logs the `0.30 m` threshold and requests
   PX4 `AUTO.LAND`; PX4 completes the final descent and motor stop.

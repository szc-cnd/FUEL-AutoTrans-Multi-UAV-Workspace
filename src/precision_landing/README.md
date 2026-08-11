# Precision landing operator guide

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

4. Verify the body transform. The default rotation maps camera coordinates to
   body coordinates so a marker moved toward the image top is a positive
   forward body error. If command directions are wrong, stop testing, then
   correct the mounting transform by reversing/rotating
   `camera_to_body.rotation` (and update `translation_m` for the measured
   camera offset). The shipped rotation is for an ordinary downward optical
   camera whose image top points aircraft-forward:
   `[[0,-1,0],[-1,0,0],[0,0,-1]]`. It maps image-right to aircraft-right
   (negative body-FLU Y), image-top to aircraft-forward (positive body-FLU X),
   and camera-positive Z to body-down. The controller uses the resulting
   positive downward distance as marker height. The rotation must remain a
   proper right-handed 3×3 matrix.

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
takeoff/handoff node. It does not start MAVROS or the downward camera. This
launch is intentionally configured for a real-vehicle automatic start:
once MAVROS, local pose, the setpoint plugin, and `precision_landing_node` are
all ready, a 5 s warning countdown begins and the vehicle takes off.

Use an open test area, remove all unrelated setpoint publishers, confirm an RC
mode-takeover path, place the 0.60 m marker below the takeoff point, then start
the already-configured MAVROS and `/usb_cam` camera before running:

```bash
roslaunch precision_landing landing_test.launch
```

Watch the test controller and landing controller in separate terminals:

```bash
rostopic echo /landing_test/status
rostopic echo /landing/state
rostopic echo /mavros/state
```

No separate start service call is required. After the 5 s countdown the test
node captures the current local XY/yaw, streams a position setpoint for 2 s,
requests `OFFBOARD`, arms, climbs 2.50 m relative to the starting local Z, and
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

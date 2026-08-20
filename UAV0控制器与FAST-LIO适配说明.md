# UAV0 控制器与 FAST-LIO 适配说明

## 启动

```bash
cd /home/asus/match_ws
source devel/setup.bash
bash shfiles/start_uav0_six_terminator.sh
```

六分屏依次启动 MAVROS、MID360、FAST-LIO、高频视觉位姿、FUEL/RViz，
以及 FUEL bridge + AutoTrans NMPC + logger + rosbag。脚本不会自动解锁、
切换 OFFBOARD 或发布目标点。

## 状态来源

| 用途 | 话题 |
|---|---|
| NMPC 位置和线速度 | `/UAV0/fast_lio/Odom_high_freq` |
| NMPC 控制姿态和外力估计姿态 | `/UAV0/mavros/local_position/odom` |
| 角速度和机体系加速度 | `/UAV0/mavros/imu/data` |
| FAST-LIO 普通点云校正里程计（诊断/回滚） | `/UAV0/fast_lio/Odometry` |
| FAST-LIO 机体系 Livox IMU | `/UAV0/livox/imu_body` |

Livox IMU 按 UAV0 的 15° 安装俯仰角转换，向量和协方差使用同一旋转。
原有 CAV0 的质量、惯量、推力模型、外力估计和 MPC 参数未替换为 UAV1 数值。

## MPC 失败与规划握手

- 单次求解失败后短时保持最近一次有限控制量，然后进入锁点悬停恢复。
- 恢复期间拒绝旧轨迹、清空旧命令和外力补偿；不会因恢复超时自动降落。
- 恢复成功发布 `/UAV0/planning_restart_trigger`，FUEL 从最新高频里程计重新规划。
- CH10 合法人工降落会发布 `/UAV0/planning_stop_trigger`；若 NMPC 正在恢复，控制器停止
  NMPC/setpoint 并直接请求 PX4 `AUTO.LAND`，不再依赖失效中的求解器下降。
- bridge 使用非 latched 发布，不再因本地超时伪造 `ACTION_ABORT`。

## 日志

CSV、文本日志、自动 rosbag 默认写入 `~/.ros/autotrans_mpc_logs` 下的同一个单次运行目录。
rosbag 由 logger 统一启动并在退出时以 SIGINT 正常写入索引；六窗脚本不再重复录包。logger 记录
高频实际轨迹、MAVROS 状态、FUEL 原始 B-spline、PositionCommand、桥接后轨迹、
stop/restart 事件和控制器 rosout。退出时若已安装 evo，会在运行目录下生成
`evo_report/summary.md`；未安装时 summary 会记录明确失败原因。

## 上机前检查

```bash
rostopic hz /UAV0/fast_lio/Odom_high_freq
rostopic hz /UAV0/mavros/local_position/odom
rostopic echo -n 1 /UAV0/livox/imu_body
rostopic echo -n 1 /drone_0_traj_server/heartbeat
```

首次验证必须无桨进行：检查六屏依赖等待、CH8 三段模式、模拟求解失败后的悬停、
stop/restart 重新规划，以及只有 CH10/合法任务流程能触发降落。

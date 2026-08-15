# 动态避障飞行日志设计

## 目标

每次执行 `ros_start_ldop_detection.sh` 时自动创建一个独立的带时间戳日志目录，记录动态目标 ID 变化、目标位置/速度/尺寸、无人机 `/Odometry` 以及原始候选框和稳定轨迹框数量，供飞行结束后复盘。

## 生命周期

- 启动脚本打开独立日志终端，并在 `/home/oem/ldop_logs/<时间戳>/` 启动后台记录器。
- 记录器先创建日志文件，再等待 ROS Master 和目标话题；因此即使 LDOP 稍后启动，日志目录仍然存在。
- 停止脚本通过 PID 文件向记录器发送退出信号，记录器写入结束时间并保留全部日志，不删除历史目录。

## 文件

- `dynamic_frames.csv`：每帧目标 ID 集合和目标数量。
- `dynamic_objects.csv`：每个目标的 ID、位置、速度、尺寸、类别和运动模型。
- `id_events.csv`：ID 集合变化前后的对照。
- `odometry.csv`：无人机位置和速度。
- `marker_counts.csv`：原始候选框与稳定轨迹框数量。
- `metadata.txt`、`logger_console.log`：日志时间、话题和运行状态。

## 约束

不改变 `/ldop/dynamic_objects` 接口，不安装新依赖；日志只追加新目录，不覆盖历史飞行记录。

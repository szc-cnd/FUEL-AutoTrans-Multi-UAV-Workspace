# 动态避障飞行日志实施计划

> **For agentic workers:** 本计划在当前会话内执行；用户已明确要求每次飞行保留日志。

**目标：** 将动态目标与飞控里程计记录接入现有一键启动/停止脚本。

**架构：** Python ROS 记录器负责订阅和写 CSV；启动脚本创建带时间戳目录并保存记录器 PID；停止脚本只结束当前记录器，历史日志永久保留。

**技术栈：** Bash、Python 3、rospy、ROS 消息、CSV。

## 全局约束

- 日志目录：`/home/oem/ldop_logs/<时间戳>/`。
- 记录器必须在 ROS Master 或目标话题暂不可用时继续等待。
- 每次启动使用新目录，不覆盖历史记录。
- 本地和远程脚本同步，Git 提交备注使用中文。

## 任务

- [x] 添加 `ros_flight_logger.py`，记录动态目标、ID 变化、Odometry 和 Marker 数量。
- [x] 添加 `ros_start_flight_logger.sh`，创建目录、启动记录器并写 PID。
- [x] 修改 `ros_start_ldop_detection.sh` 自动打开日志终端。
- [x] 修改 `ros_stop_all.sh` 安全结束记录器并保留日志。
- [x] 同步远程、检查 Python 语法和 Shell 参数、提交并告知日志位置。

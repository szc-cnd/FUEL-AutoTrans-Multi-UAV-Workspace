# LDOP 一键启动脚本设计

日期：2026-08-03

## 背景

当前雷达定位与 LDOP 检测需要分别执行启动命令：

1. `/home/oem/scripts/ros_start_lidar_only.sh` 启动 Livox 驱动、MAVROS、Fast-LIO 和定位桥接节点。
2. 另开终端，加载 `/home/oem/ldop_ws/ldop_env.sh` 后执行 `roslaunch ldop run_ldop.launch`。

现有 Fast-LIO 启动文件会默认启动 RViz。用户希望雷达初始化阶段不启动 RViz，并把完整启动流程放入之前的脚本目录；最终只保留 LDOP 启动文件中的 RViz。

## 目标

- 保留现有 `ros_start_lidar_only.sh` 的四个启动标签页和命令职责。
- 修改 Fast-LIO 启动参数，使雷达定位脚本不启动 RViz。
- 新增一个完整启动脚本，依次启动雷达定位链路，并在定位话题可用后启动 LDOP 检测与 LDOP RViz。
- 在本地 LDOP 仓库的 `scripts/` 目录保留脚本副本，并同步到远程 `/home/oem/scripts/`，便于 VS Code 查看每次变更。
- 启动脚本不重新编译、不安装依赖；编译仍由用户单独执行。

## 非目标

- 不修改雷达驱动、Fast-LIO、MAVROS 或 LDOP 的检测算法。
- 不把雷达初始化 RViz 配置删除；仅通过 `rviz:=false` 禁止本次启动。
- 不改变 `ros_stop_all.sh` 的现有清理范围。
- 不把所有节点改为后台守护进程；保留每个链路独立终端，方便观察日志和手动停止。

## 方案比较

### 方案 A：包装现有雷达脚本（推荐）

新增 `/home/oem/scripts/ros_start_ldop_detection.sh`。该脚本调用现有 `ros_start_lidar_only.sh`，然后再打开一个 LDOP 标签页。LDOP 标签页先加载 LDOP 环境，并持续检查 `/cloud_registered` 与 `/Odometry` 是否已经有实际消息；两者都可用后才执行 `roslaunch ldop run_ldop.launch`。

优点是复用已有四个启动项，避免两份启动命令逐渐不一致；修改范围小，仍然可以单独启动雷达定位。代价是完整启动依赖现有雷达脚本的路径和终端环境，因此脚本会对必要文件和 `gnome-terminal` 做启动前检查。

### 方案 B：在完整脚本中重复五个启动项

完整脚本自己定义 Livox、MAVROS、Fast-LIO、桥接和 LDOP 五个标签页。这样单文件更直观，但会复制现有命令，后续修改容易出现两个脚本不一致。

### 方案 C：后台进程监督器

用一个脚本在后台启动并监督全部 ROS 节点。它可以统一处理退出和重启，但会隐藏日志、改变当前使用方式，也超出本次“放入脚本文件夹并完整启动”的需求。

本次采用方案 A。

## 详细设计

### 1. 雷达定位脚本

修改 `scripts/ros_start_lidar_only.sh` 中的 Fast-LIO 命令：

```bash
roslaunch fast_lio mapping_mid360.launch rviz:=false
```

其余 Livox、MAVROS 和 `fastlio_fusion` 启动命令保持不变。这样该脚本仍启动四个链路标签页，但不会启动 Fast-LIO 的 RViz。

### 2. 完整启动脚本

文件名：`scripts/ros_start_ldop_detection.sh`

执行流程：

```text
检查 gnome-terminal、ros_start_lidar_only.sh 和 LDOP 环境脚本
        |
调用 ros_start_lidar_only.sh
        |
打开 LDOP 标签页
        |
加载 /home/oem/ldop_ws/ldop_env.sh
        |
等待 /cloud_registered 有实际消息
        |
等待 /Odometry 有实际消息
        |
执行 roslaunch ldop run_ldop.launch
```

LDOP 标签页只启动 `run_ldop.launch`，由该 launch 文件负责 LDOP 检测节点和 LDOP RViz。脚本不再启动第二个雷达 RViz。

等待逻辑使用 ROS 话题实际消息作为条件，而不是只依赖固定秒数：

- `/cloud_registered`：点云输入，供 LDOP 聚类与跟踪。
- `/Odometry`：定位里程计，供 LDOP 坐标变换/融合流程使用。

等待期间在 LDOP 标签页显示当前状态；如果话题尚未出现或没有消息，则周期性重试。用户可以在该标签页用 `Ctrl+C` 终止等待或启动的 LDOP launch。

### 3. 错误处理与停止方式

- 必要脚本或环境文件不存在时，打印明确错误并不启动 LDOP。
- `gnome-terminal` 不可用时，打印安装/运行环境错误并退出。
- 不在启动脚本中执行 `catkin build` 或安装操作。
- 继续使用 `/home/oem/scripts/ros_stop_all.sh` 停止 ROS 进程；该脚本的现有行为不变。
- 启动脚本使用固定的工作空间和脚本绝对路径，避免从不同当前目录执行时找不到文件。

## 文件与同步关系

本地仓库：`E:/比赛功能包/LDOP`

- `scripts/ros_start_lidar_only.sh`
- `scripts/ros_start_ldop_detection.sh`
- `docs/superpowers/specs/2026-08-03-ldop-unified-startup-script-design.md`

远程端：`oem@192.168.31.163`

- `/home/oem/scripts/ros_start_lidar_only.sh`
- `/home/oem/scripts/ros_start_ldop_detection.sh`
- `/home/oem/ldop_ws/src/LDOP/docs/superpowers/specs/2026-08-03-ldop-unified-startup-script-design.md`

实现完成后分别计算本地和远程文件校验和，并检查执行权限，确保两端内容一致。之后在本地仓库提交一次 Git 记录；远程源码与脚本文件保持与本地对应版本同步。

## 验证计划

1. Shell 语法检查：`bash -n` 检查两个脚本。
2. 检查两个脚本均具有执行权限。
3. 检查 Fast-LIO 启动命令包含 `rviz:=false`。
4. 在远程端运行完整脚本，确认出现四个雷达定位标签页和一个 LDOP 标签页。
5. 确认雷达定位运行时没有 Fast-LIO RViz，LDOP 标签页最终启动 `ldop_node` 与 LDOP RViz。
6. 检查 `/cloud_registered`、`/Odometry`、`/ldop/dynamic_object_markers` 等关键话题。
7. 比较本地与远程脚本的 SHA-256；运行 LDOP 既有测试，确认没有回归。


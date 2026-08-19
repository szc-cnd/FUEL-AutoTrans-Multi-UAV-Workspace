# UAV1 代码同步与部署流程

本文用于把当前主机（UAV0 电脑）上的双机搜索降落代码同步到 UAV1 电脑。两台电脑应使用同一份 Git 仓库、同一分支和相同的 ROS 发行版（当前为 Ubuntu/ROS Noetic）。

## 1. 同步前确认

在当前主机检查本次要同步的提交：

```bash
cd ~/match_ws
git status --short
git log -1 --oneline
```

工作区中与本次功能有关的提交示例：

```text
950712a 双机复用统一搜索降落相机外参
```

如果 `git status` 中有未提交的代码修改，先确认是否需要提交；不要把临时编译产物、日志或实验文件同步到 UAV1。

## 2. 配置 UAV1 的 SSH 访问

将下面的地址替换为 UAV1 电脑的实际用户名和 IP：

```bash
ssh <uav1_user>@<uav1_ip>
```

首次连接时确认主机指纹。登录后确认工作空间存在：

```bash
test -d ~/match_ws/src/precision_landing
rosversion -d
```

如果 UAV1 尚未配置仓库，应先在 UAV1 上克隆同一个仓库，并切换到与当前主机相同的分支。

## 3. 同步代码

推荐使用 Git 同步，不直接覆盖整个工作空间：

```bash
ssh <uav1_user>@<uav1_ip> '
  cd ~/match_ws &&
  git fetch --all --prune &&
  git checkout vehicle/cav0 &&
  git pull --ff-only
'
```

如果 UAV1 上存在未提交修改，`git pull --ff-only` 会停止，不会覆盖这些修改。先在 UAV1 上查看：

```bash
ssh <uav1_user>@<uav1_ip> 'cd ~/match_ws && git status --short'
```

同步后核对两台电脑的提交号：

```bash
git -C ~/match_ws rev-parse HEAD
ssh <uav1_user>@<uav1_ip> 'cd ~/match_ws && git rev-parse HEAD'
```

两行输出应一致。

## 4. 编译 UAV1 工作空间

```bash
ssh <uav1_user>@<uav1_ip> '
  cd ~/match_ws &&
  source /opt/ros/noetic/setup.bash &&
  catkin_make -DCATKIN_WHITELIST_PACKAGES='precision_landing;exploration_control' -j2 &&
  source devel/setup.bash
'
```

如果 UAV1 需要完整工程，也可以执行 `catkin_make -j2`。编译失败时先保留终端输出，不要删除 `build` 或 `devel` 目录。

## 5. 检查 UAV1 的相机和外参

当前搜索降落节点使用的共享文件是：

```text
~/match_ws/src/precision_landing/config/landing_search.yaml
```

UAV0 和 UAV1 应使用相同的数值外参。检查文件是否存在：

```bash
ssh <uav1_user>@<uav1_ip> \
  'grep -A5 camera_to_body ~/match_ws/src/precision_landing/config/landing_search.yaml'
```

如果 UAV1 后续也启用前视 D435，前视相机静态 TF 使用的标定文件默认是：

```text
~/handeye_calibration/body_camera_03.yaml
```

如果该文件不存在，先从已完成标定的电脑复制，再检查内容，不要用空文件替代：

```bash
scp ~/handeye_calibration/body_camera_03.yaml \
  <uav1_user>@<uav1_ip>:~/handeye_calibration/body_camera_03.yaml
```

下视搜索的数值外参来自 `landing_search.yaml`，最终精降控制的同组外参来自
`precision_landing.yaml`；两者存储格式不同，但应保持为同一组标定结果。UAV1 的相机
设备名、相机信息 URL 和下视图像话题可能与 UAV0 不同，只修改 UAV1 入口的设备参数，
不修改共享外参数值。

## 6. 启动前检查

在 UAV1 电脑上确认下视相机和 FAST-LIO 话题：

```bash
rostopic list | grep -E 'UAV1/(down_camera|fast_lio)'
rostopic hz /UAV1/fast_lio/Odometry
```

确认下视图像、相机内参和里程计均有数据后，再启动双机入口。无桨测试时先使用隔离输出配置，不要直接接通飞控控制输出。

## 7. 启动后验证

```bash
rosnode list | grep -E 'UAV0|UAV1|landing'
rostopic echo -n1 /UAV1/landing/search/status
rostopic echo -n1 /UAV1/landing/search/candidates
rosparam get /UAV1/landing_search_node/camera_to_body
```

应能看到 UAV1 的搜索节点、精降节点和独立命名空间。UAV1 搜索节点仍使用同一套算法和 `landing_search.yaml`，只将输入输出切换到 `/UAV1/...` 话题。

双机协同时还应检查：

```bash
rostopic echo -n1 /dual_uav_landing/assignments_ready
rostopic echo -n1 /UAV1/landing/assigned_id
rostopic echo -n1 /dual_uav_landing/release_uav1
```

## 8. 版本回退

如果同步后发现问题，在 UAV1 上回退到已验证提交：

```bash
ssh <uav1_user>@<uav1_ip> \
  'cd ~/match_ws && git checkout <已验证提交号> && catkin_make -j2'
```

回退前先保存 `git status --short` 和终端日志。任何新的代码修改都必须在对应电脑上提交 Git，并记录提交号，避免两台电脑再次出现代码版本不一致。

## 9. 现场最小检查表

- 两台电脑 `git rev-parse HEAD` 一致
- 两台电脑均已 `source /opt/ros/noetic/setup.bash` 和 `source ~/match_ws/devel/setup.bash`
- UAV1 下视图像、相机内参、FAST-LIO 里程计均有数据
- `landing_search.yaml` 的相机到机体外参一致
- `body_camera_03.yaml` 存在且内容为已确认的标定结果
- `/UAV0`、`/UAV1` 话题命名空间没有串线
- 无桨检查通过后再进行实际飞行

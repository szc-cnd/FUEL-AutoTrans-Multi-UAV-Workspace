# UAV1 代码同步与部署流程

本文用于把当前主机（UAV0 电脑）上“搜索降落功能及其引起的关联修改”同步到
UAV1 电脑，而不是覆盖 UAV1 的整个工程。两台电脑应使用同一份 Git 仓库和相同的
ROS 发行版（当前为 Ubuntu/ROS Noetic）。

## 1. 同步范围

必须同步以下功能闭包：

- `src/precision_landing/`：搜索、平台识别、候选平台发布、双机平台分配、精确降落、消息和配置
- `src/control/`：UAV1 接力、等待/释放逻辑、分配目标接收、双机坐标转换和安全跟随
- `src/Diff-Planner`：UAV1 接收分配平台后生成搜索/接近轨迹所依赖的子模块版本
- 上述目录在 `CMakeLists.txt`、`package.xml`、`msg/`、`launch/`、`config/`、`src/`、`include/` 和 `test/` 中的关联修改
- UAV1 运行所需但不进入 Git 的相机标定文件和本机设备参数

其中磁盘目录 `src/control/` 的 ROS 包名是 `exploration_control`。编译命令应使用包名，
同步文件时应使用目录名。

`src/uav0_competition_bringup/launch/` 中也有因双机搜索降落产生的协调入口修改。这部分
必须保留在共同 Git 历史并参与审计，但它属于 UAV0 主控入口，不能在 UAV1 电脑上启动。

默认不覆盖以下内容：

- UAV1 的 PX4、MAVROS 和飞控参数
- UAV1 的 FAST-LIO 本地配置及修改
- UAV1 的相机设备规则、网卡、主机名和系统服务
- 与搜索降落无关的规划器、检测器、日志和编译产物
- UAV1 工作区中尚未确认的未提交修改

## 2. 同步前确认

在当前主机检查本次要同步的提交：

```bash
cd ~/match_ws
git status --short
git log -1 --oneline
```

当前搜索降落功能的基础提交为：

```text
0cdcb4e 统一双机坐标对齐与航点转换参数
799c10e 支持运行时分配降落平台ID
ab1d378 协调双机选择不同降落平台
ffce686 实现前机双平台搜索与后机顺序释放
950712a 双机复用统一搜索降落相机外参
```

后续因为搜索降落新增的提交也必须加入同步范围。执行同步前先用以下命令核对相关
目录从 UAV1 当前版本到 UAV0 目标版本之间的实际变化：

```bash
git diff --name-status <UAV1当前提交>..<UAV0目标提交> -- \
  src/precision_landing src/control src/Diff-Planner \
  src/uav0_competition_bringup/launch
```

如果 `git status` 中有未提交的代码修改，先确认是否需要提交；不要把临时编译产物、
日志或实验文件同步到 UAV1。

## 3. 配置 UAV1 的 SSH 访问

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

## 4. 同步搜索降落功能闭包

先读取 UAV1 状态，任何写入前都必须执行：

```bash
ssh <uav1_user>@<uav1_ip> \
  'cd ~/match_ws && git branch --show-current && git status --short && git rev-parse HEAD'
```

若 UAV1 与 UAV0 使用同一分支，且从 UAV1 当前提交到目标提交之间没有无关改动，可使用
快进同步：

```bash
ssh <uav1_user>@<uav1_ip> '
  cd ~/match_ws &&
  git fetch --all --prune &&
  git checkout vehicle/cav0 &&
  git pull --ff-only
'
```

若目标分支同时包含 UAV1 不应接收的无关修改，则只挑选搜索降落相关提交，不直接
`git pull`。当前基础提交按顺序执行：

```bash
ssh <uav1_user>@<uav1_ip> '
  cd ~/match_ws &&
  git fetch --all --prune &&
  git cherry-pick 0cdcb4e 799c10e ab1d378 ffce686 950712a
'
```

如果某个提交已经存在，先用 `git log --oneline` 确认，不重复挑选。发生冲突时停止并
检查冲突文件，不执行 `git reset --hard`，也不覆盖 UAV1 的本地版本。

同步完成后按内容校验，不强制要求两台电脑的仓库 HEAD 完全一致，因为 UAV1 可以保留
自己的专用提交：

```bash
git -C ~/match_ws rev-parse HEAD
ssh <uav1_user>@<uav1_ip> \
  'cd ~/match_ws && git log --oneline -10 && git status --short'

git -C ~/match_ws ls-tree -r HEAD \
  src/precision_landing src/control src/Diff-Planner | sha256sum
ssh <uav1_user>@<uav1_ip> \
  'cd ~/match_ws && git ls-tree -r HEAD src/precision_landing src/control src/Diff-Planner | sha256sum'
```

若 UAV1 有专用修改导致目录哈希不同，应逐项查看差异，确认差异仅属于 UAV1 的设备或
部署配置。同步完成后在 UAV1 上创建一个明确提交，提交信息记录 UAV0 的来源提交号；
不要留下未提交的搜索降落代码。

如果提交更新了 `src/Diff-Planner` 的子模块指针，还必须在 UAV1 上同步到记录的版本：

```bash
ssh <uav1_user>@<uav1_ip> '
  cd ~/match_ws &&
  git submodule update --init --recursive src/Diff-Planner &&
  git -C src/Diff-Planner status --short &&
  git submodule status src/Diff-Planner
'
```

若子模块中存在 UAV1 本地修改，停止同步并先审查，不强制切换子模块版本。

## 5. 编译 UAV1 搜索降落相关包

```bash
ssh <uav1_user>@<uav1_ip> '
  cd ~/match_ws &&
  source /opt/ros/noetic/setup.bash &&
  catkin_make -DCATKIN_WHITELIST_PACKAGES='precision_landing;exploration_control' -j2 &&
  source devel/setup.bash &&
  cd ~/match_ws/src/Diff-Planner &&
  catkin_make -j2 &&
  source devel/setup.bash
'
```

这里的 `exploration_control` 对应磁盘目录 `src/control`。如果消息接口变化导致其他包
依赖失败，应根据编译输出补充相关依赖包，不得仅复制可执行文件。`Diff-Planner` 是独立
Catkin 工作空间，需要在其目录内单独编译；启动终端也必须依次加载主工作空间和
`~/match_ws/src/Diff-Planner/devel/setup.bash`。编译失败时先保留终端输出，不要删除
`build` 或 `devel` 目录。

## 6. 检查 UAV1 的相机和外参

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

## 7. 启动前检查

在 UAV1 电脑上确认下视相机和 FAST-LIO 话题：

```bash
rostopic list | grep -E 'UAV1/(down_camera|fast_lio)'
rostopic hz /UAV1/fast_lio/Odometry
```

确认下视图像、相机内参和里程计均有数据后，再启动双机入口。无桨测试时先使用隔离输出配置，不要直接接通飞控控制输出。

## 8. 启动后验证

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

## 9. 版本回退

如果同步后发现问题，在 UAV1 上回退到已验证提交：

```bash
ssh <uav1_user>@<uav1_ip> \
  'cd ~/match_ws && git checkout <已验证提交号> && catkin_make -j2'
```

回退前先保存 `git status --short` 和终端日志。任何新的代码修改都必须在对应电脑上提交 Git，并记录提交号，避免两台电脑再次出现代码版本不一致。

## 10. 现场最小检查表

- UAV1 已包含全部搜索降落基础提交和后续相关提交
- `src/precision_landing`、`src/control` 的差异均已逐项确认
- `src/Diff-Planner` 子模块处于主仓库记录的提交，且没有未审查修改
- UAV0 协调入口只在 UAV0 启动，UAV1 不重复启动协调器
- UAV1 没有未提交的搜索降落代码
- 两台电脑均已 `source /opt/ros/noetic/setup.bash` 和 `source ~/match_ws/devel/setup.bash`
- UAV1 已加载 `source ~/match_ws/src/Diff-Planner/devel/setup.bash`
- UAV1 下视图像、相机内参、FAST-LIO 里程计均有数据
- `landing_search.yaml` 的相机到机体外参一致
- `body_camera_03.yaml` 存在且内容为已确认的标定结果
- `/UAV0`、`/UAV1` 话题命名空间没有串线
- 无桨检查通过后再进行实际飞行

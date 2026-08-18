# CAV0 控制器与 FAST-LIO 同步改造计划

> 目标分支：`vehicle/cav0`<br>
> 执行仓库：`D:\Study\codex project\UAV\FUEL-AutoTrans-Multi-UAV-Workspace-cav0`<br>
> 参照实现：当前 `vehicle/cav1` 的 AutoTrans、FAST-LIO 高频里程计和六分屏启动流程<br>
> 原则：核心控制行为与 CAV1 对齐，保留 FUEL 接口，CAV0 实机参数不改，不因 MPC 求解失败自动降落。

## 1. 先确认的现状

CAV0 目前不能直接覆盖 CAV1 的控制器目录。两边的模式入口、状态估计、求解失败处理和轨迹接口都有差异；直接整目录替换会丢失 FUEL 的 `PositionCommand` 入口、显式 yaw 多项式和 `bspline -> PolynomialTraj` bridge。

### 1.1 模式逻辑差异

| 功能 | CAV0 当前行为 | 需要达到的 CAV1 核心行为 |
|---|---|---|
| MANUAL_CTRL 进入方式 | 主要只处理 CH8 低位起飞 | CH8 低位起飞；中位进入 AUTO_HOVER；高位进入 CMD_CTRL，并检查 PX4/Odom/速度前置条件 |
| 起飞预流 | 周期性发布约 0.53 的悬停推力 | 按 CAV1 语义，在 OFFBOARD 前只发送低值占位 setpoint，避免残留控制量 |
| Odom 有效性 | 仍使用 0.5 s 新鲜度作为总入口条件 | 按 CAV1 逻辑区分“曾收到”和“当前新鲜度”；新鲜度只用于对应安全门控，不让一次短暂延迟直接触发错误状态跳转 |
| NMPC 状态姿态 | 全部取 FAST-LIO 四元数 | 平移、速度取 FAST-LIO 高频 Odom；控制姿态/外力姿态取 MAVROS `local_position/odom` |
| MPC 失败 | 记录错误后仍读取求解器缓冲并发布 | 校验状态和有限值；短时保持最近一次有效输出；锁存当前位置悬停并后台重试；不自动降落 |
| 轨迹接收 | FUEL 轨迹入口逻辑 | 保留 FUEL 入口，但增加时间戳、ID、有限值和恢复期间接收门控 |
| AUTO_LAND | 可能未验证本周期 MPC 成功就请求降落 | 只允许人工/任务降落流程；请求前验证当前控制输出和 PX4 状态，MPC 失败本身不能触发降落 |

结论：不是追求文件逐字相同，而是让主 FSM、安全门控和求解失败处理与 CAV1 一致，同时保留 FUEL 必需的适配层。

## 2. 已确定的接口和参数决策

### 2.1 高频状态话题

所有 **CAV0 运行链路** 的位置、速度消费者统一改为：

```text
/UAV0/fast_lio/Odom_high_freq
```

以下接口保持不变：

```text
/UAV0/fast_lio/Odometry                 # FAST-LIO 继续并行发布，供诊断/回滚
/UAV0/fast_lio/cloud_registered         # 点云，没有“高频点云”替代品
/UAV0/mavros/local_position/odom        # 外力估计和控制姿态
/UAV0/mavros/imu/data                    # 外力估计 IMU，继续使用 MAVROS
```

“全仓消费者”指 CAV0 的 AutoTrans、logger、simple controller、FUEL、相机位姿桥、leader/follower 中属于 UAV0 的输入、launch 默认值和文档。历史归档实验中明确属于 VINS/UKF 的通用默认值不做无依据的批量替换；正在使用的 CAV0 启动入口必须全部走高频话题。若同一工作区同时运行 UAV1，UAV1 使用自己的已验证话题，不把 UAV0 的参数硬编码到 UAV1。

### 2.2 FAST-LIO 与 Livox 源码

CAV0 的 `FAST_LIO`、`livox_ros_driver2`、`Livox-SDK2` 当前是 gitlink，目标对象无法从现有公开远端可靠恢复。按本计划：

1. 在 Windows CAV0 仓库把 CAV0 机器上实际使用的三套源码和配置导入为可追踪源码，去除不可构建的嵌套 gitlink 状态。
2. 以 CAV0 的网络地址、雷达外参、frame、频率和标定文件为基线。
3. 只移植 CAV1 的高频积分、IMU 坐标转换节点、话题参数化和安装规则。
4. 不从 CAV1 复制 `MID360_config.json`、`mid360.yaml` 的物理参数覆盖 CAV0；`imu_install_pitch_deg` 只保留一个 launch 参数入口，值由 CAV0 已有标定填写。此次不以“可能双重旋转”作为阻断条件，但禁止同一旋转在两个节点重复实现。
5. 普通 `Odometry` 必须继续发布，便于回滚和对比；高频 Odom 是新增并行输出，不替代普通输出的发布。

### 2.3 CAV0 实机参数

完整保留 CAV0 当前 `config/mpc.yaml`、`config/model.yaml` 的实机数值，包括质量、重力、NMPC 步长、权重、推力上下限、固定悬停点、外力估计和补偿参数。不得把 CAV1 的质量、推力系数、补偿开关或限幅值带入 CAV0。

允许新增的只有：

- 高频 Odom 话题参数；
- CAV1 安全恢复所需的阈值/计数/超时参数；
- FUEL 停止/恢复话题参数；
- logger/evo 的输入话题参数。

新增参数必须有中文注释，说明是否参与控制逻辑以及单位。

## 3. 分阶段实施

### 阶段 A：建立可构建的传感器源码基线

涉及目录：

- `src/FAST_LIO/`
- `src/livox_ros_driver2/`
- `src/Livox-SDK2/`

具体工作：

1. 从 CAV0 Windows 工作区现有源码导入完整文件，保留许可证和原始配置。
2. 合并 CAV1 的 `mapping_mid360.launch` 参数：`vehicle_ns`、原始 IMU、机体系 IMU、`odom_topic`、`high_freq_odom_topic`、`imu_mps2_topic`。
3. 移植 `scripts/livox_imu_to_body.py`，默认命名空间改为 UAV0；旋转角速度、线加速度和协方差时只允许一个转换节点生效。
4. 移植 `IMU_Processing.hpp` 的平均加速度模长接口，以及 `laserMapping.cpp` 的最新 IMU 预测积分和高频 Odom 发布。高频输出必须带有效 header、frame 和 twist，且与普通 Odom 使用同一世界坐标语义。
5. 保留 `livox_ros_driver2` 的消息生成依赖和 `add_dependencies`；不得照抄 CAV1 中删除这些依赖的破坏性改动。
6. CMake 安装 Python 节点和 launch；清理所有对不存在文件的引用。
7. `Livox-SDK2` 仅作为可追踪依赖导入，不改算法。

阶段 A 的完成条件：

- `roslaunch fast_lio mapping_mid360.launch vehicle_ns:=UAV0` 能启动；
- 同时看到 `/UAV0/fast_lio/Odometry` 和 `/UAV0/fast_lio/Odom_high_freq`；
- 高频 Odom 频率、时间戳单调性、frame 和位置连续性通过脚本检查；
- 普通 Odom 仍可被手动切回，且没有重复发布同一 IMU 旋转。

### 阶段 B：全链路改成高频 Odom

逐个修改实际 CAV0 启动入口和参数默认值，不做全仓无差别字符串替换。至少核对：

- `payload_mpc_controller/launch/quad_wind_mpc_controller.launch`
- `autotrans_reference_bridge/launch/uav0_fuel_controller.launch`
- `autotrans_reference_bridge/launch/uav0_fuel_planner.launch`
- FUEL 的 `run_swarm_indoor1_fuel_exploration.launch` 及其 include 链
- `exploration_control/cxr_egoctrl_v1.cpp` 的 `odom_topic`
- CAV0 leader/follower、相机位姿桥、logger、目标上报和比赛启动脚本
- 所有 CAV0 实际使用的 RViz、记录和诊断 launch

统一规则：

```text
AutoTrans 平移/速度       -> /UAV0/fast_lio/Odom_high_freq
FUEL 规划器 odometry      -> /UAV0/fast_lio/Odom_high_freq
simple controller         -> /UAV0/fast_lio/Odom_high_freq
视觉位姿回传              -> 由高频 Odom 生成
外力姿态                  -> /UAV0/mavros/local_position/odom
外力 IMU                  -> /UAV0/mavros/imu/data
```

每个 launch 都要把话题作为 arg 传递，禁止在节点内部再次写死旧 `/UAV0/fast_lio/Odometry`。启动时用 `roslaunch --args` 和 `rosparam get` 检查最终解析值。

### 阶段 C：合并 CAV1 控制器核心，保留 FUEL 扩展

涉及 `payload_mpc_controller` 的 `mpc_fsm.*`、`mpc_controller.*`、`mpc_input.*`、`mpc_wrapper.*`、节点入口和测试。

#### CAV1 核心逻辑必须移植

1. MANUAL_CTRL 的 CH8 三段入口、OFFBOARD/连接状态/Odom/速度前置检查和中文原因日志。
2. CAV1 的低值预流、手动模式 setpoint 清理和退出 OFFBOARD 后的安全处理。
3. `setEstimateState(translation_odom, force_attitude_odom)`：位置/速度取高频 FAST-LIO，姿态取 MAVROS 融合 Odom；四元数顺序和 frame 明确校验。
4. MPC wrapper/controller 必须消费求解返回值：失败时不读取未经验证的 ACADO 缓冲；对控制输入做 `isfinite`、范围和维度检查。
5. 保留最近一次有效 body-rate/thrust，最多短时保持（使用 CAV1 当前参数）；超过保持窗口进入 `MPC_RECOVERY_HOVER`。
6. 恢复流程：锁存当前高频位置和 MAVROS 航向、清零外力补偿、阻止新轨迹、重置 hover 参考、后台重新准备/求解；成功若干周期后才解除恢复状态。
7. 恢复超时只继续锁存悬停并告警，绝不因为求解失败请求 PX4 `AUTO.LAND`。
8. 仅在人工或比赛规定降落流程中允许 `AUTO_LAND`，并保留 CAV1 的 PX4 状态确认和请求重试门控。
9. 轨迹接收增加时间戳、trajectory ID、系数有限值、piece duration 和起点连续性校验；恢复期间拒收旧轨迹。

#### 必须保留的 FUEL 扩展

1. `/UAV0/planning/pos_cmd` `PositionCommand` 入口和入口目标限速器。
2. `PolynomialTraj.has_yaw`、`yaw_trajectory[]` 解析和 yaw 多项式优先级。
3. `fuel_autotrans_bridge_node` 的 B-spline 输入、UAV0 frame、FUEL 命名空间和 CAV0 参数。
4. simple/autotrans 互斥启动方式，不能同时启动两套控制器。

不要把 CAV1 已删除的 FUEL API 反向删掉；采用“CAV1 FSM 外壳 + CAV0 FUEL 输入适配器”的合并方式。

### 阶段 D：定义 FUEL 与 MPC 恢复握手

CAV1 的 `/planning_stop_trigger`、`/planning_restart_trigger` 没有 FUEL 消费者，不能只复制发布器。CAV0 采用以下 UAV0 命名接口（并保留 CAV1 的 Empty 消息语义）：

```text
/UAV0/planning_stop_trigger               std_msgs/Empty
/UAV0/planning_restart_trigger            std_msgs/Empty
/UAV0/planning/safety_hold                std_msgs/Bool（FUEL -> 控制器）
```

具体语义：

1. `planning_stop_trigger`：只在明确的人工/任务降落或终止接管时发布一次。FUEL `traj_server` 收到后停止继续发布旧 `PositionCommand`，清除当前交接状态；MPC 单次或连续求解失败进入 `MPC_RECOVERY_HOVER` 时不发布该话题。
2. MPC 恢复期间由控制器本地锁存位置、阻断 `PolynomialTraj` 和 `PositionCommand` 的新轨迹消费；FUEL 可以继续运行自己的规划 FSM，但其输出不能绕过控制器的恢复门控。
3. `planning_restart_trigger`：MPC 连续成功恢复后发布一次。新增 FUEL 探索 FSM subscriber；回调仅在已触发任务、已有高频 Odom 且未处于 `FINISH/landing` 时，清零重试计时，设置 `static_state=true`，切到 `PLAN_TRAJ`。现有 `PLAN_TRAJ` 将从最新高频 Odom 重新规划并发布 B-spline，控制器只接受该时刻之后的新轨迹。
4. `planning/safety_hold` 的方向保持为 FUEL -> 控制器，继续用于规划器碰撞/规划失败的门控；控制器不得把它当作反向恢复通知。若需要记录控制器恢复状态，新增只读日志字段或单独的 `autotrans_recovery_state` 话题。
5. 新接口通过 FUEL 和 AutoTrans launch 的 arg/remap 配置，禁止使用全局无 namespace 话题。
6. 保留 FUEL 原有 `/UAV0/planning/replan` 用于规划器自己的正常重规划；不要把它替代 `planning_restart_trigger`，因为它只会让 `traj_server` 截断当前轨迹，不能驱动探索 FSM 重新选点。

同时修正 `fuel_autotrans_bridge_node.cpp`：

- 输出 publisher 不使用 latched；
- 删除“0.5 s 没有 B-spline 就自动 ACTION_ABORT”的逻辑；
- 只有收到明确的无效输入或显式 abort 时才发布 abort；
- 按 `PolynomialTraj`/`Eigen::Map` 约定写入降幂系数，补充位置与 yaw piece 数量、时长和 finite 校验；
- 保留 `has_yaw` 和 `yaw_trajectory`。

在 `quadrotor_msgs` 中确认 `PolynomialMatrix.msg`/`PolynomialTraj.msg` 字段存在；若缺失，只修改 `.msg` 和生成依赖后重新构建，不手工提交生成的 ROS 代码。

### 阶段 E：六分屏和中文运行提示

新增或改造 CAV0 的：

- `shfiles/start_uav0_six_terminator.sh`
- `shfiles/terminator_uav0_six.conf`

结构镜像 CAV1 的 `start_uav1_six_terminator.sh`，但替换为 UAV0/FUEL：

1. UAV0 MAVROS（连接状态、IMU/姿态/里程计/ESC 频率）；
2. UAV0 MID360；
3. UAV0 FAST-LIO；
4. 高频 Odom 到 `/UAV0/mavros/vision_pose/pose`；
5. FUEL Planner/RViz；
6. FUEL bridge、AutoTrans、logger、自动 rosbag。

窗口尺寸、六分屏比例、标题格式、等待超时和中文提示统一采用 CAV1 规则：`[等待]`、`[就绪]`、`[启动]`、`[退出]`、`[错误]`。脚本不自动解锁、不切换 OFFBOARD、不发布目标点。

启动等待至少包括：ROS master、UAV0 MAVROS connected、Livox lidar/IMU、`/UAV0/fast_lio/Odom_high_freq`、点云、FUEL heartbeat、视觉位姿稳定时间。第 6 屏不得重复启动第 5 屏的 FUEL planner。

### 阶段 F：logger、Evo 和文档同步

1. logger 的 `odom_topic` 默认改为高频 Odom，继续记录 MAVROS 姿态、MAVROS IMU、FUEL 原始 B-spline、`PositionCommand` 和 AutoTrans PolynomialTraj。
2. 移植 CAV1 的自动 rosbag、`enable_evo_report`、`summary.md` 和中文统计摘要；报告中的实际轨迹使用高频 Odom，参考轨迹使用桥接后的有效轨迹。
3. 记录恢复状态、求解失败码、最近有效输出年龄、stop/restart 事件和最终降落原因，便于区分“求解失败悬停”和“任务降落”。
4. 更新 CAV0 的比赛说明、启动命令、topic 表和故障排查文档，所有运行示例使用 `/UAV0/fast_lio/Odom_high_freq`。
5. 不把 bag、图片、视频、PDF、build/devel、`.orig` 或 Evo 结果放进 Git。

## 4. 模式验收矩阵

| 场景 | 预期状态 | 预期输出/提示 |
|---|---|---|
| CH8 低位、未连接 PX4 | MANUAL_CTRL | `[等待]`/前置条件原因，不发送有效控制 |
| CH8 低位、OFFBOARD 且数据有效 | AUTO_TAKEOFF | 使用 CAV0 起飞高度/速度参数，平滑爬升 |
| CH8 中位 | AUTO_HOVER | 锁存当前位置和航向，持续有效 setpoint |
| CH8 高位 | CMD_CTRL | 接受有效 FUEL PolynomialTraj 或 PositionCommand |
| 轨迹恢复期间收到旧轨迹 | 保持 MPC_RECOVERY_HOVER | 拒收并记录 trajectory ID/时间原因 |
| 单次求解失败 | 当前模式不变 | 短时保持最近有效有限输出，记录错误码 |
| 连续求解失败 | MPC_RECOVERY_HOVER | 锁存位置、清零补偿、发 stop；不 AUTO.LAND |
| MPC 恢复成功 | HOVER/CMD_CTRL 等待新轨迹 | 发 restart，FUEL 从高频 Odom 重新规划 |
| 任务/人工降落 | AUTO_LAND | 仅在合法降落流程和 PX4 状态确认后请求降落 |
| OFFBOARD 丢失或状态估计失效 | MANUAL/受控保护 | 停止有效控制输出并给出中文原因，不伪造轨迹 |

## 5. 测试与验证

### 静态检查

```bash
python3 -m py_compile src/FAST_LIO/scripts/livox_imu_to_body.py
python3 -m py_compile src/AutoTrans-quadrotor-wind-mpc/controller/payload_mpc_controller/scripts/*.py
git diff --check
```

补充测试：

- FAST-LIO 高频 Odom 发布、时间戳、frame、IMU 旋转和协方差；
- 所有 CAV0 launch 的最终 Odom 参数；
- CAV0 三段 CH8 模式入口和 AUTO_LAND 授权；
- MPC 非零返回、NaN/Inf、短时保持、恢复成功和恢复超时；
- FUEL stop/restart 回调、旧 `PositionCommand` 停止、新 B-spline 重新发布；
- bridge 非 latched、无超时 abort、系数顺序和 yaw 字段；
- 六分屏脚本的中文提示、六个 pane、等待话题和 UAV0 命名空间；
- CAV0 YAML 参数值与改造前快照逐项相等。

### 构建和台架验证

1. 在 Windows 仓库完成 lint、Python/XML 解析和单元测试。
2. 中文提交并推送 `origin/vehicle/cav0`。
3. CAV0 Linux 工作区只执行快进拉取，再用统一 `catkin_make` 构建消息、Livox、FAST-LIO、FUEL、bridge 和 AutoTrans。
4. 静态检查通过后再做无桨台架：验证六分屏启动、话题频率、模式切换、模拟 MPC 失败和 stop/restart 握手。
5. 最后才进行实机低高度验证；首先验证悬停和规定降落点流程，再验证规划轨迹。

## 6. 提交与回滚规则

- 所有修改只在 Windows CAV0 仓库完成；不得直接在 `/home/asus/match_ws` 修改。
- 分组提交，建议中文提交：`移植CAV1高频里程计链路`、`同步CAV0控制器安全状态机`、`保留FUEL接口并接入恢复握手`、`统一CAV0六分屏与日志`。
- 每次提交前显式检查 `git diff`、`git diff --check`、暂存文件列表和未跟踪文件；禁止 `git add .`。
- 推送前 `git fetch origin vehicle/cav0`，只有远端仍是本地祖先时才执行普通 `git push origin vehicle/cav0`；禁止强制推送。
- 若构建失败，先按阶段回滚到“普通 Odom + 原 FUEL bridge”基线，再逐项启用高频输出、控制器合并和恢复握手；不得删除 CAV0 实机参数或生成文件来掩盖错误。

## 7. 完成定义

只有同时满足以下条件，才可认为 CAV0 已与 CAV1 核心同步：

1. CAV0 三段 CH8 模式入口、状态估计、求解失败恢复和合法降落门控与 CAV1 行为一致；
2. FUEL 的 B-spline、PositionCommand、yaw 和 bridge 接口仍可用，且恢复时不会继续发布旧轨迹；
3. CAV0 所有活动位置/速度消费者使用高频 Odom，普通 Odom 仍并行可回滚；
4. MAVROS 融合姿态和 MAVROS IMU 的来源保持明确，外力补偿不因换频率而改变物理参数；
5. 六分屏布局、等待逻辑和中文运行提示与 CAV1 同构；
6. logger/Evo 能生成包含高频实际轨迹、FUEL 参考轨迹和 MPC 恢复事件的报告；
7. 静态测试、消息生成、catkin 构建和无桨台架验证全部通过。

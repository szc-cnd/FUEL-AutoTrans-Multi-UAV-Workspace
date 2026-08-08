# UAV0 FUEL 接入 AutoTrans 控制器实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 CAV0 的 `~/match_ws` 中同步 CAV1 已验证的 AutoTrans 控制器和实施设计，将 UAV0 的 FUEL B-spline 规划轨迹桥接为 AutoTrans 可接收的分段多项式轨迹；保留原 `px4ctrl/cxr_egoctrl`，通过 `simple/autotrans` 互斥入口切换控制器。

**Architecture:** FUEL 和 AutoTrans 统一使用 `/UAV0/fast_lio/Odometry` 雷达里程计。新增 `fuel_autotrans_bridge` 订阅 `/UAV0/planning/bspline`，将 B-spline 各有效 knot 区间转换为 `quadrotor_msgs/PolynomialTraj`，同时保留 FUEL yaw/yaw_rate；AutoTrans 订阅该轨迹和 UAV0 MAVROS 输入，仅发布 `/UAV0/mavros/setpoint_raw/attitude`。原控制器保持原源码和 launch 不变。

**Tech Stack:** ROS Noetic, catkin, C++14, Eigen, MAVROS/PX4, FUEL `bspline/Bspline`, `quadrotor_msgs/PolynomialTraj`。

---

## 1. 同步与基线保护

- [ ] 确认 CAV0 目标目录为 `/home/asus/match_ws`，不初始化根 Git，不覆盖已有嵌套仓库。
- [ ] 从 CAV1 同步 `AutoTrans-quadrotor-wind-mpc/controller/payload_mpc_controller`，不复制 build/devel/log。
- [ ] 从 CAV1 同步 `Diff-Planner替换AutoTrans控制器实施设计.md` 到 CAV0 `~/match_ws`。
- [ ] 记录同步前 CAV0 文件清单和目标路径，发现同名文件时先备份或停止，不静默覆盖。

## 2. 消息与依赖

- [ ] 在 CAV0 现有 `quadrotor_msgs` 包中新增 `PolynomialMatrix.msg` 和 `PolynomialTraj.msg`，保留旧 `PolynomialTrajectory.msg`。
- [ ] 更新 `quadrotor_msgs/CMakeLists.txt` 的 `add_message_files()`，保持旧消息继续生成。
- [ ] 确认 CAV0 `traj_utils/PolyTraj` 与 FUEL `bspline/Bspline` 的消息类型来自同一 ROS master、同一工作空间生成结果。
- [ ] 不把生成的 ROS message 代码手工复制到 source；由 catkin 生成。

## 3. FUEL 轨迹桥接

- [ ] 在 `autotrans_reference_bridge` 包中新增 FUEL 专用桥接节点和 launch。
- [ ] 输入默认为 `/UAV0/planning/bspline`，类型为 `bspline/Bspline`；输出默认为 `/UAV0/planning/autotrans_trajectory`，类型为 `quadrotor_msgs/PolynomialTraj`。
- [ ] 校验 `order`、`knots`、`pos_pts`、`yaw_pts`、`yaw_dt`、`start_time` 和每个 knot 区间的正时长。
- [ ] 使用 FUEL `NonUniformBspline::evaluateDeBoor()` 采样并在每个有效 knot 区间内重构幂基多项式；不得把控制点直接当成多项式系数。
- [ ] 位置 piece 使用 `num_dim=3` 和 Eigen 列主序 `[x,y,z]` 系数；yaw piece 使用 `num_dim=1`，系数为 `c0,c1,...`。
- [ ] 对 `yaw_pts` 做连续角度展开，避免跨越 `-pi/pi` 产生不连续 yaw。
- [ ] 保持 `traj_id`、`start_time`、piece 数量、duration 和 yaw/位置时间边界一致；不一致时拒绝整条轨迹并报警。
- [ ] 为桥接增加离线转换检查：采样比较 B-spline 与转换后多项式的位置、速度、加速度、yaw、yaw_rate 误差。
- [ ] 保留原 Diff-Planner `reference_bridge_node`，不能让 FUEL 和 Diff-Planner 共用错误的消息回调。

## 4. AutoTrans UAV0 适配

- [ ] 将 AutoTrans launch 的 `odom_topic` 改为 `/UAV0/fast_lio/Odometry`。
- [ ] 将 IMU、state、extended_state、RC、battery、ESC、service、setpoint 全部通过 launch 参数指向 `/UAV0/mavros/...`。
- [ ] 将轨迹输入指向 `/UAV0/planning/autotrans_trajectory`，目标/触发话题按现有 UAV0 约定配置。
- [ ] 确认控制器的 body-rate 是机体系角速度命令、单位 `rad/s`；MAVROS `thrust` 是归一化推力，不是牛顿力。
- [ ] 保留现有 CH8/CH10 状态机：CH8 低位请求 `AUTO_TAKEOFF`，中位 `AUTO_HOVER`，高位 `CMD_CTRL`；CH10 保持降落触发。
- [ ] 保留 FUEL 有效 yaw/yaw_rate；没有 yaw 扩展字段的旧轨迹仍回退到 `use_fix_yaw` 逻辑。
- [ ] 实现 PX4 退出 OFFBOARD 时同周期清空 trajectory queue、清零 applied disturbance、回到 `MANUAL_CTRL/HOVER`；重新进入不得恢复旧轨迹。
- [ ] 不修改 ACADO 状态、输入、OnlineData 维度，不重新生成 solver。

## 5. 互斥启动入口

- [ ] 新增 UAV0 集成启动入口，提供 `controller_mode=simple|autotrans`。
- [ ] 两种模式共同启动传感器/FUEL 规划部分；`simple` 只启动原 `px4ctrl/cxr_egoctrl`，`autotrans` 只启动 FUEL bridge + AutoTrans。
- [ ] 禁止两套控制器同时启动；启动后检查 `/UAV0/mavros/setpoint_raw/attitude` 只有一个 publisher。
- [ ] 保留原 `run_ctrl_lio.launch`、`cxr_egoctrl_v1` 源码和原入口，便于落回简单控制器。
- [ ] 不自动解锁、不自动切 OFFBOARD、不在飞行中切换控制器模式。

## 6. 验证

- [ ] `python3`/C++ 静态检查和 `catkin_make -DCMAKE_BUILD_TYPE=Release`。
- [ ] 检查 `rosmsg show quadrotor_msgs/PolynomialTraj`、`PolynomialMatrix`、`bspline/Bspline`。
- [ ] 隔离 ROS master 发布模拟 B-spline，确认 bridge 输出 piece 数量、duration、系数排列和 yaw 字段正确。
- [ ] 检查 `rostopic info`：FUEL -> bridge -> AutoTrans -> UAV0 MAVROS，且没有原控制器并发 publisher。
- [ ] 先只运行雷达、FAST-LIO、规划和 bridge，确认轨迹后再在地面进行控制器启动检查；不直接实飞。
- [ ] 实飞前确认 PX4/MAVROS、定位、RPM、电池、人工接管和 CH6/CH8/CH10；首测保持外力补偿关闭。

## 7. 提交边界

- [ ] 只提交 CAV0 本次新增/修改的 source、message、launch、配置和文档；不提交 build/devel/log/bag。
- [ ] 不修改 CAV1、UAV2、UAV3 或 AutoTrans `main`，除非用户另行要求。
- [ ] 编译和链路验证失败时不声称完成；记录失败命令和原因。

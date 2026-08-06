# UAV0 FUEL 规划器接入 AutoTrans MPC 实施设计

## 1. 文档状态

本文档是实施设计，不是执行记录。当前只完成方案整理，不修改控制器、消息、launch、YAML 或飞行参数。

目标是在 CAV1 的 ~/match_ws 中保留 UAV0 前机的 FUEL 规划器，用 AutoTrans MPC 替换原来的 px4ctrl/cxr_egoctrl 控制器。

## 2. 已确认的设计约束

- FUEL 规划器和 AutoTrans MPC 都使用雷达里程计：/UAV0/fast_lio/Odometry。
- 第一阶段不增加坐标旋转、平移或轴交换。
- UAV0 的点云使用 FUEL 当前配置的 /UAV0/fast_lio/cloud_registered。
- FUEL 规划器输出的偏航轨迹必须保留，AutoTrans 使用 FUEL 的 yaw 和 yaw_rate。
- CH6/QGC/PX4 负责进入和退出 OFFBOARD；AutoTrans 不自动解锁、不自动切换 OFFBOARD。
- CH8 当前逻辑为：
  - 低位 <1300：请求 AUTO_TAKEOFF；
  - 中位 1300~1700：AUTO_HOVER；
  - 过渡区 1700~1800：不触发新的自动模式；
  - 高位 >1800：CMD_CTRL，允许执行规划轨迹。
- CH10 继续作为降落触发通道。
- CH6 退出 OFFBOARD 时，AutoTrans 必须清空当前规划轨迹、清零已应用外力并回到 MANUAL_CTRL。
- 同一时间只能有一个节点向 /UAV0/mavros/setpoint_raw/attitude 发布控制指令。

## 3. 当前接口事实

FUEL 发布的原始轨迹为：

~~~
/UAV0/planning/bspline
类型：bspline/Bspline
~~~

其字段包括：

~~~
order
traj_id
start_time
knots
pos_pts
yaw_pts
yaw_dt
~~~

FUEL 的 traj_server 会根据该 B-spline 生成逐时刻的 PositionCommand，其中包含位置、速度、加速度、yaw 和 yaw_dot。但 AutoTrans 当前执行链路使用完整的 PolynomialTraj 轨迹，而不是逐点 PositionCommand。

AutoTrans 当前轨迹消息为 quadrotor_msgs/PolynomialTraj。其中位置 piece 使用 PolynomialMatrix 保存三维多项式系数，系数按 Eigen 列主序排列。控制器按 num_dim x (num_order + 1) 恢复系数矩阵，不能改变现有 data[] 排列。

## 4. 总体数据链路

~~~
/UAV0/fast_lio/Odometry
        |
        +-----------------------> FUEL Planner
        |                              |
/UAV0/fast_lio/cloud_registered    /UAV0/planning/bspline
                                       |
                                       v
                             fuel_autotrans_bridge
                                       |
                         /UAV0/planning/autotrans_trajectory
                                       |
                                       v
                               AutoTrans MPC
                                       |
                     /UAV0/mavros/setpoint_raw/attitude
~~~

FUEL 的 /UAV0/planning/pos_cmd 可以保留用于可视化、诊断或日志，但不再作为 AutoTrans 的主控制输入。

## 5. 轨迹消息设计

在 quadrotor_msgs/PolynomialTraj.msg 末尾增加可选偏航字段：

~~~
bool has_yaw
quadrotor_msgs/PolynomialMatrix[] yaw_trajectory
~~~

约定如下：

- trajectory[] 继续保存三维位置多项式，不改变现有字段含义。
- yaw_trajectory[] 与位置 piece 数量一致。
- 每个 yaw piece 的 num_dim=1。
- yaw piece 的 duration 必须与对应位置 piece 相同。
- yaw 系数按 c0, c1, ... 表示：

~~~
yaw(t) = c0 + c1*t + c2*t^2 + ...
~~~

- 控制器对 yaw 多项式求导得到 yaw_rate，单位为 rad/s。
- has_yaw=false 或 yaw 数据无效时，保持原有 yaw 回退逻辑，不影响 Diff-Planner 轨迹。
- Diff-Planner 桥接发送的消息将 has_yaw=false，不需要改变其原有行为。
- FUEL 桥接发送的消息将 has_yaw=true。

## 6. FUEL 轨迹桥接

在现有 autotrans_reference_bridge 包中增加 FUEL 专用桥接节点，避免修改 FUEL 规划器本身。

输入：

~~~
/UAV0/planning/bspline
bspline/Bspline
~~~

输出：

~~~
/UAV0/planning/autotrans_trajectory
quadrotor_msgs/PolynomialTraj
~~~

桥接过程：

1. 校验 traj_id、start_time、order、knots、位置控制点和 yaw 控制点数量。
2. 根据 FUEL 的 knots 和 B-spline 阶数，将每个有效 knot 区间转换为局部幂基多项式。
3. 位置多项式按 [x,y,z] 三维矩阵写入现有 trajectory[]。
4. yaw B-spline 使用 yaw_dt 和 yaw 控制点转换为一维 yaw_trajectory[]。
5. 对 yaw 控制点先做连续角度展开，避免跨越 -pi/pi 时产生大跳变。
6. 位置和 yaw 使用相同的 start_time、piece 时间边界和 duration。
7. 由 yaw 多项式求导的 yaw_rate 由 AutoTrans 控制器使用。

不能直接把 B-spline 控制点当作多项式系数。桥接必须通过采样重构测试，确认转换后的轨迹在各 piece 内与 FUEL 原始 B-spline 的位置、速度、加速度、yaw 和 yaw_rate 一致。

如果位置和 yaw 的有效时长不一致，桥接节点应拒绝该消息并报警，不能静默截断或重复末端数据。

## 7. AutoTrans 控制器改造边界

控制器接收 PolynomialTraj 后：

- 继续按原逻辑加载位置多项式队列；
- 对 has_yaw=true 的轨迹同步加载 yaw 多项式队列；
- NMPC 预测窗口的每个采样时刻同时使用位置和 yaw 轨迹；
- yaw 的一阶导数作为 yaw_rate 传入平坦性计算；
- yaw 数据缺失、piece 数量不一致、duration 不一致或系数非有限时拒绝执行该轨迹；
- 无 yaw 轨迹时回退到原有 use_fix_yaw 逻辑。

use_fix_yaw 只影响悬停参考和没有规划 yaw 的旧轨迹，不覆盖有效的 FUEL yaw。

本改造不改变：

- ACADO 状态量、输入量和 OnlineData 维度；
- 推力模型和在线推力映射；
- 外力估计或扰动补偿公式；
- MAVROS 的 body_rate + normalized thrust 输出接口；
- 四元数分量顺序和世界系/机体系约定。

因此不需要重新生成 ACADO solver，但需要重新生成 ROS 消息并重新编译 catkin 工作空间。

## 8. CH8、CH10 与 OFFBOARD 状态门控

### 8.1 AUTO_TAKEOFF

CH8 低位只在 MANUAL_CTRL 且起飞安全条件满足时请求 AUTO_TAKEOFF。

AUTO_TAKEOFF 阶段：

- 不执行 FUEL 轨迹；
- 不自动解锁；
- 不自动切换 OFFBOARD；
- 仅在 PX4 已由 CH6/QGC 切入 OFFBOARD 后运行起飞控制；
- 若 CH8 切到中位，立即切换到 AUTO_HOVER；
- 若 CH10 触发且飞机仍在地面，拒绝执行降落流程。

### 8.2 AUTO_HOVER

CH8 中位时保持悬停参考。收到 FUEL 轨迹也不执行，直到 CH8 切到高位进入 CMD_CTRL。

### 8.3 CMD_CTRL

CH8 高位进入 CMD_CTRL。只有在轨迹存在、时间戳有效、轨迹未过期且安全状态有效时，才执行 FUEL 轨迹；否则保持悬停并输出节流诊断。

### 8.4 OFFBOARD 退出

当 PX4 状态从 OFFBOARD 变为其他模式时，AutoTrans 应在同一控制周期执行：

~~~
清空 trajectory queue
清零 applied disturbance
exec_traj_state = HOVER
fsm_state = MANUAL_CTRL
~~~

再次进入 OFFBOARD 后，必须重新按照 CH8 逻辑进入 AUTO_TAKEOFF、AUTO_HOVER 或 CMD_CTRL，不能恢复旧轨迹。

## 9. Launch 和节点替换

UAV0 FUEL 规划器、FAST-LIO 和雷达启动方式保持不变，只调整控制链路：

- 保留 FUEL Planner、exploration manager 和 FUEL traj_server；
- 停止或删除 UAV0 原来的 px4ctrl/cxr_egoctrl 控制器启动节点；
- 启动 FUEL 专用桥接节点；
- 启动 AutoTrans MPC，并将其 odom 输入 remap 到 /UAV0/fast_lio/Odometry；
- AutoTrans 的 MAVROS 输入输出统一使用 /UAV0/mavros/...；
- 确认没有第二个节点发布 /UAV0/mavros/setpoint_raw/attitude。

目标连接关系：

~~~
/UAV0/planning/bspline
    Publisher: FUEL Planner
    Subscriber: fuel_autotrans_bridge

/UAV0/planning/autotrans_trajectory
    Publisher: fuel_autotrans_bridge
    Subscriber: AutoTrans MPC

/UAV0/mavros/setpoint_raw/attitude
    Publisher: AutoTrans MPC
~~~

## 10. 验证计划

### 10.1 消息和编译验证

~~~
cd ~/match_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
rosmsg show quadrotor_msgs/PolynomialTraj
rosmsg show quadrotor_msgs/PolynomialMatrix
~~~

确认新增 yaw 字段已生成，且现有 Diff-Planner bridge 仍能编译。

### 10.2 B-spline 转换单元测试

构造包含多个 position piece 和 yaw piece 的模拟 bspline/Bspline，检查：

- trajectory_id 和 start_time 保持不变；
- position piece 数量正确；
- yaw piece 数量与 position piece 相同；
- duration 一一对应；
- 采样点的位置、速度、加速度误差在设定阈值内；
- yaw 连续，没有 2*pi 跳变；
- yaw_rate 与 FUEL 求导结果一致；
- 非法 knots、空控制点、非有限值和时长不一致会被拒绝。

### 10.3 ROS 链路验证

~~~
rostopic type /UAV0/fast_lio/Odometry
rostopic type /UAV0/planning/bspline
rostopic type /UAV0/planning/autotrans_trajectory
rostopic info /UAV0/planning/bspline
rostopic info /UAV0/planning/autotrans_trajectory
rostopic info /UAV0/mavros/setpoint_raw/attitude
~~~

预期：

~~~
/UAV0/fast_lio/Odometry                  nav_msgs/Odometry
/UAV0/planning/bspline                   bspline/Bspline
/UAV0/planning/autotrans_trajectory      quadrotor_msgs/PolynomialTraj
~~~

确认 UAV0 原控制器不再订阅或发布同一控制输出。

### 10.4 状态机验证

不安装桨叶或使用安全台架，依次验证：

~~~
CH8 低位 -> AUTO_TAKEOFF 请求/等待
CH8 中位 -> AUTO_HOVER
CH8 高位 -> CMD_CTRL
收到 FUEL 轨迹 -> 执行 POLY_TRAJ
CH6 退出 OFFBOARD -> 清空轨迹并回 MANUAL_CTRL
~~~

重点查看日志中的状态转换、轨迹 id、yaw 来源和 OFFBOARD 退出清理信息。

### 10.5 首次实飞策略

1. 外力补偿先关闭，外力估计只记录。
2. 先只运行雷达、FAST-LIO、FUEL、bridge 和 AutoTrans，不执行自主探索。
3. 用单个短距离目标验证位置和 yaw 跟踪。
4. 确认 CH8/CH6/CH10 状态切换和人工接管正常后，再开启 FUEL 自主探索。
5. 首次自主探索时限制区域、速度和轨迹时长，保持人工接管。

## 11. 验收标准

- FUEL 和 AutoTrans 都只使用 /UAV0/fast_lio/Odometry。
- FUEL yaw 与 yaw_rate 能到达 AutoTrans NMPC 参考。
- 位置、速度、加速度和 yaw 使用统一的轨迹时间戳。
- CH8 三档行为与当前定义一致。
- CH6 退出 OFFBOARD 后不会继续执行旧轨迹。
- /UAV0/mavros/setpoint_raw/attitude 只有 AutoTrans 一个发布者。
- 旧 Diff-Planner bridge、旧消息和无 yaw 轨迹仍能正常工作。
- 没有编译错误、消息 MD5 不一致或轨迹 piece 数量不一致。

## 12. 不在本次范围内

- 不修改 FUEL 规划算法和 frontier 选择逻辑；
- 不修改 FAST-LIO 外参或雷达驱动参数；
- 不修改坐标轴方向、坐标原点或 yaw 符号约定；
- 不修改 ACADO 模型和生成 solver；
- 不启用自动解锁或自动 OFFBOARD；
- 不在未完成台架验证前进行自主探索实飞。

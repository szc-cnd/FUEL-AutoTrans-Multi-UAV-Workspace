# CAV0 控制器与 FUEL 规划器安全审查

## 1. 审查范围

本文针对 CAV0 的 FUEL 规划、轨迹桥接和 AutoTrans NMPC 控制链路，记录已实施的简化安全处理和仍需注意的边界。当前链路为：

```text
RViz/入口触发
    -> FUEL 规划器
    -> /UAV0/planning/bspline
    -> fuel_autotrans_bridge
    -> /UAV0/planning/autotrans_trajectory
    -> AutoTrans NMPC
    -> /UAV0/mavros/setpoint_raw/attitude
```

FUEL 的 `/UAV0/planning/pos_cmd` 仍保留给原来的简单控制器和 logger 使用，但 AutoTrans NMPC 不再订阅或执行 `PositionCommand`。

## 2. 总体结论

本次已经完成以下修改：

| 问题 | 当前处理 |
| --- | --- |
| 轨迹结束时访问空队列 | 已增加队列保护、终点缓存和异常悬停 |
| B-spline 0.5 s 超时误中止 | 已删除桥接超时判断 |
| 桥接重放旧轨迹 | 已取消 latched 发布，控制器启动状态为空队列 |
| NMPC 求解失败 | 已检查有限值，优先回退上一组有效输入 |
| AUTO.LAND 接管确认 | 已等待 `/UAV0/mavros/state.mode == AUTO.LAND` |
| PositionCommand 与完整轨迹混用 | AutoTrans 已只接收 `PolynomialTraj` |
| 内部推力上限不一致 | `max_thrust` 已调整为 `26.5 N`，与归一化上限一致 |
| 起飞阶段外力估计干扰 | `AUTO_TAKEOFF` 阶段清空外力估计状态，不更新外力窗口 |

以下内容仍是明确的系统边界，而不是本次新增的复杂运行时机制：

- 不用“没有新 B-spline”判断规划器失联；不新增规划器心跳。
- 不在桥接节点中增加每 `0.02 s` 的速度、加速度、jerk 和段间跳变采样检查。
- 不自动做坐标变换；启动文件和 FAST-LIO 必须保证轨迹与 odom 使用同一坐标约定。

## 3. 轨迹队列与结束处理

### 3.1 原风险

旧逻辑在 `POLY_TRAJ` 分支中访问 `trajectory_data.traj_queue.front()`。如果轨迹被中止、队列被清空或状态不同步，可能对空队列调用 `.front()`，造成未定义行为甚至控制器退出。

### 3.2 当前实现

文件：

```text
src/AutoTrans-quadrotor-wind-mpc/controller/payload_mpc_controller/src/mpc_input.cpp
src/AutoTrans-quadrotor-wind-mpc/controller/payload_mpc_controller/src/mpc_fsm.cpp
```

当前规则：

1. `exec_traj == 1` 时，`traj_queue` 必须非空；所有 `front()` 访问前都经过非空检查。
2. 接收一条完整 `ACTION_ADD` 时，先在临时队列中检查全部 piece，再一次性替换旧队列。
3. 接收成功后缓存最后一段轨迹的终点位置和 yaw。
4. 正常结束时悬停在缓存终点。
5. `ACTION_ABORT`、非法轨迹、队列为空或执行状态异常时，清空轨迹并悬停当前位置。
6. 异常路径不会继续执行旧轨迹，也不会调用旧队列的 `.front()`。

这保留了“正常轨迹结束后悬停在终点”的行为，同时避免异常中止时继续飞向旧轨迹。

## 4. 桥接节点的发布和失效策略

文件：

```text
src/autotrans_reference_bridge/src/fuel_autotrans_bridge_node.cpp
```

### 4.1 删除 B-spline 超时中止

FUEL 只在生成新规划时发布 B-spline，而一条正常轨迹可能持续超过 `0.5 s`。因此不能用“距离上次 B-spline 的时间”判断规划器失联。

当前已经删除：

- `trajectory_timeout` 参数；
- 周期性 timeout callback；
- 因没有新 B-spline 而自动发布 `ACTION_ABORT` 的逻辑。

当前保留：

- B-spline 的 piece、阶数、knots、持续时间和有限值检查；
- 拟合失败时不发布 `ACTION_ADD`；
- 拟合失败时发布一次普通的、非 latched `ACTION_ABORT`；
- 规划器明确发布的中止语义仍由控制器处理。

### 4.2 取消 latched

`/UAV0/planning/autotrans_trajectory` 现在是普通 publisher，不再把上一轮实验的 `ACTION_ADD` 自动重放给新启动的控制器。控制器构造时也会清空本地轨迹队列，初始进入 `HOVER`。

### 4.3 规划器停止发布时的实际行为

取消超时后，如果规划器进程仍在运行但暂时不发布新 B-spline：

1. 当前已经完整接收的 `PolynomialTraj` 继续执行到它声明的结束时间；
2. 轨迹正常结束后悬停在终点；
3. 不会自动恢复旧的 `PositionCommand`；
4. 等待新的有效 `PolynomialTraj` 或明确 `ACTION_ABORT`。

如果需要检测“规划器进程仍在但不再工作”，应单独增加规划器心跳或节点状态监测。本次没有加入该机制，避免把正常的重规划间隔误判为失联。

## 5. AutoTrans 输入接口

AutoTrans 控制器现在只订阅：

```text
/UAV0/planning/autotrans_trajectory
    quadrotor_msgs/PolynomialTraj
```

控制器不再订阅：

```text
/UAV0/planning/pos_cmd
/UAV0/move_base_simple/goal
cmd_trigger
```

因此控制行为为：

```text
收到完整 PolynomialTraj
    -> 执行轨迹
    -> 正常结束后悬停在终点

收到 ACTION_ABORT 或轨迹非法
    -> 清空轨迹
    -> 悬停当前位置

没有新 PolynomialTraj
    -> 保持当前位置/当前悬停参考
```

原来的简单控制器入口仍保留在 `uav0_fuel_controller.launch`，不会因为 AutoTrans 的输入接口调整而删除。

## 6. NMPC 求解失败保护

文件：

```text
src/AutoTrans-quadrotor-wind-mpc/controller/payload_mpc_controller/src/mpc_controller.cpp
src/AutoTrans-quadrotor-wind-mpc/controller/payload_mpc_controller/src/mpc_fsm.cpp
```

当前处理顺序：

1. 求解成功后检查预测状态和控制输入是否有限。
2. 发布前将物理推力和机体系角速度限制在配置范围内。
3. 求解失败且存在上一组有效输出时，回退上一组有效预测状态和控制输入。
4. 尚无历史有效输出时，使用零机体系角速度和悬停物理推力作为内部保底；落地状态下最终归一化推力发送为 `0`。
5. 连续失败超过 `0.2 s` 时，清空轨迹并锁定当前位置悬停，但不由 AutoTrans 自动切换 PX4 飞行模式。

这里的 `body_rate.x/y/z` 是发送给 MAVROS/PX4 的机体系角速度命令，单位通常为 `rad/s`；`AttitudeTarget.thrust` 是 MAVROS/PX4 归一化推力，不是牛顿推力。

## 7. AUTO.LAND 接管

AutoTrans 仍负责发送降落请求，但不把 `mode_sent=true` 当成 PX4 已经接管。当前流程为：

```text
达到 AutoTrans 降落交接条件
    -> 请求 /UAV0/mavros/set_mode: AUTO.LAND
    -> 每秒最多重试一次
    -> 等待 /UAV0/mavros/state.mode == AUTO.LAND
    -> 确认后 AutoTrans 退出到手动状态
```

如果服务请求发送成功但 PX4 实际没有进入 `AUTO.LAND`，AutoTrans 不会立即停止当前降落控制，而是继续等待并重试。该流程不负责自动解锁。

## 8. 起飞和外力估计

`AUTO_TAKEOFF` 阶段不向外力观察器加入样本，并持续清空外力估计窗口；进入 `AUTO_HOVER` 或 `CMD_CTRL` 后再重新采集。这样可以避免起飞瞬态、地面支持力和推力快速变化污染 `f_Q`。

外力估计姿态默认来自：

```text
/UAV0/mavros/local_position/odom
```

NMPC 的位置、速度和控制状态仍来自：

```text
/UAV0/fast_lio/Odometry
```

因此默认 `use_px4_imu_attitude=false` 时不需要运行时 yaw 标定。`/UAV0/mavros/imu/data` 仍提供外力估计的机体系加速度和角速度，RPM 来自 `/UAV0/mavros/esc_status`。

## 9. 推力上限一致性

当前配置：

```yaml
mass_q: 1.844
hover_percentage: 0.54
max_thrust: 26.5
max_normalized_thrust: 0.8
```

`max_thrust` 是 NMPC 内部物理推力上限，单位 `N`；`max_normalized_thrust` 是发送给 MAVROS/PX4 的归一化推力上限。启动时会检查初始推力映射对应的最大归一化输出是否覆盖 `max_thrust`，不一致时拒绝启动，避免 NMPC 允许的推力高于飞控实际收到的推力。

本次不把 `max_normalized_thrust` 提高到 `1.0`，除非重新完成电机、ESC、电池和机体最大推力验证。

## 10. 坐标系边界

本次不在桥接器或控制器中自动做坐标变换，也不新增运行时 frame 检查。系统必须通过启动配置保证：

```text
/UAV0/fast_lio/Odometry 的位置、速度、姿态
/UAV0/planning/bspline 的轨迹数值
桥接输出的 frame_id: UAV0/camera_init
```

使用同一套世界坐标约定。若重新启动 FAST-LIO、改变雷达外参、改变坐标原点或 yaw，必须重新做坐标一致性验证；不能依赖控制器自动修正。

## 11. 有意不加入的复杂检查

桥接后的多项式拟合目前只做基础数据检查，不在每 `0.02 s` 额外采样验证速度、加速度、jerk、最小高度和段间跳变。原因是：

- FUEL 已经负责规划轨迹的碰撞和运动学约束；
- 额外采样会增加桥接复杂度和调试负担；
- 本次优先修复会导致崩溃、旧轨迹重放、错误模式接管和输出非有限值的直接风险。

如果后续实测发现桥接拟合造成明显尖峰，再单独增加轨迹采样拒绝机制，不与本次基础链路修改混在一起。

## 12. 实飞前检查

在 CAV0 Ubuntu 工作空间完成统一验证：

```bash
cd ~/match_ws
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

链路检查：

```bash
rostopic type /UAV0/planning/bspline
rostopic type /UAV0/planning/autotrans_trajectory
rostopic info /UAV0/planning/autotrans_trajectory
rostopic echo /UAV0/mavros/state
rostopic echo /UAV0/mavros/setpoint_raw/attitude
```

重点确认：

- `autotrans_trajectory` 的 publisher 是 `fuel_autotrans_bridge`；
- `autotrans_trajectory` 的 subscriber 是 `mpc_controller_node`；
- 控制器没有 `/UAV0/planning/pos_cmd` subscriber；
- 未解锁、未进入 OFFBOARD 时不进行正式飞行；
- 首次实飞保持 `enable_disturbance_compensation=false`；
- 轨迹结束后保持终点悬停，收到 `ACTION_ABORT` 后保持当前位置；
- PX4 实际进入 `AUTO.LAND` 后 AutoTrans 才停止降落控制；
- `max_thrust` 与最终归一化推力上限一致。

## 13. 自动化和台架验证

本地 Windows 工作树可执行：

```powershell
git diff --check
python -m unittest discover -s src/AutoTrans-quadrotor-wind-mpc/controller/payload_mpc_controller/test
python -m unittest discover -s src/autotrans_reference_bridge/test
```

在 Ubuntu ROS 工作空间还应验证：

1. 完整轨迹正常执行，结束后悬停在缓存终点；
2. 执行中发送 `ACTION_ABORT`，不崩溃并悬停当前位置；
3. 空队列进入轨迹状态时不崩溃并回到 `HOVER`；
4. 连续发送两条轨迹时，新轨迹完整替换旧轨迹；
5. 轨迹结束或中止后，旧的 `PositionCommand` 不会重新激活运动；
6. NMPC 求解失败时不发布非有限 body rate 或 thrust；
7. PX4 拒绝 `AUTO.LAND` 时保持 AutoTrans 降落控制并继续重试；
8. CAV0 传感器、MAVROS、规划器和桥接启动顺序与《仿真顺序比赛》一致。

## 14. 审查边界

- 本文描述的是 CAV0 `vehicle/cav0` 分支的实现方案和验证要求。
- 本次不修改 CAV1、UAV2、UAV3 或 `main` 分支。
- 不手动修改 ACADO 生成代码，不重新生成 solver。
- 当前 Windows 环境不能代替 Ubuntu ROS、MAVROS/PX4 和无桨台架验证；未执行的 ROS 实测不能宣称已经通过。

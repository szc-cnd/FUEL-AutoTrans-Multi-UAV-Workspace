#include "mpc_input.h"
#include <algorithm>
#include <cmath>
RC_Data_t::RC_Data_t()
{
    rcv_stamp = ros::Time(0);

    last_mode = -1.0;
    last_gear = -1.0;
    last_land = -1.0;

    // Parameter initilation is very important in RC-Free usage!
    is_manual_mode = true;
    mode_valid = false;
    is_hover_mode = false;
    enter_hover_mode = false;
    is_command_mode = false;
    enter_command_mode = false;
    enter_land_mode = false;
    is_takeoff_mode = false;
    toggle_reboot = false;
    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}

void RC_Data_t::set_mode_params(int mode_ch, int land_ch, int low_th, int mid_low_th,
                                int mid_high_th, int high_th)
{
    mode_channel = mode_ch;
    land_channel = land_ch;
    low_threshold = low_th;
    mid_low_threshold = mid_low_th;
    mid_high_threshold = mid_high_th;
    high_threshold = high_th;
}

void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    enter_command_mode = false;
    enter_hover_mode = false;
    enter_land_mode = false;
    toggle_reboot = false;

    if (msg.channels.size() < 4)
    {
        ROS_WARN_THROTTLE(1.0, "[MPCCtrl] /mavros/rc/in has less than 4 channels.");
        is_manual_mode = true;
        is_command_mode = false;
        is_hover_mode = false;
        is_takeoff_mode = false;
        mode_valid = false;
        return;
    }

    for (int i = 0; i < 4; i++)
    {
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;
        if (ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1 - DEAD_ZONE);
        else if (ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else
            ch[i] = 0.0;
    }

    if (mode_channel < 0 || msg.channels.size() <= static_cast<size_t>(mode_channel))
    {
        ROS_WARN_THROTTLE(1.0, "[MPCCtrl] /mavros/rc/in has no CH%d for mode selection.", mode_channel + 1);
        // 模式通道缺失时按手动请求处理，避免通道异常时沿用上一帧 AUTO_HOVER/CMD_CTRL 状态。
        is_manual_mode = true;
        mode_valid = false;
        is_command_mode = false;
        is_hover_mode = false;
        is_takeoff_mode = false;
        return;
    }

    // CH8(数组下标7)作为三段主模式通道：低位起飞、中位悬停、高位命令。
    // PWM/us 由 mavros_msgs/RCIn.channels 提供；CH6 的 OFFBOARD 仍由 PX4/QGC 管理。
    // 1700~1800 us 是未定义过渡区，不触发新的自动模式。
    mode = static_cast<double>(msg.channels[mode_channel]);
    check_validity();
    if (!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }

    const bool mode_pwm_valid = mode >= 800.0 && mode <= 2200.0;
    if (!mode_pwm_valid)
    {
        // 异常 PWM 不能触发起飞或自动控制，交给状态机执行安全退出。
        is_manual_mode = true;
        mode_valid = false;
        is_takeoff_mode = false;
        is_hover_mode = false;
        is_command_mode = false;
        last_mode = mode;
        return;
    }

    const bool last_command_mode = last_mode > high_threshold;
    const bool last_hover_mode = last_mode >= mid_low_threshold && last_mode <= mid_high_threshold;

    mode_valid = true;
    is_manual_mode = false;
    is_takeoff_mode = mode < low_threshold;
    is_hover_mode = mode >= mid_low_threshold && mode <= mid_high_threshold;
    is_command_mode = mode > high_threshold;
    // 低位触发一次起飞；已经处于自动控制时，低位不会重新启动起飞流程。
    // 中位是悬停，高位是命令/轨迹控制。
    enter_hover_mode = !last_hover_mode && is_hover_mode;
    enter_command_mode = !last_command_mode && is_command_mode;

    // CH6 不由 AutoTrans 读取；OFFBOARD 由 PX4/QGC 的 RC_MAP_OFFB_SW 负责切换。

    // CH10(数组下标9)只作为降落触发通道。必须先看到低位，再看到高位，避免上电时高位误触发。
    if (land_channel >= 0 && msg.channels.size() > static_cast<size_t>(land_channel))
    {
        land = static_cast<double>(msg.channels[land_channel]);
        if (!have_init_last_land)
        {
            have_init_last_land = true;
            last_land = land;
        }
        enter_land_mode = last_land < low_threshold && land > high_threshold;
        last_land = land;
    }
    else
    {
        ROS_WARN_THROTTLE(1.0, "[MPCCtrl] /mavros/rc/in has no CH%d for landing trigger.", land_channel + 1);
    }

    // CH8 已改为主模式通道，不再兼任飞控重启触发，避免模式切换和重启命令冲突。
    reboot_cmd = mode;
    toggle_reboot = false;

    last_mode = mode;
}

void RC_Data_t::check_validity()
{
    if (mode >= 800.0 && mode <= 2200.0)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC data validity check fail. mode PWM=%f", mode);
    }
}

bool RC_Data_t::check_centered()
{
    bool centered = abs(ch[0]) < 1e-5 && abs(ch[1]) < 1e-5 && abs(ch[2]) < 1e-5 && abs(ch[3]) < 1e-5;
    return centered;
}

Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = ros::Time(0);
    q.setIdentity();
    rcv_new_msg = false;
};

void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();
    rcv_new_msg = true;

    uav_utils::extract_odometry(pMsg, p, v, q, w);

// #define VEL_IN_BODY
#ifdef VEL_IN_BODY /* Set to 1 if the velocity in odom topic is relative to current body frame, not to world frame.*/
    Eigen::Quaternion<double> wRb_q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
    Eigen::Matrix3d wRb = wRb_q.matrix();
    v = wRb * v;

    static int count = 0;
    if (count++ % 500 == 0)
        ROS_WARN("VEL_IN_BODY!!!");
#endif
}

Start_Trigger_Data_t::Start_Trigger_Data_t()
{
    recv_start_trig = false;
};

void Start_Trigger_Data_t::feed(geometry_msgs::PoseStampedConstPtr pMsg)
{
    recv_start_trig = true;
}

Cmd_Trigger_Data_t::Cmd_Trigger_Data_t()
{
    recv_cmd_trig = false;
};

void Cmd_Trigger_Data_t::feed(geometry_msgs::PoseStampedConstPtr pMsg)
{
    recv_cmd_trig = true;
}

Imu_Data_t::Imu_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    filtered_a = accel_filter.apply(a);
    filtered_w = gyro_filter.apply(w);
}

State_Data_t::State_Data_t()
{
}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg)
{

    current_state = *pMsg;
    rcv_stamp = ros::Time::now();
}

ExtendedState_Data_t::ExtendedState_Data_t()
{
}

void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr pMsg)
{
    current_extended_state = *pMsg;
    rcv_stamp = ros::Time::now();
}

Trajectory_Data_t::Trajectory_Data_t()
{
    total_traj_start_time = ros::Time(0);
    total_traj_end_time = ros::Time(0);
    exec_traj = 0;
}

void Trajectory_Data_t::feed(quadrotor_msgs::PolynomialTrajConstPtr pMsg)
{

    // #1. try to execuse the action
    const quadrotor_msgs::PolynomialTraj &traj = *pMsg;
    if (traj.action == quadrotor_msgs::PolynomialTraj::ACTION_ADD)
    {
        const ros::Time now = ros::Time::now();
        if ((int)traj.trajectory_id < 1)
        {
            ROS_ERROR_THROTTLE(1.0, "[轨迹] 拒绝轨迹：trajectory_id 无效。");
            return;
        }
        if (pMsg->header.stamp.isZero())
        {
            ROS_ERROR_THROTTLE(1.0, "[轨迹] 拒绝轨迹：header.stamp 为空。");
            return;
        }
        const double age = (now - pMsg->header.stamp).toSec();
        if (age > 0.8 || age < -0.1)
        {
            ROS_WARN_THROTTLE(1.0, "[轨迹] 拒绝旧轨迹或过早轨迹：id=%u，时间差=%.3f s。",
                              traj.trajectory_id, age);
            return;
        }
        if (have_last_trajectory_id &&
            (traj.trajectory_id <= last_trajectory_id ||
             pMsg->header.stamp <= last_trajectory_stamp))
        {
            ROS_WARN_THROTTLE(1.0, "[轨迹] 拒绝重复或倒退轨迹：id=%u。", traj.trajectory_id);
            return;
        }
        if (traj.trajectory.empty())
        {
            ROS_ERROR_THROTTLE(1.0, "[轨迹] 拒绝轨迹：piece 为空。");
            return;
        }

        oneTraj_Data_t traj_data;
        traj_data.traj_start_time = pMsg->header.stamp;
        double t_total = 0;
        for (auto &piece : traj.trajectory)
        {
            if (piece.num_dim != 3 || piece.num_order < 0 || piece.num_order > 20 ||
                piece.duration <= 0.0 || !std::isfinite(piece.duration) ||
                piece.data.size() != static_cast<size_t>(piece.num_dim) *
                                         static_cast<size_t>(piece.num_order + 1))
            {
                ROS_ERROR_THROTTLE(1.0, "[轨迹] 拒绝轨迹：piece 维度、阶数、duration 或 data 无效。");
                return;
            }
            for (const double coefficient : piece.data)
            {
                if (!std::isfinite(coefficient))
                {
                    ROS_ERROR_THROTTLE(1.0, "[轨迹] 拒绝轨迹：piece 系数不是有限值。");
                    return;
                }
            }
            traj_data.traj.emplace_back(piece.duration, Eigen::Map<const Eigen::MatrixXd>(&piece.data[0], piece.num_dim, (piece.num_order + 1)));
            t_total += piece.duration;
        }
        traj_data.traj_end_time = traj_data.traj_start_time + ros::Duration(t_total);
        if (!std::isfinite(t_total) || t_total <= 0.0)
        {
            ROS_ERROR_THROTTLE(1.0, "[轨迹] 拒绝轨迹：总 duration 无效。");
            return;
        }
        last_end_position = traj_data.traj.getJuncPos(traj_data.traj.getPieceNum());
        last_end_position_valid = last_end_position.allFinite();
        if (!last_end_position_valid)
        {
            ROS_ERROR_THROTTLE(1.0, "[轨迹] 拒绝轨迹：终点位置不是有限值。");
            return;
        }
        if (ros::Time::now() < traj_data.traj_start_time) // Future traj
        {
            // A future trajectory
            while ((!traj_queue.empty()) && traj_queue.back().traj_start_time > traj_data.traj_start_time)
            {
                traj_queue.pop_back(); // remove old trajectory (remove the traj newer than the new traj)
            }
            traj_queue.push_back(traj_data);
            // adjust_end_time();
            total_traj_end_time = traj_queue.back().traj_end_time;
            total_traj_start_time = traj_queue.front().traj_start_time;
        }
        else
        {
            while ((!traj_queue.empty()) && traj_queue.front().traj_start_time < traj_data.traj_start_time)
            {
                traj_queue.pop_front(); // remove old trajectory
            }
            traj_queue.push_front(traj_data);
            // adjust_end_time();
            total_traj_end_time = traj_queue.back().traj_end_time;
            total_traj_start_time = traj_queue.front().traj_start_time;
        }
        exec_traj = 1;
        last_trajectory_id = traj.trajectory_id;
        last_trajectory_stamp = pMsg->header.stamp;
        have_last_trajectory_id = true;
        ROS_INFO("[轨迹] 收到有效轨迹：id=%u，piece 数量=%zu。",
                 traj.trajectory_id, traj.trajectory.size());
    }
    else if (traj.action == quadrotor_msgs::PolynomialTraj::ACTION_ABORT)
    {
        ROS_WARN("[MPCCtrl] Aborting the trajectory.");
        total_traj_start_time = ros::Time(0);
        total_traj_end_time = ros::Time(0);
        traj_queue.clear();
        exec_traj = -1;
        last_end_position_valid = false;
    }
    else if (traj.action == quadrotor_msgs::PolynomialTraj::ACTION_WARN_IMPOSSIBLE)
    {
        total_traj_start_time = ros::Time(0);
        total_traj_end_time = ros::Time(0);
        traj_queue.clear();
        exec_traj = -1;
        last_end_position_valid = false;
    }
}

Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    yaw = uav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
}

Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    double vlotage = 0;
    for (size_t i = 0; i < pMsg->cell_voltage.size(); ++i)
    {
        vlotage += pMsg->cell_voltage[i];
    }
    volt = 0.8 * volt + 0.2 * vlotage; // Naive LPF, cell_voltage has a higher frequency

    // volt = 0.8 * volt + 0.2 * pMsg->voltage; // Naive LPF
    percentage = pMsg->percentage;

    static ros::Time last_print_t = ros::Time(0); // mark
    if (percentage > 0.05)
    {
        if ((rcv_stamp - last_print_t).toSec() > 10)
        {
            ROS_INFO("[MPCCtrl] Voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
    else if (percentage < 0.0)
    {
        return; // no battery info
    }
    else
    {
        if ((rcv_stamp - last_print_t).toSec() > 1)
        {
            ROS_ERROR("[MPCCtrl] Dangerous! voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }
}

Rpm_Data_t::Rpm_Data_t()
{
    rcv_stamp = ros::Time(0);
}

void Rpm_Data_t::feed(mavros_msgs::ESCStatusConstPtr pMsg)
{
    if (pMsg->esc_status.size() < 4)
    {
        ROS_WARN_THROTTLE(1.0, "[MPCCtrl] /mavros/esc_status has less than 4 ESC rpm values.");
        return;
    }

    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    // /mavros/esc_status 来自 PX4 ESC_STATUS/BDShot。
    // rpm 为电机机械转速，单位 rpm。
    for (int i = 0; i < 4; ++i)
    {
        rpm[i] = pMsg->esc_status[i].rpm;
        rpm_vec(i) = rpm[i];
    }
    filtered_rpm = rpm_filter.apply(rpm_vec);
}

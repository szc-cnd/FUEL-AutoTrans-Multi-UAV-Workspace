#ifndef __INPUT_H
#define __INPUT_H

#include <ros/ros.h>
#include <Eigen/Dense>
#include <cstdint>

#include <sensor_msgs/Imu.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/ExtendedState.h>
#include <sensor_msgs/BatteryState.h>
#include <uav_utils/utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/ESCStatus.h>
#include <polynomial_trajectory.h>
#include <quadrotor_msgs/PolynomialTraj.h>
#include <deque>
#include "lowpassfilter2p.h"

class RC_Data_t
{
public:
  double mode;
  double gear;
  double land;
  double reboot_cmd;
  double last_mode;
  double last_gear;
  double last_land;
  double last_reboot_cmd;
  bool have_init_last_mode{false};
  bool have_init_last_gear{false};
  bool have_init_last_land{false};
  bool have_init_last_reboot_cmd{false};
  double ch[4];

  mavros_msgs::RCIn msg;
  ros::Time rcv_stamp;

  bool is_manual_mode;
  bool is_command_mode;
  bool enter_command_mode;
  bool is_hover_mode;
  bool enter_hover_mode;
  bool enter_land_mode;
  // CH8 低位请求 AUTO_TAKEOFF，中位保持 AUTO_HOVER，高位进入 CMD_CTRL；CH6 只由 QGC/PX4 处理 OFFBOARD。
  bool is_takeoff_mode;
  bool toggle_reboot;

  int mode_channel{7};
  int land_channel{9};
  int low_threshold{1300};
  int mid_low_threshold{1300};
  int mid_high_threshold{1700};
  int high_threshold{1800};

  static constexpr double REBOOT_THRESHOLD_VALUE = 0.5;
  static constexpr double DEAD_ZONE = 0.25;

  RC_Data_t();
  void set_mode_params(int mode_ch, int land_ch, int low_th, int mid_low_th,
                       int mid_high_th, int high_th);
  void check_validity();
  bool check_centered();
  void feed(mavros_msgs::RCInConstPtr pMsg);
  bool is_received(const ros::Time &now_time);
};

class Odom_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p{Eigen::Vector3d::Zero()};
  Eigen::Vector3d v{Eigen::Vector3d::Zero()};
  Eigen::Quaterniond q;
  Eigen::Vector3d w{Eigen::Vector3d::Zero()};

  nav_msgs::Odometry msg;
  ros::Time rcv_stamp{0};
  bool rcv_new_msg;

  Odom_Data_t();
  void feed(nav_msgs::OdometryConstPtr pMsg);
};

class Cmd_Trigger_Data_t
{
public:
  bool recv_cmd_trig;

  Cmd_Trigger_Data_t();
  void feed(geometry_msgs::PoseStampedConstPtr pMsg);
};

class Imu_Data_t
{
public:
  Eigen::Quaterniond q;
  Eigen::Vector3d w{Eigen::Vector3d::Zero()};
  Eigen::Vector3d a{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filtered_w{Eigen::Vector3d::Zero()};
  Eigen::Vector3d filtered_a{Eigen::Vector3d::Zero()};

  sensor_msgs::Imu msg;
  ros::Time rcv_stamp{0};

  LowPassFilter2p<Eigen::Vector3d> accel_filter;
  LowPassFilter2p<Eigen::Vector3d> gyro_filter;

  Imu_Data_t();
  void feed(sensor_msgs::ImuConstPtr pMsg);
  void set_filter_params(double accel_sample_freq, double accel_cutoff_freq, double gyro_sample_freq, double gyro_cutoff_freq)
  {
    accel_filter.set_cutoff_frequency(accel_sample_freq, accel_cutoff_freq);
    accel_filter.reset(a);
    gyro_filter.set_cutoff_frequency(gyro_sample_freq, gyro_cutoff_freq);
    gyro_filter.reset(w);
  }
};

class State_Data_t
{
public:
  mavros_msgs::State current_state;
  mavros_msgs::State state_before_offboard;
  ros::Time rcv_stamp{0};

  State_Data_t();
  void feed(mavros_msgs::StateConstPtr pMsg);
};

class ExtendedState_Data_t
{
public:
  mavros_msgs::ExtendedState current_extended_state;
  ros::Time rcv_stamp{0};

  ExtendedState_Data_t();
  void feed(mavros_msgs::ExtendedStateConstPtr pMsg);
};

struct oneTraj_Data_t
{
public:
  ros::Time traj_start_time{0};
  ros::Time traj_end_time{0};
  PayloadMPC::Trajectory traj;
  // FUEL 提供的 yaw 多项式，单位 rad；其导数由控制器计算为 yaw_rate(rad/s)。
  bool has_yaw{false};
  PayloadMPC::Trajectory yaw_traj;
};

class Trajectory_Data_t
{
public:
  uint32_t trajectory_id{0};
  ros::Time total_traj_start_time{0};
  ros::Time total_traj_end_time{0};
  int exec_traj = 0; // use for aborting the trajectory, 0 means no trajectory is executing
                     // -1 means the trajectory is aborting, 1 means the trajectory is executing
  std::deque<oneTraj_Data_t> traj_queue;
  // 正常轨迹结束后的悬停终点；只在完整 ACTION_ADD 校验成功后更新。
  Eigen::Vector3d ending_position{Eigen::Vector3d::Zero()};
  double ending_yaw{0.0};
  bool ending_yaw_valid{false};
  bool ending_pose_valid{false};

  Trajectory_Data_t();
  void clear();
  void abort();
  void adjust_end_time()
  {
    if (traj_queue.size() < 2)
    {
      return;
    }
    for (auto it = traj_queue.begin(); it != (traj_queue.end() - 1); it++)
    {
      it->traj_end_time = (it + 1)->traj_start_time;
    }
  }
  void feed(quadrotor_msgs::PolynomialTrajConstPtr pMsg);
};

class Command_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Vector3d p;
  Eigen::Vector3d v;
  Eigen::Vector3d a;
  Eigen::Vector3d j;
  double yaw;
  double yaw_rate;

  quadrotor_msgs::PositionCommand msg;
  ros::Time rcv_stamp{0};

  Command_Data_t();
  void feed(quadrotor_msgs::PositionCommandConstPtr pMsg);
};

class Battery_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  double volt{0.0};
  double percentage{0.0};

  sensor_msgs::BatteryState msg;
  ros::Time rcv_stamp{0};

  Battery_Data_t();
  void feed(sensor_msgs::BatteryStateConstPtr pMsg);
};

class Rpm_Data_t
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  mavros_msgs::ESCStatus msg;
  ros::Time rcv_stamp{0};
  int rpm[4]{0, 0, 0, 0};
  Eigen::Vector4d rpm_vec{Eigen::Vector4d::Zero()};
  Eigen::Vector4d filtered_rpm;
  LowPassFilter2p<Eigen::Vector4d> rpm_filter;

  Rpm_Data_t();
  void feed(mavros_msgs::ESCStatusConstPtr pMsg);
  void set_filter_params(double sample_freq, double cutoff_freq)
  {
    rpm_filter.set_cutoff_frequency(sample_freq, cutoff_freq);
    rpm_filter.reset(rpm_vec);
  }
};

#endif

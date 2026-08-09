#pragma once

#include <thread>

#include <Eigen/Eigen>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>

#include <queue>
#include "mpc_wrapper.h"
#include "mpc_params.h"
#include "polynomial_trajectory.h"

namespace PayloadMPC
{

  enum STATE
  {
    kPosX = 0,
    kPosY = 1,
    kPosZ = 2,
    kOriW = 3,
    kOriX = 4,
    kOriY = 5,
    kOriZ = 6,
    kVelX = 7,
    kVelY = 8,
    kVelZ = 9,

  };

  enum INPUT_BODYRATE
  {
    kThrust = 0,
    kRateX = 1,
    kRateY = 2,
    kRateZ = 3
  };

  struct TorquesAndThrust
  {
    Eigen::Vector3d body_torques;
    double collective_thrust;
  };

  class MpcController
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static_assert(kStateSize == 10,
                  "MpcController: Wrong model size. Number of states does not match.");
    static_assert(kInputSize == 4,
                  "MpcController: Wrong model size. Number of inputs does not match.");

    MpcController(MpcParams &params);

    void execMPC(const Eigen::Matrix<real_t, kStateSize, kSamples + 1> &reference_state,
                 const Eigen::Matrix<real_t, kInputSize, kSamples + 1> &reference_input,
                 const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                 Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                 Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs);
    void execMPC(const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                 Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                 Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs);
    bool solverFailurePersistent(double duration_sec) const;
    bool usingFallbackOutput() const { return using_fallback_output_; }
    bool lastMpcSolveSucceeded() const { return last_mpc_solve_success_; }
    void setHoverReference(const Eigen::Ref<const Eigen::Vector3d> &quad_position, const double yaw);
    // 保留 PositionCommand 入口参考函数供旧接口兼容；当前 AutoTrans 节点不订阅 PositionCommand，
    // 位置/速度/加速度使用 ENU 世界系，yaw 单位 rad，yaw_rate 单位 rad/s。
    bool setPositionCommandReference(const Eigen::Ref<const Eigen::Vector3d> &position,
                                     const Eigen::Ref<const Eigen::Vector3d> &velocity,
                                     const Eigen::Ref<const Eigen::Vector3d> &acceleration,
                                     const Eigen::Ref<const Eigen::Vector3d> &jerk,
                                     double yaw,
                                     double yaw_rate);
    // 根据位置多项式和可选的规划 yaw 生成 NMPC 参考；yaw_rate 单位 rad/s。
    void setTrajectoyReference(Trajectory &traj, double tstart, double start_yaw,
                               const Trajectory* yaw_traj = nullptr);
    double getTimeStep(){return mpc_time_step_;}
    void setDynamicParams(const real_t mass_q)
      {mpc_wrapper_.setDynamicParams(mass_q);}
    void setExternalForce(const Eigen::Ref<const Eigen::Vector3d>& fq)
      {mpc_wrapper_.setExternalForce(fq); fq_=fq;}
    // Thrust to control
    std::queue<std::pair<ros::Time, double>> timed_thrust;
    double thr_scale_compensate;
    const double rho2 = 0.998; // do not change
    double thrustscale_;
    double P;
    quadrotor_msgs::Px4ctrlDebug debug;

    void resetThrustMapping(void);
    // 清除尚未与 RPM 对齐的推力指令，但保留最后有效 thrustscale 和 RLS 协方差。
    void clearThrustCommandHistory(void);
    double convertThrust(const double& thrust, const double voltage);
    bool estimateThrustModel(
       const Eigen::Vector3d &est_a,
      const Eigen::Quaterniond &quad_q,
      const Eigen::Vector4d &rpm,
      const double voltage,
      const MpcParams &param);
    double AccurateThrustAccMapping(
    const double des_acc_z,
    double voltage,
    const MpcParams &param) const;

  private:
    double last_yaw_;
    double last_yaw_dot_;
    // Internal helper functions.

    // void offCallback(const std_msgs::Empty::ConstPtr& msg);
    void calculate_yaw(Eigen::Vector3d &vel, const double dt, double &yaw, double &yawdot);
    double inline rotor2thrust(const double& sqrt_kf, const Eigen::Vector4d& rpm);
    double inline acc2thrust(const double& Ml, const double& Mq,const double& l_length, const double& g, const double& acc_z,
                              const Eigen::Quaterniond& quad, const Eigen::Vector3d& cable, const Eigen::Vector3d& dcable);
    void computeQuadrotorFlatness(const Eigen::Vector3d &acc,
                                  const Eigen::Vector3d &jerk,
                                  double yaw,
                                  double yaw_dot,
                                  Eigen::Quaterniond &quat,
                                  double &thr,
                                  Eigen::Vector3d &omg) const;
    void preparationThread();
    double angle_limit(double ang);
    double angle_diff(double a, double b);

    // Parameters
    MpcParams& params_;

    Eigen::Vector3d fq_;
    // MPC
    MpcWrapper mpc_wrapper_;
    const double mpc_time_step_;

    // Preparation Thread
    std::thread preparation_thread_;

    // Variables
    real_t timing_feedback_, timing_preparation_;
    bool solve_from_scratch_;
    bool last_mpc_solve_success_{true};
    bool using_fallback_output_{false};
    bool has_last_valid_output_{false};
    ros::Time solver_failure_start_{0};
    Eigen::Matrix<real_t, kStateSize, kSamples + 1> last_valid_predicted_states_;
    Eigen::Matrix<real_t, kInputSize, kSamples> last_valid_control_inputs_;
    
  public:
    Eigen::Matrix<real_t, kStateSize, kSamples + 1> reference_states_;
    Eigen::Matrix<real_t, kInputSize, kSamples + 1> reference_inputs_;
    Eigen::Matrix<real_t, kInputSize, kSamples + 1> hover_input_;
  };

} // namespace MPC

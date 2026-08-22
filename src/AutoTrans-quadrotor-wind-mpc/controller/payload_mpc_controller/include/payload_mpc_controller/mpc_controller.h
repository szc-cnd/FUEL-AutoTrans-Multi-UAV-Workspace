#pragma once

#include <mutex>
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
    ~MpcController();

    void execMPC(const Eigen::Matrix<real_t, kStateSize, kSamples + 1> &reference_state,
                 const Eigen::Matrix<real_t, kInputSize, kSamples + 1> &reference_input,
                 const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                 Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                 Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs);
    void execMPC(const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                 Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                 Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs);
    void setHoverReference(const Eigen::Ref<const Eigen::Vector3d> &quad_position, const double yaw);
    void setTrajectoyReference(Trajectory &traj, double tstart,double start_yaw);
    double getTimeStep(){return mpc_time_step_;}
    void setDynamicParams(const real_t mass_q)
      {mpc_wrapper_.setDynamicParams(mass_q);}
    void setExternalForce(const Eigen::Ref<const Eigen::Vector3d>& fq);
    bool lastMpcSolveSuccessful() const { return last_mpc_solve_success_; }
    bool hasRecentValidControl(const ros::Time &now, double max_age) const;
    const Eigen::Matrix<real_t, kInputSize, 1> &lastValidControlInput() const
    {
      return last_valid_control_input_;
    }
    void clearLastValidControl();
    // 等待异步准备线程结束，再基于实测状态和固定悬停参考重建完整 ACADO 工作区。
    bool resetForHover(
      const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> estimated_state,
      const Eigen::Ref<const Eigen::Vector3d> hover_position,
      double hover_yaw);
    bool restoreNominalVelocityLimits();
    bool recoveryVelocityLimitsRelaxed() const
      { return recovery_velocity_limits_relaxed_; }
    void waitForPreparation();
    // Thrust to control
    std::queue<std::pair<ros::Time, double>> timed_thrust;
    double thr_scale_compensate;
    const double rho2 = 0.998; // do not change
    double thrustscale_{0.0};
    double P;
    quadrotor_msgs::Px4ctrlDebug debug;

    void resetThrustMapping(void);
    // 返回当前在线推力映射对应的悬停比例，不向 RLS 历史队列写入样本。
    double currentHoverPercentage() const;
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
    double last_yaw_{0.0};
    double last_yaw_dot_{0.0};
    bool yaw_reference_initialized_{false};
    // Internal helper functions.

    // void offCallback(const std_msgs::Empty::ConstPtr& msg);
    void calculate_yaw(const Eigen::Vector3d &vel, const double dt,
                       double &yaw_state, double &yawdot_state);
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
    std::mutex external_force_mutex_;
    // MPC
    MpcWrapper mpc_wrapper_;
    const double mpc_time_step_;

    // Preparation Thread
    std::thread preparation_thread_;

    // Variables
    real_t timing_feedback_, timing_preparation_;
    bool solve_from_scratch_;
    bool last_mpc_solve_success_{true};
    bool recovery_velocity_limits_relaxed_{false};
    // 仅用于记录一次“求解失败 -> 有效输出恢复”的诊断边沿，不参与 NMPC 控制计算。
    bool mpc_failure_active_{false};
    // 仅缓存最近一次通过有限值检查的 MPC 首个控制输入，用于短时故障保持。
    Eigen::Matrix<real_t, kInputSize, 1> last_valid_control_input_{
        Eigen::Matrix<real_t, kInputSize, 1>::Zero()};
    ros::Time last_valid_control_time_{0};
    
  public:
    Eigen::Matrix<real_t, kStateSize, kSamples + 1> reference_states_;
    Eigen::Matrix<real_t, kInputSize, kSamples + 1> reference_inputs_;
    Eigen::Matrix<real_t, kInputSize, kSamples + 1> hover_input_;
  };

} // namespace MPC

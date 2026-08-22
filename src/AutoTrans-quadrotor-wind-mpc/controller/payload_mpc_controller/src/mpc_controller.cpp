#include "mpc_controller.h"
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <ctime>
#include <cmath>

namespace PayloadMPC
{

  MpcController::MpcController(MpcParams &params) : params_(params), mpc_time_step_(params_.step_T_)
  {
    auto & q_gain_ = params_.q_gain_;
    auto & r_gain_ = params_.r_gain_;
    
    mpc_wrapper_.setDynamicParams(params_.dyn_params_.mass_q);
    Eigen::Matrix<real_t, kCostSize, kCostSize> Q = (Eigen::Matrix<real_t, kCostSize, 1>() << q_gain_.Q_pos_xy, q_gain_.Q_pos_xy, q_gain_.Q_pos_z,
                                                     q_gain_.Q_attitude_rp, q_gain_.Q_attitude_rp, q_gain_.Q_attitude_rp, q_gain_.Q_attitude_yaw,
                                                     q_gain_.Q_velocity, q_gain_.Q_velocity, q_gain_.Q_velocity)
                                                        .finished()
                                                        .asDiagonal();
    Eigen::Matrix<real_t, kInputSize, kInputSize> R = (Eigen::Matrix<real_t, kInputSize, 1>() << r_gain_.R_thrust, r_gain_.R_pitchroll, r_gain_.R_pitchroll, r_gain_.R_yaw).finished().asDiagonal();

    Eigen::Matrix<real_t, kStateSize, 1> initial_state = (Eigen::Matrix<real_t, kStateSize, 1>() << 0.0, 0.0, 0.0,
                                                          1.0, 0.0, 0.0, 0.0,
                                                          0.0, 0.0, 0.0)
                                                             .finished();
    reference_states_ = initial_state.replicate(1,kSamples+1);
    Eigen::Matrix<real_t, kInputSize, 1> initial_input = (Eigen::Matrix<real_t, kInputSize, 1>() << params_.dyn_params_.mass_q * params_.gravity_, 0, 0, 0).finished();

    hover_input_ = initial_input.replicate(1, kSamples + 1);
    reference_inputs_ = hover_input_;

    mpc_wrapper_.initialize(Q, R, initial_state, initial_input, params_.state_cost_exponential_, params_.input_cost_exponential_);
    fq_.setZero();
    mpc_wrapper_.setExternalForce(fq_);
    mpc_wrapper_.setLimits(
        params_.min_thrust_, params_.max_thrust_,
        params_.max_bodyrate_xy_, params_.max_bodyrate_z_,
        params_.max_velocity_xy_, params_.max_velocity_z_);

    // first_traj_received_ = false;
    solve_from_scratch_ = false;
    timing_feedback_ = 0;
    timing_preparation_ = 0;
    preparation_thread_ = std::thread(&MpcController::preparationThread, this);
    
    
  }

  MpcController::~MpcController()
  {
    waitForPreparation();
  }

  void MpcController::execMPC(const Eigen::Matrix<real_t, kStateSize, kSamples + 1> &reference_states,
                              const Eigen::Matrix<real_t, kInputSize, kSamples + 1> &reference_inputs,
                              const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                              Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                              Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs)
  {
    const clock_t start = clock();

    if (preparation_thread_.joinable())
      preparation_thread_.join(); // waiting the preparation_thread_ finished

    // Get the feedback from MPC.

    mpc_wrapper_.setTrajectory(reference_states, reference_inputs);

    if (solve_from_scratch_)
    {
      ROS_INFO("[NMPC] 使用悬停参考作为初始猜测求解。");
      last_mpc_solve_success_ = mpc_wrapper_.solve(estimated_state);
      solve_from_scratch_ = false;
    }
    else
    {
      constexpr bool do_preparation_step(false); // the preparation step has been done by another thread
      last_mpc_solve_success_ = mpc_wrapper_.update(estimated_state, do_preparation_step);
    }

    if (!last_mpc_solve_success_)
    {
      mpc_failure_active_ = true;
      // 这里只写入有限占位值；FSM 会在恢复状态发布拉平姿态目标。
      predicted_states = estimated_state.replicate(1, kSamples + 1);
      control_inputs = hover_input_.leftCols(kSamples);
      ROS_ERROR_THROTTLE(5.0, "[OUTPUT] NMPC 求解失败，等待恢复状态输出安全姿态。");
    }
    else
    {
      mpc_wrapper_.getStates(predicted_states);
      mpc_wrapper_.getInputs(control_inputs);
      if (!predicted_states.allFinite() || !control_inputs.allFinite())
      {
        last_mpc_solve_success_ = false;
        mpc_failure_active_ = true;
        predicted_states = estimated_state.replicate(1, kSamples + 1);
        control_inputs = hover_input_.leftCols(kSamples);
        ROS_ERROR_THROTTLE(5.0, "[OUTPUT] NMPC 输出含非有限值，等待恢复状态输出安全姿态。");
      }
      else
      {
        last_valid_control_input_ = control_inputs.col(0);
        last_valid_control_time_ = ros::Time::now();
        if (mpc_failure_active_)
        {
          ROS_INFO("[OUTPUT] NMPC 恢复正常。");
          mpc_failure_active_ = false;
        }
      }
    }

    // Start a thread to prepare for the next execution.
    preparation_thread_ = std::thread(&MpcController::preparationThread, this);

    // Timing
    const clock_t end = clock();
    timing_feedback_ = 0.9*timing_feedback_ + 0.1* double(end - start) / CLOCKS_PER_SEC;
    if (params_.print_info_)
    ROS_INFO_THROTTLE(5.0, "[NMPC] 计算耗时：反馈延迟=%1.2f ms，总耗时=%1.2f ms。",
                      timing_feedback_ * 1000, (timing_feedback_ + timing_preparation_) * 1000);
  }

  void MpcController::execMPC(const Eigen::Matrix<real_t, kStateSize, 1> &estimated_state,
                              Eigen::Matrix<real_t, kStateSize, kSamples + 1> &predicted_states,
                              Eigen::Matrix<real_t, kInputSize, kSamples> &control_inputs)
  {
    execMPC(reference_states_, reference_inputs_, estimated_state, predicted_states, control_inputs);
  }

  void MpcController::setExternalForce(
      const Eigen::Ref<const Eigen::Vector3d> &fq)
  {
    std::lock_guard<std::mutex> lock(external_force_mutex_);
    fq_ = fq;
  }

  bool MpcController::resetForHover(
      const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> estimated_state,
      const Eigen::Ref<const Eigen::Vector3d> hover_position,
      double hover_yaw)
  {
    if (!estimated_state.allFinite() || !hover_position.allFinite() || !std::isfinite(hover_yaw))
    {
      ROS_ERROR("[NMPC恢复] 无法重置求解器：状态或悬停参考不是有限值。");
      return false;
    }

    // ACADO 使用全局工作区；完整重置前必须等待上一轮准备线程结束。
    if (preparation_thread_.joinable())
      preparation_thread_.join();

    const auto &q_gain = params_.q_gain_;
    const auto &r_gain = params_.r_gain_;
    const Eigen::Matrix<real_t, kCostSize, kCostSize> Q =
        (Eigen::Matrix<real_t, kCostSize, 1>()
             << q_gain.Q_pos_xy, q_gain.Q_pos_xy, q_gain.Q_pos_z,
             q_gain.Q_attitude_rp, q_gain.Q_attitude_rp, q_gain.Q_attitude_rp,
             q_gain.Q_attitude_yaw, q_gain.Q_velocity, q_gain.Q_velocity,
             q_gain.Q_velocity)
            .finished()
            .asDiagonal();
    const Eigen::Matrix<real_t, kInputSize, kInputSize> R =
        (Eigen::Matrix<real_t, kInputSize, 1>()
             << r_gain.R_thrust, r_gain.R_pitchroll, r_gain.R_pitchroll, r_gain.R_yaw)
            .finished()
            .asDiagonal();
    const Eigen::Matrix<real_t, kInputSize, 1> initial_input =
        (Eigen::Matrix<real_t, kInputSize, 1>()
             << params_.dyn_params_.mass_q * params_.gravity_, 0.0, 0.0, 0.0)
            .finished();

    Eigen::Matrix<real_t, kStateSize, 1> reset_state = estimated_state;
    Eigen::Quaterniond measured_attitude(
        reset_state(kOriW), reset_state(kOriX), reset_state(kOriY), reset_state(kOriZ));
    if (!measured_attitude.coeffs().allFinite() || measured_attitude.norm() <= 1.0e-6)
    {
      ROS_ERROR("[NMPC恢复] 无法重置求解器：测量姿态四元数无效。");
      return false;
    }
    measured_attitude.normalize();
    Eigen::Quaterniond hover_attitude(Eigen::AngleAxisd(hover_yaw, Eigen::Vector3d::UnitZ()));
    if (measured_attitude.coeffs().dot(hover_attitude.coeffs()) < 0.0)
      measured_attitude.coeffs() = -measured_attitude.coeffs();
    reset_state(kOriW) = measured_attitude.w();
    reset_state(kOriX) = measured_attitude.x();
    reset_state(kOriY) = measured_attitude.y();
    reset_state(kOriZ) = measured_attitude.z();

    const double measured_velocity_xy = std::max(
        std::abs(static_cast<double>(reset_state(kVelX))),
        std::abs(static_cast<double>(reset_state(kVelY))));
    const double measured_velocity_z = std::abs(static_cast<double>(reset_state(kVelZ)));
    const real_t recovery_velocity_xy = static_cast<real_t>(std::max(
        static_cast<double>(params_.max_velocity_xy_),
        measured_velocity_xy + params_.safety_.mpc_recovery_velocity_margin_xy));
    const real_t recovery_velocity_z = static_cast<real_t>(std::max(
        static_cast<double>(params_.max_velocity_z_),
        measured_velocity_z + params_.safety_.mpc_recovery_velocity_margin_z));

    mpc_wrapper_.setDynamicParams(params_.dyn_params_.mass_q);
    recovery_velocity_limits_relaxed_ = false;
    mpc_wrapper_.initialize(Q, R, reset_state, initial_input,
                            params_.state_cost_exponential_, params_.input_cost_exponential_);
    const bool limits_ok = mpc_wrapper_.setLimits(
        params_.min_thrust_, params_.max_thrust_,
        params_.max_bodyrate_xy_, params_.max_bodyrate_z_,
        recovery_velocity_xy, recovery_velocity_z);
    recovery_velocity_limits_relaxed_ = limits_ok &&
        (recovery_velocity_xy > params_.max_velocity_xy_ + 1.0e-6 ||
         recovery_velocity_z > params_.max_velocity_z_ + 1.0e-6);
    setExternalForce(Eigen::Vector3d::Zero());
    mpc_wrapper_.setExternalForce(Eigen::Vector3d::Zero());
    hover_input_ = initial_input.replicate(1, kSamples + 1);
    setHoverReference(hover_position, hover_yaw);
    const bool reference_ok = mpc_wrapper_.setTrajectory(reference_states_, reference_inputs_);

    solve_from_scratch_ = false;
    // 重置只完成求解器准备，不代表本周期已经得到有效 MPC 解。
    last_mpc_solve_success_ = false;
    mpc_failure_active_ = false;
    preparation_thread_ = std::thread(&MpcController::preparationThread, this);
    ROS_WARN("[NMPC恢复] 已基于当前状态完整重置 ACADO 求解器。");
    return limits_ok && reference_ok;
  }

  bool MpcController::restoreNominalVelocityLimits()
  {
    if (!recovery_velocity_limits_relaxed_)
    {
      // 离开自动状态时也要用最新缓存外力重新准备，避免下次进入时沿用旧 OnlineData。
      last_mpc_solve_success_ = false;
      if (!preparation_thread_.joinable())
        preparation_thread_ = std::thread(&MpcController::preparationThread, this);
      return true;
    }

    if (preparation_thread_.joinable())
      preparation_thread_.join();
    const bool limits_ok = mpc_wrapper_.setLimits(
        params_.min_thrust_, params_.max_thrust_,
        params_.max_bodyrate_xy_, params_.max_bodyrate_z_,
        params_.max_velocity_xy_, params_.max_velocity_z_);
    if (limits_ok)
    {
      recovery_velocity_limits_relaxed_ = false;
      // 改回正常硬约束后，必须重新准备并在新约束下成功求解，旧解不能计入恢复次数。
      last_mpc_solve_success_ = false;
      preparation_thread_ = std::thread(&MpcController::preparationThread, this);
      ROS_INFO("[NMPC恢复] 实际速度已回到正常范围，恢复配置的速度硬约束。");
    }
    return limits_ok;
  }

  void MpcController::waitForPreparation()
  {
    if (preparation_thread_.joinable())
      preparation_thread_.join();
  }

  bool MpcController::hasRecentValidControl(const ros::Time &now, double max_age) const
  {
    if (max_age < 0.0 || last_valid_control_time_.isZero() ||
        !last_valid_control_input_.allFinite())
      return false;
    const double age = (now - last_valid_control_time_).toSec();
    return age >= 0.0 && age <= max_age;
  }

  void MpcController::clearLastValidControl()
  {
    last_valid_control_time_ = ros::Time(0);
    last_valid_control_input_.setZero();
  }
  // drone pos
  void MpcController::setHoverReference(const Eigen::Ref<const Eigen::Vector3d> &quad_position, const double yaw)
  {
    // HOVER、起飞和安全恢复都以锁存的实际航向重新建立偏航状态；
    // POLY_TRAJ 内的新轨迹不会调用这里，因此跨重规划保持连续。
    last_yaw_ = angle_limit(yaw);
    last_yaw_dot_ = 0.0;
    yaw_reference_initialized_ = true;

    Eigen::Matrix<real_t, 3, 1> quad_pos = quad_position.cast<real_t>();
    Eigen::Quaterniond quad_q;
    double thr;
    Eigen::Vector3d omg_;
    computeQuadrotorFlatness(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), yaw, 0.0, quad_q, thr, omg_);
    Eigen::Quaternion<real_t> q = quad_q.cast<real_t>();
    
    Eigen::Matrix<real_t, kStateSize, 1> reference_state;
    q.normalize();
    reference_state << quad_pos.x(), quad_pos.y(), quad_pos.z(),
        q.w(), q.x(), q.y(), q.z(),
        0.0, 0.0, 0.0;

    Eigen::Matrix<real_t, kInputSize, 1> hover_in = hover_input_.col(0);
    hover_in(kThrust,0)=thr;
    reference_states_ = reference_state.replicate(1, kSamples + 1);
    reference_inputs_ = hover_in.replicate(1, kSamples + 1);
  }

  void MpcController::setTrajectoyReference(Trajectory &traj, double tstart, double start_yaw)
  {
    const double t_step = mpc_time_step_;
    double t_all = traj.getTotalDuration() - 1.0e-3;
    double t = tstart;
    double yaw, yaw_dot;
    Eigen::Vector3d pos, vel, acc, jerk, snap, crackle;

    Eigen::Vector3d pos_quad, vel_quad, acc_quad, jerk_quad;
    Eigen::Quaterniond quat;
    double thr;
    Eigen::Vector3d omg;

    if (!yaw_reference_initialized_)
    {
      last_yaw_ = angle_limit(start_yaw);
      last_yaw_dot_ = 0.0;
      yaw_reference_initialized_ = true;
    }

    // 预测窗口使用局部偏航状态。成员状态只在 i==0 时推进一次，
    // 不能把窗口末端的预测偏航写回当前控制周期。
    double predicted_yaw = last_yaw_;
    double predicted_yaw_dot = last_yaw_dot_;

    for (int i = 0; i < (kSamples + 1); i++)
    {
      Eigen::MatrixXd pvajs;
      if (t > t_all)
      { // if t is larger than the total time, use the last point
        // TODO : Consider 2 trajectories
        t = t_all;
        pvajs = traj.getPVAJSC(t);
        pos = pvajs.col(0);
        vel = Eigen::Vector3d::Zero();
        acc =  Eigen::Vector3d::Zero();
        jerk = Eigen::Vector3d::Zero();
        snap = Eigen::Vector3d::Zero();
        crackle =  Eigen::Vector3d::Zero();
      }
      else
      {
        pvajs = traj.getPVAJSC(t);
        pos = pvajs.col(0);
        vel = pvajs.col(1);
        acc = pvajs.col(2);
        jerk = pvajs.col(3);
        snap = pvajs.col(4);
        crackle = pvajs.col(5);
      }
  
      // 普通四旋翼版本中，输入轨迹直接表示无人机参考轨迹，不再表示负载轨迹。
      pos_quad = pos;
      vel_quad = vel;
      acc_quad = acc;
      jerk_quad = jerk;
      if (params_.use_fix_yaw_)
      {
        yaw = start_yaw;
        yaw_dot = 0.0; 
      }
      else
      {
        if (i == 0)
        {
          calculate_yaw(vel_quad, t_step, last_yaw_, last_yaw_dot_);
          predicted_yaw = last_yaw_;
          predicted_yaw_dot = last_yaw_dot_;
        }
        else
        {
          calculate_yaw(vel_quad, t_step, predicted_yaw, predicted_yaw_dot);
        }
        yaw = predicted_yaw;
        yaw_dot = predicted_yaw_dot;
      }
      computeQuadrotorFlatness(acc_quad, jerk_quad, yaw, yaw_dot, quat, thr, omg);

      Eigen::Matrix<real_t, kStateSize, 1> reference_state;
      reference_state << pos_quad.x(), pos_quad.y(), pos_quad.z(),
          quat.w(), quat.x(), quat.y(), quat.z(),
          vel_quad.x(), vel_quad.y(), vel_quad.z();
      reference_states_.col(i) = reference_state;

      Eigen::Matrix<real_t, kInputSize, 1> reference_input;
      reference_input << thr, omg.x(), omg.y(), omg.z();
      reference_inputs_.col(i) = reference_input;
      t += t_step;
    }
  }

  void MpcController::preparationThread()
  {
    const clock_t start = clock();

    Eigen::Vector3d external_force;
    {
      std::lock_guard<std::mutex> lock(external_force_mutex_);
      external_force = fq_;
    }
    // ACADO OnlineData 只在准备线程或已 join 的完整重置路径中写入。
    mpc_wrapper_.setExternalForce(external_force);
    mpc_wrapper_.prepare();

    // Timing
    const clock_t end = clock();
    timing_preparation_ = timing_preparation_*0.9 + 0.1* double(end - start) / CLOCKS_PER_SEC;
  }

  double MpcController::angle_limit(double ang)
  {
    while (ang > M_PI)
    {
      ang -= 2.0 * M_PI;
    }
    while (ang <= -M_PI)
    {
      ang += 2.0 * M_PI;
    }
    return ang;
  }
  double MpcController::angle_diff(double a, double b)
  {
    double d1, d2;
    d1 = a-b;
    d2 = 2*M_PI - fabs(d1);
    if(d1 > 0)
      d2 *= -1.0;
    if(fabs(d1) < fabs(d2))
      return(d1);
    else
      return(d2);
  }
  void MpcController::calculate_yaw(const Eigen::Vector3d &vel, const double dt,
                                    double &yaw_state, double &yawdot_state)
  {
    const double YAW_DOT_MAX_PER_SEC = params_.max_bodyrate_z_;
    constexpr double kYawHoldSpeed = 0.12;

    double yaw_temp;
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * dt;

    // tangent line
    if (vel.head<2>().norm() < kYawHoldSpeed)
    {
      yaw_temp = yaw_state;
    }
    else
    {
      yaw_temp = atan2(vel(1), vel(0));     
    }
    double yaw_diff = angle_diff(yaw_temp, yaw_state);
    
    if (yaw_diff > max_yaw_change )
    {
      yaw_diff = max_yaw_change;
      yawdot_state = YAW_DOT_MAX_PER_SEC;
    }
    else if(yaw_diff < -max_yaw_change)
    {
      yaw_diff = -max_yaw_change;
      yawdot_state = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yawdot_state = yaw_diff / dt;
    }

    yaw_state = angle_limit(yaw_state + yaw_diff);
  }

  void MpcController::computeQuadrotorFlatness(const Eigen::Vector3d &acc,
                                               const Eigen::Vector3d &jerk,
                                               double yaw,
                                               double yaw_dot,
                                               Eigen::Quaterniond &quat,
                                               double &thr,
                                               Eigen::Vector3d &omg) const
  {
    // F_des 是世界系期望总力，单位 N；f_Q 为世界系外力估计，单位 N。
    Eigen::Vector3d force_des =
        params_.dyn_params_.mass_q * (acc + Eigen::Vector3d(0.0, 0.0, params_.gravity_)) - fq_;
    if (force_des.norm() < 1.0e-6)
    {
      force_des = Eigen::Vector3d(0.0, 0.0, params_.dyn_params_.mass_q * params_.gravity_);
    }

    Eigen::Vector3d b3 = force_des.normalized();
    Eigen::Vector3d b1_yaw(cos(yaw), sin(yaw), 0.0);
    Eigen::Vector3d b2 = b3.cross(b1_yaw);
    if (b2.norm() < 1.0e-6)
    {
      b2 = Eigen::Vector3d(-sin(yaw), cos(yaw), 0.0);
    }
    b2.normalize();
    Eigen::Vector3d b1 = b2.cross(b3);
    b1.normalize();

    Eigen::Matrix3d R;
    R.col(0) = b1;
    R.col(1) = b2;
    R.col(2) = b3;

    quat = Eigen::Quaterniond(R);
    quat.normalize();
    thr = force_des.dot(b3);

    Eigen::Vector3d force_dot = params_.dyn_params_.mass_q * jerk;
    Eigen::Vector3d h = (force_dot - b3 * (b3.dot(force_dot))) / std::max(thr, 1.0e-6);
    // body_rate.x/y/z 是机体系角速度参考，单位 rad/s，后续直接发给 MAVROS AttitudeTarget。
    omg.x() = -h.dot(b2);
    omg.y() = h.dot(b1);
    omg.z() = yaw_dot * b3.z();
  }

  double MpcController::convertThrust(const double& thrust, const double voltage)
  {
    (void)voltage; // 模式 1 通过 RPM 间接适应电压变化，当前不直接使用电压模型。
    // double des_acc_norm = thrust;
    // This compensates for an acceleration component in thrust direction due
    // to the square of the body-horizontal velocity.
    // des_acc_norm -= param.rt_drag.k_thrust_horz * (pow(est_v.x(), 2) + pow(est_v.y(), 2));

    if (!std::isfinite(thrustscale_) || thrustscale_ <= 0.0)
    {
      ROS_ERROR_THROTTLE(5.0, "[推力映射] thrustscale 无效，恢复配置的悬停映射。");
      resetThrustMapping();
    }

    const double normalized_thrust_raw = thrust / thrustscale_;
    if (!std::isfinite(normalized_thrust_raw))
    {
      ROS_ERROR_THROTTLE(
          1.0,
          "[MPC CTRL] NORMALIZED_THRUST_SATURATED non-finite raw command; publishing zero.");
      clearThrustCommandHistory();
      return 0.0;
    }

    const double normalized_thrust = std::max(
        0.0, std::min(normalized_thrust_raw, params_.thr_map_.max_normalized_thrust));
    const bool saturated =
        std::abs(normalized_thrust - normalized_thrust_raw) > 1.0e-9;

    if (saturated)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "[MPC CTRL] NORMALIZED_THRUST_SATURATED raw=%.4f sent=%.4f limit=%.4f",
          normalized_thrust_raw, normalized_thrust, params_.thr_map_.max_normalized_thrust);
    }
    else if (params_.thr_map_.accurate_thrust_model == 1)
    {
      // 只记录实际发送且未饱和的归一化推力，供 35~45 ms 后与原始 RPM 对齐。
      timed_thrust.emplace(std::make_pair(ros::Time::now(), normalized_thrust));
    }

    return normalized_thrust;
  }

  bool MpcController::estimateThrustModel(
      const Eigen::Vector3d &est_a,
      const Eigen::Quaterniond &quad_q,
      const Eigen::Vector4d &rpm,
      const double voltage,
      const MpcParams &param)
  {
    (void)est_a;
    (void)quad_q;
    (void)voltage;

    ros::Time t_now = ros::Time::now();
    while (timed_thrust.size() >= 1)
    {
      // Choose data before 35~45ms ago
      std::pair<ros::Time, double> t_t = timed_thrust.front();
      double time_passed = (t_now - t_t.first).toSec();
      if (time_passed > 0.045) // 45ms
      {
        // printf("continue, time_passed=%f\n", time_passed);
        timed_thrust.pop();
        continue;
      }
      if (time_passed < 0.035) // 35ms
      {
        // printf("skip, time_passed=%f\n", time_passed);
        return false;
      }

      /***********************************************************/
      /* Recursive least squares algorithm with vanishing memory */
      /***********************************************************/
      double normlized_thr = t_t.second;
      timed_thrust.pop();
      if (param.thr_map_.accurate_thrust_model != 1)
      {
        clearThrustCommandHistory();
        return false;
      }

      if (!rpm.allFinite() || rpm.minCoeff() < param.thr_map_.min_learning_rpm ||
          !std::isfinite(normlized_thr) || normlized_thr <= 0.0 ||
          normlized_thr > param.thr_map_.max_normalized_thrust)
      {
        ROS_WARN_THROTTLE(5.0, "[推力映射] 拒绝 RLS 更新：RPM 或归一化推力样本无效。");
        clearThrustCommandHistory();
        return false;
      }

      const double thr_fb = rotor2thrust(param.force_estimator_param_.sqrt_kf, rpm);
      const double denominator = rho2 + normlized_thr * P * normlized_thr;
      if (!std::isfinite(thr_fb) || thr_fb <= 0.0 ||
          !std::isfinite(thrustscale_) || thrustscale_ <= 0.0 ||
          !std::isfinite(P) || P <= 0.0 ||
          !std::isfinite(denominator) || denominator <= 0.0)
      {
        ROS_WARN_THROTTLE(5.0, "[推力映射] 拒绝 RLS 更新：估计器状态无效。");
        clearThrustCommandHistory();
        return false;
      }

      /***********************************************************/
      /* Model: physical thrust = thrustscale_ * normalized thrust */
      /***********************************************************/
      const double gamma = 1.0 / denominator;
      const double K = gamma * P * normlized_thr;
      const double raw_thrustscale =
          thrustscale_ + K * (thr_fb - normlized_thr * thrustscale_);
      const double candidate_thrustscale =
          param.thr_map_.filter_factor * raw_thrustscale +
          (1.0 - param.thr_map_.filter_factor) * thrustscale_;
      const double candidate_P = (1.0 - K * normlized_thr) * P / rho2;

      const double weight = param.gravity_ * param.dyn_params_.mass_q;
      const double min_thrustscale = weight / 0.8;
      const double max_thrustscale = weight / 0.1;
      const double previous_hover_percentage = weight / thrustscale_;
      const double candidate_hover_percentage = weight / candidate_thrustscale;

      const bool candidate_valid =
          std::isfinite(candidate_thrustscale) &&
          candidate_thrustscale > 0.0 &&
          std::isfinite(candidate_P) && candidate_P > 0.0 &&
          std::isfinite(candidate_hover_percentage) &&
          candidate_thrustscale >= min_thrustscale &&
          candidate_thrustscale <= max_thrustscale &&
          candidate_hover_percentage >= 0.1 &&
          candidate_hover_percentage <= 0.8 &&
          std::abs(candidate_hover_percentage - previous_hover_percentage) <=
              param.thr_map_.max_hover_percentage_step;
      if (!candidate_valid)
      {
        ROS_WARN_THROTTLE(
            1.0,
            "[MPC CTRL] RLS_REJECTED candidate hover=%.4f previous=%.4f scale=%.3f",
            candidate_hover_percentage, previous_hover_percentage, candidate_thrustscale);
        clearThrustCommandHistory();
        return false;
      }

      // 候选值全部通过检查后再同时提交，拒绝更新时保留上一组有效 thrustscale/P。
      thrustscale_ = candidate_thrustscale;
      P = candidate_P;
      debug.hover_percentage = candidate_hover_percentage;
      if (param.thr_map_.print_val)
      {
        ROS_INFO_THROTTLE(
            1.0,
            "[MPC CTRL] thrust model ACTIVE: thrustscale=%.3f N hover=%.4f rpm_thrust=%.3f N normalized=%.4f",
            thrustscale_, debug.hover_percentage, thr_fb, normlized_thr);
      }

      return true;
    }

    return false;
  }

  double inline MpcController::rotor2thrust(const double& sqrt_kf, const Eigen::Vector4d& rpm)
  {
    return (sqrt_kf * rpm).squaredNorm();
  }
  double inline MpcController::acc2thrust(const double& Ml, const double& Mq,const double& l_length, const double& g, const double& acc_z,
                              const Eigen::Quaterniond& quad, const Eigen::Vector3d& cable, const Eigen::Vector3d& dcable)

{
  const double q_w = quad.w();
  const double q_x = quad.x();
  const double q_y = quad.y();
  const double q_z = quad.z();
  const double cable_x = cable.x();
  const double cable_y = cable.y();
  const double cable_z = cable.z();
  
  const double q_sqr = q_w*q_w - q_x*q_x - q_y*q_y + q_z*q_z;
  const double dcable_sqr = dcable.squaredNorm();//dcable_x^2 + dcable_y^2 + dcable_z^2;
  const double qwqxminusqyqz = 2*q_w*q_x - 2*q_y*q_z;
  const double qwqyminusqxqz = 2*q_w*q_y + 2*q_x*q_z;

  double des_T = -(Mq*(acc_z - g*(q_sqr) + (g*(Ml + Mq)*(q_sqr) - Ml*(g + (Mq*cable_z*l_length*(dcable_sqr))/(Ml + Mq))*(q_sqr) - (Ml*Mq*cable_x*l_length*(qwqyminusqxqz)*(dcable_sqr))/(Ml + Mq) + (Ml*Mq*cable_y*l_length*(qwqxminusqyqz)*(dcable_sqr))/(Ml + Mq))/Mq))/((Ml*cable_x*(qwqyminusqxqz)*(cable_z*(q_sqr) - cable_y*(qwqxminusqyqz) + cable_x*(qwqyminusqxqz)))/(Ml + Mq) - (Ml*cable_y*(qwqxminusqyqz)*(cable_z*(q_sqr) - cable_y*(qwqxminusqyqz) + cable_x*(qwqyminusqxqz)))/(Ml + Mq) + (Ml*cable_z*(cable_z*(q_sqr) - cable_y*(qwqxminusqyqz) + cable_x*(qwqyminusqxqz))*(q_sqr))/(Ml + Mq) - 1);

  return des_T; //accelration in body z axis(including g)
}

  void MpcController::resetThrustMapping(void)
  {
    thrustscale_ = (params_.gravity_ * (params_.dyn_params_.mass_q)) / params_.thr_map_.hover_percentage;
    thr_scale_compensate = 1.0;
    P = 1e6;
    clearThrustCommandHistory();
    debug.hover_percentage = params_.thr_map_.hover_percentage;
  }

  double MpcController::currentHoverPercentage() const
  {
    const double weight = params_.gravity_ * params_.dyn_params_.mass_q;
    if (!std::isfinite(weight) || weight <= 0.0 ||
        !std::isfinite(thrustscale_) || thrustscale_ <= 0.0)
    {
      return params_.thr_map_.hover_percentage;
    }
    const double hover_percentage = weight / thrustscale_;
    if (!std::isfinite(hover_percentage) || hover_percentage < 0.1 ||
        hover_percentage > 0.8)
    {
      return params_.thr_map_.hover_percentage;
    }
    return hover_percentage;
  }

  void MpcController::clearThrustCommandHistory(void)
  {
    std::queue<std::pair<ros::Time, double>> empty;
    timed_thrust.swap(empty);
  }

}

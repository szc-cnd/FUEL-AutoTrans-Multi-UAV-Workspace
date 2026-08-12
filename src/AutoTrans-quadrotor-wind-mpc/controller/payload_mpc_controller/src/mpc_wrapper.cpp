/**
 * @file mpc_wrapper.cpp
 * @author Haojia Li (hlied@connect.ust.hk)
 * @brief MPC interface. Wrapper for ACADO.
 * Thanks for rpg_mpc
 * @version 1.0
 * @date 2022-07-09
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "mpc_wrapper.h"
#include <cmath>

namespace PayloadMPC
{
  ACADOvariables acadoVariables;
  ACADOworkspace acadoWorkspace;

  MpcWrapper::MpcWrapper()
  {
    ;
    // Please use the initialize function to initialize the MPC
    // const Eigen::Matrix<real_t, kStateSize, 1> hover_state =
    //   (Eigen::Matrix<real_t, kStateSize, 1>() << 0.0, 0.0, 0.6,
    //                                         1.0, 0.0, 0.0, 0.0,
    //                                         0.0, 0.0, 0.0,
    //                                         0.0, 0.0, 0.0,
    //                                         0.0, 0.0, 0.0,
    //                                         0.0, 0.0, -1.0,
    //                                         0.0, 0.0, 0.0).finished();
  }

  void MpcWrapper::initialize(
      const Eigen::Ref<const Eigen::Matrix<real_t, kCostSize, kCostSize>> &Q,
      const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, kInputSize>> &R,
      const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> &initial_state,
      const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, 1>> &initial_input,
      real_t state_cost_scaling,
      real_t input_cost_scaling)
  // Clear solver memory.
  {
    memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
    memset(&acadoVariables, 0, sizeof(acadoVariables));

    setCosts(Q, R, state_cost_scaling, input_cost_scaling);
    setDynamicParams(mass_q_);

    // Initialize the solver.
    acado_initializeSolver();

    // Initialize the states and controls.
    kHoverInput_ = initial_input;

    // Initialize states x and xN and input u.
    acado_initial_state_ = initial_state;

    acado_states_ = initial_state.replicate(1, kSamples + 1);

    acado_inputs_ = kHoverInput_.replicate(1, kSamples);

    // Initialize references y and yN.
    acado_reference_states_.block(0, 0, kStateSize, kSamples) =
        initial_state.replicate(1, kSamples);

    acado_reference_states_.block(kStateSize, 0, kCostSize - kStateSize, kSamples) =
        Eigen::Matrix<real_t, kCostSize - kStateSize, kSamples>::Zero();

    acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
        kHoverInput_.replicate(1, kSamples);

    acado_reference_end_state_.segment(0, kStateSize) =
        initial_state;

    acado_reference_end_state_.segment(kStateSize, kCostSize - kStateSize) =
        Eigen::Matrix<real_t, kCostSize - kStateSize, 1>::Zero();

    // Initialize Cost matrix W and WN.
    if (!(acado_W_.trace() > 0.0))
    {
      acado_W_ = W_.replicate(1, kSamples);
      acado_W_end_ = WN_;
    }

    // Initialize solver.
    acado_initializeNodesByForwardSimulation();
    acado_preparationStep();
    acado_is_prepared_ = true;
  }

  // Set cost matrices with optional scaling.
  bool MpcWrapper::setCosts(
      const Eigen::Ref<const Eigen::Matrix<real_t, kCostSize, kCostSize>> &Q,
      const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, kInputSize>> &R,
      real_t state_cost_scaling, real_t input_cost_scaling)
  {
    if (state_cost_scaling < 0.0 || input_cost_scaling < 0.0)
    {
      ROS_ERROR("[NMPC] 代价缩放参数错误：必须为非负数。");
      return false;
    }
    W_.block(0, 0, kCostSize, kCostSize) = Q;
    W_.block(kCostSize, kCostSize, kInputSize, kInputSize) = R;
    WN_ = W_.block(0, 0, kCostSize, kCostSize);

    // ROS_INFO_STREAM("weight_matrix: " << std::endl << W_);

    real_t state_scale{1.0};
    real_t input_scale{1.0};
    for (int i = 0; i < kSamples; i++)
    {
      state_scale = exp(-real_t(i) / real_t(kSamples) * real_t(state_cost_scaling));
      input_scale = exp(-real_t(i) / real_t(kSamples) * real_t(input_cost_scaling));
      acado_W_.block(0, i * kRefSize, kCostSize, kCostSize) =
          W_.block(0, 0, kCostSize, kCostSize) * state_scale;
      acado_W_.block(kCostSize, i * kRefSize + kCostSize, kInputSize, kInputSize) =
          W_.block(kCostSize, kCostSize, kInputSize, kInputSize) * input_scale;
    }
    acado_W_end_ = WN_ * state_scale;

    return true;
  }

  // Set the input limits.
  bool MpcWrapper::setLimits(real_t min_thrust, real_t max_thrust,
                             real_t max_rollpitchrate, real_t max_yawrate,
                             real_t max_velocity_xy, real_t max_velocity_z)
  {
    if (min_thrust <= 0.0 || min_thrust > max_thrust)
    {
      ROS_ERROR("[NMPC] 最小推力设置错误，保持原值。");
      return false;
    }

    if (max_thrust <= 0.0 || min_thrust > max_thrust)
    {
      ROS_ERROR("[NMPC] 最大推力设置错误，保持原值。");
      return false;
    }

    if (max_rollpitchrate <= 0.0)
    {
      ROS_ERROR("[NMPC] 最大横滚/俯仰角速度设置错误，保持原值。");
      return false;
    }

    if (max_yawrate <= 0.0)
    {
      ROS_ERROR("[NMPC] 最大偏航角速度设置错误，保持原值。");
      return false;
    }

    if (!std::isfinite(max_velocity_xy) || !std::isfinite(max_velocity_z) ||
        max_velocity_xy <= 0.0 || max_velocity_z <= 0.0)
    {
      ROS_ERROR("[NMPC] 最大世界系速度设置错误，单位必须为 m/s。保持原值。");
      return false;
    }

    // 边界顺序必须与 ACADO 模型中的 subjectTo 顺序一致：
    // 输入边界顺序为 [T(N), w_x(rad/s), w_y(rad/s), w_z(rad/s)]。
    Eigen::Matrix<real_t, 4, 1> lower_bounds = Eigen::Matrix<real_t, 4, 1>::Zero();
    Eigen::Matrix<real_t, 4, 1> upper_bounds = Eigen::Matrix<real_t, 4, 1>::Zero();
    lower_bounds << min_thrust,
        -max_rollpitchrate, -max_rollpitchrate, -max_yawrate;
    upper_bounds << max_thrust,
        max_rollpitchrate, max_rollpitchrate, max_yawrate;
    Eigen::Matrix<real_t, 1, 1> lower_affine_bounds;
    lower_affine_bounds << -1.1;
    Eigen::Matrix<real_t, 1, 1> upper_affine_bounds;
    upper_affine_bounds << -0.1;

    acado_lower_bounds_ =
        lower_bounds.replicate(1, kSamples);

    acado_upper_bounds_ =
        upper_bounds.replicate(1, kSamples);

    // 速度状态约束由 ACADO 保存为仿射边界，顺序为 [v_x, v_y, v_z]，单位 m/s。
    Eigen::Matrix<real_t, kStateConstraintSize, 1> lower_velocity_bounds;
    lower_velocity_bounds << -max_velocity_xy, -max_velocity_xy, -max_velocity_z;
    Eigen::Matrix<real_t, kStateConstraintSize, 1> upper_velocity_bounds;
    upper_velocity_bounds << max_velocity_xy, max_velocity_xy, max_velocity_z;
    acado_lower_affine_bounds_ = lower_velocity_bounds.replicate(1, kSamples);
    acado_upper_affine_bounds_ = upper_velocity_bounds.replicate(1, kSamples);
    return true;
  }

  // Set a reference pose.
  bool MpcWrapper::setReferencePose(
      const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> reference_state)
  {
    acado_reference_states_.block(0, 0, kStateSize, kSamples) =
        reference_state.replicate(1, kSamples);

    acado_reference_states_.block(kStateSize, 0, kCostSize - kStateSize, kSamples) =
        Eigen::Matrix<real_t, kCostSize - kStateSize, kSamples>::Zero();

    acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
        kHoverInput_.replicate(1, kSamples);

    acado_reference_end_state_.segment(0, kStateSize) =
        reference_state;

    acado_reference_end_state_.segment(kStateSize, kCostSize - kStateSize) =
        Eigen::Matrix<real_t, kCostSize - kStateSize, 1>::Zero();

    acado_initializeNodesByForwardSimulation();
    return true;
  }

  // Set a reference trajectory.
  bool MpcWrapper::setTrajectory(
      const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, kSamples + 1>> reference_states,
      const Eigen::Ref<const Eigen::Matrix<real_t, kInputSize, kSamples + 1>> inputs)
  {

    acado_reference_states_.block(0, 0, kStateSize, kSamples) =
        reference_states.block(0, 0, kStateSize, kSamples);

    acado_reference_states_.block(kStateSize, 0, kCostSize - kStateSize, kSamples) =
        Eigen::Matrix<real_t, kCostSize - kStateSize, kSamples>::Zero();

    acado_reference_states_.block(kCostSize, 0, kInputSize, kSamples) =
        inputs.block(0, 0, kInputSize, kSamples);

    acado_reference_end_state_.segment(0, kStateSize) =
        reference_states.col(kSamples);
    acado_reference_end_state_.segment(kStateSize, kCostSize - kStateSize) =
        Eigen::Matrix<real_t, kCostSize - kStateSize, 1>::Zero();

    return true;
  }

  // Reset states and inputs and calculate new solution.
  bool MpcWrapper::solve(
      const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> state)
  {
    acado_states_ = state.replicate(1, kSamples + 1);

    acado_inputs_ = kHoverInput_.replicate(1, kSamples);

    return update(state);
  }

  // Calculate new solution from last known solution.
  bool MpcWrapper::update(
      const Eigen::Ref<const Eigen::Matrix<real_t, kStateSize, 1>> state,
      bool do_preparation)
  {
    if (!acado_is_prepared_)
    {
      ROS_WARN("[NMPC] 求解器尚未完成准备就被触发，已中止本次求解。");
      return false;
    }

    // Check if estimated and reference quaternion live in sthe same hemisphere.
    acado_initial_state_ = state;
    if (acado_initial_state_.segment(3, 4).dot(
            Eigen::Matrix<real_t, 4, 1>(acado_reference_states_.block(3, 0, 4, 1))) < (real_t)0.0)
    {
      acado_initial_state_.segment(3, 4) = -acado_initial_state_.segment(3, 4);
    }

    // Perform feedback step and reset preparation check.
    int ret = acado_feedbackStep();
    acado_is_prepared_ = false;

    // Prepare if the solver if wanted
    if (do_preparation)
    {
      prepare();
    }
    if (ret != 0)
    {
      ROS_ERROR("[NMPC] 反馈求解失败，错误码=%d。", ret);
      ROS_ERROR("[NMPC] 求解器错误：%s。", acado_getErrorString(ret));
      return false;
    }

    return true;
  }

  // Prepare the solver.
  // Must be triggered between iterations if not done in the update function.
  // template <typename T>

  bool MpcWrapper::prepare()
  {
    int ret = acado_preparationStep();
    if (ret != 0)
    {
      ROS_ERROR("[NMPC] 求解准备失败，错误码=%d。", ret);
      ROS_ERROR("[NMPC] 求解器错误：%s。", acado_getErrorString(ret));
      return false;
    }
    acado_is_prepared_ = true;
    return true;
  }

  void MpcWrapper::setDynamicParams(const real_t mass_q)
  {
    mass_q_ = mass_q;
    for (int i = 0; i < kSamples + 1; i++)
    {
      acado_online_data_(0, i) = mass_q_;
    }
  }

  void MpcWrapper::setExternalForce(const Eigen::Ref<const Eigen::Vector3d> &fq)
  {
    // f_Q 是世界系无人机外力估计，单位 N；预测时域内按常值扰动处理。
    acado_online_data_.block<3, kSamples + 1>(1, 0) = fq.cast<real_t>().replicate(1, kSamples + 1);
  }

  // Get a specific state.
  void MpcWrapper::getState(const int node_index,
                            Eigen::Ref<Eigen::Matrix<real_t, kStateSize, 1>> return_state)
  {
    // return_state = acado_states_.col(node_index).cast<T>();
    return_state = acado_states_.col(node_index);
  }

  // Get all states.
  void MpcWrapper::getStates(
      Eigen::Ref<Eigen::Matrix<real_t, kStateSize, kSamples + 1>> return_states)
  {
    return_states = acado_states_;
  }

  // Get a specific input.
  void MpcWrapper::getInput(const int node_index,
                            Eigen::Ref<Eigen::Matrix<real_t, kInputSize, 1>> return_input)
  {
    return_input = acado_inputs_.col(node_index);
  }

  // Get all inputs.
  void MpcWrapper::getInputs(
      Eigen::Ref<Eigen::Matrix<real_t, kInputSize, kSamples>> return_inputs)
  {
    return_inputs = acado_inputs_;
  }

} // namespace rpg_mpc

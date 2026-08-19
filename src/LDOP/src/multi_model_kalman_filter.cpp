#include <ldop/multi_model_kalman_filter.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace ldopcore {

namespace {

Eigen::VectorXd positionMeasurementVector(const Eigen::Vector3d& measurement) {
  Eigen::VectorXd vector(3);
  vector << measurement.x(), measurement.y(), measurement.z();
  return vector;
}

std::shared_ptr<LinearKalmanFilter> configureLinearFilter(
    const std::shared_ptr<MotionModel>& model,
    const MultiModelKalmanFilterConfig& config) {
  auto filter = std::make_shared<LinearKalmanFilter>(model);
  filter->setAdaptiveParams(config.adaptive_window_size,
                            config.adaptive_alpha,
                            config.adaptive_r_alpha,
                            config.adaptive_min_noise_ratio);
  filter->setCovLimitParams(config.enable_cov_limit,
                            config.max_pos_cov,
                            config.max_vel_cov,
                            config.max_acc_cov);
  return filter;
}

std::shared_ptr<ExtendedKalmanFilter> configureExtendedFilter(
    const std::shared_ptr<MotionModel>& model,
    const MultiModelKalmanFilterConfig& config) {
  auto filter = std::make_shared<ExtendedKalmanFilter>(model);
  filter->setAdaptiveParams(config.adaptive_window_size,
                            config.adaptive_alpha,
                            config.adaptive_r_alpha,
                            config.adaptive_min_noise_ratio);
  filter->setCovLimitParams(config.enable_cov_limit,
                            config.max_pos_cov,
                            config.max_vel_cov,
                            config.max_acc_cov);
  return filter;
}

}  // namespace

MultiModelKalmanFilterConfig buildMultiModelKalmanFilterConfig(
    const MultiModelKalmanFilterParams& params) {
  const MultiModelKalmanFilterParams defaults;
  MultiModelKalmanFilterConfig config;
  config.default_dt = params.default_dt > 0.0 ? params.default_dt : defaults.default_dt;
  config.adaptive_window_size =
      params.adaptive_window_size > 0 ? params.adaptive_window_size : defaults.adaptive_window_size;
  config.adaptive_alpha = std::clamp(params.adaptive_alpha, 0.0, 1.0);
  config.adaptive_r_alpha = std::clamp(params.adaptive_r_alpha, 0.0, 1.0);
  config.adaptive_min_noise_ratio = std::clamp(params.adaptive_min_noise_ratio, 0.0, 1.0);
  config.max_pos_cov = params.max_pos_cov > 0.0 ? params.max_pos_cov : defaults.max_pos_cov;
  config.max_vel_cov = params.max_vel_cov > 0.0 ? params.max_vel_cov : defaults.max_vel_cov;
  config.max_acc_cov = params.max_acc_cov > 0.0 ? params.max_acc_cov : defaults.max_acc_cov;
  config.enable_cov_limit = params.enable_cov_limit;
  config.ca_human.jerk_sigma = params.ca_human_jerk_sigma;
  config.ca_human.init_cov = params.ca_human_init_cov;
  config.ca_human.meas_noise = params.ca_human_meas_noise;
  config.ca_human.z_process_noise = params.ca_human_z_process_noise;
  config.ca_uav.jerk_sigma = params.ca_uav_jerk_sigma;
  config.ca_uav.init_cov = params.ca_uav_init_cov;
  config.ca_uav.meas_noise = params.ca_uav_meas_noise;
  config.cv.acc_sigma = params.cv_acc_sigma;
  config.cv.init_cov = params.cv_init_cov;
  config.cv.meas_noise = params.cv_meas_noise;
  config.ctra.init_cov = params.ctra_init_cov;
  config.ctra.process_noise = params.ctra_process_noise;
  config.ctra.meas_noise = params.ctra_meas_noise;
  return config;
}

std::shared_ptr<MotionModel> createMotionModel(
    const MotionModelType model_type,
    const MultiModelKalmanFilterConfig& config) {
  switch (model_type) {
    case MotionModelType::CA2D: {
      auto model = std::make_shared<CAMotionModel>(config.ca_human, false);
      model->setDt(config.default_dt);
      return model;
    }
    case MotionModelType::CA3D: {
      auto model = std::make_shared<CAMotionModel>(config.ca_uav, true);
      model->setDt(config.default_dt);
      return model;
    }
    case MotionModelType::CV3D: {
      auto model = std::make_shared<CVMotionModel>(config.cv);
      model->setDt(config.default_dt);
      return model;
    }
    case MotionModelType::CTRA: {
      auto model = std::make_shared<CTRAMotionModel>(config.ctra);
      model->setDt(config.default_dt);
      return model;
    }
  }
  throw std::invalid_argument("unsupported motion model");
}

KalmanFilterBase::KalmanFilterBase(std::shared_ptr<MotionModel> model)
    : model_(std::move(model)) {
  if (model_ == nullptr) {
    throw std::invalid_argument("motion model must not be null");
  }

  state_dim_ = model_->stateDim();
  measurement_dim_ = model_->measurementDim();
  process_noise_ = model_->getProcessNoiseQ();
  measurement_noise_ = model_->getMeasNoiseR();
  min_process_noise_ = process_noise_ * 0.5;
  min_measurement_noise_ = measurement_noise_ * 0.5;
}

void KalmanFilterBase::setAdaptiveParams(const int window_size,
                                         const double alpha,
                                         const double r_alpha,
                                         const double min_noise_ratio) {
  adaptive_window_size_ = std::max(window_size, 0);
  adaptive_alpha_ = std::clamp(alpha, 0.0, 1.0);
  adaptive_r_alpha_ = std::clamp(r_alpha, 0.0, 1.0);
  const double ratio =
      std::isfinite(min_noise_ratio) && min_noise_ratio > 0.0 ? min_noise_ratio : 0.5;

  // 下限来自当前模型的基础 Q/R，和 LDOT 一样用来避免自适应估计把噪声压到 0。
  min_process_noise_ = model_->getProcessNoiseQ() * ratio;
  min_measurement_noise_ = model_->getMeasNoiseR() * ratio;
}

void KalmanFilterBase::setCovLimitParams(const bool enable,
                                         const double max_pos_cov,
                                         const double max_vel_cov,
                                         const double max_acc_cov) {
  enable_cov_limit_ = enable;
  max_pos_cov_ = std::isfinite(max_pos_cov) && max_pos_cov > 0.0 ? max_pos_cov : max_pos_cov_;
  max_vel_cov_ = std::isfinite(max_vel_cov) && max_vel_cov > 0.0 ? max_vel_cov : max_vel_cov_;
  max_acc_cov_ = std::isfinite(max_acc_cov) && max_acc_cov > 0.0 ? max_acc_cov : max_acc_cov_;
}

void KalmanFilterBase::initialize(const Eigen::VectorXd& detection) {
  if (detection.size() < measurement_dim_ || !detection.allFinite()) {
    throw std::invalid_argument("initial detection must be finite and contain [x,y,z]");
  }

  // LDOT 语义：初始化不是先从零状态做一次 update，而是让运动模型直接生成初始状态。
  state_ = model_->getInitState(detection);
  covariance_ = model_->getInitCovP();
  process_noise_ = model_->getProcessNoiseQ();
  measurement_noise_ = model_->getMeasNoiseR();
  innovation_buffer_.clear();
  initialized_ = true;
  clampCovariance();
}

void KalmanFilterBase::initialize(const Eigen::Vector3d& detection) {
  initialize(positionMeasurementVector(detection));
}

void KalmanFilterBase::initializeState(const Eigen::VectorXd& state) {
  if (state.size() != state_dim_ || !state.allFinite()) {
    throw std::invalid_argument("initial state must be finite and match model dimension");
  }

  // 只在 tracker 确认类别变化并切换模型时使用。协方差重置为新模型的初值，
  // 因为旧模型的状态维度和物理含义可能不同，直接搬 P 反而会制造虚假的相关性。
  state_ = state;
  covariance_ = model_->getInitCovP();
  process_noise_ = model_->getProcessNoiseQ();
  measurement_noise_ = model_->getMeasNoiseR();
  innovation_buffer_.clear();
  initialized_ = true;
  model_->normalizeYaw(state_);
  clampCovariance();
}

void KalmanFilterBase::setDt(const double dt) {
  model_->setDt(dt);
}

Eigen::Vector3d KalmanFilterBase::position() const {
  if (state_.size() < 3) {
    return Eigen::Vector3d::Zero();
  }
  return state_.segment<3>(0);
}

Eigen::Vector3d KalmanFilterBase::velocity() const {
  if (state_.size() < 6) {
    return Eigen::Vector3d::Zero();
  }

  switch (model_->type()) {
    case MotionModelType::CA2D:
      return Eigen::Vector3d(state_(3), state_(4), 0.0);
    case MotionModelType::CA3D:
    case MotionModelType::CV3D:
      return state_.segment<3>(3);
    case MotionModelType::CTRA: {
      // CTRA 只有标量速度和 yaw，对外转换为世界坐标系水平速度。
      const double speed = state_(3);
      const double yaw = state_(5);
      return Eigen::Vector3d(speed * std::cos(yaw), speed * std::sin(yaw), 0.0);
    }
  }

  return Eigen::Vector3d::Zero();
}

void KalmanFilterBase::setVelocity(const Eigen::Vector3d& velocity) {
  if (state_.size() < 6 || !velocity.allFinite()) {
    return;
  }
  switch (model_->type()) {
    case MotionModelType::CA2D:
      state_(3) = velocity.x();
      state_(4) = velocity.y();
      break;
    case MotionModelType::CA3D:
    case MotionModelType::CV3D:
      state_.segment<3>(3) = velocity;
      break;
    case MotionModelType::CTRA: {
      const double speed = std::hypot(velocity.x(), velocity.y());
      state_(3) = speed;
      if (speed > 1.0e-3) {
        state_(5) = std::atan2(velocity.y(), velocity.x());
      }
      break;
    }
  }
  model_->normalizeYaw(state_);
}

void KalmanFilterBase::updateInnovationBuffer(const Eigen::VectorXd& innovation) {
  if (adaptive_window_size_ <= 1) {
    innovation_buffer_.clear();
    return;
  }

  innovation_buffer_.push_back(innovation);
  if (innovation_buffer_.size() > static_cast<std::size_t>(adaptive_window_size_)) {
    innovation_buffer_.pop_front();
  }
}

void KalmanFilterBase::update(const Eigen::Vector3d& measurement) {
  update(positionMeasurementVector(measurement));
}

Eigen::MatrixXd KalmanFilterBase::innovationSampleCovariance() const {
  Eigen::MatrixXd sample_covariance = Eigen::MatrixXd::Zero(measurement_dim_, measurement_dim_);
  if (innovation_buffer_.empty()) {
    return sample_covariance;
  }

  for (const auto& innovation : innovation_buffer_) {
    sample_covariance += innovation * innovation.transpose();
  }
  sample_covariance /= static_cast<double>(innovation_buffer_.size());
  return sample_covariance;
}

void KalmanFilterBase::estimateAdaptiveR(const Eigen::VectorXd& /*innovation*/,
                                         const Eigen::MatrixXd& measurement_matrix) {
  if (adaptive_window_size_ <= 1 ||
      innovation_buffer_.size() < static_cast<std::size_t>(adaptive_window_size_)) {
    return;
  }

  // C_gamma ≈ H * P_pred * H^T + R，因此 R 可由样本 innovation 协方差反推。
  const Eigen::MatrixXd sample_covariance = innovationSampleCovariance();
  const Eigen::MatrixXd estimated_measurement_noise =
      sample_covariance - measurement_matrix * covariance_ * measurement_matrix.transpose();

  for (int index = 0; index < measurement_dim_; ++index) {
    const double diagonal_estimate = std::max(estimated_measurement_noise(index, index), 0.0);
    const double smoothed_value =
        adaptive_r_alpha_ * diagonal_estimate +
        (1.0 - adaptive_r_alpha_) * measurement_noise_(index, index);
    measurement_noise_(index, index) =
        std::max(smoothed_value, min_measurement_noise_(index, index));
  }
}

void KalmanFilterBase::estimateAdaptiveQ(const Eigen::VectorXd& /*innovation*/,
                                         const Eigen::MatrixXd& measurement_matrix,
                                         const Eigen::MatrixXd& measurement_noise,
                                         const Eigen::MatrixXd& kalman_gain) {
  if (adaptive_window_size_ <= 1 ||
      innovation_buffer_.size() < static_cast<std::size_t>(adaptive_window_size_)) {
    return;
  }

  // LDOT 同款估计：P_prop = P_pred - Q_old，剩余 innovation 协方差映射回状态空间得到 Q_new。
  const Eigen::MatrixXd sample_covariance = innovationSampleCovariance();
  const Eigen::MatrixXd propagated_covariance = covariance_ - process_noise_;
  const Eigen::MatrixXd unexplained_covariance =
      sample_covariance -
      measurement_matrix * propagated_covariance * measurement_matrix.transpose() -
      measurement_noise;
  const Eigen::MatrixXd estimated_process_noise =
      kalman_gain * unexplained_covariance * kalman_gain.transpose();

  for (int index = 0; index < state_dim_; ++index) {
    const double diagonal_estimate = std::max(estimated_process_noise(index, index), 0.0);
    const double smoothed_value =
        adaptive_alpha_ * diagonal_estimate +
        (1.0 - adaptive_alpha_) * process_noise_(index, index);
    process_noise_(index, index) = std::max(smoothed_value, min_process_noise_(index, index));
  }
}

void KalmanFilterBase::clampCovariance() {
  if (!initialized_ || covariance_.rows() == 0) {
    return;
  }

  covariance_ = 0.5 * (covariance_ + covariance_.transpose());
  if (!enable_cov_limit_) {
    return;
  }

  const auto clamp_diagonal = [this](const int index, const double limit) {
    covariance_(index, index) = std::clamp(covariance_(index, index), 0.0, limit);
  };

  // 三个模型都把位置放在 [0,1,2]；速度/加速度/yaw 的含义按状态维度和模型类型区分。
  for (int index = 0; index < std::min(3, state_dim_); ++index) {
    clamp_diagonal(index, max_pos_cov_);
  }

  switch (model_->type()) {
    case MotionModelType::CA2D:
      clamp_diagonal(3, max_vel_cov_);
      clamp_diagonal(4, max_vel_cov_);
      clamp_diagonal(5, max_acc_cov_);
      clamp_diagonal(6, max_acc_cov_);
      break;
    case MotionModelType::CA3D:
      for (int index = 3; index < 6; ++index) {
        clamp_diagonal(index, max_vel_cov_);
      }
      for (int index = 6; index < 9; ++index) {
        clamp_diagonal(index, max_acc_cov_);
      }
      break;
    case MotionModelType::CV3D:
      for (int index = 3; index < 6; ++index) {
        clamp_diagonal(index, max_vel_cov_);
      }
      break;
    case MotionModelType::CTRA:
      clamp_diagonal(3, max_vel_cov_);
      clamp_diagonal(4, max_acc_cov_);
      clamp_diagonal(5, max_acc_cov_);
      clamp_diagonal(6, max_acc_cov_);
      break;
  }
}

LinearKalmanFilter::LinearKalmanFilter(std::shared_ptr<MotionModel> model)
    : KalmanFilterBase(std::move(model)) {}

void LinearKalmanFilter::predict() {
  if (!initialized_) {
    return;
  }

  const Eigen::MatrixXd transition = model_->getTransitionF(state_);
  // 关闭自适应时，Q 每次从模型读取，因此 dt 改变会体现在本次 predict 的基础 Q 上。
  if (adaptive_window_size_ <= 1) {
    process_noise_ = model_->getProcessNoiseQ();
  }
  state_ = transition * state_;
  covariance_ = transition * covariance_ * transition.transpose() + process_noise_;
  clampCovariance();
  model_->normalizeYaw(state_);
}

void LinearKalmanFilter::update(const Eigen::VectorXd& measurement) {
  if (!initialized_) {
    return;
  }
  if (measurement.size() < measurement_dim_ || !measurement.allFinite()) {
    throw std::invalid_argument("measurement must be finite and contain [x,y,z]");
  }

  const Eigen::MatrixXd measurement_matrix = model_->getMeasurementH(state_);
  Eigen::VectorXd innovation = measurement - measurement_matrix * state_;
  model_->normalizeYawInResidual(innovation);

  updateInnovationBuffer(innovation);
  estimateAdaptiveR(innovation, measurement_matrix);

  const Eigen::MatrixXd temporary_innovation_covariance =
      measurement_matrix * covariance_ * measurement_matrix.transpose() + measurement_noise_;
  const Eigen::MatrixXd temporary_kalman_gain =
      covariance_ * measurement_matrix.transpose() * temporary_innovation_covariance.inverse();
  const Eigen::MatrixXd previous_process_noise = process_noise_;
  estimateAdaptiveQ(innovation, measurement_matrix, measurement_noise_, temporary_kalman_gain);
  covariance_ = covariance_ - previous_process_noise + process_noise_;

  const Eigen::MatrixXd innovation_covariance =
      measurement_matrix * covariance_ * measurement_matrix.transpose() + measurement_noise_;
  const Eigen::MatrixXd kalman_gain =
      covariance_ * measurement_matrix.transpose() * innovation_covariance.inverse();

  state_ = state_ + kalman_gain * innovation;
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  covariance_ = (identity - kalman_gain * measurement_matrix) * covariance_;
  clampCovariance();
  model_->normalizeYaw(state_);
}

ExtendedKalmanFilter::ExtendedKalmanFilter(std::shared_ptr<MotionModel> model)
    : KalmanFilterBase(std::move(model)) {}

void ExtendedKalmanFilter::predict() {
  if (!initialized_) {
    return;
  }

  // EKF 协方差传播必须在预测前状态处线性化；CTRA 的 F 依赖 yaw/yaw_rate，
  // 若先更新 state_ 再求雅可比，会把协方差投影到错误的运动切线方向上。
  const Eigen::VectorXd previous_state = state_;
  const Eigen::MatrixXd transition_jacobian = model_->getTransitionF(previous_state);
  state_ = model_->stateTransition(previous_state);
  if (adaptive_window_size_ <= 1) {
    process_noise_ = model_->getProcessNoiseQ();
  }
  covariance_ = transition_jacobian * covariance_ * transition_jacobian.transpose() + process_noise_;
  clampCovariance();
  model_->normalizeYaw(state_);
}

void ExtendedKalmanFilter::update(const Eigen::VectorXd& measurement) {
  if (!initialized_) {
    return;
  }
  if (measurement.size() < measurement_dim_ || !measurement.allFinite()) {
    throw std::invalid_argument("measurement must be finite and contain [x,y,z]");
  }

  const Eigen::VectorXd predicted_measurement = model_->stateToMeasurement(state_);
  Eigen::VectorXd innovation = measurement - predicted_measurement;
  model_->normalizeYawInResidual(innovation);

  const Eigen::MatrixXd measurement_jacobian = model_->getMeasurementH(state_);
  updateInnovationBuffer(innovation);
  estimateAdaptiveR(innovation, measurement_jacobian);

  const Eigen::MatrixXd temporary_innovation_covariance =
      measurement_jacobian * covariance_ * measurement_jacobian.transpose() + measurement_noise_;
  const Eigen::MatrixXd temporary_kalman_gain =
      covariance_ * measurement_jacobian.transpose() * temporary_innovation_covariance.inverse();
  const Eigen::MatrixXd previous_process_noise = process_noise_;
  estimateAdaptiveQ(innovation, measurement_jacobian, measurement_noise_, temporary_kalman_gain);
  covariance_ = covariance_ - previous_process_noise + process_noise_;

  const Eigen::MatrixXd innovation_covariance =
      measurement_jacobian * covariance_ * measurement_jacobian.transpose() + measurement_noise_;
  const Eigen::MatrixXd kalman_gain =
      covariance_ * measurement_jacobian.transpose() * innovation_covariance.inverse();

  state_ = state_ + kalman_gain * innovation;
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  const Eigen::MatrixXd projection = identity - kalman_gain * measurement_jacobian;
  // EKF 的数值误差更明显，沿用 LDOT 里 EKF 的 Joseph 形式。
  covariance_ = projection * covariance_ * projection.transpose() +
                kalman_gain * measurement_noise_ * kalman_gain.transpose();
  clampCovariance();
  model_->normalizeYaw(state_);
}

std::shared_ptr<KalmanFilterBase> createKalmanFilter(
    const MotionModelType model_type,
    const MultiModelKalmanFilterConfig& config) {
  const std::shared_ptr<MotionModel> model = createMotionModel(model_type, config);
  if (model_type == MotionModelType::CTRA) {
    return configureExtendedFilter(model, config);
  }
  return configureLinearFilter(model, config);
}

std::shared_ptr<KalmanFilterBase> createKalmanFilter(
    const bool is_human,
    const bool is_vehicle,
    const bool is_uav,
    const bool is_other,
    const MultiModelKalmanFilterConfig& config) {
  if (is_human) {
    return createKalmanFilter(MotionModelType::CA2D, config);
  }
  if (is_uav) {
    return createKalmanFilter(MotionModelType::CA3D, config);
  }
  if (is_vehicle) {
    return createKalmanFilter(MotionModelType::CTRA, config);
  }
  if (is_other) {
    return createKalmanFilter(MotionModelType::CV3D, config);
  }
  return createKalmanFilter(MotionModelType::CV3D, config);
}

}  // namespace ldopcore

#include <ldop/motion_model.h>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace ldopcore {

namespace {

Eigen::MatrixXd diagonalFromValues(const int dimension, const std::vector<double>& values) {
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Identity(dimension, dimension);
  for (int index = 0; index < dimension && index < static_cast<int>(values.size()); ++index) {
    const double value = values[static_cast<std::size_t>(index)];
    matrix(index, index) = std::isfinite(value) && value > 0.0 ? value : matrix(index, index);
  }
  return matrix;
}

Eigen::VectorXd initPositionState(const int dimension, const Eigen::VectorXd& detection) {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(dimension);
  // tracker 传入的 detection 固定是 [x,y,z]；模型只负责把它放到自己的状态布局开头。
  state(0) = detection.size() > 0 ? detection(0) : 0.0;
  state(1) = detection.size() > 1 ? detection(1) : 0.0;
  state(2) = detection.size() > 2 ? detection(2) : 0.0;
  return state;
}

Eigen::MatrixXd positionMeasurementMatrix(const int measurement_dim, const int state_dim) {
  Eigen::MatrixXd measurement = Eigen::MatrixXd::Zero(measurement_dim, state_dim);
  measurement(0, 0) = 1.0;
  measurement(1, 1) = 1.0;
  measurement(2, 2) = 1.0;
  return measurement;
}

Eigen::VectorXd positionMeasurement(const Eigen::VectorXd& state) {
  Eigen::VectorXd measurement(3);
  measurement(0) = state(0);
  measurement(1) = state(1);
  measurement(2) = state(2);
  return measurement;
}

}  // namespace

Eigen::VectorXd MotionModel::stateTransition(const Eigen::VectorXd& state) const {
  return getTransitionF(state) * state;
}

double MotionModel::wrapToPi(double angle) {
  constexpr double kPi = 3.14159265358979323846;
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

CAMotionModel::CAMotionModel(const CAParams& params, const bool use_3d)
    : use_3d_(use_3d),
      params_(params) {
  state_dim_ = use_3d_ ? 9 : 7;
  measurement_dim_ = 3;
}

MotionModelType CAMotionModel::type() const {
  return use_3d_ ? MotionModelType::CA3D : MotionModelType::CA2D;
}

Eigen::VectorXd CAMotionModel::getInitState(const Eigen::VectorXd& detection) const {
  return initPositionState(state_dim_, detection);
}

Eigen::MatrixXd CAMotionModel::getInitCovP() const {
  return diagonalFromValues(state_dim_, params_.init_cov);
}

Eigen::MatrixXd CAMotionModel::getProcessNoiseQ() const {
  // Discrete white noise jerk model。CA3D 每个轴都有 pos/vel/acc；
  // CA2D 只在 x/y 上估计速度和加速度，z 轴单独作为慢变化位置处理。
  Eigen::MatrixXd process_noise = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
  const double dt2 = dt_ * dt_;
  const double dt3 = dt2 * dt_;
  const double dt4 = dt3 * dt_;
  const double dt5 = dt4 * dt_;
  const double jerk_variance = std::max(params_.jerk_sigma, 1e-9);
  const double q = jerk_variance * jerk_variance;

  const auto set_q_block = [&](const int position_index, const int velocity_index, const int acceleration_index) {
    process_noise(position_index, position_index) = dt5 / 20.0 * q;
    process_noise(position_index, velocity_index) = dt4 / 8.0 * q;
    process_noise(position_index, acceleration_index) = dt3 / 6.0 * q;
    process_noise(velocity_index, position_index) = dt4 / 8.0 * q;
    process_noise(velocity_index, velocity_index) = dt3 / 3.0 * q;
    process_noise(velocity_index, acceleration_index) = dt2 / 2.0 * q;
    process_noise(acceleration_index, position_index) = dt3 / 6.0 * q;
    process_noise(acceleration_index, velocity_index) = dt2 / 2.0 * q;
    process_noise(acceleration_index, acceleration_index) = dt_ * q;
  };

  if (use_3d_) {
    set_q_block(0, 3, 6);
    set_q_block(1, 4, 7);
    set_q_block(2, 5, 8);
  } else {
    set_q_block(0, 3, 5);
    set_q_block(1, 4, 6);
    process_noise(2, 2) = std::max(params_.z_process_noise, 1e-9);
  }

  return process_noise;
}

Eigen::MatrixXd CAMotionModel::getMeasNoiseR() const {
  return diagonalFromValues(measurement_dim_, params_.meas_noise);
}

Eigen::MatrixXd CAMotionModel::getTransitionF(const Eigen::VectorXd& /*state*/) const {
  Eigen::MatrixXd transition = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  const double half_dt2 = 0.5 * dt_ * dt_;

  if (use_3d_) {
    // CA3D: [x,y,z,vx,vy,vz,ax,ay,az]。
    transition(0, 3) = dt_;
    transition(0, 6) = half_dt2;
    transition(1, 4) = dt_;
    transition(1, 7) = half_dt2;
    transition(2, 5) = dt_;
    transition(2, 8) = half_dt2;
    transition(3, 6) = dt_;
    transition(4, 7) = dt_;
    transition(5, 8) = dt_;
  } else {
    // CA2D: [x,y,z,vx,vy,ax,ay]，z 位置保持，水平面按 CA 外推。
    transition(0, 3) = dt_;
    transition(0, 5) = half_dt2;
    transition(1, 4) = dt_;
    transition(1, 6) = half_dt2;
    transition(3, 5) = dt_;
    transition(4, 6) = dt_;
  }

  return transition;
}

Eigen::MatrixXd CAMotionModel::getMeasurementH(const Eigen::VectorXd& /*state*/) const {
  return positionMeasurementMatrix(measurement_dim_, state_dim_);
}

Eigen::VectorXd CAMotionModel::stateToMeasurement(const Eigen::VectorXd& state) const {
  return positionMeasurement(state);
}

void CAMotionModel::normalizeYaw(Eigen::VectorXd& /*state*/) const {}

void CAMotionModel::normalizeYawInResidual(Eigen::VectorXd& /*residual*/) const {}

CVMotionModel::CVMotionModel(const CVParams& params)
    : params_(params) {
  state_dim_ = 6;
  measurement_dim_ = 3;
}

MotionModelType CVMotionModel::type() const {
  return MotionModelType::CV3D;
}

Eigen::VectorXd CVMotionModel::getInitState(const Eigen::VectorXd& detection) const {
  return initPositionState(state_dim_, detection);
}

Eigen::MatrixXd CVMotionModel::getInitCovP() const {
  return diagonalFromValues(state_dim_, params_.init_cov);
}

Eigen::MatrixXd CVMotionModel::getProcessNoiseQ() const {
  // Discrete white noise acceleration model。
  Eigen::MatrixXd process_noise = Eigen::MatrixXd::Zero(state_dim_, state_dim_);
  const double dt2 = dt_ * dt_;
  const double dt3 = dt2 * dt_;
  const double dt4 = dt2 * dt2;
  const double acc_variance = std::max(params_.acc_sigma, 1e-9);
  const double q = acc_variance * acc_variance;

  const auto set_q_block = [&](const int position_index, const int velocity_index) {
    process_noise(position_index, position_index) = dt4 / 4.0 * q;
    process_noise(position_index, velocity_index) = dt3 / 2.0 * q;
    process_noise(velocity_index, position_index) = dt3 / 2.0 * q;
    process_noise(velocity_index, velocity_index) = dt2 * q;
  };
  set_q_block(0, 3);
  set_q_block(1, 4);
  set_q_block(2, 5);
  return process_noise;
}

Eigen::MatrixXd CVMotionModel::getMeasNoiseR() const {
  return diagonalFromValues(measurement_dim_, params_.meas_noise);
}

Eigen::MatrixXd CVMotionModel::getTransitionF(const Eigen::VectorXd& /*state*/) const {
  Eigen::MatrixXd transition = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  transition(0, 3) = dt_;
  transition(1, 4) = dt_;
  transition(2, 5) = dt_;
  return transition;
}

Eigen::MatrixXd CVMotionModel::getMeasurementH(const Eigen::VectorXd& /*state*/) const {
  return positionMeasurementMatrix(measurement_dim_, state_dim_);
}

Eigen::VectorXd CVMotionModel::stateToMeasurement(const Eigen::VectorXd& state) const {
  return positionMeasurement(state);
}

void CVMotionModel::normalizeYaw(Eigen::VectorXd& /*state*/) const {}

void CVMotionModel::normalizeYawInResidual(Eigen::VectorXd& /*residual*/) const {}

CTRAMotionModel::CTRAMotionModel(const CTRAParams& params)
    : params_(params) {
  state_dim_ = 7;
  measurement_dim_ = 3;
}

MotionModelType CTRAMotionModel::type() const {
  return MotionModelType::CTRA;
}

Eigen::VectorXd CTRAMotionModel::getInitState(const Eigen::VectorXd& detection) const {
  return initPositionState(state_dim_, detection);
}

Eigen::MatrixXd CTRAMotionModel::getInitCovP() const {
  return diagonalFromValues(state_dim_, params_.init_cov);
}

Eigen::MatrixXd CTRAMotionModel::getProcessNoiseQ() const {
  return diagonalFromValues(state_dim_, params_.process_noise);
}

Eigen::MatrixXd CTRAMotionModel::getMeasNoiseR() const {
  return diagonalFromValues(measurement_dim_, params_.meas_noise);
}

Eigen::VectorXd CTRAMotionModel::stateTransition(const Eigen::VectorXd& state) const {
  // CTRA: [x,y,z,v,a,yaw,yaw_rate]。z 当前按 LDOT 保持不变。
  const double x = state(0);
  const double y = state(1);
  const double z = state(2);
  const double v = state(3);
  const double a = state(4);
  const double yaw = state(5);
  const double omega = state(6);

  const double yaw_next = yaw + omega * dt_;
  const double v_next = v + a * dt_;
  Eigen::VectorXd next_state(state_dim_);

  if (std::abs(omega) < 1e-3) {
    const double displacement = v * dt_ + 0.5 * a * dt_ * dt_;
    next_state(0) = x + displacement * std::cos(yaw);
    next_state(1) = y + displacement * std::sin(yaw);
  } else {
    const double omega_inv = 1.0 / omega;
    const double omega_inv_sq = omega_inv * omega_inv;
    const double sin_yaw = std::sin(yaw);
    const double cos_yaw = std::cos(yaw);
    const double sin_yaw_next = std::sin(yaw_next);
    const double cos_yaw_next = std::cos(yaw_next);
    next_state(0) =
        x + omega_inv_sq *
                (v_next * omega * sin_yaw_next + a * cos_yaw_next -
                 v * omega * sin_yaw - a * cos_yaw);
    next_state(1) =
        y + omega_inv_sq *
                (-v_next * omega * cos_yaw_next + a * sin_yaw_next +
                 v * omega * cos_yaw - a * sin_yaw);
  }

  next_state(2) = z;
  next_state(3) = v_next;
  next_state(4) = a;
  next_state(5) = yaw_next;
  next_state(6) = omega;
  return next_state;
}

Eigen::MatrixXd CTRAMotionModel::getTransitionF(const Eigen::VectorXd& state) const {
  const double v = state(3);
  const double a = state(4);
  const double yaw = state(5);
  const double omega = state(6);
  const double yaw_next = yaw + omega * dt_;
  const double v_next = v + a * dt_;
  const double sin_yaw = std::sin(yaw);
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw_next = std::sin(yaw_next);
  const double cos_yaw_next = std::cos(yaw_next);

  Eigen::MatrixXd transition = Eigen::MatrixXd::Identity(state_dim_, state_dim_);
  if (std::abs(omega) < 1e-3) {
    const double displacement = v * dt_ + 0.5 * a * dt_ * dt_;
    transition(0, 3) = dt_ * cos_yaw;
    transition(0, 4) = 0.5 * dt_ * dt_ * cos_yaw;
    transition(0, 5) = -displacement * sin_yaw;
    transition(1, 3) = dt_ * sin_yaw;
    transition(1, 4) = 0.5 * dt_ * dt_ * sin_yaw;
    transition(1, 5) = displacement * cos_yaw;
  } else {
    const double omega_inv = 1.0 / omega;
    const double omega_inv_sq = omega_inv * omega_inv;
    const double omega_inv_cube = omega_inv_sq * omega_inv;

    transition(0, 3) = -omega_inv * (sin_yaw - sin_yaw_next);
    transition(0, 4) = -omega_inv_sq * (cos_yaw - cos_yaw_next) +
                       omega_inv * dt_ * sin_yaw_next;
    transition(0, 5) = omega_inv_sq * a * (sin_yaw - sin_yaw_next) +
                       omega_inv * (v_next * cos_yaw_next - v * cos_yaw);
    transition(0, 6) = omega_inv_cube * 2.0 * a * (cos_yaw - cos_yaw_next) +
                       omega_inv_sq *
                           (v * sin_yaw - v_next * sin_yaw_next -
                            a * dt_ * sin_yaw_next) +
                       omega_inv * dt_ * v_next * cos_yaw_next;

    transition(1, 3) = omega_inv * (cos_yaw - cos_yaw_next);
    transition(1, 4) = -omega_inv_sq * (sin_yaw - sin_yaw_next) -
                       omega_inv * dt_ * cos_yaw_next;
    transition(1, 5) = omega_inv_sq * a * (-cos_yaw + cos_yaw_next) +
                       omega_inv * (v_next * sin_yaw_next - v * sin_yaw);
    transition(1, 6) = omega_inv_cube * 2.0 * a * (sin_yaw - sin_yaw_next) +
                       omega_inv_sq *
                           (v_next * cos_yaw_next - v * cos_yaw +
                            a * dt_ * cos_yaw_next) +
                       omega_inv * dt_ * v_next * sin_yaw_next;
  }

  transition(3, 4) = dt_;
  transition(5, 6) = dt_;
  return transition;
}

Eigen::MatrixXd CTRAMotionModel::getMeasurementH(const Eigen::VectorXd& /*state*/) const {
  return positionMeasurementMatrix(measurement_dim_, state_dim_);
}

Eigen::VectorXd CTRAMotionModel::stateToMeasurement(const Eigen::VectorXd& state) const {
  return positionMeasurement(state);
}

void CTRAMotionModel::normalizeYaw(Eigen::VectorXd& state) const {
  state(5) = wrapToPi(state(5));
}

void CTRAMotionModel::normalizeYawInResidual(Eigen::VectorXd& /*residual*/) const {
  // 当前 CTRA 观测只有 [x,y,z]，residual 里没有角度项。
}

}  // namespace ldopcore

#include "precision_landing/landing_controller.hpp"

#include <algorithm>
#include <cmath>

namespace precision_landing {

LandingController::LandingController(const ControllerConfig& config) : config_(config) {}

ControlCommand LandingController::compute(const Eigen::Vector2d& error_body,
                                          double yaw_rad,
                                          double descent_speed,
                                          double dt_sec) {
  const double alpha = std::max(0.0, std::min(1.0, config_.filter_alpha));
  if (!initialized_) {
    filtered_error_ = error_body;
    initialized_ = true;
  } else {
    filtered_error_ = alpha * error_body + (1.0 - alpha) * filtered_error_;
  }

  Eigen::Vector2d deadbanded_error = filtered_error_;
  for (int i = 0; i < deadbanded_error.size(); ++i) {
    if (std::abs(deadbanded_error[i]) < config_.error_deadband) {
      deadbanded_error[i] = 0.0;
    }
  }

  Eigen::Vector2d velocity_body = config_.kp_xy * deadbanded_error;
  const double max_speed = std::max(0.0, config_.max_xy_speed);
  if (velocity_body.norm() > max_speed && velocity_body.norm() > 0.0) {
    velocity_body *= max_speed / velocity_body.norm();
  }

  Eigen::Matrix2d rotation;
  rotation << std::cos(yaw_rad), -std::sin(yaw_rad),
              std::sin(yaw_rad), std::cos(yaw_rad);
  Eigen::Vector2d velocity_enu = rotation * velocity_body;

  Eigen::Vector2d delta = velocity_enu - previous_velocity_enu_;
  const double max_delta = std::max(0.0, config_.max_xy_accel) * std::max(dt_sec, 0.001);
  if (delta.norm() > max_delta && delta.norm() > 0.0) {
    delta *= max_delta / delta.norm();
  }
  previous_velocity_enu_ += delta;

  ControlCommand command;
  command.publish = true;
  command.velocity_enu.head<2>() = previous_velocity_enu_;
  command.velocity_enu.z() = -std::max(0.0, descent_speed);
  command.yaw_rad = yaw_rad;
  return command;
}

void LandingController::reset() {
  filtered_error_.setZero();
  previous_velocity_enu_.setZero();
  initialized_ = false;
}

}  // namespace precision_landing

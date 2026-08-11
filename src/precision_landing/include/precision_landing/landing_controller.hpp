#pragma once

#include <Eigen/Core>

#include "precision_landing/types.hpp"

namespace precision_landing {

struct ControllerConfig {
  double kp_xy{0.55};
  double max_xy_speed{0.60};
  double max_xy_accel{0.80};
  double error_deadband{0.02};
  double filter_alpha{0.35};
};

class LandingController {
 public:
  explicit LandingController(const ControllerConfig& config);

  ControlCommand compute(const Eigen::Vector2d& error_body,
                         double yaw_rad,
                         double descent_speed,
                         double dt_sec);
  void reset();

 private:
  ControllerConfig config_;
  Eigen::Vector2d filtered_error_{Eigen::Vector2d::Zero()};
  Eigen::Vector2d previous_velocity_enu_{Eigen::Vector2d::Zero()};
  bool initialized_{false};
};

}  // namespace precision_landing

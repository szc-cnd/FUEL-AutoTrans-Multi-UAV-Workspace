#pragma once

#include <Eigen/Core>
#include <cstdint>
#include <string>

namespace precision_landing {

enum class LandingState {
  IDLE,
  PRECHECK,
  ACQUIRE,
  ALIGN,
  DESCEND_HIGH,
  FIXED_XY_DESCENT,
  DESCEND_MID,
  DESCEND_FINAL,
  REACQUIRE,
  REQUEST_AUTO_LAND,
  DONE,
  ABORT_HOLD,
  PASSIVE_ABORT
};

struct TargetObservation {
  bool valid{false};
  int id{-1};
  Eigen::Vector3d position_camera{Eigen::Vector3d::Zero()};
  Eigen::Vector2d image_center_px{Eigen::Vector2d::Zero()};
  double reprojection_error_px{0.0};
  double stamp_sec{0.0};
};

struct ControlCommand {
  bool publish{false};
  Eigen::Vector3d velocity_enu{Eigen::Vector3d::Zero()};
  double yaw_rad{0.0};
};

inline const char* toString(LandingState state) {
  switch (state) {
    case LandingState::IDLE: return "IDLE";
    case LandingState::PRECHECK: return "PRECHECK";
    case LandingState::ACQUIRE: return "ACQUIRE";
    case LandingState::ALIGN: return "ALIGN";
    case LandingState::DESCEND_HIGH: return "DESCEND_HIGH";
    case LandingState::FIXED_XY_DESCENT: return "FIXED_XY_DESCENT";
    case LandingState::DESCEND_MID: return "DESCEND_MID";
    case LandingState::DESCEND_FINAL: return "DESCEND_FINAL";
    case LandingState::REACQUIRE: return "REACQUIRE";
    case LandingState::REQUEST_AUTO_LAND: return "REQUEST_AUTO_LAND";
    case LandingState::DONE: return "DONE";
    case LandingState::ABORT_HOLD: return "ABORT_HOLD";
    case LandingState::PASSIVE_ABORT: return "PASSIVE_ABORT";
  }
  return "UNKNOWN";
}

}  // namespace precision_landing

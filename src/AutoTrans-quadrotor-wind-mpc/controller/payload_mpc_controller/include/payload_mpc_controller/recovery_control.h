#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include <Eigen/Eigen>
#include <Eigen/Geometry>

namespace PayloadMPC
{

struct RecoveryAttitudeCommand
{
    Eigen::Quaterniond orientation{Eigen::Quaterniond::Identity()};
    double normalized_thrust{0.0};
    double tilt_rad{0.0};
    bool valid{false};
};

inline bool finiteQuaternion(const Eigen::Quaterniond &q)
{
    return q.coeffs().allFinite() && std::isfinite(q.norm()) && q.norm() > 1.0e-6;
}

inline double attitudeTiltRad(const Eigen::Quaterniond &attitude)
{
    if (!finiteQuaternion(attitude))
        return std::numeric_limits<double>::infinity();

    const Eigen::Quaterniond normalized = attitude.normalized();
    const double body_z_world_z = std::max(-1.0, std::min(1.0,
        normalized.toRotationMatrix()(2, 2)));
    return std::acos(body_z_world_z);
}

inline RecoveryAttitudeCommand makeRecoveryAttitudeCommand(
    const Eigen::Quaterniond &current_attitude,
    double locked_yaw,
    double hover_percentage,
    double max_normalized_thrust,
    double max_compensation_tilt_rad)
{
    RecoveryAttitudeCommand command;
    if (!finiteQuaternion(current_attitude) || !std::isfinite(locked_yaw) ||
        !std::isfinite(hover_percentage) || hover_percentage < 0.0 ||
        !std::isfinite(max_normalized_thrust) || max_normalized_thrust <= 0.0 ||
        !std::isfinite(max_compensation_tilt_rad) || max_compensation_tilt_rad <= 0.0 ||
        max_compensation_tilt_rad >= 0.5 * M_PI)
    {
        return command;
    }

    command.orientation = Eigen::Quaterniond(
        Eigen::AngleAxisd(locked_yaw, Eigen::Vector3d::UnitZ()));
    command.orientation.normalize();
    command.tilt_rad = attitudeTiltRad(current_attitude);

    const double current_vertical_projection = std::cos(command.tilt_rad);
    const double minimum_projection = std::cos(max_compensation_tilt_rad);
    const double compensated_projection = std::max(current_vertical_projection,
                                                    minimum_projection);
    command.normalized_thrust = std::max(0.0, std::min(
        hover_percentage / compensated_projection, max_normalized_thrust));
    command.valid = command.orientation.coeffs().allFinite() &&
                    std::isfinite(command.normalized_thrust);
    return command;
}

inline bool recoveryStateConverged(
    const Eigen::Vector3d &position,
    const Eigen::Vector3d &locked_position,
    const Eigen::Vector3d &velocity,
    const Eigen::Quaterniond &attitude,
    double max_position_error,
    double max_velocity_xy,
    double max_velocity_z,
    double max_tilt_rad)
{
    if (!position.allFinite() || !locked_position.allFinite() || !velocity.allFinite() ||
        !finiteQuaternion(attitude) || !std::isfinite(max_position_error) ||
        !std::isfinite(max_velocity_xy) || !std::isfinite(max_velocity_z) ||
        !std::isfinite(max_tilt_rad))
    {
        return false;
    }

    return (position - locked_position).norm() <= max_position_error &&
           velocity.head<2>().norm() <= max_velocity_xy &&
           std::abs(velocity.z()) <= max_velocity_z &&
           attitudeTiltRad(attitude) <= max_tilt_rad;
}

inline bool conservativeLastValidInput(
    const Eigen::Vector3d &velocity,
    const Eigen::Quaterniond &attitude,
    const Eigen::Vector4d &control_input,
    double max_velocity_xy,
    double max_velocity_z,
    double max_tilt_rad,
    double max_bodyrate,
    double min_thrust,
    double max_thrust)
{
    if (!velocity.allFinite() || !finiteQuaternion(attitude) ||
        !control_input.allFinite() || !std::isfinite(max_bodyrate) ||
        !std::isfinite(min_thrust) || !std::isfinite(max_thrust))
    {
        return false;
    }

    return velocity.head<2>().norm() <= max_velocity_xy &&
           std::abs(velocity.z()) <= max_velocity_z &&
           attitudeTiltRad(attitude) <= max_tilt_rad &&
           control_input.tail<3>().cwiseAbs().maxCoeff() <= max_bodyrate &&
           control_input.x() >= min_thrust && control_input.x() <= max_thrust;
}

} // namespace PayloadMPC

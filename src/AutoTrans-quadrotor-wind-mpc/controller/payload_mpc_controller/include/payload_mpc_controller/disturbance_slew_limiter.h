#pragma once

#include <algorithm>
#include <cmath>

#include <Eigen/Core>

namespace PayloadMPC
{

struct DisturbanceSlewLimitResult
{
    Eigen::Vector3d value{Eigen::Vector3d::Zero()};
    bool valid{false};
    bool limited_xy{false};
    bool limited_z{false};
};

class DisturbanceSlewLimiter
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    DisturbanceSlewLimitResult update(const Eigen::Vector3d &target,
                                      double dt,
                                      double max_rate_xy,
                                      double max_rate_z)
    {
        DisturbanceSlewLimitResult result;
        if (!target.allFinite() || !std::isfinite(dt) || dt <= 0.0 ||
            !std::isfinite(max_rate_xy) || max_rate_xy <= 0.0 ||
            !std::isfinite(max_rate_z) || max_rate_z <= 0.0)
        {
            reset();
            return result;
        }

        const Eigen::Vector2d target_xy = target.head<2>();
        const Eigen::Vector2d delta_xy = target_xy - value_.head<2>();
        const double max_delta_xy = max_rate_xy * dt;
        const double delta_xy_norm = delta_xy.norm();
        if (delta_xy_norm > max_delta_xy)
        {
            value_.head<2>() += delta_xy * (max_delta_xy / delta_xy_norm);
            result.limited_xy = true;
        }
        else
        {
            value_.head<2>() = target_xy;
        }

        const double delta_z = target.z() - value_.z();
        const double max_delta_z = max_rate_z * dt;
        if (std::abs(delta_z) > max_delta_z)
        {
            value_.z() += std::copysign(max_delta_z, delta_z);
            result.limited_z = true;
        }
        else
        {
            value_.z() = target.z();
        }

        result.value = value_;
        result.valid = true;
        return result;
    }

    void reset()
    {
        value_.setZero();
    }

    const Eigen::Vector3d &value() const
    {
        return value_;
    }

private:
    Eigen::Vector3d value_{Eigen::Vector3d::Zero()};
};

} // namespace PayloadMPC

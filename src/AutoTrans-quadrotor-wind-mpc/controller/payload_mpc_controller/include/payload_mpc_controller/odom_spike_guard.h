#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace PayloadMPC
{

struct OdomSpikeGuardConfig
{
    bool enabled{true};
    double max_sample_interval{0.10};
    double max_position_residual_xy{0.08};
    double max_position_residual_z{0.06};
    double max_velocity_jump_xy{0.60};
    double max_velocity_jump_z{0.60};
    double fault_duration{0.10};
    int recovery_good_samples{5};
};

enum class OdomSpikeGuardResult
{
    ACCEPTED,
    REJECTED_TRANSIENT,
    FAULT_LATCHED,
    REJECTED_PERSISTENT,
    RECOVERED
};

class OdomSpikeGuard
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit OdomSpikeGuard(const OdomSpikeGuardConfig &config)
        : config_(config)
    {
    }

    OdomSpikeGuardResult evaluate(double stamp,
                                  const Eigen::Vector3d &position,
                                  const Eigen::Vector3d &velocity)
    {
        if (!config_.enabled)
        {
            updateRawSample(stamp, position, velocity);
            clearFaultState();
            return OdomSpikeGuardResult::ACCEPTED;
        }

        const bool finite = std::isfinite(stamp) && position.allFinite() && velocity.allFinite();
        double position_residual_xy = 0.0;
        double position_residual_z = 0.0;
        double velocity_jump_xy = 0.0;
        double velocity_jump_z = 0.0;
        bool anomalous = !finite;

        if (finite && have_previous_raw_)
        {
            const double dt = stamp - previous_stamp_;
            if (dt > 0.0 && dt <= config_.max_sample_interval)
            {
                const Eigen::Vector3d predicted_position = previous_position_ + previous_velocity_ * dt;
                const Eigen::Vector3d position_residual = position - predicted_position;
                const Eigen::Vector3d velocity_jump = velocity - previous_velocity_;
                position_residual_xy = position_residual.head<2>().norm();
                position_residual_z = std::abs(position_residual.z());
                velocity_jump_xy = velocity_jump.head<2>().norm();
                velocity_jump_z = std::abs(velocity_jump.z());
                anomalous = position_residual_xy > config_.max_position_residual_xy ||
                            position_residual_z > config_.max_position_residual_z ||
                            velocity_jump_xy > config_.max_velocity_jump_xy ||
                            velocity_jump_z > config_.max_velocity_jump_z;
            }
        }

        last_position_residual_xy_ = position_residual_xy;
        last_position_residual_z_ = position_residual_z;
        last_velocity_jump_xy_ = velocity_jump_xy;
        last_velocity_jump_z_ = velocity_jump_z;
        updateRawSample(stamp, position, velocity);

        if (anomalous)
        {
            recovery_good_count_ = 0;
            if (!have_anomaly_start_)
            {
                have_anomaly_start_ = true;
                anomaly_start_stamp_ = stamp;
            }
            const double anomaly_duration = std::max(stamp - anomaly_start_stamp_, 0.0);
            if (!fault_active_ && anomaly_duration >= config_.fault_duration)
            {
                fault_active_ = true;
                return OdomSpikeGuardResult::FAULT_LATCHED;
            }
            return fault_active_ ? OdomSpikeGuardResult::REJECTED_PERSISTENT
                                 : OdomSpikeGuardResult::REJECTED_TRANSIENT;
        }

        if (fault_active_)
        {
            ++recovery_good_count_;
            if (recovery_good_count_ < config_.recovery_good_samples)
                return OdomSpikeGuardResult::REJECTED_PERSISTENT;

            clearFaultState();
            return OdomSpikeGuardResult::RECOVERED;
        }

        have_anomaly_start_ = false;
        anomaly_start_stamp_ = 0.0;
        recovery_good_count_ = 0;
        return OdomSpikeGuardResult::ACCEPTED;
    }

    bool faultActive() const { return fault_active_; }
    double lastPositionResidualXY() const { return last_position_residual_xy_; }
    double lastPositionResidualZ() const { return last_position_residual_z_; }
    double lastVelocityJumpXY() const { return last_velocity_jump_xy_; }
    double lastVelocityJumpZ() const { return last_velocity_jump_z_; }

private:
    void updateRawSample(double stamp,
                         const Eigen::Vector3d &position,
                         const Eigen::Vector3d &velocity)
    {
        if (!std::isfinite(stamp) || !position.allFinite() || !velocity.allFinite())
            return;
        previous_stamp_ = stamp;
        previous_position_ = position;
        previous_velocity_ = velocity;
        have_previous_raw_ = true;
    }

    void clearFaultState()
    {
        fault_active_ = false;
        have_anomaly_start_ = false;
        anomaly_start_stamp_ = 0.0;
        recovery_good_count_ = 0;
    }

    OdomSpikeGuardConfig config_;
    bool have_previous_raw_{false};
    double previous_stamp_{0.0};
    Eigen::Vector3d previous_position_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d previous_velocity_{Eigen::Vector3d::Zero()};
    bool have_anomaly_start_{false};
    double anomaly_start_stamp_{0.0};
    bool fault_active_{false};
    int recovery_good_count_{0};
    double last_position_residual_xy_{0.0};
    double last_position_residual_z_{0.0};
    double last_velocity_jump_xy_{0.0};
    double last_velocity_jump_z_{0.0};
};

} // namespace PayloadMPC

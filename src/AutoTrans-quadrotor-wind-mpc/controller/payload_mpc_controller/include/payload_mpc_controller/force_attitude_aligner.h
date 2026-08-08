#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace PayloadMPC
{
struct ForceAttitudeAlignmentConfig
{
    double duration{1.0};
    std::size_t min_samples{20};
    double max_body_rate{0.15};
    double max_speed{0.10};
    double max_tilt_error{5.0 * 3.14159265358979323846 / 180.0};
    double max_yaw_std{2.0 * 3.14159265358979323846 / 180.0};
};

enum class ForceAttitudeSampleResult
{
    ACCUMULATING,
    ALIGNED,
    ALREADY_ALIGNED,
    DUPLICATE,
    INVALID,
    MOVING,
    TILT_MISMATCH,
    YAW_UNSTABLE
};

// 在地面静止时估计 PX4 ENU 世界系到 FAST-LIO camera_init 世界系的固定偏航旋转。
// 输入四元数均表示机体系到各自世界系的姿态；Eigen 分量访问顺序为 w,x,y,z。
class ForceAttitudeAligner
{
public:
    explicit ForceAttitudeAligner(const ForceAttitudeAlignmentConfig &config)
        : config_(config)
    {
    }

    void reset()
    {
        aligned_ = false;
        yaw_offset_ = 0.0;
        yaw_std_ = 0.0;
        q_yaw_offset_.setIdentity();
        resetWindow();
    }

    ForceAttitudeSampleResult addSample(
        double sample_stamp,
        const Eigen::Quaterniond &q_lio,
        const Eigen::Quaterniond &q_px4,
        const Eigen::Vector3d &body_rate,
        const Eigen::Vector3d &lio_velocity)
    {
        if (aligned_)
        {
            return ForceAttitudeSampleResult::ALREADY_ALIGNED;
        }
        if (!std::isfinite(sample_stamp) || !quaternionFinite(q_lio) ||
            !quaternionFinite(q_px4) || !body_rate.allFinite() ||
            !lio_velocity.allFinite())
        {
            resetWindow();
            return ForceAttitudeSampleResult::INVALID;
        }
        if (have_last_stamp_ && sample_stamp <= last_sample_stamp_)
        {
            if (std::abs(sample_stamp - last_sample_stamp_) < 1.0e-9)
            {
                return ForceAttitudeSampleResult::DUPLICATE;
            }
            resetWindow();
            return ForceAttitudeSampleResult::INVALID;
        }
        if (body_rate.norm() > config_.max_body_rate ||
            lio_velocity.norm() > config_.max_speed)
        {
            resetWindow();
            return ForceAttitudeSampleResult::MOVING;
        }

        const Eigen::Quaterniond normalized_lio = q_lio.normalized();
        const Eigen::Quaterniond normalized_px4 = q_px4.normalized();
        const Eigen::Quaterniond q_relative =
            (normalized_lio * normalized_px4.conjugate()).normalized();
        const Eigen::Vector3d relative_rpy = quaternionToRpy(q_relative);
        if (std::abs(relative_rpy.x()) > config_.max_tilt_error ||
            std::abs(relative_rpy.y()) > config_.max_tilt_error)
        {
            resetWindow();
            return ForceAttitudeSampleResult::TILT_MISMATCH;
        }

        if (!have_first_stamp_)
        {
            first_sample_stamp_ = sample_stamp;
            have_first_stamp_ = true;
        }
        last_sample_stamp_ = sample_stamp;
        have_last_stamp_ = true;
        sin_sum_ += std::sin(relative_rpy.z());
        cos_sum_ += std::cos(relative_rpy.z());
        ++sample_count_;

        if (sample_count_ < config_.min_samples ||
            sample_stamp - first_sample_stamp_ < config_.duration)
        {
            return ForceAttitudeSampleResult::ACCUMULATING;
        }

        const double concentration = std::min(
            1.0,
            std::hypot(sin_sum_, cos_sum_) / static_cast<double>(sample_count_));
        yaw_std_ = concentration > 0.0
                       ? std::sqrt(std::max(0.0, -2.0 * std::log(concentration)))
                       : 3.14159265358979323846;
        if (!std::isfinite(yaw_std_) || yaw_std_ > config_.max_yaw_std)
        {
            resetWindow();
            return ForceAttitudeSampleResult::YAW_UNSTABLE;
        }

        yaw_offset_ = std::atan2(sin_sum_, cos_sum_);
        q_yaw_offset_ = Eigen::Quaterniond(
            Eigen::AngleAxisd(yaw_offset_, Eigen::Vector3d::UnitZ()));
        aligned_ = true;
        return ForceAttitudeSampleResult::ALIGNED;
    }

    bool transformPx4Attitude(
        const Eigen::Quaterniond &q_px4,
        Eigen::Quaterniond &q_force) const
    {
        if (!aligned_ || !quaternionFinite(q_px4))
        {
            return false;
        }
        q_force = (q_yaw_offset_ * q_px4.normalized()).normalized();
        return quaternionFinite(q_force);
    }

    bool aligned() const { return aligned_; }
    double yawOffset() const { return yaw_offset_; }
    double yawStd() const { return yaw_std_; }
    std::size_t sampleCount() const { return sample_count_; }

private:
    static bool quaternionFinite(const Eigen::Quaterniond &q)
    {
        return q.coeffs().allFinite() && std::isfinite(q.norm()) && q.norm() > 1.0e-6;
    }

    static Eigen::Vector3d quaternionToRpy(const Eigen::Quaterniond &q)
    {
        const double sin_roll = 2.0 * (q.w() * q.x() + q.y() * q.z());
        const double cos_roll = 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y());
        const double sin_pitch = std::max(
            -1.0,
            std::min(1.0, 2.0 * (q.w() * q.y() - q.z() * q.x())));
        const double sin_yaw = 2.0 * (q.w() * q.z() + q.x() * q.y());
        const double cos_yaw = 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z());
        return Eigen::Vector3d(
            std::atan2(sin_roll, cos_roll),
            std::asin(sin_pitch),
            std::atan2(sin_yaw, cos_yaw));
    }

    void resetWindow()
    {
        sample_count_ = 0;
        sin_sum_ = 0.0;
        cos_sum_ = 0.0;
        first_sample_stamp_ = 0.0;
        last_sample_stamp_ = 0.0;
        have_first_stamp_ = false;
        have_last_stamp_ = false;
    }

    ForceAttitudeAlignmentConfig config_;
    bool aligned_{false};
    double yaw_offset_{0.0};
    double yaw_std_{0.0};
    Eigen::Quaterniond q_yaw_offset_{Eigen::Quaterniond::Identity()};
    std::size_t sample_count_{0};
    double sin_sum_{0.0};
    double cos_sum_{0.0};
    double first_sample_stamp_{0.0};
    double last_sample_stamp_{0.0};
    bool have_first_stamp_{false};
    bool have_last_stamp_{false};
};
} // namespace PayloadMPC

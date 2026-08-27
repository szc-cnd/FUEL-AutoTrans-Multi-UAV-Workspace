#pragma once

#include <algorithm>
#include <cmath>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace PayloadMPC
{

enum class ForceObserverSyncResult
{
    READY,
    WAITING_FOR_INPUT,
    DUPLICATE_TARGET,
    OUT_OF_RANGE,
    INTERPOLATION_GAP,
    STALE,
    INVALID_OUTPUT
};

inline const char *forceObserverSyncResultName(ForceObserverSyncResult result)
{
    switch (result)
    {
    case ForceObserverSyncResult::READY: return "ready";
    case ForceObserverSyncResult::WAITING_FOR_INPUT: return "waiting_for_input";
    case ForceObserverSyncResult::DUPLICATE_TARGET: return "duplicate_target";
    case ForceObserverSyncResult::OUT_OF_RANGE: return "out_of_range";
    case ForceObserverSyncResult::INTERPOLATION_GAP: return "interpolation_gap";
    case ForceObserverSyncResult::STALE: return "stale";
    case ForceObserverSyncResult::INVALID_OUTPUT: return "invalid_output";
    }
    return "unknown";
}

enum class ForceObserverInputResult
{
    ACCEPTED,
    DUPLICATE,
    OUT_OF_ORDER,
    INVALID,
    SOURCE_RESET
};

struct ForceObserverSynchronizedInput
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    double stamp{0.0};
    double age{0.0};
    double acceleration_offset{0.0};
    double attitude_offset{0.0};
    double rpm_offset{0.0};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
    Eigen::Vector4d rpm{Eigen::Vector4d::Zero()};
};

class ForceObserverInputSynchronizer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    void configure(double history_duration,
                   double max_interpolation_gap,
                   double max_age,
                   double reset_backjump)
    {
        history_duration_ = history_duration;
        max_interpolation_gap_ = max_interpolation_gap;
        max_age_ = max_age;
        reset_backjump_ = reset_backjump;
        reset();
    }

    ForceObserverInputResult addAcceleration(double stamp, const Eigen::Vector3d &value)
    {
        return addSample(stamp, value, acceleration_samples_);
    }

    ForceObserverInputResult addAttitude(double stamp, const Eigen::Quaterniond &value)
    {
        if (!value.coeffs().allFinite() || value.norm() <= 1.0e-9)
            return ForceObserverInputResult::INVALID;
        return addSample(stamp, value.normalized(), attitude_samples_);
    }

    ForceObserverInputResult addRpm(double stamp, const Eigen::Vector4d &value)
    {
        return addSample(stamp, value, rpm_samples_);
    }

    ForceObserverSyncResult synchronize(double now, ForceObserverSynchronizedInput &output)
    {
        if (acceleration_samples_.empty() || attitude_samples_.empty() || rpm_samples_.empty())
            return ForceObserverSyncResult::WAITING_FOR_INPUT;

        const double target = std::min(
            acceleration_samples_.back().stamp,
            std::min(attitude_samples_.back().stamp, rpm_samples_.back().stamp));
        if (last_output_stamp_ > 0.0 && target <= last_output_stamp_ + kStampTolerance)
            return ForceObserverSyncResult::DUPLICATE_TARGET;
        if (!std::isfinite(now) || now + kStampTolerance < target || now - target > max_age_)
            return ForceObserverSyncResult::STALE;

        ForceObserverSyncResult result = interpolateVector(
            acceleration_samples_, target, output.acceleration);
        if (result != ForceObserverSyncResult::READY)
            return result;
        result = interpolateAttitude(attitude_samples_, target, output.attitude);
        if (result != ForceObserverSyncResult::READY)
            return result;
        result = interpolateVector(rpm_samples_, target, output.rpm);
        if (result != ForceObserverSyncResult::READY)
            return result;

        if (!output.acceleration.allFinite() || !output.attitude.coeffs().allFinite() ||
            output.attitude.norm() <= 1.0e-9 || !output.rpm.allFinite())
            return ForceObserverSyncResult::INVALID_OUTPUT;

        output.attitude.normalize();
        output.stamp = target;
        output.age = now - target;
        output.acceleration_offset = acceleration_samples_.back().stamp - target;
        output.attitude_offset = attitude_samples_.back().stamp - target;
        output.rpm_offset = rpm_samples_.back().stamp - target;
        last_output_stamp_ = target;
        return ForceObserverSyncResult::READY;
    }

    void reset()
    {
        acceleration_samples_.clear();
        attitude_samples_.clear();
        rpm_samples_.clear();
        last_output_stamp_ = 0.0;
    }

    std::size_t accelerationSampleCount() const { return acceleration_samples_.size(); }
    std::size_t attitudeSampleCount() const { return attitude_samples_.size(); }
    std::size_t rpmSampleCount() const { return rpm_samples_.size(); }

private:
    static constexpr double kStampTolerance = 1.0e-9;

    template <typename Value>
    struct TimedSample
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        double stamp;
        Value value;
    };

    template <typename Value>
    using SampleBuffer = std::vector<TimedSample<Value>, Eigen::aligned_allocator<TimedSample<Value>>>;

    template <typename Value>
    ForceObserverInputResult addSample(double stamp, const Value &value, SampleBuffer<Value> &samples)
    {
        if (!std::isfinite(stamp) || stamp <= 0.0 || !isFinite(value))
            return ForceObserverInputResult::INVALID;
        if (!samples.empty())
        {
            if (stamp + kStampTolerance < samples.back().stamp)
            {
                const double backjump = samples.back().stamp - stamp;
                if (backjump + kStampTolerance < reset_backjump_)
                    return ForceObserverInputResult::OUT_OF_ORDER;

                reset();
                samples.push_back({stamp, value});
                return ForceObserverInputResult::SOURCE_RESET;
            }
            if (stamp <= samples.back().stamp + kStampTolerance)
                return ForceObserverInputResult::DUPLICATE;
        }

        samples.push_back({stamp, value});
        const double cutoff = stamp - history_duration_;
        while (samples.size() > 2 && samples[1].stamp < cutoff)
            samples.erase(samples.begin());
        return ForceObserverInputResult::ACCEPTED;
    }

    template <typename Value>
    static bool isFinite(const Value &value)
    {
        return value.allFinite();
    }

    static bool isFinite(const Eigen::Quaterniond &value)
    {
        return value.coeffs().allFinite();
    }

    template <typename Value>
    ForceObserverSyncResult bracket(const SampleBuffer<Value> &samples,
                                    double target,
                                    std::size_t &lower,
                                    std::size_t &upper,
                                    double &alpha) const
    {
        if (samples.empty() || target + kStampTolerance < samples.front().stamp ||
            target > samples.back().stamp + kStampTolerance)
            return ForceObserverSyncResult::OUT_OF_RANGE;

        auto it = std::lower_bound(
            samples.begin(), samples.end(), target,
            [](const TimedSample<Value> &sample, double stamp) { return sample.stamp < stamp; });
        if (it != samples.end() && std::abs(it->stamp - target) <= kStampTolerance)
        {
            lower = upper = static_cast<std::size_t>(it - samples.begin());
            alpha = 0.0;
            return ForceObserverSyncResult::READY;
        }
        if (it == samples.begin() || it == samples.end())
            return ForceObserverSyncResult::OUT_OF_RANGE;

        upper = static_cast<std::size_t>(it - samples.begin());
        lower = upper - 1;
        const double gap = samples[upper].stamp - samples[lower].stamp;
        if (!std::isfinite(gap) || gap <= 0.0 || gap > max_interpolation_gap_)
            return ForceObserverSyncResult::INTERPOLATION_GAP;
        alpha = (target - samples[lower].stamp) / gap;
        return ForceObserverSyncResult::READY;
    }

    template <typename Value>
    ForceObserverSyncResult interpolateVector(const SampleBuffer<Value> &samples,
                                              double target,
                                              Value &value) const
    {
        std::size_t lower = 0;
        std::size_t upper = 0;
        double alpha = 0.0;
        const ForceObserverSyncResult result = bracket(samples, target, lower, upper, alpha);
        if (result != ForceObserverSyncResult::READY)
            return result;
        value = lower == upper
                    ? samples[lower].value
                    : samples[lower].value + alpha * (samples[upper].value - samples[lower].value);
        return ForceObserverSyncResult::READY;
    }

    ForceObserverSyncResult interpolateAttitude(const SampleBuffer<Eigen::Quaterniond> &samples,
                                                double target,
                                                Eigen::Quaterniond &attitude) const
    {
        std::size_t lower = 0;
        std::size_t upper = 0;
        double alpha = 0.0;
        const ForceObserverSyncResult result = bracket(samples, target, lower, upper, alpha);
        if (result != ForceObserverSyncResult::READY)
            return result;
        if (lower == upper)
        {
            attitude = samples[lower].value;
            return ForceObserverSyncResult::READY;
        }

        Eigen::Quaterniond upper_attitude = samples[upper].value;
        if (samples[lower].value.dot(upper_attitude) < 0.0)
            upper_attitude.coeffs() *= -1.0;
        attitude = samples[lower].value.slerp(alpha, upper_attitude).normalized();
        return ForceObserverSyncResult::READY;
    }

    double history_duration_{0.5};
    double max_interpolation_gap_{0.03};
    double max_age_{0.1};
    double reset_backjump_{0.1};
    double last_output_stamp_{0.0};
    SampleBuffer<Eigen::Vector3d> acceleration_samples_;
    SampleBuffer<Eigen::Quaterniond> attitude_samples_;
    SampleBuffer<Eigen::Vector4d> rpm_samples_;
};

} // namespace PayloadMPC

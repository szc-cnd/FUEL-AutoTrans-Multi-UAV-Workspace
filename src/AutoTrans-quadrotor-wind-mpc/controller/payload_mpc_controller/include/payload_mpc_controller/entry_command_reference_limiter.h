#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <cmath>

namespace PayloadMPC
{

// 入口 PositionCommand 的平移参考限速器；位置单位 m，速度 m/s，加速度 m/s^2。
// 它生成规划器世界系参考，不改变 MAVROS/PX4 的机体系角速度语义。
class EntryCommandReferenceLimiter
{
public:
  struct Reference
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
  };

  void configure(double max_velocity, double max_acceleration)
  {
    max_velocity_ = max_velocity;
    max_acceleration_ = max_acceleration;
  }

  void clear()
  {
    initialized_ = false;
    target_.setZero();
    reference_ = Reference();
  }

  void reset(const Eigen::Ref<const Eigen::Vector3d> &start,
             const Eigen::Ref<const Eigen::Vector3d> &target)
  {
    reference_.position = start;
    reference_.velocity.setZero();
    reference_.acceleration.setZero();
    target_ = target;
    initialized_ = start.allFinite() && target.allFinite();
  }

  void setTarget(const Eigen::Ref<const Eigen::Vector3d> &target)
  {
    if (target.allFinite())
      target_ = target;
  }

  bool initialized() const { return initialized_; }
  const Reference &reference() const { return reference_; }

  bool update(double dt)
  {
    if (!initialized_ || !std::isfinite(dt) || dt <= 0.0 ||
        !std::isfinite(max_velocity_) || max_velocity_ <= 0.0 ||
        !std::isfinite(max_acceleration_) || max_acceleration_ <= 0.0)
      return false;

    const Eigen::Vector3d previous_velocity = reference_.velocity;
    const Eigen::Vector3d error = target_ - reference_.position;
    const double distance = error.norm();
    Eigen::Vector3d desired_velocity = Eigen::Vector3d::Zero();
    if (distance > 1.0e-6)
    {
      const double braking_speed = std::sqrt(2.0 * max_acceleration_ * distance);
      desired_velocity = error / distance * std::min(max_velocity_, braking_speed);
    }

    Eigen::Vector3d velocity_delta = desired_velocity - previous_velocity;
    const double max_delta = max_acceleration_ * dt;
    if (velocity_delta.norm() > max_delta && max_delta > 0.0)
      velocity_delta *= max_delta / velocity_delta.norm();

    const Eigen::Vector3d next_velocity = previous_velocity + velocity_delta;
    const Eigen::Vector3d step = 0.5 * (previous_velocity + next_velocity) * dt;
    const bool can_safely_snap = distance > 1.0e-6 &&
                                 step.dot(error) >= distance * distance &&
                                 previous_velocity.norm() <= max_delta + 1.0e-9;

    if (can_safely_snap || distance <= 1.0e-6)
    {
      reference_.position = target_;
      reference_.velocity.setZero();
      reference_.acceleration = -previous_velocity / dt;
    }
    else
    {
      reference_.position += step;
      reference_.velocity = next_velocity;
      reference_.acceleration = velocity_delta / dt;
    }
    return reference_.position.allFinite() && reference_.velocity.allFinite() &&
           reference_.acceleration.allFinite();
  }

private:
  bool initialized_{false};
  double max_velocity_{0.2};
  double max_acceleration_{0.3};
  Eigen::Vector3d target_{Eigen::Vector3d::Zero()};
  Reference reference_;
};

}

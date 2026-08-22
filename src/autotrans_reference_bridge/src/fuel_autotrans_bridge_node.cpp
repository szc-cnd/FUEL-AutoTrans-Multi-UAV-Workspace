#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>

#include <bspline/Bspline.h>
#include <bspline/non_uniform_bspline.h>
#include <quadrotor_msgs/PolynomialTraj.h>

namespace
{

constexpr int kYawOrder = 3;
constexpr double kEpsilon = 1.0e-8;

class FuelAutoTransBridge
{
public:
  FuelAutoTransBridge()
      : nh_(), private_nh_("~"), last_input_time_(0), last_trajectory_id_(1), abort_sent_(false)
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/UAV0/planning/bspline");
    private_nh_.param<std::string>("output_topic", output_topic_,
                                   "/UAV0/planning/autotrans_trajectory");
    private_nh_.param<std::string>("frame_id", frame_id_, "UAV0/camera_init");
    private_nh_.param("trajectory_timeout", trajectory_timeout_, 2.0);

    output_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTraj>(output_topic_, 2, true);
    input_sub_ = nh_.subscribe(input_topic_, 2, &FuelAutoTransBridge::bsplineCallback, this);
    timeout_timer_ = nh_.createTimer(ros::Duration(0.05), &FuelAutoTransBridge::timeoutCallback, this);

    ROS_INFO("[fuel_autotrans_bridge] %s (bspline/Bspline) -> %s (quadrotor_msgs/PolynomialTraj), "
             "frame=%s timeout=%.3f s",
             input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(), trajectory_timeout_);
  }

private:
  bool finiteValue(double value) const
  {
    return std::isfinite(value);
  }

  bool unwrapYaw(const bspline::Bspline& input, Eigen::MatrixXd& yaw_points) const
  {
    if (input.yaw_pts.size() < static_cast<std::size_t>(kYawOrder + 1) ||
        !finiteValue(input.yaw_dt) || input.yaw_dt <= kEpsilon)
    {
      return false;
    }

    yaw_points.resize(static_cast<int>(input.yaw_pts.size()), 1);
    double previous = input.yaw_pts.front();
    if (!finiteValue(previous))
    {
      return false;
    }
    yaw_points(0, 0) = previous;
    for (std::size_t i = 1; i < input.yaw_pts.size(); ++i)
    {
      double current = input.yaw_pts[i];
      if (!finiteValue(current))
      {
        return false;
      }
      while (current - previous > M_PI)
      {
        current -= 2.0 * M_PI;
      }
      while (current - previous < -M_PI)
      {
        current += 2.0 * M_PI;
      }
      yaw_points(static_cast<int>(i), 0) = current;
      previous = current;
    }
    return true;
  }

  // FUEL 的 evaluateDeBoor() 在当前版本不是 const 成员函数；这里只读取样条，
  // 但接口必须保留可调用的非常量引用，避免复制或修改样条内部数据。
  bool fitPiece(fast_planner::NonUniformBspline& spline,
                double span_start_u,
                double duration,
                int degree,
                int dimension,
                bool yaw_piece,
                quadrotor_msgs::PolynomialMatrix& output) const
  {
    if (degree < 0 || degree > 12 || dimension <= 0 || duration <= kEpsilon)
    {
      return false;
    }

    Eigen::MatrixXd vandermonde(degree + 1, degree + 1);
    Eigen::MatrixXd values(degree + 1, dimension);
    for (int sample = 0; sample <= degree; ++sample)
    {
      const double t = duration * static_cast<double>(sample) / static_cast<double>(degree);
      double power = 1.0;
      for (int order = 0; order <= degree; ++order)
      {
        vandermonde(sample, order) = power;
        power *= t;
      }

      const Eigen::VectorXd value = spline.evaluateDeBoor(span_start_u + t);
      if (value.size() != dimension)
      {
        return false;
      }
      for (int dim = 0; dim < dimension; ++dim)
      {
        if (!finiteValue(value(dim)))
        {
          return false;
        }
        values(sample, dim) = value(dim);
      }
    }

    const Eigen::MatrixXd coefficients = vandermonde.fullPivLu().solve(values);
    if (!coefficients.allFinite())
    {
      return false;
    }

    output.num_order = static_cast<uint32_t>(degree);
    output.num_dim = static_cast<uint32_t>(dimension);
    output.duration = duration;
    output.data.reserve(static_cast<std::size_t>((degree + 1) * dimension));
    // AutoTrans treats the last matrix column as the constant term, so serialize
    // the ascending-power fit in reverse order.
    for (int order = degree; order >= 0; --order)
    {
      for (int dim = 0; dim < dimension; ++dim)
      {
        output.data.push_back(coefficients(order, dim));
      }
    }
    (void)yaw_piece;
    return true;
  }

  bool buildTrajectory(const bspline::Bspline& input,
                       quadrotor_msgs::PolynomialTraj& output) const
  {
    if (input.traj_id < 1 || input.traj_id > std::numeric_limits<uint32_t>::max() ||
        input.start_time.isZero() || input.order < 1 || input.order > 12)
    {
      ROS_ERROR_THROTTLE(1.0, "[fuel_autotrans_bridge] Reject invalid trajectory metadata.");
      return false;
    }

    const int point_count = static_cast<int>(input.pos_pts.size());
    const int degree = input.order;
    const int expected_knot_count = point_count + degree + 1;
    if (point_count < degree + 1 || static_cast<int>(input.knots.size()) != expected_knot_count)
    {
      ROS_ERROR_THROTTLE(1.0, "[fuel_autotrans_bridge] Reject B-spline: points/knots/order mismatch.");
      return false;
    }

    Eigen::MatrixXd position_points(point_count, 3);
    Eigen::VectorXd knots(input.knots.size());
    for (int i = 0; i < point_count; ++i)
    {
      position_points(i, 0) = input.pos_pts[i].x;
      position_points(i, 1) = input.pos_pts[i].y;
      position_points(i, 2) = input.pos_pts[i].z;
      if (!position_points.row(i).allFinite())
      {
        ROS_ERROR_THROTTLE(1.0, "[fuel_autotrans_bridge] Reject B-spline: non-finite position point.");
        return false;
      }
    }
    for (std::size_t i = 0; i < input.knots.size(); ++i)
    {
      knots(static_cast<int>(i)) = input.knots[i];
      if (!finiteValue(input.knots[i]) ||
          (i > 0 && input.knots[i] < input.knots[i - 1]))
      {
        ROS_ERROR_THROTTLE(1.0, "[fuel_autotrans_bridge] Reject B-spline: invalid knot vector.");
        return false;
      }
    }

    fast_planner::NonUniformBspline position_spline(position_points, degree, 0.1);
    position_spline.setKnot(knots);
    const int first_span = degree;
    const int end_span = point_count;
    const double total_duration = knots(end_span) - knots(first_span);
    if (!finiteValue(total_duration) || total_duration <= kEpsilon)
    {
      ROS_ERROR_THROTTLE(1.0, "[fuel_autotrans_bridge] Reject B-spline: invalid duration.");
      return false;
    }

    Eigen::MatrixXd yaw_points;
    if (!unwrapYaw(input, yaw_points))
    {
      ROS_ERROR_THROTTLE(1.0, "[fuel_autotrans_bridge] Reject B-spline: invalid yaw trajectory.");
      return false;
    }
    fast_planner::NonUniformBspline yaw_spline(yaw_points, kYawOrder, input.yaw_dt);
    const double yaw_duration = yaw_spline.getTimeSum();
    if (!finiteValue(yaw_duration) || std::fabs(yaw_duration - total_duration) > 0.02)
    {
      ROS_ERROR_THROTTLE(1.0,
                         "[fuel_autotrans_bridge] Reject B-spline: position/yaw duration mismatch "
                         "(%.4f vs %.4f s).",
                         total_duration, yaw_duration);
      return false;
    }

    output = quadrotor_msgs::PolynomialTraj();
    output.header.stamp = input.start_time;
    output.header.frame_id = frame_id_;
    output.trajectory_id = static_cast<uint32_t>(input.traj_id);
    output.action = quadrotor_msgs::PolynomialTraj::ACTION_ADD;
    output.has_yaw = true;
    output.trajectory.reserve(static_cast<std::size_t>(end_span - first_span));
    output.yaw_trajectory.reserve(static_cast<std::size_t>(end_span - first_span));

    for (int span = first_span; span < end_span; ++span)
    {
      const double duration = knots(span + 1) - knots(span);
      if (duration <= kEpsilon)
      {
        continue;
      }

      quadrotor_msgs::PolynomialMatrix position_piece;
      quadrotor_msgs::PolynomialMatrix yaw_piece;
      if (!fitPiece(position_spline, knots(span), duration, degree, 3, false, position_piece) ||
          !fitPiece(yaw_spline, (knots(span) - knots(first_span)) + yaw_spline.getKnot()(kYawOrder),
                    duration, kYawOrder, 1, true, yaw_piece))
      {
        ROS_ERROR_THROTTLE(1.0, "[fuel_autotrans_bridge] Reject B-spline: polynomial fit failed.");
        return false;
      }
      // 确保控制器的 yaw 与位置 piece 使用同一局部时间区间。
      yaw_piece.duration = position_piece.duration;
      output.trajectory.push_back(position_piece);
      output.yaw_trajectory.push_back(yaw_piece);
    }

    if (output.trajectory.empty() || output.trajectory.size() != output.yaw_trajectory.size())
    {
      ROS_ERROR_THROTTLE(1.0, "[fuel_autotrans_bridge] Reject B-spline: no valid pieces.");
      return false;
    }
    return true;
  }

  void bsplineCallback(const bspline::BsplineConstPtr& input)
  {
    quadrotor_msgs::PolynomialTraj output;
    if (!buildTrajectory(*input, output))
    {
      return;
    }
    output_pub_.publish(output);
    last_input_time_ = ros::Time::now();
    last_trajectory_id_ = output.trajectory_id;
    abort_sent_ = false;
    ROS_INFO_THROTTLE(1.0, "[fuel_autotrans_bridge] Forward FUEL trajectory id=%u, pieces=%zu, yaw=enabled.",
                      output.trajectory_id, output.trajectory.size());
  }

  void timeoutCallback(const ros::TimerEvent&)
  {
    if (last_input_time_.isZero() || abort_sent_ ||
        (ros::Time::now() - last_input_time_).toSec() <= trajectory_timeout_)
    {
      return;
    }

    quadrotor_msgs::PolynomialTraj abort_message;
    abort_message.header.stamp = ros::Time::now();
    abort_message.header.frame_id = frame_id_;
    abort_message.trajectory_id = last_trajectory_id_;
    abort_message.action = quadrotor_msgs::PolynomialTraj::ACTION_ABORT;
    abort_message.has_yaw = false;
    output_pub_.publish(abort_message);
    abort_sent_ = true;
    ROS_ERROR("[fuel_autotrans_bridge] FUEL B-spline timeout; ACTION_ABORT published.");
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber input_sub_;
  ros::Publisher output_pub_;
  ros::Timer timeout_timer_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double trajectory_timeout_;
  ros::Time last_input_time_;
  uint32_t last_trajectory_id_;
  bool abort_sent_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fuel_autotrans_bridge");
  FuelAutoTransBridge bridge;
  ros::spin();
  return 0;
}

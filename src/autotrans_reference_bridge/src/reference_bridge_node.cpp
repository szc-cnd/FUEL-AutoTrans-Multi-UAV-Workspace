#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <ros/ros.h>
#include <quadrotor_msgs/PolynomialTraj.h>
#include <traj_utils/PolyTraj.h>

namespace
{

class ReferenceBridge
{
public:
  ReferenceBridge()
      : private_nh_("~"), last_input_time_(0), last_trajectory_id_(1), abort_sent_(false)
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/drone_1_planning/trajectory");
    private_nh_.param<std::string>("output_topic", output_topic_,
                                   "/drone_1_planning/autotrans_trajectory");
    private_nh_.param<std::string>("frame_id", frame_id_, "camera_init");
    private_nh_.param("trajectory_timeout", trajectory_timeout_, 0.5);

    output_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTraj>(output_topic_, 2, true);
    input_sub_ = nh_.subscribe(input_topic_, 2, &ReferenceBridge::trajectoryCallback, this);
    timeout_timer_ = nh_.createTimer(ros::Duration(0.05), &ReferenceBridge::timeoutCallback, this);

    ROS_INFO("[autotrans_reference_bridge] %s -> %s, timeout=%.3f s, frame_id=%s",
             input_topic_.c_str(), output_topic_.c_str(), trajectory_timeout_, frame_id_.c_str());
  }

private:
  bool finiteValue(double value) const
  {
    return std::isfinite(value);
  }

  bool validateInput(const traj_utils::PolyTraj& input, std::size_t& segment_count,
                     std::size_t& coefficient_count) const
  {
    if (input.traj_id < 1)
    {
      ROS_ERROR_THROTTLE(1.0, "[autotrans_reference_bridge] Reject trajectory: traj_id must start from 1.");
      return false;
    }
    if (input.start_time.isZero())
    {
      ROS_ERROR_THROTTLE(1.0, "[autotrans_reference_bridge] Reject trajectory: start_time is zero.");
      return false;
    }

    segment_count = input.duration.size();
    coefficient_count = static_cast<std::size_t>(input.order) + 1;
    if (segment_count == 0 || coefficient_count == 0)
    {
      ROS_ERROR_THROTTLE(1.0, "[autotrans_reference_bridge] Reject trajectory: empty segments or coefficients.");
      return false;
    }

    const std::size_t expected_size = segment_count * coefficient_count;
    if (input.coef_x.size() != expected_size || input.coef_y.size() != expected_size ||
        input.coef_z.size() != expected_size)
    {
      ROS_ERROR_THROTTLE(1.0,
                         "[autotrans_reference_bridge] Reject trajectory: coefficient length mismatch.");
      return false;
    }

    for (std::size_t i = 0; i < segment_count; ++i)
    {
      if (!finiteValue(input.duration[i]) || input.duration[i] <= 0.0F)
      {
        ROS_ERROR_THROTTLE(1.0, "[autotrans_reference_bridge] Reject trajectory: invalid duration.");
        return false;
      }
    }
    for (std::size_t i = 0; i < expected_size; ++i)
    {
      if (!finiteValue(input.coef_x[i]) || !finiteValue(input.coef_y[i]) || !finiteValue(input.coef_z[i]))
      {
        ROS_ERROR_THROTTLE(1.0, "[autotrans_reference_bridge] Reject trajectory: non-finite coefficient.");
        return false;
      }
    }
    return true;
  }

  void trajectoryCallback(const traj_utils::PolyTrajConstPtr& input)
  {
    std::size_t segment_count = 0;
    std::size_t coefficient_count = 0;
    if (!validateInput(*input, segment_count, coefficient_count))
    {
      return;
    }

    quadrotor_msgs::PolynomialTraj output;
    // 输出 header.stamp 与 Diff-Planner 的 start_time 保持一致，便于 AutoTrans 做时间对齐。
    output.header.stamp = input->start_time;
    // frame_id 是 FAST-LIO 的雷达世界系名称，不做坐标旋转或平移。
    output.header.frame_id = frame_id_;
    output.trajectory_id = static_cast<std::uint32_t>(input->traj_id);
    output.action = quadrotor_msgs::PolynomialTraj::ACTION_ADD;
    output.trajectory.reserve(segment_count);

    for (std::size_t segment = 0; segment < segment_count; ++segment)
    {
      quadrotor_msgs::PolynomialMatrix piece;
      piece.num_order = input->order;
      piece.num_dim = 3;
      piece.duration = input->duration[segment];
      piece.data.reserve(3 * coefficient_count);

      // AutoTrans 用 Eigen::Map 按列主序恢复 [x; y; z]，因此这里逐阶交错三轴系数。
      for (std::size_t order = 0; order < coefficient_count; ++order)
      {
        const std::size_t index = segment * coefficient_count + order;
        piece.data.push_back(input->coef_x[index]);
        piece.data.push_back(input->coef_y[index]);
        piece.data.push_back(input->coef_z[index]);
      }
      output.trajectory.push_back(piece);
    }

    output_pub_.publish(output);
    last_input_time_ = ros::Time::now();
    last_trajectory_id_ = output.trajectory_id;
    abort_sent_ = false;
    ROS_INFO_THROTTLE(1.0, "[autotrans_reference_bridge] Forward trajectory id=%u, segments=%zu.",
                      output.trajectory_id, segment_count);
  }

  void timeoutCallback(const ros::TimerEvent&)
  {
    if (last_input_time_.isZero() || (ros::Time::now() - last_input_time_).toSec() <= trajectory_timeout_ ||
        abort_sent_)
    {
      return;
    }

    quadrotor_msgs::PolynomialTraj abort_message;
    abort_message.header.stamp = ros::Time::now();
    abort_message.header.frame_id = frame_id_;
    abort_message.trajectory_id = last_trajectory_id_;
    abort_message.action = quadrotor_msgs::PolynomialTraj::ACTION_ABORT;
    output_pub_.publish(abort_message);
    abort_sent_ = true;
    ROS_ERROR("[autotrans_reference_bridge] Input trajectory timeout; ACTION_ABORT published.");
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
  std::uint32_t last_trajectory_id_;
  bool abort_sent_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "autotrans_reference_bridge");
  ReferenceBridge bridge;
  ros::spin();
  return 0;
}

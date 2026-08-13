#include <cmath>
#include <cstdint>
#include <string>

#include <ros/ros.h>
#include <quadrotor_msgs/PolynomialTraj.h>
#include <traj_utils/PolyTraj.h>

namespace
{

class ReferenceBridge
{
public:
  ReferenceBridge() : private_nh_("~")
  {
    private_nh_.param<std::string>("input_topic", input_topic_, "/drone_1_planning/trajectory");
    private_nh_.param<std::string>("output_topic", output_topic_,
                                   "/drone_1_planning/autotrans_trajectory");
    private_nh_.param<std::string>("frame_id", frame_id_, "UAV1/camera_init");

    // 非 latched：控制器重启后不能自动收到上一次实验的旧轨迹。
    output_pub_ = nh_.advertise<quadrotor_msgs::PolynomialTraj>(output_topic_, 2, false);
    input_sub_ = nh_.subscribe(input_topic_, 2, &ReferenceBridge::trajectoryCallback, this);

    ROS_INFO("[autotrans_reference_bridge] %s -> %s, frame_id=%s; no replan timeout abort.",
             input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str());
  }

private:
  static bool finiteValue(double value)
  {
    return std::isfinite(value);
  }

  bool validateInput(const traj_utils::PolyTraj& input, std::size_t& segment_count,
                    std::size_t& coefficient_count) const
  {
    if (input.traj_id < 1)
    {
      ROS_ERROR_THROTTLE(1.0, "[autotrans_reference_bridge] 拒绝轨迹：traj_id 必须从 1 开始。");
      return false;
    }
    if (input.start_time.isZero())
    {
      ROS_ERROR_THROTTLE(1.0, "[autotrans_reference_bridge] 拒绝轨迹：start_time 为空。");
      return false;
    }

    segment_count = input.duration.size();
    coefficient_count = static_cast<std::size_t>(input.order) + 1;
    if (segment_count == 0 || coefficient_count == 0 || coefficient_count > 20)
    {
      ROS_ERROR_THROTTLE(1.0, "[autotrans_reference_bridge] 拒绝轨迹：段数或多项式阶数无效。");
      return false;
    }

    const std::size_t expected_size = segment_count * coefficient_count;
    if (input.coef_x.size() != expected_size || input.coef_y.size() != expected_size ||
        input.coef_z.size() != expected_size)
    {
      ROS_ERROR_THROTTLE(1.0, "[autotrans_reference_bridge] 拒绝轨迹：系数长度不匹配。");
      return false;
    }

    for (std::size_t i = 0; i < segment_count; ++i)
    {
      if (!finiteValue(input.duration[i]) || input.duration[i] <= 0.0F)
      {
        ROS_ERROR_THROTTLE(1.0, "[autotrans_reference_bridge] 拒绝轨迹：duration 无效。");
        return false;
      }
    }
    for (std::size_t i = 0; i < expected_size; ++i)
    {
      if (!finiteValue(input.coef_x[i]) || !finiteValue(input.coef_y[i]) ||
          !finiteValue(input.coef_z[i]))
      {
        ROS_ERROR_THROTTLE(1.0, "[autotrans_reference_bridge] 拒绝轨迹：存在非有限系数。");
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
    // 轨迹时间和坐标系必须与 FAST-LIO/AutoTrans 约定一致，不在桥接器中做坐标变换。
    output.header.stamp = input->start_time;
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

      // Eigen::Map 按列主序恢复 3 x (order + 1) 矩阵；按每阶 x/y/z 交错写入，
      // 与 traj_utils/PolyTraj 的 coef_x/coef_y/coef_z 语义保持一致。
      piece.data.reserve(3 * coefficient_count);
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
    ROS_INFO_THROTTLE(1.0, "[autotrans_reference_bridge] 已转发轨迹 id=%u，段数=%zu。",
                      output.trajectory_id, segment_count);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber input_sub_;
  ros::Publisher output_pub_;
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
};

}  // namespace

int main(int argc, char** argv)
{
  ros::init(argc, argv, "autotrans_reference_bridge");
  ReferenceBridge bridge;
  ros::spin();
  return 0;
}

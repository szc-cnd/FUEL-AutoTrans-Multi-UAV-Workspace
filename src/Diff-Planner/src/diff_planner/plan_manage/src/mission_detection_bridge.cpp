// Adapter between target_reporting (which already transforms and confirms the
// detector results) and the FUEL task-search interface.  The reporting package
// publishes positions in the competition `channel` frame; in this planner the
// `world` frame has the same coordinates, so this node deliberately performs
// no TF, odometry, or camera-extrinsic conversion.

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace {

constexpr int kColor = 0;
constexpr int kQrCode = 1;
constexpr int kThermal = 2;
constexpr int kTargetCount = 3;

const char* targetName(int type) {
  static const char* names[kTargetCount] = {"color", "qrcode", "thermal"};
  return type >= 0 && type < kTargetCount ? names[type] : "unknown";
}

int targetTypeFromReport(const std::string& target_type) {
  if (target_type == "color" || target_type == "color_tag") return kColor;
  if (target_type == "qrcode" || target_type == "qr_code") return kQrCode;
  if (target_type == "thermal" || target_type == "thermal_source") return kThermal;
  return -1;
}

bool finitePoint(const geometry_msgs::Point& point) {
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

}  // namespace

class MissionDetectionBridge {
 public:
  explicit MissionDetectionBridge(ros::NodeHandle& private_nh) : nh_(private_nh) {
    nh_.param("world_frame", world_frame_, std::string("world"));
    nh_.param("observation_input_topic", observation_input_topic_,
              std::string("/UAV0/target_reporting/observation"));
    nh_.param("input_frame", input_frame_, std::string("channel"));
    nh_.param("color_output_topic", output_topics_[kColor],
              std::string("/UAV0/mission/detection/color"));
    nh_.param("qrcode_output_topic", output_topics_[kQrCode],
              std::string("/UAV0/mission/detection/qrcode"));
    nh_.param("thermal_output_topic", output_topics_[kThermal],
              std::string("/UAV0/mission/detection/thermal"));
    nh_.param("color_candidate_topic", candidate_topics_[kColor],
              std::string("/UAV0/mission/detection/candidate/color"));
    nh_.param("qrcode_candidate_topic", candidate_topics_[kQrCode],
              std::string("/UAV0/mission/detection/candidate/qrcode"));
    nh_.param("thermal_candidate_topic", candidate_topics_[kThermal],
              std::string("/UAV0/mission/detection/candidate/thermal"));
    nh_.param("report_topic", report_topic_,
              std::string("/UAV0/mission/detection/report"));
    nh_.param("confirmation_count", confirmation_count_, 3);
    nh_.param("confirmation_radius", confirmation_radius_, 0.45);
    nh_.param("confirmation_max_gap", confirmation_max_gap_, 0.80);

    confirmation_count_ = std::max(1, confirmation_count_);
    confirmation_radius_ = std::max(0.01, confirmation_radius_);
    confirmation_max_gap_ = std::max(0.05, confirmation_max_gap_);

    observation_sub_ = nh_.subscribe(observation_input_topic_, 20,
                                     &MissionDetectionBridge::observationCallback, this);
    for (int type = 0; type < kTargetCount; ++type) {
      output_pubs_[type] =
          nh_.advertise<geometry_msgs::PoseStamped>(output_topics_[type], 1, true);
      candidate_pubs_[type] =
          nh_.advertise<geometry_msgs::PoseStamped>(candidate_topics_[type], 5, false);
    }
    report_pub_ = nh_.advertise<std_msgs::String>(report_topic_, 10, true);

    ROS_WARN("[mission_detection_bridge] ready: observation=%s input_frame=%s "
             "confirm_hits>=%d.",
             observation_input_topic_.c_str(), input_frame_.c_str(), confirmation_count_);
  }

 private:
  struct ConfirmationState {
    bool has_candidate{false};
    bool published{false};
    std::string target_id;
    int hits{0};
    ros::Time stamp;
    geometry_msgs::Point point;
  };

  void observationCallback(const std_msgs::StringConstPtr& msg) {
    try {
      std::stringstream input(msg->data);
      boost::property_tree::ptree report;
      boost::property_tree::read_json(input, report);

      const std::string message_type = report.get<std::string>("message_type", "");
      if (message_type != "observation" && message_type != "confirmed_target") return;

      const int type = targetTypeFromReport(report.get<std::string>("target_type", ""));
      if (type < 0 || type >= kTargetCount || states_[type].published) return;

      const std::string frame = report.get<std::string>("position.frame_id", "");
      const std::string unit = report.get<std::string>("position.unit", "m");
      // target_reporting uses the business name `channel`; `world` is accepted
      // too for a future reporter configured with the planner's native name.
      if ((!input_frame_.empty() && frame != input_frame_ && frame != world_frame_) ||
          unit != "m") {
        ROS_WARN_THROTTLE(1.0,
                          "[mission_detection_bridge] reject %s report: frame=%s unit=%s; "
                          "expected frame=%s and metres.",
                          targetName(type), frame.c_str(), unit.c_str(), input_frame_.c_str());
        return;
      }

      geometry_msgs::Point point;
      point.x = report.get<double>("position.x");
      point.y = report.get<double>("position.y");
      point.z = report.get<double>("position.z");
      if (!finitePoint(point)) {
        ROS_WARN_THROTTLE(1.0, "[mission_detection_bridge] reject %s report: non-finite point.",
                          targetName(type));
        return;
      }

      const std::string target_id = report.get<std::string>("target_id", "");
      const int hits = report.get<int>("hits", confirmation_count_);
      const bool confirmed = report.get<bool>("confirmed", hits >= confirmation_count_);
      const double timestamp = report.get<double>("timestamp", 0.0);
      const ros::Time stamp = timestamp > 0.0 ? ros::Time(timestamp) : ros::Time::now();
      if (!confirmed) {
        publishCandidate(type, stamp, point);
      }
      processDetection(type, target_id, hits, stamp, point);
    } catch (const std::exception& error) {
      ROS_WARN_THROTTLE(1.0, "[mission_detection_bridge] invalid observation JSON: %s",
                        error.what());
    }
  }

  void publishCandidate(int type, const ros::Time& stamp,
                        const geometry_msgs::Point& point) {
    if (type < 0 || type >= kTargetCount) return;
    geometry_msgs::PoseStamped candidate;
    candidate.header.stamp = stamp;
    candidate.header.frame_id = world_frame_;
    candidate.pose.position = point;
    candidate.pose.orientation.w = 1.0;
    candidate_pubs_[type].publish(candidate);
  }

  void processDetection(int type, const std::string& target_id, int hits,
                        const ros::Time& stamp, const geometry_msgs::Point& point) {
    if (type < 0 || type >= kTargetCount || states_[type].published) return;

    ConfirmationState& state = states_[type];
    const bool same_candidate =
        state.has_candidate && (target_id.empty() || state.target_id.empty() ||
                                target_id == state.target_id) &&
        std::fabs((stamp - state.stamp).toSec()) <= confirmation_max_gap_ &&
        distance(point, state.point) <= confirmation_radius_;
    if (!same_candidate) {
      state.hits = 0;
    }
    state.has_candidate = true;
    state.target_id = target_id;
    state.stamp = stamp;
    state.point = point;
    state.hits = std::max(state.hits + 1, hits);

    ROS_INFO_THROTTLE(0.5,
                      "[mission_detection_bridge] %s candidate hits=%d/%d channel=(%.2f, %.2f, %.2f).",
                      targetName(type), state.hits, confirmation_count_, point.x, point.y,
                      point.z);
    if (state.hits < confirmation_count_) return;

    geometry_msgs::PoseStamped output;
    output.header.stamp = stamp;
    output.header.frame_id = world_frame_;
    output.pose.position = point;
    output.pose.orientation.w = 1.0;
    output_pubs_[type].publish(output);
    state.published = true;

    std_msgs::String normalized_report;
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3)
           << "{\"detected\":true,\"type\":\"" << targetName(type)
           << "\",\"source_target_id\":\"" << target_id << "\",\"frame\":\""
           << world_frame_ << "\",\"x\":" << point.x << ",\"y\":" << point.y
           << ",\"z\":" << point.z
           << ",\"corridor_frame_valid\":true"
           << ",\"corridor_x\":" << point.x << ",\"corridor_y\":" << point.y
           << ",\"corridor_z\":" << point.z << ",\"confirmation_count\":"
           << state.hits << "}";
    normalized_report.data = stream.str();
    report_pub_.publish(normalized_report);
    ROS_WARN("[mission_detection_bridge] CONFIRMED %s at channel/world=(%.3f, %.3f, %.3f); "
             "published %s.",
             targetName(type), point.x, point.y, point.z, output_topics_[type].c_str());
  }

  static double distance(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
    const double dx = lhs.x - rhs.x;
    const double dy = lhs.y - rhs.y;
    const double dz = lhs.z - rhs.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  ros::NodeHandle nh_;
  ros::Subscriber observation_sub_;
  std::array<ros::Publisher, kTargetCount> output_pubs_;
  std::array<ros::Publisher, kTargetCount> candidate_pubs_;
  ros::Publisher report_pub_;
  std::array<ConfirmationState, kTargetCount> states_;

  std::string world_frame_;
  std::string observation_input_topic_;
  std::string input_frame_;
  std::array<std::string, kTargetCount> output_topics_;
  std::array<std::string, kTargetCount> candidate_topics_;
  std::string report_topic_;
  int confirmation_count_{3};
  double confirmation_radius_{0.45};
  double confirmation_max_gap_{0.80};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "mission_detection_bridge");
  ros::NodeHandle private_nh("~");
  try {
    MissionDetectionBridge bridge(private_nh);
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("[mission_detection_bridge] startup failed: %s", error.what());
    return 1;
  }
  return 0;
}

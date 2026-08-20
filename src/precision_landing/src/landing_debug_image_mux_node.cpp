#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>

#include <string>

namespace {

class LandingDebugImageMux {
 public:
  LandingDebugImageMux() : node_(), private_node_("~") {
    const std::string search_image_topic = private_node_.param<std::string>(
        "search_image_topic", "/landing/search/debug_image");
    const std::string precision_image_topic = private_node_.param<std::string>(
        "precision_image_topic", "/landing/debug_image");
    const std::string trigger_topic =
        private_node_.param<std::string>("trigger_topic", "/need_to_land");
    const std::string output_topic = private_node_.param<std::string>(
        "output_topic", "/landing/combined_debug_image");

    output_ = node_.advertise<sensor_msgs::Image>(output_topic, 1);
    search_subscriber_ = node_.subscribe(
        search_image_topic, 1, &LandingDebugImageMux::searchImageCallback, this);
    precision_subscriber_ = node_.subscribe(
        precision_image_topic, 1,
        &LandingDebugImageMux::precisionImageCallback, this);
    trigger_subscriber_ = node_.subscribe(
        trigger_topic, 2, &LandingDebugImageMux::triggerCallback, this);

    ROS_INFO_STREAM("Landing debug view ready: search=" << search_image_topic
                    << ", precision=" << precision_image_topic
                    << ", output=" << output_topic);
  }

 private:
  void triggerCallback(const std_msgs::BoolConstPtr& message) {
    if (message->data && !precision_view_active_) {
      precision_view_active_ = true;
      ROS_INFO("Landing debug view switched to precision landing");
    }
  }

  void searchImageCallback(const sensor_msgs::ImageConstPtr& message) {
    if (!precision_view_active_) {
      output_.publish(message);
    }
  }

  void precisionImageCallback(const sensor_msgs::ImageConstPtr& message) {
    if (precision_view_active_) {
      output_.publish(message);
    }
  }

  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::Subscriber search_subscriber_;
  ros::Subscriber precision_subscriber_;
  ros::Subscriber trigger_subscriber_;
  ros::Publisher output_;
  bool precision_view_active_{false};
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "landing_debug_image_mux");
  LandingDebugImageMux mux;
  ros::spin();
  return 0;
}

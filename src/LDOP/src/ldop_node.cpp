// 动静态点云分离，动态目标的处理和不确定性风险
#include <ros/ros.h>
#include <ldop/ldop.h>

int main(int argc, char** argv) {
  // 初始化 ROS 节点，并准备公有/私有命名空间句柄。
  ros::init(argc, argv, "ldop_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // 创建主对象后，会在构造函数里完成参数加载、接口注册和线程启动。
  ldopcore::Ldop node(nh, pnh);

  // AsyncSpinner 异步处理订阅回调，线程数交由 ROS 自行决定。
  ros::AsyncSpinner spinner(0);
  spinner.start();

  // 主线程只负责等待关闭信号。
  ros::waitForShutdown();
  return 0;
}

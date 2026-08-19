#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <sstream>
#include <string>

#include <Eigen/Core>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <ldop/DynamicObject.h>
#include <ldop/motion_model.h>
#include <ros/time.h>
#include <std_msgs/Header.h>
#include <visualization_msgs/Marker.h>

namespace ldopcore {

enum class ObjectClass : std::uint8_t {
  Unknown = 0U,
  Human = 1U,
  Vehicle = 2U,
  Uav = 3U,
  Other = 4U,
};

struct BoundingBox3D {
  geometry_msgs::Point center;
  geometry_msgs::Vector3 size;
  double yaw{0.0};
};

struct DynamicObjectDetection {
  // detection_id 仅在当前帧内有效，后续跟踪阶段会再分配稳定 track id。
  std::uint32_t detection_id{0U};
  BoundingBox3D bbox;
  ros::Time stamp;
  // 当前 detection 的点数只服务于 tracker 的合并/分离保护；
  // 不进入对外 ROS 消息，避免把聚类实现细节固化成公共接口。
  std::size_t point_count{0U};
  // 仅在 LDOP 内部模块间传递，不改变现有 ROS DynamicObject 消息接口。
  bool corridor_realtime_only{false};
  bool corridor_provisional{false};
  // mapper 通道状态机分配的内部源轨迹 ID；0 表示无通道来源。
  std::uint32_t corridor_source_track_id{0U};
  // 同一检测簇混入了多个 mapper 来源时保持显式冲突，不能把 source=0
  // 当成普通无来源点后重新关联到任一旧轨迹。
  bool corridor_source_conflict{false};
};

struct TrackHistorySample {
  // 轨迹历史保存模型原生状态；未来 predictor 按 motion_model_type 解释状态布局，
  // 避免 tracker 同时维护公共投影和模型向量两套状态来源。
  ros::Time stamp;
  ObjectClass object_class{ObjectClass::Unknown};
  MotionModelType motion_model_type{MotionModelType::CV3D};
  Eigen::VectorXd model_state;
  Eigen::MatrixXd model_covariance;
  bool matched{false};
};

// 几何点构造在 marker、跟踪和后续调试输出里都很常见，集中到这里避免各模块各写一份临时小函数。
inline geometry_msgs::Point makePoint(const double x, const double y, const double z) {
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

// 三维中心距离是几何语义，不属于 tracker 私有状态；放在通用层能减少模块内薄 helper。
inline double distance3D(const geometry_msgs::Point& lhs, const geometry_msgs::Point& rhs) {
  return std::hypot(std::hypot(lhs.x - rhs.x, lhs.y - rhs.y), lhs.z - rhs.z);
}

// ROS 点类型和 Eigen 向量的转换属于跨模块边界小操作，统一放在通用层避免各模块保留同义 helper。
inline Eigen::Vector3d pointToEigen(const geometry_msgs::Point& point) {
  return Eigen::Vector3d(point.x, point.y, point.z);
}

inline geometry_msgs::Vector3 eigenToVector(const Eigen::Vector3d& vector) {
  geometry_msgs::Vector3 ros_vector;
  ros_vector.x = vector.x();
  ros_vector.y = vector.y();
  ros_vector.z = vector.z();
  return ros_vector;
}

// Vector3 的分量级比较属于通用几何小操作，集中后 tracker 不需要保留同义薄 helper。
inline double maxComponent(const geometry_msgs::Vector3& vector) {
  return std::max({vector.x, vector.y, vector.z});
}

inline geometry_msgs::Vector3 componentMax(const geometry_msgs::Vector3& lhs,
                                           const geometry_msgs::Vector3& rhs) {
  geometry_msgs::Vector3 result;
  result.x = std::max(lhs.x, rhs.x);
  result.y = std::max(lhs.y, rhs.y);
  result.z = std::max(lhs.z, rhs.z);
  return result;
}

// RViz DELETEALL marker 只依赖 header 和可选 namespace，各可视化模块共用同一份构造规则。
inline visualization_msgs::Marker makeDeleteAllMarker(const std_msgs::Header& header,
                                                      const std::string& marker_namespace = "") {
  visualization_msgs::Marker marker;
  marker.header = header;
  marker.ns = marker_namespace;
  marker.id = 0;
  marker.action = visualization_msgs::Marker::DELETEALL;
  return marker;
}

// 内部枚举转 ROS 消息常量；DynamicObject 和 DynamicObjectPrediction 的 CLASS_* 值相同。
inline std::uint8_t toRosObjectClass(const ObjectClass object_class) {
  switch (object_class) {
    case ObjectClass::Human:
      return ldop::DynamicObject::CLASS_HUMAN;
    case ObjectClass::Vehicle:
      return ldop::DynamicObject::CLASS_VEHICLE;
    case ObjectClass::Uav:
      return ldop::DynamicObject::CLASS_UAV;
    case ObjectClass::Other:
      return ldop::DynamicObject::CLASS_OTHER;
    case ObjectClass::Unknown:
      break;
  }
  return ldop::DynamicObject::CLASS_UNKNOWN;
}

// 内部枚举转 ROS 消息常量；DynamicObject 和 DynamicObjectPrediction 的 MOTION_MODEL_* 值相同。
inline std::uint8_t toRosMotionModelType(const MotionModelType model_type) {
  switch (model_type) {
    case MotionModelType::CA2D:
      return ldop::DynamicObject::MOTION_MODEL_CA2D;
    case MotionModelType::CA3D:
      return ldop::DynamicObject::MOTION_MODEL_CA3D;
    case MotionModelType::CV3D:
      return ldop::DynamicObject::MOTION_MODEL_CV3D;
    case MotionModelType::CTRA:
      return ldop::DynamicObject::MOTION_MODEL_CTRA;
  }
  return ldop::DynamicObject::MOTION_MODEL_CV3D;
}

// 统一把 steady_clock 的时间差转换成毫秒，避免各模块重复写样板代码。
inline double elapsedMs(const std::chrono::steady_clock::time_point& start,
                        const std::chrono::steady_clock::time_point& end) {
  return std::chrono::duration<double, std::milli>(end - start).count();
}

// 统一毫秒日志输出精度，保证不同模块的 timing 日志格式一致。
inline std::string formatFloatMs(const double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(3) << value;
  return stream.str();
}

struct LdopProcessingTimingTotals {
  // 非 verbose 日志只保留用户需要的模块主流程耗时；这里不混入发布耗时和回调耗时，
  // 避免“模块耗时”和 LDOP 外层调度耗时两个口径互相污染。
  double map_module_ms{0.0};
  double cluster_module_ms{0.0};
  double tracking_module_ms{0.0};
  double prediction_module_ms{0.0};
  double ldop_total_ms{0.0};
};

// 非 verbose 汇总行是运行时排查性能的固定契约，集中格式化能避免 processingLoop 里堆叠字符串细节。
inline std::string formatLdopProcessingTimingSummary(
    const std::uint64_t sequence,
    const LdopProcessingTimingTotals& timing) {
  std::ostringstream stream;
  stream << "LDOP timing summary frame #" << sequence
         << ": mapModule=" << formatFloatMs(timing.map_module_ms)
         << "ms, clusterModule=" << formatFloatMs(timing.cluster_module_ms)
         << "ms, trackingModule=" << formatFloatMs(timing.tracking_module_ms)
         << "ms, predictionModule=" << formatFloatMs(timing.prediction_module_ms)
         << "ms, total=" << formatFloatMs(timing.ldop_total_ms) << "ms";
  return stream.str();
}

}  // namespace ldopcore

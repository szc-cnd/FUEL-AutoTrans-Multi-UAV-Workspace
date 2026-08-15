#pragma once

#include <cstddef>
#include <vector>

#include <ldop/ufomap_mapper.h>
#include <ldop/utils.h>
#include <ros/node_handle.h>
#include <std_msgs/Header.h>
#include <ufo/map/types.hpp>
#include <visualization_msgs/MarkerArray.h>

namespace ldopcore {

struct DynamicObjectClustererConfig {
  std::size_t min_points{5U};
  double max_extent{3.0};
  int connectivity{6};
  double vertical_merge_max_z_gap{0.4};
  double vertical_merge_min_xy_overlap_ratio{0.3};
};

struct DynamicObjectClustererParams {
  int min_points{5};           //大于0
  double max_extent{3.0};      //大于0
  int connectivity{6};         // 只能6/18/26三选一
  double vertical_merge_max_z_gap{0.4};             //大于0 ，上下碎片合并允许的竖向空隙，单位米；只用于当前帧内的检测后处理。
  double vertical_merge_min_xy_overlap_ratio{0.3};  //大于0 ，水平投影有面积重叠时，交叠面积至少占较小投影面积的比例；用于避免擦边误合并。
};

struct DynamicObjectClustererTimingStats {
  // 单帧动态聚类主流程整体耗时，单位为毫秒。
  double process_dynamic_objects_ms{0.0};
  // 检测框生成与 marker 构建的分阶段耗时，单位为毫秒。
  double build_detections_ms{0.0};
  double build_object_markers_ms{0.0};
};

struct DynamicObjectClustererFrameResult {
  std::vector<DynamicObjectDetection> detections;
  visualization_msgs::MarkerArray dynamic_object_markers_msg;
  DynamicObjectClustererTimingStats timing;
};

class DynamicObjectClusterer {
 public:
  // 构造时直接从 ROS 参数服务器读取聚类参数，并在内部构建运行时配置。
  // verbose 只控制模块自己的 timing 日志，不影响聚类结果。
  DynamicObjectClusterer(ros::NodeHandle& pnh, bool verbose = false);

  // 聚类模块自己的主流程：统一生成当前帧动态目标与对应可视化，并在模块内记录日志。
  DynamicObjectClustererFrameResult processDynamicObjects(
      const std_msgs::Header& header,
      const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const;

 private:
  // 统一在模块内部读取 ROS 参数并构建运行时配置；clusterer 没有额外 runtime 重建阶段。
  void loadParameters();

  // 下面两个子步骤只服务于模块内部主流程，不再额外暴露测试入口。
  std::vector<DynamicObjectDetection> buildDetections(
      const ros::Time& stamp,
      const std::vector<UfomapDynamicClusterPoint>& dynamic_points) const;

  // 为当前帧动态目标生成 RViz 线框包围盒；每帧先发 DELETEALL 清理旧框。
  visualization_msgs::MarkerArray buildObjectMarkers(
      const std_msgs::Header& header,
      const std::vector<DynamicObjectDetection>& detections) const;
  ros::NodeHandle pnh_;
  DynamicObjectClustererParams params_;
  DynamicObjectClustererConfig config_;
  bool verbose_{false};
};

}  // namespace ldopcore

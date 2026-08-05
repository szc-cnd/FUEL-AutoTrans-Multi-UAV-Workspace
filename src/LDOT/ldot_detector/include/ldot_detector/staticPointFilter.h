/*
    FILE: staticPointFilter.h
    ---------------------------------
    静态点滤波器头文件
*/
#ifndef ONBOARDDETECTOR_STATICPOINTFILTER_H
#define ONBOARDDETECTOR_STATICPOINTFILTER_H

#include <Eigen/Dense>
#include <ldot_detector/lidarDetector.h>
#include <ldot_detector/utils.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <cstdint>  // 2026-07-27: 连续命中使用单调帧编号，避免时间戳回绕影响判定。
#include <unordered_map>
#include <unordered_set>

namespace onboardDetector {

// 体素状态结构体
struct VoxelStatus {
  int hit_count;
  double last_seen_time;
  // 2026-07-27: 保留最近32个处理帧的命中窗口；稀疏Livox无需逐帧连续命中，摆球又不会因长期累计烧入背景。
  std::uint64_t hit_window;
  std::uint64_t last_hit_frame;
  bool confirmed_static;
  // 2026-07-27: 保存体素索引，供动态历史扫掠区域反向清除哈希表元素。
  int x_idx;
  int y_idx;
  int z_idx;

  VoxelStatus()
      : hit_count(0), last_seen_time(0.0), hit_window(0), last_hit_frame(0),
        confirmed_static(false), x_idx(0), y_idx(0), z_idx(0) {}
};

class StaticPointFilter {
public:
  StaticPointFilter();
  ~StaticPointFilter();

  // 设置参数
  void setParams(bool enabled, float voxel_size, int hit_threshold,
                 double time_threshold, int ray_cast_decrement = 1);

  // 仅更新地图
  // sensor_position: 传感器在全局坐标系中的位置（用于近距离累积抑制）
  // protected_boxes: 动态物体保护区域（可选），射线投射时会跳过这些区域
  void updateMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                 double current_time,
                 const Eigen::Vector3d &sensor_position,
                 const std::vector<onboardDetector::box3D> *protected_boxes = nullptr);

  // 点级过滤 (原 filter 函数)
  void filterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                    const std::vector<onboardDetector::box3D> &protected_boxes =
                        std::vector<onboardDetector::box3D>());

  // 2026-07-27: 检测会话启停时清空背景；动态轨迹确认后主动清除包围盒/扫掠区域污染。
  void reset();
  void clearBoxes(const std::vector<onboardDetector::box3D> &boxes);

  // 2026-07-27: 向检测主循环公开背景规模，便于日志直接验证静态建模是否真正收敛。
  std::size_t voxelCount() const;
  std::size_t confirmedStaticVoxelCount() const;

  
  // 清理旧体素
  void cleanMap(double current_time);

  // 检查带尺寸的包围框是否与静态地图碰撞
  // center: 包围框中心位置
  // size: 包围框尺寸 (x_width, y_width, z_width)
  // inflation: 膨胀系数（米），用于安全裕度
  bool checkBoxCollision(const Eigen::Vector3d &center,
                         const Eigen::Vector3d &size, double inflation = 0.0);

private:
  // 计算体素键值的辅助函数
  long long getVoxelKey(const pcl::PointXYZ &point);
  
  // 【新增】从三维坐标计算体素键值
  long long getVoxelKeyFromCoords(int x_idx, int y_idx, int z_idx);

  // 【新增】使用 Bresenham 3D 算法进行射线投射
  // 从 sensor_position 到 end_point 的射线路径上的所有体素进行清除操作
  // protected_boxes: 动态物体保护区域（可选），射线投射时会跳过这些区域
  void rayCast(const Eigen::Vector3d &sensor_position,
               const pcl::PointXYZ &end_point,
               const std::vector<onboardDetector::box3D> *protected_boxes = nullptr);

  // 检查点是否在边界框内
  bool isPointInBox(const pcl::PointXYZ &pt, const onboardDetector::box3D &box);

  // 检查点是否为静态 (基于当前地图) - 原始版本
  bool isPointStatic(const pcl::PointXYZ &pt);

  // 距离自适应阈值 - 远距离降低判定门槛
  // 使用成员变量 sensor_position_ 计算距离
  int getAdaptiveThreshold(const pcl::PointXYZ &pt);

  // 2026-07-27: 将某体素的滑动命中窗口推进到当前帧，并同步静态确认状态。
  void advanceObservationWindow(VoxelStatus &status, std::uint64_t frame_index,
                                int adaptive_threshold);

  bool enabled_;
  float voxel_size_;
  int hit_threshold_;
  double time_threshold_;

  // 射线投射相关参数
  int ray_cast_decrement_;        // 射线穿过体素时的递减值
  int ray_cast_skip_counter_;     // 射线投射跳过计数器（用于降采样）

  // 帧计数器，用于控制清理频率
  int frame_count_;
  // 2026-07-27: 每次 updateMap 只允许同一体素增加一次连续帧计数。
  std::uint64_t map_update_index_;
  static constexpr std::uint64_t kObservationWindowFrames = 32;

  // 体素地图：键值 -> 状态
  std::unordered_map<long long, VoxelStatus> voxel_map_;

  // 【新增】当前帧命中的体素集合，用于射线投射时保护当前帧观测到的体素
  std::unordered_set<long long> current_frame_hits_;

  // 【新增】传感器位置（全局坐标系），用于距离计算
  // 在 updateMap 时更新，供 isPointStatic 等函数使用
  Eigen::Vector3d sensor_position_;
};

} // namespace onboardDetector

#endif // ONBOARDDETECTOR_STATICPOINTFILTER_H

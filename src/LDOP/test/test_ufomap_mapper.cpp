#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <ldop/ufomap_mapper.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>

namespace ldopcore {
namespace {

sensor_msgs::PointCloud2 makeCloud(const std::vector<geometry_msgs::Point>& points,
                                   const double stamp = 10.0) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp = ros::Time(stamp);

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  for (const auto& point : points) {
    *iter_x = static_cast<float>(point.x);
    *iter_y = static_cast<float>(point.y);
    *iter_z = static_cast<float>(point.z);
    ++iter_x;
    ++iter_y;
    ++iter_z;
  }
  cloud.is_dense = true;
  return cloud;
}

geometry_msgs::Point makePoint(const double x, const double y, const double z) {
  geometry_msgs::Point point;
  point.x = x;
  point.y = y;
  point.z = z;
  return point;
}

nav_msgs::Odometry makeOdom(const geometry_msgs::Point& position,
                            const double stamp = 10.0,
                            const double yaw = 0.0) {
  nav_msgs::Odometry odom;
  odom.header.frame_id = "map";
  odom.header.stamp = ros::Time(stamp);
  odom.pose.pose.position = position;
  odom.pose.pose.orientation.z = std::sin(0.5 * yaw);
  odom.pose.pose.orientation.w = std::cos(0.5 * yaw);
  return odom;
}

std::unique_ptr<UfomapMapper> makeMapper(const std::string& test_namespace,
                                         const bool corridor_enabled = false,
                                         const bool ground_filter_enabled = false,
                                         const int warmup_frames = 0) {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~" + test_namespace);

  pnh.setParam("ufomap_resolution", 0.5);
  pnh.setParam("ufomap_depth_levels", 8);
  pnh.setParam("ufomap_min_range", 0.0);
  pnh.setParam("ufomap_max_range", 8.0);
  pnh.setParam("ufomap_insert_hit_depth", 0);
  pnh.setParam("ufomap_insert_miss_depth", 0);
  pnh.setParam("ufomap_ray_casting_depth", 0);
  pnh.setParam("ufomap_num_threads", 0);
  pnh.setParam("ufomap_insert_only_valid", false);
  pnh.setParam("ufomap_inflate_unknown", 0);
  pnh.setParam("ufomap_inflate_unknown_compensation", false);
  pnh.setParam("ufomap_ray_passthrough_hits", false);
  pnh.setParam("ufomap_inflate_hits_dist", 0.0);
  pnh.setParam("ufomap_simple_ray_casting", false);
  pnh.setParam("ufomap_sliding_window_size", 0);
  pnh.setParam("ufomap_parallel", false);
  pnh.setParam("ufomap_propagate", true);
  pnh.setParam("ufomap_down_sampling_method", "none");
  pnh.setParam("static_map_visualization_max_z", 5.0);
  pnh.setParam("ufomap_ground_filter_enabled", ground_filter_enabled);
  pnh.setParam("ufomap_warmup_frames", warmup_frames);
  if (ground_filter_enabled) {
    pnh.setParam("ufomap_ground_estimation_frames", 1);
    pnh.setParam("ufomap_ground_estimation_min_points", 20);
    pnh.setParam("ufomap_ground_estimation_radius", 4.0);
    pnh.setParam("ufomap_ground_estimation_candidate_band", 0.10);
    pnh.setParam("ufomap_ground_estimation_inlier_threshold", 0.02);
  }
  if (corridor_enabled) {
    pnh.setParam("corridor_dynamic_enabled", true);
    pnh.setParam("corridor_width", 1.5);
    pnh.setParam("corridor_wall_clearance", 0.10);
    pnh.setParam("corridor_roi_min_forward", -0.50);
    pnh.setParam("corridor_roi_max_forward", 3.00);
    pnh.setParam("corridor_roi_min_z", 0.15);
    pnh.setParam("corridor_roi_max_z", 1.50);
    pnh.setParam("corridor_detection_voxel", 0.10);
    pnh.setParam("corridor_history_frames", 5);
    pnh.setParam("corridor_min_confirm_hits", 3);
    pnh.setParam("corridor_min_lateral_speed", 0.20);
    pnh.setParam("corridor_min_lateral_span", 0.12);
    pnh.setParam("corridor_publish_unknown_as_dynamic", true);
    pnh.setParam("corridor_static_confirm_frames", 4);
    pnh.setParam("corridor_confirmed_static_confirm_frames", 15);
    pnh.setParam("corridor_static_lateral_speed", 0.20);
    pnh.setParam("corridor_reactivation_displacement", 0.15);
    pnh.setParam("corridor_forward_alignment_cos", 0.85);
    pnh.setParam("corridor_max_missed_frames", 5);
    pnh.setParam("corridor_max_forward_speed", 0.80);
    pnh.setParam("corridor_max_vertical_speed", 0.80);
    pnh.setParam("corridor_association_gate", 0.25);
    pnh.setParam("corridor_track_timeout", 0.60);
    pnh.setParam("corridor_min_cluster_points", 3);
    pnh.setParam("corridor_min_confirm_extent", 0.18);
    pnh.setParam("corridor_min_confirm_second_extent", 0.06);
    pnh.setParam("corridor_min_confirm_extent_frames", 2);
    pnh.setParam("corridor_max_cluster_extent", 0.60);
    pnh.setParam("corridor_max_candidates", 1);
    pnh.setParam("corridor_reject_candidate_count", 12);
    pnh.setParam("corridor_min_wall_points", 4);
  }

  return std::make_unique<UfomapMapper>(nh, pnh, false);
}

void integrateHits(UfomapMapper& mapper, const std::vector<geometry_msgs::Point>& hits) {
  mapper.processInputCloud(makeCloud(hits), makeOdom(makePoint(0.0, 0.0, 0.0)));
}

std::vector<geometry_msgs::Point> makeBallCluster(const double ball_x,
                                                  const double ball_y) {
  std::vector<geometry_msgs::Point> points;
  // 直径约0.3m的紧凑球点簇，真实回波跨度超过确认门并覆盖多个0.1m检测体素。
  for (int index = 0; index < 8; ++index) {
    points.push_back(makePoint(ball_x + 0.08 * static_cast<double>(index % 4),
                               ball_y + 0.08 * static_cast<double>(index / 4),
                               0.60 + 0.08 * static_cast<double>(index % 2)));
  }
  return points;
}

std::vector<geometry_msgs::Point> makeStaticCorridorWalls() {
  std::vector<geometry_msgs::Point> points;
  for (int x_index = -5; x_index <= 5; ++x_index) {
    const double x = 0.15 * static_cast<double>(x_index);
    points.push_back(makePoint(x, -0.75, 0.35));
    points.push_back(makePoint(x, -0.75, 0.55));
    points.push_back(makePoint(x, 0.75, 0.35));
    points.push_back(makePoint(x, 0.75, 0.55));
  }
  return points;
}

std::vector<geometry_msgs::Point> makeCorridorFrame(const double ball_y,
                                                    const double ball_x = 0.45) {
  auto points = makeStaticCorridorWalls();
  const auto ball = makeBallCluster(ball_x, ball_y);
  points.insert(points.end(), ball.begin(), ball.end());
  return points;
}

std::vector<geometry_msgs::Point> makeElevatedGroundFrame(const double ball_y) {
  std::vector<geometry_msgs::Point> points;
  for (int x_index = -5; x_index <= 5; ++x_index) {
    for (int y_index = -3; y_index <= 3; ++y_index) {
      points.push_back(makePoint(0.15 * static_cast<double>(x_index),
                                 2.00 + 0.15 * static_cast<double>(y_index), 0.95));
    }
  }
  const auto ball = makeBallCluster(0.45, ball_y);
  points.insert(points.end(), ball.begin(), ball.end());
  return points;
}

void appendCompactCluster(std::vector<geometry_msgs::Point>& points,
                          const double center_x,
                          const double center_y) {
  points.push_back(makePoint(center_x - 0.02, center_y - 0.02, 0.58));
  points.push_back(makePoint(center_x + 0.02, center_y - 0.02, 0.58));
  points.push_back(makePoint(center_x - 0.02, center_y + 0.02, 0.62));
  points.push_back(makePoint(center_x + 0.02, center_y + 0.02, 0.62));
}

void appendSlenderCluster(std::vector<geometry_msgs::Point>& points,
                          const double center_x,
                          const double center_y) {
  for (std::size_t index = 0U; index < 4U; ++index) {
    const double x = center_x - 0.12 + 0.08 * static_cast<double>(index);
    points.push_back(makePoint(x, center_y - 0.02, 0.59));
    points.push_back(makePoint(x, center_y + 0.02, 0.61));
  }
}

std::vector<geometry_msgs::Point> makeSparseCorridorFrame(const double ball_y,
                                                          const double ball_x = 0.45) {
  auto points = makeStaticCorridorWalls();
  appendCompactCluster(points, ball_x, ball_y);
  return points;
}

TEST(UfomapMapperTest, QueryNodeReportsOccupiedUnknownAndOutOfMapSeparately) {
  auto mapper = makeMapper("ufomap_mapper_states");
  integrateHits(*mapper, {makePoint(2.0, 0.0, 0.0)});

  const auto occupied = mapper->queryNode(ufo::Point(2.0F, 0.0F, 0.0F));
  EXPECT_EQ(occupied.state, UfomapOccupancyState::Occupied);
  EXPECT_TRUE(occupied.exists);
  EXPECT_TRUE(occupied.occupied);
  EXPECT_FALSE(occupied.free);
  EXPECT_FALSE(occupied.unknown);

  const auto unknown = mapper->queryNode(ufo::Point(0.0F, 2.0F, 0.0F));
  EXPECT_EQ(unknown.state, UfomapOccupancyState::Unknown);
  EXPECT_FALSE(unknown.exists);
  EXPECT_FALSE(unknown.occupied);
  EXPECT_FALSE(unknown.free);
  EXPECT_TRUE(unknown.unknown);

  const auto out_of_map = mapper->queryNode(ufo::Point(1.0e6F, 0.0F, 0.0F));
  EXPECT_EQ(out_of_map.state, UfomapOccupancyState::OutOfMap);
  EXPECT_FALSE(out_of_map.exists);
  EXPECT_FALSE(out_of_map.occupied);
  EXPECT_FALSE(out_of_map.free);
  EXPECT_FALSE(out_of_map.unknown);
  EXPECT_TRUE(out_of_map.out_of_map);
}

TEST(UfomapMapperTest, QueryNodeAndLocalOccupiedSupportCoarserDepth) {
  auto mapper = makeMapper("ufomap_mapper_depth");
  integrateHits(*mapper, {makePoint(2.0, 0.0, 0.0)});

  const auto leaf = mapper->queryNode(ufo::Point(2.0F, 0.0F, 0.0F), 0U);
  const auto coarse = mapper->queryNode(ufo::Point(2.0F, 0.0F, 0.0F), 1U);

  EXPECT_EQ(leaf.depth, 0U);
  EXPECT_EQ(coarse.depth, 1U);
  EXPECT_GT(coarse.voxel_size, leaf.voxel_size);
  EXPECT_EQ(coarse.state, UfomapOccupancyState::Occupied);

  UfomapLocalOccupiedQuery query;
  query.depth = 1U;
  query.intersects.emplace_back(ufo::Point(2.0F, 0.0F, 0.0F), 1.0F);
  const auto occupied_nodes = mapper->queryLocalOccupied(query);

  ASSERT_FALSE(occupied_nodes.empty());
  EXPECT_EQ(occupied_nodes.front().depth, 1U);
  EXPECT_GT(occupied_nodes.front().voxel_size, leaf.voxel_size);
}

TEST(UfomapMapperTest, NearestOccupiedQueryReturnsClosestOccupiedNodeInRadius) {
  auto mapper = makeMapper("ufomap_mapper_nearest");
  integrateHits(*mapper, {makePoint(2.0, 0.0, 0.0), makePoint(4.0, 0.0, 0.0)});

  const auto nearest = mapper->queryNearestOccupied(
      ufo::Point(2.3F, 0.0F, 0.0F), 3.0, 0U, 1U);

  ASSERT_TRUE(nearest.has_value());
  EXPECT_LT(nearest->center.x, 3.0F);
  EXPECT_EQ(nearest->depth, 0U);

  const auto outside_radius = mapper->queryNearestOccupied(
      ufo::Point(-2.0F, 0.0F, 0.0F), 0.5, 0U, 1U);

  EXPECT_FALSE(outside_radius.has_value());
}

TEST(UfomapMapperTest, StationaryFirstObservationIsHeldOutThenReleasedStatic) {
  auto mapper = makeMapper("ufomap_mapper_corridor_first_observation", true);
  const auto points = makeCorridorFrame(-0.25);
  const auto first = mapper->processInputCloud(
      makeCloud(points, 20.0), makeOdom(makePoint(0.0, 0.0, 0.0), 20.0));

  EXPECT_GT(first.runtime_stats.corridor_candidate_point_count, 0U);
  // 未知簇必须先有横向运动证据，不能只凭连续出现两帧就作为动态发布。
  EXPECT_EQ(first.classification.dynamic_point_count, 0U);
  UfomapFrameResult result;
  for (std::size_t index = 1U; index < 5U; ++index) {
    const double stamp = 20.0 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(points, stamp), makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
    EXPECT_EQ(result.classification.dynamic_point_count, 0U);
  }
  const auto ball_node = mapper->queryNode(ufo::Point(0.57F, -0.23F, 0.62F));
  EXPECT_TRUE(ball_node.occupied);
}

TEST(UfomapMapperTest, CorridorNominalWallsProtectFirstClusterBeforeWallsAppear) {
  auto mapper = makeMapper("ufomap_mapper_corridor_nominal_walls", true);
  const auto points = makeBallCluster(0.45, -0.25);
  const auto first = mapper->processInputCloud(
      makeCloud(points, 25.0), makeOdom(makePoint(0.0, 0.0, 0.0), 25.0));

  EXPECT_TRUE(first.runtime_stats.corridor_wall_valid);
  EXPECT_GT(first.runtime_stats.corridor_candidate_point_count, 0U);
  EXPECT_EQ(first.classification.dynamic_point_count, 0U);
  UfomapFrameResult result;
  for (std::size_t index = 1U; index < 5U; ++index) {
    const double stamp = 25.0 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(points, stamp), makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
    EXPECT_EQ(result.classification.dynamic_point_count, 0U);
  }
  const auto ball_node = mapper->queryNode(ufo::Point(0.57F, -0.23F, 0.62F));
  EXPECT_TRUE(ball_node.occupied);
}

TEST(UfomapMapperTest, SparseStationaryClusterIsNeverPublishedDynamic) {
  auto mapper = makeMapper("ufomap_mapper_corridor_sparse_cluster", true);
  auto points = makeStaticCorridorWalls();
  points.push_back(makePoint(0.55, -0.20, 0.55));
  points.push_back(makePoint(0.60, -0.20, 0.60));
  points.push_back(makePoint(0.55, -0.15, 0.60));

  const auto first = mapper->processInputCloud(
      makeCloud(points, 26.0), makeOdom(makePoint(0.0, 0.0, 0.0), 26.0));

  EXPECT_EQ(first.classification.dynamic_point_count, 0U);
  UfomapFrameResult result;
  for (std::size_t index = 1U; index < 5U; ++index) {
    const double stamp = 26.0 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(points, stamp), makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
    EXPECT_EQ(result.classification.dynamic_point_count, 0U);
  }
  const auto node = mapper->queryNode(ufo::Point(0.57F, -0.18F, 0.58F));
  EXPECT_TRUE(node.occupied);
}

TEST(UfomapMapperTest, CorridorLongitudinalRoiLeavesDistantClusterToStaticMap) {
  auto mapper = makeMapper("ufomap_mapper_corridor_longitudinal_roi", true);
  const auto points = makeBallCluster(3.50, -0.25);
  const auto result = mapper->processInputCloud(
      makeCloud(points, 27.0), makeOdom(makePoint(0.0, 0.0, 0.0), 27.0));

  EXPECT_EQ(result.runtime_stats.corridor_candidate_point_count, 0U);
  EXPECT_EQ(result.classification.dynamic_point_count, 0U);
  const auto distant_node = mapper->queryNode(ufo::Point(3.57F, -0.23F, 0.62F));
  EXPECT_TRUE(distant_node.occupied);
}

TEST(UfomapMapperTest, CorridorLateralMotionBecomesConfirmed) {
  auto mapper = makeMapper("ufomap_mapper_corridor_motion", true);
  const std::vector<double> ball_positions{-0.25, -0.10, 0.05, 0.20};
  UfomapFrameResult last_result;
  std::uint32_t source_track_id = 0U;
  for (std::size_t index = 0U; index < ball_positions.size(); ++index) {
    const double stamp = 30.0 + 0.1 * static_cast<double>(index);
    last_result = mapper->processInputCloud(
        makeCloud(makeCorridorFrame(ball_positions[index]), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
    if (!last_result.classification.dynamic_cluster_points.empty()) {
      const std::uint32_t current_source =
          last_result.classification.dynamic_cluster_points.front().corridor_source_track_id;
      EXPECT_NE(current_source, 0U);
      if (source_track_id == 0U) {
        source_track_id = current_source;
      } else {
        EXPECT_EQ(current_source, source_track_id);
      }
    }
  }

  EXPECT_GT(last_result.runtime_stats.corridor_confirmed_point_count, 0U);
  EXPECT_GT(last_result.classification.dynamic_point_count, 0U);
  ASSERT_FALSE(last_result.classification.dynamic_cluster_points.empty());
  const auto& cluster_point = last_result.classification.dynamic_cluster_points.front();
  EXPECT_TRUE(cluster_point.measured_velocity_valid);
  EXPECT_GT(std::hypot(cluster_point.measured_velocity.x,
                       cluster_point.measured_velocity.y),
            0.10);
}

TEST(UfomapMapperTest, SingleVoxelLateralNoiseNeverBecomesConfirmed) {
  auto mapper = makeMapper("ufomap_mapper_corridor_single_voxel_noise", true);
  const std::vector<double> lateral_positions{-0.25, -0.10, 0.05, 0.20};
  UfomapFrameResult result;
  for (std::size_t index = 0U; index < lateral_positions.size(); ++index) {
    std::vector<geometry_msgs::Point> points;
    appendCompactCluster(points, 0.45, lateral_positions[index]);
    const double stamp = 30.5 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(points, stamp), makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  EXPECT_EQ(result.runtime_stats.corridor_confirmed_point_count, 0U);
  ASSERT_FALSE(result.classification.dynamic_cluster_points.empty());
  EXPECT_TRUE(result.classification.dynamic_cluster_points.front().provisional);
}

TEST(UfomapMapperTest, SparseLateralMotionSurvivesEndpointAndReversalWithoutMapInsertion) {
  auto mapper = makeMapper("ufomap_mapper_corridor_sparse_motion", true);
  UfomapFrameResult result;
  for (std::size_t index = 0U; index < 10U; ++index) {
    const double lateral = -0.36 + 0.08 * static_cast<double>(index);
    const double stamp = 30.8 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeSparseCorridorFrame(lateral), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  ASSERT_EQ(result.runtime_stats.corridor_confirmed_point_count, 0U);
  ASSERT_GT(result.classification.dynamic_point_count, 0U);
  ASSERT_FALSE(result.classification.dynamic_cluster_points.empty());
  EXPECT_TRUE(result.classification.dynamic_cluster_points.front().provisional);

  // 命中数已超过 unresolved_too_long 门槛；端点停留超过普通静态释放的
  // 4 帧仍应保留，因为该稀疏簇已经形成强多帧横向运动资格。
  for (std::size_t index = 0U; index < 6U; ++index) {
    const double stamp = 31.8 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeSparseCorridorFrame(0.36), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
    EXPECT_GT(result.classification.dynamic_point_count, 0U);
  }
  EXPECT_FALSE(mapper->queryNode(ufo::Point(0.43F, 0.34F, 0.58F)).occupied);

  const std::vector<double> reverse_positions{0.28, 0.20, 0.12, 0.04};
  for (std::size_t index = 0U; index < reverse_positions.size(); ++index) {
    const double stamp = 32.4 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeSparseCorridorFrame(reverse_positions[index]), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
    EXPECT_GT(result.classification.dynamic_point_count, 0U);
    EXPECT_EQ(result.runtime_stats.corridor_confirmed_point_count, 0U);
  }
  EXPECT_FALSE(mapper->queryNode(ufo::Point(0.43F, 0.02F, 0.58F)).occupied);
}

TEST(UfomapMapperTest, OldMotionQualificationCannotConfirmLaterStaticShape) {
  auto mapper = makeMapper("ufomap_mapper_corridor_stale_motion_shape", true);
  UfomapFrameResult result;
  for (std::size_t index = 0U; index < 10U; ++index) {
    const double lateral = -0.36 + 0.08 * static_cast<double>(index);
    const double stamp = 32.8 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeSparseCorridorFrame(lateral), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }
  ASSERT_EQ(result.runtime_stats.corridor_confirmed_point_count, 0U);

  // 让强横向运动证据离开 5 帧 history 窗口，但尚未达到 15 帧静态释放门槛。
  for (std::size_t index = 0U; index < 6U; ++index) {
    const double stamp = 33.8 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeSparseCorridorFrame(0.36), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  // 后续两帧实体尺寸来自静止大簇，不能借用窗口外的旧运动资格完成确认。
  for (std::size_t index = 0U; index < 2U; ++index) {
    const double stamp = 34.4 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeCorridorFrame(0.32), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }
  EXPECT_EQ(result.runtime_stats.corridor_confirmed_point_count, 0U);
  ASSERT_FALSE(result.classification.dynamic_cluster_points.empty());
  EXPECT_TRUE(result.classification.dynamic_cluster_points.front().provisional);
}

TEST(UfomapMapperTest, SparseStationaryClusterReleasesStaticAfterFourFrames) {
  auto mapper = makeMapper("ufomap_mapper_corridor_sparse_static_four_frames", true);
  UfomapFrameResult result;
  for (std::size_t index = 0U; index < 4U; ++index) {
    const double stamp = 33.0 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeSparseCorridorFrame(0.0), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  EXPECT_EQ(result.classification.dynamic_point_count, 0U);
  EXPECT_TRUE(mapper->queryNode(ufo::Point(0.43F, -0.02F, 0.58F)).occupied);
}

TEST(UfomapMapperTest, SingleLargeExtentOutlierCannotConfirmSparseTrack) {
  auto mapper = makeMapper("ufomap_mapper_corridor_single_extent_outlier", true);
  const std::vector<double> compact_positions{-0.25, -0.10};
  UfomapFrameResult result;
  for (std::size_t index = 0U; index < compact_positions.size(); ++index) {
    const double stamp = 33.5 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeSparseCorridorFrame(compact_positions[index]), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }
  auto outlier_frame = makeStaticCorridorWalls();
  const auto physical_outlier = makeBallCluster(0.45, 0.05);
  outlier_frame.insert(outlier_frame.end(), physical_outlier.begin(), physical_outlier.end());
  result = mapper->processInputCloud(
      makeCloud(outlier_frame, 33.7), makeOdom(makePoint(0.0, 0.0, 0.0), 33.7));

  const std::vector<double> remaining_positions{0.20, 0.32, 0.44};
  for (std::size_t index = 0U; index < remaining_positions.size(); ++index) {
    const double stamp = 33.8 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeSparseCorridorFrame(remaining_positions[index]), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  EXPECT_EQ(result.runtime_stats.corridor_confirmed_point_count, 0U);
  ASSERT_FALSE(result.classification.dynamic_cluster_points.empty());
  EXPECT_TRUE(result.classification.dynamic_cluster_points.front().provisional);
}

TEST(UfomapMapperTest, ContinuousSlenderLateralClusterCannotPassShapeEvidence) {
  auto mapper = makeMapper("ufomap_mapper_corridor_continuous_slender", true);
  const std::vector<double> lateral_positions{-0.25, -0.10, 0.05, 0.20, 0.35};
  UfomapFrameResult result;
  for (std::size_t index = 0U; index < lateral_positions.size(); ++index) {
    auto frame = makeStaticCorridorWalls();
    appendSlenderCluster(frame, 0.45, lateral_positions[index]);
    const double stamp = 34.1 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(frame, stamp), makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  EXPECT_EQ(result.runtime_stats.corridor_confirmed_point_count, 0U);
  ASSERT_FALSE(result.classification.dynamic_cluster_points.empty());
  EXPECT_TRUE(result.classification.dynamic_cluster_points.front().provisional);
}

TEST(UfomapMapperTest, TwoRecentPhysicalExtentFramesConfirmMovingTrack) {
  auto mapper = makeMapper("ufomap_mapper_corridor_two_extent_frames", true);
  UfomapFrameResult result;
  const std::vector<double> compact_positions{-0.25, -0.10};
  for (std::size_t index = 0U; index < compact_positions.size(); ++index) {
    const double stamp = 34.2 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeSparseCorridorFrame(compact_positions[index]), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  for (std::size_t index = 0U; index < 2U; ++index) {
    const double lateral = 0.05 + 0.15 * static_cast<double>(index);
    auto frame = makeStaticCorridorWalls();
    const auto ball = makeBallCluster(0.45, lateral);
    frame.insert(frame.end(), ball.begin(), ball.end());
    const double stamp = 34.4 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(frame, stamp), makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
    if (index == 0U) {
      EXPECT_EQ(result.runtime_stats.corridor_confirmed_point_count, 0U);
    }
  }

  EXPECT_GT(result.runtime_stats.corridor_confirmed_point_count, 0U);
  ASSERT_FALSE(result.classification.dynamic_cluster_points.empty());
  EXPECT_FALSE(result.classification.dynamic_cluster_points.front().provisional);
}

TEST(UfomapMapperTest, ConfirmedCorridorTrackKeepsSourceIdAcrossTurnReset) {
  auto mapper = makeMapper("ufomap_mapper_corridor_turn_reset", true);
  const std::vector<double> ball_positions{-0.25, -0.10, 0.05, 0.20};
  UfomapFrameResult result;
  for (std::size_t index = 0U; index < ball_positions.size(); ++index) {
    const double stamp = 31.0 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeCorridorFrame(ball_positions[index]), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }
  ASSERT_GT(result.runtime_stats.corridor_confirmed_point_count, 0U);
  ASSERT_FALSE(result.classification.dynamic_cluster_points.empty());
  const std::uint32_t source_track_id =
      result.classification.dynamic_cluster_points.front().corridor_source_track_id;
  ASSERT_NE(source_track_id, 0U);

  const double turn_stamp = 31.4;
  result = mapper->processInputCloud(
      makeCloud(makeBallCluster(0.45, 0.20), turn_stamp),
      makeOdom(makePoint(0.0, 0.0, 0.0), turn_stamp, 0.50));

  EXPECT_GT(result.runtime_stats.corridor_confirmed_point_count, 0U);
  ASSERT_FALSE(result.classification.dynamic_cluster_points.empty());
  EXPECT_EQ(result.classification.dynamic_cluster_points.front().corridor_source_track_id,
            source_track_id);
}

TEST(UfomapMapperTest, MotionQualifiedSparseTrackDoesNotReleaseOnTurnReset) {
  auto mapper = makeMapper("ufomap_mapper_corridor_sparse_turn_reset", true);
  UfomapFrameResult result;
  for (std::size_t index = 0U; index < 10U; ++index) {
    const double lateral = -0.36 + 0.08 * static_cast<double>(index);
    const double stamp = 35.0 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeSparseCorridorFrame(lateral), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }
  ASSERT_EQ(result.runtime_stats.corridor_confirmed_point_count, 0U);
  ASSERT_FALSE(result.classification.dynamic_cluster_points.empty());
  const std::uint32_t source_track_id =
      result.classification.dynamic_cluster_points.front().corridor_source_track_id;
  ASSERT_NE(source_track_id, 0U);

  std::vector<geometry_msgs::Point> turn_points;
  appendCompactCluster(turn_points, 0.45, 0.36);
  const auto turn_result = mapper->processInputCloud(
      makeCloud(turn_points, 36.0),
      makeOdom(makePoint(0.0, 0.0, 0.0), 36.0, 0.50));
  EXPECT_EQ(turn_result.runtime_stats.corridor_confirmed_point_count, 0U);
  EXPECT_GT(turn_result.runtime_stats.corridor_candidate_point_count, 0U);
  EXPECT_FALSE(mapper->queryNode(ufo::Point(0.45F, 0.36F, 0.60F)).occupied);

  std::vector<geometry_msgs::Point> next_points;
  appendCompactCluster(next_points, 0.45, 0.28);
  const auto reacquired = mapper->processInputCloud(
      makeCloud(next_points, 36.1),
      makeOdom(makePoint(0.0, 0.0, 0.0), 36.1, 0.50));
  EXPECT_EQ(reacquired.classification.dynamic_point_count, 0U);

  std::vector<geometry_msgs::Point> confirmed_direction_points;
  appendCompactCluster(confirmed_direction_points, 0.45, 0.20);
  const auto direction_confirmed = mapper->processInputCloud(
      makeCloud(confirmed_direction_points, 36.2),
      makeOdom(makePoint(0.0, 0.0, 0.0), 36.2, 0.50));
  ASSERT_GT(direction_confirmed.classification.dynamic_point_count, 0U);
  ASSERT_FALSE(direction_confirmed.classification.dynamic_cluster_points.empty());
  EXPECT_TRUE(direction_confirmed.classification.dynamic_cluster_points.front().provisional);
  EXPECT_EQ(direction_confirmed.classification.dynamic_cluster_points.front().corridor_source_track_id,
            source_track_id);
}

TEST(UfomapMapperTest, ConfirmedCorridorMotionBypassesUfomapWarmup) {
  auto mapper = makeMapper("ufomap_mapper_corridor_warmup", true, false, 30);
  const std::vector<double> ball_positions{-0.25, -0.10, 0.05, 0.20};
  UfomapFrameResult last_result;
  for (std::size_t index = 0U; index < ball_positions.size(); ++index) {
    const double stamp = 32.0 + 0.1 * static_cast<double>(index);
    last_result = mapper->processInputCloud(
        makeCloud(makeCorridorFrame(ball_positions[index]), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  EXPECT_FALSE(last_result.runtime_stats.warmup_ready);
  EXPECT_GT(last_result.runtime_stats.corridor_confirmed_point_count, 0U);
  EXPECT_GT(last_result.classification.dynamic_point_count, 0U);
}

TEST(UfomapMapperTest, GroundFilteredConfirmedBallStillFeedsDynamicClusterer) {
  auto mapper = makeMapper("ufomap_mapper_corridor_raw_ground", true, true, 0);
  const std::vector<double> ball_positions{-0.25, -0.10, 0.05, 0.20};
  UfomapFrameResult last_result;
  for (std::size_t index = 0U; index < ball_positions.size(); ++index) {
    const double stamp = 33.0 + 0.1 * static_cast<double>(index);
    last_result = mapper->processInputCloud(
        makeCloud(makeElevatedGroundFrame(ball_positions[index]), stamp),
        makeOdom(makePoint(0.0, 0.0, 1.20), stamp));
  }

  ASSERT_TRUE(last_result.runtime_stats.ground_plane_ready);
  EXPECT_GT(last_result.runtime_stats.corridor_confirmed_point_count, 0U);
  EXPECT_GE(last_result.classification.dynamic_cluster_points.size(), 4U);
  EXPECT_GT(last_result.classification.dynamic_point_count, 0U);
  EXPECT_TRUE(last_result.classification.dynamic_indices.empty());
  const auto ball_node = mapper->queryNode(ufo::Point(0.57F, 0.22F, 0.62F));
  EXPECT_FALSE(ball_node.occupied);
}

TEST(UfomapMapperTest, GroundFilteredBallReleasedStaticIsRestoredToMap) {
  auto mapper = makeMapper("ufomap_mapper_corridor_raw_static_release", true, true, 0);
  const std::vector<double> moving_positions{-0.25, -0.10, 0.05, 0.20};
  UfomapFrameResult result;
  for (std::size_t index = 0U; index < moving_positions.size(); ++index) {
    const double stamp = 34.0 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeElevatedGroundFrame(moving_positions[index]), stamp),
        makeOdom(makePoint(0.0, 0.0, 1.20), stamp));
    const bool ball_in_static = std::any_of(
        result.classification.static_points.begin(),
        result.classification.static_points.end(),
        [&](const ufo::Point& point) {
          return point.x >= 0.44F && point.x <= 0.70F &&
                 point.y >= static_cast<float>(moving_positions[index] - 0.01) &&
                 point.y <= static_cast<float>(moving_positions[index] + 0.09) &&
                 point.z >= 0.59F && point.z <= 0.69F;
        });
    EXPECT_FALSE(ball_in_static);
    EXPECT_FALSE(mapper->queryNode(ufo::Point(0.57F, moving_positions[index] + 0.02F,
                                              0.62F)).occupied);
  }

  ASSERT_TRUE(result.runtime_stats.ground_plane_ready);
  ASSERT_GT(result.runtime_stats.corridor_confirmed_point_count, 0U);
  ASSERT_GT(result.classification.dynamic_point_count, 0U);
  EXPECT_FALSE(mapper->queryNode(ufo::Point(0.57F, 0.22F, 0.62F)).occupied);

  for (std::size_t index = 0U; index < 20U; ++index) {
    const double stamp = 34.4 + 0.1 * static_cast<double>(index);
    result = mapper->processInputCloud(
        makeCloud(makeElevatedGroundFrame(0.20), stamp),
        makeOdom(makePoint(0.0, 0.0, 1.20), stamp));
  }

  EXPECT_EQ(result.runtime_stats.corridor_confirmed_point_count, 0U);
  EXPECT_EQ(result.classification.dynamic_point_count, 0U);
  EXPECT_GE(result.classification.static_point_count, 4U);
  EXPECT_TRUE(mapper->queryNode(ufo::Point(0.57F, 0.22F, 0.62F)).occupied);
  // 估计平面自身没有 released-static 标志，不能随球一起回填为占据。
  EXPECT_FALSE(mapper->queryNode(ufo::Point(0.0F, 2.0F, 0.95F)).occupied);
}

TEST(UfomapMapperTest, ConfirmedCorridorTrackIsPrioritizedAmongClutter) {
  auto mapper = makeMapper("ufomap_mapper_corridor_candidate_rejection", true);
  const std::vector<double> ball_positions{-0.25, -0.10, 0.05, 0.20};
  for (std::size_t index = 0U; index < ball_positions.size(); ++index) {
    const double stamp = 35.0 + 0.1 * static_cast<double>(index);
    mapper->processInputCloud(
        makeCloud(makeCorridorFrame(ball_positions[index]), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  auto cluttered_frame = makeCorridorFrame(0.35);
  appendCompactCluster(cluttered_frame, 1.10, -0.45);
  appendCompactCluster(cluttered_frame, 1.70, 0.00);
  appendCompactCluster(cluttered_frame, 2.30, -0.45);
  const auto result = mapper->processInputCloud(
      makeCloud(cluttered_frame, 35.4),
      makeOdom(makePoint(0.0, 0.0, 0.0), 35.4));

  EXPECT_GT(result.runtime_stats.corridor_confirmed_point_count, 0U);
  EXPECT_GT(result.classification.dynamic_point_count, 0U);
  const auto held_out_clutter = mapper->queryNode(ufo::Point(1.70F, 0.0F, 0.60F));
  EXPECT_FALSE(held_out_clutter.occupied);
}

TEST(UfomapMapperTest, StaticCorridorWallsDoNotCreateCandidates) {
  auto mapper = makeMapper("ufomap_mapper_corridor_static_walls", true);
  for (std::size_t index = 0U; index < 8U; ++index) {
    const double stamp = 40.0 + 0.1 * static_cast<double>(index);
    const auto result = mapper->processInputCloud(
        makeCloud(makeStaticCorridorWalls(), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
    EXPECT_EQ(result.runtime_stats.corridor_candidate_point_count, 0U);
    EXPECT_EQ(result.classification.dynamic_point_count, 0U);
  }
}

TEST(UfomapMapperTest, StaticInternalClusterNeverPublishesAndReleasesStatic) {
  auto mapper = makeMapper("ufomap_mapper_corridor_static_internal", true);
  std::vector<geometry_msgs::Point> points;
  appendCompactCluster(points, 0.60, 0.0);
  UfomapFrameResult last_result;
  for (std::size_t index = 0U; index < 6U; ++index) {
    const double stamp = 50.0 + 0.1 * static_cast<double>(index);
    last_result = mapper->processInputCloud(
        makeCloud(points, stamp), makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
    EXPECT_EQ(last_result.classification.dynamic_point_count, 0U);
  }

  EXPECT_EQ(last_result.classification.dynamic_point_count, 0U);
  const auto static_node = mapper->queryNode(ufo::Point(0.58F, -0.02F, 0.58F));
  EXPECT_TRUE(static_node.occupied);
}

TEST(UfomapMapperTest, ConfirmedCorridorTrackSurvivesEndpointThenDowngradesAfterLongStop) {
  auto mapper = makeMapper("ufomap_mapper_corridor_confirmed_stop", true);
  const std::vector<double> moving_positions{-0.25, -0.10, 0.05, 0.20};
  for (std::size_t index = 0U; index < moving_positions.size(); ++index) {
    const double stamp = 57.0 + 0.1 * static_cast<double>(index);
    mapper->processInputCloud(
        makeCloud(makeCorridorFrame(moving_positions[index]), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  UfomapFrameResult stopped_result;
  for (std::size_t index = 0U; index < 8U; ++index) {
    const double stamp = 57.4 + 0.1 * static_cast<double>(index);
    stopped_result = mapper->processInputCloud(
        makeCloud(makeCorridorFrame(0.20), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  // 摆球端点会短暂停速；该阶段必须继续保持确认轨迹和动态输出。
  EXPECT_GT(stopped_result.runtime_stats.corridor_confirmed_point_count, 0U);
  EXPECT_GT(stopped_result.classification.dynamic_point_count, 0U);

  for (std::size_t index = 8U; index < 20U; ++index) {
    const double stamp = 57.4 + 0.1 * static_cast<double>(index);
    stopped_result = mapper->processInputCloud(
        makeCloud(makeCorridorFrame(0.20), stamp),
        makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  EXPECT_EQ(stopped_result.runtime_stats.corridor_confirmed_point_count, 0U);
  EXPECT_EQ(stopped_result.classification.dynamic_point_count, 0U);
  const auto static_node = mapper->queryNode(ufo::Point(0.57F, 0.18F, 0.62F));
  EXPECT_TRUE(static_node.occupied);
}

TEST(UfomapMapperTest, ForwardMovingClusterIsNotConfirmedAsLateralDynamic) {
  auto mapper = makeMapper("ufomap_mapper_corridor_forward_motion", true);
  UfomapFrameResult last_result;
  for (std::size_t index = 0U; index < 12U; ++index) {
    std::vector<geometry_msgs::Point> points;
    appendCompactCluster(points, 0.45 + 0.12 * static_cast<double>(index), 0.0);
    const double stamp = 55.0 + 0.1 * static_cast<double>(index);
    last_result = mapper->processInputCloud(
        makeCloud(points, stamp), makeOdom(makePoint(0.0, 0.0, 0.0), stamp));
  }

  EXPECT_EQ(last_result.runtime_stats.corridor_confirmed_point_count, 0U);
  EXPECT_EQ(last_result.classification.dynamic_point_count, 0U);
  const auto static_node = mapper->queryNode(ufo::Point(1.77F, 0.0F, 0.60F));
  EXPECT_TRUE(static_node.occupied);
}

}  // namespace
}  // namespace ldopcore

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_ufomap_mapper");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

#include <gtest/gtest.h>

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

sensor_msgs::PointCloud2 makeCloud(const std::vector<geometry_msgs::Point>& points) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "map";
  cloud.header.stamp = ros::Time(10.0);

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

nav_msgs::Odometry makeOdom(const geometry_msgs::Point& position) {
  nav_msgs::Odometry odom;
  odom.header.frame_id = "map";
  odom.header.stamp = ros::Time(10.0);
  odom.pose.pose.position = position;
  odom.pose.pose.orientation.w = 1.0;
  return odom;
}

std::unique_ptr<UfomapMapper> makeMapper(const std::string& test_namespace) {
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

  return std::make_unique<UfomapMapper>(nh, pnh, false);
}

void integrateHits(UfomapMapper& mapper, const std::vector<geometry_msgs::Point>& hits) {
  mapper.processInputCloud(makeCloud(hits), makeOdom(makePoint(0.0, 0.0, 0.0)));
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

}  // namespace
}  // namespace ldopcore

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_ufomap_mapper");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

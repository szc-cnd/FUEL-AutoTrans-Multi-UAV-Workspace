#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include <Eigen/Geometry>
#include <ldop/FusedDynamicObjectArray.h>
#include <ldop/FusedDynamicObjectVirtualImuArray.h>
#include <ldop/dynamic_object_pose_fusion.h>
#include <ros/ros.h>

namespace ldopcore {
namespace {

geometry_msgs::PoseStamped makePoseStamped(const double stamp,
                                           const Eigen::Vector3d& position,
                                           const Eigen::Quaterniond& q) {
  geometry_msgs::PoseStamped msg;
  msg.header.frame_id = "mocap";
  msg.header.stamp = ros::Time(stamp);
  msg.pose.position.x = position.x();
  msg.pose.position.y = position.y();
  msg.pose.position.z = position.z();
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  return msg;
}

DynamicObjectDetection makeDetection(const double stamp,
                                     const Eigen::Vector3d& center,
                                     const Eigen::Vector3d& size,
                                     const std::size_t point_count = 12U) {
  DynamicObjectDetection detection;
  detection.stamp = ros::Time(stamp);
  detection.bbox.center.x = center.x();
  detection.bbox.center.y = center.y();
  detection.bbox.center.z = center.z();
  detection.bbox.size.x = size.x();
  detection.bbox.size.y = size.y();
  detection.bbox.size.z = size.z();
  detection.point_count = point_count;
  return detection;
}

double vector3Norm(const geometry_msgs::Vector3& vector) {
  return std::sqrt(vector.x * vector.x + vector.y * vector.y + vector.z * vector.z);
}

TEST(DynamicObjectPoseFusionMessagesTest, FusedObjectCarriesPerObjectStampAndSourceIndex) {
  ldop::FusedDynamicObjectArray array;
  array.header.frame_id = "map";
  array.header.stamp = ros::Time(10.02);
  array.objects.resize(1U);
  array.objects.front().header.frame_id = "map";
  array.objects.front().header.stamp = ros::Time(10.0);
  array.objects.front().mocap_source_index = 2U;
  array.objects.front().lidar_updated_in_current_frame = true;

  EXPECT_EQ(array.objects.front().header.stamp, ros::Time(10.0));
  EXPECT_EQ(array.objects.front().mocap_source_index, 2U);
  EXPECT_TRUE(array.objects.front().lidar_updated_in_current_frame);
}

TEST(DynamicObjectPoseFusionMessagesTest, VirtualImuCarriesEvalAndAvailabilityTime) {
  ldop::FusedDynamicObjectVirtualImuArray array;
  array.header.frame_id = "map";
  array.samples.resize(1U);
  array.samples.front().header.stamp = ros::Time(11.0);
  array.samples.front().available_stamp = ros::Time(11.04);
  array.samples.front().mocap_source_index = 1U;
  array.samples.front().virtual_specific_force.z = 9.80665;

  EXPECT_EQ(array.samples.front().header.stamp, ros::Time(11.0));
  EXPECT_EQ(array.samples.front().available_stamp, ros::Time(11.04));
  EXPECT_EQ(array.samples.front().mocap_source_index, 1U);
  EXPECT_NEAR(array.samples.front().virtual_specific_force.z, 9.80665, 1e-9);
}

TEST(DynamicObjectPoseFusionConfigTest, BuildsRuntimeConfigFromBoundedParams) {
  DynamicObjectPoseFusionParams params;
  params.mocap_topics = {"/mocap/a", "/mocap/b"};
  params.mocap_to_ldop_translation = {1.0, 2.0, 3.0};
  params.offset_alpha = 1.5;
  params.bspline_min_samples = 3;
  params.bspline_future_samples = 10;

  const DynamicObjectPoseFusionConfig config = buildPoseFusionConfig(params);

  EXPECT_EQ(config.mocap_topics.size(), 2U);
  EXPECT_EQ(config.mocap_topics.front(), "/mocap/a");
  EXPECT_DOUBLE_EQ(config.mocap_to_ldop_translation.x(), 1.0);
  EXPECT_DOUBLE_EQ(config.mocap_to_ldop_translation.y(), 2.0);
  EXPECT_DOUBLE_EQ(config.mocap_to_ldop_translation.z(), 3.0);
  EXPECT_DOUBLE_EQ(config.offset_alpha, 1.0);
  EXPECT_EQ(config.bspline_min_samples, 5U);
  EXPECT_EQ(config.bspline_future_samples, 4U);
}

TEST(DynamicObjectPoseFusionTest, InitializesOffsetFromLidarCentroidInRigidFrame) {
  constexpr double kPi = 3.14159265358979323846;
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  config.mocap_to_ldop_translation = Eigen::Vector3d(1.0, 0.0, 0.0);
  DynamicObjectPoseFusion fusion(config);

  const Eigen::Quaterniond q(Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()));
  EXPECT_FALSE(fusion.processMocapPose(0U, makePoseStamped(10.0, {2.0, 0.0, 0.0}, q))
                   .fused_objects.has_value());

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(10.0);
  const auto snapshot = fusion.updateLidarAnchors(
      header, {makeDetection(10.0, Eigen::Vector3d(3.0, 1.0, 0.0),
                             Eigen::Vector3d(1.0, 1.0, 1.0))});

  ASSERT_TRUE(snapshot.fused_objects.has_value());
  ASSERT_EQ(snapshot.fused_objects->objects.size(), 1U);
  const auto& object = snapshot.fused_objects->objects.front();
  EXPECT_EQ(object.header.stamp, ros::Time(10.0));
  EXPECT_EQ(object.mocap_source_index, 0U);
  EXPECT_NEAR(object.offset_body.x, 1.0, 1e-9);
  EXPECT_NEAR(object.offset_body.y, 0.0, 1e-9);
  EXPECT_NEAR(object.offset_body.z, 0.0, 1e-9);
  EXPECT_TRUE(object.lidar_updated_in_current_frame);
}

TEST(DynamicObjectPoseFusionTest, AppliesCurrentRigidRotationToStoredOffset) {
  constexpr double kPi = 3.14159265358979323846;
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  DynamicObjectPoseFusion fusion(config);

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(1.0);
  fusion.processMocapPose(0U, makePoseStamped(1.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  fusion.updateLidarAnchors(
      header, {makeDetection(1.0, Eigen::Vector3d(1.0, 0.0, 0.0),
                             Eigen::Vector3d(1.0, 1.0, 1.0))});

  const Eigen::Quaterniond yaw90(Eigen::AngleAxisd(kPi / 2.0, Eigen::Vector3d::UnitZ()));
  const auto snapshot =
      fusion.processMocapPose(0U, makePoseStamped(1.02, {0.0, 0.0, 0.0}, yaw90));

  ASSERT_TRUE(snapshot.fused_objects.has_value());
  const auto& pose = snapshot.fused_objects->objects.front().fused_pose;
  EXPECT_NEAR(pose.position.x, 0.0, 1e-9);
  EXPECT_NEAR(pose.position.y, 1.0, 1e-9);
  EXPECT_NEAR(pose.position.z, 0.0, 1e-9);
}

TEST(DynamicObjectPoseFusionTest, AssociatesDetectionsWithGlobalMinimumDistance) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a", "/mocap/b"};
  config.association_max_distance = 2.0;
  DynamicObjectPoseFusion fusion(config);

  fusion.processMocapPose(0U, makePoseStamped(2.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  fusion.processMocapPose(1U, makePoseStamped(2.0, {10.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(2.0);
  const auto snapshot = fusion.updateLidarAnchors(
      header,
      {makeDetection(2.0, {10.0, 0.0, -0.5}, {1.0, 1.0, 1.0}),
       makeDetection(2.0, {0.0, 0.0, -0.5}, {1.0, 1.0, 1.0})});

  ASSERT_TRUE(snapshot.fused_objects.has_value());
  ASSERT_EQ(snapshot.fused_objects->objects.size(), 2U);
  EXPECT_EQ(snapshot.fused_objects->objects[0].mocap_source_index, 0U);
  EXPECT_NEAR(snapshot.fused_objects->objects[0].offset_body.x, 0.0, 1e-9);
  EXPECT_NEAR(snapshot.fused_objects->objects[0].offset_body.z, -0.5, 1e-9);
  EXPECT_EQ(snapshot.fused_objects->objects[1].mocap_source_index, 1U);
  EXPECT_NEAR(snapshot.fused_objects->objects[1].offset_body.x, 0.0, 1e-9);
  EXPECT_NEAR(snapshot.fused_objects->objects[1].offset_body.z, -0.5, 1e-9);
}

TEST(DynamicObjectPoseFusionTest, RejectsOffsetObservationJumpAndKeepsPreviousOffset) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  config.offset_update_max_delta = 0.2;
  config.offset_observation_max_norm = 5.0;
  config.association_max_distance = 10.0;
  DynamicObjectPoseFusion fusion(config);

  fusion.processMocapPose(0U, makePoseStamped(3.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(3.0);
  fusion.updateLidarAnchors(
      header, {makeDetection(3.0, {0.0, 0.0, -0.5}, {1.0, 1.0, 1.0})});

  fusion.processMocapPose(0U, makePoseStamped(3.1, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  header.stamp = ros::Time(3.1);
  const auto snapshot = fusion.updateLidarAnchors(
      header, {makeDetection(3.1, {3.0, 0.0, 0.0}, {1.0, 1.0, 1.0})});

  EXPECT_FALSE(snapshot.fused_objects.has_value());
  const auto propagated =
      fusion.processMocapPose(0U, makePoseStamped(3.12, {0.0, 0.0, 0.0},
                                                  Eigen::Quaterniond::Identity()));
  ASSERT_TRUE(propagated.fused_objects.has_value());
  EXPECT_NEAR(propagated.fused_objects->objects.front().offset_body.x, 0.0, 1e-9);
  EXPECT_NEAR(propagated.fused_objects->objects.front().offset_body.z, -0.5, 1e-9);
}

TEST(DynamicObjectPoseFusionTest, StopsPublishingAfterLidarAnchorTimeout) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  config.lidar_anchor_timeout = 0.1;
  DynamicObjectPoseFusion fusion(config);

  fusion.processMocapPose(0U, makePoseStamped(4.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(4.0);
  fusion.updateLidarAnchors(
      header, {makeDetection(4.0, {0.0, 0.0, -0.5}, {1.0, 1.0, 1.0})});

  EXPECT_TRUE(fusion.processMocapPose(0U, makePoseStamped(4.05, {0.0, 0.0, 0.0},
                                                          Eigen::Quaterniond::Identity()))
                  .fused_objects.has_value());
  EXPECT_FALSE(fusion.processMocapPose(0U, makePoseStamped(4.2, {0.0, 0.0, 0.0},
                                                          Eigen::Quaterniond::Identity()))
                   .fused_objects.has_value());
}

TEST(DynamicObjectPoseFusionTest, RejectsRepeatedMocapStamp) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  DynamicObjectPoseFusion fusion(config);

  fusion.processMocapPose(0U, makePoseStamped(5.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(5.0);
  fusion.updateLidarAnchors(
      header, {makeDetection(5.0, {0.0, 0.0, -0.5}, {1.0, 1.0, 1.0})});

  const auto repeated =
      fusion.processMocapPose(0U, makePoseStamped(5.0, {1.0, 0.0, 0.0},
                                                  Eigen::Quaterniond::Identity()));

  EXPECT_FALSE(repeated.fused_objects.has_value());
}

TEST(DynamicObjectPoseFusionTest, GeneratesVirtualImuForConstantVelocityTrajectory) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  config.mocap_nominal_rate = 50.0;
  config.virtual_imu_rate_hz = 200.0;
  config.bspline_min_samples = 8U;
  config.bspline_future_samples = 2U;
  config.gravity_z = -9.80665;
  DynamicObjectPoseFusion fusion(config);

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(6.0);
  fusion.processMocapPose(0U, makePoseStamped(6.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  fusion.updateLidarAnchors(
      header, {makeDetection(6.0, {0.0, 0.0, 0.0}, {1.0, 1.0, 1.0})});

  DynamicObjectPoseFusionSnapshot last_snapshot;
  for (int index = 1; index <= 12; ++index) {
    const double t = 6.0 + 0.02 * static_cast<double>(index);
    last_snapshot = fusion.processMocapPose(
        0U, makePoseStamped(t, {t - 6.0, 0.0, 0.0}, Eigen::Quaterniond::Identity()));
  }

  ASSERT_TRUE(last_snapshot.virtual_imu.has_value());
  ASSERT_FALSE(last_snapshot.virtual_imu->samples.empty());
  for (const auto& sample : last_snapshot.virtual_imu->samples) {
    EXPECT_EQ(sample.mocap_source_index, 0U);
    EXPECT_NEAR(sample.virtual_linear_velocity.x, 1.0, 1e-6);
    EXPECT_NEAR(sample.virtual_linear_velocity.y, 0.0, 1e-6);
    EXPECT_NEAR(vector3Norm(sample.virtual_linear_acceleration), 0.0, 1e-6);
    EXPECT_NEAR(sample.virtual_specific_force.z, 9.80665, 1e-6);
    EXPECT_EQ(sample.available_stamp, ros::Time(6.24));
  }
}

TEST(DynamicObjectPoseFusionTest, DoesNotDuplicateVirtualImuEvalStampsAcrossCallbacks) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  config.mocap_nominal_rate = 50.0;
  config.virtual_imu_rate_hz = 200.0;
  config.bspline_min_samples = 8U;
  config.bspline_future_samples = 2U;
  DynamicObjectPoseFusion fusion(config);

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(7.0);
  fusion.processMocapPose(0U, makePoseStamped(7.0, {0.0, 0.0, 0.0},
                                              Eigen::Quaterniond::Identity()));
  fusion.updateLidarAnchors(
      header, {makeDetection(7.0, {0.0, 0.0, 0.0}, {1.0, 1.0, 1.0})});

  std::vector<ros::Time> stamps;
  for (int index = 1; index <= 14; ++index) {
    const double t = 7.0 + 0.02 * static_cast<double>(index);
    const auto snapshot = fusion.processMocapPose(
        0U, makePoseStamped(t, {0.0, t - 7.0, 0.0}, Eigen::Quaterniond::Identity()));
    if (snapshot.virtual_imu.has_value()) {
      for (const auto& sample : snapshot.virtual_imu->samples) {
        stamps.push_back(sample.header.stamp);
      }
    }
  }

  ASSERT_GT(stamps.size(), 4U);
  for (std::size_t index = 1U; index < stamps.size(); ++index) {
    EXPECT_LT(stamps[index - 1U], stamps[index]);
  }
}

TEST(DynamicObjectPoseFusionTest, BuildsBoxMarkerAtFusedPoseAndOffsetLineOnlyWhenVerbose) {
  DynamicObjectPoseFusionConfig config;
  config.mocap_topics = {"/mocap/a"};
  DynamicObjectPoseFusion quiet_fusion(config, false);
  DynamicObjectPoseFusion verbose_fusion(config, true);

  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(8.0);

  quiet_fusion.processMocapPose(0U, makePoseStamped(8.0, {0.0, 0.0, 0.0},
                                                    Eigen::Quaterniond::Identity()));
  const auto quiet_snapshot = quiet_fusion.updateLidarAnchors(
      header, {makeDetection(8.0, {0.0, 0.0, -0.5}, {1.0, 2.0, 3.0})});
  ASSERT_TRUE(quiet_snapshot.markers.has_value());
  EXPECT_EQ(quiet_snapshot.markers->markers.size(), 2U);

  verbose_fusion.processMocapPose(0U, makePoseStamped(8.0, {0.0, 0.0, 0.0},
                                                      Eigen::Quaterniond::Identity()));
  const auto verbose_snapshot = verbose_fusion.updateLidarAnchors(
      header, {makeDetection(8.0, {0.0, 0.0, -0.5}, {1.0, 2.0, 3.0})});
  ASSERT_TRUE(verbose_snapshot.markers.has_value());
  EXPECT_EQ(verbose_snapshot.markers->markers.size(), 3U);
}

}  // namespace
}  // namespace ldopcore

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_dynamic_object_pose_fusion");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
